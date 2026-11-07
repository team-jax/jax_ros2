"""arm_control_node: sim joint angles in, motor commands out (as topics).

    /arm_control/joint_command  sensor_msgs/JointState, sim joint names (elbow, wrist_pitch, ...), rad
      -> calibration.json -> per-joint speed limit (max_speed_deg_s of the motor command, rate_hz steps)
      -> AK45 joints:  /ak45/command  (ak45_node sends and resends them)
      -> servo joints: /servo/command (stm_bridge puts them on the CAN bus)
    /object_pose  geometry_msgs/PoseStamped from vision_node: only logged for now (there is no camera
                  frame conversion yet, so the arm is never moved by it)

Rules, like ak45_node: nothing is sent before the first command; a message with a length mismatch, an
unknown name or a non-finite value is dropped whole; joints not in a message keep their target.
An AK45 starts from its /joint_states feedback and does not move without it. A servo's first command
jumps (CAN_STM sends no pulse before its first command, so its position is unknown).
"""
from math import atan2, degrees, isfinite

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from . import calibration
from .motion import Mover
from .ros_io import Ak45Topics, ServoTopic

COMMAND_TOPIC = "/arm_control/joint_command"
OBJECT_POSE_TOPIC = "/object_pose"


class ArmControlNode(Node):
    def __init__(self):
        super().__init__("arm_control_node")
        cal_file = self.declare_parameter("calibration_file", "").value
        hz = self.declare_parameter("rate_hz", 50.0).value
        rate = self.declare_parameter("max_speed_deg_s", 30.0).value
        self.cal = calibration.load(cal_file or None)
        self.joints = {j.joint: j for j in self.cal.joints}
        self.servo = ServoTopic(self)
        self.ak45 = Ak45Topics(self)
        self.mover = Mover(rate, hz)
        self.create_subscription(JointState, COMMAND_TOPIC, self.on_command, 10)
        self.create_subscription(PoseStamped, OBJECT_POSE_TOPIC, self.on_object_pose, 10)
        self.create_timer(1.0 / hz, self.on_timer)
        unverified = [j.joint for j in self.cal.joints if not j.verified]
        self.get_logger().info(
            f"{self.cal.path}: {len(self.joints)} joints, {rate:g} deg/s at {hz:g} Hz. "
            f"Unverified calibration: {', '.join(unverified) or 'none'}")

    def on_object_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = degrees(2.0 * atan2(q.z, q.w))     # the pose carries a z-axis yaw only
        self.get_logger().info(
            f"/object_pose ({msg.header.frame_id}): x {p.x:.3f} y {p.y:.3f} z {p.z:.3f} m, yaw {yaw:.1f} deg "
            "(logged only, the arm does not move)", throttle_duration_sec=1.0)

    def on_command(self, msg):
        log = self.get_logger()
        if len(msg.name) != len(msg.position):
            log.warning(f"name and position lengths differ ({len(msg.name)}, {len(msg.position)}); "
                        "message ignored", throttle_duration_sec=5.0)
            return
        for n, p in zip(msg.name, msg.position):
            if n not in self.joints:
                log.warning(f"unknown joint {n}; message ignored", throttle_duration_sec=5.0)
                return
            if not isfinite(p):
                log.warning(f"non-finite target for {n}; message ignored", throttle_duration_sec=5.0)
                return
        for n, p in zip(msg.name, msg.position):
            m, clipped = self.joints[n].to_motor(degrees(p))
            if clipped:
                log.warning(f"{n}: {degrees(p):.1f} deg clipped to motor {m:.1f}",
                            throttle_duration_sec=5.0)
            self.mover.set_target(n, m)

    def on_timer(self):
        servo = []
        for n in self.mover.needs_start():
            j = self.joints[n]
            if j.motor == "servo":
                t = self.mover.target[n]
                self.mover.start(n, t)
                servo.append((j, t))
                continue
            fb = self.ak45.feedback_deg(j)
            if fb is None:
                self.get_logger().warning(f"no /joint_states for ak45_{j.id}; {n} waits",
                                          throttle_duration_sec=5.0)
                continue
            self.mover.start(n, fb)
        ak45 = []
        for n, deg in self.mover.step().items():
            j = self.joints[n]
            if j.motor == "ak45":
                ak45.append((j, deg))
            else:
                servo.append((j, deg))
        if ak45:
            self.ak45.publish(ak45)
        if servo:
            self.servo.publish(servo)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
