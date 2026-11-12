"""approach_node: the camera's cube -> the model's approach -> /arm_control/joint_command, at 60 Hz.

    /object_pose (vision_node, frame shoulder_roll_axis) --PoseGate--> plan_from_pose (deploy README 7.1-7.3,
    tables in arm_control/model) --check_plan (calibration.json)--> /arm_control/joint_command, 60 Hz
    -> arm_control_node (calibration, speed limit) -> ak45_node / stm_bridge.

Nothing moves until a start:
    ros2 service call /approach/start std_srvs/srv/Trigger      # plan from the newest stable pose, check, go
    ros2 service call /approach/stop  std_srvs/srv/Trigger      # stop; the AK45s are told to stay where they are

start is refused when the pose is not in shoulder_roll_axis (vision_node needs camera_to_robot), not
stable or too old, off the reachable table, at a height that says the frame is wrong, or when
calibration.json would cut a joint short or is not verified (force:=true passes the last two; a pose
problem is never forced). The plan is the whole approach, computed and checked before the first frame.

The first frame is held hold_s seconds (the arm gets there at arm_control_node's speed limit), then one
frame per 1/rate_hz. The AK45s are watched as in play_trajectory (watch.TrackingWatch): one more than
max_error_deg from its command for 0.5 s stops the run. At the end the arm holds the pregrasp pose
(phase HOLD on /approach/phase). The learned grasp starts from there once /pressure exists.

finger_rotation is left out unless send_finger_rotation:=true: calibration.json allows it [0, 0] deg
(the old IK pose) and the model turns it 0 -> -170 deg; set its zero, sign and range on the robot first.
"""
import time
from math import atan2, degrees, radians
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import calibration
from .approach_plan import PoseGate, check_plan, plan_from_pose
from .ros_io import Ak45Topics, ak45_topic_name
from .watch import TrackingWatch

COMMAND_TOPIC = "/arm_control/joint_command"
OBJECT_POSE_TOPIC = "/object_pose"
PHASE_TOPIC = "/approach/phase"
REPO_TABLES = Path(__file__).resolve().parent.parent / "model"


def default_tables_dir():
    if REPO_TABLES.is_dir():
        return REPO_TABLES
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("arm_control")) / "model"


class ApproachNode(Node):
    def __init__(self):
        super().__init__("approach_node")
        p = self.declare_parameter
        self.tables_dir = p("tables_dir", "").value or str(default_tables_dir())
        self.rate_hz = p("rate_hz", 60.0).value
        self.hold_s = p("hold_s", 5.0).value
        self.handover_s = p("handover_s", 8.65).value
        self.table_e = p("table_e", 0.0).value
        self.max_z_error = p("max_z_error_m", 0.05).value
        self.send_frot = p("send_finger_rotation", False).value
        self.force = p("force", False).value
        self.max_error = p("max_error_deg", 15.0).value
        self.cal = calibration.load(p("calibration_file", "").value or None)
        self.gate = PoseGate(p("stable_poses", 2).value, p("stable_tol_m", 0.01).value, p("max_pose_age_s", 3.0).value)

        self.pub = self.create_publisher(JointState, COMMAND_TOPIC, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.phase_pub = self.create_publisher(String, PHASE_TOPIC, latched)
        self.ak45 = Ak45Topics(self, command=False)
        self.create_subscription(PoseStamped, OBJECT_POSE_TOPIC, self.on_pose, 10)
        self.create_service(Trigger, "/approach/start", self.on_start)
        self.create_service(Trigger, "/approach/stop", self.on_stop)
        self.timer = None
        self.plan = None
        self.set_phase("IDLE")
        self.get_logger().info(
            f"tables {self.tables_dir}, {self.rate_hz:g} Hz, handover {self.handover_s:g} s, "
            f"finger_rotation {'sent' if self.send_frot else 'NOT sent'}, force {self.force}. "
            "Waiting for /object_pose; start with: ros2 service call /approach/start std_srvs/srv/Trigger")

    # --- inputs ---------------------------------------------------------------------------------
    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = degrees(2.0 * atan2(q.z, q.w))
        self.gate.add(msg.header.frame_id, (p.x, p.y, p.z), yaw, time.monotonic())
        if self.gate.reason:
            self.get_logger().warning(self.gate.reason, throttle_duration_sec=5.0)
        elif self.timer is None:
            self.get_logger().info(f"/object_pose x {p.x:.3f} y {p.y:.3f} z {p.z:.3f} m, yaw {yaw:.1f} deg",
                                   throttle_duration_sec=2.0)

    def set_phase(self, phase):
        self.phase = phase
        self.phase_pub.publish(String(data=phase))

    # --- services -------------------------------------------------------------------------------
    def on_start(self, _req, res):
        res.success, res.message = self._start()
        (self.get_logger().info if res.success else self.get_logger().error)(f"start: {res.message}")
        return res

    def _start(self):
        if self.timer is not None:
            return False, f"already running ({self.phase})"
        pose, why = self.gate.ready(time.monotonic())
        if pose is None:
            return False, why
        try:
            plan = plan_from_pose(pose[0], pose[1], self.tables_dir, self.send_frot, self.table_e,
                                  self.max_z_error, self.handover_s)
        except ValueError as e:
            return False, str(e)
        problems = check_plan(plan, self.cal)
        for pr in problems:
            self.get_logger().warning(f"check: {pr}")
        if problems and not self.force:
            return False, f"{len(problems)} check(s) failed (see above); fix them or set force:=true"
        self.plan, self.i = plan, 0
        self.hold_ticks = max(1, int(self.hold_s * self.rate_hz))
        self.watch = TrackingWatch(self.max_error)
        self.names = list(plan.frames[0][1])
        self.motors = [self.cal[n] for n in self.names if self.cal[n].motor == "ak45"]
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_tick)
        self.set_phase("HOLD_FIRST")
        return True, (f"cube ROS ({plan.ros[0]:.3f}, {plan.ros[1]:.3f}, {plan.ros[2]:.3f}) -> world "
                      f"({plan.world[0]:.3f}, {plan.world[1]:.3f}, {plan.world[2]:.3f}) m, offset "
                      f"({plan.offset[0] * 100:.1f}, {plan.offset[1] * 100:.1f}) cm, hand theta {plan.theta_deg:.1f} deg; "
                      f"{len(plan.frames)} frames ({len(plan.frames) / self.rate_hz:.1f} s) after {self.hold_s:g} s on frame 0")

    def on_stop(self, _req, res):
        if self.timer is None:
            res.success, res.message = True, f"not running ({self.phase})"
            return res
        self._halt("stop requested")
        res.success, res.message = True, "stopped; the AK45s were told to stay where they are"
        return res

    # --- the run --------------------------------------------------------------------------------
    def publish(self, frame):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(frame)
        msg.position = [float(v) for v in frame.values()]
        self.pub.publish(msg)

    def on_tick(self):
        if self.i < self.hold_ticks:            # frame 0 while the arm gets there; not watched
            self.publish(self.plan.frames[0][1])
            self.i += 1
            return
        k = self.i - self.hold_ticks
        if k >= len(self.plan.frames):
            self.timer.cancel()
            self.timer = None
            self.set_phase("HOLD")
            self.get_logger().info("approach done: holding the pregrasp pose. The learned grasp needs /pressure "
                                   "(not on the STM yet), so it does not start.")
            return
        phase, frame = self.plan.frames[k]
        if phase != self.phase:
            self.set_phase(phase)
        self.publish(frame)
        self.i += 1
        expected = {ak45_topic_name(j): j.to_motor(degrees(frame[j.joint]))[0] for j in self.motors}
        feedback = {ak45_topic_name(j): self.ak45.feedback_deg(j) for j in self.motors}
        reason = self.watch.check(expected, feedback, time.monotonic())
        if reason:
            self._halt(reason)

    def _halt(self, reason):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        here = [(j.joint, self.ak45.feedback_deg(j)) for j in self.motors]
        here = {n: radians(self.cal[n].to_sim(fb)) for n, fb in here if fb is not None}
        if here:
            self.publish(here)
        self.set_phase("STOPPED")
        self.get_logger().error(f"stopped: {reason}; AK45s told to stay at their feedback angle "
                                f"({len(here)}/{len(self.motors)} had feedback), servos keep their last command")


def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
