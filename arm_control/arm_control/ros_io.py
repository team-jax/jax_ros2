"""ROS 2 side. AK45s go through ak45_ros2's ak45_node, servos through stm_bridge; arm_control only
publishes topics and never touches CAN.

ak45_node (ak45_ros2 README, docs/ros2.md section 13):
  /ak45/command  sensor_msgs/JointState, names ak45_1..ak45_4 (CAN id 1..4), position in rad,
                 RELIABLE KeepLast(10). The node resends the latest target at 10 Hz, rejects a joint
                 whose |target| exceeds max_command_deg, and sends nothing before the first command.
  /joint_states  feedback of the motors that answer, rad.
stm_bridge:
  /servo/command sensor_msgs/JointState, names = calibration.json `name` (W_PITCH, F1_1, ...),
                 position = servo MOTOR angle in rad (0..pi). stm_bridge sends one CAN frame per joint.
The rad in both is the motor angle (motor_deg from calibration.json), not the sim joint angle.
"""
import time
from math import degrees, radians

from sensor_msgs.msg import JointState

AK45_COMMAND = "/ak45/command"
AK45_STATES = "/joint_states"
SERVO_COMMAND = "/servo/command"


def ak45_topic_name(joint):
    return f"ak45_{joint.id}"


class Ak45Topics:
    def __init__(self, node, command=True):
        """command=False: feedback only (pick_place and play_trajectory command through arm_control_node)."""
        self.node = node
        self.pub = node.create_publisher(JointState, AK45_COMMAND, 10) if command else None
        self.state = {}
        node.create_subscription(JointState, AK45_STATES, self._on_state, 10)

    def _on_state(self, msg):
        now = time.monotonic()
        for n, p in zip(msg.name, msg.position):
            self.state[n] = (degrees(p), now)

    def feedback_deg(self, joint, max_age=0.5):
        got = self.state.get(ak45_topic_name(joint))
        if got is None or time.monotonic() - got[1] > max_age:
            return None
        return got[0]

    def publish(self, targets):
        """targets: [(Joint, motor_deg), ...]."""
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [ak45_topic_name(j) for j, _ in targets]
        msg.position = [radians(d) for _, d in targets]
        self.pub.publish(msg)


class ServoTopic:
    """/servo/command publisher: the servo counterpart of Ak45Topics.publish."""

    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(JointState, SERVO_COMMAND, 10)

    def publish(self, targets):
        """targets: [(Joint, motor_deg), ...]."""
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [j.name for j, _ in targets]
        msg.position = [radians(d) for _, d in targets]
        self.pub.publish(msg)
