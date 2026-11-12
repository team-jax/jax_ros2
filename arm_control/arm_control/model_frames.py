"""ROS <-> deploy model: the object frame and the joint names.

Frame (IntergrateSim ROS2_INTEGRATION_PLAN.md section 3; NOT measured on the robot yet):
  ROS shoulder_roll_axis: origin on the first shoulder motor axis, x robot left, y back, z up.
  deploy world: the robot_v3 model's world; that axis is at (0, 0.212, 1.2075) m.
  p_world = Rz(-90 deg) p_ros + (0, 0.212, 1.2075)
  A cube's yaw is only defined mod 90 deg, so the 90 deg turn leaves it as it is; it is wrapped
  into (-45, 45] deg, the range the grasp policy was trained on.

Joints: deploy (robot_v3 URDF) name -> the sim joint names of /arm_control/joint_command and
calibration.json. Angles pass through, except finger_rotation: calibration.json's 0 there is the old
IK pose URDF -180 deg (its note), so ros = deploy + 180 deg.
"""
from math import pi

import numpy as np

SHOULDER_ROLL_AXIS_WORLD = np.array([0.0, 0.212, 1.2075])
_RZ = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])   # Rz(-90 deg)

ROS_NAME = {
    "shoulder_pitch_to_shoulder_roll": "shoulder_roll",
    "shoulder_roll_to_shoulder_yaw": "shoulder_yaw",
    "shoulder_yaw_to_upper_arm": "upper_arm_roll",
    "upper_arm_to_under_arm": "elbow",
    "under_arm_to_wrist_pitch": "wrist_pitch",
    "wrist_pitch_to_wrist_yaw": "wrist_yaw",
    "wrist_yaw_to_hand_base": "hand_roll",
    "hand_base_to_rotation_base": "finger_rotation",
    "hand_base_to_finger_1_1": "finger1_joint1",
    "finger_1_1_to_finger_1_2": "finger1_joint2",
    "hand_base_to_finger_2_1": "finger2_joint1",
    "finger_1_2_to_finger_2_2": "finger2_joint2",
    "rotation_base_to_finger_3_1": "finger3_joint1",
    "finger_3_1_to_finger_3_2": "finger3_joint2",
}
OFFSET_RAD = {"finger_rotation": pi}


def ros_to_world(p):
    return _RZ @ np.asarray(p, dtype=float) + SHOULDER_ROLL_AXIS_WORLD


def world_to_ros(p):
    return _RZ.T @ (np.asarray(p, dtype=float) - SHOULDER_ROLL_AXIS_WORLD)


def cube_yaw_deg(yaw_deg):
    """A cube's yaw wrapped into (-45, 45] deg."""
    y = float(yaw_deg) % 90.0
    return y - 90.0 if y > 45.0 else y


def to_ros_command(deploy):
    """{deploy joint: rad} -> {ROS joint: rad} for /arm_control/joint_command."""
    out = {}
    for name, q in deploy.items():
        ros = ROS_NAME[name]
        out[ros] = float(q) + OFFSET_RAD.get(ros, 0.0)
    return out
