"""Camera -> model approach -> motors: arm.launch.py + vision_node (result_json) + approach_node.

    ros2 launch arm_control approach.launch.py \
        result_json:=<StereoDepthSeg --out>/inference.json \
        camera_to_robot:="[tx, ty, tz, roll_deg, pitch_deg, yaw_deg]" [force:=true] [send_finger_rotation:=false]
    ros2 service call /approach/start std_srvs/srv/Trigger       # moves the arm
    ros2 service call /approach/stop  std_srvs/srv/Trigger

camera_to_robot has no default: the approach needs the cube in shoulder_roll_axis, and the transform
must be measured on the robot (vision_node README). Do not run calibrate --ros alongside.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_launch = os.path.join(get_package_share_directory("arm_control"), "launch", "arm.launch.py")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("result_json", description="the camera program's inference.json"),
        DeclareLaunchArgument("camera_to_robot", description="[tx, ty, tz, roll_deg, pitch_deg, yaw_deg], measured"),
        DeclareLaunchArgument("force", default_value="false",
                              description="start despite unverified calibration (never a pose problem)"),
        DeclareLaunchArgument("send_finger_rotation", default_value="false",
                              description="command finger_rotation (needs its calibration first)"),
        DeclareLaunchArgument("servo_channel", default_value="can0"),
        DeclareLaunchArgument("max_speed_deg_s", default_value="30.0"),
        DeclareLaunchArgument("calibration_file", default_value=""),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(arm_launch), launch_arguments={
            "servo_channel": arg("servo_channel"), "max_speed_deg_s": arg("max_speed_deg_s"),
            "calibration_file": arg("calibration_file")}.items()),
        Node(package="vision_node", executable="vision_node", name="vision_node", output="screen",
             emulate_tty=True,
             parameters=[{"result_json": arg("result_json"), "camera_to_robot": arg("camera_to_robot")}]),
        Node(package="arm_control", executable="approach_node", name="approach_node", output="screen",
             emulate_tty=True,
             parameters=[{"force": arg("force"), "send_finger_rotation": arg("send_finger_rotation"),
                          "calibration_file": arg("calibration_file")}]),
    ])
