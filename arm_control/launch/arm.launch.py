"""ak45_node (ak45_ros2, its own params file) + stm_bridge_node + arm_control_node.

    ros2 launch arm_control arm.launch.py [servo_channel:=can0] [max_speed_deg_s:=30.0] [calibration_file:=...]

Both CAN users share one bus: ak45_node (AK45 4x, extended ids, always can0) and stm_bridge_node (servo
frames, standard id 5, servo_channel). Do not run `ros2 run arm_control calibrate --ros` together with
this launch: arm_control_node and calibrate would command the same motors.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ak45_launch = os.path.join(get_package_share_directory("ak45_ros2"), "launch", "ak45_node.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("servo_channel", default_value="can0",
                              description="SocketCAN interface of stm_bridge_node"),
        DeclareLaunchArgument("max_speed_deg_s", default_value="30.0",
                              description="speed limit of every motor command, deg/s"),
        DeclareLaunchArgument("calibration_file", default_value="",
                              description="empty: the package's calibration.json"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(ak45_launch)),
        Node(
            package="stm_bridge",
            executable="stm_bridge_node",
            name="stm_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[{"servo_channel": LaunchConfiguration("servo_channel")}],
        ),
        Node(
            package="arm_control",
            executable="arm_control_node",
            name="arm_control_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "max_speed_deg_s": LaunchConfiguration("max_speed_deg_s"),
                "calibration_file": LaunchConfiguration("calibration_file"),
            }],
        ),
    ])
