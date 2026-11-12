"""vision_node with camera_to_robot (and any other parameter) read from config/camera.yaml.

    ros2 launch vision_node vision.launch.py result_json:=/path/to/runs/cube_test/inference.json [config:=/other.yaml]

The yaml is passed to the node as a parameters file (node name vision_node). Without result_json the node
publishes nothing (it needs result_json or fixed_pose). The yaml value is only a default of the launch:
`ros2 run vision_node vision_node` without this launch still starts with an empty camera_to_robot.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("vision_node"), "config", "camera.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config,
                              description="parameters yaml (camera_to_robot ...)"),
        DeclareLaunchArgument("result_json", default_value="",
                              description="path of SteroDepthSeg's inference.json"),
        Node(
            package="vision_node",
            executable="vision_node",
            name="vision_node",
            output="screen",
            parameters=[LaunchConfiguration("config"), {"result_json": LaunchConfiguration("result_json")}],
        ),
    ])
