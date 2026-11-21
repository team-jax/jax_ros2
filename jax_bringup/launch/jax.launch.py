"""The whole arm: ak45_node, stm_bridge, arm_control_node, vision_node, approach_node.

    ros2 launch jax_bringup jax.launch.py                              # camera (config/jax.yaml result_json)
    ros2 launch jax_bringup jax.launch.py pose:="[0.10, -0.30, 20.0]"  # no camera: fixed cube [x m, y m, yaw deg]
        [force:=true] [grasp:=false] [fingers:=true] [result_json:=/path/inference.json]
fingers:=true: after the approach the arm holds and only the fingers move (close, hold 2 s, open): a finger
servo check that needs no pressure sensors (without them every finger closes to its limit).

Settings: config/jax.yaml. Usually started with `ros2 run jax_bringup up`, which first stops old nodes and
resets can0. A fixed pose has no height, so approach_node's table-height check is turned off for it.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    arg = lambda n: LaunchConfiguration(n).perform(context).strip()   # noqa: E731
    config = arg("config")
    pose = yaml.safe_load(arg("pose")) if arg("pose") else None
    flags = {"force": bool(yaml.safe_load(arg("force"))), "grasp": bool(yaml.safe_load(arg("grasp"))),
             "grasp_mode": "fingers" if yaml.safe_load(arg("fingers")) else "policy"}
    node = lambda pkg, exe, name, params: Node(package=pkg, executable=exe, name=name, output="screen",  # noqa: E731
                                                emulate_tty=True, parameters=params)
    if pose is None:
        vision = [os.path.join(share("vision_node"), "config", "camera.yaml"), config]
        if arg("result_json"):
            vision.append({"result_json": arg("result_json")})
        mode = "camera"
    else:
        if not (isinstance(pose, list) and len(pose) == 3):
            raise RuntimeError(f"pose must be [x, y, yaw_deg], got {arg('pose')!r}")
        vision = [{"fixed_pose": [float(v) for v in pose]}]
        flags["max_z_error_m"] = -1.0
        mode = f"fixed pose {pose}"
    return [
        LogInfo(msg=f"jax_bringup: {mode}, force {flags['force']}, grasp {flags['grasp']} ({flags['grasp_mode']}), "
                    f"settings {config}"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share("ak45_ros2"), "launch", "ak45_node.launch.py"))),
        node("stm_bridge", "stm_bridge_node", "stm_bridge", [config]),
        node("arm_control", "arm_control_node", "arm_control_node", [config]),
        node("vision_node", "vision_node", "vision_node", vision),
        node("arm_control", "approach_node", "approach_node", [config, flags]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share("jax_bringup"), "config", "jax.yaml")),
        DeclareLaunchArgument("result_json", default_value="", description="override jax.yaml's inference.json path"),
        DeclareLaunchArgument("pose", default_value="", description="[x, y, yaw_deg]: fixed cube, no camera"),
        DeclareLaunchArgument("force", default_value="false", description="start although calibration is unverified"),
        DeclareLaunchArgument("grasp", default_value="true", description="grasp and lift after the approach"),
        DeclareLaunchArgument("fingers", default_value="false",
                              description="instead of the learned grasp: finger test (arm still, close, hold, open)"),
        OpaqueFunction(function=nodes),
    ])
