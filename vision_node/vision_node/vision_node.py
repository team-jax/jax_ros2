"""vision_node (skeleton): /object_pose for tests, no camera yet.

    ros2 run vision_node vision_node --ros-args -p fixed_pose:="[0.10, -0.30, 20.0]"

fixed_pose = [x, y, yaw]: x, y in metres, yaw in DEGREES (counter-clockwise seen from above, like
`pick_place --cube x y yaw`). Without the parameter, or with one that is not three finite numbers,
nothing is published.

/object_pose  geometry_msgs/PoseStamped at 1 Hz, frame_id "shoulder_roll_axis": origin = centre of the
right shoulder roll motor's axis, x = robot's left (+), y = back (+, the front is -y), z = up. The
rotation is a yaw about z only. fixed_pose has no height, so position.z is 0.0 and means nothing.
The camera (SteroDepthSeg) is not connected here.
"""
from math import cos, isfinite, radians, sin

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

FRAME_ID = "shoulder_roll_axis"
TOPIC = "/object_pose"
PERIOD_S = 1.0


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        # dynamic typing: a list like [1, 2, 3] (integers) or a string is then ours to reject, not a crash
        pose = self.declare_parameter("fixed_pose", None, ParameterDescriptor(dynamic_typing=True)).value
        self.pose = None
        if pose is None or (isinstance(pose, (list, tuple)) and len(pose) == 0):
            self.get_logger().info("no fixed_pose: nothing is published (the camera is not connected)")
        elif (not isinstance(pose, (list, tuple)) or len(pose) != 3
              or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and isfinite(v) for v in pose)):
            self.get_logger().error(f"fixed_pose must be three finite numbers [x, y, yaw], got {pose!r}; "
                                    "nothing is published")
        else:
            self.pose = tuple(float(v) for v in pose)
            self.pub = self.create_publisher(PoseStamped, TOPIC, 10)
            self.create_timer(PERIOD_S, self.on_timer)
            self.get_logger().info(f"{TOPIC} at {1 / PERIOD_S:g} Hz, {FRAME_ID}: x {self.pose[0]:g} m, "
                                   f"y {self.pose[1]:g} m, yaw {self.pose[2]:g} deg")

    def on_timer(self):
        x, y, yaw = self.pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z = sin(radians(yaw) / 2.0)
        msg.pose.orientation.w = cos(radians(yaw) / 2.0)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):    # Ctrl+C, or SIGTERM from kill / launch
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
