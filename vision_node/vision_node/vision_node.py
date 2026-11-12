"""vision_node: /object_pose from a fixed test pose (fixed_pose) or from the camera program's inference.json (result_json).

    ros2 run vision_node vision_node --ros-args -p fixed_pose:="[0.10, -0.30, 20.0]"

fixed_pose = [x, y, yaw]: x, y in metres, yaw in DEGREES (counter-clockwise seen from above, like
`pick_place --cube x y yaw`). Without the parameter, or with one that is not three finite numbers,
nothing is published.

/object_pose  geometry_msgs/PoseStamped at 1 Hz, frame_id "shoulder_roll_axis": origin = centre of the
right shoulder roll motor's axis, x = robot's left (+), y = back (+, the front is -y), z = up. The
rotation is a yaw about z only. fixed_pose has no height, so position.z is 0.0 and means nothing.

Camera mode (parameter result_json = path of SteroDepthSeg's inference.json; it is rewritten about
once a second by z_phone_stereo.py, which runs in its own venv: this node only reads the file):

    ros2 run vision_node vision_node --ros-args -p result_json:=/path/to/runs/cube_test/inference.json

  poll_hz        2.0           how often the file is looked at; a changed file content is published once
  max_age_s      3.0           a file modified longer ago than this is not used (warning only)
  cube_labels    ["blob", "kite"]  allowed label of the cube; ["*"] = every label. "blob" = a bump over the
                               table plane, "kite" = what COCO (YOLO) called the cube in a real file
  cube_dim_mm    [40.0, 80.0]  both of dims_mm[0], dims_mm[1] must be inside (a 5 cm cube measured 60-68)
  depth_range_m  [0.3, 1.0]    depth_rect_m must be inside
  camera_to_robot  []          [tx, ty, tz, roll_deg, pitch_deg, yaw_deg]: p_robot = R · p_cam + t,
                               R = Rz(yaw)·Ry(pitch)·Rx(roll). TEMPORARY, the phone mount is not fixed yet.

  work_x_m, work_y_m, work_z_m  unset  [min, max] in ROBOT coordinates; a candidate outside is dropped (reason
                               logged). Needs camera_to_robot: without it the zone is not applied (one warning)
  score_weights  [1.0, 1.0]    [squareness, centrality]: score = (w0*(1 - |w-h|/max(w,h)) + w1*(1 - mean(|w-c|, |h-c|)/half)) / (w0+w1),
                               c = middle and half = half width of cube_dim_mm
  min_margin     0.05          best minus second best score must be at least this, else nothing ("ambiguous")
  track_tol_m    0.03          a candidate this close to the last published position is kept (no margin check);
                               the last position is forgotten after 3 s without a publish

A candidate is label in cube_labels, status "ok", size and depth inside the ranges, a valid center and yaw, inside
the work zone. One candidate is published; with several the best score wins if it leads by min_margin (or one is
near the last published position); otherwise nothing is, and the reason is logged (for each dropped object: label,
status, dims, depth or zone). Broken JSON (file being rewritten) is skipped.
frame_id: camera_to_robot empty -> "camera" (position = camera coordinates as they are, OpenCV: x right,
y down, z forward; NOT robot coordinates); camera_to_robot given -> "shoulder_roll_axis".
The yaw is rotation.yaw_deg of the camera program as it is, CAMERA-REFERENCED, not converted to the
robot frame (that needs the phone mount). position.z is the transformed z. fixed_pose and result_json
together: error, nothing is published.
"""
import time
from math import cos, isfinite, radians, sin

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from vision_node.camera_result import camera_to_robot, numbers, read_result, select_cube

FRAME_ID = "shoulder_roll_axis"
CAMERA_FRAME_ID = "camera"
TOPIC = "/object_pose"
PERIOD_S = 1.0
PREV_MAX_AGE_S = 3.0      # the previous published position is forgotten after this many seconds


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        # dynamic typing: a list like [1, 2, 3] (integers) or a string is then ours to reject, not a crash
        dyn = ParameterDescriptor(dynamic_typing=True)
        pose = self.declare_parameter("fixed_pose", None, dyn).value
        result_json = self.declare_parameter("result_json", "", dyn).value
        self.poll_hz = self.declare_parameter("poll_hz", 2.0, dyn).value
        self.max_age_s = self.declare_parameter("max_age_s", 3.0, dyn).value
        self.cube_labels = self.declare_parameter("cube_labels", ["blob", "kite"], dyn).value
        self.cube_dim_mm = self.declare_parameter("cube_dim_mm", [40.0, 80.0], dyn).value
        self.depth_range_m = self.declare_parameter("depth_range_m", [0.3, 1.0], dyn).value
        self.camera_to_robot = self.declare_parameter("camera_to_robot", None, dyn).value
        self.work = [self.declare_parameter(f"work_{a}_m", None, dyn).value for a in "xyz"]
        self.score_weights = self.declare_parameter("score_weights", [1.0, 1.0], dyn).value
        self.min_margin = self.declare_parameter("min_margin", 0.05, dyn).value
        self.track_tol_m = self.declare_parameter("track_tol_m", 0.03, dyn).value
        self.pose = None
        if result_json not in ("", None):
            self.setup_camera_mode(result_json, pose)
            return
        if pose is None or (isinstance(pose, (list, tuple)) and len(pose) == 0):
            self.get_logger().info("no fixed_pose: nothing is published (no result_json either)")
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
        self.publish_pose(x, y, 0.0, yaw, FRAME_ID)

    def publish_pose(self, x, y, z, yaw_deg, frame_id):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.z = sin(radians(yaw_deg) / 2.0)
        msg.pose.orientation.w = cos(radians(yaw_deg) / 2.0)
        self.pub.publish(msg)

    def setup_camera_mode(self, path, fixed_pose):
        """Validates the camera parameters; on any problem logs an error and publishes nothing."""
        err = None
        c2r = self.camera_to_robot
        if fixed_pose is not None and not (isinstance(fixed_pose, (list, tuple)) and len(fixed_pose) == 0):
            err = "fixed_pose and result_json are both set; use one"
        elif not isinstance(path, str):
            err = f"result_json must be a string, got {path!r}"
        elif c2r is not None and not (isinstance(c2r, (list, tuple)) and len(c2r) == 0) \
                and numbers(c2r, 6) is None:
            err = f"camera_to_robot must be empty or six finite numbers [tx, ty, tz, roll, pitch, yaw], got {c2r!r}"
        elif (not isinstance(self.cube_labels, (list, tuple)) or len(self.cube_labels) == 0
              or not all(isinstance(v, str) and v for v in self.cube_labels)):
            err = f"cube_labels must be a non-empty list of strings (or ['*']), got {self.cube_labels!r}"
        elif numbers(self.cube_dim_mm, 2) is None or not 0 < self.cube_dim_mm[0] <= self.cube_dim_mm[1]:
            err = f"cube_dim_mm must be [min, max] with 0 < min <= max, got {self.cube_dim_mm!r}"
        elif numbers(self.depth_range_m, 2) is None or not 0 < self.depth_range_m[0] <= self.depth_range_m[1]:
            err = f"depth_range_m must be [min, max] with 0 < min <= max, got {self.depth_range_m!r}"
        elif numbers([self.poll_hz], 1) is None or self.poll_hz <= 0:
            err = f"poll_hz must be a positive number, got {self.poll_hz!r}"
        elif numbers([self.max_age_s], 1) is None or self.max_age_s <= 0:
            err = f"max_age_s must be a positive number, got {self.max_age_s!r}"
        elif any(w is not None and not (isinstance(w, (list, tuple)) and len(w) == 0)
                 and (numbers(w, 2) is None or w[0] > w[1]) for w in self.work):
            err = f"work_x_m / work_y_m / work_z_m must each be empty or [min, max] with min <= max, got {self.work!r}"
        elif numbers(self.score_weights, 2) is None or min(self.score_weights) < 0 or sum(self.score_weights) <= 0:
            err = f"score_weights must be two numbers >= 0 with a positive sum, got {self.score_weights!r}"
        elif numbers([self.min_margin], 1) is None or self.min_margin < 0:
            err = f"min_margin must be a number >= 0, got {self.min_margin!r}"
        elif numbers([self.track_tol_m], 1) is None or self.track_tol_m < 0:
            err = f"track_tol_m must be a number >= 0, got {self.track_tol_m!r}"
        if err:
            self.get_logger().error(f"{err}; nothing is published")
            return
        self.path = path
        self.cube_labels = tuple(self.cube_labels)
        self.cube_dim_mm = numbers(self.cube_dim_mm, 2)
        self.depth_range_m = numbers(self.depth_range_m, 2)
        self.c2r = numbers(c2r, 6) if c2r else None
        self.work = tuple(numbers(w, 2) if w else None for w in self.work)
        self.score_weights = numbers(self.score_weights, 2)
        self.min_margin, self.track_tol_m = float(self.min_margin), float(self.track_tol_m)
        self.prev = None             # (position in the published frame, time.monotonic()) of the last publish
        self.last_raw = None
        self.last_note = {}
        self.pub = self.create_publisher(PoseStamped, TOPIC, 10)
        self.create_timer(1.0 / self.poll_hz, self.on_poll)
        if self.c2r is None and any(w is not None for w in self.work):
            self.get_logger().warning("work_x_m / work_y_m / work_z_m are set but camera_to_robot is empty: "
                                      "the work zone is NOT applied (it needs robot coordinates)")
        if self.c2r is None:
            self.get_logger().warning("임시: 카메라 좌표, 로봇 좌표 아님 (camera_to_robot is empty): "
                                      f"frame_id {CAMERA_FRAME_ID}; the yaw is camera-referenced")
        else:
            self.get_logger().info(f"camera_to_robot {self.c2r}: frame_id {FRAME_ID}; the yaw is camera-referenced")
        self.get_logger().info(f"{TOPIC} from {path} (poll_hz {self.poll_hz:g}, max_age_s {self.max_age_s:g}, "
                               f"cube_labels {list(self.cube_labels)}, cube_dim_mm {self.cube_dim_mm}, depth_range_m {self.depth_range_m}, "
                               f"work (x, y, z) {self.work}, score_weights {self.score_weights}, "
                               f"min_margin {self.min_margin:g}, track_tol_m {self.track_tol_m:g})")

    def note(self, reason, level="warning"):
        """Log when the reason changes, otherwise at most every 10 s (the file is polled 2x a second)."""
        last, t = self.last_note.get(level, (None, 0.0))
        if reason != last or time.monotonic() - t > 10.0:
            getattr(self.get_logger(), level)(reason)
            self.last_note[level] = (reason, time.monotonic())

    def on_poll(self):
        raw, why = read_result(self.path, time.time(), self.max_age_s)
        if raw is None:
            self.note(why)
            return
        if raw == self.last_raw:     # same file content as the last look: nothing new
            return
        self.last_raw = raw
        now = time.monotonic()
        if self.prev is not None and now - self.prev[1] > PREV_MAX_AGE_S:   # not updated for 3 s: forget it
            self.prev = None
        pick, why = select_cube(raw, self.cube_dim_mm, self.depth_range_m, self.cube_labels, self.c2r, self.work,
                                self.score_weights, self.min_margin, self.prev and self.prev[0], self.track_tol_m)
        if pick is None:
            self.note(why)
            return
        if pick["dropped"]:
            self.note("work zone dropped: " + "; ".join(pick["dropped"]), level="info")
        x, y, z = pick["pos"]
        self.prev = (pick["pos"], now)
        self.publish_pose(x, y, z, pick["yaw_deg"], CAMERA_FRAME_ID if self.c2r is None else FRAME_ID)


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
