"""vision_node with result_json, end to end, no camera: a temporary inference.json is written by this script.

    python3 tests/check_camera_mode.py        (after colcon build and sourcing install/setup.bash)

Real `ros2 run` processes, a subscriber here. The file is changed while the node runs; old files are
made by setting the modification time back.
"""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from math import atan2, degrees

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile

failures = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail != "" else ""))
    if not ok:
        failures.append(name)


def obj(instance=1, center=(0.0715, 0.0906, 0.4457), yaw=6.05, dims=(60.3, 55.5, 2.8), depth=0.442):
    return {"instance": instance, "label": "blob", "status": "ok", "dims_mm": list(dims), "depth_rect_m": depth,
            "center_cam1_m": list(center), "rotation": {"yaw_deg": yaw}}


NOISE = {"instance": 9, "label": "blob", "status": "ok", "dims_mm": [93.2, 38.1, 7.1], "depth_rect_m": 0.789,
         "center_cam1_m": [0.5, -0.2, 0.789], "rotation": {"yaw_deg": 80.0}}


def write(path, *objs, age=0.0, raw=None):
    """Replaces the file atomically; `age` seconds back in time as modification time."""
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(raw if raw is not None else json.dumps({"objects": {"seg": list(objs)}, "ms": time.time()}))
    t = time.time() - age
    os.utime(tmp, (t, t))
    os.replace(tmp, path)


class Run:
    def __init__(self, rosnode, *params):
        self.got, self.rosnode = [], rosnode
        self.sub = rosnode.create_subscription(PoseStamped, "/object_pose",
                                               lambda m: self.got.append((time.monotonic(), m)), QoSProfile(depth=50))
        self.err = tempfile.TemporaryFile()
        args = ["ros2", "run", "vision_node", "vision_node", "--ros-args"]
        for p in params:
            args += ["-p", p]
        self.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=self.err, start_new_session=True)
        self.publishers = 0

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.rosnode, timeout_sec=0.05)
            self.publishers = max(self.publishers, self.rosnode.count_publishers("/object_pose"))

    def alive(self):
        return self.proc.poll() is None

    def stop(self):
        if self.alive():
            os.killpg(self.proc.pid, signal.SIGTERM)
            self.proc.wait(timeout=10)
        self.rosnode.destroy_subscription(self.sub)
        self.err.seek(0)
        return self.err.read().decode(errors="replace")


def yaw_of(m):
    return round(degrees(2 * atan2(m.pose.orientation.z, m.pose.orientation.w)), 6)


def main():
    rclpy.init()
    node = rclpy.create_node("check_vision_camera")
    d = tempfile.mkdtemp()
    p = os.path.join(d, "inference.json")

    # --- 1. camera_to_robot empty: camera frame, values as they are
    write(p, obj(), NOISE)
    r = Run(node, f"result_json:={p}")
    r.spin(3.0)
    log = r.stop()
    check("camera frame: published once for an unchanged file (no duplicates)", len(r.got) == 1, f"{len(r.got)} messages in 3 s")
    if r.got:
        m = r.got[-1][1]
        check("camera frame: frame_id is camera, not shoulder_roll_axis", m.header.frame_id == "camera", m.header.frame_id)
        check("camera frame: position = center_cam1_m as it is",
              (m.pose.position.x, m.pose.position.y, m.pose.position.z) == (0.0715, 0.0906, 0.4457),
              (m.pose.position.x, m.pose.position.y, m.pose.position.z))
        check("camera frame: yaw = rotation.yaw_deg (camera-referenced)", yaw_of(m) == 6.05, yaw_of(m))
        q = m.pose.orientation
        check("camera frame: x, y of quaternion 0, unit length", q.x == 0 and q.y == 0 and abs(q.z ** 2 + q.w ** 2 - 1) < 1e-12)
        check("camera frame: stamp set", m.header.stamp.sec > 0)
    check("camera frame: start log shows the allowed labels", "cube_labels ['blob', 'kite']" in log)
    check("camera frame: log says it is camera coordinates, once", log.count("임시: 카메라 좌표, 로봇 좌표 아님") == 1,
          log.count("임시: 카메라 좌표, 로봇 좌표 아님"))

    # --- 2. dynamic: change, same content, broken JSON, two candidates, stale, recovery
    write(p, obj(), NOISE)
    r = Run(node, f"result_json:={p}", "max_age_s:=3.0")
    r.spin(2.0)
    n0 = len(r.got)
    check("dynamic: first value published", n0 == 1, n0)
    write(p, obj(center=(0.2, -0.1, 0.5), yaw=-30.0), NOISE)
    r.spin(1.5)
    check("dynamic: file changed -> new value published", len(r.got) == n0 + 1 and r.got[-1][1].pose.position.x == 0.2
          and yaw_of(r.got[-1][1]) == -30.0, [(m.pose.position.x, yaw_of(m)) for _, m in r.got])
    n1 = len(r.got)
    write(p, raw='{"objects": {"seg": [{"instance": 1, "label": "bl')
    r.spin(1.5)
    check("dynamic: broken JSON -> nothing published, node still alive", len(r.got) == n1 and r.alive())
    write(p, obj(1), obj(2, center=(0.3, 0.3, 0.5)))
    r.spin(1.5)
    check("dynamic: two candidates -> nothing published", len(r.got) == n1 and r.alive())
    write(p, obj(center=(0.25, 0.0, 0.6), yaw=10.0))
    r.spin(1.5)
    check("dynamic: after the broken/ambiguous files, a good file publishes again",
          len(r.got) == n1 + 1 and r.got[-1][1].pose.position.x == 0.25, len(r.got) - n1)
    n2 = len(r.got)
    write(p, obj(center=(0.9, 0.9, 0.9), yaw=1.0), age=10.0)
    r.spin(4.0)
    check("stale: a 10 s old file (max_age_s 3) is not published", len(r.got) == n2 and r.alive(), len(r.got) - n2)
    write(p, obj(center=(0.31, 0.0, 0.6), yaw=11.0))
    r.spin(1.5)
    check("stale: fresh file publishes again", len(r.got) == n2 + 1 and r.got[-1][1].pose.position.x == 0.31)
    os.remove(p)
    r.spin(1.5)
    check("missing file: nothing more published, node alive", len(r.got) == n2 + 1 and r.alive())
    log = r.stop()
    check("log: stale warning present, not repeated at every poll (4 s stale, 2 polls/s)",
          1 <= log.count("(stale)") <= 2, log.count("(stale)"))
    check("log: broken JSON and ambiguity reasons present", "not valid JSON" in log and "ambiguous" in log)
    check("log: warnings are throttled (<= 12 lines for ~12 s)", log.count("WARN") <= 12, log.count("WARN"))

    # --- 3. camera_to_robot given: robot frame. yaw 90 about z, t=[1,0,0]: Rz·[a,b,c] = [-b,a,c] -> [1-b, a, c]
    write(p, obj(center=(0.0715, 0.0906, 0.4457), yaw=6.05), NOISE)
    r = Run(node, f"result_json:={p}", "camera_to_robot:=[1.0, 0.0, 0.0, 0.0, 0.0, 90.0]")
    r.spin(3.0)
    r.stop()
    if r.got:
        m = r.got[-1][1]
        got = (round(m.pose.position.x, 9), round(m.pose.position.y, 9), round(m.pose.position.z, 9))
        check("robot frame: frame_id shoulder_roll_axis", m.header.frame_id == "shoulder_roll_axis", m.header.frame_id)
        check("robot frame: position = [1-0.0906, 0.0715, 0.4457] by hand", got == (0.9094, 0.0715, 0.4457), got)
        check("robot frame: yaw still the camera value 6.05", yaw_of(m) == 6.05, yaw_of(m))
    else:
        check("robot frame: published", False)

    # --- 4. bad parameters: nothing published, no publisher, process ends its work quietly
    write(p, obj(), NOISE)
    for bad in ("camera_to_robot:=[1.0, 0.0, 0.0, 0.0, 0.0]", "camera_to_robot:=[.nan, 0.0, 0.0, 0.0, 0.0, 0.0]",
                "cube_dim_mm:=[80.0, 40.0]", "cube_labels:=[]", "cube_labels:=abc", "depth_range_m:=[0.3]", "poll_hz:=0.0", "max_age_s:=-1.0",
                "fixed_pose:=[0.1, -0.3, 20.0]"):
        r = Run(node, f"result_json:={p}", bad)
        r.spin(2.5)
        log = r.stop()
        check(f"{bad}: nothing published, no publisher, error logged",
              len(r.got) == 0 and r.publishers == 0 and "ERROR" in log, f"{len(r.got)} msgs, {r.publishers} pubs")

    # --- 5. integer parameters are accepted (poll_hz:=2, max_age_s:=3, ranges as ints)
    write(p, obj(), NOISE)
    r = Run(node, f"result_json:={p}", "poll_hz:=2", "max_age_s:=3", "cube_dim_mm:=[40, 80]", "depth_range_m:=[1, 1]")
    r.spin(2.5)
    log = r.stop()
    check("integer parameters accepted (depth_range_m [1, 1] excludes the cube -> nothing, no error)",
          len(r.got) == 0 and "ERROR" not in log and "no cube candidate" in log, log[-200:])

    node.destroy_node()
    rclpy.shutdown()
    print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
