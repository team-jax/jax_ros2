"""vision_node end to end: the real executable, one process per case, a subscriber here.

    python3 tests/check_node.py        (after colcon build and sourcing install/setup.bash)

Without fixed_pose, or with one that is not three finite numbers: nothing is published. With
fixed_pose:=[x,y,yaw]: PoseStamped at 1 Hz, frame_id shoulder_roll_axis, yaw about z only.
"""
import os
import signal
import subprocess
import sys
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


def run_case(node, seconds, *params):
    """Starts vision_node with -p params, collects /object_pose for `seconds`, stops it. Returns
    [(receive time, msg)] and the number of publishers that were seen on /object_pose."""
    got = []
    sub = node.create_subscription(PoseStamped, "/object_pose", lambda m: got.append((time.monotonic(), m)),
                                   QoSProfile(depth=50))
    args = ["ros2", "run", "vision_node", "vision_node", "--ros-args"]
    for p in params:
        args += ["-p", p]
    proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    publishers = 0
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        publishers = max(publishers, node.count_publishers("/object_pose"))
    os.killpg(proc.pid, signal.SIGTERM)
    proc.wait(timeout=10)
    node.destroy_subscription(sub)
    return got, publishers


def main():
    rclpy.init()
    node = rclpy.create_node("check_vision_node")

    got, pubs = run_case(node, 4.0)
    check("no parameter: nothing published", len(got) == 0, f"{len(got)} messages")
    check("no parameter: not even a publisher on /object_pose", pubs == 0, f"{pubs} publishers")

    for bad in ("[0.1, 0.2]", "[0.1, -0.3, 20.0, 5.0]", "[.nan, 0.0, 0.0]", "[.inf, 0.0, 0.0]", "abc"):
        got, pubs = run_case(node, 3.5, f"fixed_pose:={bad}")
        check(f"fixed_pose:={bad}: nothing published", len(got) == 0 and pubs == 0, f"{len(got)} messages")

    got, pubs = run_case(node, 6.5, "fixed_pose:=[0.10, -0.30, 20.0]")
    check("fixed_pose: published", len(got) >= 5, f"{len(got)} messages in 6.5 s")
    times = [t for t, _ in got]
    gaps = [b - a for a, b in zip(times, times[1:])]
    check("1 Hz: every gap between 0.9 and 1.1 s", gaps and all(0.9 <= g <= 1.1 for g in gaps),
          [round(g, 3) for g in gaps])
    if got:
        m = got[-1][1]
        check("frame_id is shoulder_roll_axis", m.header.frame_id == "shoulder_roll_axis", m.header.frame_id)
        check("position x, y in metres", (round(m.pose.position.x, 6), round(m.pose.position.y, 6)) == (0.1, -0.3))
        check("position z is 0.0 (fixed_pose has no height)", m.pose.position.z == 0.0)
        q = m.pose.orientation
        check("rotation is a yaw about z only", q.x == 0.0 and q.y == 0.0)
        check("yaw 20 deg (counter-clockwise from above)", round(degrees(2 * atan2(q.z, q.w)), 6) == 20.0,
              round(degrees(2 * atan2(q.z, q.w)), 6))
        check("quaternion has unit length", abs(q.z ** 2 + q.w ** 2 - 1.0) < 1e-12)
        check("stamp is set", m.header.stamp.sec > 0)

    got, _ = run_case(node, 3.5, "fixed_pose:=[1, 2, -90]")
    check("integer list [1, 2, -90] is accepted, yaw -90", got and round(
        degrees(2 * atan2(got[-1][1].pose.orientation.z, got[-1][1].pose.orientation.w)), 6) == -90.0)

    node.destroy_node()
    rclpy.shutdown()
    print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
