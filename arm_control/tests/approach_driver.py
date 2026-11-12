"""Test driver for approach_node: plays vision_node's part, calls /approach/start, records what is sent.

    python3 approach_driver.py --out run.json [--frame shoulder_roll_axis] [--xyz -0.048 -0.285 -0.4475]
                               [--yaw 20] [--wait 4] [--timeout 25]

Publishes /object_pose at 1 Hz (like vision_node on a ~1 s inference.json), waits --wait s, calls
/approach/start, then records /arm_control/joint_command and /approach/phase until the phase is HOLD or
STOPPED (or --timeout). Writes {"start": {success, message}, "phases": [...], "commands": [[t, names,
positions], ...], "final_phase": ...}.
"""
import argparse
import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from math import cos, radians, sin


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--frame", default="shoulder_roll_axis")
    ap.add_argument("--xyz", nargs=3, type=float, default=[-0.048, -0.285, -0.4475])
    ap.add_argument("--yaw", type=float, default=20.0)
    ap.add_argument("--wait", type=float, default=4.0)
    ap.add_argument("--timeout", type=float, default=25.0)
    ap.add_argument("--stop-after", type=float, default=None, help="call /approach/stop this many s after start")
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("approach_driver")
    pose_pub = node.create_publisher(PoseStamped, "/object_pose", 10)
    out = {"start": None, "phases": [], "commands": [], "stop": None}

    def pose():
        m = PoseStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = a.frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = a.xyz
        m.pose.orientation.z, m.pose.orientation.w = sin(radians(a.yaw) / 2), cos(radians(a.yaw) / 2)
        pose_pub.publish(m)

    node.create_timer(1.0, pose)
    t0 = time.monotonic()
    node.create_subscription(JointState, "/arm_control/joint_command",
                             lambda m: out["commands"].append([time.monotonic() - t0, list(m.name), list(m.position)]), 100)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/approach/phase", lambda m: out["phases"].append([time.monotonic() - t0, m.data]), latched)
    start = node.create_client(Trigger, "/approach/start")
    stop = node.create_client(Trigger, "/approach/stop")

    end = time.monotonic() + a.wait
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not start.wait_for_service(timeout_sec=10.0):
        out["start"] = {"success": False, "message": "no /approach/start service"}
    else:
        fut = start.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        r = fut.result()
        out["start"] = {"success": r.success, "message": r.message} if r else {"success": False, "message": "timeout"}
        out["start_t"] = time.monotonic() - t0
    deadline = time.monotonic() + a.timeout
    stopped = False
    while time.monotonic() < deadline and out["start"]["success"]:
        rclpy.spin_once(node, timeout_sec=0.02)
        if a.stop_after is not None and not stopped and time.monotonic() - t0 - out["start_t"] > a.stop_after:
            fut = stop.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            out["stop"] = {"success": fut.result().success, "message": fut.result().message}
            stopped = True
        if out["phases"] and out["phases"][-1][1] in ("HOLD", "STOPPED"):
            break
    for _ in range(20):          # the last messages in flight
        rclpy.spin_once(node, timeout_sec=0.02)
    out["final_phase"] = out["phases"][-1][1] if out["phases"] else None
    with open(a.out, "w") as f:
        json.dump(out, f)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
