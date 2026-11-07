"""Load for tests/chain_vcan.sh: six servos and one AK45 moved by arm_control_node at 50 Hz.

Counts the joints that arm_control_node publishes on /servo/command (one servo frame each is expected on
the bus) and prints them as JSON, with the motor angle every servo should end at.
"""
import json
import sys
import time
from math import radians
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "stm_bridge"))

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from arm_control import calibration

START = {"wrist_pitch": 0, "wrist_yaw": 0, "hand_roll": 0, "finger1_joint1": 0, "finger1_joint2": 10, "finger2_joint1": 0,
         "elbow": 0}
GOAL = {"wrist_pitch": 60, "wrist_yaw": -60, "hand_roll": 100, "finger1_joint1": 45, "finger1_joint2": 100,
        "finger2_joint1": -70, "elbow": 20}


def main():
    rclpy.init()
    node = rclpy.create_node("chain_load")
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=2000)
    pub = node.create_publisher(JointState, "/arm_control/joint_command", 10)
    seen = {"joints": 0, "messages": 0, "last": None, "t": time.monotonic()}
    state = {"messages": 0}

    def on_servo(m):
        seen["joints"] += len(m.name)
        seen["messages"] += 1
        seen["t"] = time.monotonic()

    node.create_subscription(JointState, "/servo/command", on_servo, qos)
    node.create_subscription(JointState, "/servo/state", lambda m: state.update(messages=state["messages"] + 1), qos)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def send(d):
        m = JointState()
        m.name = list(d)
        m.position = [radians(v) for v in d.values()]
        pub.publish(m)

    spin(3.0)                  # discovery
    send(START)
    spin(1.5)
    send(GOAL)
    while True:                # until /servo/command has been quiet for 2.5 s
        spin(0.2)
        if time.monotonic() - seen["t"] > 2.5:
            break
    cal = calibration.load()
    final = {cal[n].name: round(cal[n].to_motor(v)[0] * 10) for n, v in GOAL.items() if cal[n].motor == "servo"}
    print(json.dumps({"servo_joints_published": seen["joints"], "servo_messages": seen["messages"],
                      "servo_state_messages": state["messages"], "final_x10": final}))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
