"""stm_bridge_node end to end, no hardware. A python-can virtual bus stands in for can0. Without
python-can a recording fake replaces ServoCan (the node logic is still checked, the bus is not).

    python3 tests/check_node.py
"""
import sys
import time
from math import radians

import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState

from stm_bridge import stm_bridge_node

try:
    import can
except ImportError:
    can = None

failures = []


def check(name, got, want):
    ok = got == want
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {got}" + ("" if ok else f" (want {want})"))
    if not ok:
        failures.append(name)


class FakeCan:
    sent = []

    def __init__(self, interface="socketcan", channel="can0"):
        pass

    def send(self, frame):
        FakeCan.sent.append(str(frame))

    def close(self):
        pass


def main():
    args = ["--ros-args", "-p", "servo_interface:=virtual", "-p", "servo_channel:=t"]
    if can is None:
        print("SKIP  python-can not installed: FakeCan instead of a virtual bus")
        stm_bridge_node.ServoCan = FakeCan
    rclpy.init(args=args)
    node = stm_bridge_node.StmBridgeNode()
    peer = rclpy.create_node("test_peer")
    commands = peer.create_publisher(JointState, "/servo/command", 10)
    states = []
    peer.create_subscription(JointState, "/servo/state", lambda m: states.append(m), 10)
    rx = can.Bus(interface="virtual", channel="t") if can else None
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(peer)

    def run(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.01)
        if rx:
            frames = []
            while (m := rx.recv(timeout=0)) is not None:
                frames.append(f"{m.arbitration_id:03X}#{bytes(m.data).hex().upper()}")
            return frames
        frames, FakeCan.sent = FakeCan.sent, []
        return frames

    def send(names, degs, positions=None):
        msg = JointState()
        msg.name = names
        msg.position = positions if positions is not None else [radians(d) for d in degs]
        commands.publish(msg)

    time.sleep(0.5)    # discovery
    frames = run(0.5)
    check("no frame and no state before the first command", (frames, len(states)), ([], 0))

    send(["W_PITCH", "NOPE"], [20.0, 0.0])
    frames = run(0.5)
    check("unknown name: message dropped whole", (frames, len(states)), ([], 0))

    send(["W_PITCH", "F1_1"], [0.0], positions=[1.0])
    frames = run(0.5)
    check("length mismatch: message dropped whole", (frames, len(states)), ([], 0))

    send(["W_PITCH", "F1_1"], None, positions=[1.0, float("nan")])
    frames = run(0.5)
    check("NaN: message dropped whole", (frames, len(states)), ([], 0))

    send(["W_PITCH", "F1_1"], [110.0, 90.0])
    frames = run(0.5)
    check("W_PITCH 110 deg is motor 9", "005#09044C" in frames, True)
    check("F1_1 is motor 0 at 90 deg", "005#000384" in frames, True)
    check("exactly two frames", len(frames), 2)
    last = states[-1]
    check("state names in motor_id order", list(last.name), ["F1_1", "W_PITCH"])
    check("state is the angle sent, rad", [round(p, 6) for p in last.position], [round(radians(90.0), 6), round(radians(110.0), 6)])

    send(["W_PITCH"], [200.0])
    frames = run(0.5)
    check("200 deg is clipped to 180", frames, ["005#090708"])
    check("state holds the clipped angle and the other servo's last one",
          [round(p, 6) for p in states[-1].position], [round(radians(90.0), 6), round(radians(180.0), 6)])

    node.destroy_node()
    peer.destroy_node()
    if rx:
        rx.shutdown()
    rclpy.shutdown()
    print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
