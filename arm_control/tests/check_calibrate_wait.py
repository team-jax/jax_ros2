"""calibrate --ros waits for stm_bridge before the first servo command (RosOutput.wait_for_stm_bridge).
No bus. The other end is a node in this process. The real-process version is tests/calibrate_vcan.sh.

    python3 tests/check_calibrate_wait.py
"""
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "stm_bridge"))

import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

from arm_control import calibration
from arm_control.calibrate import RosOutput, Session

failures = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail != "" else ""))
    if not ok:
        failures.append(name)


def main():
    rclpy.init()
    node = rclpy.create_node("check_calibrate")
    peer = rclpy.create_node("check_stm_bridge")
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    ex.add_node(peer)
    spinner = threading.Thread(target=ex.spin, daemon=True)
    spinner.start()
    cal = calibration.load()
    out = RosOutput(node, servo_wait_s=1.0)
    sess = Session(cal, out, 30, out=lambda s: None, sleep=lambda t: None)
    got = []

    # 1) nobody on /servo/command: refused after the wait, nothing published, the joint not marked as moved
    t0 = time.monotonic()
    sess.handle("j W_PITCH")
    try:
        sess.handle("m 100")
        check("no subscriber: refused", False, "no exception")
    except RuntimeError as e:
        took = time.monotonic() - t0
        check("no subscriber: refused with RuntimeError after about 1 s", 0.9 < took < 1.6, f"{took:.2f} s")
        check("the message says what to check", "stm_bridge" in str(e) and "nothing sent" in str(e), str(e))
    check("the joint's last command is not recorded", "wrist_pitch" not in sess.pos)

    # 2) the subscriber appears 0.6 s after the command started waiting: the first message is not lost
    def late():
        time.sleep(0.6)
        peer.create_subscription(JointState, "/servo/command", lambda m: got.append(m), 10)

    out.servo_wait_s = 5.0
    threading.Thread(target=late, daemon=True).start()
    t0 = time.monotonic()
    sess.handle("m 100")
    took = time.monotonic() - t0
    time.sleep(0.5)
    check("late subscriber: the command waited for it", 0.5 < took < 2.5, f"{took:.2f} s")
    check("late subscriber: the first command arrived (W_PITCH 100 deg = 1.745329 rad)",
          len(got) == 1 and list(got[0].name) == ["W_PITCH"] and round(got[0].position[0], 6) == 1.745329, len(got))

    # 3) with the subscriber there, a second command does not wait
    t0 = time.monotonic()
    sess.handle("m +10")
    check("subscriber present: no wait", time.monotonic() - t0 < 0.5 + 1.0, f"{time.monotonic() - t0:.2f} s (includes the ramp)")
    time.sleep(0.5)
    check("the second move arrived too", len(got) > 1)

    ex.shutdown()       # stop the spin thread first: shutting rclpy down under it aborts the process
    spinner.join(timeout=5)
    node.destroy_node()
    peer.destroy_node()
    rclpy.shutdown()
    print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
