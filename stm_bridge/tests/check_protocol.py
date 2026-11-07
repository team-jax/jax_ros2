"""Byte-level checks with no device and no ROS. Exit code 0 when everything passes.

    python3 tests/check_protocol.py

Expected bytes come from team-jax/CAN_STM Stm32servosepc.md section 8 (cansend examples), not from
this code.
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from stm_bridge.protocol import SERVO_STDID, SERVO_COUNT, servo_frame
from stm_bridge.servo_names import SERVO_IDS

failures = []


def check(name, got, want):
    ok = got == want
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {got}" + ("" if ok else f" (want {want})"))
    if not ok:
        failures.append(name)


def raises(name, fn):
    try:
        fn()
    except ValueError:
        print(f"PASS  {name}: rejected")
        return
    print(f"FAIL  {name}: accepted")
    failures.append(name)


def main():
    check("StdId is 5", SERVO_STDID, 5)
    check("servo 0 -> 0 deg", str(servo_frame(0, 0)), "005#000000")
    check("servo 3 -> 90 deg", str(servo_frame(3, 90)), "005#030384")
    check("servo 9 -> 180 deg", str(servo_frame(9, 180)), "005#090708")
    check("servo 3 -> 200 deg is clipped to 180", str(servo_frame(3, 200)), "005#030708")
    check("servo 3 -> -5 deg is clipped to 0", str(servo_frame(3, -5)), "005#030000")
    check("servo 4 -> 22.84 deg rounds to 228", str(servo_frame(4, 22.84)), "005#0400E4")
    check("servo 9 -> 110 deg", str(servo_frame(9, 110)), "005#09044C")
    check("frame is a standard frame", servo_frame(1, 1).extended, False)
    raises("servo motor_id 10", lambda: servo_frame(10, 90))
    raises("servo motor_id -1", lambda: servo_frame(-1, 90))

    check("servo names cover motor_id 0..9 once", sorted(SERVO_IDS.values()), list(range(SERVO_COUNT)))
    # the same names and ids as arm_control's calibration.json (skipped until that package exists)
    cal = ROOT.parent / "arm_control" / "calibration.json"
    if cal.exists():
        rows = json.loads(cal.read_text())["joints"]
        check("names match calibration.json", {r["name"]: r["id"] for r in rows if r["motor"] == "servo"},
              SERVO_IDS)
    else:
        print(f"SKIP  names vs calibration.json: {cal} not found")

    print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
