"""approach_plan: camera pose -> checked approach plan, with no ROS.

    python3 -B arm_control/tests/check_approach_plan.py
"""
import sys
from math import degrees, radians
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "stm_bridge"))
from arm_control import calibration  # noqa: E402
from arm_control.approach_plan import PoseGate, plan_from_pose, check_plan  # noqa: E402

TABLES = ROOT / "model"
REF_ROS = (-0.048, -0.285, -0.4475)      # = deploy world (-0.285, 0.26, 0.760), the reference cube
failures = []


def check(name, got, want):
    ok = got == want
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {got}" + ("" if ok else f" (want {want})"))
    if not ok:
        failures.append(name)


def raises(name, fn):
    try:
        fn()
    except ValueError as e:
        print(f"PASS  {name}: rejected ({e})")
        return
    print(f"FAIL  {name}: accepted")
    failures.append(name)


# --- PoseGate: which /object_pose may start the arm -----------------------------------------------
g = PoseGate(stable=2, tol_m=0.01, max_age_s=3.0)
g.add("camera", (0.1, 0.2, 0.5), 0.0, now=0.0)
check("camera-frame pose is not used", g.ready(now=0.1)[0], None)
check("... and says why", "camera" in g.ready(now=0.1)[1], True)
g = PoseGate(stable=2, tol_m=0.01, max_age_s=3.0)
g.add("shoulder_roll_axis", REF_ROS, 10.0, now=0.0)
check("one pose is not enough (stable=2)", g.ready(now=0.1)[0], None)
g.add("shoulder_roll_axis", (REF_ROS[0] + 0.03, REF_ROS[1], REF_ROS[2]), 10.0, now=1.0)
check("a pose 3 cm away restarts the count", g.ready(now=1.1)[0], None)
g.add("shoulder_roll_axis", (REF_ROS[0] + 0.035, REF_ROS[1], REF_ROS[2]), 12.0, now=2.0)
pose = g.ready(now=2.1)[0]
check("two poses within 1 cm: ready, the mean of the two", None if pose is None else [round(v, 4) for v in pose[0]],
      [round(REF_ROS[0] + 0.0325, 4), REF_ROS[1], REF_ROS[2]])
check("yaw is the mean of the two (deg)", None if pose is None else round(pose[1], 3), 11.0)
check("stale after max_age_s", g.ready(now=5.5)[0], None)

# --- plan_from_pose -----------------------------------------------------------------------------------
plan = plan_from_pose(REF_ROS, 20.0, TABLES)
check("reference cube -> deploy world", [round(v, 4) for v in plan.world], [-0.285, 0.26, 0.76])
check("offset (dx, dy)", [round(v, 6) for v in plan.offset], [0.0, 0.0])
check("frames: 481 RAISE + 39 SETTLE + 45 INSERT", len(plan.frames), 565)
check("phases", list(dict.fromkeys(p for p, _ in plan.frames)), ["RAISE", "SETTLE", "INSERT"])
check("finger_rotation is not sent by default (calibration [0, 0])",
      "finger_rotation" in plan.frames[0][1], False)
check("13 joints per frame", len(plan.frames[-1][1]), 13)
check("elbow at the end of INSERT (deg, README 7.2 + 7.3, lowered pregrasp)",
      round(degrees(plan.frames[-1][1]["elbow"]), 1), round(degrees(plan.final_deploy[3]), 1))
plan14 = plan_from_pose(REF_ROS, 20.0, TABLES, send_finger_rotation=True)
check("send_finger_rotation=True: 14 joints", len(plan14.frames[-1][1]), 14)
raises("cube 32 cm in front of the table grid (ROS y -0.6 -> world x -0.6)",
       lambda: plan_from_pose((-0.048, -0.6, -0.4475), 0.0, TABLES))
raises("cube height 10 cm off the table (bad camera_to_robot?)",
       lambda: plan_from_pose((REF_ROS[0], REF_ROS[1], REF_ROS[2] + 0.10), 0.0, TABLES))
check("max_z_error_m < 0 skips the height check",
      len(plan_from_pose((REF_ROS[0], REF_ROS[1], REF_ROS[2] + 0.10), 0.0, TABLES, max_z_error_m=-1).frames), 565)

# --- check_plan against calibration.json ------------------------------------------------------------
cal = calibration.load(ROOT / "calibration.json")
problems = check_plan(plan, cal)
check("13 joints: nothing clipped", [p for p in problems if "cut short" in p], [])
check("unverified calibration is reported", any("not verified" in p for p in problems), True)
check("14 joints: finger_rotation would be cut short",
      any(p.startswith("finger_rotation") and "cut short" in p for p in check_plan(plan14, cal)), True)

print(f"\n{len(failures)} failed" if failures else "\nall passed")
raise SystemExit(1 if failures else 0)
