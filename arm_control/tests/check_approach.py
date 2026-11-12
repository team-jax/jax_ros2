"""Approach (RAISE -> SETTLE -> INSERT) from the deploy tables, with no ROS and no torch.

    python3 -B arm_control/tests/check_approach.py      # needs numpy (python3-numpy on Jazzy)

Expected numbers come from IntergrateSim artifacts/deploy_2026-11-05/README.md, not from this code:
  7.1 knot commands at the reference cube, 7.2 handover pose, 7.3 insertion deltas, 8.x theta,
  and the ROS <-> deploy frame reference point in IntergrateSim/ROS2_INTEGRATION_PLAN.md section 3.
"""
import sys
from math import radians
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from arm_control import model_tables, model_frames  # noqa: E402
from arm_control.model_tables import Approach, ApproachTables, read_table  # noqa: E402

TABLES = ROOT / "model"
failures = []


def close(name, got, want, tol):
    got, want = np.asarray(got, float), np.asarray(want, float)
    ok = got.shape == want.shape and bool(np.all(np.abs(got - want) <= tol))
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {np.round(got, 3).tolist()}"
          + ("" if ok else f" (want {want.tolist()} +-{tol})"))
    if not ok:
        failures.append(name)


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


# --- .pt tables read without torch -------------------------------------------------------------
t = read_table(TABLES / "raise_grid_v65_basec.pt")
check("raise table shapes (ts, gx, gy, dq)",
      [t["ts"].shape, t["gx"].shape, t["gy"].shape, t["dq"].shape],
      [(6,), (13,), (31,), (6, 13, 31, 8)])
close("raise table ts (README 7.1)", t["ts"], [5.0, 5.6, 6.2, 6.8, 7.4, 8.0], 1e-6)
close("raise table gx ends (README 7.1: -0.20 .. +0.04 m)", t["gx"][[0, -1]], [-0.20, 0.04], 1e-6)
close("raise table gy ends (README 7.1: -0.30 .. +0.30 m)", t["gy"][[0, -1]], [-0.30, 0.30], 1e-6)

# --- 7.1 knot commands at the reference cube (dx, dy) = (0, 0), e = 0 ---------------------------
ref = ApproachTables(TABLES, (0.0, 0.0))
knots = {5.0: [-37.15, 0, 1.38, 62.86, -5.19, 55.39, -8.87, -170.1],
         5.6: [-34.78, 0, 1.40, 63.01, -5.11, 52.87, -8.93, -170.1],
         6.2: [-32.29, 0, 1.40, 62.75, -5.10, 50.55, -8.95, -170.1],
         6.8: [-29.71, 0, 1.39, 62.34, -5.12, 48.47, -8.95, -170.1],
         7.4: [-26.95, 0, 1.36, 61.51, -5.22, 46.56, -8.90, -170.1],
         8.0: [-24.13, 0, 1.33, 60.31, -5.33, 44.94, -8.84, -170.1]}
for when, want in knots.items():
    close(f"7.1 q_cmd({when} s) at the reference cube, deg", np.rad2deg(ref.q_cmd(when)), want, 0.011)
close("7.1 q_cmd past 8 s holds the last knot", np.rad2deg(ref.q_cmd(9.0)), knots[8.0], 0.011)
close("7.1 q_cmd(0) is the hanging pose", ref.q_cmd(0.0), np.zeros(8), 1e-12)

# --- 7.3 insertion deltas (e = 0, 12 deg included). The README examples predate the 11-11
#     pregrasp lowering (e_F), so they are checked with lower_pregrasp=False. ----------------------
deltas = {(0.0, 0.0): [7.33, 0, 0.20, -1.86, -0.02, -5.46, 12.52, 0],
          (0.0, -0.10): [7.75, 0, -2.00, -2.69, -0.21, -5.05, 11.45, 0],
          (0.0, 0.20): [9.95, 0, 5.62, -5.35, -0.85, -3.36, 16.79, 0],
          (0.02, -0.28): [15.38, 0, -14.16, -13.01, 2.10, 3.29, 0.51, 0]}
for xy, want in deltas.items():
    close(f"7.3 insertion delta at {xy}, deg",
          np.rad2deg(ApproachTables(TABLES, xy, lower_pregrasp=False).insertion()), want, 0.011)
lowered = ApproachTables(TABLES, (0.0, 0.0)).insertion()
unlowered = ApproachTables(TABLES, (0.0, 0.0), lower_pregrasp=False).insertion()
check("7.3 the 0.5 cm lowering (e_F = -0.005) changes the delta", bool(np.abs(lowered - unlowered).max() > 1e-6), True)

# --- theta (README 8.x) -----------------------------------------------------------------------------
for xy, want in {(0.0, 0.0): 0.0, (0.0, -0.10): 15.9, (0.0, 0.20): -33.8, (0.02, -0.28): 57.6}.items():
    close(f"theta at {xy}, deg", ApproachTables(TABLES, xy).theta_deg, want, 0.06)

# --- reach mask ---------------------------------------------------------------------------------------
raises("cube outside the table grid", lambda: ApproachTables(TABLES, (0.30, 0.0)))
raises("non-finite cube offset", lambda: ApproachTables(TABLES, (float("nan"), 0.0)))

# --- the 60 Hz approach, README 7.1-7.3 (servo mode, real robot: fixed 8.65 s handover) -----------
a = Approach(ApproachTables(TABLES, (0.0, 0.0)))
frames = []
while a.phase != "HOLD":
    frames.append((a.phase, a.t, a.step()))
    if len(frames) > 2000:
        break
phases = [p for p, _, _ in frames]
check("phase order", list(dict.fromkeys(phases)), ["RAISE", "SETTLE", "INSERT"])
check("RAISE runs t = 0 .. 8.0 s inclusive (481 ticks, as V3Controller)", phases.count("RAISE"), 481)
check("SETTLE 8.0167 .. 8.65 s, fixed handover of README 7.2 (39 ticks)", phases.count("SETTLE"), 39)
check("INSERT is 30 + 15 ticks", phases.count("INSERT"), 45)
q = np.array([f for _, _, f in frames])
names = model_tables.DEPLOY_JOINTS
arm = [names.index(n) for n in model_tables.ARM]
step = np.abs(np.diff(q[: 481 + 39, arm[:7]], axis=0)).max()
check("RAISE/SETTLE: 7 arm joints move <= 0.5 rad/s * dt", bool(step <= 0.5 / 60 + 1e-12), True)
handover = q[481 + 39 - 1, arm]
close("7.2 handover pose = knot 8.0 within 3 deg", np.rad2deg(handover), knots[8.0], 3.0)
d = ApproachTables(TABLES, (0.0, 0.0)).insertion()
close("7.3 insertion end = clamp(q_h + delta)", q[-1, arm], np.clip(handover + d, *model_tables.ARM_LIMITS.T), 1e-9)
thumb = names.index("rotation_base_to_finger_3_1")
close("fingers: thumb first joint -30 deg after 4 s, others 0", np.rad2deg(q[-1, [names.index(n) for n in model_tables.FINGERS]]),
      [0, 0, -30, 0, 0, 0], 1e-9)
close("thumb at 1 s is still 0", q[60, thumb], 0.0, 1e-12)
check("HOLD repeats the last frame", bool(np.array_equal(a.step(), q[-1])), True)

# --- ROS shoulder_roll_axis <-> deploy world (ROS2_INTEGRATION_PLAN.md section 3) -------------------
close("ROS reference point -> deploy world", model_frames.ros_to_world((-0.048, -0.285, -0.4475)),
      (-0.285, 0.26, 0.760), 1e-9)
close("deploy world -> ROS round trip", model_frames.world_to_ros(model_frames.ros_to_world((0.1, -0.3, -0.45))),
      (0.1, -0.3, -0.45), 1e-12)
check("cube yaw is taken mod 90 into +-45 deg", [model_frames.cube_yaw_deg(v) for v in (0, 50, -50, 135, 90)],
      [0.0, -40.0, 40.0, 45.0, 0.0])

# --- deploy joint -> ROS /arm_control/joint_command --------------------------------------------------
cmd = model_frames.to_ros_command(dict(zip(names, q[-1])))
check("14 ROS joints", sorted(cmd), sorted(model_frames.ROS_NAME.values()))
close("elbow passes through (rad)", cmd["elbow"], q[-1, names.index("upper_arm_to_under_arm")], 0)
close("finger_rotation: deploy -170.1 deg -> old-IK 9.9 deg (deploy + 180)", np.rad2deg(cmd["finger_rotation"]), 9.9, 1e-6)

print(f"\n{len(failures)} failed" if failures else "\nall passed")
raise SystemExit(1 if failures else 0)
