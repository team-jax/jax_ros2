"""The learned grasp (grasp.py, robot_v3.py) without ROS, torch or MuJoCo.

    python3 -B arm_control/tests/check_grasp.py

Expected numbers come from JaxIsaacLab artifacts/deploy_2026-11-05, not from this code:
  - handover_state_25550_example.json (README 7.8): joint angles -> hand / fingertip midpoint / cube offset,
    recorded in the sim with base_link at (0, 0.01, -0.0575);
  - grasp_lift_25550.onnx outputs, computed with onnxruntime 1.32 (the .npz here is exported from it);
  - README 4: shoulder pitch axis (0, 0.16, 1.2075), 7.7: L pose (0, 0, 7.2, 90, 0, -2.6, 0) deg.
The last part runs the whole grasp against a crude fake hand (force grows past a touch angle): it shows
the phases and the rules run end to end, not that the real cube will be held.
"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from arm_control import model_tables as mt  # noqa: E402
from arm_control.grasp import LIMITS, FingerTest, GraspController, MlpPolicy, Q_L  # noqa: E402
from arm_control.robot_v3 import RobotV3, rot_to_quat  # noqa: E402

MODEL = HERE.parent / "model"
failures = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f": {detail}" if not (isinstance(detail, str) and detail == "") else ""))
    if not ok:
        failures.append(name)


# --- forward kinematics against the recorded handover state ---------------------------------------
ARM_Q = [-0.22, -0.0, 7.2, 89.71, 0.11, -2.65, 0.14, -169.41]
FINGER_Q = [10.47, 11.09, -30.27, 0.0, 0.0, 25.51]
sim = RobotV3(MODEL / "robot_v3.urdf", base_pos=[0, 0.01, -0.0575])
q = dict(zip(mt.ARM + mt.FINGERS, np.deg2rad(ARM_Q + FINGER_Q)))
rot, hand = sim.hand(q)
mid = sim.fingertip_mid(q)
check("hand position = recorded (-0.2636, 0.2956, 0.8923) within 0.5 mm",
      np.abs(hand - [-0.2636, 0.2956, 0.8923]).max() < 5e-4, hand.round(4))
check("hand quaternion = recorded (-0.4816, 0.4533, -0.5167, 0.5437) within 1e-3",
      np.abs(rot_to_quat(rot) - [-0.4816, 0.4533, -0.5167, 0.5437]).max() < 1e-3, rot_to_quat(rot).round(4))
check("fingertip midpoint = recorded (-0.3946, 0.2962, 0.8909) within 0.5 mm",
      np.abs(mid - [-0.3946, 0.2962, 0.8909]).max() < 5e-4, mid.round(4))
check("cube - midpoint = recorded (-33.8, -13.6, -25.3) mm within 0.5 mm",
      np.abs((np.array([-0.4284, 0.2826, 0.8656]) - mid) * 1000 - [-33.8, -13.6, -25.3]).max() < 0.5)
robot = RobotV3(MODEL / "robot_v3.urdf")
check("README 4 placement: shoulder pitch axis at (0, 0.16, 1.2075)",
      np.abs(robot.link("Shoulder_Pitch", {})[1] - [0, 0.16, 1.2075]).max() < 1e-9)
check("ROS shoulder_roll_axis origin (model_frames) at (0, 0.212, 1.2075)",
      np.abs(robot.link("Shoulder_Roll", {})[1] - [0, 0.212, 1.2075]).max() < 1e-9)

# Jacobian: a small joint step moves the point / turns the hand as J says
names = [mt.ARM[i] for i in (0, 2, 3, 4, 5, 6)]
jac = robot.point_jacobian(q, names, mid)
dq = np.deg2rad([0.1, -0.1, 0.1, 0.05, -0.05, 0.1])
q2 = dict(q)
for n, d in zip(names, dq):
    q2[n] += d
r0, p0 = robot.hand(q)
r1, p1 = robot.hand(q2)
moved = (p1 + r1 @ (r0.T @ (mid - p0))) - mid
check("Jacobian predicts the point's move within 2 % (0.1 deg steps)",
      np.linalg.norm(jac[:3] @ dq - moved) < 0.02 * np.linalg.norm(moved), (jac[:3] @ dq - moved).round(7))

# --- the policy file against onnxruntime -------------------------------------------------------------
pol = MlpPolicy(MODEL / "grasp_lift_25550.npz")
mean = pol.w["mean"]
for name, obs, want in (
        ("obs = mean", mean, [-2.01632, 1.16214, -1.71433, -6.59797, -1.93908, -1.40684, 0.34002, 0.28749, -1.81679, -0.76059, 0.33506]),
        ("obs = mean + 0.1", mean + 0.1, [-2.86636, -11.26828, 5.04961, 19.9135, 9.46742, 7.31615, -0.8271, -3.56271, 0.7926, 4.38726, 0.44019]),
        ("obs = 0", np.zeros(45), [-11.05406, -33.77904, 41.97832, 114.81274, 66.32388, 0.18679, -1.82077, -1.87458, 7.42157, 9.31471, -0.40017])):
    got = pol.raw(obs)
    check(f"policy {name}: same 11 outputs as onnxruntime (within 1e-3 relative)",
          np.all(np.abs(got - want) <= 1e-3 * np.maximum(1, np.abs(want))), np.round(got, 4).tolist())
pol.reset()
a1 = pol(mean)
check("filter: first call = 0.25 * clip(raw, +-1)", np.allclose(a1, 0.25 * np.clip(pol.raw(mean), -1, 1)))


# --- the whole grasp against a fake hand -------------------------------------------------------------
def adc(fn):
    gf = np.asarray(fn) * 1000 / 9.80665
    loads = [5, 55, 100, 165, 260, 350, 470, 590, 750, 975, 1290, 1585, 1920, 2465, 3240, 4000]
    res = [60, 19.5, 11.5, 8.8, 7.5, 6.8, 5.4, 4.4, 4, 3.5, 3.1, 2.9, 2.7, 2.6, 2.55, 2.5]
    r = np.exp(np.interp(np.log(np.clip(gf, 5, 4000)), np.log(loads), np.log(res)))
    return np.where(gf < 5, 0, np.round(1023 * 10 / (r + 10)) / 1023)


def chain(cube, touch_deg=(6, 6, -26), stiffness=40.0, ticks=1500):
    tables = mt.ApproachTables(MODEL, cube[:2] - mt.TABLE_ORIGIN)
    ap = mt.Approach(tables)
    while ap.phase != "HOLD":
        ap.step()
    g = GraspController(robot, MlpPolicy(MODEL / "grasp_lift_25550.npz"), ap.last, cube, tables.theta_deg)
    pad, outs, phases = np.zeros(6), [], []
    for _ in range(ticks):
        outs.append(g.step(pad, adc(pad)))
        phases.append(g.phase)
        if touch_deg is not None:
            pad = np.r_[np.zeros(3), np.maximum(0, (g.target[8:11] - np.deg2rad(touch_deg)) * stiffness)]
        if g.phase in ("DONE", "TIMEOUT"):
            break
    return g, ap.last, np.array(outs), phases


cube = np.array([-0.285, 0.26, 0.76])            # the reference cube on the table (deploy world)
g, start, outs, phases = chain(cube)
seq = [p for i, p in enumerate(phases) if i == 0 or p != phases[i - 1]]
check("fake hand: phases GRASP -> RETURN -> DONE", seq == ["GRASP", "RETURN", "DONE"], seq)
t_done = len(phases) / 60
check("fake hand: done 4-9 s after the grasp starts (sim 5.00-8.72 s, README 3)", 4.0 <= t_done <= 9.0, f"{t_done:.2f} s")
check("fake hand: the arm ends at the L pose (README 7.7) within 0.01 deg",
      np.abs(np.rad2deg(g.target[:7] - Q_L)).max() < 0.01, np.round(np.rad2deg(g.target[:7]), 2).tolist())
check("fake hand: RotationBody target -170 deg", abs(np.rad2deg(g.target[7]) + 170) < 1e-9)
check("fake hand: the cube estimate rose >= 4 cm", g.cube[2] - g.lift_z >= 0.04, f"{(g.cube[2] - g.lift_z) * 100:.1f} cm")
step = np.abs(np.diff(np.vstack([start, outs]), axis=0)).max(axis=0)
check("first output = the INSERT end except the hand (0.5 s blend: no jump > 1 deg on the arm)",
      np.rad2deg(step[:8]).max() < 1.0, np.round(np.rad2deg(step[:8]), 2).tolist())
check("every target finite and inside the robot_v3 limits", np.all(np.isfinite(outs))
      and np.all(outs >= LIMITS[:, 0] - 1e-9) and np.all(outs <= LIMITS[:, 1] + 1e-9))
check("finger first joints inside 0..30 / thumb -30..0 deg",
      np.all(outs[:, 8:10] >= -1e-9) and np.all(outs[:, 8:10] <= np.deg2rad(30) + 1e-9)
      and np.all(outs[:, 10] <= 1e-9) and np.all(outs[:, 10] >= np.deg2rad(-30) - 1e-9))

g2, _, _, phases2 = chain(cube, touch_deg=None, ticks=700)
check("no contact at all: GRASP_TIMEOUT after 10 s, never RETURN",
      g2.phase == "TIMEOUT" and "RETURN" not in phases2 and len(phases2) == 600, (g2.phase, len(phases2), g2.reason))

for c in (np.array([-0.285, 0.16, 0.76]), np.array([-0.265, 0.40, 0.76])):
    gc, _, _, _ = chain(c)
    check(f"fake hand at world xy {c[:2].tolist()}: DONE", gc.phase == "DONE", (gc.phase, gc.reason))

# --- finger test (grasp_mode fingers): arm still, fingers close / hold / open ---------------------------
tables = mt.ApproachTables(MODEL, cube[:2] - mt.TABLE_ORIGIN)
ap = mt.Approach(tables)
while ap.phase != "HOLD":
    ap.step()
ft = FingerTest(ap.last)
outs, phases = [], []
for _ in range(1000):
    outs.append(ft.step(np.zeros(6)))
    phases.append(ft.phase)
    if ft.phase == "DONE":
        break
outs = np.array(outs)
seq = [p for i, p in enumerate(phases) if i == 0 or p != phases[i - 1]]
check("finger test, no sensors: FINGERS -> CLOSED -> OPEN -> DONE", seq == ["FINGERS", "CLOSED", "OPEN", "DONE"], seq)
check("finger test: closed at the limits 30, 30, 0 and tips 60 deg", np.allclose(np.rad2deg(ft.closed), [30, 30, 0, 60, 60, 60]),
      np.round(np.rad2deg(ft.closed), 1).tolist())
check("finger test: ends open (0, 0, -30, 0, 0, 0)", np.allclose(np.rad2deg(outs[-1, 8:]), [0, 0, -30, 0, 0, 0]))
check("finger test: the arm (8 joints) never moves", np.abs(outs[:, :8] - ap.last[:8]).max() < 1e-12)
check("finger test: first output keeps the hand as INSERT left it (no jump > 1 deg)",
      np.rad2deg(np.abs(outs[0, 8:] - ap.last[8:])).max() < 1.0, np.round(np.rad2deg(outs[0, 8:] - ap.last[8:]), 2).tolist())
check("finger test: 6-9 s in all (about 3.5 s closing, 2 s hold, 1.5 s open)", 6 <= len(phases) / 60 <= 9, f"{len(phases) / 60:.2f} s")
ft2 = FingerTest(ap.last)
for _ in range(400):
    ft2.step([0, 0, 0] + [max(0.0, (q - t) * 40) for q, t in zip(ft2.target[8:11], np.deg2rad([6, 6, -26]))])
    if ft2.phase != "FINGERS":
        break
check("finger test with pads: stops closing near the touch angle (fingers < 10 deg, thumb < -20 deg)",
      ft2.phase == "CLOSED" and np.rad2deg(ft2.closed[0]) < 10 and np.rad2deg(ft2.closed[2]) < -20,
      np.round(np.rad2deg(ft2.closed[:3]), 1).tolist())

print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
sys.exit(1 if failures else 0)
