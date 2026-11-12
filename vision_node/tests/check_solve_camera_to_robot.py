"""tools/solve_camera_to_robot.py: recovers a known transform and agrees with camera_to_robot (no ROS).

    python3 vision_node/tests/check_solve_camera_to_robot.py
"""
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "..", "tools", "solve_camera_to_robot.py")
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
from solve_camera_to_robot import solve  # noqa: E402
from vision_node.camera_result import camera_to_robot  # noqa: E402

failures = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail != "" else ""))
    if not ok:
        failures.append(name)


Z = -0.4525


def rot(roll, pitch, yaw):
    """Independent matrix form of Rz(yaw)·Ry(pitch)·Rx(roll), degrees (not the code under test)."""
    r, p, y = np.radians([roll, pitch, yaw])
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def synth(true, robot_xy, noise=None):
    """Camera points that the transform `true` maps onto robot points (x, y, Z); optional noise (m)."""
    t = np.array(true[:3])
    r = rot(*true[3:])
    pts = []
    for k, (x, y) in enumerate(robot_xy):
        cam = r.T @ (np.array([x, y, Z]) - t)
        if noise is not None:
            cam = cam + noise[k]
        pts.append((tuple(cam), (x, y)))
    return pts


def fmt(pts):
    args = []
    for c, r in pts:
        args += ["--point", f"{c[0]:.9f},{c[1]:.9f},{c[2]:.9f}:{r[0]},{r[1]}"]
    return args


def run(*args):
    p = subprocess.run([sys.executable, TOOL, *args], capture_output=True, text=True)
    return p.returncode, p.stdout, p.stderr


XY = [(0.10, -0.30), (-0.05, -0.25), (0.15, -0.40), (0.02, -0.35)]
for true in ([0.12, -0.45, 0.30, 20.0, -35.0, 110.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
             [-0.2, 0.1, 0.5, -170.0, 10.0, -60.0], [0.3, -0.6, 0.4, 150.0, 60.0, 5.0]):
    got, res = solve(synth(true, XY), Z)
    check(f"synthetic {true[3:]}: six numbers recovered", np.allclose(got, true, atol=1e-6), [round(v, 6) for v in got])
    check(f"synthetic {true[3:]}: residuals 0", max(res) < 1e-6, [round(v, 9) for v in res])

# the solved numbers fed to the existing camera_to_robot give the robot coordinates
true = [0.12, -0.45, 0.30, 20.0, -35.0, 110.0]
pts = synth(true, XY)
got, _ = solve(pts, Z)
ok = all(np.allclose(camera_to_robot(c, got), (r[0], r[1], Z), atol=1e-9) for c, r in pts)
check("camera_to_robot(point, solved) = robot x, y, -0.4525 for every point", ok)

# through the command line, with the printed -p line
code, out, err = run(*fmt(pts))
m = re.search(r'-p camera_to_robot:="\[([^\]]+)\]"', out)
check("CLI: exit 0 and a paste line", code == 0 and m is not None, out[-200:])
if m:
    vals = [float(v) for v in m.group(1).split(",")]
    check("CLI: pasted six numbers give the robot coordinates through camera_to_robot",
          len(vals) == 6 and all(np.allclose(camera_to_robot(c, vals), (r[0], r[1], Z), atol=1e-5) for c, r in pts), vals)
check("CLI: one residual line per point, all 0.00 mm", len(re.findall(r"^point \d+: residual 0\.00 mm", out, re.M)) == 4, out)
check("CLI: no warning for a perfect fit", "WARNING" not in err, err)

# --z changes the robot z
true2 = [0.12, -0.45, 0.30, 20.0, -35.0, 110.0]
r2 = rot(*true2[3:])
pts_z = [(tuple(r2.T @ (np.array([x, y, -0.40]) - np.array(true2[:3]))), (x, y)) for x, y in XY]
code, out, _ = run(*fmt(pts_z), "--z", "-0.40")
m = re.search(r'-p camera_to_robot:="\[([^\]]+)\]"', out)
check("--z -0.40: recovered", m and np.allclose([float(v) for v in m.group(1).split(",")], true2, atol=1e-5), out[-150:])

# 3 points are enough; fewer, collinear, bad input are refused
code, out, _ = run(*fmt(synth(true, XY[:3])))
check("3 points: accepted", code == 0 and "3 points" in out, out[-100:])
code, _, err = run(*fmt(synth(true, XY[:2])))
check("2 points: refused (exit 2)", code == 2 and "at least 3" in err, err)
line = [(0.0, -0.3), (0.1, -0.3), (0.2, -0.3), (0.3, -0.3)]
code, _, err = run(*fmt(synth(true, line)))
check("points on one line: refused (exit 2)", code == 2 and "one line" in err, err)
code, _, err = run("--point", "1,2:3,4", "--point", "x", "--point", "1,1,1:0,0")
check("bad --point text: refused with the expected format", code == 2 and "cx,cy,cz:rx,ry" in err, err)
code, _, err = run("--point", "1,2,nan:3,4", "--point", "1,1,1:0,0", "--point", "2,1,1:0,1")
check("NaN: refused", code == 2 and "finite" in err, err)

# warning for a large mean residual: one point moved by 0.2 m in the camera
noise = [np.zeros(3), np.zeros(3), np.array([0.2, 0.0, 0.0]), np.zeros(3)]
code, out, err = run(*fmt(synth(true, XY, noise)))
mean = float(re.search(r"mean residual ([0-9.]+) mm", out).group(1))
check("one point 200 mm off: mean residual above 20 mm -> WARNING (exit 0, line still printed)",
      code == 0 and mean > 20 and "WARNING" in err and "-p camera_to_robot" in out, f"mean {mean}")
# small noise (2 mm): no warning, residual about the noise
rng = np.random.default_rng(1)
noise = [rng.normal(0, 0.002, 3) for _ in XY]
code, out, err = run(*fmt(synth(true, XY, noise)))
mean = float(re.search(r"mean residual ([0-9.]+) mm", out).group(1))
check("2 mm noise: no warning, mean residual small", code == 0 and "WARNING" not in err and 0 < mean < 10, f"mean {mean}")

# Kabsch never returns a mirror: for non-planar points that are mirrored, det(R) = +1 and the residual is large
from solve_camera_to_robot import kabsch  # noqa: E402
cube = np.array([[0, 0, 0], [1, 0, 0], [0, 2, 0], [0, 0, 3], [1, 1, 1]], dtype=float)
r_true = rot(10, 20, 30)
good = cube @ r_true.T + np.array([1.0, 2.0, 3.0])
r, t = kabsch(cube, good)
check("kabsch: 3D points, rotation recovered, residual 0", np.allclose(r, r_true) and np.allclose(cube @ r.T + t, good))
bad = good * np.array([-1.0, 1.0, 1.0])            # mirror image of the target
r, t = kabsch(cube, bad)
check("kabsch: mirrored 3D target -> proper rotation (det +1)", abs(np.linalg.det(r) - 1) < 1e-9, np.linalg.det(r))
check("kabsch: mirrored 3D target -> residual is large (not hidden)", np.abs(cube @ r.T + t - bad).max() > 0.1)

print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
sys.exit(1 if failures else 0)
