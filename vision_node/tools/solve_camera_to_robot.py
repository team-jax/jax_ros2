#!/usr/bin/env python3
"""Solves vision_node's camera_to_robot parameter from points seen by the camera at known robot positions.

    python3 vision_node/tools/solve_camera_to_robot.py \
        --point "0.0715,0.0906,0.4457:0.10,-0.30" --point "...:..." --point "...:..."

--point "cx,cy,cz:rx,ry": cx,cy,cz = center_cam1_m of the cube in the camera program's inference.json
(camera coordinates, metres: x right, y down, z forward); rx,ry = the cube centre in robot coordinates
(metres; origin = centre of the right shoulder roll motor axis, x = robot's left, y = back, z = up).
The robot z is the same for every point: --z, default -0.4525 m (shoulder axis 1212.5 mm above the
floor, desk 735 mm, cube centre 25 mm above the desk: 0.735 + 0.025 - 1.2125).
At least 3 points, not on one line. More points average the measurement error out.

Method: rigid transform between corresponding points (Kabsch, SVD, det(R) = +1 so no mirror), the same
convention as camera_to_robot in vision_node/camera_result.py: p_robot = R · p_cam + t,
R = Rz(yaw) · Ry(pitch) · Rx(roll). Prints the six numbers, the residual of every point in mm and a
line to paste into the vision_node command. A mean residual above 20 mm is warned about.

Limit: all points are at one height, so they lie in a plane. A residual of 0 therefore does NOT prove
the axes are right: a mirrored input (one coordinate with the wrong sign) is also fitted perfectly, with a
different rotation. Check the signs of what is typed, and check the result with a cube at a new place.
"""
import argparse
import sys
from math import atan2, degrees, hypot, isfinite

import numpy as np

DEFAULT_Z = -0.4525
WARN_MEAN_MM = 20.0
LINE_TOL = 1e-3     # second / first singular value of the centred points below this = "on one line"


def parse_point(text):
    """'cx,cy,cz:rx,ry' -> ((cx, cy, cz), (rx, ry)); ValueError when it is not that."""
    try:
        cam, rob = text.split(":")
        c = [float(v) for v in cam.split(",")]
        r = [float(v) for v in rob.split(",")]
    except ValueError:
        raise ValueError(f"bad --point {text!r}: expected 'cx,cy,cz:rx,ry' (numbers, metres)") from None
    if len(c) != 3 or len(r) != 2 or not all(isfinite(v) for v in c + r):
        raise ValueError(f"bad --point {text!r}: expected 'cx,cy,cz:rx,ry' (three and two finite numbers)")
    return tuple(c), tuple(r)


def is_collinear(points):
    s = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
    return s[0] == 0 or s[1] / s[0] < LINE_TOL


def kabsch(cam, rob):
    """R, t with rob ≈ cam @ R.T + t (det R = +1). cam, rob: (n, 3) arrays."""
    cc, rc = cam.mean(axis=0), rob.mean(axis=0)
    u, _, vt = np.linalg.svd((cam - cc).T @ (rob - rc))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d if d != 0 else 1.0]) @ u.T
    return r, rc - r @ cc


def to_rpy_deg(r):
    """(roll, pitch, yaw) in degrees with R = Rz(yaw) · Ry(pitch) · Rx(roll); pitch in [-90, 90]."""
    sp = float(np.clip(-r[2, 0], -1.0, 1.0))
    pitch = np.arcsin(sp)
    if abs(sp) > 1 - 1e-9:                  # gimbal lock: only roll -/+ yaw is defined, put it all in yaw
        roll, yaw = 0.0, atan2(-r[0, 1], r[1, 1])
    else:
        roll, yaw = atan2(r[2, 1], r[2, 2]), atan2(r[1, 0], r[0, 0])
    return degrees(roll), degrees(pitch), degrees(yaw)


def solve(points, z):
    """points: [((cx, cy, cz), (rx, ry)), ...]. Returns (params [tx,ty,tz,roll,pitch,yaw], residuals_mm).
    Raises ValueError for fewer than 3 points or points on one line."""
    if len(points) < 3:
        raise ValueError(f"need at least 3 points, got {len(points)}")
    cam = np.array([p[0] for p in points], dtype=float)
    rob = np.array([[p[1][0], p[1][1], z] for p in points], dtype=float)
    if is_collinear(cam) or is_collinear(rob):
        raise ValueError("the points lie on one line: the rotation is not determined; "
                         "put the cube at points that are not in a row")
    r, t = kabsch(cam, rob)
    res_mm = [1000.0 * float(np.linalg.norm(r @ c + t - p)) for c, p in zip(cam, rob)]
    return [float(v) for v in t] + list(to_rpy_deg(r)), res_mm


def glue_point_values(argv):
    """'--point -0.05,..' -> '--point=-0.05,..': argparse would take a value that starts with '-' for an option
    (robot y is negative in front of the robot, so that is the usual case)."""
    out, i = [], 0
    while i < len(argv):
        if argv[i] == "--point" and i + 1 < len(argv):
            out.append("--point=" + argv[i + 1])
            i += 2
        else:
            out.append(argv[i])
            i += 1
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="camera_to_robot from points at known robot positions")
    ap.add_argument("--point", action="append", default=[], metavar="cx,cy,cz:rx,ry",
                    help="center_cam1_m : robot x,y (metres); repeat, at least 3")
    ap.add_argument("--z", type=float, default=DEFAULT_Z, help=f"robot z of every point, m (default {DEFAULT_Z})")
    a = ap.parse_args(glue_point_values(sys.argv[1:] if argv is None else list(argv)))
    try:
        if not isfinite(a.z):
            raise ValueError("--z must be finite")
        params, res = solve([parse_point(p) for p in a.point], a.z)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    names = ("tx", "ty", "tz", "roll_deg", "pitch_deg", "yaw_deg")
    print("camera_to_robot: " + ", ".join(f"{n} {v:.6f}" for n, v in zip(names, params)))
    for i, m in enumerate(res, 1):
        print(f"point {i}: residual {m:.2f} mm")
    mean = sum(res) / len(res)
    print(f"mean residual {mean:.2f} mm, max {max(res):.2f} mm, {len(res)} points, robot z {a.z:g} m")
    if mean > WARN_MEAN_MM:
        print(f"WARNING: mean residual {mean:.1f} mm is above {WARN_MEAN_MM:g} mm: a point is wrong "
              "(typo, cube not at the measured spot, wrong detection) or the axes are mixed up. "
              "Do not use these numbers before checking.", file=sys.stderr)
    print('-p camera_to_robot:="[' + ", ".join(f"{v:.6f}" for v in params) + ']"')
    return 0


if __name__ == "__main__":
    sys.exit(main())
