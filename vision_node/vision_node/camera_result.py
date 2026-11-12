"""Pure helpers (no ROS) for reading the camera program's inference.json and choosing the cube.

The camera program (SteroDepthSeg, z_phone_stereo.py) overwrites <out>/inference.json about once a
second. Layout checked on real files (phone_stereo/runs/cube_test/inference.json):

    {"objects": {"seg": [ {"instance": 1, "label": "blob", "status": "ok", "dims_mm": [w, h, thick],
                           "depth_rect_m": 0.469, "center_cam1_m": [x, y, z],
                           "rotation": {"yaw_deg": 6.0, ...}, ...}, ... ]}, ...}

`objects` maps a method name ("seg" there) to a list; all lists are looked at, the name is not fixed.
center_cam1_m is in camera coordinates (OpenCV: x right, y down, z forward), metres.
"""
import json
import os
from math import cos, isfinite, radians, sin


def numbers(v, n):
    """v as a tuple of n finite floats, or None."""
    if (isinstance(v, (list, tuple)) and len(v) == n
            and all(isinstance(a, (int, float)) and not isinstance(a, bool) and isfinite(a) for a in v)):
        return tuple(float(a) for a in v)
    return None


def read_result(path, now, max_age_s):
    """Reads the file. Returns (raw bytes, None), or (None, reason) when it is missing, unreadable or
    older than max_age_s (file modification time vs `now`, both epoch seconds): an arm must not use
    an old position."""
    try:
        with open(path, "rb") as f:
            raw = f.read()
            mtime = os.fstat(f.fileno()).st_mtime
    except OSError as e:
        return None, f"cannot read {path}: {e.strerror or e}"
    age = now - mtime
    if age > max_age_s:
        return None, f"{path} is {age:.1f} s old (max_age_s {max_age_s:g}): not used"
    return raw, None


def select_cube(raw, cube_dim_mm, depth_range_m):
    """Chooses the cube in the file content `raw` (bytes or str).

    Candidate: label "blob", status "ok", dims_mm[0] and dims_mm[1] both inside cube_dim_mm
    (inclusive), depth_rect_m inside depth_range_m (inclusive). Exactly one candidate is needed;
    with none or several nothing is chosen (never pick "some" cube).
    Returns ({"xyz": (x, y, z) camera metres, "yaw_deg": camera-referenced yaw, "instance": n}, None)
    or (None, reason)."""
    try:
        data = json.loads(raw)
    except ValueError as e:          # JSONDecodeError, and UnicodeDecodeError for bad bytes: both ValueError
        return None, f"not valid JSON (probably being rewritten): {e}"
    objects = data.get("objects") if isinstance(data, dict) else None
    if not isinstance(objects, dict):
        return None, "no 'objects' dict in the file"
    rows = [o for lst in objects.values() if isinstance(lst, list) for o in lst if isinstance(o, dict)]
    lo_d, hi_d = cube_dim_mm
    lo_z, hi_z = depth_range_m
    cands = []
    for o in rows:
        dims = numbers(o.get("dims_mm"), 3)
        depth = o.get("depth_rect_m")
        if (o.get("label") == "blob" and o.get("status") == "ok" and dims is not None
                and lo_d <= dims[0] <= hi_d and lo_d <= dims[1] <= hi_d
                and isinstance(depth, (int, float)) and not isinstance(depth, bool) and lo_z <= depth <= hi_z):
            cands.append(o)
    if len(cands) == 0:
        return None, f"no cube candidate among {len(rows)} objects"
    if len(cands) > 1:
        return None, f"{len(cands)} cube candidates (instances {[c.get('instance') for c in cands]}): ambiguous"
    c = cands[0]
    xyz = numbers(c.get("center_cam1_m"), 3)
    rot = c.get("rotation")
    yaw = rot.get("yaw_deg") if isinstance(rot, dict) else None
    if xyz is None or not isinstance(yaw, (int, float)) or isinstance(yaw, bool) or not isfinite(yaw):
        return None, f"candidate instance {c.get('instance')} has no valid center_cam1_m / rotation.yaw_deg"
    return {"xyz": xyz, "yaw_deg": float(yaw), "instance": c.get("instance")}, None


def camera_to_robot(p_cam, params):
    """TEMPORARY camera -> robot transform: p_robot = R · p_cam + t.

    params = (tx, ty, tz, roll_deg, pitch_deg, yaw_deg); t in metres. R = Rz(yaw) · Ry(pitch) · Rx(roll)
    (fixed axes, roll about x first, then pitch about y, then yaw about z; each a right-handed rotation
    of the point, positive = counter-clockwise looking against the axis; same as ROS rpy). Hand-checked
    examples are in tests/check_camera_result.py. The robot frame is the one of /object_pose.
    """
    tx, ty, tz, roll, pitch, yaw = params
    cr, sr = cos(radians(roll)), sin(radians(roll))
    cp, sp = cos(radians(pitch)), sin(radians(pitch))
    cy, sy = cos(radians(yaw)), sin(radians(yaw))
    x, y, z = p_cam
    y, z = cr * y - sr * z, sr * y + cr * z          # Rx(roll)
    x, z = cp * x + sp * z, -sp * x + cp * z         # Ry(pitch)
    x, y = cy * x - sy * y, sy * x + cy * y          # Rz(yaw)
    return (x + tx, y + ty, z + tz)
