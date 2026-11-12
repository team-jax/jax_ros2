"""Pure helpers (no ROS) for reading the camera program's inference.json and choosing the cube.

The camera program (SteroDepthSeg, z_phone_stereo.py) overwrites <out>/inference.json about once a
second. Layout checked on real files (phone_stereo/runs/cube_test/inference.json):

    {"objects": {"seg": [ {"instance": 1, "label": "blob" (or "kite"), "status": "ok", "dims_mm": [w, h, thick],
                           "depth_rect_m": 0.469, "center_cam1_m": [x, y, z],
                           "rotation": {"yaw_deg": 6.0, ...}, ...}, ... ]}, ...}

`objects` maps a method name ("seg" there) to a list; all lists are looked at, the name is not fixed.
center_cam1_m is in camera coordinates (OpenCV: x right, y down, z forward), metres.
"""
import json
import os
from math import cos, dist, isfinite, radians, sin


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
    if now - mtime > max_age_s:      # no age in the text: the node logs a changed reason at once, the same one rarely
        return None, f"{path} was not modified within max_age_s {max_age_s:g} s: not used (stale)"
    return raw, None


DEFAULT_LABELS = ("blob", "kite")


def reject_reason(o, cube_labels, cube_dim_mm, depth_range_m):
    """None if object `o` is a candidate, else why not (the first failing one of label, status, dims, depth)."""
    label = o.get("label")
    if "*" not in cube_labels and label not in cube_labels:
        return f"label {label!r} not in cube_labels {list(cube_labels)}"
    if o.get("status") != "ok":
        return f"status {o.get('status')!r} is not 'ok'"
    dims = numbers(o.get("dims_mm"), 3)
    if dims is None:
        return f"dims_mm {o.get('dims_mm')!r} invalid"
    if not (cube_dim_mm[0] <= dims[0] <= cube_dim_mm[1] and cube_dim_mm[0] <= dims[1] <= cube_dim_mm[1]):
        return f"dims_mm {dims[0]:g}x{dims[1]:g} outside cube_dim_mm {list(cube_dim_mm)}"
    depth = o.get("depth_rect_m")
    if not isinstance(depth, (int, float)) or isinstance(depth, bool) or not isfinite(depth):
        return f"depth_rect_m {depth!r} invalid"
    if not depth_range_m[0] <= depth <= depth_range_m[1]:
        return f"depth_rect_m {depth:g} outside depth_range_m {list(depth_range_m)}"
    return None


def score_candidate(dims, cube_dim_mm, weights=(1.0, 1.0)):
    """Score in [0, 1] of a candidate with dims_mm[0], dims_mm[1] = dims (both inside cube_dim_mm):

        squareness = 1 - |w - h| / max(w, h)                      (1 = w equals h)
        centrality = 1 - mean(|w - c|, |h - c|) / half            (c = middle, half = half width of cube_dim_mm; 1 = both in the middle)
        score      = (weights[0] * squareness + weights[1] * centrality) / (weights[0] + weights[1])

    Hand example, cube_dim_mm [40, 80] (c 60, half 20), 50 x 40: squareness 1 - 10/50 = 0.8,
    centrality 1 - mean(10, 20)/20 = 0.25, equal weights -> 0.525."""
    w, h = dims[0], dims[1]
    square = 1.0 - abs(w - h) / max(w, h)
    c = (cube_dim_mm[0] + cube_dim_mm[1]) / 2.0
    half = (cube_dim_mm[1] - cube_dim_mm[0]) / 2.0
    central = 1.0 - ((abs(w - c) + abs(h - c)) / 2.0) / half if half > 0 else 1.0
    return (weights[0] * square + weights[1] * max(0.0, central)) / (weights[0] + weights[1])


AXES = ("x", "y", "z")


def work_reject_reason(p, work):
    """None if the robot-frame position p is inside work = (x_range, y_range, z_range) (each None or
    (min, max), inclusive), else why not."""
    for name, v, r in zip(AXES, p, work):
        if r is not None and not r[0] <= v <= r[1]:
            return f"robot {name} {v:.3f} outside work_{name}_m {[r[0], r[1]]}"
    return None


def select_cube(raw, cube_dim_mm, depth_range_m, cube_labels=DEFAULT_LABELS, c2r=None, work=(None, None, None),
                weights=(1.0, 1.0), min_margin=0.05, prev=None, track_tol_m=0.03):
    """Chooses the cube in the file content `raw` (bytes or str).

    Candidate: label in cube_labels ("*" in it allows every label; COCO/YOLO may call the cube "kite"),
    status "ok", dims_mm[0] and dims_mm[1] both inside cube_dim_mm (inclusive), depth_rect_m inside
    depth_range_m (inclusive), a valid center_cam1_m and rotation.yaw_deg. Then:
      1. work zone: only when c2r (camera_to_robot, 6 numbers) is given, a candidate whose robot position
         is outside work (x, y, z ranges, None = no limit) is dropped; without c2r the zone is not applied.
      2. previous choice: prev = last published position (in the published frame: robot if c2r else
         camera). A candidate within track_tol_m (metres) of it wins (the nearest one), so the choice
         does not jump between frames; no margin check then.
      3. otherwise the candidates are ranked by score_candidate; with one candidate that is the cube;
         with more, the best one only if its score exceeds the second's by at least min_margin, else
         nothing ("ambiguous": scores and positions in the reason).
    Returns ({"xyz": camera metres, "pos": position in the published frame, "yaw_deg": camera-referenced
    yaw, "instance": n, "score": s, "tracked": bool, "dropped": [work-zone reasons]}, None) or (None, reason)."""
    try:
        data = json.loads(raw)
    except ValueError as e:          # JSONDecodeError, and UnicodeDecodeError for bad bytes: both ValueError
        return None, f"not valid JSON (probably being rewritten): {e}"
    objects = data.get("objects") if isinstance(data, dict) else None
    if not isinstance(objects, dict):
        return None, "no 'objects' dict in the file"
    rows = [o for lst in objects.values() if isinstance(lst, list) for o in lst if isinstance(o, dict)]
    cands, dropped, work_dropped = [], [], []
    for o in rows:
        why = reject_reason(o, cube_labels, cube_dim_mm, depth_range_m)
        xyz = numbers(o.get("center_cam1_m"), 3)
        rot = o.get("rotation")
        yaw = rot.get("yaw_deg") if isinstance(rot, dict) else None
        if why is None and (xyz is None or not isinstance(yaw, (int, float)) or isinstance(yaw, bool)
                            or not isfinite(yaw)):
            why = "center_cam1_m / rotation.yaw_deg invalid"
        if why is None:
            pos = camera_to_robot(xyz, c2r) if c2r else xyz
            if c2r:
                why = work_reject_reason(pos, work)
                if why is not None:
                    work_dropped.append(f"instance {o.get('instance')}: {why}")
        if why is not None:
            dropped.append(f"instance {o.get('instance')}: {why}")
            continue
        cands.append({"xyz": xyz, "pos": pos, "yaw_deg": float(yaw), "instance": o.get("instance"),
                      "score": score_candidate(numbers(o.get("dims_mm"), 3), cube_dim_mm, weights),
                      "tracked": False, "dropped": work_dropped})
    if len(cands) == 0:
        return None, f"no cube candidate among {len(rows)} objects" + (": " + "; ".join(dropped) if dropped else "")
    if prev is not None:
        near = [(dist(c["pos"], prev), c) for c in cands]
        near = [t for t in near if t[0] <= track_tol_m]
        if near:
            best = min(near, key=lambda t: t[0])[1]
            best["tracked"] = True
            return best, None
    cands.sort(key=lambda c: -c["score"])
    if len(cands) > 1 and cands[0]["score"] - cands[1]["score"] < min_margin:
        a, b = cands[0], cands[1]
        return None, (f"ambiguous: {len(cands)} cube candidates, 1st instance {a['instance']} score {a['score']:.3f} at "
                      f"{tuple(round(v, 3) + 0.0 for v in a["pos"])}, 2nd instance {b['instance']} score {b['score']:.3f} at "
                      f"{tuple(round(v, 3) + 0.0 for v in b["pos"])}: margin {a['score'] - b['score']:.3f} < min_margin {min_margin:g}")
    return cands[0], None


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
