"""camera_result.py without ROS: reading inference.json and choosing the cube.

    python3 vision_node/tests/check_camera_result.py

Samples follow the real inference.json layout ({"objects": {"seg": [...]}}); only the keys the code
reads are kept. Cube and noise values are from the first real sample (cube dims_mm [60.3, 55.5, 2.8],
depth_rect_m 0.442, center_cam1_m [0.0715, 0.0906, 0.4457]; noise bbox [607,14,640,26], dims_mm
[93.2, 38.1, 7.1], depth_rect_m 0.789).
"""
import json
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from vision_node.camera_result import camera_to_robot, read_result, score_candidate, select_cube  # noqa: E402

failures = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f": {detail}" if detail != "" else ""))
    if not ok:
        failures.append(name)


DIM, DEPTH = (40.0, 80.0), (0.3, 1.0)


def cube(instance=1, **kw):
    o = {"instance": instance, "label": "blob", "status": "ok", "bbox_xyxy": [467, 262, 546, 330],
         "depth_rect_m": 0.442, "dims_mm": [60.3, 55.5, 2.8], "center_cam1_m": [0.0715, 0.0906, 0.4457],
         "rotation": {"yaw_deg": 6.05, "pitch_deg": 37.3, "roll_deg": -19.1, "tilt_deg": 37.6}, "warnings": []}
    o.update(kw)
    return o


NOISE = {"instance": 2, "label": "blob", "status": "ok", "bbox_xyxy": [607, 14, 640, 26],
         "depth_rect_m": 0.789, "dims_mm": [93.2, 38.1, 7.1], "center_cam1_m": [0.5, -0.2, 0.789],
         "rotation": {"yaw_deg": 80.0}, "warnings": []}


def doc(*objs, method="seg"):
    return json.dumps({"timestamp_ns": 1, "objects": {method: list(objs)}})


pick, why = select_cube(doc(cube(), NOISE), DIM, DEPTH)
check("cube + noise blob: exactly the cube", pick is not None and pick["instance"] == 1, (pick, why))
check("center_cam1_m and yaw taken from the cube",
      pick and pick["xyz"] == (0.0715, 0.0906, 0.4457) and pick["yaw_deg"] == 6.05, pick)
check("noise alone (dims 93.2 x 38.1, depth 0.789) is not a candidate",
      select_cube(doc(NOISE), DIM, DEPTH)[0] is None)

pick, why = select_cube(doc(NOISE), DIM, DEPTH)
check("0 candidates: nothing, reason says so", pick is None and "no cube candidate" in why, why)
pick, why = select_cube(doc(), DIM, DEPTH)
check("empty list (like my_view): nothing", pick is None and "no cube candidate" in why, why)
pick, why = select_cube(doc(cube(1), cube(3, center_cam1_m=[0.2, 0.1, 0.5])), DIM, DEPTH)
check("2 candidates: nothing, reason says ambiguous", pick is None and "ambiguous" in why, why)
check("method name is not fixed (other key than 'seg')", select_cube(doc(cube(), method="x"), DIM, DEPTH)[0] is not None)

for name, o in (("label not blob", cube(label="box")), ("status not ok", cube(status="bad")),
                ("width 39.9 below range", cube(dims_mm=[39.9, 55.5, 2.8])),
                ("height 80.1 above range", cube(dims_mm=[60.3, 80.1, 2.8])),
                ("depth 0.29 below range", cube(depth_rect_m=0.29)),
                ("depth 1.01 above range", cube(depth_rect_m=1.01))):
    check(f"excluded: {name}", select_cube(doc(o), DIM, DEPTH)[0] is None)
for name, o in (("width 40.0 (edge)", cube(dims_mm=[40.0, 55.5, 2.8])),
                ("height 80.0 (edge)", cube(dims_mm=[60.3, 80.0, 2.8])),
                ("depth 0.3 (edge)", cube(depth_rect_m=0.3)), ("depth 1.0 (edge)", cube(depth_rect_m=1.0)),
                ("thickness 2.8 is ignored", cube(dims_mm=[60.3, 55.5, 99.0]))):
    check(f"included: {name}", select_cube(doc(o), DIM, DEPTH)[0] is not None)

pick, why = select_cube(doc(cube(center_cam1_m=None)), DIM, DEPTH)
check("candidate without center: nothing, reason", pick is None and "center_cam1_m" in why, why)
pick, why = select_cube(doc(cube(rotation={"yaw_deg": float("nan")})), DIM, DEPTH)
check("candidate with NaN yaw: nothing", pick is None and "yaw_deg" in why, why)

good = doc(cube(), NOISE)
for name, raw in (("truncated JSON", good[:len(good) // 2]), ("empty file", ""), ("not JSON", "abc"),
                  ("bad bytes", b"\xff\xfe\x00{"), ("JSON list", "[1, 2]"), ("no objects", "{}"),
                  ("objects is a list", '{"objects": []}')):
    pick, why = select_cube(raw, DIM, DEPTH)
    check(f"{name}: skipped without exception", pick is None and isinstance(why, str), why)

with tempfile.TemporaryDirectory() as d:
    p = os.path.join(d, "inference.json")
    now = time.time()
    raw, why = read_result(p, now, 3.0)
    check("missing file: reason, no exception", raw is None and "cannot read" in why, why)
    with open(p, "w") as f:
        f.write(good)
    raw, why = read_result(p, time.time(), 3.0)
    check("fresh file is read", raw == good.encode() and why is None, why)
    os.utime(p, (now - 10, now - 10))
    raw, why = read_result(p, now, 3.0)
    check("10 s old file with max_age_s 3: not used (reason has no changing number)", raw is None and "stale" in why, why)
    os.utime(p, (now - 2.9, now - 2.9))
    check("2.9 s old file with max_age_s 3: used", read_result(p, now, 3.0)[0] is not None)


def near(a, b):
    return len(a) == len(b) and all(abs(x - y) < 1e-9 for x, y in zip(a, b))


# camera_to_robot: expected values calculated by hand (R = Rz(yaw)·Ry(pitch)·Rx(roll), then + t)
for name, p, par, want in (
        ("all zero: input unchanged", (0.1, 0.2, 0.3), (0, 0, 0, 0, 0, 0), (0.1, 0.2, 0.3)),
        ("translation only", (1, 2, 3), (10, 20, 30, 0, 0, 0), (11, 22, 33)),
        ("yaw 90, t=[1,0,0], p=[1,0,0]: Rz·p=[0,1,0], +t", (1, 0, 0), (1, 0, 0, 0, 0, 90), (1, 1, 0)),
        ("roll 90 about x: [0,1,0] -> [0,0,1]", (0, 1, 0), (0, 0, 0, 90, 0, 0), (0, 0, 1)),
        ("pitch 90 about y: [0,0,1] -> [1,0,0]", (0, 0, 1), (0, 0, 0, 0, 90, 0), (1, 0, 0)),
        ("yaw -90: [1,0,0] -> [0,-1,0]", (1, 0, 0), (0, 0, 0, 0, 0, -90), (0, -1, 0)),
        ("order: roll 90 then yaw 90 on [1,0,0] -> [0,1,0] (reverse order would give [0,0,1])",
         (1, 0, 0), (0, 0, 0, 90, 0, 90), (0, 1, 0)),
        ("all three 90, p=[1,2,3], t=[10,20,30]: Rx [1,-3,2], Ry [2,-3,-1], Rz [3,2,-1], +t",
         (1, 2, 3), (10, 20, 30, 90, 90, 90), (13, 22, 29)),
        ("yaw 30 on [1,0,0] -> [cos30, sin30, 0] = [0.8660254, 0.5, 0]",
         (1, 0, 0), (0, 0, 0, 0, 0, 30), (0.8660254037844386, 0.5, 0))):
    got = camera_to_robot(p, par)
    check(f"transform: {name}", near(got, want), tuple(round(v, 12) for v in got))

# cube_labels
pick, why = select_cube(doc(cube(label="kite")), DIM, DEPTH)
check("label kite passes with the default cube_labels", pick is not None and pick["instance"] == 1, why)
pick, why = select_cube(doc(cube(label="person")), DIM, DEPTH)
check("label person is rejected by default, reason names the label", pick is None and "label 'person'" in why, why)
pick, why = select_cube(doc(cube(label="person")), DIM, DEPTH, ["*"])
check('cube_labels ["*"]: person passes too', pick is not None, why)
check("cube_labels [\"kite\"] only: blob is rejected", select_cube(doc(cube()), DIM, DEPTH, ["kite"])[0] is None)
pick, why = select_cube(doc(cube(label="kite"), cube(2, label="blob")), DIM, DEPTH)
check("kite and blob both candidates: ambiguous", pick is None and "ambiguous" in why, why)

# dropped objects are named with the reason in the log text
_, why = select_cube(doc(NOISE, cube(3, label="person"), cube(4, status="bad"), cube(5, depth_rect_m=2.0),
                         cube(6, dims_mm=[30.0, 55.5, 2.8])), DIM, DEPTH)
check("reason: 5 objects counted", "no cube candidate among 5 objects" in why, why)
check("reason: dims of the noise blob (instance 2)", "instance 2: dims_mm 93.2x38.1 outside cube_dim_mm [40.0, 80.0]" in why, why)
check("reason: label of instance 3", "instance 3: label 'person' not in cube_labels ['blob', 'kite']" in why, why)
check("reason: status of instance 4", "instance 4: status 'bad' is not 'ok'" in why, why)
check("reason: depth of instance 5", "instance 5: depth_rect_m 2 outside depth_range_m [0.3, 1.0]" in why, why)
check("reason: dims of instance 6", "instance 6: dims_mm 30x55.5 outside" in why, why)

# ---- work zone, score, margin, previous choice -------------------------------------------------
# score by hand, cube_dim_mm [40, 80] (c = 60, half = 20), equal weights:
#   50 x 40: squareness 1 - 10/50 = 0.8, centrality 1 - mean(10, 20)/20 = 0.25 -> (0.8 + 0.25)/2 = 0.525
#   60 x 60: 1, 1 -> 1.0;  40 x 40: squareness 1, centrality 1 - 20/20 = 0 -> 0.5
#   weights (3, 1) on 50 x 40: (3*0.8 + 0.25)/4 = 0.6625
check("score 50x40 by hand = 0.525", abs(score_candidate((50, 40, 3), DIM) - 0.525) < 1e-12, score_candidate((50, 40, 3), DIM))
check("score 60x60 = 1.0", score_candidate((60, 60, 3), DIM) == 1.0)
check("score 40x40 = 0.5", abs(score_candidate((40, 40, 3), DIM) - 0.5) < 1e-12)
check("score 50x40 with weights (3, 1) = 0.6625", abs(score_candidate((50, 40, 3), DIM, (3, 1)) - 0.6625) < 1e-12)

C2R = (0, 0, 0, 0, 0, 90)        # Rz(90): camera (a, b, c) -> robot (-b, a, c)
WORK = ((-0.2, 0.2), (-0.3, 0.3), (0.0, 0.6))
at = lambda instance, c, **kw: cube(instance, center_cam1_m=list(c), **kw)    # noqa: E731


def sel(*objs, **kw):
    kw.setdefault("c2r", C2R)
    kw.setdefault("work", WORK)
    return select_cube(doc(*objs), DIM, DEPTH, **kw)


# (a) one cube
pick, why = sel(at(1, (0.0715, 0.0906, 0.4457)))
check("(a) one cube: published, position in robot frame by hand (-0.0906, 0.0715, 0.4457)",
      pick is not None and all(abs(u - v) < 1e-9 for u, v in zip(pick["pos"], (-0.0906, 0.0715, 0.4457))), (pick, why))
check("(a) one cube: xyz stays the camera value, no margin check, nothing dropped",
      pick["xyz"] == (0.0715, 0.0906, 0.4457) and pick["dropped"] == [] and not pick["tracked"])
# (b) cube + object outside the zone: robot x = -b; b = 0.5 -> x -0.5 (outside [-0.2, 0.2])
pick, why = sel(at(1, (0.0715, 0.0906, 0.4457)), at(2, (0.1, 0.5, 0.4457)))
check("(b) object outside the zone is dropped, the cube is chosen", pick is not None and pick["instance"] == 1, (pick, why))
check("(b) drop reason is reported", pick and pick["dropped"] == ["instance 2: robot x -0.500 outside work_x_m [-0.2, 0.2]"], pick and pick["dropped"])
pick, why = sel(at(2, (0.1, 0.5, 0.4457)))
check("(b) only an outside object: nothing, reason names x and the zone",
      pick is None and "robot x -0.500 outside work_x_m [-0.2, 0.2]" in why, why)
pick, why = sel(at(1, (0.0715, 0.0906, 0.4457)), work=(None, (0.0, 0.05), None))
check("work_y_m [0, 0.05]: robot y = 0.0715 outside -> dropped (only y active)", pick is None and "work_y_m [0.0, 0.05]" in why, why)
pick, why = sel(at(1, (0.0715, 0.0906, 0.9)), work=(None, None, (0.0, 0.6)))
check("work_z_m [0, 0.6]: z 0.9 outside -> dropped", pick is None and "robot z 0.900 outside work_z_m" in why, why)
pick, why = sel(at(1, (0.0, 0.2, 0.6)))
check("zone edges are inclusive (x -0.2, z 0.6)", pick is not None, why)
# (e) camera_to_robot empty: the zone is not applied, position stays camera
pick, why = sel(at(1, (5.0, 5.0, 0.5)), c2r=None)
check("(e) no camera_to_robot: zone not applied, pos = camera xyz", pick is not None and pick["pos"] == (5.0, 5.0, 0.5), (pick, why))

# (c) cube + a second object of similar size: margin stops it
sq, sq2 = (60.3, 55.5, 2.8), (58.0, 57.0, 2.8)        # by hand: 60.3x55.5 -> (0.920398 + 0.88)/2 = 0.9002; 58x57 -> (0.982759 + 0.875)/2 = 0.9293; margin 0.0291
pick, why = sel(at(1, (0.0715, 0.0906, 0.4457), dims_mm=list(sq)), at(2, (0.0, 0.15, 0.4457), dims_mm=list(sq2)))
check("(c) similar second object: nothing published, ambiguous with both scores and positions",
      pick is None and why.startswith("ambiguous:") and "1st instance 2 score 0.929" in why
      and "2nd instance 1 score 0.900" in why and "(-0.15, 0.0, 0.446)" in why, why)
pick, why = sel(at(1, (0.0715, 0.0906, 0.4457), dims_mm=list(sq)), at(2, (0.0, 0.15, 0.4457), dims_mm=list(sq2)),
                min_margin=0.02)
check("(c) margin 0.0291 >= 0.02: the better one is chosen", pick is not None and pick["instance"] == 2, why)
# clear winner: 60x60 (1.0) vs 50x40 (0.525): margin 0.475 < 0.5 -> ambiguous at 0.5, chosen at 0.4
pick, why = sel(at(1, (0.0, 0.1, 0.45), dims_mm=[60, 60, 3]), at(2, (0.0, -0.1, 0.45), dims_mm=[50, 40, 3]), min_margin=0.4)
check("clear winner (1.0 vs 0.525, min_margin 0.4): chosen", pick is not None and pick["instance"] == 1, why)
pick, why = sel(at(1, (0.0, 0.1, 0.45), dims_mm=[60, 60, 3]), at(2, (0.0, -0.1, 0.45), dims_mm=[50, 40, 3]), min_margin=0.5)
check("clear winner but min_margin 0.5 > 0.475: ambiguous", pick is None and "ambiguous" in why, why)

# (d) two candidates take turns being first; the previous choice keeps the pick steady
A, B = (0.0, 0.10, 0.45), (0.0, -0.10, 0.45)         # camera -> robot (-0.10, 0.0) and (0.10, 0.0): 0.2 m apart
seq = [(([60, 60, 3]), ([50, 40, 3])),                # A 1.0, B 0.525: A clear
       (([57, 59, 3]), ([58, 58, 3])),                # A ~0.97, B ~0.97: margin tiny
       (([50, 45, 3]), ([60, 59, 3])),                # B clearly better now
       (([60, 60, 3]), ([60, 59, 3]))]                # tiny margin again
prev, picked = None, []
for da, db in seq:
    pick, why = sel(at(1, A, dims_mm=list(da)), at(2, B, dims_mm=list(db)), prev=prev, min_margin=0.05)
    picked.append(pick["instance"] if pick else None)
    if pick:
        prev = pick["pos"]
check("(d) without prev frame 2 would be ambiguous", sel(at(1, A, dims_mm=[57, 59, 3]), at(2, B, dims_mm=[58, 58, 3]))[0] is None)
check("(d) without prev frame 3 would flip to instance 2", sel(at(1, A, dims_mm=[50, 45, 3]), at(2, B, dims_mm=[60, 59, 3]))[0]["instance"] == 2)
check("(d) with prev: 1, 1, 1, 1 (no jumping)", picked == [1, 1, 1, 1], picked)
pick, why = sel(at(1, (0.0, 0.1005, 0.45), dims_mm=[50, 45, 3]), at(2, B, dims_mm=[60, 59, 3]), prev=(-0.10, 0.0, 0.45))
check("(d) tracked flag set, position moved 0.5 mm still follows", pick["instance"] == 1 and pick["tracked"], pick)
pick, why = sel(at(1, A, dims_mm=[50, 45, 3]), at(2, B, dims_mm=[60, 59, 3]), prev=(-0.10, 0.05, 0.45), track_tol_m=0.03)
check("(d) prev 0.05 m away (> track_tol_m 0.03): not followed, best score wins", pick["instance"] == 2 and not pick["tracked"], pick)
pick, why = sel(at(1, A, dims_mm=[50, 45, 3]), at(2, B, dims_mm=[60, 59, 3]), prev=(-0.10, 0.05, 0.45), track_tol_m=0.06)
check("(d) same prev with track_tol_m 0.06: followed", pick["instance"] == 1 and pick["tracked"], pick)
pick, why = sel(at(2, B, dims_mm=[60, 59, 3]), prev=(-0.10, 0.0, 0.45))
check("(d) the previous object is gone: the remaining one is chosen", pick is not None and pick["instance"] == 2)
pick, why = sel(at(1, (0.0, 0.1, 0.45)), at(2, (0.0, 0.1, 0.45)), prev=None)
check("two candidates with identical score: ambiguous (default min_margin 0.05)", pick is None and "ambiguous" in why, why)


# ---- fixed mount (config/camera.yaml): phone upside down, 45 deg down, lens 17.4 cm in front, 14.6 cm right of the shoulder roll axis,
# tz -0.0715 chosen so that a real cube reading (robot z -0.365) lands on the table height -0.4525
MOUNT = (-0.146, -0.174, -0.0715, 135.129, 0.988, 0.992)
for name, p, want in (("cam (0,0,0.45) on the optical axis", (0, 0, 0.45), (-0.146, -0.4915, -0.3904)),
                      ("cam (0.1,0,0.45)", (0.1, 0, 0.45), (-0.046, -0.4898, -0.3921)),
                      ("cam (0,0.1,0.45)", (0, 0.1, 0.45), (-0.1436, -0.5624, -0.3198))):
    got = camera_to_robot(p, MOUNT)
    check(f"mount: {name} (given by the user, within 1e-3)", all(abs(a - b) < 1e-3 for a, b in zip(got, want)),
          tuple(round(v, 5) for v in got))
pick, why = select_cube(doc(cube(center_cam1_m=[0.0045, -0.0399, 0.4981], depth_rect_m=0.4981)), DIM, DEPTH, c2r=MOUNT)
check("mount: real cube reading cam (0.0045, -0.0399, 0.4981) passes depth_range_m, robot z -0.4527 (within 1e-3)",
      pick is not None and abs(pick["pos"][2] + 0.4527) < 1e-3, (pick, why))

print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
sys.exit(1 if failures else 0)
