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
from vision_node.camera_result import camera_to_robot, read_result, select_cube  # noqa: E402

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

print(f"\n{'all passed' if not failures else f'{len(failures)} failed: ' + ', '.join(failures)}")
sys.exit(1 if failures else 0)
