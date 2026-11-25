"""휴대폰 카메라 탭의 '세그먼트(폰 뎁스)' 모드: 폰 스테레오 프로그램의 inference.json → 큐브 자리 (x, y, yaw). ROS 없음.

폰 스테레오 프로그램 (team-jax/SteroDepthSeg, z_phone_stereo.py; Galaxy S25 두 렌즈 → FFS 깊이 → YOLO seg + lib_blobs) 이
<out>/inference.json 과 inference.png 를 1초쯤마다 덮어쓴다. 이 파일은 그 json 만 읽는다 (같은 모양의 json 을 쓰는 다른 프로그램도 된다):

    {"objects": {"seg": [ {"instance": 1, "label": "blob"|"kite"…, "status": "ok", "dims_mm": [w, h, 두께], "depth_rect_m": 0.44,
                           "center_cam1_m": [x, y, z], "rotation": {"yaw_deg", "pitch_deg", "roll_deg", "normal_cam1"…},
                           "warnings": [...]}, ... ]}, ...}

center_cam1_m = 카메라 좌표 (OpenCV: x 오른쪽, y 아래, z 앞, m). 고르기·좌표 변환은 team-jax/jax_ros2 vision-select 의
vision_node/vision_node/camera_result.py (select_cube, camera_to_robot) 와 tools/solve_camera_to_robot.py (solve) 를 그대로 옮겼다
— 뜻·숫자가 같아야 하므로 고칠 때 그쪽과 맞출 것. 기본 거치 자세 = vision_node/config/camera.yaml (CAD 임시값, 실물 보정 전).

로봇 좌표 = pick_plan 과 같다: 원점 = 어깨 roll 축, +x = 로봇 왼쪽, +y = 뒤 (책상은 −y), +z = 위, 책상 면 z = −d.
"""
import json
import os
from math import atan2, cos, degrees, dist, hypot, isfinite, radians, sin, sqrt, tan

DEFAULT_C2R = [0.174, -0.1177, -0.012, 135.129, 0.988, 0.992]   # camera.yaml: [tx, ty, tz (m), roll, pitch, yaw (deg)] — 미검증 CAD 값
# 정우 CAMERA.md (JaxIsaacLab deploy_2026-11-05, CAD 2026-11-12) 와 맞춰 봄 (phone_table.py, test_phone_table.py): 위치 3개는 같은 점
# (프로파일 중심선 앞 117.7 mm, 어깨 피치축 − 7 mm = roll 축 − 12 mm), 회전은 CAMERA.md 의 '앞을 보고 45° 아래' × 광축 둘레 180°
# (폰이 뒤집혀 끼워짐) 에서 1.4° 다르다 (IMU 로 잰 값이라). 책상 높이 재기는 phone_table 의 CAD 값을 쓴다.
DEFAULT_LABELS = ("blob", "kite")     # blob = 책상 평면 위로 튀어나온 덩어리 (lib_blobs), kite = COCO(YOLO)가 큐브를 부른 이름
def default_repo_dir():
    """폰 스테레오 저장소 기본 위치: JAX_DEPTH_REPO, 아니면 흔한 자리 중 z_phone_stereo.py 가 있는 첫 곳 (철자 Stero/Stereo 둘 다),
    아무 데도 없으면 팀 README 의 ~/Documents/SteroDepthSeg."""
    if os.environ.get("JAX_DEPTH_REPO"):
        return os.environ["JAX_DEPTH_REPO"]
    cands = [os.path.expanduser(f"~/{d}/{n}") for d in ("Documents", "Desktop", "") for n in ("SteroDepthSeg", "StereoDepthSeg")]
    for c in cands:
        if os.path.isfile(os.path.join(c, "z_phone_stereo.py")):
            return c
    return cands[0]


DEFAULT_CFG = {
    "repo_dir": default_repo_dir(),
    "run_dir": "",                    # 빈 값 = <repo_dir>/phone_stereo/runs/dash
    "camera_to_robot": list(DEFAULT_C2R),
    "max_age_s": 3.0,                 # 이보다 오래된 inference.json 은 안 쓴다 (팔이 옛 자리로 가면 안 된다)
    "cube_dim_mm": [40.0, 80.0],      # dims_mm[0], dims_mm[1] 둘 다 이 안 (5 cm 큐브가 60~68 로 재졌다)
    "depth_range_m": [0.3, 1.0],
    "cube_labels": ["*"],             # 이름 상관없이 크기·높이로 (2026-11-25: YOLO 가 큐브를 'cell phone' 이라 불러 뎁스가 큐브를 버렸다 — 예전 blob·kite 만)
    "work_x_m": [-0.40, 0.40], "work_y_m": [-0.70, -0.10], "work_z_m": None,   # [min, max] 로봇 좌표, 밖의 후보는 버림 (2026-11-25: 책상 밖
                                      # x 0.66 y −0.93 의 사람·손을 큐브로 집으려 했다 → 팔이 닿는 쪽 책상만. 비우면 끔)
    "yaw_manual_deg": None,           # 숫자면 추정 대신 이 각을 쓴다
    "pos_fix_m": [-0.02, 0.04],       # 카메라로 찾은 큐브 자리 고침 [x, y] m (거치 자리 tx·ty 에 더함 = ArUco·뎁스 둘 다). 2026-11-25 사용자:
                                      # 카메라로 집으면 약 3 cm 앞(−y)으로 감 → y +0.03, 그 뒤 '왼쪽으로 2 cm, 앞으로 1 cm 더' → x −0.02 (+x = 로봇 왼쪽),
                                      # y +0.04. 중력·마커로는 못 재는 자리·yaw 오차 (마지막 카메라 보정)
    "wifi_host": "",                  # 2026-11-25 사용자 (USB 불안정): "" = USB(adb) · "auto" = 핫스팟 (PC 기본 게이트웨이 = 폰) · IP = Tailscale 등
    "send_fps": 1.0,                  # Wi-Fi 일 때 폰이 1 초에 보내는 쌍 수 (USB 는 전부)
    "text_rot180": True,              # 결과 그림 글자를 180° 돌려 쓰게 (--text-rot180): 대시보드가 그림을 뒤집어 보여 줄 때 바로 읽힘
    "save_rgbd": False,               # 폰 프로그램을 --rgbd-viewer 로 켠다 → rgbd/rgbd.npz (5초마다) — 책상 높이 재기(phone_table)가 읽는다
}
CUBE_SIDE = 0.05
Z_TOL = (-0.02, 0.03)   # 큐브 중심 높이가 책상 위 0 ~ 큐브 한 변 (보이는 면의 가운데라 중심~윗면) 에서 이만큼 벗어나면 경고


# ---- camera_result.py (vision_node) 에서 옮긴 것 ---------------------------------------------------------------------

def numbers(v, n):
    """v 를 유한한 float n 개 튜플로, 아니면 None."""
    if (isinstance(v, (list, tuple)) and len(v) == n
            and all(isinstance(a, (int, float)) and not isinstance(a, bool) and isfinite(a) for a in v)):
        return tuple(float(a) for a in v)
    return None


def read_result(path, now, max_age_s):
    """(bytes, None, 나이 s) 또는 (None, 이유, 나이 s 또는 None) — 없거나 못 읽거나 max_age_s 보다 오래됨 (파일 수정 시각 기준)."""
    try:
        with open(path, "rb") as f:
            raw = f.read()
            mtime = os.fstat(f.fileno()).st_mtime
    except OSError as e:
        return None, f"결과 파일을 못 읽음: {path} ({e.strerror or e})", None
    age = now - mtime
    if age > max_age_s:
        return None, f"결과가 오래됨 ({_age(age)} 전) — 폰 스테레오 프로그램이 꺼졌거나 멈춤", age
    return raw, None, age


def _age(s):
    return f"{s:.0f}초" if s < 120 else f"{s / 60:.0f}분" if s < 7200 else f"{s / 3600:.0f}시간"


def reject_reason(o, cube_labels, cube_dim_mm, depth_range_m):
    """None = 후보, 아니면 왜 아닌지 (label, status, dims, depth 순서로 처음 걸린 것)."""
    label = o.get("label")
    if "*" not in cube_labels and label not in cube_labels:
        return f"label {label!r} 은 큐브 이름 {list(cube_labels)} 이 아님"
    if o.get("status") != "ok":
        return f"status {o.get('status')!r}"
    dims = numbers(o.get("dims_mm"), 3)
    if dims is None:
        return f"크기 없음 ({o.get('size_status') or o.get('dims_mm')!r})"
    if not (cube_dim_mm[0] <= dims[0] <= cube_dim_mm[1] and cube_dim_mm[0] <= dims[1] <= cube_dim_mm[1]):
        return f"크기 {dims[0]:g}x{dims[1]:g} mm 가 {list(cube_dim_mm)} 밖"
    depth = o.get("depth_rect_m")
    if not isinstance(depth, (int, float)) or isinstance(depth, bool) or not isfinite(depth):
        return f"거리 값 없음 ({depth!r})"
    if not depth_range_m[0] <= depth <= depth_range_m[1]:
        return f"거리 {depth:g} m 가 {list(depth_range_m)} 밖"
    return None


def score_candidate(dims, cube_dim_mm, weights=(1.0, 1.0)):
    """[0, 1] 점수: (정사각에 가까움 + 크기 범위 한가운데에 가까움) / 2. 손 계산 예: [40, 80] 에서 50x40 → 0.525."""
    w, h = dims[0], dims[1]
    square = 1.0 - abs(w - h) / max(w, h)
    c = (cube_dim_mm[0] + cube_dim_mm[1]) / 2.0
    half = (cube_dim_mm[1] - cube_dim_mm[0]) / 2.0
    central = 1.0 - ((abs(w - c) + abs(h - c)) / 2.0) / half if half > 0 else 1.0
    return (weights[0] * square + weights[1] * max(0.0, central)) / (weights[0] + weights[1])


def work_reject_reason(p, work):
    for name, v, r in zip("xyz", p, work):
        if r is not None and not r[0] <= v <= r[1]:
            return f"로봇 {name} {v:.3f} 가 작업 구역 {[r[0], r[1]]} 밖"
    return None


def camera_to_robot(p_cam, params):
    """p_robot = R · p_cam + t. params = (tx, ty, tz, roll, pitch, yaw °), R = Rz(yaw) · Ry(pitch) · Rx(roll) (ROS rpy 와 같음)."""
    tx, ty, tz, roll, pitch, yaw = params
    cr, sr = cos(radians(roll)), sin(radians(roll))
    cp, sp = cos(radians(pitch)), sin(radians(pitch))
    cy, sy = cos(radians(yaw)), sin(radians(yaw))
    x, y, z = p_cam
    y, z = cr * y - sr * z, sr * y + cr * z          # Rx(roll)
    x, z = cp * x + sp * z, -sp * x + cp * z         # Ry(pitch)
    x, y = cy * x - sy * y, sy * x + cy * y          # Rz(yaw)
    return (x + tx, y + ty, z + tz)


def select_cube(raw, cube_dim_mm, depth_range_m, cube_labels=DEFAULT_LABELS, c2r=None, work=(None, None, None),
                weights=(1.0, 1.0), min_margin=0.05, prev=None, track_tol_m=0.03, prefer=None, prefer_tol_m=0.10):
    """inference.json 내용 raw 에서 큐브 하나를 고른다 → (dict, None) 또는 (None, 이유). vision_node 의 select_cube 와 같은 규칙:
    후보 = 이름·status ok·크기·거리·center·yaw 가 맞고 (c2r 이 있으면) 작업 구역 안. 직전에 고른 자리(prev) 3 cm 안의 후보가 있으면 그것.
    아니면 점수 1등 — 2등과 min_margin 이상 차이 날 때만 (아니면 '헷갈림'). dict 에는 원래 객체(obj)도 넣는다."""
    try:
        data = json.loads(raw)
    except ValueError as e:
        return None, f"JSON 이 아님 (쓰는 중일 수 있음): {e}"
    objects = data.get("objects") if isinstance(data, dict) else None
    if not isinstance(objects, dict):
        return None, "파일에 'objects' 가 없음"
    rows = [o for lst in objects.values() if isinstance(lst, list) for o in lst if isinstance(o, dict)]
    cands, dropped = [], []
    for o in rows:
        why = reject_reason(o, cube_labels, cube_dim_mm, depth_range_m)
        xyz = numbers(o.get("center_cam1_m"), 3)
        rot = o.get("rotation")
        yaw = rot.get("yaw_deg") if isinstance(rot, dict) else None
        if why is None and (xyz is None or not isinstance(yaw, (int, float)) or isinstance(yaw, bool) or not isfinite(yaw)):
            why = "center_cam1_m / rotation.yaw_deg 없음"
        if why is None:
            pos = camera_to_robot(xyz, c2r) if c2r else xyz
            if c2r:
                why = work_reject_reason(pos, work)
        if why is not None:
            dropped.append(f"{o.get('instance')}번: {why}")
            continue
        cands.append({"xyz": xyz, "pos": pos, "yaw_deg": float(yaw), "instance": o.get("instance"), "obj": o,
                      "score": score_candidate(numbers(o.get("dims_mm"), 3), cube_dim_mm, weights), "tracked": False})
    if not cands:
        return None, f"큐브 후보 없음 (객체 {len(rows)}개)" + (": " + "; ".join(dropped) if dropped else "")
    if prev is not None:
        near = [(dist(c["pos"], prev), c) for c in cands]
        near = [t for t in near if t[0] <= track_tol_m]
        if near:
            best = min(near, key=lambda t: t[0])[1]
            best["tracked"] = True
            return best, None
    if prefer is not None and len(cands) > 1:
        # 큐브가 여럿이면 ArUco 가 본 그 큐브 (x, y) 에 가장 가까운 후보 (2026-11-25: 큐브 2개면 점수가 비슷해 '헷갈림' 으로 뎁스를 버렸다).
        # ArUco 자리는 책상 높이 d 가 틀리면 몇 cm 어긋나므로 넉넉히 prefer_tol_m 안 — 그 차이는 phone_fuse 가 'ArUco·뎁스 차' 로 따로 본다
        near = sorted((hypot(c["pos"][0] - prefer[0], c["pos"][1] - prefer[1]), i, c) for i, c in enumerate(cands))
        if near[0][0] <= prefer_tol_m and (len(near) == 1 or near[1][0] - near[0][0] >= 0.03):
            near[0][2]["matched_aruco_m"] = round(near[0][0], 3)
            return near[0][2], None
    cands.sort(key=lambda c: -c["score"])
    if len(cands) > 1 and cands[0]["score"] - cands[1]["score"] < min_margin:
        a, b = cands[0], cands[1]
        return None, (f"헷갈림: 큐브 후보 {len(cands)}개 — {a['instance']}번 (점수 {a['score']:.2f}, x {a['pos'][0]:.3f} y {a['pos'][1]:.3f}) / "
                      f"{b['instance']}번 (점수 {b['score']:.2f}, x {b['pos'][0]:.3f} y {b['pos'][1]:.3f}). 큐브만 보이게 치우기")
    return cands[0], None


# ---- solve_camera_to_robot.py (vision_node/tools) 에서 옮긴 것 ----------------------------------------------------------

def solve_mount(points, z):
    """points = [((cx, cy, cz), (rx, ry)), …] (카메라 좌표 center_cam1_m, 그때 큐브 중심의 로봇 x·y), z = 그 점들의 로봇 z
    → ([tx, ty, tz, roll, pitch, yaw], 점마다 잔차 mm). 강체 변환 (Kabsch, det +1). 3점 이상, 한 줄 위면 ValueError.
    주의: 점이 모두 한 높이(평면)라 잔차 0 이 축이 맞다는 증거는 아니다 — 부호를 잘못 넣어도 딱 맞는다. 새 자리의 큐브로 확인할 것."""
    import numpy as np
    if len(points) < 3:
        raise ValueError(f"점이 3개 이상 필요 ({len(points)}개)")
    cam = np.array([p[0] for p in points], dtype=float)
    rob = np.array([[p[1][0], p[1][1], z] for p in points], dtype=float)
    for a in (cam, rob):
        s = np.linalg.svd(a - a.mean(axis=0), compute_uv=False)
        if s[0] == 0 or s[1] / s[0] < 1e-3:
            raise ValueError("점들이 한 줄 위에 있다 — 회전이 안 정해진다. 일렬이 아닌 자리에 놓기")
    cc, rc = cam.mean(axis=0), rob.mean(axis=0)
    u, _, vt = np.linalg.svd((cam - cc).T @ (rob - rc))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d if d != 0 else 1.0]) @ u.T
    t = rc - r @ cc
    res = [1000.0 * float(np.linalg.norm(r @ c + t - p)) for c, p in zip(cam, rob)]
    sp = float(np.clip(-r[2, 0], -1.0, 1.0))
    pitch = np.arcsin(sp)
    if abs(sp) > 1 - 1e-9:
        roll, yaw = 0.0, atan2(-r[0, 1], r[1, 1])
    else:
        roll, yaw = atan2(r[2, 1], r[2, 2]), atan2(r[1, 0], r[0, 0])
    return [float(v) for v in t] + [degrees(roll), float(np.degrees(pitch)), degrees(yaw)], res


# ---- 여기부터 대시보드 것 ----------------------------------------------------------------------------------------------

def _fold(deg):
    return (deg + 45.0) % 90.0 - 45.0     # 큐브는 90° 마다 같다 → [−45, 45)


def table_yaw(rotation, c2r):
    """폰 프로그램의 rotation (카메라 기준, 보이는 면 하나의 법선·면 안 회전) → 책상 위에서 본 큐브 각 (로봇 좌표, [−45, 45)°).
    → (yaw 또는 None, 어느 면으로 구했나 'side'|'top'|'mixed'|None).

    폰 프로그램의 yaw_deg 는 '면이 카메라에 대해 좌우로 돌아선 각'이지 책상 위 각이 아니라 그대로 못 쓴다. 대신 법선을 로봇 좌표로 돌려
      옆면 (법선이 거의 수평) → 법선의 수평 방향이 곧 큐브 각.
      윗면 (법선이 거의 위)   → 면 안의 긴 변 방향 (roll_deg) 을 로봇 좌표로 돌린 방향.
      그 사이 (두 면이 섞여 평면 하나로 맞춰짐) → 옆면 식으로 구하되 'mixed' (믿기 어려움)."""
    if not isinstance(rotation, dict):
        return None, None
    n = numbers(rotation.get("normal_cam1"), 3)
    if n is None:      # 법선이 없으면 yaw·pitch 로 되만든다: yaw = atan2(nx, −nz), pitch = atan2(−ny, −nz)
        y, p = rotation.get("yaw_deg"), rotation.get("pitch_deg")
        if numbers([y, p], 2) is None or abs(y) > 89 or abs(p) > 89:
            return None, None
        n = (tan(radians(y)), -tan(radians(p)), -1.0)
    k = sqrt(sum(v * v for v in n))
    if k < 1e-9:
        return None, None
    n = tuple(v / k for v in n)
    rot = (0.0, 0.0, 0.0) + tuple(c2r[3:])
    nr = camera_to_robot(n, rot)
    if abs(nr[2]) > 0.85:                 # 윗면
        a = rotation.get("roll_deg")
        if numbers([a], 1) is None:
            return None, "top"
        hx = (-n[2], 0.0, n[0])           # n × (0, 1, 0): 면 안의 수평 (lib_stereo.plane_orientation 과 같은 기저)
        kh = hypot(hx[0], hx[2])
        if kh < 0.1:
            return None, "top"
        hx = tuple(v / kh for v in hx)
        vx = (hx[1] * n[2] - hx[2] * n[1], hx[2] * n[0] - hx[0] * n[2], hx[0] * n[1] - hx[1] * n[0])   # hx × n
        e = tuple(cos(radians(a)) * h + sin(radians(a)) * v for h, v in zip(hx, vx))
        er = camera_to_robot(e, rot)
        return _fold(degrees(atan2(er[1], er[0]))), "top"
    if hypot(nr[0], nr[1]) < 1e-6:
        return None, None
    return _fold(degrees(atan2(nr[1], nr[0]))), ("side" if abs(nr[2]) < 0.5 else "mixed")


def load_cfg(path):
    cfg = json.loads(json.dumps(DEFAULT_CFG))
    try:
        cfg.update(clean_cfg(json.loads(open(path).read())))
    except (OSError, ValueError, TypeError):
        pass
    return cfg


def clean_cfg(d):
    """받은 dict 에서 아는 키만, 값이 맞는 것만 (틀리면 ValueError)."""
    out = {}
    for k in ("repo_dir", "run_dir"):
        if k in d:
            if not isinstance(d[k], str) or len(d[k]) > 500 or "\0" in d[k]:
                raise ValueError(f"{k}: 폴더 경로 글자")
            out[k] = os.path.expanduser(d[k].strip())      # '~/…' 그대로면 cwd·exists 가 못 찾는다
    if out.get("repo_dir") == "":
        out["repo_dir"] = default_repo_dir()               # 비우면 자동으로 찾은 곳
    if "camera_to_robot" in d:
        v = numbers(d["camera_to_robot"], 6)
        if v is None or max(abs(a) for a in v[:3]) > 3 or max(abs(a) for a in v[3:]) > 360:
            raise ValueError("camera_to_robot: 숫자 6개 [tx, ty, tz (m, ±3), roll, pitch, yaw (°)]")
        out["camera_to_robot"] = list(v)
    if "max_age_s" in d:
        v = numbers([d["max_age_s"]], 1)
        if v is None or not 0.5 <= v[0] <= 30:
            raise ValueError("max_age_s: 0.5 ~ 30 초")
        out["max_age_s"] = v[0]
    for k in ("cube_dim_mm", "depth_range_m"):
        if k in d:
            v = numbers(d[k], 2)
            if v is None or not 0 < v[0] <= v[1]:
                raise ValueError(f"{k}: [작은 값, 큰 값] (0 보다 큼)")
            out[k] = list(v)
    if "cube_labels" in d:
        v = d["cube_labels"]
        if not isinstance(v, list) or not v or not all(isinstance(s, str) and s for s in v):
            raise ValueError("cube_labels: 이름 목록 (['*'] = 전부)")
        out["cube_labels"] = v
    for k in ("work_x_m", "work_y_m", "work_z_m"):
        if k in d:
            v = None if d[k] in (None, []) else numbers(d[k], 2)
            if d[k] not in (None, []) and (v is None or v[0] > v[1]):
                raise ValueError(f"{k}: 비우거나 [min, max]")
            out[k] = list(v) if v else None
    if "pos_fix_m" in d:
        v = numbers(d["pos_fix_m"], 2)
        if v is None or max(abs(a) for a in v) > 0.10:
            raise ValueError("pos_fix_m: [x, y] m, 각각 ±0.10 안")
        out["pos_fix_m"] = list(v)
    if "wifi_host" in d:
        v = d["wifi_host"]
        if not isinstance(v, str) or len(v) > 100 or not all(ch.isalnum() or ch in ".-:_" for ch in v.strip()):
            raise ValueError("wifi_host: 비우기(USB) · auto(핫스팟) · 폰 IP/이름")
        out["wifi_host"] = v.strip()
    if "send_fps" in d:
        v = numbers([d["send_fps"]], 1)
        if v is None or not 0.2 <= v[0] <= 15:
            raise ValueError("send_fps: 0.2 ~ 15")
        out["send_fps"] = v[0]
    if "text_rot180" in d:
        if not isinstance(d["text_rot180"], bool):
            raise ValueError("text_rot180: true / false")
        out["text_rot180"] = d["text_rot180"]
    if "save_rgbd" in d:
        if not isinstance(d["save_rgbd"], bool):
            raise ValueError("save_rgbd: true / false")
        out["save_rgbd"] = d["save_rgbd"]
    if "yaw_manual_deg" in d:
        v = d["yaw_manual_deg"]
        if v in (None, ""):
            out["yaw_manual_deg"] = None
        else:
            v = numbers([v], 1)
            if v is None or abs(v[0]) > 180:
                raise ValueError("yaw_manual_deg: 비우거나 ±180°")
            out["yaw_manual_deg"] = v[0]
    return out


def run_dir(cfg):
    return cfg["run_dir"] or os.path.join(cfg["repo_dir"], "phone_stereo", "runs", "dash")


def read_frame(cfg, now, max_age_s=None):
    """폰 프로그램이 마지막으로 저장한 화면 (<결과 폴더>/inference.png, 1초쯤마다) → (BGR 배열, 나이 s, 수정 시각) 또는 (None, 나이 | None, None).
    ArUco (phone_detect) 에 넣을 프레임 한 장이다. max_age_s (기본 = cfg max_age_s) 보다 오래됐거나 쓰는 중이라 못 읽으면 None.
    주의: 이 그림에는 폰 프로그램이 객체 테두리·글자를 덧그려서 마커가 가려질 수 있다 — 영상 스트림을 직접 받게 되면 이 함수만 바꾼다."""
    path = os.path.join(run_dir(cfg), "inference.png")
    try:
        mtime = os.stat(path).st_mtime
    except OSError:
        return None, None, None
    age = now - mtime
    if age > (cfg["max_age_s"] if max_age_s is None else max_age_s):
        return None, age, None
    import cv2
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    return (img, age, mtime) if img is not None else (None, age, None)


def evaluate(cfg, table_d, now, prev=None, prefer=None):
    """지금 inference.json → 화면에 줄 dict: ok, reason / cube{x, y, yaw_deg}, z, cam, dims_mm, yaw_from, warn[], age_s."""
    path = os.path.join(run_dir(cfg), "inference.json")
    out = {"ok": False, "warn": [], "file": path, "age_s": None, "usable": True}
    raw, why, age = read_result(path, now, cfg["max_age_s"])
    out["age_s"] = None if age is None else round(age, 1)
    if raw is None:
        return {**out, "reason": why, "stale": age is not None}
    c2r = tuple(cfg["camera_to_robot"])
    work = tuple(tuple(cfg[k]) if cfg[k] else None for k in ("work_x_m", "work_y_m", "work_z_m"))
    pick, why = select_cube(raw, tuple(cfg["cube_dim_mm"]), tuple(cfg["depth_range_m"]), tuple(cfg["cube_labels"]), c2r, work, prev=prev, prefer=prefer)
    if pick is None:
        return {**out, "reason": why}
    o = pick["obj"]
    x, y, z = pick["pos"]
    warn = out["warn"]
    if list(c2r) == DEFAULT_C2R:
        warn.append("폰 거치 자세가 보정 전 임시값(CAD) — 자리가 몇 cm 틀릴 수 있다 (준비 → 거치 자세 보정)")
    elif cfg.get("_imu_tilt") and [round(float(v), 4) for v in c2r[:3]] == [round(v, 4) for v in DEFAULT_C2R[:3]]:
        out.setdefault("note", []).append("거치 자세: 기울기는 폰 중력 센서로 맞춤 · 카메라 자리·yaw 는 CAD 값")
    h = z + table_d                       # 책상 면 위 높이
    if not Z_TOL[0] <= h <= CUBE_SIDE + Z_TOL[1]:
        out["usable"] = False             # 화면이 '집으러 가기' 를 막는다
        warn.append(f"큐브 높이가 책상 위 {h * 100:.1f} cm 로 나옴 (0~5 cm 여야 함) — 거치 자세·책상 높이가 안 맞아 이 자리는 못 쓴다")
    yaw, src = table_yaw(o.get("rotation"), c2r)
    if cfg["yaw_manual_deg"] is not None:
        yaw, src = _fold(cfg["yaw_manual_deg"]), "manual"
    elif yaw is None:
        yaw = 0.0
        warn.append("큐브 각을 못 구함 — 0° 로 둠 (준비에서 각을 직접 넣을 수 있다)")
    elif src == "mixed":
        warn.append("두 면이 섞여 보여 각이 부정확할 수 있다")
    for w in o.get("warnings") or []:
        if isinstance(w, str):
            warn.append("폰 프로그램 경고: " + w[:80])
    dims = numbers(o.get("dims_mm"), 3)
    out.update(ok=True, cube={"x": round(x, 4), "y": round(y, 4), "yaw_deg": round(yaw, 1)}, z=round(z, 4), height_cm=round(h * 100, 1),
               pos=pick["pos"], cam=[round(v, 4) for v in pick["xyz"]], dims_mm=[round(v, 1) for v in dims], yaw_from=src,
               label=o.get("label"), instance=pick["instance"], score=round(pick["score"], 3), tracked=pick["tracked"])
    return out
