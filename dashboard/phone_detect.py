"""휴대폰 사진 한 장 → 큐브 자리 (x, y, yaw) — ArUco 만 쓴다 (OpenCV 4.5 옛 API: Dictionary_get / detectMarkers).

    기준 종이 (A4 가로, dashboard/phone_markers/phone_markers.pdf 1쪽): ArUco 5x5 (DICT_5X5_50) ID 0~5, 한 변 32 mm. 책상에 붙인다.
    큐브 윗면: ArUco 4x4 (DICT_4X4_50) ID 0~4, 한 변 40 mm (같은 PDF 2쪽 = ID 0, 탑 쌓기 stack_markers.pdf = ID 0~4).
    두 사전의 ID 가 겹치므로 (0~4) 사전마다 따로 찾고 (split_markers), 같은 자리를 두 사전이 다 읽으면 둘 다 버린다.

좌표 = 로봇 실좌표 (pick_plan 과 같음): 원점 = 어깨 roll 축, +x = 로봇 왼쪽, +y = 뒤 (책상은 −y), 위에서 본 yaw 반시계.
종이 좌표 (mm): 원점 = 종이 한가운데 십자, +x = 종이 오른쪽, +y = 종이 위쪽 ('로봇 쪽' 화살표). 종이를 로봇과 나란히 놓으면
(rot 0) 종이 +x = 로봇 +x, 종이 +y = 로봇 +y. 종이 가운데가 놓인 자리 (x, y, rot) 는 cfg 로 준다.

큐브 윗면은 책상보다 cube_h (5 cm) 높다 → 책상 평면 호모그래피만 쓰면 카메라 반대쪽으로 밀려 보인다 (비스듬히 찍으면 몇 cm).
그래서 기준 마커로 카메라 자세를 구해 (solvePnP; 초점거리는 훑어서 재투영 오차가 작고 큐브 마커 한 변이 40 mm 에 가장
가깝게 나오는 값) 큐브 마커 꼭짓점을 높이 cube_h 평면에 쏜다. 확인값: 그렇게 잰 큐브 마커 한 변.

기준 종이는 없어도 된다 (2026-11-24): 종이 마커가 하나도 안 보이면 카메라 = 거치대의 CAD 자세 (phone_table.cad_camera) 에
cfg 의 'intr' (폰 보정값 K·dist) 과 'td' (책상 높이 d, m) 로 놓고 같은 식으로 쏜다. 이때는 렌즈 자리·거치 각 오차가 그대로
자리에 들어간다 (각 1° ≈ 1 cm). camera.src = 'paper' | 'cad'.
"""
import math

REF_DICT_NAME = "DICT_5X5_50"    # 기준 종이
CUBE_DICT_NAME = "DICT_4X4_50"   # 큐브 윗면 (4X4_50/100/250/1000 은 ID 0~4 무늬가 같다)
CUBE_BITS = 4                    # 큐브 마커 안쪽 칸 수 (테두리 포함 한 변 = CUBE_BITS + 2 칸)
REF_SIZE_MM = 32.0
REF_MM = {0: (-122.0, 78.0), 1: (122.0, 78.0), 2: (122.0, 0.0), 3: (122.0, -78.0), 4: (-122.0, -78.0), 5: (-122.0, 0.0)}   # 마커 중심 (종이 mm)
CUBE_ID = 0
CUBE_MARKER_MM = 40.0
CUBE_IDS = tuple(range(0, 5))     # 탑 쌓기: 큐브마다 다른 마커 (4x4 ID 0~4, 같은 40 mm) — detect_cubes
MAX_LEVELS = 5                    # detect_cubes 가 보는 층 수 (윗면 높이 = 층 × cube_h). cfg "max_levels" 로 줄인다 —
                                  # 실시간 인식(live_api)은 2026-11-25 탑 쌓기를 없애 1 (큐브는 늘 책상 위): 기준 종이 없이 층을 고르면
                                  # 큐브마다 마커 크기 오차(±7%)로 한 층(5 cm = 10%) 틀렸다
DEFAULT_CFG = {"x": -0.05, "y": -0.32, "rot_deg": 0.0, "cube_h": 0.05}   # 종이 가운데 (실좌표 m), 종이 돌아간 각, 큐브 높이
MAX_SIDE = 2000   # 이보다 큰 사진은 줄여서 찾는다
CAD_LEVEL_FRAC = 0.25   # 기준 종이 없을 때: 큐브 층이 두 층 사이 이 비율 (0 = 딱, 0.5 = 한가운데) 넘게 어중간하면 층 의심 (level_unsure)
MARKER_WARN_FRAC = 0.30  # 2026-11-25 사용자 'ArUco 오류 너무 많음, 범위 최대한 널널하게': 마커 한 변 경고 10% → 30% (45.9 mm 도 경고였다)
CAD_SIDE_TOL = 0.30     # (0.04 → 0.30 같은 이유 — 탑 쌓기를 없애 늘 책상 위라 층을 가릴 일이 없다) 기준 종이 없을 때: 마커 한 변이 40 mm 에서 이 비율 넘게 다르면 unsure (종이 있을 때 0.06 보다 좁게)


def as_image(image):
    """사진 바이트 (JPEG·PNG) 또는 이미 풀린 그림 (BGR numpy 배열 — 영상 프레임) → BGR 배열 (못 읽으면 None).
    detect·detect_cubes·phone_table.measure_aruco 가 둘 다 받게 하는 한 곳. 배열은 복사한다 (부른 쪽 프레임에 안 그린다)."""
    import cv2
    import numpy as np
    if isinstance(image, np.ndarray):
        if image.ndim == 2:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return image[:, :, :3].copy() if image.ndim == 3 and image.shape[2] >= 3 and image.dtype == np.uint8 else None
    try:
        return cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
    except (TypeError, ValueError, cv2.error):
        return None


def _dictionary(cv2, name=REF_DICT_NAME):
    return cv2.aruco.Dictionary_get(getattr(cv2.aruco, name))


def _quad_hit(cv2, np, a, b):
    """두 꼭짓점 4x2 가 같은 자리인가 (한쪽 가운데가 다른 쪽 안)."""
    return (cv2.pointPolygonTest(b.astype(np.float32), tuple(float(v) for v in a.mean(axis=0)), False) >= 0
            or cv2.pointPolygonTest(a.astype(np.float32), tuple(float(v) for v in b.mean(axis=0)), False) >= 0)


def split_markers(cv2, np, gray, prm, ref_ids=None, cube_ids=None):
    """사진 (회색) → (기준 {id: 4x2}, 큐브 {id: 4x2}). 기준 = 5x5 사전, 큐브 = 4x4 사전 — 사전마다 detectMarkers 를 따로 부른다
    (ID 0~4 가 두 사전에 다 있다). ids 를 주면 그 안의 것만. 한 마커를 두 사전이 다 읽으면 (오검출) 어느 쪽인지 몰라 둘 다 버린다."""
    def one(name, keep):
        corners, ids, _ = cv2.aruco.detectMarkers(gray, _dictionary(cv2, name), parameters=prm)
        ids = [] if ids is None else [int(i) for i in ids.ravel()]
        return {i: c.reshape(4, 2).astype(np.float64) for i, c in zip(ids, corners) if keep is None or i in keep}
    refs, cubes = one(REF_DICT_NAME, ref_ids), one(CUBE_DICT_NAME, cube_ids)
    bad_r = {i for i, a in refs.items() if any(_quad_hit(cv2, np, a, b) for b in cubes.values())}
    bad_c = {i for i, b in cubes.items() if any(_quad_hit(cv2, np, a, b) for a in refs.values())}
    return ({i: c for i, c in refs.items() if i not in bad_r}, {i: c for i, c in cubes.items() if i not in bad_c})


def ref_corners_real(cfg):
    """{id: 4x2 (TL, TR, BR, BL) 실좌표 m} — detectMarkers 꼭짓점 순서와 같다."""
    import numpy as np
    c, s = math.cos(math.radians(cfg["rot_deg"])), math.sin(math.radians(cfg["rot_deg"]))
    h = REF_SIZE_MM / 2
    out = {}
    for i, (mx, my) in REF_MM.items():
        pts = [(mx - h, my + h), (mx + h, my + h), (mx + h, my - h), (mx - h, my - h)]
        out[i] = np.array([(cfg["x"] + (c * px - s * py) / 1000, cfg["y"] + (s * px + c * py) / 1000) for px, py in pts])
    return out


def _cameras(cv2, np, obj, img, w, h):
    """기준 꼭짓점(책상 z=0) → 초점거리 후보마다 (K, R, 카메라 자리, 재투영 rms px). 초점거리를 모르니 훑는다."""
    obj3 = np.hstack([obj, np.zeros((len(obj), 1))]).astype(np.float64)
    out = []
    for f in np.geomspace(0.45, 2.6, 36) * max(w, h):
        K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
        try:
            ok, rv, tv = cv2.solvePnP(obj3, img, K, None, flags=cv2.SOLVEPNP_IPPE)
        except cv2.error:
            continue
        if not ok:
            continue
        pr, _ = cv2.projectPoints(obj3, rv, tv, K, None)
        rms = float(np.sqrt(np.mean(np.sum((pr.reshape(-1, 2) - img) ** 2, axis=1))))
        R, _ = cv2.Rodrigues(rv)
        C = (-R.T @ tv).ravel()
        if C[2] > 0.08:   # 카메라가 책상 아래·바로 위일 리 없다 (거울 해)
            out.append((K, R, C, rms))
    return out


def _top(np, cam, cpix, cube_h):
    """큐브 마커 꼭짓점(사진) → 높이 cube_h 평면 위 실좌표 4x2, 그렇게 잰 한 변 (mm)."""
    K, R, C, _ = cam
    rays = (R.T @ np.linalg.inv(K) @ np.hstack([cpix, np.ones((4, 1))]).T).T
    top = np.array([(C + d * (cube_h - C[2]) / d[2])[:2] for d in rays])
    return top, float(np.mean([np.linalg.norm(top[i] - top[(i + 1) % 4]) for i in range(4)])) * 1000


def _tilt(cv2, np, cam, cpix):
    """큐브 마커 한 장의 기울기 (°, 책상 면과의 각): 마커만으로 자세를 풀어 (IPPE, 재투영 오차가 작은 쪽) 그 법선을 기준 마커로 구한
    카메라 자세로 로봇 틀에 돌린다. 탑 쌓기가 '놓은 큐브가 기울었나' 를 본다. 마커가 작으면 (20 px 안팎) 몇 도 흔들린다. 못 풀면 None."""
    K, R = cam[0], cam[1]
    h = CUBE_MARKER_MM / 2000.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    try:
        n, rvs, _, _ = cv2.solvePnPGeneric(obj, cpix.astype(np.float64), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return None
    if not n:
        return None
    Rm, _ = cv2.Rodrigues(rvs[0])
    up = R.T @ Rm[:, 2]
    return float(np.degrees(np.arccos(min(1.0, abs(float(up[2])) / max(1e-9, float(np.linalg.norm(up)))))))


def detect(image_bytes, cfg=None):
    """사진(JPEG/PNG 바이트) → dict. ok=False 여도 annotated(BGR 그림)·refs 는 될 수 있는 만큼 채운다."""
    import cv2
    import numpy as np
    cfg = {**DEFAULT_CFG, **(cfg or {})}
    out = {"ok": False, "refs": [], "cube_seen": False, "warn": [], "cfg": cfg}
    img = as_image(image_bytes)
    if img is None:
        return {**out, "reason": "사진을 못 읽음 (JPEG·PNG 만)"}
    k = MAX_SIDE / max(img.shape[:2])
    if k < 1:
        img = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
    h, w = img.shape[:2]
    prm = cv2.aruco.DetectorParameters_create()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    real = ref_corners_real(cfg)
    CID = int(cfg.get("cube_id", CUBE_ID))   # 집을 큐브 마커 ID (2026-11-25: 1번만 있으면 '0번 안 보임' 으로 못 집었다 — live 가 보이는 큐브로 고른다)
    rf, found = split_markers(cv2, np, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), prm, set(real), {CID})   # rf = 기준, found = 큐브
    vis = img.copy()
    lw = max(2, w // 400)
    refs = sorted(rf)
    for i in refs:
        cv2.polylines(vis, [rf[i].astype(np.int32)], True, (60, 200, 60), lw)
        cv2.putText(vis, str(i), tuple(int(v) for v in rf[i].mean(axis=0)), cv2.FONT_HERSHEY_SIMPLEX, w / 900, (60, 200, 60), lw)
    out.update(refs=refs, cube_seen=CID in found, annotated=vis, size=[w, h])
    if CID in found:
        cv2.polylines(vis, [found[CID].astype(np.int32)], True, (0, 140, 255), lw)
    if CID not in found:
        return {**out, "reason": f"큐브 마커(ID {CID})가 안 보임 — 큐브 윗면이 보이게 다시" + (f" (기준 마커는 {len(refs)}개 보임)" if refs else "")}
    if not refs:   # 기준 종이 없음 → CAD 거치 자세 + 책상 높이
        cad = _cad(cv2, np, cfg, found, w, h, (CID,))
        if cad is None:
            return {**out, "reason": "기준 종이가 안 보이고 폰 보정값·책상 높이도 없어 자리를 못 구함"}
        cam, cpix = cad["cam"], cad["und"][CID]
        top, side = _top(np, cam, cpix, cfg["cube_h"])
        out["camera"] = {"src": "cad", "pos": [round(float(v), 3) for v in cam[2]], "f_px": round(float(cam[0][0, 0]), 1), "image_rot_deg": cad["q"] * 90}
        return _finish_one(cv2, out, vis, lw, top, side, found[CID])
    if len(refs) < 2:
        out["warn"].append("기준 마커가 1개만 보임 — 정확도가 떨어진다 (2개 이상 나오게)")

    obj = np.vstack([real[i] for i in refs])
    pix = np.vstack([rf[i] for i in refs])
    H, _ = cv2.findHomography(pix, obj)   # 사진 → 책상 평면 (실좌표 m)
    if H is None:
        return {**out, "reason": "기준 마커로 책상 평면을 못 구함"}
    back = cv2.perspectiveTransform(pix.reshape(-1, 1, 2), H).reshape(-1, 2)
    out["ref_rms_mm"] = round(float(np.sqrt(np.mean(np.sum((back - obj) ** 2, axis=1)))) * 1000, 2)
    cpix = found[CID]
    flat = cv2.perspectiveTransform(cpix.reshape(-1, 1, 2), H).reshape(-1, 2)   # 높이 보정 없는 값 (책상 평면에 쏜 것)
    cams = _cameras(cv2, np, obj, pix, w, h)
    if cams:
        # 재투영 오차가 거의 같은 후보(바로 위에서 찍으면 초점거리가 안 정해진다) 중 큐브 마커 한 변이 원래 크기에 가장 가까운 것
        lo = min(c[3] for c in cams)
        cam = min((c for c in cams if c[3] <= 1.25 * lo + 0.3), key=lambda c: abs(_top(np, c, cpix, cfg["cube_h"])[1] - CUBE_MARKER_MM))
        top, side = _top(np, cam, cpix, cfg["cube_h"])
        out.update(camera={"src": "paper", "pos": [round(float(v), 3) for v in cam[2]], "f_px": round(float(cam[0][0, 0]), 1), "reproj_px": round(cam[3], 2)},
                   height_shift_mm=round(float(np.linalg.norm(flat.mean(axis=0) - top.mean(axis=0))) * 1000, 1))
    else:
        top = flat
        side = float(np.mean([np.linalg.norm(top[i] - top[(i + 1) % 4]) for i in range(4)])) * 1000
        out["warn"].append("카메라 자세를 못 구해 큐브 높이 보정 없음 — 비스듬한 사진이면 몇 cm 틀릴 수 있다 (위에서 찍기)")
    return _finish_one(cv2, out, vis, lw, top, side, cpix)


def _cad(cv2, np, cfg, found, w, h, ids):
    """기준 종이 없을 때의 카메라 (phone_table.cad_camera) — cfg 에 intr·td 가 없으면 None. image_rot_q = 사진 회전 설정 (phone_table.rot_from_c2r)."""
    import phone_table as PT
    return PT.cad_camera(cv2, np, found, cfg.get("intr"), w, h, cfg.get("td"), ids, cfg.get("image_rot_q"), cfg.get("c2r"))


def _finish_one(cv2, out, vis, lw, top, side, cpix):
    """detect(): 윗면 4 꼭짓점 (실좌표) → cube {x, y, yaw_deg} + 그림."""
    out["cube_marker_mm"] = round(side, 1)
    if abs(side - CUBE_MARKER_MM) > MARKER_WARN_FRAC * CUBE_MARKER_MM:
        out["warn"].append(f"큐브 마커 한 변이 {side:.1f} mm 로 재짐 (원래 {CUBE_MARKER_MM:.0f} mm) — 인쇄 크기·큐브 높이·사진 각도 확인")
    cx, cy = top.mean(axis=0)
    ex = (top[1] - top[0]) + (top[2] - top[3])   # 마커 +x (TL→TR, BL→BR)
    yaw = math.degrees(math.atan2(ex[1], ex[0]))
    yaw = (yaw + 45.0) % 90.0 - 45.0             # 큐브는 90° 마다 같다
    out.update(ok=True, cube={"x": round(float(cx), 4), "y": round(float(cy), 4), "yaw_deg": round(yaw, 1)})
    c = tuple(int(v) for v in cpix.mean(axis=0))
    cv2.circle(vis, c, lw * 3, (0, 140, 255), -1)
    cv2.arrowedLine(vis, c, tuple(int(v) for v in (cpix[1] + cpix[2]) / 2), (0, 140, 255), lw)
    return out


def detect_cubes(image_bytes, cfg=None, ids=CUBE_IDS):
    """탑 쌓기: 사진 한 장 → 큐브 여러 개 (마커 ID 가 ids 안인 것 모두). detect() 는 그대로 (ID CUBE_ID 하나).

    돌려주는 것: {ok, cubes: [{id, x, y, yaw_deg, level, top_cm, marker_mm}], refs, warn, annotated, …}.
    level = 그 큐브 밑에 깔린 큐브 수 (0 = 책상 위, 1 = 한 층 위 …), top_cm = 윗면 높이 (= (level + 1) × cube_h).
    높이: 기준 마커로 구한 카메라 자세에서 마커 꼭짓점을 높이 h 평면에 쏘면 마커 한 변이 h 에 따라 달라진다 (카메라가 50 cm 위면
    한 층에 약 12 %) → 한 변이 40 mm 에 가장 가까운 층. 초점거리를 몰라서, 재투영 오차가 비슷한 초점거리 후보 중 모든 큐브의
    한 변이 가장 잘 맞는 것을 고른다 (같으면 낮은 층). 바로 위에서 찍으면 초점거리와 높이가 서로 바뀌어도 사진이 같아
    층을 못 가린다 — 비스듬히(30° 이상) 찍어야 한다. 그럴 때는 warn 에 적는다."""
    import cv2
    import numpy as np
    cfg = {**DEFAULT_CFG, **(cfg or {})}
    out = {"ok": False, "refs": [], "cubes": [], "warn": [], "cfg": cfg}
    img = as_image(image_bytes)
    if img is None:
        return {**out, "reason": "사진을 못 읽음 (JPEG·PNG 만)"}
    k = MAX_SIDE / max(img.shape[:2])
    if k < 1:
        img = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
    h, w = img.shape[:2]
    prm = cv2.aruco.DetectorParameters_create()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    # 큐브 윗면의 흰 네모(50 mm)도 네모 후보로 잡힌다 — 기본값(0.05)이면 그 안의 마커(40 mm)와 '너무 가까운 후보'로 묶여 마커 쪽이 버려진다
    # (어두운 책상·큐브 옆면 위에서 12 장 중 6 장을 놓쳤다). 가까운 후보도 둘 다 남긴다
    prm.minMarkerDistanceRate = 0.02
    real = ref_corners_real(cfg)
    rf, found = split_markers(cv2, np, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), prm, set(real), set(ids))   # rf = 기준 (5x5), found = 큐브 (4x4)
    vis = img.copy()
    lw = max(2, w // 400)
    refs = sorted(rf)
    seen = sorted(found)
    for i in refs:
        cv2.polylines(vis, [rf[i].astype(np.int32)], True, (60, 200, 60), lw)
    out.update(refs=refs, annotated=vis, size=[w, h], seen=seen)
    if not seen:
        return {**out, "reason": f"큐브 마커(ID {min(ids)}~{max(ids)})가 하나도 안 보임 — 큐브 윗면이 보이게 다시" + (f" (기준 마커는 {len(refs)}개 보임)" if refs else "")}
    src = "paper"
    if refs:
        if len(refs) < 2:
            out["warn"].append("기준 마커가 1개만 보임 — 정확도가 떨어진다 (2개 이상 나오게)")
        obj = np.vstack([real[i] for i in refs])
        pix = np.vstack([rf[i] for i in refs])
        cams = _cameras(cv2, np, obj, pix, w, h)
        if not cams:
            return {**out, "reason": "기준 마커로 카메라 자세를 못 구함 — 큐브 높이를 알 수 없다. 기준 종이가 더 크게·또렷하게 나오게 다시"}
    else:          # 기준 종이 없음 → CAD 거치 자세 + 책상 높이 (초점거리를 아니까 후보 하나)
        cad = _cad(cv2, np, cfg, found, w, h, ids)
        if cad is None:
            return {**out, "reason": "기준 종이가 안 보이고 폰 보정값·책상 높이도 없어 자리를 못 구함"}
        if "cam" not in cad:
            return {**out, "reason": "기준 종이가 안 보임 · " + cad["why"]}
        cams, src = [cad["cam"]], "cad"
        out["image_rot_deg"], out["image_rot_src"] = cad["q"] * 90, cad["rot_src"]
        found = {**found, **cad["und"]}       # 왜곡 푼 꼭짓점으로 쏜다 (그림은 원래 꼭짓점)
    ch = float(cfg["cube_h"])

    def fit(cam):   # 큐브마다 한 변이 가장 잘 맞는 층 → (점수, [(id, 층, 윗면 4x2, 한 변 mm, 층 경계까지 비율)])
        rows, cost = [], 0.0
        for i in seen:
            es = []
            for lv in range(max(1, int(cfg.get("max_levels") or MAX_LEVELS))):
                top, side = _top(np, cam, found[i], (lv + 1) * ch)
                es.append((abs(side - CUBE_MARKER_MM) / CUBE_MARKER_MM, lv, top, side))
            best = min(es, key=lambda r: (r[0], r[1]))
            e2 = min((r[0] for r in es if abs(r[1] - best[1]) == 1), default=None)
            frac = best[0] / (best[0] + e2) if e2 is not None and best[0] + e2 > 0 else 0.0   # 0 = 그 층 딱, 0.5 = 두 층 사이
            rows.append((i, best[1], best[2], best[3], frac))
            cost += best[0] + 0.004 * best[1]   # 거의 같으면 낮은 층
        return cost, rows
    lo = min(c[3] for c in cams)
    near = [c for c in cams if c[3] <= 1.25 * lo + 0.3]
    fits = [(fit(c), c) for c in near]
    (cost, rows), cam = min(fits, key=lambda x: x[0][0])
    # 다른 초점거리 후보가 다른 층으로도 비슷하게 맞으면 (바로 위에서 찍음) 층을 믿을 수 없다
    alt = [f for f, _ in fits if [r[1] for r in f[1]] != [r[1] for r in rows] and f[0] - sum(0.004 * r[1] for r in f[1]) < 0.03 * len(seen)]
    one_level = int(cfg.get("max_levels") or MAX_LEVELS) <= 1     # 늘 책상 위 (탑 쌓기 없음) — 층 의심 경고는 뜻이 없다
    if alt and not one_level:
        out["warn"].append("사진이 거의 바로 위에서 찍혀 큐브 높이(층)가 확실하지 않음 — 비스듬히(30° 이상 기울여) 다시 찍기")
        out["level_unsure"] = True
    # CAD 모드 (기준 종이 없음) 는 초점거리 후보가 하나라 위 판정이 늘 꺼진다 (2026-11-24 검토) → 큐브마다 두 층 사이 어중간함으로 본다:
    # 마커 한 변이 가장 맞는 층과 이웃 층 사이 CAD_LEVEL_FRAC 넘게 와 있으면 층 의심 (2층 맨 위가 1층으로 읽히면 든 큐브가 기존 큐브를 누른다)
    if src == "cad" and not one_level:
        amb = [r[0] for r in rows if r[4] > CAD_LEVEL_FRAC]
        if amb:
            out["warn"].append("큐브 " + ", ".join(map(str, amb)) + " 의 층이 확실하지 않음 (기준 종이 없이 두 층 사이로 재짐) — 기준 종이를 같이 보이게 하거나 큐브를 폰 가까이")
            out["level_unsure"] = True
    out["camera"] = {"src": src, "pos": [round(float(v), 3) for v in cam[2]], "f_px": round(float(cam[0][0, 0]), 1), "reproj_px": round(cam[3], 2)}
    for i, lv, top, side, frac in rows:
        cx, cy = top.mean(axis=0)
        ex = (top[1] - top[0]) + (top[2] - top[3])
        yaw = (math.degrees(math.atan2(ex[1], ex[0])) + 45.0) % 90.0 - 45.0
        c = {"id": i, "x": round(float(cx), 4), "y": round(float(cy), 4), "yaw_deg": round(yaw, 1), "level": lv,
             "top_cm": round((lv + 1) * ch * 100, 1), "marker_mm": round(side, 1)}
        t = _tilt(cv2, np, cam, found[i])
        if t is not None:
            c["tilt_deg"] = round(t, 1)
        if abs(side - CUBE_MARKER_MM) > (CAD_SIDE_TOL if src == "cad" else 0.06) * CUBE_MARKER_MM or (src == "cad" and frac > CAD_LEVEL_FRAC and not one_level):
            c["unsure"] = True
            out["warn"].append(f"큐브 {i}: 마커 한 변이 {side:.1f} mm 로 재짐 (원래 {CUBE_MARKER_MM:.0f}) — 높이·자리가 틀릴 수 있다")
        out["cubes"].append(c)
        col = (0, 140, 255) if lv == 0 else (255, 120, 0)
        cpix = found[i]
        cv2.polylines(vis, [cpix.astype(np.int32)], True, col, lw)
        p0 = tuple(int(v) for v in cpix.mean(axis=0))
        cv2.arrowedLine(vis, p0, tuple(int(v) for v in (cpix[1] + cpix[2]) / 2), col, lw)
        cv2.putText(vis, f"{i}" + (f" +{lv}" if lv else ""), tuple(int(v) for v in cpix[0]), cv2.FONT_HERSHEY_SIMPLEX, w / 900, col, lw)
    out["ok"] = True
    return out
