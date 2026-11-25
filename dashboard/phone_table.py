"""휴대폰 카메라 탭의 '책상 높이 재기': 거치대에 끼운 폰의 사진 한 장 (또는 폰 뎁스) → 책상 높이 d (어깨 roll 축 ~ 책상 윗면, m).

보정 단계 없이 폰이 CAD 자리에 있다고 믿고 잰다. 거치 사양 = team-jax/JaxIsaacLab artifacts/deploy_2026-11-05/CAMERA.md (2026-11-12 CAD):
    카메라 = 바닥에서 1.2005 m (어깨 피치축 − 7 mm), 프로파일 중심선 앞 117.7 mm, 좌우 가운데, 앞을 보고 45° 아래.
    카메라 좌표 (정우 '베이스' 축: x = 뒤, y = 로봇 오른쪽, z = 위) 에서 어깨 피치축 = (−0.032, 0, +0.007).
    광학 좌표 (OpenCV: x 영상 오른쪽, y 영상 아래, z 시선) → 카메라 좌표 = R_OPT.

우리 로봇 좌표 (pick_plan 과 같음): 원점 = 어깨 roll 축, +x = 로봇 왼쪽, +y = 뒤 (앞 = −y), +z = 위, 책상 면 z = −d.
축 대응:  우리 (x, y, z) = (−y_c, x_c, z_c) + CAM_POS      (정우 y = 로봇 오른쪽이라 부호가 바뀌고, 정우 x = 뒤 = 우리 y)
    CAM_POS = (0.174, −0.1177, −0.012): 팀 camera.yaml 의 CAD 점 (거치대 꼭짓점 P − 어깨 roll 축 R0) — phone_depth.DEFAULT_C2R 의 앞 3개와 같다.
    CAMERA.md 와 맞춰 보면: 어깨 피치축 = 우리 좌표 (0.174, −0.1497, −0.005) → roll 축이 피치축보다 5 mm 위. 그래서
    CAMERA.md 의 '책상 윗면 0.735 m, 카메라 0.4655 m 위' = 우리 d 0.4655 + 0.012 = 0.4775 (역기구학 탭 기본값) 로 딱 맞는다.
    회전: DEFAULT_C2R 의 뒤 3개 (IMU 로 잰 값, 폰이 뒤집혀 끼워짐) = 아래 R_CAD × 광축 둘레 180° 에서 1.4° 다르다.

재는 법: 책상 평면을 광학 좌표에서 구한다 (법선 n, 카메라에서 평면까지 수직 거리 h).
    ArUco  = 기준 종이 마커 (5x5 ID 0~5, 32 mm) 꼭짓점으로 solvePnP. 초점거리는 모르므로 재투영 오차가 가장 작은 값으로 맞춘다.
    큐브   = 큐브 윗면 마커 (4x4 ID 0~4, 40 mm) 하나하나로 solvePnP (초점거리 = 폰 보정 파일) → CAD 자세로 마커 높이 − cube_h (measure_cubes).
             기준 종이가 없어도 된다. 이쪽은 CAD 회전으로 z 를 읽으므로 거치 각 1° 에 약 8 mm 틀린다 (마커 법선 값 d_normal 로 확인).
    뎁스   = 폰 스테레오 프로그램이 저장한 rgbd/rgbd.npz (--rgbd-viewer) 의 점들에 평면 맞추기 (RANSAC).
    d = h − CAM_POS z = h + 0.012.   책상이 수평이면 수직 거리 h 는 폰이 얼마나 기울었든 같다 → 45° 가 1° 틀려도 d 는 안 변한다
    (CAD 회전으로 점을 옮겨 z 를 읽으면 1° 에 약 8 mm 틀린다 — 그 값은 d_cad_rot 으로 같이 준다).
    기울기 검사: 잰 법선과 CAD 가 말하는 '위' 의 각 (사진이 90° 단위로 돌아가 있어도 되게 4 방향 중 가장 가까운 것). 2° 넘으면 경고.

오차 (솔직하게): ① 렌즈 자리 — CAD 점은 거치대 꼭짓점이고 렌즈는 거기서 경사를 따라 0~40 mm 위 → d 가 0~28 mm 크게 나올 수 있다 (미반영).
② 초점거리 — 사진에서 추정한다. 거리가 그 오차만큼 비례해 틀린다 (1 % = 약 5 mm). 렌즈 왜곡은 안 본다. ③ 인쇄 크기 (100 mm 선 확인).
그래서 처음 한 번은 줄자로 잰 값과 맞춰 볼 것.
"""
import json
import math
import os
import threading

import phone_detect as PD

CAM_POS = (0.174, -0.1177, -0.012)      # 카메라 자리 (우리 로봇 좌표 m) — CAMERA.md / camera.yaml CAD
CAM_PITCH_DEG = 45.0                    # 아래로 기운 각 (CAD)
_S = math.sqrt(0.5)
R_OPT = ((0.0, _S, -_S), (1.0, 0.0, 0.0), (0.0, -_S, -_S))            # CAMERA.md 2.1: 광학 → 카메라 좌표 (정우 축)
R_CAD = ((-1.0, 0.0, 0.0), (0.0, _S, -_S), (0.0, -_S, -_S))           # 광학 → 우리 로봇 좌표 = [[0,−1,0],[1,0,0],[0,0,1]] · R_OPT
D_RANGE = (0.44, 0.60)                  # 계획이 받는 범위 (hand_ik.D_RANGE 와 같게 — phone_api 가 그쪽 값으로 덮어쓴다)
TILT_WARN_DEG = 2.0
MIN_REFS = 3                            # 초점거리까지 맞추려면 마커 3개 이상
DEPTH_MAX_AGE_S = 20.0                  # rgbd.npz 는 5초마다 저장된다
DEPTH_Z_RANGE = (0.2, 1.5)              # 평면 맞추기에 쓰는 점의 거리 (m)


def cam_to_robot(p_opt):
    """광학 좌표의 점 → 우리 로봇 좌표 (CAD 자세)."""
    return tuple(sum(R_CAD[i][j] * p_opt[j] for j in range(3)) + CAM_POS[i] for i in range(3))


def plane_to_d(n, p0):
    """광학 좌표의 책상 평면 (법선 n, 평면 위 한 점 p0) → dict(d, h, tilt_deg, image_rot_deg, cam_pitch_deg, d_cad_rot).
    d = 수직 거리 h − CAM_POS z. tilt = 법선과 CAD '위' (R_CAD 3행) 의 각 — 사진이 광축 둘레로 90° 단위 돌아간 4 경우 중 가장 작은 것."""
    n, p0 = [float(v) for v in n], [float(v) for v in p0]
    k = math.sqrt(sum(v * v for v in n))
    n = [v / k for v in n]
    if sum(a * b for a, b in zip(n, p0)) > 0:      # 법선이 카메라 쪽 (책상에서 위) 을 보게
        n = [-v for v in n]
    h = -sum(a * b for a, b in zip(n, p0))
    best = None
    for q in range(4):                             # 광학 좌표를 광축 둘레로 q × 90° 돌린 뒤 R_CAD
        c, s = round(math.cos(q * math.pi / 2)), round(math.sin(q * math.pi / 2))
        rot = lambda v: (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])   # noqa: E731
        up = sum(R_CAD[2][j] * rot(n)[j] for j in range(3))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, up))))
        if best is None or ang < best[0]:
            best = (ang, q * 90, -(sum(R_CAD[2][j] * rot(p0)[j] for j in range(3)) + CAM_POS[2]))
    return {"d": h - CAM_POS[2], "h": h, "tilt_deg": best[0], "image_rot_deg": best[1],
            "cam_pitch_deg": math.degrees(math.asin(max(-1.0, min(1.0, abs(n[2]))))), "d_cad_rot": best[2]}


def _finish(out, pl):
    """plane_to_d 결과를 화면용으로 (반올림 + 기울기 경고)."""
    out.update(ok=True, d=round(pl["d"], 4), cam_above_table=round(pl["h"], 4), tilt_deg=round(pl["tilt_deg"], 2),
               image_rot_deg=pl["image_rot_deg"], cam_pitch_deg=round(pl["cam_pitch_deg"], 2), d_cad_rot=round(pl["d_cad_rot"], 4))
    if pl["tilt_deg"] > TILT_WARN_DEG:
        out["warn"].append(f"책상 평면이 CAD 자세에서 {pl['tilt_deg']:.1f}° 기울어 보임 (카메라 기울기 {pl['cam_pitch_deg']:.1f}°, CAD {CAM_PITCH_DEG:.0f}°) "
                           "— 폰이 거치대에 제대로 안 끼워졌거나 거치 각이 다르다")
    return out


# ---- ArUco: 기준 종이 한 장 --------------------------------------------------------------------------------------------

def _pose(cv2, np, obj3, pix, f, w, h):
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
    try:
        ok, rv, tv = cv2.solvePnP(obj3, pix, K, None, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None
    if not ok or float(tv[2]) <= 0:
        return None
    rv, tv = cv2.solvePnPRefineLM(obj3, pix, K, None, rv, tv)      # IPPE 는 호모그래피 해라 재투영 최소가 아니다 → 이 f 에서의 최소로
    pr, _ = cv2.projectPoints(obj3, rv, tv, K, None)
    rms = float(np.sqrt(np.mean(np.sum((pr.reshape(-1, 2) - pix) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rv)
    return rms, R, tv.ravel()


def _fit_focal(cv2, np, obj3, pix, w, h):
    """초점거리(px)를 훑어 재투영 오차가 가장 작은 것 → (f, (rms, R, t), [f 낮은 쪽, 높은 쪽] 2σ) 또는 None.
    평면 한 장이라도 비스듬히 (거치대 45°) 보면 가까운 쪽·먼 쪽 마커의 크기 차이로 초점거리가 정해진다. 바로 위에서 보면 안 정해진다."""
    m = max(w, h)
    rows = [(f, _pose(cv2, np, obj3, pix, f, w, h)) for f in np.geomspace(0.4, 3.0, 60) * m]
    rows = [(f, p) for f, p in rows if p is not None]
    if not rows:
        return None
    f0 = min(rows, key=lambda r: r[1][0])[0]
    fine = [(f, _pose(cv2, np, obj3, pix, f, w, h)) for f in np.geomspace(f0 / 1.15, f0 * 1.15, 141)]
    fine = [(f, p) for f, p in fine if p is not None]
    f, best = min(fine, key=lambda r: r[1][0])
    # 2σ: 꼭짓점 잡음이 재투영 오차만 하다고 보고 Δχ² = 4 안쪽 (자유도 = 좌표 2N − 미지수 7). 잡음 바닥 0.15 px
    dof = max(1, 2 * len(pix) - 7)
    lim = max(best[0], 0.15) ** 2 * (1 + 4.0 / dof)
    near = [ff for ff, p in fine if p[0] ** 2 <= lim]
    return float(f), best, (float(min(near)), float(max(near)))


def measure_aruco(image_bytes, intr=None):
    """사진 (JPEG/PNG 바이트, 거치대의 폰으로 찍은 것) → dict(ok, d, tilt_deg, f_px, d_pm_mm, refs, warn, annotated …). 종이가 놓인 자리는 몰라도 된다.
    intr (폰 보정값) 을 주면 사진에서 맞춘 초점거리와 비교만 한다 (f_calib, 8 % 넘게 다르면 경고 — 큐브 마커 재기가 그 값을 쓴다)."""
    import cv2
    import numpy as np
    out = {"ok": False, "refs": [], "warn": []}
    img = PD.as_image(image_bytes)
    if img is None:
        return {**out, "reason": "사진을 못 읽음 (JPEG·PNG 만)"}
    k = PD.MAX_SIDE / max(img.shape[:2])
    if k < 1:
        img = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
    h, w = img.shape[:2]
    prm = cv2.aruco.DetectorParameters_create()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    sheet = PD.ref_corners_real({"x": 0.0, "y": 0.0, "rot_deg": 0.0})     # 종이 좌표 (m) — 평면만 필요해 놓인 자리는 안 쓴다
    found, _ = PD.split_markers(cv2, np, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), prm, set(sheet), set(PD.CUBE_IDS))   # 기준 (5x5) 만, 큐브 (4x4) 와 겹친 것은 뺀다
    refs = sorted(found)
    vis = img.copy()
    lw = max(2, w // 400)
    for i in refs:
        cv2.polylines(vis, [found[i].astype(np.int32)], True, (60, 200, 60), lw)
        cv2.putText(vis, str(i), tuple(int(v) for v in found[i].mean(axis=0)), cv2.FONT_HERSHEY_SIMPLEX, w / 900, (60, 200, 60), lw)
    out.update(refs=refs, annotated=vis, size=[w, h])
    if len(refs) < MIN_REFS:
        return {**out, "reason": f"기준 종이의 마커가 {len(refs)}개만 보임 ({MIN_REFS}개 이상 필요) — 종이를 화면 가운데쯤 책상에 평평하게 두고 다시"}
    obj = np.vstack([sheet[i] for i in refs])
    obj3 = np.hstack([obj, np.zeros((len(obj), 1))]).astype(np.float64)
    pix = np.vstack([found[i] for i in refs])
    fit = _fit_focal(cv2, np, obj3, pix, w, h)
    if fit is None:
        return {**out, "reason": "기준 마커로 카메라 자세를 못 구함 — 종이가 더 크게·또렷하게 나오게 다시"}
    f, (rms, R, t), (f_lo, f_hi) = fit
    pl = plane_to_d(R[:, 2], t)
    ds = []
    for ff in (f_lo, f_hi):                        # 초점거리 흔들림이 d 에 주는 폭
        p = _pose(cv2, np, obj3, pix, ff, w, h)
        if p is not None:
            ds.append(plane_to_d(p[1][:, 2], p[2])["d"])
    pm = max([abs(v - pl["d"]) for v in ds] or [0.0]) * 1000
    out.update(f_px=round(f, 1), reproj_px=round(rms, 2), d_pm_mm=round(pm, 1))
    sk = scaled_K(np, intr, w, h) if intr else None
    if intr and sk is None:
        out["warn"].append(intr_mismatch(intr, w, h))
    if sk is not None:
        fc = float(sk[0][0, 0])
        out["f_calib"] = round(fc, 1)
        if abs(f / fc - 1) > 0.08:
            out["warn"].append(f"사진에서 맞춘 초점거리 {f:.0f} px 가 폰 보정값 {fc:.0f} px 와 {abs(f / fc - 1) * 100:.0f} % 다름 — 보정 파일이 이 폰·화면 것이 아닐 수 있다")
    if len(refs) < 5:
        out["warn"].append(f"기준 마커 {len(refs)}개만 보임 — 초점거리 추정이 덜 정확하다 (6개 다 나오게)")
    if rms > 1.5:
        out["warn"].append(f"재투영 오차 {rms:.1f} px — 종이가 울었거나 사진이 흐리다")
    if pm > 8:
        out["warn"].append(f"초점거리가 잘 안 정해짐 (d ± {pm:.0f} mm) — 종이가 더 크게 나오게")
    cv2.putText(vis, f"d {pl['d']:.3f} m  tilt {pl['tilt_deg']:.1f} deg", (lw * 6, int(h - lw * 8)), cv2.FONT_HERSHEY_SIMPLEX, w / 900, (60, 200, 60), lw)
    if pl["tilt_deg"] <= 2 * TILT_WARN_DEG and rms <= 1.5:
        rot_vote(pl["image_rot_deg"] // 90, strong=True, src="paper")   # 종이 평면 하나 = 거울 문제 없음 → 큐브 마커의 사진 회전을 잠근다
    return _finish(out, pl)


# ---- ArUco: 큐브 윗면 마커 (기준 종이 없이) ----------------------------------------------------------------------------
# 마커 한 변 (40 mm) 을 아니까 마커 하나로 카메라 좌표 자세가 나온다 (solvePnP IPPE_SQUARE). 초점거리는 마커 하나로는 못 정하므로
# 폰 보정 파일 (calib_phone_pair*.json 의 K1·dist1 = 왼쪽 렌즈, live.jpg 와 같은 렌즈) 을 쓴다. 그 다음 CAD 거치 자세로 로봇 좌표:
#     마커 가운데 z = (CAD '위' · t) + CAM_POS z  →  책상 면 = 그 z − (층 + 1) × cube_h,  d = −책상 면 z.
# 사진이 광축 둘레로 90° 단위 돌아가 있을 수 있어 (폰이 뒤집혀 끼워짐) 마커 법선이 CAD '위' 와 가장 맞는 방향을 고른다.
# 마커 법선 자체 (40 mm 한 장이라 몇 도 흔들림) 로 잰 수직 거리 값은 d_normal 로 같이 준다 (거치 각이 틀리면 둘이 벌어진다).

CUBE_TILT_WARN_DEG = 6.0


def load_intrinsics(path):
    """폰 보정 파일 → {K, dist, size [w, h], file} 또는 None."""
    try:
        d = json.loads(open(path).read())
        K = [[float(v) for v in row] for row in d["K1"]]
        return {"K": K, "dist": [float(v) for v in d.get("dist1") or []], "size": [int(v) for v in d["image_size"]], "file": os.path.basename(path)}
    except (OSError, ValueError, KeyError, TypeError, IndexError):
        return None


ASPECT_TOL = 0.01                       # 프레임 가로세로 비가 보정 image_size 와 이보다 다르면 (잘린 화면·다른 모드) 보정값을 안 쓴다


def intr_mismatch(intr, w, h):
    """보정값을 이 프레임 (w × h) 에 쓸 수 없으면 이유, 쓸 수 있으면 None. 비율만 같으면 K 를 그 배율로 줄여 쓴다 (scaled_K)."""
    try:
        iw, ih = float(intr["size"][0]), float(intr["size"][1])
    except (KeyError, TypeError, ValueError, IndexError):
        return "보정 파일에 image_size 가 없음"
    if iw <= 0 or ih <= 0 or w <= 0 or h <= 0:
        return "보정·프레임 크기가 틀림"
    if abs((w / h) / (iw / ih) - 1) > ASPECT_TOL:
        return (f"프레임 {w}×{h} 의 가로세로 비가 보정 파일 ({intr.get('file')}) 의 {iw:.0f}×{ih:.0f} 와 다름 — 잘렸거나 다른 카메라 모드라 "
                "초점거리·주점이 틀린다 (보정값 안 씀)")
    return None


def scaled_K(np, intr, w, h):
    """보정 크기와 다른 프레임 (줄인 것 등) 이면 K 를 그 비율로. 가로세로 비가 다르면 (intr_mismatch) None — 조용히 틀리지 않게."""
    if intr_mismatch(intr, w, h):
        return None
    K = np.array(intr["K"], dtype=np.float64)
    sx, sy = w / float(intr["size"][0]), h / float(intr["size"][1])
    K[0] *= sx
    K[1] *= sy
    return K, np.array(intr.get("dist") or [0.0] * 5, dtype=np.float64)


def undistort(cv2, np, pix, K, dist):
    """사진 꼭짓점 → 왜곡 없는 (같은 K 의) 픽셀."""
    return cv2.undistortPoints(np.asarray(pix, np.float64).reshape(-1, 1, 2), K, dist, P=K).reshape(-1, 2)


def r_opt_robot(np, q):
    """사진이 광축 둘레로 q × 90° 돌아간 경우의 광학 → 로봇 회전 (plane_to_d 와 같은 규약)."""
    c, s = round(math.cos(q * math.pi / 2)), round(math.sin(q * math.pi / 2))
    return np.array(R_CAD) @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def marker_poses(cv2, np, und, K, size_m):
    """{id: 왜곡 푼 꼭짓점 4x2} → {id: [(R, t), …]} (IPPE_SQUARE 의 두 해, 카메라 좌표)."""
    h = size_m / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    out = {}
    for i, c in und.items():
        try:
            n, rvs, tvs, _ = cv2.solvePnPGeneric(obj, c.astype(np.float64), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            continue
        sols = []
        for rv, tv in zip(rvs[:n], tvs[:n]):
            if float(tv[2]) > 0:
                sols.append((cv2.Rodrigues(rv)[0], tv.ravel().astype(np.float64)))
        if sols:
            out[i] = sols
    return out


# 사진 회전 q (광축 둘레 0/90/180/270°) 고르기 — 2026-11-24 검토: 마커 하나가 화면 가운데쯤이면 IPPE 의 두 해 중 '거울' 해의 법선이
# 180° 돌린 CAD '위' 와 거의 같아 0° / 180° 를 못 가린다 (합성 640×480·마커 18 px·잡음 0.3 px: 400번 중 101번 틀림, d −20 ~ +44 mm).
# 큐브 마커 3개가 퍼져 있어도 둘의 차가 마커당 15° 쯤이라 프레임마다 고르면 안 된다. 거치는 고정이라 q 는 설정·잠금으로 정한다. 우선순위:
#   1. 기준 종이로 책상 높이를 잼 (measure_aruco: 마커 여러 개로 평면 하나 — 거울 문제 없음) → 잠금 (서버를 다시 켜기 전까지)
#   2. 설정 q_cfg = 폰 뎁스 설정의 camera_to_robot 회전에 가장 가까운 q (rot_from_c2r — 팀 camera.yaml/IMU 값, 기본은 뒤집힘 = 180°)
#      — 프레임이 다른 q 를 확실하게 (마커당 ROT_MARGIN_DEG 넘게) 말하면 설정이 틀렸을 수 있어 그 프레임은 거절
#   3. 설정도 없으면 마커당 ROT_MARGIN_DEG 넘게 확실한 프레임만 (합성 3만9천 번 중 이 차를 넘고 틀린 것 0), ROT_LOCK_N 번 쌓이면 잠금
#   그 밖은 '못 잼' (큐브 자리·책상 높이 안 줌).
ROT_MARGIN_DEG = 20.0
ROT_LOCK_N = 5
_ROT = {"q": None, "n": 0, "alt": None, "alt_n": 0, "src": None}
_ROT_LK = threading.Lock()


def rot_reset():
    with _ROT_LK:
        _ROT.update(q=None, n=0, alt=None, alt_n=0, src=None)


def rot_vote(q, strong=False, src="cubes"):
    """확실한 사진 회전 한 표. strong (기준 종이) 이면 바로 잠근다."""
    with _ROT_LK:
        if strong:
            _ROT.update(q=q, n=ROT_LOCK_N, alt=None, alt_n=0, src=src)
        elif q == _ROT["q"]:
            _ROT.update(n=min(_ROT["n"] + 1, 10 ** 6), alt=None, alt_n=0)
        elif _ROT["n"] < ROT_LOCK_N:            # 아직 안 잠김 — 다른 값이면 처음부터
            _ROT.update(q=q, n=1, alt=None, alt_n=0, src=src)
        else:                                   # 잠긴 값과 반대인 확실한 프레임: 연달아 ROT_LOCK_N 번이면 바꾼다
            _ROT.update(alt_n=_ROT["alt_n"] + 1 if _ROT["alt"] == q else 1, alt=q)
            if _ROT["alt_n"] >= ROT_LOCK_N:
                _ROT.update(q=q, n=ROT_LOCK_N, alt=None, alt_n=0, src=src)


def rot_locked():
    """잠긴 사진 회전 (q, 출처) 또는 None."""
    with _ROT_LK:
        return (_ROT["q"], _ROT["src"]) if _ROT["q"] is not None and _ROT["n"] >= ROT_LOCK_N else None


def rot_costs(np, poses):
    """q 4 가지마다 (마커당 평균 각°, q, {id: (R, t, 각°)}) — 작은 것부터. 각 = 마커 법선 (두 해 중 '위' 에 가까운 것) 과 그 q 의 CAD '위'."""
    rows = []
    for q in range(4):
        up = r_opt_robot(np, q)[2]                 # 광학 좌표 벡터 v 의 로봇 z = up · v
        ch, cost = {}, 0.0
        for i, sols in poses.items():
            R, t = min(sols, key=lambda s: -float(up @ s[0][:, 2]))
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(up @ R[:, 2])))))
            ch[i] = (R, t, ang)
            cost += ang
        rows.append((cost / len(poses), q, ch))
    return sorted(rows, key=lambda r: r[0])


def rot_from_c2r(c2r, tol_deg=10.0):
    """폰 뎁스 설정 camera_to_robot [tx, ty, tz, roll, pitch, yaw] 의 회전에 tol_deg 안으로 가까운 사진 회전 q, 없으면 None."""
    import numpy as np
    import phone_depth as DP
    try:
        R = np.array([DP.camera_to_robot(e, (0.0, 0.0, 0.0) + tuple(float(v) for v in c2r[3:6])) for e in np.eye(3)]).T
    except (TypeError, ValueError, IndexError):
        return None
    best = None
    for q in range(4):
        E = R @ r_opt_robot(np, q).T
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(E)) - 1) / 2))))
        if best is None or ang < best[0]:
            best = (ang, q)
    return best[1] if best[0] <= tol_deg else None


def choose_rot(np, poses, q_cfg=None):
    """사진 회전 고르기 → {q, ch, src 'lock'|'cfg'|'frame', margin_deg} 또는 {q: None, why} (poses 가 비었거나 못 가림). 위 우선순위."""
    if not poses:
        return {"q": None, "why": "마커 자세를 못 구함"}
    rows = rot_costs(np, poses)
    margin = rows[1][0] - rows[0][0]
    sure = margin >= ROT_MARGIN_DEG
    lk = rot_locked()
    if lk is not None and lk[1] == "paper":
        r = next(r for r in rows if r[1] == lk[0])
        return {"q": lk[0], "ch": r[2], "src": "lock", "lock_src": lk[1], "margin_deg": round(margin, 1)}
    if q_cfg is not None:
        if sure and rows[0][1] != q_cfg:
            return {"q": None, "margin_deg": round(margin, 1),
                    "why": f"큐브 마커가 사진 방향 {rows[0][1] * 90}° 로 보임 (설정 = 폰 뎁스 camera_to_robot 은 {q_cfg * 90}°) — "
                           "폰을 돌려 끼웠으면 camera_to_robot 을 고치거나 기준 종이로 책상 높이를 한 번 재기"}
        r = next(r for r in rows if r[1] == q_cfg)
        return {"q": q_cfg, "ch": r[2], "src": "cfg", "margin_deg": round(margin, 1)}
    if sure:
        rot_vote(rows[0][1])
    lk = rot_locked()
    if lk is not None:
        r = next(r for r in rows if r[1] == lk[0])
        return {"q": lk[0], "ch": r[2], "src": "lock", "lock_src": lk[1], "margin_deg": round(margin, 1)}
    if sure:
        return {"q": rows[0][1], "ch": rows[0][2], "src": "frame", "margin_deg": round(margin, 1)}
    return {"q": None, "margin_deg": round(margin, 1),
            "why": f"사진이 0° 인지 180° 인지 큐브 마커로 못 가림 (두 방향 차 {margin:.0f}° < {ROT_MARGIN_DEG:.0f}°, 마커가 화면 가운데쯤이면 그렇다) "
                   "— 기준 종이로 책상 높이를 한 번 재거나 (사진 방향이 잠긴다), 폰 뎁스 설정 camera_to_robot 을 거치 자세로 두기"}


def pick_rot(np, poses, q_cfg=None):
    """choose_rot 의 옛 모양: (q, {id: (R, t, 각°)}) 또는 None (못 가리면 None — 예전처럼 아무 q 나 고르지 않는다)."""
    r = choose_rot(np, poses, q_cfg)
    return None if r["q"] is None else (r["q"], r["ch"])


def cad_camera(cv2, np, found, intr, w, h, td, ids, q_cfg=None, c2r=None):
    """기준 종이 없이 큐브 위치를 구할 때 쓰는 카메라 (phone_detect 의 _cameras 와 같은 모양 (K, R 세상→카메라, 자리 C, 0.0)).
    세상 = 로봇 좌표를 책상 면이 z 0 이 되게 올린 것 (C z = 카메라의 책상 위 높이). found = {id: 사진 꼭짓점}.
    → {cam, und {id: 왜곡 푼 꼭짓점}, q, rot_src} 또는 None (보정값·책상 높이가 없음) 또는 {why} (보정값이 이 프레임 것이 아님 · 사진 회전을 못 가림)."""
    if not intr or td is None:
        return None
    sk = scaled_K(np, intr, w, h)
    if sk is None:
        return {"why": intr_mismatch(intr, w, h)}
    K, dist = sk
    und = {i: undistort(cv2, np, c, K, dist) for i, c in found.items() if i in ids}
    pr = choose_rot(np, marker_poses(cv2, np, und, K, PD.CUBE_MARKER_MM / 1000.0), q_cfg)
    if pr["q"] is None:
        return {"why": pr["why"]}
    q = pr["q"]
    R = r_opt_robot(np, q).T
    C = np.array([CAM_POS[0], CAM_POS[1], CAM_POS[2] + float(td)])
    src = pr["src"]
    if c2r is not None and rot_from_c2r(c2r) == q:
        # 폰 뎁스 설정의 거치 자세(camera_to_robot)를 그대로 — 뎁스와 같은 거치 자세를 쓴다 (2026-11-25: 예전엔 여기만 45° 딱 맞춘 CAD 회전·자리라
        # 뎁스(45.1°)·실제(IMU 46.3°)와 달라 ArUco·뎁스가 몇 cm 어긋났다). 자동 보정(phone_api.autocal)이 이 값을 IMU 로 고친다.
        import phone_depth as DP
        Ro = np.array([DP.camera_to_robot(e, (0.0, 0.0, 0.0) + tuple(float(v) for v in c2r[3:6])) for e in np.eye(3)]).T
        R = Ro.T
        C = np.array([float(c2r[0]), float(c2r[1]), float(c2r[2]) + float(td)])
        src = f"{src} · 거치 자세 = 폰 뎁스 설정"
    return {"cam": (K, R, C, 0.0), "und": und, "q": q, "rot_src": src}


def measure_cubes(image, intr, cube_h=None, q_cfg=None):
    """사진 (거치대의 폰) 의 큐브 윗면 마커 (4x4 ID 0~4, 40 mm) → dict(ok, d, n, ids, d_pm_mm, tilt_deg, d_normal, warn, annotated …).
    큐브가 쌓여 있으면 마커 높이 차이로 층을 나눠 뺀다. 여러 개면 중앙값."""
    import cv2
    import numpy as np
    cube_h = float(PD.DEFAULT_CFG["cube_h"] if cube_h is None else cube_h)
    out = {"ok": False, "ids": [], "warn": []}
    img = PD.as_image(image)
    if img is None:
        return {**out, "reason": "사진을 못 읽음 (JPEG·PNG 만)"}
    k = PD.MAX_SIDE / max(img.shape[:2])
    if k < 1:
        img = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
    h, w = img.shape[:2]
    prm = cv2.aruco.DetectorParameters_create()
    prm.minMarkerDistanceRate = 0.02                # detect_cubes 와 같은 이유 (흰 네모와 마커가 둘 다 남게)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, raw = PD.split_markers(cv2, np, gray, prm, set(PD.REF_MM), set(PD.CUBE_IDS))   # 큐브 (4x4) 만, 기준 (5x5) 와 겹친 것은 뺀다
    found = {}
    for i, c in raw.items():
        # 꼭짓점 다듬기 창 = 마커 칸 하나 (한 변 / (CUBE_BITS + 2) = 4x4 는 6). 기본 창 (5 px) 은 20 px 마커에서 꼭짓점을 안쪽으로 끌어 마커가 3 % 작게
        # → 3 % 멀게 → d 가 7 mm 크게 나왔다 (합성 영상, 5x5 때). 칸 크기 창이면 0.2 % 안
        c = c.astype(np.float32)
        win = int(max(2, min(5, round(float(np.mean([np.linalg.norm(c[j] - c[(j + 1) % 4]) for j in range(4)])) / (PD.CUBE_BITS + 2)))))
        cv2.cornerSubPix(gray, c, (win, win), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        found[i] = c.astype(np.float64)
    vis = img.copy()
    lw = max(2, w // 400)
    for i, c in found.items():
        cv2.polylines(vis, [c.astype(np.int32)], True, (0, 140, 255), lw)
    out.update(ids=sorted(found), annotated=vis, size=[w, h])
    if not found:
        return {**out, "reason": f"큐브 윗면 마커 (4x4 ID {PD.CUBE_IDS[0]}~{PD.CUBE_IDS[-1]}) 가 안 보임"}
    if not intr:
        return {**out, "reason": "폰 보정값 (초점거리) 없음 — 보정 파일이 없거나 표에 없는 폰 모델 (phone_api.CALIB_MODELS) 이라 큐브 마커로는 못 잼. 기준 종이나 뎁스로"}
    sk = scaled_K(np, intr, w, h)
    if sk is None:
        return {**out, "reason": intr_mismatch(intr, w, h) + " — 기준 종이나 뎁스로"}
    K, dist = sk
    und = {i: undistort(cv2, np, c, K, dist) for i, c in found.items()}
    poses = marker_poses(cv2, np, und, K, PD.CUBE_MARKER_MM / 1000.0)
    if not poses:
        return {**out, "reason": "큐브 마커로 자세를 못 구함 — 마커가 더 크게·또렷하게 나오게"}
    pr = choose_rot(np, poses, q_cfg)
    if pr["q"] is None:
        return {**out, "reason": pr["why"]}
    q, ch = pr["q"], pr["ch"]
    out["image_rot_src"] = pr["src"]
    up = r_opt_robot(np, q)[2]
    rows = []
    for i, (R, t, ang) in ch.items():
        side = float(np.mean([np.linalg.norm(und[i][j] - und[i][(j + 1) % 4]) for j in range(4)]))
        rows.append({"id": i, "z": float(up @ t) + CAM_POS[2], "h": -float(R[:, 2] @ t), "tilt": ang, "px": side, "dist": float(np.linalg.norm(t))})
    # 쌓인 큐브: 낮은 쪽 마커들 (가장 낮은 것에서 반 층 안) 의 중앙값에서 cube_h 몇 개 위인가. 층 사이에 어중간하면 (0.3 층 넘게) 뺀다
    zs = sorted(r["z"] for r in rows)
    z0 = float(np.median([z for z in zs if z - zs[0] < cube_h / 2]))
    drop = []
    for r in rows:
        x = (r["z"] - z0) / cube_h
        r["level"] = int(max(0, min(PD.MAX_LEVELS - 1, round(x))))
        r["d"] = (r["level"] + 1) * cube_h - r["z"]
        r["d_normal"] = r["h"] + (r["level"] + 1) * cube_h - CAM_POS[2]
        if abs(x - round(x)) > 0.3:
            drop.append(r)
    if drop:
        out["warn"].append("큐브 " + ", ".join(str(r["id"]) for r in drop) + " 는 몇 층인지 어중간해 뺌 (기울었거나 흐림)")
        rows = [r for r in rows if r not in drop] or rows
    ds = np.array([r["d"] for r in rows])
    d = float(np.median(ds))
    px = float(np.mean([r["px"] for r in rows]))
    # 흔들림 추정: 꼭짓점 0.3 px 잡음 → 거리 비율 0.3/한 변, 수직으로는 sin 45°. 여러 개면 퍼짐과 비교해 큰 쪽
    one = float(np.mean([r["dist"] * 0.3 / max(r["px"], 1.0) * _S for r in rows])) / math.sqrt(len(rows))
    spread = float(ds.max() - ds.min()) if len(rows) > 1 else 0.0
    pm = max(one, spread / 2) * 1000
    tilt = float(np.median([r["tilt"] for r in rows]))
    out.update(ok=True, d=round(d, 4), n=len(rows), d_pm_mm=round(pm, 1), spread_mm=round(spread * 1000, 1), tilt_deg=round(tilt, 1),
               image_rot_deg=q * 90, d_normal=round(float(np.median([r["d_normal"] for r in rows])), 4),
               cam_above_table=round(d + CAM_POS[2], 4), marker_px=round(px, 1), f_px=round(float(K[0, 0]), 1), calib=intr.get("file"),
               cubes=[{"id": r["id"], "d": round(r["d"], 4), "level": r["level"], "tilt_deg": round(r["tilt"], 1), "px": round(r["px"], 1)} for r in rows])
    if tilt > CUBE_TILT_WARN_DEG:
        out["warn"].append(f"큐브 마커가 CAD 거치 자세보다 {tilt:.0f}° 기울어 보임 — 폰이 거치대에 제대로 안 끼워졌거나 큐브가 기울었다")
    if px < 20:
        out["warn"].append(f"큐브 마커가 작게 보임 (한 변 {px:.0f} px) — 거리 오차가 커진다 (큐브를 폰 가까이)")
    if spread > 0.012:
        out["warn"].append(f"큐브마다 책상 높이가 {spread * 1000:.0f} mm 까지 다름 — 큐브가 기울었거나 마커가 흐리다")
    for r in rows:
        c = und[r["id"]]
        cv2.putText(vis, f"{r['id']}" + (f" +{r['level']}" if r["level"] else ""), tuple(int(v) for v in found[r["id"]][0]),
                    cv2.FONT_HERSHEY_SIMPLEX, w / 900, (0, 140, 255), lw)
    cv2.putText(vis, f"d {d:.3f} m  cubes {len(rows)}", (lw * 6, int(h - lw * 8)), cv2.FONT_HERSHEY_SIMPLEX, w / 900, (0, 140, 255), lw)
    return out


# ---- 뎁스: 폰 스테레오 프로그램의 rgbd.npz ------------------------------------------------------------------------------

def fit_plane(points, tol=0.004, iters=300, seed=0):
    """(N, 3) 점 (광학 좌표 m) 에서 가장 큰 평면 (RANSAC 뒤 인라이어 최소제곱) → (법선, 평면 위 점, 인라이어 비율, 잔차 rms m) 또는 None."""
    import numpy as np
    p = np.asarray(points, dtype=np.float64)
    if len(p) < 200:
        return None
    rng = np.random.default_rng(seed)
    best, best_n = None, 0
    for _ in range(iters):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        k = np.linalg.norm(n)
        if k < 1e-9:
            continue
        n /= k
        cnt = int((np.abs((p - a) @ n) < tol).sum())
        if cnt > best_n:
            best, best_n = (n, a), cnt
    if best is None or best_n < 0.3 * len(p):
        return None
    n, a = best
    for _ in range(2):
        inl = p[np.abs((p - a) @ n) < tol]
        a = inl.mean(axis=0)
        n = np.linalg.svd(inl - a, full_matrices=False)[2][2]
    res = (p - a) @ n
    inl = np.abs(res) < tol
    return n, a, float(inl.mean()), float(np.sqrt(np.mean(res[inl] ** 2)))


def measure_depth(run_dir, now, max_age_s=DEPTH_MAX_AGE_S):
    """<run_dir>/rgbd/rgbd.npz (points_left_m = 왼쪽 렌즈 광학 좌표 점, valid) → dict(ok, d, tilt_deg, inlier, rms_mm, age_s, warn) 또는 ok False + reason."""
    import numpy as np
    path = os.path.join(run_dir, "rgbd", "rgbd.npz")
    out = {"ok": False, "warn": [], "file": path, "age_s": None}
    try:
        age = now - os.stat(path).st_mtime
    except OSError:
        return {**out, "missing": True, "reason": "뎁스 맵 없음 — 준비의 '뎁스 맵 저장' 을 켜고 폰 프로그램을 다시 켜기 (터미널이면 --rgbd-viewer)"}
    out["age_s"] = round(age, 1)
    if age > max_age_s:
        return {**out, "stale": True, "reason": f"뎁스 맵이 오래됨 ({age:.0f}초 전) — 폰 프로그램이 꺼졌거나 멈춤"}
    try:
        with np.load(path) as z:
            pts, valid = z["points_left_m"], z["valid"]
    except Exception as e:      # 쓰는 중이면 깨진 zip
        return {**out, "reason": f"뎁스 맵을 못 읽음 (쓰는 중일 수 있음 — 다시 누르기): {type(e).__name__}"}
    ok = valid.astype(bool) & np.isfinite(pts).all(axis=2) & (pts[..., 2] >= DEPTH_Z_RANGE[0]) & (pts[..., 2] <= DEPTH_Z_RANGE[1])
    p = pts[ok]
    if len(p) > 20000:
        p = p[:: len(p) // 20000 + 1]
    fit = fit_plane(p)
    if fit is None:
        return {**out, "reason": f"책상 평면을 못 찾음 (쓸 수 있는 점 {len(p)}개) — 책상이 화면의 큰 부분을 차지해야 한다"}
    n, a, frac, rms = fit
    out.update(inlier=round(frac, 2), rms_mm=round(rms * 1000, 1))
    try:
        st = json.loads(open(os.path.join(run_dir, "rgbd", "frame.json")).read()).get("scale_status")
        if st != "metric":
            out["warn"].append(f"폰 스테레오의 거리 눈금이 확인 안 됨 (scale_status {st})")
    except (OSError, ValueError, AttributeError):
        pass
    return _finish(out, plane_to_d(n, a))


# ---- 둘을 한 줄로 -------------------------------------------------------------------------------------------------------

NAME = {"aruco": "기준 종이", "cubes": "큐브 마커", "depth": "뎁스"}


CUBE_CONFIRM_M = 0.02    # 큐브 마커 값은 지금 d 칸·기준 종이·뎁스 중 하나와 이 안이어야 넣는다 (d_normal 과도 이 안)


def combine(aruco, depth, cubes=None, d_now=None):
    """세 결과 (없으면 None) → 화면에 줄 dict: ok, d, source 'aruco'|'cubes'|'depth' (기준 종이 → 큐브 마커 → 뎁스 순으로 먼저),
    in_range, fill_ok ('넣기' 버튼), diffs {source: mm}, diff_mm (첫 다른 것과), text, warn, reason.
    d_now = 화면의 지금 d 칸. 큐브 마커 값으로 넣을 때는 (2026-11-24 검토) 보이는 가장 낮은 마커를 1층으로 보므로 책상에 누운 마커면 +5 cm,
    탑만 보이면 −5 cm × (층 − 1) 틀려도 범위 안이다 → d 칸·기준 종이·뎁스 중 어느 것과도 2 cm 넘게 다르거나 마커 법선 값 (d_normal) 과
    2 cm 넘게 다르면 넣지 않는다 (fill_ok False, reason)."""
    got = [(k, r) for k, r in (("aruco", aruco), ("cubes", cubes), ("depth", depth)) if r and r.get("ok")]
    out = {"ok": bool(got), "aruco": aruco, "cubes": cubes, "depth": depth, "range": list(D_RANGE), "warn": []}
    if not got:
        out["reason"] = next((r.get("reason") for r in (cubes, aruco, depth) if r and r.get("reason")), None) or "잰 값 없음"
        return out
    src, main = got[0]
    d = main["d"]
    label = NAME[src] + (f" {main['n']}개" if src == "cubes" else "")
    text = f"책상 높이 d = {d:.3f} m ({label})"
    out["diffs"] = {}
    for k, r in got[1:]:
        mm = round(abs(r["d"] - d) * 1000, 1)
        out["diffs"][k] = mm
        text += f" · {NAME[k]} {r['d']:.3f} m · 차이 {mm:.0f} mm"
        if mm > 10:
            out["warn"].append(f"{NAME[src]} 와 {NAME[k]} 가 1 cm 넘게 다름 — 줄자로 확인")
    if out["diffs"]:
        out["diff_mm"] = next(iter(out["diffs"].values()))
    out.update(d=d, source=src, text=text, tilt_deg=main["tilt_deg"], in_range=D_RANGE[0] - 1e-9 <= d <= D_RANGE[1] + 1e-9)
    out["fill_ok"] = out["in_range"]
    if not out["in_range"]:
        out["reason"] = f"계획이 받는 범위 {D_RANGE[0]:g} ~ {D_RANGE[1]:g} m 밖이라 역기구학 탭에 넣지 않음 — 폰이 거치대에 있는지, 큐브·종이가 책상 위인지 확인"
    elif src == "cubes":
        refs = [(NAME[k], r["d"]) for k, r in got[1:]]
        if d_now is not None and math.isfinite(d_now):
            refs.insert(0, ("지금 d 칸", float(d_now)))
        dn = main.get("d_normal")
        why = None
        if dn is not None and abs(dn - d) > CUBE_CONFIRM_M:
            why = (f"큐브 마커의 CAD 자세 값 {d:.3f} m 와 마커 법선 값 {dn:.3f} m 가 {abs(dn - d) * 1000:.0f} mm 다름 "
                   f"(허용 {CUBE_CONFIRM_M * 1000:.0f}) — 거치 각이 CAD 와 다르거나 마커가 기울었다")
        elif not refs or min(abs(v - d) for _, v in refs) > CUBE_CONFIRM_M:
            why = (f"큐브 마커 값 {d:.3f} m 가 " + (" · ".join(f"{k} {v:.3f}" for k, v in refs) or "비교할 값") +
                   f" 어느 것과도 {CUBE_CONFIRM_M * 100:.0f} cm 안이 아님 — 큐브 마커는 보이는 가장 낮은 마커를 1층으로 본다 "
                   "(책상에 누운 마커면 +5 cm, 탑만 보이면 −5 cm). 기준 종이·뎁스로 재거나 줄자 값을 d 칸에 넣고 다시")
        if why:
            out.update(fill_ok=False, reason=why)
    for _, r in got:
        out["warn"] += r.get("warn", [])
    return out
