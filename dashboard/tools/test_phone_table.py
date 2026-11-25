#!/usr/bin/env python3
"""phone_table (책상 높이 재기) 시험 — 폰 없이: /usr/bin/python3 dashboard/tools/test_phone_table.py [그림 저장 폴더]

  1. CAD 값이 서로 맞는지: R_CAD = 축 대응 × CAMERA.md R_OPT, 어깨 피치축 (정우 카메라 좌표 (−0.032, 0, 0.007)) 의 우리 좌표,
     phone_depth.DEFAULT_C2R (팀 camera.yaml) 과 같은 위치·(뒤집힘 빼고) 2° 안의 회전, CAD 책상 0.4655 m 아래 → d 0.4775
  2. ArUco: 기준 종이를 높이 d 인 책상에 놓고 CAD 거치 자세 (test_phone_detect 의 render) 로 그린 사진 → 잰 d 가 4 mm 안 (초점거리를 사진에서 추정해서 생기는 흔들림: 15장 평균 1.1·최대 4.1 mm), 기울기 0.5° 안.
     뒤집힌 사진 (180°), 종이가 화면 가운데가 아닐 때, 거치 각이 48° 일 때 (d 는 그대로·기울기 경고), 범위 밖 (넣지 않음)
  3. 뎁스: CAD 자세에서 본 책상 + 큐브 + 잡음 점으로 만든 rgbd.npz → d 3 mm 안. 오래된 파일·없는 파일
"""
import math
import os
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import phone_depth as DP             # noqa: E402
import phone_detect as PD            # noqa: E402
import phone_table as PT             # noqa: E402
import test_phone_detect as TD       # noqa: E402

bad = []
SAVE = Path(sys.argv[1]) if len(sys.argv) > 1 else None


def check(name, ok, detail=""):
    print(("PASS  " if ok else "FAIL  ") + name + (f": {detail}" if detail != "" else ""))
    if not ok:
        bad.append(name)


# ---- 1. CAD 값 ----
RC, RO = np.array(PT.R_CAD), np.array(PT.R_OPT)
AX = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])          # 정우 카메라 좌표 (x 뒤, y 오른쪽, z 위) → 우리 (x 왼쪽, y 뒤, z 위)
check("R_CAD = 축 대응 × R_OPT, 회전 행렬", np.allclose(RC, AX @ RO, atol=1e-9) and abs(np.linalg.det(RC) - 1) < 1e-9)
check("시선 = 앞(−y)·아래 45°, 영상 오른쪽 = 로봇 오른쪽(−x)", np.allclose(RC @ [0, 0, 1], [0, -PT._S, -PT._S]) and np.allclose(RC @ [1, 0, 0], [-1, 0, 0]))
pitch_axis = AX @ np.array([-0.032, 0, 0.007]) + PT.CAM_POS
check("어깨 피치축 (CAMERA.md) = 우리 좌표 (0.174, −0.1497, −0.005)", np.allclose(pitch_axis, [0.174, -0.1497, -0.005], atol=1e-6), pitch_axis.round(4).tolist())
check("위치 = phone_depth.DEFAULT_C2R 앞 3개", np.allclose(PT.CAM_POS, DP.DEFAULT_C2R[:3]))
R_team = np.array([DP.camera_to_robot(e, (0, 0, 0) + tuple(DP.DEFAULT_C2R[3:])) for e in np.eye(3)]).T
E = R_team @ (RC @ np.diag([-1, -1, 1.0])).T
ang = math.degrees(math.acos((np.trace(E) - 1) / 2))
check("회전 = DEFAULT_C2R (뒤집힘 180° 빼고) 2° 안", ang < 2.0, f"{ang:.2f}°")
pl = PT.plane_to_d(RC.T @ [0, 0, 1.0], RC.T @ [0.1, -0.3, -0.4655])
check("CAD 책상 (카메라 0.4655 m 아래) → d 0.4775, 기울기 0", abs(pl["d"] - 0.4775) < 1e-9 and pl["tilt_deg"] < 1e-6 and abs(pl["d_cad_rot"] - 0.4775) < 1e-9)


# ---- 2. ArUco ----
def photo(d, pitch=45.0, sheet=(0.0, 0.0, 0.0), f=1250, rot180=False, roll_deg=0.0):
    """높이 d 인 책상의 사진. 종이 = 시선이 책상과 만나는 점에서 sheet (옆 m, 앞뒤 m, 돌아간 각°) 만큼 옮긴 자리. 우리 로봇 좌표로 그린다 (책상 z = 0)."""
    h = d + PT.CAM_POS[2]                                      # 카메라의 책상 위 높이
    cam = (PT.CAM_POS[0], PT.CAM_POS[1], h)
    hit = (cam[0], cam[1] - h / math.tan(math.radians(pitch)), 0.0)
    cfg = {**PD.DEFAULT_CFG, "x": hit[0] + sheet[0], "y": hit[1] + sheet[1], "rot_deg": sheet[2]}
    img = TD.render(cfg, None, cam, f, cubes=[], target=hit)
    if rot180:
        img = cv2.rotate(img, cv2.ROTATE_180)
    return cv2.imdecode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1], cv2.IMREAD_COLOR)   # 프레임 배열 (JPEG 로 한 번 뭉갠 것)


CASES = [   # (이름, d, photo 인자, 기울기 한계 °)
    ("d 0.4725", 0.4725, {}, 0.5),
    ("d 0.50", 0.50, {}, 0.5),
    ("d 0.4775, 종이가 옆·앞으로 벗어나고 20° 돌아감", 0.4775, {"sheet": (-0.14, 0.08, 20.0)}, 0.5),
    ("d 0.46, 사진이 180° 뒤집힘, 초점 1500", 0.46, {"rot180": True, "f": 1500}, 0.5),
    ("d 0.4775, 거치 각 48° (CAD 45°)", 0.4775, {"pitch": 48.0}, None),
]
for k, (name, d, kw, tilt_max) in enumerate(CASES):
    buf = photo(d, **kw)
    r = PT.measure_aruco(buf)
    if SAVE:
        SAVE.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(SAVE / f"table_case{k}.jpg"), buf)
        cv2.imwrite(str(SAVE / f"table_case{k}_found.jpg"), r["annotated"])
    if not r["ok"]:
        check(f"ArUco {name}", False, r.get("reason")); continue
    err = (r["d"] - d) * 1000
    good = abs(err) <= 4.0 and (r["tilt_deg"] <= tilt_max if tilt_max is not None else abs(r["tilt_deg"] - 3.0) < 0.5 and any("기울어" in w for w in r["warn"]))
    check(f"ArUco {name}", good, f"d {r['d']:.4f} (오차 {err:+.1f} mm, ± {r['d_pm_mm']} mm) · 기울기 {r['tilt_deg']}° · 사진 회전 {r['image_rot_deg']}° · "
          f"초점 {r['f_px']} (참 {kw.get('f', 1250)}) · 재투영 {r['reproj_px']} px · CAD 회전으로 읽으면 {r['d_cad_rot']:.4f} · 마커 {len(r['refs'])}개")
c = PT.combine(PT.measure_aruco(photo(0.62)), None)   # 받는 범위 0.44 ~ 0.60 (2026-11-25 넓힘, 실물 책상 0.557)
check("범위 밖 (d 0.62) 은 값은 주고 넣지 않음", c["ok"] and not c["in_range"] and abs(c["d"] - 0.62) < 0.006 and "범위" in c.get("reason", ""), f"d {c.get('d')}")
r = PT.measure_aruco(np.full((600, 800, 3), 128, np.uint8))
check("마커 없는 사진 → 이유", not r["ok"] and "마커" in r["reason"])


# ---- 2b. 큐브 윗면 마커 (기준 종이 없이) — 2026-11-24 검토: 사진 회전 0°/180° 거울, 가장 낮은 마커 = 1층 가정, 보정 비율 ----
def cube_photo(d, cubes, f=1250, rot180=False):
    """높이 d 인 책상 위 큐브 [(ID, 시선이 책상과 만나는 점에서 옆 m, 앞뒤 m, yaw°, 층)] 만 (종이는 화면 밖) 그린 CAD 자세 사진."""
    h = d + PT.CAM_POS[2]
    cam = (PT.CAM_POS[0], PT.CAM_POS[1], h)
    hit = (cam[0], cam[1] - h, 0.0)
    cfg = {**PD.DEFAULT_CFG, "x": 5.0, "y": 5.0}
    img = TD.render(cfg, None, cam, f, cubes=[(i, hit[0] + dx, hit[1] + dy, yaw, lv) for i, dx, dy, yaw, lv in cubes], target=hit)
    if rot180:
        img = cv2.rotate(img, cv2.ROTATE_180)
    return cv2.imdecode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1], cv2.IMREAD_COLOR), hit


INTR = {"K": [[1250.0, 0, TD.W / 2], [0, 1250.0, TD.H / 2], [0, 0, 1]], "dist": [], "size": [TD.W, TD.H], "file": "test"}
SPREAD = [(0, -0.12, 0.06, 10.0, 0), (1, 0.10, 0.05, -20.0, 0), (2, 0.0, -0.08, 5.0, 0)]
check("사진 회전 설정: 기본 camera_to_robot (폰 뒤집힘) → 180°", PT.rot_from_c2r(DP.DEFAULT_C2R) == 2 and PT.rot_from_c2r([0, 0, 0, 0, 0, 0]) is None)
for rot in (False, True):
    PT.rot_reset()
    img, _ = cube_photo(0.4775, SPREAD, rot180=rot)
    r = PT.measure_cubes(img, INTR, q_cfg=2 if rot else 0)
    c = PT.combine(None, None, r, 0.4775) if r["ok"] else {}
    check(f"큐브 마커 3개 퍼짐 (사진 {'180°' if rot else '0°'}, 설정 맞음) → d 5 mm 안, 넣기 됨", r["ok"] and abs(r["d"] - 0.4775) <= 0.005 and c.get("fill_ok"),
          f"d {r.get('d')} 사진 회전 {r.get('image_rot_deg')} ({r.get('image_rot_src')}) · {r.get('reason') or ''}")
    r = PT.measure_cubes(img, INTR)
    check("…설정·잠금 없음 → 틀린 값 안 줌", (not r["ok"] and "180" in r["reason"]) or (r["ok"] and abs(r["d"] - 0.4775) <= 0.005), r.get("reason") or f"d {r.get('d')}")
PT.rot_reset()
img, _ = cube_photo(0.4775, [(0, 0.0, 0.0, 0.0, 0)])
r = PT.measure_cubes(img, INTR)
check("화면 가운데 마커 하나, 사진 회전 안 잠김 → 틀린 값 안 줌 (못 가리면 거절)", (not r["ok"] and "180" in r["reason"]) or (r["ok"] and abs(r["d"] - 0.4775) <= 0.006),
      r.get("reason") or f"d {r.get('d')}")
PT.measure_aruco(photo(0.4775))                 # 기준 종이로 한 번 잼 → 사진 회전 잠김
lk = PT.rot_locked()
r = PT.measure_cubes(img, INTR)
check("기준 종이로 잠근 뒤 → 가운데 마커 하나도 잼 (잠긴 회전)", lk is not None and lk[1] == "paper" and r["ok"] and abs(r["d"] - 0.4775) <= 0.006 and r["image_rot_src"] == "lock",
      f"잠금 {lk} · d {r.get('d')} · {r.get('reason') or ''}")
img, _ = cube_photo(0.4775, [(0, -0.10, 0.04, 0.0, 1), (1, -0.10, 0.04, 0.0, 0)])   # 2층 탑만 (아래 큐브는 가려짐)
r = PT.measure_cubes(img, INTR)
c = PT.combine(None, None, r, 0.4775)
check("탑만 보임 (2층) → −5 cm 라 넣기 막음", r["ok"] and not c["fill_ok"], f"d {r.get('d')} · {c.get('reason')}")
img, _ = cube_photo(0.4775, [(0, -0.10, 0.04, 0.0, -1), (1, 0.08, 0.03, 0.0, -1)])   # 큐브 없이 마커만 책상에 누움 → +5 cm
r = PT.measure_cubes(img, INTR)
c = PT.combine(None, None, r, 0.4775)
check("책상에 누운 마커 → 값 +5 cm 라 d 칸과 2 cm 넘게 달라 넣기 막음", r["ok"] and abs(r["d"] - 0.5275) < 0.008 and c["in_range"] and not c["fill_ok"] and "1층" in c["reason"],
      f"d {r.get('d')} · {c.get('reason')}")
c = PT.combine(None, {"ok": True, "d": r["d"] + 0.005, "tilt_deg": 0.0, "warn": []}, r, 0.4775)
check("…뎁스가 큐브 마커 값과 2 cm 안이면 넣기 됨", c["fill_ok"] and c["source"] == "cubes", c.get("reason"))
c = PT.combine(None, None, {**r, "d_normal": r["d"] + 0.03}, r["d"])
check("마커 법선 값과 3 cm 다름 → 넣기 막음", not c["fill_ok"] and "법선" in c["reason"], c.get("reason"))
r = PT.measure_cubes(cube_photo(0.4775, SPREAD)[0], {**INTR, "size": [TD.W, TD.W]})
check("보정 image_size 비율이 프레임과 다름 → 보정값 안 씀 (거절)", not r["ok"] and "가로세로" in r["reason"], r.get("reason"))
for td, want_unsure in ((0.4775, False), (0.4775 + 0.025, True)):
    img, hit = cube_photo(0.4775, [(0, -0.10, 0.04, 0.0, 0), (1, -0.10, 0.04, 0.0, 1), (2, 0.08, 0.0, 0.0, 0)])
    m = PD.detect_cubes(img, {**PD.DEFAULT_CFG, "x": 5.0, "y": 5.0, "intr": INTR, "td": td, "image_rot_q": 0})
    lv = {c["id"]: c["level"] for c in m.get("cubes", [])}
    good = m["ok"] and m["camera"]["src"] == "cad" and bool(m.get("level_unsure")) == want_unsure and (want_unsure or lv == {1: 1, 2: 0})
    check(f"종이 없음 (CAD) 2층 탑, 책상 높이 칸 {'맞음' if not want_unsure else '+2.5 cm (반 층) 틀림'} → 층 의심 {want_unsure}", good,
          f"층 {lv} · level_unsure {m.get('level_unsure')} · unsure {[c['id'] for c in m.get('cubes', []) if c.get('unsure')]} · {m.get('reason') or ''}")
PT.rot_reset()


# ---- 3. 뎁스 ----
def cloud(d, pitch=45.0, W=320, H=240, f=250.0, noise=0.0015, flip=True, seed=1):
    """CAD 자세의 카메라가 본 책상 (z = −d) + 5 cm 큐브 2개 + 잡음 → (H, W, 3) 광학 좌표 점, valid. flip = 폰이 뒤집혀 끼워짐 (광축 둘레 180°)."""
    rng = np.random.default_rng(seed)
    p_ = math.radians(pitch)
    R = np.array([[-1, 0, 0], [0, math.sin(p_), -math.cos(p_)], [0, -math.cos(p_), -math.sin(p_)]])   # 광학 → 로봇 (45° 면 R_CAD)
    if flip:
        R = R @ np.diag([-1, -1, 1.0])
    u, v = np.meshgrid(np.arange(W) - W / 2, np.arange(H) - H / 2)
    ray = np.stack([u / f, v / f, np.ones_like(u)], axis=-1) @ R.T           # 로봇 좌표의 시선
    h = d + PT.CAM_POS[2]
    s = -h / ray[..., 2]
    hitp = ray * s[..., None]                                                # 카메라 기준 로봇 축
    for cx, cy in ((0.05, -0.45), (-0.10, -0.55)):                           # 큐브 윗면 (책상 위 5 cm)
        top = ray * ((-h + 0.05) / ray[..., 2])[..., None]
        m = (np.abs(top[..., 0] - cx) < 0.025) & (np.abs(top[..., 1] - cy) < 0.025)
        s = np.where(m, (-h + 0.05) / ray[..., 2], s)
    s = s * (1 + rng.normal(0, noise, s.shape) / np.maximum(s, 0.1) * s)     # 거리 잡음 (멀수록 큼)
    pts = (ray * s[..., None]) @ R                                           # 로봇 축 → 광학 좌표 (R^T · p)
    valid = (ray[..., 2] < -0.05) & (rng.random(s.shape) > 0.1)
    return pts.astype(np.float32), valid


with tempfile.TemporaryDirectory() as tmp:
    os.makedirs(os.path.join(tmp, "rgbd"))
    f_npz = os.path.join(tmp, "rgbd", "rgbd.npz")
    r = PT.measure_depth(tmp, time.time())
    check("뎁스 맵 없음 → 이유", not r["ok"] and r.get("missing") and "뎁스 맵 저장" in r["reason"])
    for d, pitch in ((0.4725, 45.0), (0.50, 45.0), (0.4775, 47.5)):
        pts, valid = cloud(d, pitch)
        np.savez_compressed(f_npz, points_left_m=pts, valid=valid)
        r = PT.measure_depth(tmp, time.time())
        err = (r.get("d", 9) - d) * 1000
        good = r["ok"] and abs(err) <= 3.0 and abs(r["tilt_deg"] - abs(pitch - 45)) < 0.5 and r["image_rot_deg"] == 180
        check(f"뎁스 d {d}, 거치 각 {pitch}°", good, f"d {r.get('d')} (오차 {err:+.1f} mm) · 기울기 {r.get('tilt_deg')}° · 인라이어 {r.get('inlier')} · 잔차 {r.get('rms_mm')} mm"
              if r["ok"] else r.get("reason"))
    a = PT.measure_aruco(photo(0.4775))
    c = PT.combine(a, r)
    check("둘 다 → 한 줄 (ArUco 먼저, 차이)", c["ok"] and c["source"] == "aruco" and "뎁스" in c["text"] and c["diff_mm"] <= 5, c["text"])
    r = PT.measure_depth(tmp, time.time() + 60)
    check("오래된 뎁스 맵 → 안 씀", not r["ok"] and r.get("stale"))

print("실패: " + ", ".join(bad) if bad else "모두 통과")
sys.exit(1 if bad else 0)
