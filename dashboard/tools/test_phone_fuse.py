#!/usr/bin/env python3
"""phone_fuse.py 시험 (폰·GPU 없이): python3 dashboard/tools/test_phone_fuse.py

가짜 프레임 (test_phone_detect.render, 배열 그대로) → phone_detect.detect, 가짜 inference.json (로봇 좌표의 큐브를 카메라 좌표로 거꾸로 옮김, 윗면이 보임)
→ phone_depth.evaluate, 둘을 phone_fuse.fuse 로:
  ArUco 만 / 뎁스만 / 둘 다 맞음 → ArUco 를 씀 / 둘이 다름 (자리 4 cm, 각 30°) → 막음 / 뎁스 높이 검사 실패 → 막음 / 둘 다 없음 → 안내
"""
import json
import os
import sys
import tempfile
import time
from math import cos, hypot, radians, sin
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import phone_depth as DP             # noqa: E402
import phone_detect as PD            # noqa: E402
import phone_fuse as PF              # noqa: E402
from test_phone_detect import render  # noqa: E402

C2R, D = DP.DEFAULT_C2R, 0.4775
bad = []


def check(name, ok, detail=""):
    print(("PASS  " if ok else "FAIL  ") + name + (f": {detail}" if detail != "" else ""))
    if not ok:
        bad.append(name)


def to_cam(v, t=(0, 0, 0)):
    """로봇 좌표 → 카메라 좌표: Rᵀ · (v − t)."""
    cols = [DP.camera_to_robot(e, (0, 0, 0) + tuple(C2R[3:])) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    return np.array([sum(c[i] * (v[i] - t[i]) for i in range(3)) for c in cols])


def top_face(x, y, yaw_deg, z_top=None):
    """로봇 (x, y) 에 yaw 로 놓인 5 cm 큐브의 윗면을 거치 카메라가 본 객체 (test_phone_depth.recorded 의 윗면과 같은 식)."""
    n = to_cam((0, 0, 1.0))
    cam = to_cam((x, y, -D + 0.05 if z_top is None else z_top), C2R[:3])
    e = to_cam((cos(radians(yaw_deg)), sin(radians(yaw_deg)), 0.0))
    hx = np.cross(n, [0, 1.0, 0]); hx /= np.linalg.norm(hx); vx = np.cross(hx, n)
    return {"instance": 1, "label": "blob", "status": "ok", "depth_rect_m": round(float(cam[2]), 3), "dims_mm": [52.0, 50.5, 2.0],
            "center_cam1_m": [round(float(v), 4) for v in cam], "warnings": [],
            "rotation": {"yaw_deg": 0.0, "pitch_deg": 0.0, "roll_deg": float(np.degrees(np.arctan2(e @ vx, e @ hx))), "normal_cam1": [round(float(v), 4) for v in n]}}


def depth_of(tmp, obj, age=0.0):
    """obj (없으면 파일 없음) 를 inference.json 으로 쓰고 evaluate."""
    cfg = {**DP.load_cfg(os.path.join(tmp, "none.json")), "run_dir": tmp}
    path = os.path.join(tmp, "inference.json")
    if obj is None:
        if os.path.exists(path):
            os.remove(path)
    else:
        with open(path, "w") as f:
            json.dump({"timestamp_ns": 1, "objects": {"seg": [obj]}}, f)
        os.utime(path, (time.time() - age,) * 2)
    return DP.evaluate(cfg, D, time.time())


CUBE = (-0.06, -0.33, 20.0)
frame = render(PD.DEFAULT_CFG, CUBE, (-0.05, -0.34, 0.60), 1150)      # 프레임 = BGR 배열
keep = frame.copy()
aruco = PD.detect(frame, dict(PD.DEFAULT_CFG))
check("가짜 프레임에서 ArUco 가 큐브를 찾음", aruco["ok"], aruco.get("cube") or aruco.get("reason"))
no_marker = PD.detect(np.full((600, 800, 3), 160, np.uint8), dict(PD.DEFAULT_CFG))

with tempfile.TemporaryDirectory() as tmp:
    off = depth_of(tmp, None)
    same = depth_of(tmp, top_face(*CUBE))
    check("가짜 inference.json 에서 뎁스가 큐브를 찾음 (윗면)", same["ok"] and same["yaw_from"] == "top" and same["usable"], same.get("cube") or same.get("reason"))
    moved = depth_of(tmp, top_face(CUBE[0] + 0.04, CUBE[1], CUBE[2]))
    turned = depth_of(tmp, top_face(CUBE[0], CUBE[1], CUBE[2] + 30.0))
    high = depth_of(tmp, top_face(CUBE[0], CUBE[1], CUBE[2], z_top=-D + 0.15))
    stale = depth_of(tmp, top_face(*CUBE), age=10.0)

    # ---- 2026-11-25 '뎁스 버리고 시각화만, ArUco 를 믿자' (PF.DEPTH_ROLE = "view", 기본): 뎁스는 판정·경고에 안 들어감
    check("[뎁스=표시만] 기본값", PF.DEPTH_ROLE == "view" and PF.ARUCO_MAIN is True)
    for name, dep in (("자리 4 cm 다름", moved), ("각 30° 다름", turned), ("뎁스 높이 이상", high), ("둘 다 맞음", same)):
        r = PF.fuse(aruco, dep)
        check(f"[뎁스=표시만] {name} → ArUco 그대로, 안 막음, 뎁스 경고 없음, sources 에 뎁스 없음",
              r["ok"] and not r["blocked"] and r["cube"] == aruco["cube"] and r["used"] == "aruco" and r["sources"] == ["aruco"]
              and not any("뎁스" in w for w in r["warn"]) and r["check"] is not None, r.get("text"))
    r = PF.fuse(None, same)
    check("[뎁스=표시만] 뎁스만 (프레임 없음) → 안 집음", not r["ok"] and r.get("used") is None, r.get("reason"))
    r = PF.fuse(no_marker, same)
    check("[뎁스=표시만] 마커 없음 + 뎁스 → 안 집고 마커 이유", not r["ok"] and r["reason"] == no_marker["reason"], r.get("reason"))
    r = PF.find(None, None, same)
    check("[뎁스=표시만] find(프레임 없음, 뎁스) → 안 집음, depth 결과는 화면용으로 그대로", not r["ok"] and r["aruco"] is None and r["depth"] is same)
    # ---- 'aux' (뎁스 보조: 안 맞으면 경고만)
    PF.DEPTH_ROLE = "aux"
    r = PF.fuse(aruco, moved)
    check("[aux] 자리 4 cm 다름 → 안 막고 경고", r["ok"] and not r["blocked"] and r["cube"] == aruco["cube"] and any("뎁스(보조)" in w for w in r["warn"]), r.get("text"))
    r = PF.fuse(aruco, high)
    check("[aux] 뎁스 높이 이상 → 안 막고 경고", r["ok"] and not r["blocked"] and any("책상 위" in w for w in r["warn"]))
    r = PF.fuse(None, same)
    check("[aux] 뎁스만 → 안 집고 참고 note", not r["ok"] and any("뎁스만으로는 안 집음" in n for n in r["note"]))

    # ---- 예전 동작 (ARUCO_MAIN = False) 도 그대로 되는지
    PF.DEPTH_ROLE, PF.ARUCO_MAIN = "old", False
    r = PF.fuse(aruco, off)
    check("ArUco 만 (뎁스 꺼짐)", r["ok"] and r["used"] == "aruco" and r["sources"] == ["aruco"] and not r["blocked"] and r["cube"] == aruco["cube"]
          and r["text"] == "출처: ArUco 만", r.get("text"))
    r = PF.fuse(aruco, stale)
    check("ArUco 만 (뎁스가 3초 넘게 오래됨 → 안 씀)", r["ok"] and r["sources"] == ["aruco"] and not r["blocked"] and not r["note"], r.get("text"))
    r = PF.fuse(None, same)
    check("뎁스만 (프레임 없음)", r["ok"] and r["used"] == "depth" and r["sources"] == ["depth"] and not r["blocked"] and r["cube"] == same["cube"]
          and r["text"] == "출처: 뎁스 만", r.get("text"))
    r = PF.fuse(no_marker, same)
    check("뎁스만 (프레임에 마커 없음 → 이유를 같이)", r["ok"] and r["used"] == "depth" and any(n.startswith("ArUco:") for n in r["note"]), r["note"])
    r = PF.fuse(aruco, same)
    dp = hypot(aruco["cube"]["x"] - CUBE[0], aruco["cube"]["y"] - CUBE[1]) * 1000
    check("둘 다 맞음 → ArUco 를 씀, 안 막음", r["ok"] and r["used"] == "aruco" and r["sources"] == ["aruco", "depth"] and not r["blocked"]
          and r["cube"] == aruco["cube"] and r["check"]["dpos_mm"] <= 20 and r["check"]["dyaw_deg"] <= 15,
          f"{r.get('text')} · 차이 {r['check']} · ArUco 참값과 {dp:.1f} mm")
    r = PF.fuse(aruco, moved)
    check("자리가 4 cm 다름 → 막음", r["ok"] and r["blocked"] and r["block"] and r["check"]["dpos_mm"] > 20 and "다름" in r["warn"][0], f"{r['check']} · {r['block']}")
    r = PF.fuse(aruco, turned)
    check("각이 30° 다름 → 막음", r["ok"] and r["blocked"] and r["check"]["dpos_mm"] <= 20 and r["check"]["dyaw_deg"] > 15, f"{r['check']} · {r['block']}")
    r = PF.fuse(aruco, {**same, "yaw_from": "mixed", "cube": {**same["cube"], "yaw_deg": same["cube"]["yaw_deg"] + 30}})
    check("뎁스 각을 못 믿을 때 (두 면 섞임) 는 각을 안 봄", r["ok"] and not r["blocked"] and r["check"]["dyaw_deg"] is None, r.get("text"))
    r = PF.fuse(None, high)
    check("뎁스만 + 큐브 높이가 책상 위 0~5 cm 밖 → 막음", r["ok"] and r["blocked"] and "높이" in r["block"], r["warn"][-1][:40])
    r = PF.fuse(aruco, high)
    check("둘 다 + 뎁스 높이 검사 실패 → ArUco 가 있어도 막음", r["ok"] and r["blocked"], r["block"])
    r = PF.fuse(None, off)
    check("둘 다 없음 → 무엇을 할지 안내", not r["ok"] and "폰 프로그램을 켜고" in r["reason"], r["reason"])
    r = PF.fuse(no_marker, off)
    check("프레임에 마커 없음 + 뎁스 꺼짐 → ArUco 이유", not r["ok"] and r["reason"] == no_marker["reason"], r["reason"])
    # find = 프레임 배열 (없으면 None) + 그때의 뎁스 결과 — 프레임을 계속 읽는 루프가 부르는 모양
    r = PF.find(frame, dict(PD.DEFAULT_CFG), same)
    check("find(프레임 배열, 뎁스) → ArUco 를 씀, 프레임은 안 건드림", r["ok"] and r["used"] == "aruco" and r["sources"] == ["aruco", "depth"]
          and r["cube"] == aruco["cube"] and "annotated" in r["aruco"] and np.array_equal(frame, keep), r.get("text"))
    r = PF.find(None, None, same)
    check("find(프레임 없음, 뎁스) → 뎁스만", r["ok"] and r["used"] == "depth" and r["aruco"] is None)
    r = PF.find(frame, dict(PD.DEFAULT_CFG), None)
    check("find(프레임, 뎁스 없음) → ArUco 만", r["ok"] and r["used"] == "aruco" and r["sources"] == ["aruco"])
    PF.DEPTH_ROLE, PF.ARUCO_MAIN = "view", True
    # 프레임 읽기 (phone_depth.read_frame): 폰 프로그램이 저장한 inference.png → 배열 → find
    cfg = {**DP.load_cfg(os.path.join(tmp, "none.json")), "run_dir": tmp}
    check("read_frame: 파일 없음 → None", DP.read_frame(cfg, time.time())[0] is None)
    cv2.imwrite(os.path.join(tmp, "inference.png"), frame)
    got, age, _ = DP.read_frame(cfg, time.time())
    r = PF.find(got, dict(PD.DEFAULT_CFG), depth_of(tmp, top_face(*CUBE)))
    check("read_frame → find: 저장된 프레임 + 뎁스 → ArUco (뎁스는 표시만)", got is not None and age < 3 and r["ok"] and r["sources"] == ["aruco"] and not r["blocked"], r.get("text"))
    check("read_frame: 3초 넘은 프레임 → None", DP.read_frame(cfg, time.time() + 10)[0] is None)
check("각 차이는 90° 마다 같게", PF.yaw_diff(20, 110) == 0 and PF.yaw_diff(-44, 44) == 2 and PF.yaw_diff(170, -170) == 20)

print("\n" + ("전부 통과" if not bad else f"실패 {len(bad)}개: {bad}"))
sys.exit(1 if bad else 0)
