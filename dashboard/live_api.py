"""큐브 실시간 인식 (2026-11-24): 휴대폰 카메라·탑 쌓기·강화학습 탭이 같이 쓰는 '지금 책상 위의 큐브들' 한 곳.

사진 올리기는 없다. 두 모드:

    실물 (직접 CAN / ROS)  서버 스레드(Live)가 거치한 폰의 폰 스테레오 프로그램(SteroDepthSeg z_phone_stereo.py)의 결과 폴더를 20 Hz 로 보고
                           새 프레임이 생길 때마다 인식한다 — 프레임의 ArUco (phone_detect.detect_cubes: 4x4 ID 0~4 + 층 + 기울기;
                           기준 종이가 안 보이면 CAD 거치 자세 + 폰 보정값 + 책상 높이 d 로)
                           + 그때의 뎁스 결과 (phone_depth.evaluate) → phone_fuse 로 합친 '집을 큐브 하나' (휴대폰 탭) 와 큐브 목록.
                           프레임:  <결과 폴더>/live.jpg   — 손대지 않은 왼쪽 렌즈 그림 (그 프로그램에 --live-jpg 를 넣은 판, README) ≥ 5 Hz
                                    <결과 폴더>/inference.png — 없으면 이것 (그 프로그램이 1초쯤마다 저장, 객체 테두리·글자가 덧그려짐,
                                                               오른쪽 뎁스 색 그림은 잘라 냄)
                           마지막 프레임이 STALE_S (1.5 초, inference.png 면 그 저장 간격의 1.6 배까지 늘림 — 최대 3 초) 보다 오래되면
                           '영상 없음' → 집기·탑 쌓기를 막는다.
    가상                   카메라는 아예 안 읽는다. 큐브 = 서버의 가상 장면 (Scene, logs/virtual_scene.json): 화면 지도에서 놓기·끌기·돌리기·지우기.
                           가상 팔 재생이 끝까지 가면 (VirtualArm.play_trajectory 의 on_end) 그 계획이 옮긴 큐브를 놓을 자리로 옮긴다.
                           '놓을 때 어긋남' 을 켜면 ±1 cm · ±5° 를 무작위로 더한다.

    GET  /api/live/cubes?d=<책상 높이>&pick=<ID>   {ok, mode 'camera'|'virtual', stale, reason, age_s, src, rate_hz, stale_s, t_frame, frame_n,
                                                  cubes: [{id, x, y, yaw_deg, level, source, age_s, tilt_deg?, unsure?}], refs, warn,
                                                  table {d, measured}, pick {휴대폰 탭이 집을 큐브 하나 — phone_fuse 모양}, pipeline, scene{jitter}}
    GET  /api/live/frame.jpg                     마지막 프레임 (마커를 그린 것) JPEG
    POST /api/live/scene  {op: add|move|yaw|delete|clear|jitter, …}   가상 장면 고치기 (가상 모드에서만)
    GET  /api/phone/live?d=&pick=                예전 휴대폰 탭 주소 — 위 pick 과 같은 것 (+ fresh, pipeline)

좌표는 전부 대시보드 실좌표 (pick_plan 과 같음): 원점 = 어깨 roll 축, +x = 로봇 왼쪽, +y = 뒤 (책상은 −y). level 0 = 책상 위.
"""
import json
import math
import os
import random
import threading
import time
from collections import deque
from pathlib import Path

from fastapi import Request
from fastapi.responses import JSONResponse, Response, StreamingResponse

import phone_depth as DP

STALE_S = 1.5            # 2026-11-24 사용자: 마지막 프레임이 이보다 오래되면 '영상 없음' (집기·탑 쌓기 막음)
STALE_MAX_S = 3.0        # inference.png (1초쯤 간격) 일 때 늘려 주는 한계
POLL_S = 0.05            # 결과 폴더를 보는 간격
LIVE_JPG_FRESH_S = 3.0   # live.jpg 가 이보다 새것이면 그것을 쓴다 (아니면 inference.png)
DEPTH_POS_TOL = 0.03     # 뎁스 큐브가 ArUco 큐브와 이 안이면 같은 큐브 (source 'aruco+depth')
import phone_detect as PD   # noqa: E402
CUBE_IDS = PD.CUBE_IDS   # 큐브 = 4x4 ID 0~4 (phone_detect 한 곳)
STACK_R = 0.03           # 가상 장면: 이 안(m)이면 같은 자리 (그 위에 쌓임)
JITTER_M, JITTER_DEG = 0.01, 5.0
TD_DEFAULT = 0.5575     # = hand_ik.TABLE_D_DEFAULT (실물 책상, 2026-11-25 프로파일을 8 cm 내림)
TD_MATCH_M = 0.0005      # 계산에 쓴 d 가 화면의 d 와 이보다 다르면 그 결과를 안 준다


def fold(deg):
    return (float(deg) + 45.0) % 90.0 - 45.0


def left_panel(img):
    """inference.png = [객체 그림 (렌즈 하나) | (bbox 그림) | 뎁스 색 그림 + 눈금 70 px] 가로로 붙인 것 → 첫 칸만 (ArUco 는 가운데를 주점으로 본다)."""
    h, w = img.shape[:2]
    for n in (3, 2):     # 3 먼저 (2026-11-25 검토: --yolo both 의 3칸 1990 px 를 2칸으로 보면 960/480 = 2.0 이 통과해 1.5칸을 잘랐다)
        lw = (w - 70) / n
        if lw == int(lw) and 1.0 <= lw / h <= 2.0:
            return img[:, :int(lw)]
    return img


LIVE_MAX_LEVELS = 1     # 큐브 층 후보 수 (phone_detect max_levels). 2026-11-25 탑 쌓기를 없애 1 = 늘 책상 위 — 기준 종이 없이 층을 고르면
                        # 큐브마다 마커 크기 오차로 한 층 틀렸다. 탑 쌓기를 되살리면 phone_detect.MAX_LEVELS 로
_UNRECT = {"key": None, "maps": None, "why": None}


def unrectify(img, calib_path):
    """inference.png 의 왼쪽 칸 = 폰 프로그램이 '정류한' 왼쪽 렌즈 그림 (lib_rectify: stereoRectify, CALIB_ZERO_DISPARITY, alpha 0 —
    초점 P1 ≈ 461 px, 광축 약 4° 돌림, 왜곡 없음). 큐브 인식은 원래 렌즈 보정 (K1 ≈ 312 px · dist1) 과 거치 자세를 쓰므로
    정류 전 모습으로 되돌린다 (2026-11-25: 되돌리지 않아 큐브 마커 40 mm 가 51 mm 로 재지고 층이 3 으로, 뎁스와 9 cm 어긋났다).
    (그림, None) 또는 못 하면 (그대로, 이유). 맵은 보정 파일·크기마다 한 번 만든다."""
    import cv2
    import numpy as np
    try:
        st = os.stat(calib_path)
    except OSError:
        return img, f"보정 파일 없음 ({calib_path})"
    key = (str(calib_path), st.st_mtime_ns, img.shape[:2])
    if _UNRECT["key"] != key:
        maps, why = None, None
        try:
            with open(calib_path) as f:
                c = json.load(f)
            w, h = (int(v) for v in c["image_size"])
            if (img.shape[1], img.shape[0]) != (w, h):
                why = f"그림 {img.shape[1]}×{img.shape[0]} 이 보정 크기 {w}×{h} 와 다름 (폰 프로그램 --scale?)"
            else:
                K1, d1, K2, d2 = (np.asarray(c[k], np.float64) for k in ("K1", "dist1", "K2", "dist2"))
                R1, _, P1, _, _, _, _ = cv2.stereoRectify(K1, d1, K2, d2, (w, h), np.asarray(c["R"], np.float64),
                                                          np.asarray(c["T"], np.float64).reshape(3, 1),
                                                          flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.0)
                u, v = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
                q = cv2.undistortPoints(np.stack([u, v], -1).reshape(-1, 1, 2), K1, d1, R=R1, P=P1).reshape(h, w, 2)
                maps = (q[..., 0].astype(np.float32), q[..., 1].astype(np.float32))
        except (OSError, ValueError, KeyError, TypeError, cv2.error) as e:
            why = f"정류 되돌리기 실패: {e}"
        _UNRECT.update(key=key, maps=maps, why=why)
    if _UNRECT["maps"] is None:
        return img, _UNRECT["why"]
    return cv2.remap(img, *_UNRECT["maps"], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT), None


def pipeline_calib(dcfg):
    """폰 프로그램이 정류에 쓴 보정 파일: 대시보드가 켰으면 그때 고른 파일 (Pipeline.calib_file), 아니면 지금 폰 모델로 고른 파일.
    (프레임마다 adb 로 모델을 묻지 않게, 그리고 adb 가 끊겨도 같은 파일로 되돌리게 — 2026-11-25 검토)"""
    import phone_api
    f = getattr(phone_api.PIPE, "calib_file", None) if phone_api.PIPE is not None else None
    return Path(dcfg["repo_dir"]) / (f or phone_api.calib_choice(dcfg)["file"])


def flip_panels(img):
    """화면용: inference.png 의 그림 칸마다 180° 돌린다 (폰이 거치대에 거꾸로 끼워져 위아래가 뒤집혀 보임, 2026-11-24 사용자).
    오른쪽 70 px 뎁스 눈금도 따로 돌린다 — 폰 프로그램이 --text-rot180 으로 글자를 미리 거꾸로 써 두어 돌리면 바로 읽힌다 (2026-11-25).
    칸 모양을 모르면 통째로 돌린다. 인식(ArUco·뎁스)은 원본 그림을 쓴다."""
    import numpy as np
    h, w = img.shape[:2]
    for n in (3, 2):
        lw = (w - 70) / n
        if lw == int(lw) and 1.0 <= lw / h <= 2.0:
            lw = int(lw)
            return np.hstack([img[:, i * lw:(i + 1) * lw][::-1, ::-1] for i in range(n)] + [img[:, n * lw:][::-1, ::-1]])
    return img[::-1, ::-1]


# ---------------------------------------------------------------- 가상 장면
class Scene:
    """가상 모드의 책상 위 큐브들. level = 그 밑에 깔린 큐브 수. 바꿀 때마다 파일에 쓴다."""

    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.Lock()
        self.cubes, self.jitter, self.t, self.last_move = [], False, time.time(), None
        try:
            d = json.loads(self.path.read_text())
            self.cubes = [{"id": int(c["id"]), "x": float(c["x"]), "y": float(c["y"]), "yaw_deg": round(fold(c["yaw_deg"]), 1), "level": int(c["level"])}
                          for c in d.get("cubes", []) if int(c["id"]) in CUBE_IDS]
            self.jitter = bool(d.get("jitter"))
        except (OSError, ValueError, KeyError, TypeError):
            pass

    def _save(self):
        self.t = time.time()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps({"cubes": self.cubes, "jitter": self.jitter, "t": self.t, "last_move": self.last_move}, ensure_ascii=False))
        os.replace(tmp, self.path)

    def snapshot(self):
        with self.lock:
            return {"cubes": [dict(c) for c in self.cubes], "jitter": self.jitter, "t": self.t, "last_move": self.last_move}

    def _at(self, x, y, skip=None):
        """(x, y) 자리 (STACK_R 안) 의 큐브 중 맨 위 것, 없으면 None."""
        near = [c for c in self.cubes if c is not skip and math.hypot(c["x"] - x, c["y"] - y) <= STACK_R]
        return max(near, key=lambda c: c["level"]) if near else None

    def _covered(self, c):
        return any(o is not c and o["level"] > c["level"] and math.hypot(o["x"] - c["x"], o["y"] - c["y"]) <= STACK_R for o in self.cubes)

    def _by_id(self, i):
        return next((c for c in self.cubes if c["id"] == i), None)

    def op(self, b):
        """화면 지도에서: add {x, y, yaw_deg?} · move {id, x, y} · yaw {id, yaw_deg} · delete {id} · clear · jitter {on}."""
        op = str(b.get("op") or "")
        with self.lock:
            if op == "clear":
                self.cubes = []
            elif op == "jitter":
                self.jitter = bool(b.get("on"))
            elif op in ("add", "move", "yaw", "delete"):
                c = None
                if op != "add":
                    c = self._by_id(int(b.get("id")))
                    if c is None:
                        return {"ok": False, "error": f"큐브 {b.get('id')} 없음"}
                if op in ("add", "move"):
                    x, y = float(b["x"]), float(b["y"])
                    if not (math.isfinite(x) and math.isfinite(y) and abs(x) <= 1 and abs(y) <= 1):
                        return {"ok": False, "error": "x, y 는 ±1 m 안"}
                if op in ("move", "delete") and self._covered(c):
                    return {"ok": False, "error": f"큐브 {c['id']} 위에 다른 큐브가 있음 — 위의 것부터"}
                if op == "add":
                    free = [i for i in CUBE_IDS if self._by_id(i) is None]
                    if not free:
                        return {"ok": False, "error": f"큐브는 {len(CUBE_IDS)}개까지 (ID {CUBE_IDS[0]}~{CUBE_IDS[-1]})"}
                    if self._at(x, y):
                        return {"ok": False, "error": "그 자리에 큐브가 있음 — 끌어서 옮기기"}
                    yaw = float(b.get("yaw_deg") or 0.0)
                    self.cubes.append({"id": free[0], "x": round(x, 4), "y": round(y, 4), "yaw_deg": round(fold(yaw), 1), "level": 0})
                elif op == "move":
                    under = self._at(x, y, skip=c)
                    if under:   # 다른 큐브 위에 떨어뜨림 → 그 위에 쌓기 (자리·각을 맞춘다)
                        x, y, c["yaw_deg"] = under["x"], under["y"], under["yaw_deg"]
                    c.update(x=round(x, 4), y=round(y, 4), level=under["level"] + 1 if under else 0)
                elif op == "yaw":
                    v = float(b["yaw_deg"])
                    if not math.isfinite(v):
                        return {"ok": False, "error": "각은 숫자"}
                    c["yaw_deg"] = round(fold(v), 1)
                else:
                    self.cubes.remove(c)
            else:
                return {"ok": False, "error": "op = add · move · yaw · delete · clear · jitter"}
            self._save()
            return {"ok": True, "scene": {"cubes": [dict(c) for c in self.cubes], "jitter": self.jitter, "t": self.t}}

    def place(self, src_xy, dst_xy, yaw_deg, why=""):
        """가상 팔이 큐브를 옮겼다: src 자리 맨 위 큐브 → dst (그 자리에 큐브가 있으면 그 위). 어긋남이 켜져 있으면 ±1 cm·±5°."""
        with self.lock:
            c = self._at(*src_xy)
            if c is None or math.hypot(c["x"] - src_xy[0], c["y"] - src_xy[1]) > 0.02:
                self.last_move = {"ok": False, "t": time.time(), "why": why, "text": f"가상 장면에 집은 자리 ({src_xy[0]:.3f}, {src_xy[1]:.3f}) 의 큐브가 없음 — 장면 안 바뀜"}
                self._save()
                return self.last_move
            x, y, yaw = float(dst_xy[0]), float(dst_xy[1]), float(yaw_deg)
            dx = dy = dyaw = 0.0
            if self.jitter:
                dx, dy, dyaw = random.uniform(-JITTER_M, JITTER_M), random.uniform(-JITTER_M, JITTER_M), random.uniform(-JITTER_DEG, JITTER_DEG)
            under = self._at(x, y, skip=c)
            c.update(x=round(x + dx, 4), y=round(y + dy, 4), yaw_deg=round(fold(yaw + dyaw), 1), level=under["level"] + 1 if under else 0)
            self.last_move = {"ok": True, "t": time.time(), "why": why, "id": c["id"], "level": c["level"],
                              "jitter_mm": [round(dx * 1000, 1), round(dy * 1000, 1)], "jitter_deg": round(dyaw, 1),
                              "text": f"가상 큐브 {c['id']} → ({c['x']:.3f}, {c['y']:.3f}) {c['level'] + 1}층" +
                                      (f" (어긋남 {dx * 1000:+.0f}, {dy * 1000:+.0f} mm · {dyaw:+.1f}°)" if self.jitter else "")}
            self._save()
            return self.last_move

    def on_play(self, traj, why=""):
        """가상 팔 재생을 시작할 때 부른다 → 재생이 끝까지 가면 부를 함수 (on_end). 큐브를 옮기는 계획이 아니면 None.
        traj['real'] = {cube: [x, y, yaw], target: [x, y], target_yaw?} (pick_plan 계획, 강화학습 재생이 같은 모양으로 넣음)."""
        r = (traj or {}).get("real") or {}
        try:
            src = (float(r["cube"][0]), float(r["cube"][1]))
            dst = (float(r["target"][0]), float(r["target"][1]))
            yaw = float(r["target_yaw"] if r.get("target_yaw") is not None else r["cube"][2])
        except (KeyError, TypeError, ValueError, IndexError):
            return None

        def end(reason):
            if reason == "끝까지 재생":
                self.place(src, dst, yaw, why)
        return end

    def cubes_view(self):
        s = self.snapshot()
        return [{**c, "source": "virtual", "age_s": 0.0} for c in s["cubes"]], s


# ---------------------------------------------------------------- 실물: 폰 프레임 → 인식 (서버 스레드)
class Live:
    def __init__(self, log_dir, mode):
        self.log_dir = Path(log_dir)
        self.mode = mode
        self.lock = threading.Lock()
        self.res = None            # 마지막 프레임의 인식 결과
        self.jpeg = None           # 그 프레임 (마커를 그린 것)
        self.raw_jpeg, self.raw_t = None, 0.0   # live.jpg 받은 그대로 (스트림용, 인식 전)
        self.frame = None          # 다시 계산용 (책상 높이가 바뀌면)
        self.key = None
        self.td = TD_DEFAULT
        self.td_done = None
        self.arrivals = deque(maxlen=12)
        self.prev = {"pos": None, "t": 0.0}
        self.n = 0
        self.thread = None
        self.inject = None         # 시험: (BGR 배열, 프레임 시각) — 파일 대신
        self.unrect_why = None     # inference.png 정류 되돌리기를 못 한 이유 (unrectify)
        self.imu_fix_deg = None    # 이번 프레임에 중력 센서로 고친 기울기 (°), 못 고쳤으면 None

    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._loop, daemon=True, name="live-cubes")
            self.thread.start()

    def _loop(self):
        while True:
            try:
                if self.mode() != "virtual":
                    self._step()
            except Exception as e:   # 한 프레임이 망가져도 스레드는 산다
                with self.lock:
                    self.res = {**(self.res or {}), "error": f"{type(e).__name__}: {e}"}
            time.sleep(POLL_S)

    def _grab(self, dcfg):
        """새 프레임이 있으면 (BGR, 프레임 시각, 출처 이름), 없으면 None."""
        import cv2
        import numpy as np
        if self.inject is not None:
            img, t = self.inject
            self.inject = None
            return img, t, "test"
        run = DP.run_dir(dcfg)
        now = time.time()
        for name in ("live.jpg", "inference.png"):
            p = os.path.join(run, name)
            try:
                st = os.stat(p)
            except OSError:
                continue
            if name == "live.jpg" and now - st.st_mtime > LIVE_JPG_FRESH_S:
                continue
            key = (name, st.st_mtime_ns, st.st_size)
            if key == self.key:
                return None
            try:
                with open(p, "rb") as f:
                    raw = f.read()
            except OSError:
                return None
            img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if img is None:          # 쓰는 중 — 다음 번에
                return None
            self.key = key
            if name == "live.jpg":   # 화면 스트림은 인식(ArUco·뎁스)을 기다리지 않고 받은 그대로 바로 (들쭉날쭉 줄임, 2026-11-24)
                self.raw_jpeg, self.raw_t = raw, time.time()
            if name == "inference.png":   # 왼쪽 칸 = 정류된 그림 → 원래 렌즈 모습으로 (unrectify), 못 하면 그대로 + 이유
                img, self.unrect_why = unrectify(left_panel(img), pipeline_calib(dcfg))
            return img, st.st_mtime, name
        return None

    def _step(self):
        import phone_api
        dcfg = DP.load_cfg(self.log_dir / "phone_depth.json")
        got = self._grab(dcfg)
        if got is None:
            if self.frame is not None and self.td_done != self.td:   # 책상 높이가 바뀜 → 마지막 프레임 다시
                img, t, src = self.frame
                self._process(img, t, src, dcfg, phone_api, again=True)
            return
        img, t, src = got
        self.arrivals.append((t, src))
        self.frame = (img, t, src)
        self._process(img, t, src, dcfg, phone_api)

    def _process(self, img, t, src, dcfg, phone_api, again=False):
        import cv2
        import phone_detect as PD
        import phone_fuse as PF
        td = self.td
        c2r, self.imu_fix_deg = phone_api.live_c2r(dcfg)      # 기울기 = 이 프레임의 폰 중력 센서 (저장 안 함)
        dcfg = {**dcfg, "camera_to_robot": c2r, "_imu_tilt": self.imu_fix_deg is not None}
        # 기준 종이가 안 보이면 거치 자세 (기울기는 중력 센서) + 폰 보정값 + 책상 높이 (화면의 d 칸) 로 큐브 자리 (phone_detect)
        cam_cfg = {**phone_api._load(self.log_dir / "phone_cam.json"), "intr": phone_api.intrinsics(dcfg), "td": td,
                   "image_rot_q": phone_api.image_rot_q(dcfg), "c2r": dcfg.get("camera_to_robot"),
                   "max_levels": LIVE_MAX_LEVELS}
        now = time.monotonic()
        multi = PD.detect_cubes(img, cam_cfg)          # 큐브 여러 개 (4x4 ID 0~4, 층, 기울기)
        # 뎁스 후보가 여럿이면 휴대폰 탭이 집을 큐브 (ArUco ID 0, 없으면 처음 본 큐브) 쪽을 고르게 (2026-11-25 '헷갈림' 고침)
        aru = [c for c in multi.get("cubes", []) if c.get("id") == PD.CUBE_ID] or list(multi.get("cubes", []))[:1]
        prefer = (aru[0]["x"], aru[0]["y"]) if aru else None
        dep = DP.evaluate(dcfg, td, time.time(), self.prev["pos"] if now - self.prev["t"] <= 3.0 else None, prefer=prefer)
        if dep.get("ok"):
            self.prev.update(pos=dep.pop("pos"), t=now)
        fused = PF.find(img, cam_cfg, dep)            # 휴대폰 탭: 큐브 하나 (4x4 ID 0 + 뎁스)
        if fused.get("aruco") is not None:
            for k in ("annotated", "cfg"):
                fused["aruco"].pop(k, None)
        vis = multi.pop("annotated", None)
        multi.pop("cfg", None)
        cubes = []
        for c in multi.get("cubes", []):
            cubes.append({**c, "source": "aruco"})
        p = dep if dep.get("ok") else None
        if p:
            near = [c for c in cubes if c["level"] == 0 and math.hypot(c["x"] - p["cube"]["x"], c["y"] - p["cube"]["y"]) <= DEPTH_POS_TOL]
            if near:
                near[0]["source"] = "aruco+depth"
            else:
                cubes.append({"id": None, "x": p["cube"]["x"], "y": p["cube"]["y"], "yaw_deg": p["cube"]["yaw_deg"], "level": 0,
                              "source": "depth", "usable": p.get("usable", True)})
        jpeg = None
        if vis is None:
            vis = img
        k = 960 / max(vis.shape[:2])
        if k < 1:
            vis = cv2.resize(vis, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if ok:
            jpeg = buf.tobytes()
        with self.lock:
            if not again:
                self.n += 1
            self.res = {"t_frame": t, "t_done": time.time(), "src": src, "frame_n": self.n, "cubes": cubes,
                        "refs": multi.get("refs", []), "warn": multi.get("warn", []), "reason": None if multi.get("ok") else multi.get("reason"),
                        "level_unsure": bool(multi.get("level_unsure")), "fused": fused, "depth_age_s": dep.get("age_s"), "td": td,
                        "camera_src": (multi.get("camera") or {}).get("src")}
            self.jpeg = jpeg
            self.td_done = td

    def stale_s(self):
        """기다릴 한계: live.jpg 면 STALE_S, inference.png 면 그 저장 간격의 1.6 배 (STALE_S ~ STALE_MAX_S)."""
        a = [t for t, s in list(self.arrivals) if s == "inference.png"]   # list() = 한 번에 복사 (프레임 스레드가 붙이는 중이어도)
        if len(a) < 3 or self.res is None or self.res.get("src") != "inference.png":
            return STALE_S
        gaps = sorted(b - a_ for a_, b in zip(a, a[1:]))
        return min(STALE_MAX_S, max(STALE_S, 1.6 * gaps[len(gaps) // 2]))

    def rate(self):
        a = [t for t, _ in list(self.arrivals)]
        return round((len(a) - 1) / (a[-1] - a[0]), 1) if len(a) >= 3 and a[-1] > a[0] else None


# ---------------------------------------------------------------- 응답
def _pick_camera(res, stale, pick_id):
    """휴대폰 탭이 집을 큐브 하나 (phone_fuse 모양): ID 를 고르지 않았거나 PD.CUBE_ID (0) 면 ArUco 0 + 뎁스를 합친 것, 다른 ID 면 그 ArUco 큐브."""
    if res is None or stale:
        return {"ok": False, "stale": True, "warn": [], "note": [], "sources": [],
                "reason": "영상 없음 — 폰 프로그램이 꺼져 있거나 프레임이 멈춤 (새 프레임이 들어와야 집는다)"}
    if pick_id is not None and pick_id != PD.CUBE_ID:
        c = next((c for c in res["cubes"] if c["id"] == pick_id), None)
        if c is None:
            return {"ok": False, "warn": [], "note": [], "sources": [], "reason": f"큐브 {pick_id} 가 지금 프레임에 안 보임"}
        out = {"ok": True, "id": pick_id, "cube": {"x": c["x"], "y": c["y"], "yaw_deg": c["yaw_deg"]}, "used": "aruco", "sources": ["aruco"],
               "text": f"출처: ArUco (큐브 {pick_id})", "blocked": False, "block": "", "warn": list(res.get("warn") or []), "note": [], "check": None}
        if c.get("unsure") or res.get("level_unsure"):   # 층·자리가 확실하지 않음 (쌓인 큐브가 책상 위로 읽혔을 수 있다)
            out.update(blocked=True, block="층이 확실하지 않음")
            out["warn"].insert(0, f"큐브 {pick_id} 의 층·자리가 확실하지 않아 막음 — 기준 종이를 같이 보이게 하거나 큐브를 폰 가까이")
        elif c["level"] > 0:
            out.update(blocked=True, block="쌓인 큐브")
            out["warn"].insert(0, f"큐브 {pick_id} 는 {c['level'] + 1}층 — 휴대폰 탭은 책상 위 큐브만 집는다 (탑 쌓기 탭)")
        return out
    f = dict(res["fused"])
    f["id"] = PD.CUBE_ID if f.get("used") == "aruco" else None
    c10 = next((c for c in res["cubes"] if c.get("id") == PD.CUBE_ID), None)
    if f.get("ok") and not f.get("blocked") and c10 and (res.get("level_unsure") or c10.get("unsure") or c10["level"] > 0):
        f.update(blocked=True, block="쌓인 큐브" if c10["level"] > 0 and not c10.get("unsure") and not res.get("level_unsure") else "층이 확실하지 않음",
                 warn=[f"큐브 {PD.CUBE_ID} 이 쌓여 있거나 층이 확실하지 않아 막음 (휴대폰 탭은 책상 위 큐브만)"] + list(f.get("warn") or []))
    return f


def _pick_virtual(cubes, pick_id):
    top = [c for c in cubes if not any(o is not c and o["level"] > c["level"] and math.hypot(o["x"] - c["x"], o["y"] - c["y"]) <= STACK_R for o in cubes)]
    if not cubes:
        return {"ok": False, "warn": [], "note": [], "sources": [], "reason": "가상 큐브가 없음 — 지도를 눌러 큐브를 놓기"}
    if pick_id is None:
        c = min((c for c in top if c["level"] == 0), key=lambda c: c["id"], default=None) or min(cubes, key=lambda c: c["id"])
    else:
        c = next((c for c in cubes if c["id"] == pick_id), None)
        if c is None:
            return {"ok": False, "warn": [], "note": [], "sources": [], "reason": f"가상 큐브 {pick_id} 없음"}
    out = {"ok": True, "id": c["id"], "cube": {"x": c["x"], "y": c["y"], "yaw_deg": c["yaw_deg"]}, "used": "virtual", "sources": ["virtual"],
           "text": f"출처: 가상 큐브 {c['id']}", "blocked": False, "block": "", "warn": [], "note": [], "check": None}
    if c not in top:
        out.update(blocked=True, block="위에 큐브가 있음")
    elif c["level"] > 0:
        out.update(blocked=True, block="쌓인 큐브")
        out["warn"].append(f"큐브 {c['id']} 는 {c['level'] + 1}층 — 휴대폰 탭은 책상 위 큐브만 집는다 (탑 쌓기 탭)")
    return out


LIVE = None
SCENE = None


def cubes(td=None, pick_id=None):
    """GET /api/live/cubes 의 몸통 (다른 모듈도 부른다: rl_api 의 실물 재생 전 확인)."""
    import phone_api
    if td is not None and not (math.isfinite(td) and 0.2 <= td <= 0.8):
        td = None
    if td is not None:
        LIVE.td = td
    pipe = phone_api.PIPE
    measured = phone_api.LAST_TABLE.get("r")
    table = {"d": LIVE.td, "measured": measured}      # 카메라 모드면 아래에서 '이 결과를 계산할 때 쓴 d' 로 바꾼다
    if LIVE.mode() == "virtual":
        cs, s = SCENE.cubes_view()
        return {"ok": True, "mode": "virtual", "stale": False, "reason": None, "age_s": 0.0, "src": "virtual", "rate_hz": None,
                "stale_s": None, "t_frame": s["t"], "frame_n": None, "cubes": cs, "refs": [], "warn": [], "table": table,
                "pick": _pick_virtual(cs, pick_id), "pipeline": {}, "scene": {"jitter": s["jitter"], "last_move": s["last_move"]}}
    with LIVE.lock:
        res = LIVE.res
    now = time.time()
    lim = LIVE.stale_s()
    age = None if res is None or "t_frame" not in res else now - res["t_frame"]
    stale = age is None or age > lim
    pid = pipe.pid() if pipe else None
    pl = {"alive": pid is not None, "pid": pid, "log": (pipe.tail() if pid is None else pipe.tail(3)) if pipe else []}
    out = {"ok": not stale, "mode": "camera", "stale": stale, "age_s": None if age is None else round(age, 2), "stale_s": round(lim, 2),
           "src": res.get("src") if res else None, "rate_hz": LIVE.rate(), "t_frame": res.get("t_frame") if res else None,
           "frame_n": res.get("frame_n") if res else None, "table": table, "pipeline": pl, "scene": None,
           "error": (res or {}).get("error")}
    # 책상 높이 d 는 서버에 하나 (LIVE.td) 라 마지막에 부른 화면의 d 로 다시 계산된다 (2026-11-24 검토) — 결과에는 계산에 쓴 d 를 싣고,
    # 이 화면의 d 와 다르면 (다른 기기가 다른 d 로 부름 · 바꾼 직후 아직 다시 계산 전) 그 결과를 주지 않는다 (큐브 x·y·층이 d 에 달림)
    td_used = res.get("td") if res else None
    table["d"] = td_used
    mismatch = td is not None and td_used is not None and abs(td_used - td) > TD_MATCH_M
    if stale:
        out.update(cubes=[], refs=[], warn=[], reason="영상 없음" + (f" — 마지막 프레임 {age:.1f}초 전" if age is not None else
                                                                 " — 폰 프로그램의 프레임이 아직 없음 (폰 프로그램 켜기)"))
    elif mismatch:
        out.update(ok=False, td_mismatch=True, cubes=[], refs=[], warn=[],
                   reason=f"책상 높이가 이 화면 d {td:.4f} m 와 다른 {td_used:.4f} m 로 계산된 결과 — 다시 계산 중 "
                          "(계속 이러면 다른 기기·탭이 다른 d 칸으로 보고 있다: d 칸을 맞추기)")
    else:
        out.update(cubes=[{**c, "age_s": round(age, 2)} for c in res["cubes"]], refs=res["refs"], warn=res["warn"], reason=res["reason"],
                   level_unsure=res["level_unsure"], depth_age_s=res.get("depth_age_s"))
        if res.get("src") == "inference.png" and LIVE.unrect_why:   # 정류 되돌리기 실패 → 마커 크기·층·자리가 틀린다 (조용히 넘기지 않음)
            out["warn"] = list(out["warn"]) + [f"그림 정류 되돌리기 못 함 ({LIVE.unrect_why}) — 큐브 마커 크기·층·자리가 틀릴 수 있다"]
            out["level_unsure"] = True
    if mismatch:
        out["pick"] = {"ok": False, "stale": False, "warn": [], "note": [], "sources": [], "reason": out["reason"]}
        return out
    out["pick"] = _pick_camera(res, stale, pick_id)
    if res is not None and not stale:
        out["pick"]["depth"] = res["fused"].get("depth")
    return out


def install(app, log_dir, mode):
    """server.py 가 부른다. mode() → 'direct'|'virtual'|'ros'."""
    global LIVE, SCENE
    LIVE = Live(log_dir, mode)
    SCENE = Scene(Path(log_dir) / "virtual_scene.json")
    LIVE.start()

    def _args(d, pick):
        td = float(d) if str(d).strip() else None
        pid = int(pick) if str(pick).strip() not in ("", "null", "None") else None
        return td, pid

    @app.get("/api/live/cubes")
    def _cubes(d: str = "", pick: str = ""):
        try:
            td, pid = _args(d, pick)
        except ValueError:
            return JSONResponse({"ok": False, "reason": "d (m), pick (큐브 번호) 숫자"}, status_code=400)
        return JSONResponse(cubes(td, pid))

    @app.get("/api/live/frame.jpg")
    def _frame():
        j = LIVE.jpeg
        if j is None or LIVE.mode() == "virtual":
            return Response(status_code=404)
        return Response(j, media_type="image/jpeg", headers={"cache-control": "no-store"})

    @app.get("/api/live/stream.mjpg")
    async def _stream(req: Request):
        """마지막 프레임을 MJPEG 로 계속 (브라우저 <img> 가 그대로 영상으로 보임, 최대 15 Hz). 2026-11-24 사용자: 웹 프레임이 너무 낮음
        (전엔 0.5 s 마다 한 장씩 다시 불러 최대 2 Hz). 새 프레임이 생길 때만 보낸다."""
        import asyncio
        if LIVE.mode() == "virtual":
            return Response(status_code=404)

        async def gen():
            last = None
            while not await req.is_disconnected():
                j = LIVE.raw_jpeg if LIVE.raw_jpeg is not None and time.time() - LIVE.raw_t < 1.5 else LIVE.jpeg
                if j is not None and j is not last:
                    last = j
                    yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(j)).encode() + b"\r\n\r\n" + j + b"\r\n"
                await asyncio.sleep(1 / 15)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame", headers={"cache-control": "no-store"})

    @app.get("/api/live/inference.png")
    def _inference(flip: int = 0):
        """폰 프로그램이 저장한 뎁스·세그 결과 그림 (inference.png: 객체 세그 + 뎁스 색) — 화면에 원본 영상 아래 같이 보인다 (2026-11-24 사용자).
        ?flip=1 이면 그림 칸마다 180° 돌려 보낸다 (flip_panels)."""
        if LIVE.mode() == "virtual":
            return Response(status_code=404)
        try:
            path = os.path.join(DP.run_dir(DP.load_cfg(LIVE.log_dir / "phone_depth.json")), "inference.png")
        except Exception:
            return Response(status_code=404)
        if not os.path.isfile(path):
            return Response(status_code=404)
        with open(path, "rb") as f:
            raw = f.read()
        if not raw.rstrip(b"\0").endswith(b"IEND\xaeB`\x82"):   # 폰 프로그램이 쓰는 도중의 반쪽 파일 — 디코드하지 않고(libpng 경고 방지) 직전 완성본을 보낸다
            last = getattr(_inference, "last", {}).get(bool(flip))
            if last:
                return Response(last[1], media_type=last[0], headers={"cache-control": "no-store"})
            return Response(status_code=404)
        if flip:
            import cv2
            import numpy as np
            img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:      # 쓰는 도중의 반쪽 파일이면 그대로 보낸다 (브라우저가 이전 그림을 둔다)
                ok, buf = cv2.imencode(".jpg", flip_panels(img), [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    _inference.__dict__.setdefault("last", {})[True] = ("image/jpeg", buf.tobytes())
                    return Response(buf.tobytes(), media_type="image/jpeg", headers={"cache-control": "no-store"})
        _inference.__dict__.setdefault("last", {})[bool(flip)] = ("image/png", raw)
        return Response(raw, media_type="image/png", headers={"cache-control": "no-store"})

    @app.post("/api/live/scene")
    async def _scene(req: Request):
        if mode() != "virtual":
            return JSONResponse({"ok": False, "error": "가상 모드에서만 (실물은 카메라가 본 큐브)"})
        try:
            b = await req.json()
            if not isinstance(b, dict):          # [] · null · "str" 도 400 으로 (전엔 b.get 에서 500)
                raise TypeError("body is not an object")
            return JSONResponse(SCENE.op(b))
        except (KeyError, TypeError, ValueError):
            return JSONResponse({"ok": False, "error": "값이 틀림"}, status_code=400)

    @app.get("/api/phone/live")
    def _phone_live(d: str = "", pick: str = ""):
        try:
            td, pid = _args(d, pick)
        except ValueError:
            return JSONResponse({"ok": False, "reason": "책상 높이 d 숫자 (0.2~0.8 m)"}, status_code=400)
        r = cubes(td, pid)
        return JSONResponse({**r["pick"], "fresh": not r["stale"], "pipeline": r["pipeline"], "mode": r["mode"]})
