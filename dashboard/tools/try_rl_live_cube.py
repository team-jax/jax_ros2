#!/usr/bin/env python3
"""카메라 큐브 → 강화학습 (POST /api/rl/live_cube, 강화학습 탭 '📷 카메라 큐브로 강화학습 돌리기') 시험.

  .venv/bin/python dashboard/tools/try_rl_live_cube.py

1) 판단 (서버 _rl_live_cube, 가짜 인식 결과): 받침대 없이 낮은 큐브(마커 34 mm)는 막지 않고 경고만 (2026-11-26 너무 깐깐), 정상은 위치로 바꾼다.
2) 끝에서 끝 (가상 모드 시험 서버, 빈 포트): 가상 큐브 → 위치 오프셋이 rl_run.spot_from_robot 과 같은가, 큐브 각 90° 접기,
   큐브 번호 고르기, 큐브 없음, 이상한 본문(500 없음), 그 위치로 강화학습이 끝까지 집는가 (JAX_IK_DIR 가 있으면).
팔·CAN·폰은 안 건드린다 (가상 모드).
"""
import json
import os
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
FAIL = []


def check(name, ok, detail=""):
    print(("✓ " if ok else "✗ ") + name + (f" — {detail}" if detail else ""), flush=True)
    if not ok:
        FAIL.append(name)


# ---------------------------------------------------------------- 1) 판단만 (가짜 live_api.cubes)
def _pre_snap(r):
    """학습 범위 끝으로 옮기기 전 위치 (2026-11-26 snap) — 옮겼으면 snap.from"""
    sp = dict(r.get("spot") or {})
    if r.get("snap"):
        sp["x_cm"], sp["y_cm"] = r["snap"]["from"]
    return sp


def unit():
    import live_api
    import server
    real = live_api.cubes
    live_api.LIVE = live_api.LIVE or object()

    def fake(pick, cubes):
        return lambda td=None, pick_id=None: {"ok": True, "stale": False, "cubes": cubes, "pick": pick}
    base = {"ok": True, "id": 1, "cube": {"x": -0.018, "y": -0.265, "yaw_deg": 20.0}, "text": "출처: ArUco + 뎁스", "warn": [], "blocked": False}
    try:
        live_api.cubes = fake(dict(base, depth={"height_cm": 2.0}), [{"id": 1, "marker_mm": 40.5}])
        r = server._rl_live_cube({})
        import math as _m, rl_run as _rl
        fx = server.RL_REACH_FIX_M
        rr = _m.hypot(-0.018, -0.265)
        sx = server.RL_SIDE_FIX_M
        want = _rl.spot_from_robot(-0.018 * (1 + fx / rr) + sx, -0.265 * (1 + fx / rr), 20.0)
        check(f"정상 → 위치 (어깨에서 {fx * 100:.0f} cm 더 앞, 옆 {sx * 100:+.0f} cm = 로봇 오른쪽)", r.get("ok") and _pre_snap(r) == want
              and r.get("reach_fix_cm") == round(fx * 100, 1) and r.get("side_fix_cm") == round(sx * 100, 1), str(r.get("spot") or r))
        r0 = server._rl_live_cube({"reach_fix_cm": 0, "side_fix_cm": 0})
        check("reach_fix_cm 0 → 고침 없음", r0.get("ok") and _pre_snap(r0) == {"x_cm": 2.0, "y_cm": -3.0, "yaw_deg": 20.0}, str(r0.get("spot")))
        r9 = server._rl_live_cube({"reach_fix_cm": 99})
        check("reach_fix_cm 99 → 5 cm 로 자름", r9.get("ok") and r9.get("reach_fix_cm") == 5.0, str(r9.get("reach_fix_cm")))
        live_api.cubes = fake(dict(base, blocked=True, block="층이 확실하지 않음", depth={"height_cm": -8.1}), [{"id": 1, "marker_mm": 39.0}])
        r = server._rl_live_cube({})
        check("막힌 큐브 → 안 씀, 이유 그대로", not r.get("ok") and "층이 확실하지 않음" in r.get("error", ""), r.get("error", "")[:70])
        live_api.cubes = fake(dict(base, depth={}), [{"id": 1, "marker_mm": 34.0}])
        r = server._rl_live_cube({})
        check("ArUco 만 · 마커 34 mm → 막지 않고 받침대 경고", r.get("ok") and any("받침대" in w for w in r.get("warn", [])), str(r.get("warn") or r.get("error"))[:70])
        live_api.cubes = fake(dict(base, depth={}), [{"id": 1, "marker_mm": 38.5}])
        check("ArUco 만 · 마커 38.5 mm → 씀", server._rl_live_cube({}).get("ok"))
        live_api.cubes = fake(dict(base, depth={"height_cm": -8.0}), [{"id": 1, "marker_mm": 39.5}])
        r = server._rl_live_cube({})
        check("뎁스=표시만: 마커 크기 맞음 + 뎁스만 8 cm 아래 → 씀 (뎁스 높이는 표시용 숫자만)", r.get("ok") and r.get("depth_height_cm") == -8.0
              and not any("뎁스" in w for w in r.get("warn", [])), str(r.get("warn"))[:70])
        live_api.cubes = fake(dict(base, depth={"height_cm": 1.5}), [{"id": 1, "marker_mm": 34.0}])
        r = server._rl_live_cube({})
        check("마커 34 mm → 경고만 (뎁스가 맞다고 해도)", r.get("ok") and any("받침대" in w for w in r.get("warn", [])), str(r.get("warn") or r.get("error"))[:60])
        # 휴대폰 탭 책상 높이(0.5575)로 잰 결과 → 서버 높이는 안 바꾸고 RL 받침대 높이로 다시 투영 (2026-11-25 '카메라가 이상해짐')
        import types
        saved_live = live_api.LIVE
        live_api.LIVE = types.SimpleNamespace(log_dir=HERE / "logs")
        seen = []
        live_api.cubes = lambda td=None, pick_id=None: (seen.append(td), {"ok": True, "stale": False, "mode": "camera", "table": {"d": 0.5575},
                                                                          "cubes": [{"id": 1, "marker_mm": 45.1}], "pick": dict(base)})[1]
        r = server._rl_live_cube({"reach_fix_cm": 0, "side_fix_cm": 0})
        _, _, ratio = server._rl_reproject(-0.018, -0.265, 0.5575, server.RL_TABLE_D)
        lo_d, hi_d = 0.40, 0.60                       # 마커가 40 mm 로 보이는 높이를 따로 풀기 (이분법)
        for _ in range(60):
            mid = (lo_d + hi_d) / 2
            if 45.1 * server._rl_reproject(-0.018, -0.265, 0.5575, mid)[2] > 40.0:   # 멀수록(d 클수록) 크게 보인다 → 크면 더 가까이
                hi_d = mid
            else:
                lo_d = mid
        xa, ya, _ = server._rl_reproject(-0.018, -0.265, 0.5575, mid)
        xr, yr, _ = server._rl_reproject(-0.018, -0.265, 0.5575, server.RL_TABLE_D + server.RL_MARKER_REF_DROP_M)
        xp, yp, _ = server._rl_reproject(-0.018, -0.265, 0.5575, server.RL_TABLE_D)
        xs, ys = xp + (xa - xr), yp + (ya - yr)       # 2026-11-26: 손 보정을 맞춘 마커 높이(4.75 cm 아래)에서는 예전 자리, 그 차만큼만 옮긴다
        check("0.5575 로 잰 큐브 → 마커(45.1 mm)로 잰 실제 높이로 다시 투영 (서버 높이는 안 바꿈)", r.get("ok") and seen == [None] and r.get("reproject")
              and abs(r["reproject"]["to_d"] - mid) < 2e-4 and _pre_snap(r) == __import__("rl_run").spot_from_robot(round(xs, 4), round(ys, 4), 20.0)
              and abs(r["marker_mm"] - round(45.1 * ratio, 1)) < 1e-6, f"투영 {r.get('reproject')} · 정책 높이로 본 마커 {r.get('marker_mm')} mm")
        live_api.cubes = lambda td=None, pick_id=None: {"ok": True, "stale": False, "mode": "camera", "table": {"d": 0.5575},
                                                         "cubes": [{"id": 1, "marker_mm": 39.0}], "pick": dict(base)}
        r = server._rl_live_cube({})
        check("마커 39 mm (받침대 없이 실물 책상?) → 막지 않고 경고만", r.get("ok") and any("받침대" in w for w in r.get("warn", [])), str(r.get("warn") or r.get("error"))[:80])
        # 2026-11-26 '마커가 작으면 그만큼 보정': 마커로 잰 높이가 정책보다 낮은 만큼 밀어 넣기 때 손을 더 내린다 (기본 이상, RL_MARKER_DOWN_MAX_CM 이하)
        import rl_run as _RR
        downs = {}
        for pol in (40.0, 37.0, 36.0, 35.9, 35.0, 33.0):       # 정책 높이로 본 마커 mm (화면 숫자) → 서버 책상(0.5575) 기준 날값으로 넣는다
            raw = pol / ratio
            live_api.cubes = lambda td=None, pick_id=None, raw=raw: {"ok": True, "stale": False, "mode": "camera", "table": {"d": 0.5575},
                                                                     "cubes": [{"id": 1, "marker_mm": raw}], "pick": dict(base)}
            downs[pol] = server._rl_live_cube({})
        txt = " · ".join(f"{k:g} mm → {v.get('insert_down_cm')} cm (마커 높이 {v.get('marker_drop_cm')})" for k, v in downs.items())
        check("마커 40 · 37 mm (정책 높이와 비슷) → 손 내리기 = 기본", downs[40.0]["insert_down_cm"] == _RR.INSERT_DOWN_CM == downs[37.0]["insert_down_cm"]
              and abs(downs[40.0]["marker_drop_cm"]) < 0.2, txt)
        check("마커가 작을수록 손을 더 내림 (마커 높이만큼)", downs[36.0].get("ok") and _RR.INSERT_DOWN_CM < downs[36.0]["insert_down_cm"] < downs[35.0]["insert_down_cm"] <= server.RL_MARKER_DOWN_MAX_CM
              and abs(downs[36.0]["insert_down_cm"] - downs[36.0]["marker_drop_cm"] - server.RL_MARKER_DOWN_EXTRA_CM) < 0.11, txt)
        xp, yp, _ = server._rl_reproject(base["cube"]["x"], base["cube"]["y"], 0.5575, server.RL_TABLE_D)
        r35, r40 = downs[35.9]["robot"], downs[40.0]["robot"]
        check("마커 35.9 mm (손 보정 맞춘 높이) → 자리 예전 그대로", abs(r35["x"] - xp) < 0.002 and abs(r35["y"] - yp) < 0.002,
              f"({r35['x']}, {r35['y']}) vs ({xp:.4f}, {yp:.4f})")
        check("마커 40 mm (4.75 cm 더 높음) → 같은 시선을 따라 카메라 쪽으로 1~3 cm", 0.01 < ((r40["x"] - xp) ** 2 + (r40["y"] - yp) ** 2) ** 0.5 < 0.03,
              f"({r40['x'] - xp:+.4f}, {r40['y'] - yp:+.4f}) m")
        check("마커 아주 작음 → 상한까지만 + 경고", downs[33.0].get("ok") and downs[33.0]["insert_down_cm"] == server.RL_MARKER_DOWN_MAX_CM
              and any("까지만" in w for w in downs[33.0].get("warn", [])), txt)
        live_api.LIVE = saved_live
        live_api.cubes = fake({"ok": False, "reason": "큐브 마커(ID 0)가 안 보임"}, [])
        r = server._rl_live_cube({})
        check("큐브 못 찾음 → 이유", not r.get("ok") and "못 찾음" in r.get("error", ""), r.get("error", "")[:60])
    finally:
        live_api.cubes = real


# ---------------------------------------------------------------- 2) 끝에서 끝 (가상 모드 시험 서버)

def snap():
    """2026-11-26 '정책이 거부됨': 학습 범위 끝에서 조금 벗어난 카메라 자리 → 안쪽으로, 차이는 밀어 넣기로."""
    import rl_run
    inf = rl_run.info(os.environ["JAX_IK_DIR"])
    check("범위 안 자리 → 그대로", rl_run.snap_spot(inf, -5.0, -9.0) == (-5.0, -9.0))
    s = rl_run.snap_spot(inf, -7.0, -21.5)
    check("1 cm 밖 (05:43 실물에서 거절된 자리) → 3 cm 안 안쪽", s is not None and rl_run.spot_valid(inf, *s) and ((s[0] + 7.0) ** 2 + (s[1] + 21.5) ** 2) ** 0.5 <= 3.0, str(s))
    check("멀리 밖 → None (옮기지 않음)", rl_run.snap_spot(inf, -14.0, -21.0) is None)

def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def e2e():
    import rl_run
    port = free_port()
    env = dict(os.environ, JAX_IK_DIR=os.environ.get("JAX_IK_DIR") or str(Path.home() / "Desktop" / "IK"))
    log = open(Path("/tmp") / f"try_rl_live_cube_{port}.log", "w")
    proc = subprocess.Popen([sys.executable, str(HERE / "server.py"), "--virtual", "--port", str(port)], cwd=str(HERE), env=env,
                            stdout=log, stderr=subprocess.STDOUT)
    base = f"http://127.0.0.1:{port}"

    def call(path, body=None, timeout=300):
        data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
        req = urllib.request.Request(base + path, data=data, headers={"Content-Type": "application/json", "Origin": base})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as f:
                return f.status, json.loads(f.read() or b"null")
        except urllib.error.HTTPError as e:
            return e.code, None
    try:
        for _ in range(120):
            try:
                if call("/api/state", timeout=2)[0] == 200:
                    break
            except Exception:
                time.sleep(0.5)
        call("/api/live/scene", {"op": "clear"})
        st, r = call("/api/rl/live_cube", {})
        check("큐브 없음 → ok False", st == 200 and r and not r.get("ok"), (r or {}).get("error", "")[:60])
        for body in (b"[]", b"null", b'"x"', b"{"):
            st, _ = call("/api/rl/live_cube", body)
            check(f"이상한 본문 {body!r} → 500 아님", st != 500, f"HTTP {st}")
        call("/api/live/scene", {"op": "add", "x": -0.018, "y": -0.265, "yaw_deg": 20})
        call("/api/live/scene", {"op": "add", "x": 0.05, "y": -0.30, "yaw_deg": 70})      # 1번: yaw 70 → −20 으로 접혀야
        st, r = call("/api/rl/live_cube", {"reach_fix_cm": 0, "side_fix_cm": 0})
        want = rl_run.spot_from_robot(-0.018, -0.265, 20)
        check("큐브 0 → 위치 = spot_from_robot", st == 200 and r.get("ok") and _pre_snap(r) == want and r.get("id") == 0, str((r or {}).get("spot")))
        st, r1 = call("/api/rl/live_cube", {"pick": 1, "reach_fix_cm": 0, "side_fix_cm": 0})
        check("pick 1 → 큐브 1, 각 70° → −20° 로 접음", st == 200 and r1.get("ok") and r1.get("id") == 1 and abs(r1["spot"]["yaw_deg"] + 20) < 1e-6,
              str((r1 or {}).get("spot")))
        st, r = call("/api/rl/live_cube", {})      # 기본 = RL_REACH_FIX_M 만큼 앞으로 고친 위치로 집기까지
        if (HERE.parent.parent / "IK").exists() or os.environ.get("JAX_IK_DIR"):
            st, run = call("/api/rl/run", dict(r["spot"], home=False), timeout=400)
            sm = (run or {}).get("summary") or {}
            check("앞으로 고친 위치(기본 RL_REACH_FIX_M)로 강화학습이 집어 들기 끝까지 (가상, 들고 ㄴ 자 유지)", st == 200 and run.get("ok") and sm.get("done") and sm.get("valid_spot"),
                  f"done {sm.get('done')} {sm.get('fail_reason')}")
    finally:
        proc.terminate()
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()


if __name__ == "__main__":
    unit()
    snap()
    e2e()
    print("통과" if not FAIL else f"실패 {len(FAIL)}: {FAIL}")
    sys.exit(1 if FAIL else 0)
