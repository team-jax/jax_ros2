#!/usr/bin/env python3
"""카메라 큐브 → 강화학습 (POST /api/rl/live_cube, 강화학습 탭 '📷 카메라 큐브로 강화학습 돌리기') 시험.

  .venv/bin/python dashboard/tools/try_rl_live_cube.py

1) 판단 (서버 _rl_live_cube, 가짜 인식 결과): 받침대 없이 낮은 큐브(뎁스 −8 cm · 마커 34 mm)는 막고 이유를 말한다, 정상은 위치로 바꾼다.
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
        check("정상 (뎁스 높이 맞음) → 위치", r.get("ok") and r["spot"] == {"x_cm": 2.0, "y_cm": -3.0, "yaw_deg": 20.0}, str(r.get("spot") or r))
        live_api.cubes = fake(dict(base, blocked=True, block="ArUco·뎁스가 안 맞음", depth={"height_cm": -8.1}), [{"id": 1, "marker_mm": 39.0}])
        r = server._rl_live_cube({})
        check("뎁스가 8 cm 아래 → 막고 받침대 안내", not r.get("ok") and "받침대" in r.get("error", ""), r.get("error", "")[:70])
        live_api.cubes = fake(dict(base, depth={}), [{"id": 1, "marker_mm": 34.0}])
        r = server._rl_live_cube({})
        check("ArUco 만 · 마커 34 mm → 막고 받침대 안내", not r.get("ok") and "받침대" in r.get("error", ""), r.get("error", "")[:70])
        live_api.cubes = fake(dict(base, depth={}), [{"id": 1, "marker_mm": 38.5}])
        check("ArUco 만 · 마커 38.5 mm → 씀", server._rl_live_cube({}).get("ok"))
        live_api.cubes = fake({"ok": False, "reason": "큐브 마커(ID 0)가 안 보임"}, [])
        r = server._rl_live_cube({})
        check("큐브 못 찾음 → 이유", not r.get("ok") and "못 찾음" in r.get("error", ""), r.get("error", "")[:60])
    finally:
        live_api.cubes = real


# ---------------------------------------------------------------- 2) 끝에서 끝 (가상 모드 시험 서버)
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
        st, r = call("/api/rl/live_cube", {})
        want = rl_run.spot_from_robot(-0.018, -0.265, 20)
        check("큐브 0 → 위치 = spot_from_robot", st == 200 and r.get("ok") and r["spot"] == want and r.get("id") == 0, str((r or {}).get("spot")))
        st, r1 = call("/api/rl/live_cube", {"pick": 1})
        check("pick 1 → 큐브 1, 각 70° → −20° 로 접음", st == 200 and r1.get("ok") and r1.get("id") == 1 and abs(r1["spot"]["yaw_deg"] + 20) < 1e-6,
              str((r1 or {}).get("spot")))
        if (HERE.parent.parent / "IK").exists() or os.environ.get("JAX_IK_DIR"):
            st, run = call("/api/rl/run", dict(r["spot"]), timeout=400)
            sm = (run or {}).get("summary") or {}
            check("그 위치로 강화학습이 집기 끝까지 (가상)", st == 200 and run.get("ok") and sm.get("done") and sm.get("valid_spot"),
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
    e2e()
    print("통과" if not FAIL else f"실패 {len(FAIL)}: {FAIL}")
    sys.exit(1 if FAIL else 0)
