"""카메라 보정 (역기구학으로) — 2026-11-25 사용자 '카메라 보정 좀 더 똑디 해줘, 일단 역기구학으로'.

팔이 역기구학으로 큐브를 '정해진 자리' 에 놓으면 그 자리는 팔 좌표에서 정답이다. 그때 카메라가 읽은 자리와 짝을 모아
책상 면 위 2D 회전 + 이동 (Procrustes) 을 맞춘다: 정답 = R(θ)·(카메라 − 피벗) + 목표 중심.
pos_fix_m (x·y 를 같은 양 밀기) 은 카메라가 조금 돌아가 있으면 책상 자리마다 오차가 달라져 한 곳만 맞는다 — 이것은 돌아간 것까지 맞춘다.

    logs/cam_fix.json   {"theta_deg", "pivot": [x, y], "to": [x, y], "n", "rms_raw_cm", "rms_fit_cm", "t_saved"}
                        → live_api 가 실물 카메라 큐브(x, y, yaw)에만 적용 (가상 모드·시험 주입은 안 함). 없으면 그대로.

    GET  /api/camcal              지금 보정 · 모은 짝 · 추천 자리
    POST /api/camcal/read         {target: [x, y]} → 이 요청 뒤 READ_WAIT_S 지난 프레임에서 target 가까운 큐브를 읽어 짝으로 저장
    POST /api/camcal/fit          {apply: bool} → 짝으로 맞춘 결과 (apply 면 cam_fix.json 저장)
    POST /api/camcal/reset        {clear_fix: bool} → 짝 비우기 (clear_fix 면 보정도 지움)
좌표는 대시보드 실좌표 (+x 로봇 왼쪽, −y 앞, m).
"""
import json
import math
import threading
import time
from pathlib import Path

from fastapi import Request
from fastapi.responses import JSONResponse

TARGETS = [(-0.05, -0.24), (0.05, -0.24), (-0.15, -0.26), (-0.05, -0.30), (0.02, -0.28)]   # 추천 자리 (README 의 집기·놓기 되는 칸 안)
MAX_THETA_DEG = 15.0      # 이보다 크게 돌아야 맞으면 짝이 틀린 것 (다른 큐브를 읽음 등) — 저장 안 함
MAX_SHIFT_M = 0.10
MAX_RMS_FIT_CM = 3.0      # 맞춘 뒤에도 이만큼 넘게 남으면 저장 안 함
NEAR_M = 0.12             # 읽을 때 target 에서 이 안의 큐브만
READ_WAIT_S = 2.5         # since 뒤로 이만큼 지난 프레임이어야 (1 장/초 · 중앙값 3 장 — 팔이 비킨 뒤 프레임으로)
READ_TIMEOUT_S = 20.0

_lk = threading.Lock()
_FIX = {"key": None, "v": None}
PAIRS = []                # [{"target": [x, y], "raw": [x, y], "fixed": [x, y], "yaw_deg", "id", "t"}]
LOG_DIR = None


def fix_path(log_dir=None):
    return Path(log_dir or LOG_DIR) / "cam_fix.json"


def load_fix(log_dir=None):
    """cam_fix.json (mtime 캐시) → dict 또는 None. log_dir 도 LOG_DIR 도 없으면 None."""
    if log_dir is None and LOG_DIR is None:
        return None
    p = fix_path(log_dir)
    try:
        st = p.stat()
    except OSError:
        return None
    key = (str(p), st.st_mtime_ns, st.st_size)
    if _FIX["key"] != key:
        try:
            v = json.loads(p.read_text(encoding="utf-8"))
            ok = isinstance(v.get("theta_deg"), (int, float)) and len(v.get("pivot") or []) == 2 and len(v.get("to") or []) == 2
            _FIX.update(key=key, v=v if ok else None)
        except (OSError, ValueError, AttributeError):
            _FIX.update(key=key, v=None)
    return _FIX["v"]


def apply_xy(fix, x, y):
    if not fix:
        return x, y
    th = math.radians(float(fix["theta_deg"]))
    c, s = math.cos(th), math.sin(th)
    dx, dy = x - fix["pivot"][0], y - fix["pivot"][1]
    return fix["to"][0] + c * dx - s * dy, fix["to"][1] + s * dx + c * dy


def apply_cube(fix, cube):
    """cube dict (x, y, yaw_deg) 를 고친다 (제자리). 고치기 전 x·y 는 raw_xy 에."""
    if not fix or cube is None or "x" not in cube:
        return cube
    cube.setdefault("raw_xy", [cube["x"], cube["y"]])
    x, y = apply_xy(fix, float(cube["raw_xy"][0]), float(cube["raw_xy"][1]))
    cube["x"], cube["y"] = round(x, 4), round(y, 4)
    if cube.get("yaw_deg") is not None:
        cube["yaw_deg"] = round((float(cube["yaw_deg"]) + float(fix["theta_deg"]) + 45.0) % 90.0 - 45.0, 1)
    return cube


def fit(pairs):
    """짝 (raw → target) 으로 2D 회전 + 이동. 1 개면 이동만. → dict (theta_deg, pivot, to, n, rms_raw_cm, rms_fit_cm, shift_cm, rows)"""
    n = len(pairs)
    if n == 0:
        raise ValueError("짝이 없음")
    m = [(float(p["raw"][0]), float(p["raw"][1])) for p in pairs]
    t = [(float(p["target"][0]), float(p["target"][1])) for p in pairs]
    mc = (sum(a for a, _ in m) / n, sum(b for _, b in m) / n)
    tc = (sum(a for a, _ in t) / n, sum(b for _, b in t) / n)
    th = 0.0
    if n >= 2:
        sxx = sum((a - mc[0]) * (c - tc[0]) + (b - mc[1]) * (d - tc[1]) for (a, b), (c, d) in zip(m, t))
        sxy = sum((a - mc[0]) * (d - tc[1]) - (b - mc[1]) * (c - tc[0]) for (a, b), (c, d) in zip(m, t))
        th = math.atan2(sxy, sxx)
    fx = {"theta_deg": round(math.degrees(th), 3), "pivot": [round(mc[0], 5), round(mc[1], 5)], "to": [round(tc[0], 5), round(tc[1], 5)], "n": n}
    rows = []
    for (a, b), (c, d) in zip(m, t):
        x, y = apply_xy(fx, a, b)
        rows.append({"target": [c, d], "raw_err_cm": round(math.hypot(a - c, b - d) * 100, 2), "fit_err_cm": round(math.hypot(x - c, y - d) * 100, 2)})
    rms = lambda k: round(math.sqrt(sum(r[k] ** 2 for r in rows) / n), 2)
    fx.update(rms_raw_cm=rms("raw_err_cm"), rms_fit_cm=rms("fit_err_cm"), shift_cm=round(math.hypot(tc[0] - mc[0], tc[1] - mc[1]) * 100, 2))
    fx["rows"] = rows
    return fx


def check_fit(fx):
    """저장해도 되는지 → 문제 목록 (빈 목록 = 됨)."""
    bad = []
    if abs(fx["theta_deg"]) > MAX_THETA_DEG:
        bad.append(f"회전 {fx['theta_deg']:.1f}° 가 너무 큼 (±{MAX_THETA_DEG:g}°) — 다른 큐브를 읽었거나 놓기가 빗나감")
    if fx["shift_cm"] > MAX_SHIFT_M * 100:
        bad.append(f"이동 {fx['shift_cm']:.1f} cm 가 너무 큼 ({MAX_SHIFT_M * 100:g} cm)")
    if fx["rms_fit_cm"] > MAX_RMS_FIT_CM:
        bad.append(f"맞춘 뒤에도 오차 {fx['rms_fit_cm']:.1f} cm (허용 {MAX_RMS_FIT_CM:g} cm) — 놓기가 빗나간 자리가 있다: 짝을 비우고 다시")
    return bad


def _read_now(target, since, timeout_s=READ_TIMEOUT_S):
    """since(unix 초) 뒤 READ_WAIT_S 지난 실물 프레임에서 target 가까운 큐브. 가상 모드면 가상 장면 큐브."""
    import live_api
    end = time.time() + timeout_s
    last_why = "영상 없음"
    while time.time() < end:
        virtual = live_api.LIVE.mode() == "virtual"
        if virtual:
            cs, _ = live_api.SCENE.cubes_view()
            cand = [(c["x"], c["y"], c.get("yaw_deg"), c.get("id"), [c["x"], c["y"]]) for c in cs if c.get("level", 0) == 0]
        else:
            with live_api.LIVE.lock:
                res = live_api.LIVE.res
            if res is None or res.get("t_frame", 0) < since + READ_WAIT_S or time.time() - res.get("t_frame", 0) > live_api.LIVE.stale_s():
                last_why = "팔이 비킨 뒤의 새 프레임을 기다림 (폰 프로그램이 켜져 있나?)"
                time.sleep(0.3)
                continue
            cand = [(c["x"], c["y"], c.get("yaw_deg"), c.get("id"), c.get("raw_xy") or [c["x"], c["y"]])
                    for c in res.get("cubes", []) if c.get("id") is not None and not c.get("held") and c.get("level", 0) == 0]
        near = sorted(((math.hypot(x - target[0], y - target[1]), x, y, yaw, i, raw) for x, y, yaw, i, raw in cand), key=lambda r: r[0])
        if near and near[0][0] <= NEAR_M:
            _, x, y, yaw, i, raw = near[0]
            return {"ok": True, "fixed": [round(x, 4), round(y, 4)], "raw": [round(float(raw[0]), 4), round(float(raw[1]), 4)],
                    "yaw_deg": yaw, "id": i, "err_cm": round(math.hypot(x - target[0], y - target[1]) * 100, 2)}
        last_why = f"자리 {NEAR_M * 100:.0f} cm 안에 마커가 보이는 큐브가 없음" + (f" (가장 가까운 것 {near[0][0] * 100:.0f} cm)" if near else "")
        if virtual:
            break
        time.sleep(0.5)
    return {"ok": False, "error": "카메라로 못 읽음: " + last_why}


def _floats2(v, name):
    if not (isinstance(v, (list, tuple)) and len(v) == 2):
        raise ValueError(f"{name}: [x, y]")
    a, b = float(v[0]), float(v[1])
    if not (math.isfinite(a) and math.isfinite(b) and abs(a) < 1.0 and abs(b) < 1.0):
        raise ValueError(f"{name}: 숫자 (|값| < 1 m)")
    return [a, b]


def state():
    fx = load_fix()
    with _lk:
        pairs = [dict(p) for p in PAIRS]
    out = {"ok": True, "fix": fx, "pairs": pairs, "targets": [list(t) for t in TARGETS]}
    if pairs:
        out["fit"] = fit(pairs)
        out["fit_problems"] = check_fit(out["fit"])
    return out


def read(b):
    try:
        target = _floats2(b.get("target"), "target")
    except (TypeError, ValueError) as e:
        return {"ok": False, "error": str(e)}
    since = time.time()     # 서버 시계 (프레임 시각과 같은 시계) — 이 요청 뒤 READ_WAIT_S 지난 프레임으로 읽는다 (팔이 비킨 뒤)
    r = _read_now(target, since)
    if r.get("ok"):
        with _lk:
            PAIRS.append({"target": target, "raw": r["raw"], "fixed": r["fixed"], "yaw_deg": r["yaw_deg"], "id": r["id"], "t": time.time()})
            r["n"] = len(PAIRS)
    return r


def do_fit(b):
    with _lk:
        pairs = [dict(p) for p in PAIRS]
    try:
        fx = fit(pairs)
    except ValueError as e:
        return {"ok": False, "error": str(e)}
    bad = check_fit(fx)
    out = {"ok": True, "fit": fx, "problems": bad, "saved": False}
    if b.get("apply"):
        if bad:
            return {**out, "ok": False, "error": "저장 안 함: " + " / ".join(bad)}
        if len(pairs) < 3:
            return {**out, "ok": False, "error": f"짝이 {len(pairs)} 개 — 회전까지 믿으려면 3 개 이상"}
        keep = {k: fx[k] for k in ("theta_deg", "pivot", "to", "n", "rms_raw_cm", "rms_fit_cm")}
        keep["t_saved"] = time.strftime("%Y-%m-%d %H:%M:%S")
        p = fix_path()
        tmp = p.with_suffix(".tmp")
        tmp.write_text(json.dumps(keep, ensure_ascii=False, indent=1), encoding="utf-8")
        tmp.replace(p)
        out.update(saved=True, msg=f"카메라 보정 저장: 회전 {fx['theta_deg']:+.2f}° · 오차 {fx['rms_raw_cm']:.1f} → {fx['rms_fit_cm']:.1f} cm")
    return out


def reset(b):
    with _lk:
        PAIRS.clear()
    msg = "짝을 비움"
    if b.get("clear_fix"):
        try:
            fix_path().unlink()
            msg += " · 카메라 보정 지움 (보정 없이 원래 값)"
        except FileNotFoundError:
            pass
    return {"ok": True, "msg": msg}


def install(app, log_dir):
    global LOG_DIR
    LOG_DIR = Path(log_dir)
    import asyncio

    async def body(req):
        try:
            b = await req.json()
        except Exception:
            return {}
        return b if isinstance(b, dict) else {}

    @app.get("/api/camcal")
    def _state():
        return JSONResponse(state())

    @app.post("/api/camcal/read")
    async def _read(req: Request):
        return JSONResponse(await asyncio.to_thread(read, await body(req)))

    @app.post("/api/camcal/fit")
    async def _fit(req: Request):
        return JSONResponse(do_fit(await body(req)))

    @app.post("/api/camcal/reset")
    async def _reset(req: Request):
        return JSONResponse(reset(await body(req)))
