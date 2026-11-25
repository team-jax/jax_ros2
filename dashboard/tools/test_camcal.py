#!/usr/bin/env python3
"""카메라 보정 (역기구학으로, camcal.py) 시험 — 팔·CAN·폰은 안 건드린다.

  .venv/bin/python dashboard/tools/test_camcal.py [inference.png]

1) 맞추기: 알려진 회전·이동을 넣은 가짜 짝에서 되찾나, 1 개 = 이동만, 틀린 짝은 저장 거절.
2) 적용: cam_fix.json 이 있으면 live_api 가 실물 프레임 큐브 x·y·yaw 를 고치고 raw_xy 를 남기나 (그림이 주어지면 그 프레임으로).
3) 끝에서 끝 (가상 모드 시험 서버): 가상 큐브를 놓고 /api/camcal/read → fit → apply → reset (500 없음).
"""
import json, math, os, shutil, socket, subprocess, sys, tempfile, time, urllib.request
from pathlib import Path
HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import camcal  # noqa: E402
FAIL = []


def check(name, ok, detail=""):
    print(("✓ " if ok else "✗ ") + name + (f" — {detail}" if detail else ""), flush=True)
    if not ok:
        FAIL.append(name)


def unit():
    th, to = math.radians(4.0), (0.012, -0.021)            # 카메라가 4° 돌고 1.2·−2.1 cm 밀림
    targets = [(-0.05, -0.24), (0.05, -0.24), (-0.15, -0.26), (-0.05, -0.30), (0.02, -0.28)]
    def cam(x, y):   # 정답 → 카메라가 읽는 값 (보정의 반대)
        x0, y0 = x - to[0], y - to[1]
        c, s = math.cos(-th), math.sin(-th)
        return c * x0 - s * y0, s * x0 + c * y0
    pairs = [{"target": list(t), "raw": list(cam(*t))} for t in targets]
    f = camcal.fit(pairs)
    check("4° 돌고 밀린 카메라 → 회전 4° 되찾음", abs(f["theta_deg"] - 4.0) < 0.01, f"{f['theta_deg']}°")
    check("맞춘 뒤 오차 0", f["rms_fit_cm"] < 0.01 and f["rms_raw_cm"] > 1.0, f"{f['rms_raw_cm']} → {f['rms_fit_cm']} cm")
    x, y = camcal.apply_xy(f, *cam(0.1, -0.35))
    check("짝에 없던 자리도 맞음", math.hypot(x - 0.1, y + 0.35) < 1e-4, f"({x:.4f}, {y:.4f})")
    check("저장해도 됨 (문제 없음)", camcal.check_fit(f) == [], str(camcal.check_fit(f)))
    f1 = camcal.fit(pairs[:1])
    check("짝 1 개 → 이동만", f1["theta_deg"] == 0.0 and f1["rms_fit_cm"] < 1e-6)
    bad = pairs[:4] + [{"target": [0.02, -0.28], "raw": [0.10, -0.20]}]      # 다른 큐브를 읽은 짝
    fb = camcal.fit(bad)
    check("틀린 짝 섞이면 저장 거절", camcal.check_fit(fb) != [], "; ".join(camcal.check_fit(fb))[:80])
    c = {"x": 0.0, "y": -0.3, "yaw_deg": 44.0}
    camcal.apply_cube({"theta_deg": 3.0, "pivot": [0, 0], "to": [0, 0]}, c)
    check("큐브 각도도 돌림 (90° 주기로 접음)", abs(c["yaw_deg"] - (-43.0)) < 1e-6 and c["raw_xy"] == [0.0, -0.3], str(c))


def apply_live(img_path):
    import cv2
    import live_api, phone_depth as DP, phone_api
    tmp = Path(tempfile.mkdtemp())
    real_logs = HERE / "logs"
    for n in ("phone_depth.json", "phone_cam.json"):
        if (real_logs / n).exists():
            shutil.copy(real_logs / n, tmp / n)
    img = cv2.imread(str(img_path))
    if img is None:
        print("  (그림 없음 — 적용 시험 건너뜀)")
        return
    dcfg = DP.load_cfg(tmp / "phone_depth.json")
    img, _ = live_api.unrectify(live_api.left_panel(img), live_api.pipeline_calib(dcfg))
    outs = {}
    for tag, fx in (("없음", None), ("있음", {"theta_deg": 0.0, "pivot": [0, 0], "to": [0.01, -0.02], "n": 3})):
        if fx:
            (tmp / "cam_fix.json").write_text(json.dumps(fx))
        L = live_api.Live(tmp, lambda: "direct")
        L._process(img, time.time(), "inference.png", dcfg, phone_api)
        outs[tag] = L.res["cubes"]
    a, b = outs["없음"], outs["있음"]
    if not a:
        print("  (이 프레임에 큐브 없음 — 적용 시험 건너뜀)")
        return
    d = (round(b[0]["x"] - a[0]["x"], 4), round(b[0]["y"] - a[0]["y"], 4))
    check("cam_fix.json 있으면 실물 프레임 큐브를 고침 (+1 cm, −2 cm)", d == (0.01, -0.02) and b[0].get("raw_xy") == [a[0]["x"], a[0]["y"]], f"차이 {d} · raw {b[0].get('raw_xy')}")
    check("cam_fix.json 없으면 그대로 (raw_xy 없음)", "raw_xy" not in a[0])
    shutil.rmtree(tmp, ignore_errors=True)


def e2e():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    log = open(f"/tmp/test_camcal_{port}.log", "w")
    proc = subprocess.Popen([sys.executable, str(HERE / "server.py"), "--virtual", "--port", str(port)], cwd=str(HERE), stdout=log, stderr=subprocess.STDOUT)
    base = f"http://127.0.0.1:{port}"
    def call(path, body=None):
        data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
        req = urllib.request.Request(base + path, data=data, headers={"Content-Type": "application/json", "Origin": base})
        try:
            with urllib.request.urlopen(req, timeout=60) as f:
                return f.status, json.loads(f.read() or b"null")
        except urllib.error.HTTPError as e:
            return e.code, None
    fix_file = None
    try:
        for _ in range(120):
            try:
                if call("/api/state")[0] == 200:
                    break
            except Exception:
                time.sleep(0.5)
        st, r = call("/api/camcal")
        check("GET /api/camcal", st == 200 and r["ok"] and len(r["targets"]) >= 3)
        call("/api/camcal/reset", {})
        call("/api/live/scene", {"op": "clear"})
        st, r = call("/api/camcal/read", {"target": [-0.05, -0.24]})
        check("큐브 없음 → 못 읽음 (500 아님)", st == 200 and not r["ok"], (r or {}).get("error", "")[:60])
        for t in ([-0.05, -0.24], [0.05, -0.24], [-0.15, -0.26]):
            call("/api/live/scene", {"op": "clear"})
            call("/api/live/scene", {"op": "add", "x": t[0] + 0.004, "y": t[1] - 0.003, "yaw_deg": 10})
            st, r = call("/api/camcal/read", {"target": t})
            check(f"가상 큐브 읽기 {t}", st == 200 and r["ok"] and abs(r["err_cm"] - 0.5) < 0.05, str(r)[:90])
        st, r = call("/api/camcal/fit", {"apply": False})
        check("맞추기 (저장 안 함)", st == 200 and r["ok"] and r["fit"]["n"] == 3 and not r["saved"], str((r or {}).get("fit", {}).get("rms_fit_cm")))
        st, r = call("/api/camcal/fit", {"apply": True})
        check("적용 → 저장", st == 200 and r["ok"] and r["saved"], (r or {}).get("msg", (r or {}).get("error", "")))
        st, r = call("/api/camcal")
        check("저장한 보정이 보임", r["fix"] and r["fix"]["n"] == 3)
        st, r = call("/api/camcal/reset", {"clear_fix": True})
        st, r2 = call("/api/camcal")
        check("보정 지우기", r["ok"] and r2["fix"] is None and r2["pairs"] == [])
        for body in (b"[]", b"null", b'"x"', b"{", b'{"target": "a"}', b'{"target": [1e9, 0]}', b'{"target": ["nan", 0]}'):
            for path in ("/api/camcal/read", "/api/camcal/fit", "/api/camcal/reset"):
                st, _ = call(path, body)
                if st >= 500:
                    check(f"이상한 본문 {path} {body!r} → 500 아님", False, f"HTTP {st}")
        check("이상한 본문 → 500 없음", True)
    finally:
        call("/api/camcal/reset", {"clear_fix": True})
        proc.terminate()
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()


if __name__ == "__main__":
    unit()
    apply_live(sys.argv[1] if len(sys.argv) > 1 else HERE.parent.parent / "StereoDepthSeg" / "phone_stereo" / "runs" / "dash" / "inference.png")
    e2e()
    print("통과" if not FAIL else f"실패 {len(FAIL)}: {FAIL}")
    sys.exit(1 if FAIL else 0)
