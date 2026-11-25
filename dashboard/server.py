"""JAX 팔 대시보드: 탭 3개(연결 / 각도 제어 / 역기구학) + 팔 3D 뷰.

    python3 dashboard/server.py                 # http://localhost:8080 (이 PC에서만), 실물 = 직접 CAN (ROS 없음)
    python3 dashboard/server.py --virtual       # 가상 팔(실물·CAN·ROS 없이, Windows 에서도)
    python3 dashboard/server.py --ros           # 실물을 ROS 노드로 (팀 ROS 버전: ak45_node 등)
    python3 dashboard/server.py --fake-hw       # 직접 CAN 을 가짜 AK45·STM32 (python-can virtual) 에 — 화면 시험용
    python3 dashboard/server.py --host 0.0.0.0  # 같은 네트워크의 폰·노트북에서도 (보호 없음, 주의)

모드 2개(+선택): 실물(직접 CAN, 기본) = direct_api.py / real_arm.py 가 이 프로세스에서 can0 을 직접 다룬다(ROS 없음).
가상 = virtual_arm.py. --ros 를 주면 '실물'이 아래 ROS 경로가 된다(팀 ROS 버전).
ROS 구독·발행은 서버 안이 아니라 자식 프로세스(ros_monitor_proc.py, /opt/ros/<distro> + <ws>/install 을 source)에서 돈다.
가상 모드에서는 자식 대신 virtual_arm.VirtualArm 이 같은 모양의 상태를 채운다(화면 위 스위치로 바꿈).
모터를 움직이는 길은 하나: 각도 제어 탭의 '보내기' -> POST /api/joint/send -> 여기서 관절명·범위·유한값·원점 검사
-> /arm_control/joint_command (arm_control_node 의 램프·calibration 클립, arm_homed 면 homing_guard 를 거침).
"""
import argparse
import asyncio
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect   # noqa: E402
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse   # noqa: E402
from fastapi.staticfiles import StaticFiles   # noqa: E402



def _own_logs():
    """서버 하나 = logs 폴더 하나. 먼저 뜬 서버가 dashboard/logs 를 쓰고(잠금 logs/.server.lock), 같은 PC 의 다른 서버(시험·가상)는
    logs/other_<포트> 를 쓴다 — 시험 서버의 계획(pick_dry_traj.json)이 실물 서버의 '지금 계획'이 되지 않게.
    JAX_DASH_LOGS 를 직접 주면 그대로. procman·services 가 import 될 때 그 변수를 읽으므로 그 전에 부른다(자식 프로세스도 물려받음)."""
    if os.environ.get("JAX_DASH_LOGS") or not sys.platform.startswith("linux"):
        return None
    import fcntl
    d = HERE / "logs"
    d.mkdir(parents=True, exist_ok=True)
    f = open(d / ".server.lock", "w")
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return f
    except OSError:
        f.close()
    port = os.environ.get("JAX_DASH_PORT") or "8080"
    for k, v in enumerate(sys.argv):
        if v == "--port" and k + 1 < len(sys.argv):
            port = sys.argv[k + 1]
        elif v.startswith("--port="):
            port = v[7:]
    port = re.sub(r"[^0-9A-Za-z_-]", "", port) or "x"
    os.environ["JAX_DASH_LOGS"] = str(d / f"other_{port}")
    print(f"다른 대시보드 서버가 {d} 를 쓰는 중 — 이 서버는 {d / ('other_' + port)} 를 쓴다", flush=True)
    return None


_LOGS_LOCK = _own_logs()      # 파일을 열어 둔 동안 잠금 유지 (프로세스가 끝나면 풀림)

from procman import IS_LINUX, LOG_DIR, ProcessManager, tail_file   # noqa: E402
from ros_bridge import RosBridge   # noqa: E402
from ros_monitor import HOMING_SRV   # noqa: E402  (rclpy 는 import 하지 않음 — 구독은 자식 프로세스)
from services import CALIBRATE_GUIDE, CAN_IFACE, CAN_UP_CMD, PICK_TRAJ, REPO, ROS_DISTRO, WS   # noqa: E402
import direct_api   # noqa: E402  (실물 직접 CAN: real_arm.py)
import procman   # noqa: E402

pm = ProcessManager()
ros = RosBridge()
can_state = {"available": IS_LINUX, "text": "Linux 아님" if not IS_LINUX else "확인 중"}
MODE = {"virtual": False, "error": "", "ros": False}   # ros: --ros 면 실물 = ROS, 아니면 실물 = 직접 CAN
VM = {"arm": None}


def _can_watch():
    while True:
        try:
            r = subprocess.run(["ip", "-details", "-json", "link", "show", CAN_IFACE],
                               capture_output=True, text=True, timeout=3)
            if r.returncode != 0:
                can_state.update(up=False, exists=False, text=f"{CAN_IFACE} 없음", state=None, bitrate=None)
            else:
                d = json.loads(r.stdout)[0]
                info = (d.get("linkinfo") or {}).get("info_data") or {}
                up = "UP" in (d.get("flags") or [])
                can_state.update(exists=True, up=up, state=info.get("state"),
                                 bitrate=(info.get("bittiming") or {}).get("bitrate"),
                                 text=f"{CAN_IFACE} {'UP' if up else 'DOWN'}"
                                      + (f" · {info.get('state')}" if info.get("state") else ""))
        except Exception as e:
            can_state.update(up=None, text=f"{CAN_IFACE} 확인 실패: {type(e).__name__}")
        time.sleep(2)


# ---------------------------------------------------------------- 가상 / 실물
def set_mode(virtual):
    """가상 모드 켜기/끄기. {ok, msg | error}."""
    virtual = bool(virtual)
    if virtual == MODE["virtual"]:
        return {"ok": True, "msg": "이미 " + ("가상" if virtual else "실물") + " 모드"}
    if virtual:
        if VM["arm"] is None:
            try:
                from virtual_arm import VirtualArm
                VM["arm"] = VirtualArm(log=pm.vlog)
            except Exception as e:
                MODE["error"] = f"가상 팔을 못 만듦: {type(e).__name__}: {e}"
                return {"ok": False, "error": MODE["error"]}
        if direct_api.DIRECT.connected():
            return {"ok": False, "error": "실물 팔이 연결돼 있음 — 연결 탭 ② 에서 팔 연결을 먼저 끊기(브레이크 0 A)"}
        r = pm.set_virtual(VM["arm"])
        if not r["ok"]:
            return r
        ros.stop()
        MODE.update(virtual=True, error="")
        return {"ok": True, "msg": "가상 모드: 실물 모터·CAN·ROS 와 상관없음"}
    VM["arm"].stop_trajectory()
    VM["arm"].stop_all()
    pm.set_virtual(None)
    MODE.update(virtual=False, error="")
    if not MODE["ros"]:
        return {"ok": True, "msg": "실물(직접 CAN) 모드: 연결 탭 ① CAN 켜기 → ② 팔 연결"}
    threading.Thread(target=ros.start, daemon=True).start()
    return {"ok": True, "msg": "실물(ROS) 모드: ROS 연결 시도 중"}


def mode_kind():
    return "virtual" if MODE["virtual"] else "ros" if MODE["ros"] else "direct"


def backend():
    k = mode_kind()
    return VM["arm"] if k == "virtual" else ros if k == "ros" else direct_api.DIRECT.backend()


def ros_possible():
    return Path(ros.setup_paths()[0]).is_file()


# ---------------------------------------------------------------- calibration / homing
HOMING_FILE = REPO / "arm_control" / "homing.json"
CALIB_FILE = REPO / "arm_control" / "calibration.json"
SERVO_RANGE = (0.0, 180.0)
_calib_cache = {"mtime": None, "data": None}


def _cmd_range(j):
    """보낼 수 있는 sim 범위 = sim_range_deg ∩ 모터 범위(서보 0~180, AK45 zero±limit)를 sim 으로 바꾼 것."""
    try:
        lo, hi = (float(v) for v in j["sim_range_deg"])
        if j["motor"] == "servo":
            mlo, mhi = SERVO_RANGE
        else:
            mlo, mhi = j["zero_deg"] - j["limit_deg"], j["zero_deg"] + j["limit_deg"]
        a, b = sorted(((mlo - j["zero_deg"]) / (j["sign"] * j["scale"]), (mhi - j["zero_deg"]) / (j["sign"] * j["scale"])))
        r = [max(lo, a), min(hi, b)]
        return r if r[0] <= r[1] else None
    except Exception:
        return None


def _calibration():
    """관절 표. 실물 직접 CAN 이 연결돼 있으면 그 팔이 쓰는 메모리 보정(저장 전 핀·영점·방향 포함)을 덮어쓴다 —
    화면 각·지금 자세·HOME 계획이 실제로 보내는 변환과 같게 (2026-11-23: 핀을 바꾸고 저장 전이면 화면이 파일 값으로 엉뚱한 각을 보였다)."""
    d = _calibration_file()
    try:
        if mode_kind() == "direct" and direct_api.DIRECT.connected():
            live = {j.joint: j for j in direct_api.DIRECT.arm.cal.joints}
            joints = []
            for row in d.get("joints") or []:
                j = live.get(row["joint"])
                if j is not None:
                    row = dict(row, id=j.id, name=j.name, sign=j.sign, scale=j.scale, zero_deg=j.zero_deg,
                               verified=j.verified, sim_range_deg=list(j.sim_range_deg))
                    row["cmd_range_deg"] = _cmd_range(row)
                joints.append(row)
            return dict(d, joints=joints, live=True)
    except Exception:
        pass
    return d


def _calibration_file():
    """calibration.json 의 관절 표(관절↔모터·ID·verified + 보낼 수 있는 범위). 파일이 바뀌면 다시 읽는다."""
    try:
        m = CALIB_FILE.stat().st_mtime
        if _calib_cache["mtime"] != m:
            d = json.loads(CALIB_FILE.read_text(encoding="utf-8"))
            joints = []
            for j in d.get("joints") or []:
                row = {k: j.get(k) for k in ("joint", "motor", "id", "name", "sign", "scale", "zero_deg",
                                             "sim_range_deg", "verified", "limit_deg", "note")}
                row["cmd_range_deg"] = _cmd_range(row)
                joints.append(row)
            _calib_cache.update(mtime=m, data={"file": str(CALIB_FILE), "joints": joints})
        return _calib_cache["data"]
    except Exception as e:
        return {"file": str(CALIB_FILE), "joints": [], "error": f"{type(e).__name__}: {e}"}


def _home_poses():
    try:
        d = json.loads(HOMING_FILE.read_text(encoding="utf-8"))
        return {str(k): float(v) for k, v in (d.get("home_sim_deg") or {}).items()}
    except Exception:
        return {}


def _homing_call(action, b):
    """action: home | release. b["joint"]: 관절 이름 또는 "all"."""
    joint = str((b if isinstance(b, dict) else {}).get("joint") or "").strip()
    snap_joints = [j.get("joint") for j in ((backend().snapshot().get("homing") or {}).get("joints") or [])]
    allowed = set(_home_poses()) | set(snap_joints)
    if joint != "all" and joint not in allowed:
        return {"ok": False, "error": f"모르는 관절 {joint!r} (허용: all, {', '.join(sorted(allowed)) or '없음'})"}
    return backend().call_trigger(f"{HOMING_SRV}{action}_{joint}", timeout=2.0)


def _to_sim(j, motor_deg):
    return (motor_deg - (j.get("zero_deg") or 0.0)) / (j["sign"] * j["scale"])


def actual_pose(snap, cal):
    """관절마다 지금 sim 각(°): AK45 = 피드백(arm_homed 면 guard 가 오프셋을 뺀 /ak45/joint_states_homed),
    서보 = /servo/state(마지막으로 보낸 값). {joint: {deg, src, age}}."""
    out = {}
    if not snap.get("available"):
        return out
    for j in cal.get("joints") or []:
        try:
            if j["motor"] == "ak45":
                key = f"ak45_{j['id']}"
                h = (snap.get("joints_homed") or {}).get(key)
                if h and h["age"] <= 1.0:
                    out[j["joint"]] = {"deg": _to_sim(j, h["deg"]), "src": "피드백(원점 보정)", "age": h["age"]}
                    continue
                v = (snap.get("joints") or {}).get(key)
                if v:
                    out[j["joint"]] = {"deg": _to_sim(j, v["deg"]), "src": "피드백", "age": v["age"]}
            else:
                v = (snap.get("servos") or {}).get(j["name"])
                if v:
                    out[j["joint"]] = {"deg": _to_sim(j, v["deg"]), "src": "보낸 값", "age": v["age"]}
        except Exception:
            continue
    return out


def cmd_gate(snap, cal, services):
    """보내기 버튼을 켜도 되는지. 화면과 POST /api/joint/send 가 같은 판정을 쓴다."""
    alive = {s["id"] for s in services if s["alive"]}
    g = {"ok": False, "reason": "", "guard": False, "homed_mode": False, "joints": {}}
    if not snap.get("available"):
        g["reason"] = snap.get("reason") if snap.get("direct") else f"{snap.get('reason') or 'ROS 없음'} — ROS 를 연결하거나 가상 모드로"
    elif (snap.get("cmd_subs") or 0) < 1:
        g["reason"] = ("arm_control_node 가 안 켜짐 (/arm_control/joint_command 구독자 없음) — 연결 탭에서 "
                       "arm_homed.launch.py / arm.launch.py / arm_control_node 켜기")
    else:
        g["ok"] = True
    h = snap.get("homing") or {}
    guard_fresh = bool(h.get("seen")) and (h.get("age") or 99) <= 3.0
    homed_mode = "arm_homed" in alive or guard_fresh
    g["guard"], g["homed_mode"] = guard_fresh, homed_mode
    homed = {x.get("joint"): bool(x.get("homed")) for x in (h.get("joints") or [])} if guard_fresh else {}
    servos = snap.get("servos") or {}
    jfb = snap.get("joints") or {}
    for j in cal.get("joints") or []:
        name, row = j["joint"], {"ok": g["ok"], "reason": "" if g["ok"] else "명령 받을 노드 없음", "note": ""}
        if g["ok"] and j["motor"] == "ak45":
            if homed_mode and not guard_fresh:
                row.update(ok=False, reason="homing_guard 상태가 안 옴 (arm_homed 켜짐인데 guard 응답 없음)")
            elif homed_mode and not homed.get(name):
                row.update(ok=False, reason="원점 없음 — 위 원점 패널에서 원점 잡기")
            elif f"ak45_{j['id']}" not in jfb:
                row["note"] = "피드백 없음: arm_control_node 가 피드백을 기다리며 안 움직인다"
        elif g["ok"] and j["motor"] == "servo" and j["name"] not in servos:
            row["note"] = "첫 명령: 서보 위치를 몰라 램프 없이 바로 그 각으로 간다"
        if j.get("cmd_range_deg") is None:
            row.update(ok=False, reason="calibration 범위가 비어 있음")
        g["joints"][name] = row
    return g


def _send_joints(b):
    joints = (b if isinstance(b, dict) else {}).get("joints")
    if not isinstance(joints, dict) or not joints:
        return {"ok": False, "error": "joints: {관절: sim 각(°)} 이 비었음"}
    cal = _calibration()
    by = {j["joint"]: j for j in cal.get("joints") or []}
    if len(joints) > len(by):
        return {"ok": False, "error": "관절이 너무 많음"}
    snap = backend().snapshot()
    gate = cmd_gate(snap, cal, pm.snapshot())
    if not gate["ok"]:
        return {"ok": False, "error": gate["reason"]}
    names, rads, shown = [], [], []
    for n, v in joints.items():
        if n not in by:
            return {"ok": False, "error": f"모르는 관절 {n!r}"}
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            return {"ok": False, "error": f"{n}: 숫자가 아님 ({v!r})"}
        lo, hi = by[n]["cmd_range_deg"] or (None, None)
        if lo is None or not (lo - 1e-6 <= v <= hi + 1e-6):
            return {"ok": False, "error": f"{n}: {v:g}° 가 보낼 수 있는 범위 {lo}~{hi}° 밖"}
        jg = gate["joints"].get(n) or {}
        if not jg.get("ok"):
            return {"ok": False, "error": f"{n}: {jg.get('reason') or '못 보냄'}"}
        names.append(n)
        rads.append(math.radians(float(v)))
        shown.append(f"{n} {float(v):.1f}°")
    fn = backend().send_joint_command
    instant = (b if isinstance(b, dict) else {}).get("instant") is True
    import inspect
    if instant and "instant" in inspect.signature(fn).parameters:   # 직접 CAN: 서보를 램프 없이 바로 (가상·ROS 는 늘 램프)
        r = fn(names, rads, instant=True)
    else:
        r = fn(names, rads)
    if r.get("ok"):
        r["msg"] = (r.get("msg") or "보냄") + (" (서보 바로)" if instant else "") + " — " + ", ".join(shown)
    return r


# ---------------------------------------------------------------- 3D: 메시 / 궤적
def _ik_dir():
    for c in (os.environ.get("JAX_IK_DIR"), REPO.parent / "IK", Path(WS) / "src" / "IK"):
        if c and (Path(c) / "motion2" / "arm" / "meshes").is_dir():
            return Path(c)
    return None


def robot_info():
    d = _ik_dir()
    mesh = d / "motion2" / "arm" / "meshes" if d else None
    ok = bool(mesh and (mesh / "Upper_Arm_1.obj").is_file())
    return {"mesh_ok": ok, "mesh_dir": str(mesh) if mesh else None,
            "looked": [os.environ.get("JAX_IK_DIR") or "(JAX_IK_DIR 없음)", str(REPO.parent / "IK"), str(Path(WS) / "src" / "IK")]}


def _trajectory():
    it = pm.items.get("pick_dry")
    params = dict(it.params) if it else {}
    out = {"ok": False, "params": params, "running": bool(it and it.alive()),
           "exit_code": it.exit_code if it else None, "file": str(PICK_TRAJ)}
    if not PICK_TRAJ.is_file():
        out["error"] = "궤적 파일 없음 (pick_place 가 아직 안 돌았거나, 팔이 못 닿아 계획이 없음)"
        return out
    try:
        d = json.loads(PICK_TRAJ.read_text(encoding="utf-8"))
    except Exception as e:
        out["error"] = f"궤적 파일을 못 읽음: {e}"
        return out
    keep = ("real", "sim", "status", "reason", "playable", "delivered_in_sim", "retries", "final_error_m",
            "joint_names", "dt", "positions", "unit", "peak_speed_deg_s", "reach_map",
            "planner", "waypoints", "marks", "checks", "problems", "grasp")   # marks = 재생 중 지금 무슨 동작 (play_phase). planner = "analytic" 이면 대시보드 계획 (pick_plan.py, 물리 없음)
    out.update(ok=True, traj={k: d.get(k) for k in keep if k in d}, mtime=PICK_TRAJ.stat().st_mtime)
    if not d.get("positions"):
        out.update(ok=False, error=f"궤적 없음: {d.get('status')} ({d.get('reason')})")
    return out


def _play_virtual(trajectory=None):
    """trajectory: 궤적을 주는 함수 (기본 = 역기구학 탭 계획 _trajectory)."""
    if not MODE["virtual"]:
        return {"ok": False, "error": "가상 모드에서만 (실물 재생은 대시보드에 없음)"}
    t = (trajectory or _trajectory)()
    if not t["ok"]:
        return {"ok": False, "error": t.get("error") or "궤적 없음"}
    traj = t["traj"]
    try:
        from arm_control import calibration as C
        from arm_control.play import check_trajectory
        fatal, problems = check_trajectory(traj, C.load())
    except Exception as e:
        fatal, problems = None, [f"검사 못 함: {e}"]
    if fatal:
        return {"ok": False, "error": f"재생 거절: {fatal}"}
    try:
        speed = float(t["params"].get("max_speed") or 30.0)
    except ValueError:
        speed = 30.0
    import live_api
    r = VM["arm"].play_trajectory(traj, speed, on_end=live_api.SCENE.on_play(traj, "역기구학 계획"))   # 끝까지 가면 가상 장면의 큐브를 옮긴다
    if r.get("ok"):
        run_record.REC.on_play(traj, t.get("params"), arm=VM["arm"], virtual=True, extra={"problems": problems, "speed": speed})
    if r.get("ok") and problems:
        r["msg"] += f" (검사 {len(problems)}건은 실물이면 --force 가 필요: 가상이라 재생)"
    return r


def _direct_can():
    c = direct_api.DIRECT.cfg
    return {"available": True, "up": True, "exists": True, "state": "ERROR-ACTIVE", "up_cmd": CAN_UP_CMD,
            "text": f"{c['channel']} ({c['interface']}{' · 가짜 AK45·STM32' if c['fake'] else ''})"}


def state():
    b = backend()
    snap = b.snapshot() if b is not None else ros.snapshot()
    cal = _calibration()
    services = pm.snapshot()
    return {
        "t": time.time(),
        "mode": {"virtual": MODE["virtual"], "kind": mode_kind(), "ros_enabled": MODE["ros"],
                 "error": MODE["error"], "ros_possible": ros_possible(),
                 "ros_setup": ros.setup_paths()[0]},
        "services": services,
        "strays": pm.strays,
        "ros": snap,
        "can": _direct_can() if mode_kind() == "direct" and direct_api.DIRECT.cfg["interface"] != "socketcan" else
               dict(can_state, up_cmd=CAN_UP_CMD) if not MODE["virtual"] else
               {"available": True, "up": True, "state": "ERROR-ACTIVE", "text": "can0 (가상) UP", "up_cmd": CAN_UP_CMD},
        "calibrate": CALIBRATE_GUIDE,
        "home_poses": _home_poses(),
        "calibration": cal,
        "pose": actual_pose(snap, cal),
        "gate": cmd_gate(snap, cal, services),
        "robot": robot_info(),
        "traj_mtime": PICK_TRAJ.stat().st_mtime if PICK_TRAJ.is_file() else None,
        "info": {"ws": WS, "distro": ROS_DISTRO, "repo": str(REPO), "logs": str(LOG_DIR),
                 "platform": sys.platform},
    }


app = FastAPI(title="JAX dashboard")
app.mount("/static", StaticFiles(directory=str(HERE / "static")), name="static")


@app.middleware("http")
async def _no_stale_static(request, call_next):
    # 화면 파일(app.js, viewer.js …)을 고친 뒤 브라우저가 옛 것을 쓰지 않게: 매번 확인(바뀌지 않았으면 304)
    resp = await call_next(request)
    if request.url.path.startswith("/static/") and "/vendor/" not in request.url.path:
        resp.headers["Cache-Control"] = "no-cache"
    return resp


@app.get("/")
def index():
    return FileResponse(str(HERE / "static" / "index.html"))


@app.get("/rl")
def rl_page():
    """강화학습 돌리기 (역기구학 집기와 따로, 시뮬 전용): static/rl.html"""
    return FileResponse(str(HERE / "static" / "rl.html"))


@app.get("/api/rl/info")
async def api_rl_info():
    import rl_run
    r = await asyncio.to_thread(rl_run.info, _ik_dir())
    return JSONResponse({**r, "robot": robot_info()})   # robot.mesh_ok: 3D 메시를 줄 수 있나 (없으면 막대 모델)


@app.post("/api/rl/run")
async def api_rl_run(req: Request):
    import rl_run
    b = await _body(req)
    if isinstance(b, dict):
        b = dict(b, limits_deg=_rl_real_limits())
    return JSONResponse(await asyncio.to_thread(rl_run.run, _ik_dir(), b))


@app.post("/api/rl/vision")
async def api_rl_vision(req: Request):
    """카메라 프로그램의 inference.json 에서 큐브 하나 → 로봇 틀 위치·RL 위치 오프셋 (ROS 없이, vision_node 의 순수 함수)."""
    import rl_run
    return JSONResponse(await asyncio.to_thread(rl_run.read_vision, await _body(req)))


def _rl_real_limits():
    """실물 보정(zero_deg·모터 범위)이 허용하는 sim 각도 범위 {관절: [lo°, hi°]}. 연결돼 있으면 그 팔의 메모리 보정, 아니면 저장 파일.
    RL 의 '놓을 자리까지 옮기기' IK 가 이 안에서만 풀게 넘긴다 (안 그러면 손목을 −74° 비틀어 실물 재생이 막힌다)."""
    try:
        out = {}
        if mode_kind() == "direct" and direct_api.DIRECT.connected():
            for j in direct_api.DIRECT.arm.cal.joints:
                lo, hi = j.motor_bounds()
                a, b = sorted((j.to_sim(lo), j.to_sim(hi)))
                out[j.joint] = [max(a, j.sim_range_deg[0]), min(b, j.sim_range_deg[1])]
        else:
            for j in json.loads(CALIB_FILE.read_text(encoding="utf-8"))["joints"]:
                r = _cmd_range(j)
                if r:
                    out[j["joint"]] = r
        return out
    except Exception:
        return {}


RL_CLIP_TOL_DEG = 1.0   # 실물 보정 범위 밖으로 이만큼(°) 넘게 잘리는 RL 궤적은 실물로 안 보낸다 (그 아래는 부동소수점·0° 근처 정도)


def _rl_clip_split(traj, cal):
    """실물 보정 범위(calibration)에 잘리는 관절을 (크게 잘림, 조금 잘림)으로 나눔. 조금 = RL_CLIP_TOL_DEG 이하."""
    from arm_control.play import clipped_joints
    big, small = [], []
    for name, lo, hi, allowed in clipped_joints(traj, cal):
        over = max(allowed[0] - lo, hi - allowed[1], 0.0)
        (big if over > RL_CLIP_TOL_DEG else small).append((name, lo, hi, allowed, over))
    return big, small


@app.get("/api/rl/real_state")
def api_rl_real_state(run_id: str = "", grip_gain: str = "", grip_add_deg: str = ""):
    """RL 탭의 '실물로 재생' 카드: 지금 모드·연결·원점·서보·첫 프레임과의 거리·재생 상태 (읽기만, 아무것도 안 보낸다)."""
    import rl_run
    kind = mode_kind()
    out = {"mode": kind, "connected": False, "can_play": False, "why": ""}
    if kind != "direct":
        out["why"] = "실물(직접 CAN) 모드에서만 — 지금은 " + ("가상" if kind == "virtual" else "ROS") + " 모드"
        return out
    if not direct_api.DIRECT.connected():
        out["why"] = "팔 연결 안 됨 — 연결 탭 ② 팔 연결"
        return out
    arm = direct_api.DIRECT.arm
    out["connected"] = True
    try:
        out["not_homed"] = [n for n, j in arm.joints.items() if j.motor == "ak45" and not arm.homing.m[f"ak45_{j.id}"].homed]
        out["unsent_servos"] = [n for n, j in arm.joints.items() if j.motor == "servo" and j.name not in arm.servo_sent]
        out["playback"] = dict(arm.play)
        out["max_speed"] = float(arm.p["max_speed"])
        out["calib_dirty"] = bool(arm.calib_dirty)
        res = rl_run.get_run(run_id) if run_id else None
        if res:
            try:
                grip = rl_run.grip_params(grip_gain, grip_add_deg)
            except ValueError as e:
                grip = None
                out["grip_error"] = str(e)
            t = rl_run.real_traj(res, grip=grip, limits=_rl_real_limits())     # 재생할 때와 같은 궤적으로 검사
            out["grip_extra_deg"] = t.get("grip_extra_deg")
            out["far"] = arm._start_far(t["joint_names"], t["positions"][0], direct_api.R.START_TOL_DEG)   # play_trajectory 가 거절하는 기준과 같게
            big, small = _rl_clip_split(t, arm.cal)
            out["clip_big"] = [f"{n} {lo:.0f}..{hi:.0f}° (허용 {al[0]:.0f}..{al[1]:.0f}°)" for n, lo, hi, al, _ in big]
    except Exception as e:     # 상태 표시용이라 실패해도 화면이 죽지 않게
        out["why"] = f"상태를 읽다 실패: {type(e).__name__}: {e}"
        return out
    out["can_play"] = not (out["not_homed"] or out["unsent_servos"] or out.get("far") or out.get("clip_big") or out["calib_dirty"]
                           or (out["playback"] or {}).get("running"))
    return out


def _rl_play_real(b):
    """강화학습 시뮬 궤적을 실물 팔로 재생한다 — 역기구학 실물 재생(direct_api.play_real)과 같은 안전 경로에 RL 전용 거절을 앞에 더한 것.
    열린 고리: 시뮬 관절 궤적을 그대로 따라갈 뿐 실물 관측(큐브 위치·관절 피드백)으로 정책을 다시 돌리지 않는다."""
    import rl_run
    from arm_control.play import check_trajectory
    if mode_kind() != "direct":
        return {"ok": False, "error": "실물(직접 CAN) 모드에서만 재생 (가상·ROS 모드는 안 됨)"}
    if not direct_api.DIRECT.connected():
        return {"ok": False, "error": "팔 연결 안 됨 (직접 CAN) — 연결 탭 ② 팔 연결"}
    res = rl_run.get_run((b or {}).get("run_id"))
    if not res:
        return {"ok": False, "error": "화면에 보인 RL 결과를 서버에서 못 찾음 (서버를 다시 켰거나 오래됨) — 다시 돌린 뒤 재생"}
    if not (b or {}).get("ack_table"):
        return {"ok": False, "error": "책상 위치 확인이 필요함: 정책은 책상 앞 끝이 어깨에서 22.5 cm, 윗면이 어깨 아래 47.75 cm 인 장면에서 학습했다"}
    sm = res.get("summary") or {}
    if not sm.get("done"):
        return {"ok": False, "error": f"시뮬에서 실패한 실행은 실물로 안 보냄 ({sm.get('fail_reason') or '끝내 못 마침'})"}
    if sm.get("valid_spot") is False:
        return {"ok": False, "error": "정책이 학습한 범위 밖 지점 — 실물로 안 보냄 (지도의 초록 칸 안에서)"}
    arm = direct_api.DIRECT.arm
    gen = arm.stop_gen          # 계획 사이 STOP 이 눌리면 play_trajectory 가 거절
    try:
        grip = rl_run.grip_params((b or {}).get("grip_gain"), (b or {}).get("grip_add_deg"))
    except (TypeError, ValueError) as e:
        return {"ok": False, "error": str(e)}
    traj = rl_run.real_traj(res, grip=grip, limits=_rl_real_limits())   # 실물만 손가락을 더 쥐게 (rl_run.squeeze_grip)
    big, small = _rl_clip_split(traj, arm.cal)
    if big:
        return {"ok": False, "error": "실물 보정 범위 밖이라 궤적이 잘림 → 재생 안 함 (force 로도 안 됨): " +
                "; ".join(f"{n} {lo:.0f}..{hi:.0f}° (허용 {al[0]:.0f}..{al[1]:.0f}°)" for n, lo, hi, al, _ in big) +
                " — 큐브 각도나 위치를 바꿔 다시"}
    fatal, problems = check_trajectory(traj, arm.cal)
    trivial = {n for n, *_ in small}      # 1° 이하로 잘리는 것은 경고로 안 올림 (0° 근처 부동소수점)
    problems = [p for p in direct_api.drop_unverified(problems) if not any(p.startswith(n + " goes") for n in trivial)]
    if fatal:
        return {"ok": False, "error": f"재생 거절: {fatal}"}
    if problems and not (b or {}).get("force"):
        return {"ok": False, "need_force": True, "problems": problems,
                "error": f"검사 {len(problems)}건 (--force 가 필요한 것): " + " / ".join(problems)}
    try:
        speed = float((b or {}).get("max_speed") or arm.p["max_speed"])
    except (TypeError, ValueError):
        return {"ok": False, "error": "max_speed 가 숫자가 아님"}
    if not 1.0 <= speed <= 90.0:
        return {"ok": False, "error": "max_speed 는 1~90 °/s"}
    r = arm.play_trajectory(traj, speed, stop_gen=gen)
    if r.get("ok"):
        arm.play["run_id"] = str(b.get("run_id"))     # RL 탭 3D 가 이 실행의 몇 번째 프레임인지 따라가게
        import run_record                              # 실행 기록 (역기구학 실물 재생과 같게 logs/runs/ 에)
        run_record.REC.on_play(traj, dict(res.get("params") or {}, max_speed=speed, run_id=str(b.get("run_id"))), arm=arm,
                               extra={"force": bool(b.get("force")), "problems": problems, "speed": speed, "planner": "rl",
                                      "grip_gain": grip[0], "grip_add_deg": grip[1]})
    if r.get("ok") and problems:
        r["msg"] = (r.get("msg") or "재생 시작") + f" (검사 {len(problems)}건 무시하고 재생: force)"
    return r


@app.post("/api/rl/play_real")
async def api_rl_play_real(req: Request):
    return JSONResponse(await asyncio.to_thread(_rl_play_real, await _body(req)))


@app.get("/api/state")
def api_state():
    return state()


@app.middleware("http")
async def _same_origin_post(request, call_next):
    # 다른 사이트(브라우저에 열린 아무 웹 페이지)가 이 서버로 POST 해 팔을 움직이지 못하게: 브라우저가 붙이는 Origin 이
    # 이 서버 주소(Host)와 다르면 거절. Origin 없는 요청(curl·시험 스크립트)은 그대로. 폰·PC 화면은 같은 주소라 영향 없음.
    if request.method == "POST":
        origin = request.headers.get("origin")
        if origin is not None and origin.split("://", 1)[-1].lower() != (request.headers.get("host") or "").lower():
            return JSONResponse({"ok": False, "error": "다른 사이트에서 온 요청 — 거절"}, status_code=403)
    return await call_next(request)


async def _body(req):
    try:
        j = await req.json()
    except Exception:
        return {}
    return j if isinstance(j, dict) else {}   # "문자열"·[]·null 본문이 와도 핸들러의 .get 이 500 으로 죽지 않게 (2026-11-24 fuzz: go_home·rl/play_real)


@app.post("/api/svc/{sid}/{action}")
async def api_svc(sid: str, action: str, req: Request):
    b = await _body(req)
    params = b.get("params") or {}
    if not isinstance(params, dict):   # {"params": "cube_x"} 같은 본문이 500 으로 죽지 않게 (2026-11-25 검토)
        params = {}
    if action == "start":
        r = pm.start(sid, params)
    elif action == "stop":
        r = pm.stop(sid)
    elif action == "restart":
        r = await asyncio.to_thread(pm.restart, sid, params)
    else:
        r = {"ok": False, "error": f"없는 동작 {action}"}
    return JSONResponse(r)


@app.post("/api/stop")
def api_stop():
    if mode_kind() == "direct":     # 직접 CAN: STOP = 지금 자리 홀드 (팔 안 떨어짐). 노드 종료가 아님
        return direct_api.DIRECT.backend().hold_all("STOP")
    if MODE["virtual"] and VM["arm"] is not None:
        VM["arm"].stop_trajectory()
    return pm.stop_all()


@app.post("/api/mode")
async def api_mode(req: Request):
    b = await _body(req)
    return JSONResponse(await asyncio.to_thread(set_mode, bool(b.get("virtual"))))


@app.post("/api/ros/connect")
async def api_ros_connect():
    if MODE["virtual"]:
        return JSONResponse({"ok": False, "error": "가상 모드 — 실물(ROS)로 바꾼 뒤 연결"})
    return JSONResponse(await asyncio.to_thread(ros.start))


@app.post("/api/homing/home")
async def api_homing_home(req: Request):
    return JSONResponse(await asyncio.to_thread(_homing_call, "home", await _body(req)))


@app.post("/api/homing/release")
async def api_homing_release(req: Request):
    return JSONResponse(await asyncio.to_thread(_homing_call, "release", await _body(req)))


@app.post("/api/joint/send")
async def api_joint_send(req: Request):
    return JSONResponse(await asyncio.to_thread(_send_joints, await _body(req)))


@app.post("/api/virtual/power")
async def api_virtual_power(req: Request):
    if not MODE["virtual"]:
        return JSONResponse({"ok": False, "error": "가상 모드에서만"})
    return JSONResponse(VM["arm"].power(bool((await _body(req)).get("on"))))


@app.post("/api/virtual/cube")
async def api_virtual_cube(req: Request):
    if not MODE["virtual"]:
        return JSONResponse({"ok": False, "error": "가상 모드에서만"})
    b = await _body(req)
    if b.get("clear"):
        return JSONResponse(VM["arm"].set_cube(None))
    try:
        v = [float(b[k]) for k in ("x", "y", "z", "yaw")]
    except (KeyError, TypeError, ValueError):
        return JSONResponse({"ok": False, "error": "x, y, z, yaw 숫자 4개"})
    if not all(math.isfinite(x) for x in v) or max(abs(x) for x in v[:3]) > 2:
        return JSONResponse({"ok": False, "error": "값이 이상함 (|x|,|y|,|z| ≤ 2 m)"})
    return JSONResponse(VM["arm"].set_cube(v))


@app.get("/api/ik/trajectory")
def api_ik_traj():
    return JSONResponse(_trajectory())


@app.get("/api/reachmap")
def api_reachmap(yaw: float = 0.0):
    import reachmap
    r = reachmap.reachmap(_ik_dir(), yaw if math.isfinite(yaw) else 0.0)
    return JSONResponse(r, status_code=200 if r["ok"] else 404)


@app.post("/api/reachmap/check")
async def api_reachmap_check(req: Request):
    import reachmap
    r = reachmap.check(_ik_dir(), await _body(req))
    return JSONResponse(r, status_code=200 if r["ok"] else (404 if "없음" in r["error"] else 400))


def _hand_ik(b, op):
    """손 위치로 보내기: 관절 한계 = calibration 의 보낼 수 있는 범위, 시작점 = 지금 자세(가까운 해 우선)."""
    import hand_ik
    cal = _calibration()
    ranges = {j["joint"]: j["cmd_range_deg"] for j in cal.get("joints") or [] if j.get("cmd_range_deg")}
    b = b if isinstance(b, dict) else {}
    cur = {k: v["deg"] for k, v in actual_pose(backend().snapshot(), cal).items()}
    if op == "fk":
        js = b.get("joints") if isinstance(b.get("joints"), dict) else cur
        # 유한한 숫자만 (1e400 = inf 가 오면 FK 가 nan 을 돌려주고 JSON 응답 만들다 500 으로 죽었다)
        js = {str(k): float(v) for k, v in js.items() if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)}
        return hand_ik.fk(_ik_dir(), js, b.get("mode") or "auto") | {"joints_used": js}
    return hand_ik.solve(_ik_dir(), b, ranges, cur)


@app.post("/api/ik/hand")
async def api_ik_hand(req: Request):
    return JSONResponse(await asyncio.to_thread(_hand_ik, await _body(req), "solve"))


@app.post("/api/ik/pair")
async def api_ik_pair(req: Request):
    """지도 판정: 큐브 집기·놓을 자리 놓기가 같은 파지로 둘 다 풀리나 (도달 맵 대신 실제 IK)."""
    import hand_ik
    cal = _calibration()
    ranges = {j["joint"]: j["cmd_range_deg"] for j in cal.get("joints") or [] if j.get("cmd_range_deg")}
    return JSONResponse(await asyncio.to_thread(hand_ik.pair, _ik_dir(), await _body(req), ranges))


_GRASP_LOCK = threading.Lock()
_GRASP_CACHE = {}   # (관절 범위, shoulder_yaw, 책상 높이 d, 집는 방향, 큐브 yaw) -> 집을 수 있는 곳 지도 (메모리만, 저장 안 함). 높이·방향 몇 개를 오가도 다시 안 풀게 6 개까지


def _grasp_mode(v, yaw=None):
    """화면의 집는 방향(auto / vertical / horizontal, 빈 값 = auto)과 큐브 yaw(°, 빈 값 = 안 줌) → (grasp, yaw). 모르는 값은 ValueError."""
    import hand_ik
    g = str(v or "auto").strip() or "auto"
    if g not in hand_ik.GRASP_MODES:
        raise ValueError(f"집는 방향(grasp)은 {' / '.join(hand_ik.GRASP_MODES)} 중 하나")
    try:
        y = None if yaw in (None, "") else float(yaw)
    except (TypeError, ValueError):
        raise ValueError("yaw 는 숫자 (°)")
    if y is not None and not (math.isfinite(y) and abs(y) <= 720):
        raise ValueError("yaw 는 −720~720 °")
    return g, (None if y is None else round(y, 1))


def _table_d(v):
    """화면의 책상 높이 칸(어깨 축 아래 m, 빈 값 = 기본) → float. 범위 밖·숫자 아님은 ValueError (hand_ik.table_d 의 말 그대로)."""
    import hand_ik
    ik = _ik_dir()
    if ik is None:
        return None
    return hand_ik.table_d(ik, v)


def _grasp_warm():
    """서버 켤 때 기본(어깨 옆 0°) 지도를 미리 계산 — 처음 열 때 기다리지 않게 (2026-11-20, 팔꿈치 ±135 로 오래 걸림)."""
    try:
        api_ik_graspmap(0.0)
    except Exception as e:
        print("graspmap 미리 계산 실패:", e, flush=True)


def _cal_ranges():
    cal = _calibration()
    return {j["joint"]: j["cmd_range_deg"] for j in cal.get("joints") or [] if j.get("cmd_range_deg")}


def _sy(v):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return 0.0
    return v if math.isfinite(v) else 0.0


@app.get("/api/ik/graspmap")
def api_ik_graspmap(shoulder_yaw: float = 0.0, d: str = "", grasp: str = "auto", yaw: str = ""):
    """책상 위 1 cm 격자마다 큐브를 집을 수 있나 (해석 IK, 정의는 hand_ik._Solver.grasp_eval). 처음 한 번 몇 초, 이후 캐시.
    grasp = 집는 방향 (auto / vertical / horizontal), yaw = 이 큐브 yaw (실좌표 °) 만 보기 (비우면 모든 yaw; 수평은 '하나라도')."""
    import hand_ik
    ranges, sy = _cal_ranges(), _sy(shoulder_yaw)
    try:
        td = _table_d(d)
        gm, gy = _grasp_mode(grasp, yaw)
    except ValueError as e:
        return JSONResponse({"ok": False, "reason": str(e)}, status_code=400)
    key = (json.dumps(ranges, sort_keys=True), round(sy, 3), None if td is None else round(td, 4), gm, gy)
    with _GRASP_LOCK:   # 미리 계산 중이면 기다렸다가 그 결과를 쓴다 (두 번 계산 안 함)
        r = _GRASP_CACHE.get(key)
        cached = r is not None
        if not cached:
            r = hand_ik.graspmap(_ik_dir(), ranges, sy, d=td, grasp=gm, yaw=gy)
            if r.get("ok"):
                for k in [k for k in _GRASP_CACHE if k[:2] != key[:2]] or (list(_GRASP_CACHE)[:1] if len(_GRASP_CACHE) >= 6 else []):
                    _GRASP_CACHE.pop(k, None)   # 관절 범위·어깨 옆이 다른 지도는 버리고, 같은 조건의 다른 책상 높이는 3 개까지
                _GRASP_CACHE[key] = r
    return JSONResponse({**r, "cached": cached})


_REACH_CACHE = {}


@app.get("/api/ik/reachxyz")
def api_ik_reachxyz(z_cm: float = 3.5, ref: str = "grip", shoulder_yaw: float = 0.0, step_cm: float = 2.0, d: str = ""):
    """'갈 수 있는 곳' (2026-11-21): 높이 z_cm(기준점의 책상 위 cm)에서 격자마다 위치 역기구학이 풀리나 (집기 조건 없음). 높이마다 캐시."""
    import hand_ik
    if not (math.isfinite(z_cm) and -10 <= z_cm <= 80) or not (0.5 <= step_cm <= 5):
        return JSONResponse({"ok": False, "reason": "z_cm −10~80, step_cm 0.5~5"}, status_code=400)
    ranges, sy = _cal_ranges(), _sy(shoulder_yaw)
    ref = "wrist" if ref == "wrist" else "grip"
    try:
        td = _table_d(d)
    except ValueError as e:
        return JSONResponse({"ok": False, "reason": str(e)}, status_code=400)
    key = (json.dumps(ranges, sort_keys=True), round(sy, 3), ref, round(z_cm, 1), round(step_cm, 2), None if td is None else round(td, 4))
    r = _REACH_CACHE.get(key)
    if r is None:
        r = hand_ik.reachxyz(_ik_dir(), ranges, round(z_cm, 1) / 100.0, ref, sy, step=step_cm / 100.0, d=td)
        if r.get("ok"):
            if len(_REACH_CACHE) > 60:
                _REACH_CACHE.clear()
            _REACH_CACHE[key] = r
        return JSONResponse({**r, "cached": False})
    return JSONResponse({**r, "cached": True})


@app.post("/api/ik/graspable")
async def api_ik_graspable(req: Request):
    """한 점 {x, y, shoulder_yaw?} (실좌표 m) 을 집을 수 있나 — 지도와 같은 판정, 그 점 그대로 (격자 칸 아님)."""
    import hand_ik
    b = await _body(req)
    try:
        x, y = float(b["x"]), float(b["y"])
        if not (math.isfinite(x) and math.isfinite(y)) or max(abs(x), abs(y)) > 2:
            raise ValueError
    except (KeyError, TypeError, ValueError):
        return JSONResponse({"ok": False, "reason": "x, y 숫자 (m, |x|,|y| ≤ 2)"}, status_code=400)
    try:
        td = _table_d(b.get("d"))
        gm, gy = _grasp_mode(b.get("grasp"), b.get("yaw"))
    except ValueError as e:
        return JSONResponse({"ok": False, "reason": str(e)}, status_code=400)
    return JSONResponse(await asyncio.to_thread(hand_ik.graspcheck, _ik_dir(), _cal_ranges(), x, y, _sy(b.get("shoulder_yaw")), td, gm, gy))


@app.post("/api/ik/fk")
async def api_ik_fk(req: Request):
    return JSONResponse(await asyncio.to_thread(_hand_ik, await _body(req), "fk"))


@app.post("/api/ik/play_virtual")
def api_ik_play():
    return JSONResponse(_play_virtual())


@app.post("/api/ik/stop_virtual")
def api_ik_stop():
    if VM["arm"] is None:
        return JSONResponse({"ok": False, "error": "가상 팔 없음"})
    return JSONResponse(VM["arm"].stop_trajectory())


def _go_home(b):
    """'HOME(모든 관절 0)으로 가기': 지금 자세 → HOME 궤적을 pick_plan.plan_home 으로 계획(책상 안 파고드는 길)하고 지금 모드의 팔로 재생.
    계획 궤적 파일(PICK_TRAJ)은 건드리지 않는다. ROS 모드는 아직 없음."""
    kind = mode_kind()
    if kind == "ros":
        return {"ok": False, "error": "ROS 모드에선 아직 없음 (가상 또는 실물 직접 CAN)"}
    if kind == "virtual" and VM["arm"] is None:
        return {"ok": False, "error": "가상 팔 없음"}
    if kind == "direct" and not direct_api.DIRECT.connected():
        return {"ok": False, "error": "팔 연결 안 됨 (연결 탭 ② 팔 연결)"}
    gen = None
    if kind == "direct":
        arm = direct_api.DIRECT.arm
        gen = arm.stop_gen            # 계획(0.2~1.2 s) 동안 STOP 이 눌리면 play_trajectory 가 거절
        if arm.calib_dirty:           # 계획은 파일(calibration.json)로, 팔은 메모리 보정으로 → 다르면 지금 자세를 잘못 읽는다
            return {"ok": False, "error": "보정을 바꾸고 저장 안 함 — 연결 탭에서 'calibration.json 저장' 뒤 HOME"}
    cal = _calibration()
    pose = actual_pose(backend().snapshot(), cal)
    now = {n: v["deg"] for n, v in pose.items() if v.get("deg") is not None}
    # 서보는 피드백이 없어 한 번도 안 보낸 서보는 각을 모른다 → 0°(HOME 값)로 보고 계획. 실물은 real_arm.play_trajectory 가
    # '한 번도 안 보낸 서보'를 따로 거절한다(첫 명령이 램프 없이 튐). AK45 를 모르면 plan_home 이 거절.
    for j in cal.get("joints") or []:
        if j["motor"] == "servo" and j["joint"] not in now:
            now[j["joint"]] = 0.0
    try:
        speed = float((b or {}).get("max_speed") or 0) or float(_trajectory()["params"].get("max_speed") or 30.0)
    except (TypeError, ValueError):
        speed = 30.0
    if not math.isfinite(speed):
        speed = 30.0
    speed = min(max(speed, 1.0), 90.0)    # 0·음수면 프레임이 경유점만 남아 검사가 빈다, 아주 작으면 프레임 수가 폭발
    try:   # 책상 높이: 화면 칸 값(body d), 없으면 마지막 계획에 쓴 칸 값, 그것도 비었으면 기본
        td = _table_d(b["d"] if isinstance(b, dict) and "d" in b else _trajectory()["params"].get("d"))
    except ValueError as e:
        return {"ok": False, "error": str(e)}
    try:
        import pick_plan
        traj = pick_plan.plan_home(now, ik=os.environ.get("JAX_IK_DIR") or None, max_speed=speed, d=td)
    except Exception as e:
        return {"ok": False, "error": f"HOME 길 계획 실패: {type(e).__name__}: {e}"}
    if traj["status"] == "already":
        return {"ok": True, "msg": traj["reason"]}
    if traj["status"] != "playable":
        return {"ok": False, "error": f"HOME 으로 못 감: {traj['reason']}"}
    from arm_control.play import check_trajectory
    if kind == "direct":
        arm = direct_api.DIRECT.arm
        fatal = direct_api.analytic_fatal(traj)
        problems = []
        if not fatal:
            fatal, problems = check_trajectory(traj, arm.cal)
            problems = direct_api.drop_unverified(problems)
        if fatal:
            return {"ok": False, "error": f"HOME 재생 거절: {fatal}"}
        if problems and not (b or {}).get("force"):   # 실물 궤적 재생(play_real)과 같은 규칙: 화면이 확인창을 띄우고 force 로 다시
            return {"ok": False, "need_force": True, "problems": problems,
                    "error": f"검사 {len(problems)}건 (--force 가 필요한 것): " + " / ".join(problems)}
        r = arm.play_trajectory(traj, speed, stop_gen=gen)
    else:
        from arm_control import calibration as C
        fatal, _ = check_trajectory(traj, C.load())
        if fatal:
            return {"ok": False, "error": f"HOME 재생 거절: {fatal}"}
        r = VM["arm"].play_trajectory(traj, speed)
    if r.get("ok"):
        run_record.REC.on_play(traj, {"max_speed": speed, "d": td, "from_deg": now}, arm=arm if kind == "direct" else VM["arm"],
                               virtual=kind == "virtual")
        r["msg"] = f"HOME 으로 가는 중 — {traj['reason']}"
    return r


@app.post("/api/arm/go_home")
async def api_go_home(req: Request):
    return JSONResponse(await asyncio.to_thread(_go_home, await _body(req)))


direct_api.install(app, mode_kind, pm, can_state, _trajectory, str(LOG_DIR / "direct.log"))

# 강화학습 탭은 위의 /api/rl/* (rl_run.py·rl_worker.py, 우리 것). 팀 rl_api.py·rl_runner.py 는 안 씀 (2026-11-24 사용자)
# 휴대폰 카메라 탭 (사진 → 큐브 자리, ArUco): 전부 phone_api.py / phone_detect.py / static/phone.js. 팔은 거기서 안 움직인다
import phone_api   # noqa: E402
phone_api.install(app, LOG_DIR, mode_kind)
import live_api   # noqa: E402  큐브 실시간 인식 (폰 프레임 스레드) + 가상 장면: /api/live/*, /api/phone/live
live_api.install(app, LOG_DIR, mode_kind)
import run_record   # noqa: E402  실물 재생마다 logs/runs/<시각>_<kind>/ 에 영상·관절·인식·계획·사건 (run_record.py, static/record.js)
run_record.install(app, LOG_DIR,
                   lambda: (VM["arm"], True) if MODE["virtual"] else (direct_api.DIRECT.arm if direct_api.DIRECT.connected() else None, False),
                   mode_kind, lambda: can_state.get("text"))
# 탑 쌓기 탭 (stack_api.py / static/stack.js) 은 2026-11-25 없앰 (사용자) — 등록하지 않는다. 다시 쓰려면 아래 두 줄과 index.html 의 탭·스크립트를 되살린다.
# import stack_api; stack_api.install(app, LOG_DIR)

_MESH_NAME = re.compile(r"[A-Za-z0-9_.-]{1,80}\.obj")


@app.get("/robot/meshes/{name}")
def robot_mesh(name: str):
    """team-jax/IK motion2/arm/meshes 의 OBJ (레포에 넣지 않고 JAX_IK_DIR 등에서 서빙)."""
    d = _ik_dir()
    if not _MESH_NAME.fullmatch(name) or d is None:
        return PlainTextResponse("메시 없음", status_code=404)
    p = d / "motion2" / "arm" / "meshes" / name
    if not p.is_file():
        return PlainTextResponse("메시 없음", status_code=404)
    return FileResponse(str(p), media_type="text/plain", headers={"Cache-Control": "max-age=3600"})


FOLLOW_LOG = LOG_DIR / "follow.log"
FOLLOW_LOG_MAX = 4 << 20   # 4 MB 넘으면 follow.log.1 로 돌리고 새로 (하나만 남김)
_follow_lock = threading.Lock()


@app.post("/api/log/follow")
async def api_log_follow(req: Request):
    """손 위치 카드 '팔도 바로 따라가기' 진단 로그: 브라우저가 틱·풀이 응답마다 JSON 한 줄씩 보낸다 -> logs/follow.log."""
    b = await _body(req)
    rows = b.get("rows") if isinstance(b, dict) and isinstance(b.get("rows"), list) else [b]
    now = time.time()
    lines = []
    for r in rows[:200]:
        if isinstance(r, dict):
            r = {"ts": round(now, 3), "t_srv": time.strftime("%H:%M:%S", time.localtime(now)), **r}
            lines.append(json.dumps(r, ensure_ascii=False, separators=(",", ":")))
    if not lines:
        return {"ok": False}
    with _follow_lock:
        FOLLOW_LOG.parent.mkdir(parents=True, exist_ok=True)
        try:
            if FOLLOW_LOG.exists() and FOLLOW_LOG.stat().st_size > FOLLOW_LOG_MAX:
                os.replace(FOLLOW_LOG, FOLLOW_LOG.with_name("follow.log.1"))
        except OSError:
            pass
        with open(FOLLOW_LOG, "a", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
    return {"ok": True, "n": len(lines)}


@app.get("/api/log/{sid}")
def api_log(sid: str, n: int = 300):
    it = pm.items.get(sid)
    if it is None:
        return PlainTextResponse("없는 항목", status_code=404)
    return PlainTextResponse("\n".join(tail_file(it.log_path, min(max(n, 1), 5000), 1 << 20)))


@app.websocket("/ws")
async def ws(sock: WebSocket):
    await sock.accept()
    try:
        while True:
            await sock.send_text(json.dumps(state()))
            await asyncio.sleep(0.25)
    except (WebSocketDisconnect, RuntimeError):
        pass
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description="JAX 팔 대시보드")
    ap.add_argument("--host", default=os.environ.get("JAX_DASH_HOST", "127.0.0.1"),
                    help="기본 127.0.0.1(이 PC만). 0.0.0.0 이면 같은 네트워크 누구나 노드를 켜고 끌 수 있음")
    ap.add_argument("--port", type=int, default=int(os.environ.get("JAX_DASH_PORT", "8080")))
    ap.add_argument("--virtual", action="store_true", default=os.environ.get("JAX_DASH_VIRTUAL") == "1",
                    help="가상 팔로 시작 (실물·CAN·ROS 없이). 화면 위 스위치로 바꿀 수 있다")
    ap.add_argument("--ros", action="store_true", default=os.environ.get("JAX_DASH_ROS") == "1",
                    help="실물을 ROS 노드(ak45_node …)로 — 팀 ROS 버전. 없으면 실물 = 직접 CAN (ROS 설치 필요 없음)")
    ap.add_argument("--can-interface", default=os.environ.get("JAX_CAN_INTERFACE", "socketcan"),
                    help="직접 CAN 의 python-can interface (socketcan, 시험은 virtual)")
    ap.add_argument("--can-channel", default=os.environ.get("JAX_CAN_CHANNEL", CAN_IFACE))
    ap.add_argument("--fake-hw", action="store_true", help="직접 CAN 을 가짜 AK45 4개 + 가짜 STM32 에 (interface virtual)")
    a = ap.parse_args()
    MODE["ros"] = a.ros
    procman.FORCE_LOCAL["on"] = not a.ros      # pick_place --dry-run 을 ros2 run 대신 이 PC 의 python 으로
    direct_api.DIRECT.cfg.update(interface="virtual" if a.fake_hw else a.can_interface,
                                 channel="fake_can0" if a.fake_hw else a.can_channel, fake=a.fake_hw)
    import uvicorn
    if a.virtual:
        r = set_mode(True)
        print("가상 모드로 시작" if r["ok"] else f"가상 모드 실패: {r['error']}", flush=True)
    if not MODE["virtual"] and MODE["ros"]:
        threading.Thread(target=ros.start, daemon=True).start()   # 시작할 때 한 번 자동 시도(실패하면 화면 버튼)
    if IS_LINUX:
        threading.Thread(target=_can_watch, daemon=True).start()
    threading.Thread(target=_grasp_warm, daemon=True).start()   # 지도 전용 IK 프로세스라 손 IK 를 안 막는다
    shown = "localhost" if a.host in ("127.0.0.1", "localhost") else a.host
    print(f"대시보드: http://{shown}:{a.port}/   ws={WS} distro={ROS_DISTRO} "
          f"모드={ {'virtual': '가상', 'ros': '실물(ROS 자식 프로세스로 연결 시도 중)', 'direct': '실물(직접 CAN, ROS 없음)'}[mode_kind()] }"
          + (f" · CAN {direct_api.DIRECT.cfg['interface']} {direct_api.DIRECT.cfg['channel']}" if not MODE["ros"] else ""), flush=True)
    if not MODE["virtual"] and MODE["ros"] and not ros_possible():
        print(f"참고: {ros.setup_paths()[0]} 없음 — 실물 모드는 못 씀. --virtual 또는 화면의 '가상' 스위치", flush=True)
    if a.host not in ("127.0.0.1", "localhost", "::1"):
        print("주의: 외부 접속 열림 — 같은 네트워크 누구나 노드를 켜고 끄고 모터 명령을 보낼 수 있다.", flush=True)
    try:
        uvicorn.run(app, host=a.host, port=a.port, log_level="warning")
    finally:
        if direct_api.DIRECT.connected():
            print("대시보드 종료: 직접 CAN 팔 연결을 닫는다 — 브레이크 0 A, AK45 힘이 빠진다(팔이 떨어질 수 있음).", flush=True)
            direct_api.DIRECT.disconnect(brake=True)
        # 서버가 꺼져도 노드는 그대로 둔다(끄면 팔이 떨어질 수 있음). 다시 켜면 pids.json 으로 다시 붙잡는다(Linux).
        ros.stop()
        alive = [it.svc.label for it in pm.items.values() if it.alive() and it.vm is None]
        if alive:
            print("대시보드 종료. 아직 켜져 있는 항목(그대로 둠): " + ", ".join(alive), flush=True)


if __name__ == "__main__":
    main()
