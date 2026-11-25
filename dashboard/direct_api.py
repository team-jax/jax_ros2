"""대시보드 '실물(직접 CAN)' 모드의 서버 쪽: real_arm.RealArm 을 켜고 끄고, 화면 버튼을 잇는다. ROS 없음.

server.py 는 연결 지점만 가진다 (backend() 가 DIRECT.backend() 를 부르고, install(app, ...) 이 아래 경로를 단다).

    POST /api/direct/connect     {max_speed}   팔 연결 (can0 을 열고 잠금 /tmp/ak45_ctrl.lock). AK45 에는 아직 아무것도 안 보냄
    POST /api/direct/disconnect                팔 연결 끊기 = 브레이크 0 A (팔이 떨어진다 — 화면이 확인창)
    POST /api/direct/hold                      STOP: 지금 자리 홀드
    POST /api/direct/limp        {joint?}      힘 빼기 (확인창)
    POST /api/direct/calib       {action, joint, value}   보정: move ±N° / zero / flip / scale / verify / save
    POST /api/ik/play_real       {force}       역기구학 궤적을 실물로 재생 (check_trajectory 통과, 문제 있으면 force 확인)
    POST /api/ik/stop_real
"""
import asyncio
import json
import sys
import threading

from fastapi import Request
from fastapi.responses import JSONResponse

import real_arm as R
import run_record

# 같은 can0 에 명령하는 ROS 노드 (대시보드 밖에서 떠 있으면 연결 거부)
ROS_PROCS = ("ak45_node", "stm_bridge_node", "arm_control_node", "homing_guard", "calibrate", "play_trajectory")
ROS_ITEMS = ("arm_launch", "arm_homed", "ak45", "stm_bridge", "arm_control", "vcan_test")   # services.py id


class Direct:
    def __init__(self):
        self.arm = None
        self.cfg = {"interface": "socketcan", "channel": "can0", "fake": False, "lock_path": R.LOCK_PATH}
        self.fake = None
        self.lines = []
        self.lock = threading.Lock()

    def backend(self):
        return self.arm if self.arm is not None and self.arm.running else R.Disconnected(lines=self.lines)

    def connected(self):
        return self.arm is not None and self.arm.running

    def connect(self, params, pm, can_state, log_file):
        with self.lock:
            if self.connected():
                return {"ok": True, "msg": "이미 연결됨"}
            strays = [s for s in pm.strays if s["name"] in ROS_PROCS]
            mine = [it.svc.label for it in pm.items.values() if it.alive() and it.vm is None and it.svc.id in ROS_ITEMS]
            if strays or mine:
                who = [f"{s['name']}(pid {s['pid']})" for s in strays] + mine
                return {"ok": False, "error": "ROS 노드가 떠 있어 연결 안 함 (같은 모터에 두 곳이 명령): " + ", ".join(who)}
            if self.cfg["interface"] == "socketcan" and sys.platform.startswith("linux"):
                if can_state.get("exists") is False:
                    return {"ok": False, "error": f"{self.cfg['channel']} 없음 — USB-CAN 연결 확인"}
                if can_state.get("up") is False:
                    return {"ok": False, "error": f"{self.cfg['channel']} DOWN — 먼저 ① CAN 켜기"}
            if self.cfg["fake"] and self.fake is None:
                sys.path.insert(0, str(R.REPO / "dashboard" / "tools"))
                from fake_can_arm import FakeArm
                self.fake = FakeArm(self.cfg["interface"], self.cfg["channel"])
            try:
                p = {"max_speed": float(params.get("max_speed") or R.DEFAULTS["max_speed"])}
            except (TypeError, ValueError):   # [] · {} 도 (2026-11-25 검토)
                return {"ok": False, "error": "max_speed 가 숫자가 아님"}
            if not 1.0 <= p["max_speed"] <= 90.0:
                return {"ok": False, "error": "max_speed 1~90 °/s"}
            try:
                arm = R.RealArm(interface=self.cfg["interface"], channel=self.cfg["channel"],
                                lock_path=None if self.cfg["interface"] == "virtual" else self.cfg["lock_path"],
                                params=p, log=self._remember, log_file=log_file)
                arm.start()
            except Exception as e:
                return {"ok": False, "error": f"팔 연결 실패: {type(e).__name__}: {e}"}
            self.arm = arm
            fake = " (가짜 AK45·STM32)" if self.fake else ""
            return {"ok": True, "msg": f"팔 연결됨: {self.cfg['interface']} {self.cfg['channel']}{fake}. AK45 는 원점을 잡기 전엔 안 잡는다"}

    def _remember(self, text):
        self.lines.append(text)
        del self.lines[:-200]

    def disconnect(self, brake=True):
        with self.lock:
            if not self.connected():
                return {"ok": True, "msg": "연결 안 돼 있음"}
            self.arm.close(brake=brake)
            self.lines = list(self.arm.lines)
            self.arm = None
            return {"ok": True, "msg": "팔 연결 끊음: 브레이크 0 A (AK45 힘 빠짐)"}


DIRECT = Direct()


def analytic_fatal(traj):
    """대시보드 해석 IK 계획(pick_plan.py, planner "analytic")이 자기 검사(손가락↔큐브·책상, 관절 한계, 길 못 찾음)에
    걸렸으면 실물 재생은 force 로도 안 된다 (팀 ROS 경로 play.check_trajectory 는 그대로 — 이것은 대시보드에서만 더함).
    거절 이유 글 또는 None."""
    if traj.get("planner") != "analytic":
        return None
    status = traj.get("status") or "?"
    fb = (traj.get("checks") or {}).get("joint_space_fallback") or []
    if status != "playable" or traj.get("playable") is False:
        return f"계획 검사 실패 ({status}): {traj.get('reason', '')} — 실물에선 force 로도 재생 안 함. 다른 자리로 다시 계획"
    if fb:
        return "계획 일부가 관절 공간 직선(길 못 찾음): " + ", ".join(map(str, fb)) + " — 실물에선 재생 안 함"
    return None


ALLOW_UNVERIFIED_CAL = True   # 2026-11-23 사용자: "가정값이어도 움직이게" — 보정 미확인(verified false)은 force 확인 없이 재생 (로그에만 남김)


def drop_unverified(problems):
    """check_trajectory 의 'calibration not verified' 경고를 뺀다 (ALLOW_UNVERIFIED_CAL). 다른 경고는 그대로 force 확인."""
    if not ALLOW_UNVERIFIED_CAL:
        return problems
    return [p for p in problems if not str(p).startswith("calibration not verified")]


def play_real(trajectory, b):
    if not DIRECT.connected():
        return {"ok": False, "error": "팔 연결 안 됨 (직접 CAN)"}
    arm = DIRECT.arm
    gen = arm.stop_gen          # 이 사이 STOP 이 눌리면 play_trajectory 가 거절
    t = trajectory()
    if not t["ok"]:
        return {"ok": False, "error": t.get("error") or "궤적 없음"}
    m = b.get("mtime")      # 화면이 보고 확인한 계획의 mtime. 그 사이 다른 기기가 다시 계획했으면 거절 (없으면 검사 안 함: 스크립트·시험)
    if m is not None:
        try:
            same = abs(float(m) - float(t["mtime"])) <= 1e-6
        except (TypeError, ValueError):
            same = False
        if not same:
            return {"ok": False, "error": "계획이 바뀜 — 화면의 계획을 확인하고 다시 재생"}
    traj = t["traj"]
    from arm_control.play import check_trajectory
    fatal = analytic_fatal(traj)
    if fatal:
        return {"ok": False, "error": f"재생 거절: {fatal}"}
    fatal, problems = check_trajectory(traj, arm.cal)
    problems = drop_unverified(problems)
    if fatal:
        return {"ok": False, "error": f"재생 거절: {fatal}"}
    if problems and not b.get("force"):
        return {"ok": False, "need_force": True, "problems": problems,
                "error": f"검사 {len(problems)}건 (--force 가 필요한 것): " + " / ".join(problems)}
    try:
        speed = float(t["params"].get("max_speed") or 30.0)
    except ValueError:
        speed = 30.0
    r = arm.play_trajectory(traj, speed, stop_gen=gen)
    if r.get("ok"):
        run_record.REC.on_play(traj, t.get("params"), arm=arm, extra={"force": bool(b.get("force")), "problems": problems, "speed": speed})
    if r.get("ok") and problems:
        r["msg"] += f" (검사 {len(problems)}건 무시하고 재생: force)"
    return r


def install(app, mode, pm, can_state, trajectory, log_file):
    """server.py 가 부른다. mode() -> "direct"|"virtual"|"ros"."""

    async def body(req):
        try:
            j = await req.json()
        except Exception:
            return {}
        return j if isinstance(j, dict) else {}   # [] · null · "문자열" 이 와도 .get 에서 500 으로 죽지 않게

    def only_direct():
        return None if mode() == "direct" else JSONResponse({"ok": False, "error": "실물(직접 CAN) 모드에서만"})

    @app.post("/api/direct/connect")
    async def _connect(req: Request):
        bad = only_direct()
        if bad:
            return bad
        b = await body(req)
        return JSONResponse(await asyncio.to_thread(DIRECT.connect, b, pm, can_state, log_file))

    @app.post("/api/direct/disconnect")
    async def _disconnect():
        return JSONResponse(await asyncio.to_thread(DIRECT.disconnect, True))

    @app.post("/api/direct/hold")
    async def _hold():
        return JSONResponse(DIRECT.backend().hold_all("STOP"))

    @app.post("/api/direct/limp")
    async def _limp(req: Request):
        b = await body(req)
        j = str(b.get("joint") or "all")
        be = DIRECT.backend()
        if not DIRECT.connected():
            return JSONResponse(be.limp())
        names = None
        if j != "all":
            try:
                names = [be.homing.by_joint(j)]
            except KeyError:
                return JSONResponse({"ok": False, "error": f"모르는 AK45 관절 {j!r}"})
        return JSONResponse(be.limp(names))

    @app.post("/api/direct/calib")
    async def _calib(req: Request):
        bad = only_direct()
        if bad:
            return bad
        b = await body(req)
        j = b.get("joint")
        return JSONResponse(DIRECT.backend().calib(str(b.get("action") or ""), j if j is None else str(j), b.get("value")))   # 목록 joint → 500 막기

    @app.post("/api/ik/play_real")
    async def _play(req: Request):
        bad = only_direct()
        if bad:
            return bad
        b = await body(req)
        return JSONResponse(await asyncio.to_thread(play_real, trajectory, b))

    @app.post("/api/ik/stop_real")
    async def _stop():
        return JSONResponse(DIRECT.backend().stop_trajectory())

    @app.get("/api/direct/log")
    def _log():
        be = DIRECT.backend()
        return JSONResponse({"lines": list(getattr(be, "lines", []))})
