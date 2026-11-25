"""가상 로봇팔: 실물·CAN·ROS 없이(Windows 포함) 대시보드를 끝까지 돌려 보는 시뮬레이션. 표준 라이브러리만.

대시보드가 가상 모드면 ROS 자식 프로세스(ros_bridge.RosBridge) 대신 이 VirtualArm 이 같은 모양의 상태
(ros_monitor.RosMonitor.snapshot)를 채우고, 노드 켜기/끄기·각도 보내기·원점 잡기/쉬게 하기를 받는다.

무엇을 흉내 내나 (가상 '노드'):
  ak45        ak45_node 규칙: 첫 명령 전엔 프레임 없음(안 잡음), |목표| > max_command_deg(360) 거부, 마지막 목표를
              계속 홀딩(command_timeout 0), 피드백 없으면/워치독 끊기면 명령 안 보냄, 온도 60 ℃ 이상이면 잠금(래치),
              /joint_states 50 Hz(0.1° 양자화), /diagnostics 2 Hz(같은 키: position_deg, speed_erpm, current_a,
              temperature_c, error_code, watchdog_ok, target_deg, command_active, position_error_deg).
              모터: 목표를 1차 지연(τ 0.08 s) + 속도 한계 60 °/s 로 따라감. 힘이 없으면(노드 꺼짐·첫 명령 전·
              전원 꺼짐) 관절이 중력으로 모터 0° 쪽으로 40 °/s 로 떨어진다(가정: 팔이 아래로 늘어진 자세가 0°).
              온도: 전류²에 비례해 오르고 28 ℃ 로 식는다(가상 숫자, 실측 아님).
  stm_bridge  /servo/command 를 0~180° 로 자르고 그대로 '보낸 값'(/servo/state)으로. 서보는 그 각으로 즉시 간다(가정).
  arm_control arm_control_node 와 같은 변환·램프: arm_control/arm_control/calibration.py(sim→모터, 클립)와
              motion.py(Mover, max_speed_deg_s, 50 Hz)를 그대로 import. 서보 첫 명령은 램프 없이 바로 감(실물과 같음).
  homing_guard arm_homed.launch.py 일 때만. arm_control/arm_control/homing.py(Homing)를 import 해 homing_guard.py 의
              on_state / on_diagnostics / on_command / on_timer / 서비스를 같은 순서로 따라 한다.
  vision      fixed_pose 파라미터(또는 화면의 가상 큐브 칸)를 /object_pose(frame shoulder_roll_axis) 로 1 Hz.

가상 전원 끄기/켜기(power): AK45 4개의 피드백이 끊기고 힘이 빠진다. 다시 켜면 엔코더 영점이 15~40° 틀어진다
(단일 엔코더가 영점을 잃는 경우 흉내) — homing_guard 가 원점을 지우고 제자리 홀드를 보내는지 볼 수 있다.
"""
import json
import math
import random
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
for _p in (REPO / "arm_control", REPO / "stm_bridge"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from arm_control import calibration as _cal   # noqa: E402  (rclpy 없음: stm_bridge.protocol, arm_control.protocol 만)
from arm_control.homing import Homing, load_home_poses   # noqa: E402
from arm_control.motion import Mover   # noqa: E402

HZ = 50.0
MAX_COMMAND_DEG = 360.0      # ak45_node.yaml
MAX_TEMP_C = 60.0
MOTOR_SPEED = 60.0           # °/s 가상 모터 속도 한계
MOTOR_TAU = 0.08             # s 1차 지연
FALL_SPEED = 40.0            # °/s 힘이 없을 때 0° 쪽으로
AMBIENT_C = 28.0

# 대시보드 항목(services.py id) -> 그 항목이 띄우는 가상 노드
SERVICE_NODES = {
    "arm_launch": ("ak45", "stm_bridge", "arm_control"),
    "arm_homed": ("ak45", "stm_bridge", "homing_guard", "arm_control"),
    "ak45": ("ak45",),
    "stm_bridge": ("stm_bridge",),
    "arm_control": ("arm_control",),
    "vision": ("vision",),
}


def _play_view(play):
    """snapshot 의 playback: 단계표(phases)는 빼고 지금 단계 글·번호·남은 초 (play_phase.view)."""
    try:
        import play_phase
        return play_phase.view(play)
    except Exception:
        p = dict(play)
        p.pop("phases", None)
        return p


def _f2(x):
    return f"{x:.2f}"


class Motor:
    def __init__(self, joint):
        self.joint = joint
        self.phys = 0.0            # 출력축의 '진짜' 각(°), 처음엔 팔이 늘어진 0°
        self.enc_off = 0.0         # 엔코더 영점 어긋남: 피드백 = phys - enc_off
        self.powered = True
        self.temp = AMBIENT_C + random.uniform(0, 2)
        self.vel = 0.0
        self.current = 0.0
        self.has_target = False
        self.target = 0.0
        self.temp_locked = False
        self.command_active = False
        self.fb_t = None           # 마지막 피드백 시각(monotonic)

    def raw(self):
        return round((self.phys - self.enc_off) * 10.0) / 10.0


class VirtualArm:
    def __init__(self, log=None):
        self.log_cb = log or (lambda node, text: None)
        self.cal = _cal.load()
        self.joints = {j.joint: j for j in self.cal.joints}
        self.home_poses = load_home_poses()
        self.lock = threading.RLock()
        self.nodes = {}                    # node -> {"since": t, "params": {...}}
        self.motors = {f"ak45_{j.id}": Motor(j) for j in self.cal.joints if j.motor == "ak45"}
        self.servo_sent = {}               # name -> (motor deg, t)
        self.diag = {}                     # RosMonitor.diag 와 같은 모양
        self.diag_t = None
        self.joint_t = None
        self.homed_fb = {}                 # ak45_N -> (deg, t)  guard 가 넘겨주는 /ak45/joint_states_homed
        self.homing_status = None
        self.homing_t = None               # time.time()
        self.guard = None                  # {"homing", "mover", "pending_hold"}
        self.ctrl = None                   # arm_control: {"mover"}
        self.obj = None
        self.cube = None                   # 화면 '가상 큐브' 칸: (x, y, z, yaw_deg)
        self.play = {"running": False}
        self._play_stop = threading.Event()
        self._tick = 0
        self._stop = threading.Event()
        threading.Thread(target=self._loop, daemon=True).start()

    # ------------------------------------------------------------------ 노드 켜기/끄기
    def node_on(self, node):
        return node in self.nodes

    def start_service(self, sid, params):
        nodes = SERVICE_NODES.get(sid)
        if not nodes:
            return False
        with self.lock:
            for n in nodes:
                self._start_node(n, sid, params or {})
        return True

    def stop_service(self, sid):
        with self.lock:
            for n in SERVICE_NODES.get(sid, ()):
                if n in self.nodes and self.nodes[n]["owner"] == sid:
                    self._stop_node(n)

    def service_alive(self, sid):
        with self.lock:
            return any(self.nodes.get(n, {}).get("owner") == sid for n in SERVICE_NODES.get(sid, ()))

    def _start_node(self, n, owner, params):
        now = time.monotonic()
        self.nodes[n] = {"since": now, "owner": owner, "params": dict(params)}
        try:     # 검사는 procman 이 build 로 한다 — 그래도 숫자가 아니거나 0 이하면 기본 (음수면 Mover 가 거꾸로 간다)
            speed = float(params.get("max_speed_deg_s") or 30.0)
        except (TypeError, ValueError):
            speed = 30.0
        if not (math.isfinite(speed) and speed > 0):
            speed = 30.0
        if n == "ak45":
            for m in self.motors.values():
                m.has_target = m.temp_locked = m.command_active = False
            self._log(n, f"[가상] ak45_node 시작: 모터 4개 ID 1~4, max_command_deg {MAX_COMMAND_DEG:g}, "
                         f"command_timeout 없음(마지막 목표 홀딩), 첫 명령 전엔 프레임 없음")
        elif n == "stm_bridge":
            self._log(n, f"[가상] stm_bridge_node 시작 (servo_channel {params.get('servo_channel', 'can0')}, 가상 버스)")
        elif n == "arm_control":
            self.ctrl = {"mover": Mover(speed, HZ)}
            unver = [j.joint for j in self.cal.joints if not j.verified]
            self._log(n, f"[가상] arm_control_node: {self.cal.path}: {len(self.joints)} joints, {speed:g} deg/s at {HZ:g} Hz. "
                         f"Unverified calibration: {', '.join(unver) or 'none'}")
        elif n == "homing_guard":
            hm = Homing({f"ak45_{j.id}": j for j in self.cal.joints if j.motor == "ak45"}, self.home_poses)
            self.guard = {"homing": hm, "mover": Mover(speed, HZ), "pending_hold": set()}
            self.homed_fb = {}
            self._log(n, "[가상] homing_guard: every AK45 starts NOT homed; call home_all with the arm at its home pose")
            self._publish_homing()
        elif n == "vision":
            fp = (params.get("fixed_pose") or "").strip()
            try:
                v = [float(x) for x in json.loads(fp)] if fp else None
            except ValueError:
                v = None
            if v and len(v) == 3:
                v = [v[0], v[1], 0.0, v[2]]
            if v and len(v) == 4:
                self.cube = tuple(v)
            self._log(n, f"[가상] vision_node: /object_pose = {self.cube or '없음 (가상 큐브 칸을 채우세요)'} (frame shoulder_roll_axis, 1 Hz)")

    def _stop_node(self, n):
        self.nodes.pop(n, None)
        if n == "ak45":
            for m in self.motors.values():
                m.has_target = m.command_active = False
            self.diag = {}
            self._log(n, "[가상] ak45_node 종료: 브레이크 0A → 홀드가 풀림. 가상 팔이 아래로 떨어진다")
        elif n == "arm_control":
            self.ctrl = None
            self._log(n, "[가상] arm_control_node 종료")
        elif n == "homing_guard":
            self.guard = None
            self.homed_fb = {}
            self._log(n, "[가상] homing_guard 종료 (마지막 원점 상태는 오래된 값으로 남음)")
        else:
            self._log(n, f"[가상] {n} 종료")

    def stop_all(self):
        with self.lock:
            for n in list(self.nodes):
                self._stop_node(n)

    def _log(self, node, text):
        owner = (self.nodes.get(node) or {}).get("owner") or node
        try:
            self.log_cb(owner, text)
        except Exception:
            pass

    # ------------------------------------------------------------------ 입력 (대시보드가 부름)
    def power(self, on):
        """AK45 4개 가상 전원. 다시 켜면 엔코더 영점이 15~40° 틀어진다."""
        with self.lock:
            for m in self.motors.values():
                if on and not m.powered:
                    m.enc_off += random.choice((-1, 1)) * random.uniform(15.0, 40.0)
                    m.temp = max(AMBIENT_C, m.temp - 2.0)
                m.powered = bool(on)
            self._log("ak45", f"[가상] AK45 전원 {'켬 — 엔코더 영점이 틀어졌다(원점 다시 잡기)' if on else '끔 — 피드백 끊김, 힘 빠짐'}")
            return {"ok": True, "msg": f"가상 AK45 전원 {'켬 (영점 틀어짐)' if on else '끔'}"}

    def set_cube(self, pose):
        with self.lock:
            self.cube = tuple(pose) if pose else None
        return {"ok": True, "msg": f"가상 큐브 {pose}" if pose else "가상 큐브 지움"}

    def joint_command(self, names, positions):
        """/arm_control/joint_command (sim 관절명, rad). arm_control_node 의 on_command 와 같은 검사."""
        with self.lock:
            if "arm_control" not in self.nodes or self.ctrl is None:
                return {"ok": False, "error": "/arm_control/joint_command 구독자 없음 (가상 arm_control_node 꺼짐)"}
            if len(names) != len(positions):
                return {"ok": False, "error": "name / position 길이가 다름"}
            for n, p in zip(names, positions):
                if n not in self.joints:
                    return {"ok": False, "error": f"unknown joint {n}; message ignored"}
                if not math.isfinite(p):
                    return {"ok": False, "error": f"non-finite target for {n}; message ignored"}
            for n, p in zip(names, positions):
                m, clipped = self.joints[n].to_motor(math.degrees(p))
                if clipped:
                    self._log("arm_control", f"[가상] {n}: {math.degrees(p):.1f} deg clipped to motor {m:.1f}")
                self.ctrl["mover"].set_target(n, m)
            return {"ok": True, "msg": f"가상: {len(names)}개 관절 목표 보냄"}

    def send_joint_command(self, names, positions):   # RosBridge 와 같은 이름 (서버가 부름)
        return self.joint_command(list(names), [float(p) for p in positions])

    def call_trigger(self, service, timeout=2.0):
        prefix = "/arm_control/"
        with self.lock:
            if self.guard is None:
                return {"ok": False, "error": f"{service} 없음: homing_guard 가 안 켜짐 (가상 arm_homed.launch.py 를 켜세요)"}
            if not service.startswith(prefix):
                return {"ok": False, "error": f"{service}: 모르는 서비스"}
            hm = self.guard["homing"]
            act, _, joint = service[len(prefix):].partition("_")
            try:
                if joint == "all":
                    names = list(hm.m) if act == "release" else [n for n, h in hm.m.items() if not h.homed]
                else:
                    names = [hm.by_joint(joint)]
            except KeyError:
                return {"ok": False, "error": f"{service} 없음"}
            if act == "home":
                return self._srv_home(names)
            if act == "release":
                return self._srv_release(names)
            return {"ok": False, "error": f"{service}: 모르는 서비스"}

    def _srv_home(self, names):
        g, now = self.guard, time.monotonic()
        texts, ok = [], True
        for n in names:
            good, text = g["homing"].home(n, now)
            texts.append(text)
            ok = ok and good
            if good:
                self._forget(n)
                g["mover"].start(n, g["homing"].m[n].raw_deg)
                self._hold([n])
                g["pending_hold"].discard(n)
        self._log("homing_guard", "[가상] " + ("; ".join(texts) or "nothing to home (all homed)"))
        self._publish_homing()
        msg = "; ".join(texts) or "nothing to home (all homed)"
        return {"ok": True, "msg": msg} if ok else {"ok": False, "error": msg}

    def _srv_release(self, names):
        hm = self.guard["homing"]
        released = [hm.m[n].joint.joint for n in names if hm.release(n)]
        for n in names:
            self._forget(n)
        text = (f"released (rest, NOT homed): {', '.join(released) or 'none were homed'}. ak45_node still "
                "holds the last target; stop ak45_node to take the torque off")
        self._log("homing_guard", "[가상] " + text)
        self._publish_homing()
        return {"ok": True, "msg": text}

    # ------------------------------------------------------------------ guard 도우미 (homing_guard.py 와 같음)
    def _forget(self, n):
        mv = self.guard["mover"]
        mv.current.pop(n, None)
        mv.target.pop(n, None)

    def _hold(self, names):
        for n in names:
            raw = self.guard["homing"].m[n].raw_deg
            if raw is not None:
                self._ak45_command(n, raw)

    def _publish_homing(self):
        if self.guard is None:
            return
        s = self.guard["homing"].status(time.monotonic())
        s["stamp"] = time.time()
        self.homing_status, self.homing_t = s, time.time()

    # ------------------------------------------------------------------ ak45 / servo 명령
    def _ak45_command(self, name, deg):
        if "ak45" not in self.nodes:
            return
        m = self.motors.get(name)
        if m is None:
            return
        if abs(deg) > MAX_COMMAND_DEG:
            self._log("ak45", f"[가상] 목표각 {deg:.1f}도가 상한 {MAX_COMMAND_DEG:.1f}도를 넘습니다. 무시합니다.")
            return
        if not m.has_target:
            self._log("ak45", f"[가상] 목표각 수신(첫 명령): {name} = {deg:.2f}도")
        m.target, m.has_target = deg, True

    def _servo_command(self, joint, deg):
        if "stm_bridge" not in self.nodes:
            return
        self.servo_sent[joint.name] = (min(max(deg, 0.0), 180.0), time.monotonic())

    # ------------------------------------------------------------------ 50 Hz
    def _loop(self):
        dt = 1.0 / HZ
        nxt = time.monotonic()
        while not self._stop.is_set():
            nxt += dt
            try:
                with self.lock:
                    self._step(dt)
            except Exception as e:   # 가상 팔이 멈추면 화면에 보이게
                self._log("ak45", f"[가상] 시뮬레이션 오류: {type(e).__name__}: {e}")
            time.sleep(max(0.0, nxt - time.monotonic()))
            if time.monotonic() - nxt > 0.5:
                nxt = time.monotonic()

    def _step(self, dt):
        now = time.monotonic()
        self._tick += 1
        ak_on = "ak45" in self.nodes
        # 1. 모터 물리 + ak45_node 재송신 규칙
        for name, m in self.motors.items():
            wd_ok = m.powered and m.fb_t is not None and now - m.fb_t <= 0.2
            if ak_on and m.has_target and not m.temp_locked and m.powered:
                if m.temp >= MAX_TEMP_C:
                    m.temp_locked, m.command_active = True, False
                    self._log("ak45", f"[가상] {name} 온도 {m.temp:.0f}도. 명령 송신을 중단합니다.")
                else:
                    m.command_active = wd_ok or m.fb_t is None
            else:
                m.command_active = False
            if m.command_active:
                goal = m.target + m.enc_off
                v = max(-MOTOR_SPEED, min(MOTOR_SPEED, (goal - m.phys) / MOTOR_TAU))
                m.current = 0.25 + 0.012 * abs(v) + 0.004 * abs(goal - m.phys)
            else:
                d = -m.phys
                v = max(-FALL_SPEED, min(FALL_SPEED, d / 0.15))
                m.current = 0.0
            m.phys += v * dt
            m.vel = v
            m.temp += (0.8 * m.current ** 2 - 0.01 * (m.temp - AMBIENT_C)) * dt
            if m.powered:
                m.fb_t = now
        if self.motors and any(m.powered for m in self.motors.values()):
            self.joint_t = now
        # 2. homing_guard: on_state (피드백) / on_timer
        if self.guard is not None:
            g, hm = self.guard, self.guard["homing"]
            changed, hold = False, []
            for n, m in self.motors.items():
                if not m.powered or not ak_on:     # /joint_states 는 ak45_node 가 발행
                    continue
                event, text = hm.feedback(n, m.raw(), now)
                if event:
                    self._log("homing_guard", "[가상] " + text)
                    self._forget(n)
                    g["pending_hold"].add(n)
                    changed = True
                if n in g["pending_hold"]:
                    hold.append(n)
                d = hm.from_raw(n, m.raw())
                if d is not None:
                    self.homed_fb[n] = (d, now)
            if hold:
                self._hold(hold)
                g["pending_hold"].difference_update(hold)
                self._log("homing_guard", f"[가상] told ak45_node to hold {', '.join(hold)} where it is now")
            for text in hm.check_lost(now):
                self._log("homing_guard", "[가상] " + text)
                changed = True
            for n in [n for n, h in hm.m.items() if h.lost]:
                self._forget(n)
                g["pending_hold"].add(n)
            for n in g["mover"].needs_start():
                h = hm.m[n]
                if h.homed and h.seen is not None and now - h.seen <= hm.max_age_s:
                    g["mover"].start(n, h.raw_deg)
            for n, deg in g["mover"].step().items():
                self._ak45_command(n, deg)
            if changed or self._tick % 25 == 0:
                self._publish_homing()
        # 3. arm_control_node on_timer
        if self.ctrl is not None:
            mv = self.ctrl["mover"]
            for n in mv.needs_start():
                j = self.joints[n]
                if j.motor == "servo":
                    t = mv.target[n]
                    mv.start(n, t)
                    self._servo_command(j, t)
                    continue
                fb = self._ctrl_feedback(j, now)
                if fb is None:
                    if self._tick % 250 == 0:
                        self._log("arm_control", f"[가상] no /joint_states for ak45_{j.id}; {n} waits")
                    continue
                mv.start(n, fb)
            for n, deg in mv.step().items():
                j = self.joints[n]
                if j.motor == "ak45":
                    self._ak45_command_from_ctrl(j, deg)
                else:
                    self._servo_command(j, deg)
        # 4. 진단 2 Hz
        if ak_on and self._tick % 25 == 0:
            self._diagnostics(now)
        # 5. vision 1 Hz
        if "vision" in self.nodes and self._tick % 50 == 0 and self.cube:
            x, y, z, yaw = self.cube
            self.obj = {"x": x, "y": y, "z": z, "yaw_deg": yaw, "frame_id": "shoulder_roll_axis",
                        "stamp": time.time(), "t": time.time()}

    def _ctrl_feedback(self, j, now, max_age=0.5):
        n = f"ak45_{j.id}"
        if self.guard is not None:     # arm_homed: /ak45/joint_states_homed
            got = self.homed_fb.get(n)
            return got[0] if got and now - got[1] <= max_age else None
        m = self.motors[n]
        on = "ak45" in self.nodes
        return m.raw() if on and m.powered and m.fb_t and now - m.fb_t <= max_age else None

    def _ak45_command_from_ctrl(self, j, deg):
        n = f"ak45_{j.id}"
        if self.guard is not None:     # /ak45/command_unguarded -> guard.on_command
            hm = self.guard["homing"]
            raw = hm.to_raw(n, deg)
            if raw is None:
                self._forget(n)
                if self._tick % 100 == 0:
                    self._log("homing_guard", f"[가상] not homed, command ignored: {j.joint}")
                return
            self.guard["mover"].set_target(n, raw)
        else:
            self._ak45_command(n, deg)

    def _diagnostics(self, now):
        t = time.time()
        for name, m in self.motors.items():
            wd_ok = m.powered and m.fb_t is not None and now - m.fb_t <= 0.2
            if not m.powered and m.fb_t is None:
                lv, msg = 1, "피드백 수신 없음 (CubeMarsTool의 Send status over CAN Rate(Hz) 확인)"
            elif not wd_ok:
                lv, msg = 2, "피드백 끊김: 200ms 초과"
            else:
                lv, msg = 0, "정상"
            pos = m.raw() if m.powered else (m.phys - m.enc_off)
            vals = {"position_deg": _f2(pos), "speed_erpm": _f2(m.vel / 6.0 * 14.0), "current_a": _f2(m.current),
                    "temperature_c": str(int(m.temp)), "error_code": "0", "watchdog_ok": "1" if wd_ok else "0",
                    "target_deg": _f2(m.target) if m.has_target else "none",
                    "command_active": "1" if m.command_active else "0",
                    "position_error_deg": _f2(m.target - pos) if m.has_target else "none"}
            self.diag[f"ak45/{name}"] = {"level": lv, "message": msg + " (가상)", "hardware_id": f"ak45_id_0x{m.joint.id:02X} (가상)",
                                         "values": vals, "t": t}
            # homing_guard.on_diagnostics: command_active 1 -> 0 이면 원점을 지운다
            if self.guard is not None:
                text = self.guard["homing"].command_active(name, m.command_active)
                if text:
                    self._log("homing_guard", "[가상] " + text)
                    self._forget(name)
                    self._publish_homing()
        self.diag["ak45/can_bus"] = {"level": 0, "message": "정상 (가상 버스)", "hardware_id": "can0 (가상)",
                                     "values": {"state": "ERROR-ACTIVE", "err_frames": "0", "bit_stuff_errors": "0",
                                                "form_errors": "0", "ack_errors": "0", "busoff_count": "0",
                                                "restart_count": "0"}, "t": t}
        self.diag_t = t

    # ------------------------------------------------------------------ 궤적 재생 (역기구학 탭, 가상 모드 전용)
    def play_trajectory(self, traj, max_speed, hold=2.0, on_end=None):
        """on_end(reason): 재생이 끝나면 (재생 스레드에서) 부른다 — live_api.Scene 이 '끝까지 재생' 이면 옮긴 큐브를 놓을 자리로."""
        from arm_control import play as P
        with self.lock:
            if self.play.get("running"):
                return {"ok": False, "error": "이미 재생 중"}
            if "arm_control" not in self.nodes:
                return {"ok": False, "error": "가상 arm_control_node 가 꺼져 있음 (연결 탭에서 arm.launch.py 또는 arm_homed 켜기)"}
            if self.guard is not None:   # arm_homed: 원점 없는 AK45 명령은 guard 가 버린다 → 서보만 움직이는 재생이 되지 않게 실물처럼 거절
                hm = self.guard["homing"]
                not_homed = [n for n in traj["joint_names"] if n in self.joints and self.joints[n].motor == "ak45"
                             and not hm.m[f"ak45_{self.joints[n].id}"].homed]
                if not_homed:
                    return {"ok": False, "error": "(가상) 원점 없는 AK45: " + ", ".join(not_homed) +
                            " — 각도 제어 탭에서 '전부 원점 잡기' 먼저 (원점 없으면 guard 가 AK45 명령을 막아 서보만 움직인다)"}
            period, slow, peak = P.schedule(traj, max_speed)
            n = len(traj["positions"])
            self._play_stop.clear()
            import play_phase
            self.play = {"running": True, "frame": 0, "frames": n, "period": period, "slow": slow, "hold": hold,
                         "seconds": hold + (n - 1) * period, "started": time.time(), "reason": None,
                         "phases": play_phase.phases(traj)}   # 지금 무슨 동작 (snapshot 이 play_phase.view 로)

        state = {"k": 0}

        def publish(names, positions):
            self.joint_command(list(names), [float(p) for p in positions])
            state["k"] += 1
            self.play["frame"] = max(0, state["k"] - max(1, int(hold * 10)))

        def run():
            try:
                reason = P.play(publish, traj, max_speed, hold, stop=self._play_stop)
            except Exception as e:
                reason = f"{type(e).__name__}: {e}"
            if on_end is not None:
                try:
                    on_end(reason or "끝까지 재생")
                except Exception as e:
                    self._log("arm_control", f"[가상] 재생 끝 처리 오류: {type(e).__name__}: {e}")
            self.play.update(running=False, reason=reason or "끝까지 재생", ended=time.time())
            self._log("arm_control", f"[가상] 궤적 재생 끝: {reason or '끝까지'}")

        threading.Thread(target=run, daemon=True).start()
        return {"ok": True, "msg": f"가상 팔로 재생: {n} 프레임, {slow:.1f}배 느리게, 약 {self.play['seconds']:.0f}초"}

    def stop_trajectory(self):
        self._play_stop.set()
        return {"ok": True, "msg": "재생 멈춤 요청"}

    # ------------------------------------------------------------------ 상태 (RosMonitor.snapshot 과 같은 모양)
    def snapshot(self):
        now_m, now = time.monotonic(), time.time()
        with self.lock:
            motors, bus = [], None
            for name, d in sorted(self.diag.items()):
                row = dict(d, name=name, age=now - d["t"])
                if name == "ak45/can_bus":
                    bus = row
                else:
                    m = self.motors[name.split("/", 1)[1]]
                    row["joint_deg"] = m.raw() if m.powered else None
                    row["joint_age"] = (now_m - m.fb_t) if m.fb_t else None
                    motors.append(row)
            joints = {}
            if "ak45" in self.nodes:   # /joint_states 는 ak45_node 가 발행
                joints = {n: {"deg": m.raw(), "age": now_m - m.fb_t} for n, m in self.motors.items() if m.fb_t}
            if self.homing_status is None:
                homing = {"available": True, "seen": False, "joints": []}
            else:
                hj = []
                for jn, j in (self.homing_status.get("joints") or {}).items():
                    hj.append(dict(j, joint=jn))
                homing = {"available": True, "seen": True, "all_homed": bool(self.homing_status.get("all_homed")),
                          "age": now - self.homing_t, "stamp": self.homing_status.get("stamp"), "joints": hj}
            return {
                "available": True, "reason": "", "virtual": True,
                "motors": motors, "bus": bus,
                "diag_age": (now - self.diag_t) if self.diag_t and "ak45" in self.nodes else None,
                "joint_age": (now_m - self.joint_t) if self.joint_t and "ak45" in self.nodes else None,
                "homing": homing,
                "joints": joints,
                "joints_homed": {n: {"deg": d, "age": now_m - t} for n, (d, t) in self.homed_fb.items()},
                "servos": {n: {"deg": d, "age": now_m - t} for n, (d, t) in self.servo_sent.items()},
                "servo_age": (now_m - max(t for _, t in self.servo_sent.values())) if self.servo_sent else None,
                "object_pose": dict(self.obj, age=now - self.obj["t"]) if self.obj else None,
                "cmd_subs": 1 if "arm_control" in self.nodes else 0,
                "virtual_state": {
                    "nodes": sorted(self.nodes), "power": all(m.powered for m in self.motors.values()),
                    "cube": list(self.cube) if self.cube else None, "playback": _play_view(self.play),
                    "phys_deg": {n: round(m.phys, 2) for n, m in self.motors.items()},
                    "enc_off_deg": {n: round(m.enc_off, 2) for n, m in self.motors.items()},
                },
                "conn": {"state": "virtual", "reason": "가상 팔", "pid": None},
            }
