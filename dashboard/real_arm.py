"""실물 팔을 ROS 없이: 이 프로세스가 can0 을 직접 열고(python-can socketcan) AK45 4개와 STM32 서보 노드를 다룬다.

대시보드의 '실물(직접 CAN)' 모드가 이것을 쓴다. 화면 쪽 인터페이스는 virtual_arm.VirtualArm 과 같다
(snapshot / send_joint_command / call_trigger / play_trajectory / stop_trajectory) — 같은 화면이 그대로 돈다.

    ak45_ros2 의 ak45_node + stm_bridge_node + arm_control_node + homing_guard 가 하던 일을 한 프로세스에서:

스레드 2개 (robot_arm v2 robot_v2.py class AK 와 같은 구조, 프레임만 CAN):
  rx       bus.recv → AK45 피드백 0x29(50 Hz 자동 업로드) 파싱, CAN 에러 프레임(bus-off 등) 집계,
           다른 프로그램이 같은 모터에 보내는 명령 프레임 감지.
  control  50 Hz. AK45 관절마다: 목표 → 사다리꼴 램프(속도 max_speed, 가속 max_acc, 한 스텝 max_step 이하)
           → carrot(명령을 측정 위치 ± carrot_deg 안으로) → 위치 프레임(모드 4). 움직이는 동안 50 Hz, 서 있으면 hold_hz(20).
           서보: arm_control.motion.Mover 로 램프 → servo_frame(StdId 5), 바뀔 때 + servo_resend_hz 로 재전송.

안전 규칙 (괄호 = 기본값, DEFAULTS):
  - 첫 명령 전엔 AK45 에 아무 프레임도 안 보낸다. 피드백 없는(또는 끊긴) 모터에는 램프를 시작하지 않는다.
  - 원점(arm_control.homing.Homing 그대로): 세션마다 원점을 다시 잡는다. 원점 없는 AK45 관절 명령은 거절.
    전원 재투입(긴 끊김)·위치 튐·힘 빼기·쉬게 하기·고장 → 원점 지움.
  - 피드백 짧은 끊김(≤ gap_keep_s 5 s): 마지막 명령 자리를 계속 보낸다(홀드 유지, 램프는 멈춤). 돌아왔을 때
    실제가 명령과 2° 넘게 다르면 지금 자리에서 다시 램프. 긴 끊김(> 5 s): 그 모터 송신 멈춤 + 원점 지움.
    다시 들어오면 지금 측정 자리에서 홀드(옛 목표로 가지 않음). 짧은 끊김이라도 돌아온 위치가 끊기기 전 자리·홀드 명령과
    resume_rehome_deg(3°) 넘게 다르면 전원 재투입 의심 → 원점 지움 + 지금 자리 홀드(엔코더 영점이 60° 미만으로 바뀐 경우).
  - rx 스레드 몸통 전체 보호(예외는 5 s 에 한 줄 기록), recv 1 s 넘게 실패하면 버스 다시 엶, 죽은 스레드는 다시 띄움(화면 빨강).
  - 누르기 멈춤: |전류| ≥ press_amps(3 A, 가정) + 오차 ≥ 2° 가 1 s → 목표를 지금 자리로(힘 유지, 명령 하나에 한 번). 0 = 끔.
  - 궤적 재생은 play.play 대신 자체 루프: 첫 프레임 홀드 동안에도 STOP·고장을 보고, 멈추면 지금 자리 홀드.
  - 위치 튐(피드백 두 개 사이 > jump_deg 60°): 지금 자리로 다시 홀드 + 원점 지움(원점 의심).
  - 막힘(명령 − 실제 > stall_deg 12° 가 stall_s 2 s): 그 관절 힘 뺌(프레임 중단 + 브레이크 0 A) + 원점 지움.
  - 온도: temp_warn 60 ℃ 경고(55 ℃ 아래로 식어야 풀림), temp_stop 75 ℃ 그 관절 힘 뺌(60 ℃ 아래에서만 원점 다시).
    에러 코드 ≠ 0: 그 관절 힘 뺌.
    (이 모터는 무부하에서 53 ℃ 까지 오른 기록이 있다 — ak45_can/docs/FINDINGS.md)
  - |모터 명령| > max_command_deg(360) 거절.
  - STOP(hold_all): 모든 AK45 를 지금 측정 자리에 홀드, 서보 램프 멈춤, 궤적 재생 멈춤. 팔이 안 떨어진다.
  - 힘 빼기(limp): AK45 프레임 중단 + 브레이크(모드 2, 0 A) 3번(2 ms 간격, 실패면 재시도). 팔이 떨어진다. 서보는 펌웨어에 '끄기'가 없어 그대로.
  - 같은 can0 을 쓰는 ak45_node 가 떠 있으면 시작 거부: ak45_can 과 같은 잠금 파일 /tmp/ak45_ctrl.lock 을 flock.
  - close(): 기본은 브레이크 0 A (프로세스가 끝나면 어차피 홀드 프레임이 끊겨 힘이 빠진다 — 숨기지 않고 명시).

프레임 (ak45_can/src/ak45_36_socketcan_control.cpp, CubeMars 매뉴얼 5.1/5.2, team-jax/CAN_STM):
  위치 = 확장 ID (4 << 8) | id, int32 BE deg×10000  (arm_control.protocol.ak45_pos_frame)
  브레이크 = 확장 ID (2 << 8) | id, int32 BE A×1000  (ak45_set_current_brake 와 같은 바이트)
  피드백 = 확장 ID (0x29 << 8) | id: pos int16×0.1°, spd int16×10 ERPM, cur int16×0.01 A, temp int8, err uint8
  서보 = 표준 ID 5, [motor_id][angle×10 H][L]  (stm_bridge.protocol.servo_frame). STM32 쪽은 피드백 없음.
"""
import math
import os
import struct
import sys
import threading
import time
from collections import deque
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
for _p in (REPO / "arm_control", REPO / "stm_bridge"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from arm_control import calibration as _cal   # noqa: E402  (rclpy 없음)
from arm_control.homing import Homing, load_home_poses   # noqa: E402
from arm_control.motion import Mover   # noqa: E402
from arm_control.protocol import AK45_ERRORS, AK45_FEEDBACK, ak45_pos_frame, parse_ak45_feedback   # noqa: E402
from arm_control.watch import TrackingWatch   # noqa: E402
from stm_bridge.protocol import SERVO_STDID, Frame, servo_frame   # noqa: E402

LOCK_PATH = "/tmp/ak45_ctrl.lock"     # ak45_can LOCK_FILE_PATH 와 같은 파일
SERVO_PIN_MAX = 9            # 서보 핀(STM32 채널) 번호 상한 — 펌웨어 motor_id 0..9 (stm_bridge protocol SERVO_COUNT)
START_TOL_DEG = 5.0           # 실물 재생: 첫 프레임과 지금 자세가 이만큼(°) 넘게 다르면 거절 (첫 프레임까지는 검사 없는 관절 직선)
AK45_MODE_BRAKE = 2

DEFAULTS = {
    "max_speed": 30.0,        # °/s  AK45 램프 속도 상한
    "servo_speed_x": 2.0,     # 서보 램프 속도 = max_speed × 이것 (2026-11-23 사용자: 서보 2배)
    "max_acc": 90.0,          # °/s² 사다리꼴 가속 (robot_arm AK_ACC)
    "max_step": 3.0,          # °    한 제어 스텝 최대 이동 (robot_arm)
    "carrot": 30.0,           # °    명령을 측정 위치 ± 이 안으로 (robot_arm 45 → 보수적으로 30)
    "ctrl_hz": 50.0,          # Hz   제어 루프 / 움직이는 동안 송신
    "hold_hz": 20.0,          # Hz   서 있을 때 홀드 재송신 (robot_arm 20 Hz)
    "fresh_s": 0.25,          # s    피드백이 이보다 오래되면 '끊김' (50 Hz 업로드 기준 12 개)
    "gap_keep_s": 5.0,        # s    이 안의 끊김은 홀드 유지, 넘으면 멈춤 + 원점 지움 (robot_arm AK_GAP_KEEP_S)
    "resume_ramp_deg": 2.0,   # °    끊겼다 돌아올 때 이보다 벌어졌으면 지금 자리에서 다시 램프
    # °  짧은 끊김(≤ gap_keep_s)에서 돌아온 위치가 끊기기 전 마지막 위치와도, 홀드하던 명령과도 이보다 다르면
    #    = 그동안 모터가 홀드를 못 했다(전원 재투입·케이블) → 엔코더 영점이 바뀌었을 수 있어 원점 지우고 지금 자리 홀드.
    #    근거: 홀드가 살아 있으면 위치는 명령 근처(가짜 모터 < 0.5°, 움직이던 중 끊겨도 램프 지연 ~1° = 30 °/s × 0.03 s),
    #    jump_deg 60° 보다 작은 영점 변화(예 30°)를 잡으려면 그보다 훨씬 작아야 함. 실물 홀드 오차는 미측정(가정).
    "resume_rehome_deg": 3.0,
    "jump_deg": 60.0,         # °    피드백 튐 = 원점 의심
    "stall_deg": 12.0,        # °    막힘: 명령 − 실제
    "stall_s": 2.0,           # s    막힘이 이만큼 이어지면 힘 뺌
    # 누르기 멈춤(바닥·물체를 작은 오차로 계속 누름 = 막힘 12° 에 안 걸리는데 전류·열만 오름, robot_arm 10-21 113 ℃):
    # |전류| ≥ press_amps 이고 |명령 − 실제| ≥ press_deg 가 press_s 이어지면 목표를 지금 자리로 풀어 준다(힘은 유지, 안 떨어짐).
    # 명령 하나에 한 번만(중력 처짐을 따라 계속 내려가지 않게). press_amps 0 = 끔. 3 A 는 실물 전류 미측정 상태의 가정.
    "press_amps": 3.0,        # A
    "press_deg": 2.0,         # °
    "press_s": 1.0,           # s
    "temp_warn": 60.0,        # ℃
    "temp_stop": 75.0,        # ℃    그 관절 힘 뺌
    "temp_hyst": 5.0,         # ℃    경고는 temp_warn − 이 값 아래로 식어야 풀림. 온도로 힘 뺀 관절은 temp_warn 아래에서만 원점 다시
    "max_command_deg": 360.0,  # °   ak45_node.yaml 과 같음
    "servo_resend_hz": 20.0,  # Hz   서보 마지막 값 재전송 (0 = 안 함). 펌웨어는 마지막 펄스를 유지한다. 2026-11-24 1 → 20: CAN 에서 빠진 프레임을 0.05 s 안에 메움 (어제 실물 손목이 끊기며 느렸다)
    "calib_step_max": 10.0,   # °    보정에서 한 번에 움직일 수 있는 양
}

# linux/can/error.h
CAN_ERR_CRTL, CAN_ERR_PROT, CAN_ERR_ACK, CAN_ERR_BUSOFF, CAN_ERR_RESTARTED = 0x04, 0x08, 0x20, 0x40, 0x100
CRTL_WARN, CRTL_PASSIVE, CRTL_ACTIVE = 0x04 | 0x08, 0x10 | 0x20, 0x40
PROT_FORM, PROT_STUFF = 0x02, 0x04


def _play_view(play):
    """snapshot 의 playback: 단계표(phases)는 빼고 지금 단계 글·번호·남은 초 (play_phase.view)."""
    try:
        import play_phase
        return play_phase.view(play)
    except Exception:
        p = dict(play)
        p.pop("phases", None)
        return p


def ak45_brake_frame(controller_id, amps=0.0):
    """모드 2 (current brake). ak45_set_current_brake 와 같은 바이트: int32 BE, A × 1000."""
    amps = min(max(float(amps), 0.0), 5.0)
    return Frame((AK45_MODE_BRAKE << 8) | controller_id, True, struct.pack(">i", int(amps * 1000.0)))


def take_lock(path=LOCK_PATH):
    """ak45_can 과 같은 flock. 다른 프로세스(ak45_node 등)가 잡고 있으면 RuntimeError. 반환: fd (또는 None)."""
    if not path:
        return None
    try:
        import fcntl
    except ImportError:       # Windows: 잠금 없음 (가짜 버스 시험용)
        return None
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        raise RuntimeError(f"다른 프로그램이 이미 AK45 버스를 쓰는 중 ({path} 잠김: ak45_node / arm.launch.py / "
                           "arm_homed.launch.py 가 떠 있나?). 그것을 끈 뒤 다시")
    return fd


class AkMotor:
    def __init__(self, joint):
        self.joint = joint
        self.id = joint.id
        self.name = f"ak45_{joint.id}"
        # 피드백
        self.pos = self.spd = self.cur = None
        self.temp = None
        self.err = 0
        self.fb_t = None            # monotonic
        self.fb_n = 0
        # 명령
        self.target = None          # 최종 목표 (raw 모터 °). None = 프레임 안 보냄
        self.cmd = None             # 램프된 지금 명령
        self.vel = 0.0
        self.sent_t = 0.0
        self.sent_n = 0
        self.last_sent = None
        self.stall_t = None
        self.fault = None           # 힘 뺀 이유 (막힘·온도·에러). 원점을 다시 잡을 때 지운다
        self.note = None            # 원점 의심 등 알림
        self.temp_warn_t = 0.0
        self.hot = False            # 온도 경고 중 (히스테리시스)
        self.temp_fault = False     # 온도로 힘 뺌 → temp_warn 아래로 식어야 원점 다시
        self.press_t = None         # 누르기 시작 시각
        self.pressed = False        # 이 명령에서 이미 한 번 풀어 줌
        self.stale_logged = False

    def fresh(self, now, fresh_s):
        return self.fb_t is not None and now - self.fb_t < fresh_s

    def active(self):
        return self.cmd is not None


class RealArm:
    """실물 AK45 4개 + 서보(STM32) 직접 CAN 제어. start() 로 버스를 열고 close() 로 닫는다."""

    def __init__(self, calibration_file=None, home_file=None, interface="socketcan", channel="can0",
                 lock_path=LOCK_PATH, params=None, log=None, log_file=None):
        self.p = dict(DEFAULTS)
        for k, v in (params or {}).items():
            if k in self.p and v is not None:
                self.p[k] = float(v)
        self.interface, self.channel, self.lock_path = interface, channel, lock_path
        self.log_cb = log or (lambda text: None)
        self.log_file = log_file
        self.lines = deque(maxlen=300)
        self.cal = _cal.load(calibration_file)
        self.joints = {j.joint: j for j in self.cal.joints}
        self.home_poses = load_home_poses(home_file)
        self.ak = {f"ak45_{j.id}": AkMotor(j) for j in self.cal.joints if j.motor == "ak45"}
        self.ak_by_id = {m.id: m for m in self.ak.values()}
        self.homing = Homing({n: m.joint for n, m in self.ak.items()}, self.home_poses,
                             gap_s=self.p["gap_keep_s"], jump_deg=self.p["jump_deg"], max_age_s=0.5)
        self.servo_mover = Mover(self.p["max_speed"] * self.p["servo_speed_x"], self.p["ctrl_hz"])
        self.servo_sent = {}        # 서보 이름 -> (모터 °, monotonic)
        self.servo_resend_t = 0.0
        self.lock = threading.RLock()
        self.bus = None
        self.lock_fd = None
        self.running = False
        self.started = None
        self.threads = []
        self.busst = {"state": "ERROR-ACTIVE", "err_frames": 0, "bit_stuff_errors": 0, "form_errors": 0,
                      "ack_errors": 0, "busoff_count": 0, "restart_count": 0, "send_errors": 0,
                      "last_send_error": "", "send_error_t": None, "send_fail_streak": 0, "rx_frames": 0,
                      "rx_errors": 0, "last_rx_error": "", "rx_error_t": None, "rx_restarts": 0, "ctrl_restarts": 0, "bus_reopens": 0}
        self._closing = False
        self.foreign = {}           # "AK45 id 2 모드 4" -> (count, monotonic)
        self.calib_dirty = False
        self.play = {"running": False}
        self._play_stop = threading.Event()
        self._play_bad = None
        self.stop_gen = 0           # STOP·힘 빼기·재생 멈춤마다 +1. 계획을 세우는 동안 STOP 이 눌렸는지 play_trajectory(stop_gen=) 가 본다
        self.rec = None             # run_record: 기록 중이면 rec(self, now) — 제어 루프가 틱마다 숫자 몇 개를 deque 에 넣는다 (파일 쓰기는 그쪽 스레드)

    # ------------------------------------------------------------------ 시작 / 끝
    def start(self):
        import can
        self._can = can
        self.lock_fd = take_lock(self.lock_path)
        try:
            if self.interface == "socketcan":
                self.bus = can.Bus(interface="socketcan", channel=self.channel)
            else:
                self.bus = can.Bus(interface=self.interface, channel=self.channel)
        except Exception:
            self._unlock()
            raise
        self.running = True
        self._closing = False
        self.started = time.time()
        self.threads = [threading.Thread(target=self._rx_loop, daemon=True, name="ak45-rx"),
                        threading.Thread(target=self._ctrl_loop, daemon=True, name="ak45-ctrl")]
        for t in self.threads:
            t.start()
        unver = [j.joint for j in self.cal.joints if not j.verified]
        self._log(f"직접 CAN 시작: {self.interface} {self.channel}, AK45 {', '.join(self.ak)} + 서보 "
                  f"{sum(1 for j in self.cal.joints if j.motor == 'servo')}개. 첫 명령 전엔 아무 프레임도 안 보냄. "
                  f"속도 {self.p['max_speed']:g} °/s, carrot {self.p['carrot']:g}°, 온도 {self.p['temp_warn']:g}/{self.p['temp_stop']:g} ℃. "
                  f"모든 AK45 원점 없음(세션마다 다시). calibration {self.cal.path}, 미확인 {', '.join(unver) or '없음'}")
        return self

    def close(self, brake=True):
        """brake=True: AK45 에 브레이크 0 A 를 보내고 닫는다(팔이 떨어진다). 닫으면 홀드 프레임이 끊겨 어차피 힘이 빠진다."""
        self._play_stop.set()
        if not self.running:
            return
        with self.lock:
            # _closing: 이 뒤로 제어 루프는 위치 프레임을 안 보내고 joint_command 는 거절 → 재생 스레드가
            # 브레이크 뒤에 위치 프레임을 다시 내보내는 레이스를 막는다
            self._closing = True
            had = [m.name for m in self.ak.values() if m.target is not None]
            for m in self.ak.values():
                m.target = m.cmd = None
        if brake and self.bus is not None:
            for m in self.ak.values():
                self._brake(m)
        self.running = False
        for t in self.threads:
            t.join(timeout=1.0)
        try:
            self.bus.shutdown()
        except Exception:
            pass
        self.bus = None
        self._unlock()
        self._log("직접 CAN 닫음" + (f": 브레이크 0 A 보냄 (홀드하던 {', '.join(had)} 힘 빠짐)" if brake else ""))

    def _unlock(self):
        if self.lock_fd is not None:
            try:
                import fcntl
                fcntl.flock(self.lock_fd, fcntl.LOCK_UN)
            except Exception:
                pass
            os.close(self.lock_fd)
            self.lock_fd = None

    # ------------------------------------------------------------------ 기록
    def _log(self, text):
        line = time.strftime("%H:%M:%S ") + text
        self.lines.append(line)
        if self.log_file:
            try:
                with open(self.log_file, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except OSError:
                pass
        try:
            self.log_cb(text)
        except Exception:
            pass

    # ------------------------------------------------------------------ 송신
    def _send(self, fr, retries=0):
        """한 프레임. retries: 실패(ENOBUFS = can0 송신 큐 가득, txqueuelen 기본 10 등)면 2 ms 쉬고 다시."""
        for k in range(retries + 1):
            bus = self.bus
            if bus is None:
                return False
            try:
                bus.send(self._can.Message(arbitration_id=fr.arb_id, is_extended_id=fr.extended, data=fr.data), timeout=0.01)
                self.busst["send_fail_streak"] = 0
                return True
            except Exception as e:
                b = self.busst
                b["send_errors"] += 1
                b["send_fail_streak"] += 1
                b["last_send_error"] = f"{type(e).__name__}: {e}"[:160]
                if b["send_error_t"] is None or time.monotonic() - b["send_error_t"] > 5.0:
                    self._log(f"CAN 송신 실패: {b['last_send_error']} (can0 DOWN·bus-off·버퍼 가득?)")
                b["send_error_t"] = time.monotonic()
                if k < retries:
                    time.sleep(0.002)
        return False

    def _brake(self, m, n=3):
        """브레이크 0 A 를 n 번, 2 ms 간격 + 프레임마다 재시도(한꺼번에 쏟아 송신 큐를 넘치지 않게). 반환: 나간 개수."""
        ok = 0
        for k in range(n):
            if self._send(ak45_brake_frame(m.id), retries=3):
                ok += 1
            if k < n - 1:
                time.sleep(0.002)
        if ok == 0:
            m.note = "브레이크 프레임 못 보냄(CAN 송신 실패) — 위치 프레임은 멈췄음, 팔 받치고 버스 확인"
            self._log(f"{m.name} ⚠ {m.note}")
        return ok

    # ------------------------------------------------------------------ rx
    def _rx_problem(self, where, e):
        b = self.busst
        now = time.monotonic()
        b["rx_errors"] += 1
        b["last_rx_error"] = f"{where}: {type(e).__name__}: {e}"[:160]
        if b["rx_error_t"] is None or now - b["rx_error_t"] > 5.0:     # 로그는 5 s 에 한 줄 (can0 DOWN 때 폭주 막기)
            self._log(f"CAN {b['last_rx_error']} (누적 {b['rx_errors']}개, 5 s 마다 한 줄만 기록)")
        b["rx_error_t"] = now

    def _rx_loop(self):
        """몸통 전체를 보호: 프레임 하나 처리 중 예외가 나도 스레드는 산다(죽으면 피드백이 끊겨 5 s 뒤 팔이 떨어진다).
        recv 가 1 s 넘게 계속 실패하면(USB-CAN 이 빠졌다 다시 꽂힘 등) 버스를 다시 연다(2 s 마다 시도)."""
        fail_since, reopen_t = None, 0.0
        while self.running:
            try:
                msg = self.bus.recv(0.02)
            except Exception as e:
                if not self.running:
                    break
                self._rx_problem("수신 오류", e)
                now = time.monotonic()
                fail_since = fail_since or now
                if now - fail_since > 1.0 and now - reopen_t > 2.0:
                    reopen_t = now
                    self._reopen_bus()
                time.sleep(0.1)
                continue
            fail_since = None
            if msg is None:
                continue
            try:
                now = time.monotonic()
                self.busst["rx_frames"] += 1
                if msg.is_error_frame:
                    self._on_error_frame(msg)
                    continue
                st = parse_ak45_feedback(msg.arbitration_id, msg.is_extended_id, bytes(msg.data))
                if st is not None:
                    self._on_feedback(st, now)
                    continue
                self._on_foreign(msg, now)
            except Exception as e:
                self._rx_problem("수신 처리 오류", e)

    def _reopen_bus(self):
        try:
            new = self._can.Bus(interface=self.interface, channel=self.channel)
        except Exception as e:
            self._rx_problem("버스 다시 열기 실패", e)
            return False
        if not self.running:
            new.shutdown()
            return False
        old, self.bus = self.bus, new
        self.busst["bus_reopens"] += 1
        try:
            old.shutdown()
        except Exception:
            pass
        self._log(f"CAN 버스 다시 엶 ({self.interface} {self.channel}, {self.busst['bus_reopens']}번째) — 짧은 끊김은 홀드 유지")
        return True

    def _check_threads(self):
        """죽은 rx / control 스레드를 다시 띄운다(제어 루프와 snapshot 이 부름)."""
        if not self.running or self._closing:
            return
        for i, (target, name) in enumerate(((self._rx_loop, "ak45-rx"), (self._ctrl_loop, "ak45-ctrl"))):
            t = self.threads[i] if i < len(self.threads) else None
            if t is not None and t.is_alive():
                continue
            if t is not None and t is threading.current_thread():
                continue
            nt = threading.Thread(target=target, daemon=True, name=name)
            self.threads[i] = nt
            self.busst["rx_restarts" if i == 0 else "ctrl_restarts"] += 1
            self._log(f"⚠ {name} 스레드가 죽어 있어 다시 띄움 (예외로 끝남?)")
            nt.start()

    def _on_error_frame(self, msg):
        cid, d = msg.arbitration_id, bytes(msg.data) + bytes(8)
        b = self.busst
        with self.lock:
            b["err_frames"] += 1
            if cid & CAN_ERR_BUSOFF:
                b["state"] = "BUS-OFF"
                b["busoff_count"] += 1
                self._log("CAN bus-off: 송신 불가. 배선·종단 저항 확인, 복구는 "
                          "sudo ip link set can0 type can bitrate 1000000 restart-ms 100 (또는 down/up)")
            elif cid & CAN_ERR_RESTARTED:
                b["state"] = "ERROR-ACTIVE"
                b["restart_count"] += 1
            elif cid & CAN_ERR_CRTL:
                c = d[1]
                if c & CRTL_PASSIVE:
                    b["state"] = "ERROR-PASSIVE"
                elif c & CRTL_WARN:
                    b["state"] = "ERROR-WARNING"
                elif c & CRTL_ACTIVE:
                    b["state"] = "ERROR-ACTIVE"
            if cid & CAN_ERR_PROT:
                if d[2] & PROT_STUFF:
                    b["bit_stuff_errors"] += 1
                if d[2] & PROT_FORM:
                    b["form_errors"] += 1
            if cid & CAN_ERR_ACK:
                b["ack_errors"] += 1

    def _on_foreign(self, msg, now):
        """우리가 보내지 않은 명령 프레임(receive_own_messages 꺼짐) = 다른 프로그램이 같은 모터에 명령 중."""
        aid = msg.arbitration_id
        if msg.is_extended_id and (aid & 0xFF) in self.ak_by_id and (aid >> 8) <= 6:
            key = f"AK45 ID {aid & 0xFF} 모드 {aid >> 8}"
        elif not msg.is_extended_id and aid == SERVO_STDID:
            key = "서보 StdId 5"
        else:
            return
        with self.lock:
            n, t = self.foreign.get(key, (0, 0.0))
            if now - t > 10.0:
                self._log(f"⚠ 다른 프로그램이 버스에 {key} 명령을 보내는 중 — ak45_node·stm_bridge_node·cansend 등을 끄세요")
            self.foreign[key] = (n + 1, now)

    def _on_feedback(self, st, now):
        m = self.ak_by_id.get(st.controller_id)
        if m is None:
            return
        P = self.p
        with self.lock:
            was_fresh = m.fresh(now, P["fresh_s"])
            old, gap = m.pos, (None if m.fb_t is None else now - m.fb_t)
            m.pos, m.spd, m.cur, m.temp, m.err = st.position_deg, st.speed_erpm, st.current_a, st.temperature_c, st.error
            m.fb_t = now
            m.fb_n += 1
            event, text = self.homing.feedback(m.name, m.pos, now)   # 긴 끊김·튐 → 원점 지움
            if text:
                self._log(text)
            jump = old is not None and abs(m.pos - old) > P["jump_deg"]
            back = not was_fresh and gap is not None          # 끊겼다(fresh_s 넘게) 돌아온 첫 피드백
            # 짧은 끊김인데 위치가 끊기기 전 자리·홀드 명령 어느 쪽과도 resume_rehome_deg 넘게 다름 = 그동안 홀드 못 함
            # (전원 재투입이면 엔코더 영점이 60° 미만으로 바뀌어도 homing.feedback 은 못 알아챈다)
            moved = None
            if back and not jump and old is not None and gap <= P["gap_keep_s"]:
                moved = abs(m.pos - old) if m.cmd is None else min(abs(m.pos - old), abs(m.pos - m.cmd))
                if moved <= P["resume_rehome_deg"]:
                    moved = None
            if moved is not None and self._unhome(m.name, f"피드백 {gap:.1f}s 끊김 뒤 {moved:.1f}° 다른 자리(전원 재투입?) — 원점 다시 잡기"):
                self._log(f"{m.name} ⚠ 피드백 {gap:.2f}s 끊김 뒤 위치 {old:+.1f}° → {m.pos:+.1f}° ({moved:.1f}° > "
                          f"{P['resume_rehome_deg']:g}°): 원점 지움")
            if m.target is not None:
                if jump:
                    # robot_arm: 응답은 이어지는데 위치만 튐 = 밀려 넘어감 또는 전원 재투입 → 지금 자리 다시 홀드, 원점 의심
                    m.target = m.cmd = m.pos
                    m.vel = 0.0
                    m.note = f"위치 튐 {old:+.1f}° → {m.pos:+.1f}°: 지금 자리 홀드, 원점 다시 잡기"
                    self._log(f"{m.name} ⚠ {m.note}")
                    self._play_abort(m, m.note)
                elif back:
                    if m.cmd is not None and gap <= P["gap_keep_s"] and moved is None:
                        off = m.pos - m.cmd
                        if abs(off) > P["resume_ramp_deg"]:
                            m.cmd, m.vel = m.pos, 0.0
                        self._log(f"{m.name} 피드백 {gap:.2f}s 끊겼다 돌아옴 — 그동안 홀드 계속 보냄, 지금 {m.pos:+.1f}°"
                                  + (f" (명령과 {off:+.1f}° → 지금 자리에서 다시 램프)" if abs(off) > P["resume_ramp_deg"] else ""))
                    else:
                        # 긴 끊김(> gap_keep_s — 제어 루프가 아직 cmd 를 안 지운 경계 순간 포함), 또는 짧은 끊김인데 자리가 바뀜:
                        # 옛 목표로 가지 않고 지금 자리 홀드. 원점은 지워져 있음(homing.feedback 또는 위)
                        m.target = m.cmd = m.pos
                        m.vel = 0.0
                        m.note = (f"피드백 {gap:.1f}s 끊김 뒤 돌아옴(전원 재투입?)"
                                  + (f", {moved:.1f}° 다른 자리" if moved is not None else "") + ": 지금 자리 홀드, 원점 다시 잡기")
                        self._log(f"{m.name} ⚠ {m.note}")
                        self._play_abort(m, m.note)
                    m.stale_logged = False
            # 온도 / 에러 코드
            if m.temp is not None:
                if m.temp >= P["temp_warn"]:
                    if not m.hot or now - m.temp_warn_t > 30.0:
                        self._log(f"{m.name} ⚠ 온도 {m.temp} ℃ ≥ {P['temp_warn']:g} ℃ — 오래 홀드·막힘 확인")
                        m.temp_warn_t = now
                    m.hot = True
                elif m.hot and m.temp <= P["temp_warn"] - P["temp_hyst"]:
                    m.hot = False
                    self._log(f"{m.name} 온도 {m.temp} ℃ — 경고 풀림")
            if m.temp is not None and m.temp >= P["temp_stop"]:
                if m.target is not None or m.fault is None:
                    self._fault(m, f"온도 {m.temp} ℃ ≥ {P['temp_stop']:g} ℃")
                    m.temp_fault = True
            if m.err and (m.target is not None or m.fault is None):
                self._fault(m, f"에러 코드 {m.err} ({AK45_ERRORS.get(m.err, '?')})")

    # ------------------------------------------------------------------ 관절 멈춤 / 원점 지움
    def _unhome(self, name, reason):
        h = self.homing.m[name]
        was = h.homed
        h.homed, h.offset_deg, h.reason = False, 0.0, reason
        return was

    def _limp_motor(self, m, reason, brake=True):
        had = m.target is not None
        m.target = m.cmd = None
        m.vel, m.stall_t, m.press_t = 0.0, None, None
        self._unhome(m.name, reason + " — 원점 다시 잡기")     # 브레이크(몇 ms) 전에: 그 사이 원점이 남아 보이지 않게
        if brake and had:
            self._brake(m)
        return had

    def _play_abort(self, m, reason):
        if self.play.get("running"):
            self._play_bad = self._play_bad or f"{m.name}: {reason}"

    def _fault(self, m, reason):
        m.fault = reason
        had = self._limp_motor(m, f"힘 뺌: {reason}")
        self._log(f"{m.name} ✖ {reason} — {'힘 뺌(브레이크 0 A, 팔이 내려갈 수 있음)' if had else '명령 금지'}")
        self._play_abort(m, reason)

    # ------------------------------------------------------------------ control 50 Hz
    def _ctrl_loop(self):
        dt0 = 1.0 / self.p["ctrl_hz"]
        nxt = time.monotonic()
        last = nxt
        while self.running:
            nxt += dt0
            now = time.monotonic()
            dt = min(0.1, now - last)
            last = now
            try:
                with self.lock:
                    self._tick(now, dt)
                    rec = self.rec
                    if rec is not None:
                        rec(self, now)
            except Exception as e:
                if now - getattr(self, "_tick_err_t", 0.0) > 5.0:
                    self._tick_err_t = now
                    self._log(f"제어 루프 오류: {type(e).__name__}: {e}")
            try:
                self._check_threads()
            except Exception:
                pass
            time.sleep(max(0.0, nxt - time.monotonic()))
            if time.monotonic() - nxt > 0.5:
                nxt = time.monotonic()

    def _tick(self, now, dt):
        P = self.p
        if self._closing:          # close() 중: 브레이크 뒤에 위치 프레임이 나가지 않게
            return
        for text in self.homing.check_lost(now):
            self._log(text)
        hold_dt = 1.0 / P["hold_hz"]
        for m in self.ak.values():
            send = None
            fresh = m.fresh(now, P["fresh_s"])
            if m.target is not None and fresh:
                if m.cmd is None:
                    m.cmd, m.vel = m.pos, 0.0
                prev = m.cmd
                d = m.target - m.cmd
                sgn = 1.0 if d >= 0 else -1.0
                acc, vmax = P["max_acc"], P["max_speed"]
                if abs(d) > m.vel * m.vel / (2.0 * acc):
                    m.vel += sgn * acc * dt
                else:
                    m.vel -= sgn * acc * dt
                    if sgn * m.vel < 0:
                        m.vel = 0.0
                m.vel = max(-vmax, min(vmax, m.vel))
                step = max(-P["max_step"], min(P["max_step"], m.vel * dt))
                if abs(step) >= abs(d) or abs(d) < 0.02:
                    m.cmd, m.vel = m.target, 0.0
                else:
                    m.cmd += step
                # carrot: 명령은 측정 ± carrot 안. 평소엔 안 걸리는 최후 방어선(런어웨이 차단)
                m.cmd = max(m.pos - P["carrot"], min(m.pos + P["carrot"], m.cmd))
                # 누르기 멈춤: 큰 전류 + 작은(막힘 기준 아래) 오차가 이어짐 → 목표를 지금 자리로(힘 유지). 명령 하나에 한 번
                if (P["press_amps"] > 0 and not m.pressed and m.cur is not None and abs(m.cur) >= P["press_amps"]
                        and abs(m.cmd - m.pos) >= P["press_deg"]):
                    m.press_t = m.press_t or now
                    if now - m.press_t > P["press_s"]:
                        m.note = (f"누르기 멈춤: 전류 {m.cur:+.2f} A · 명령−실제 {m.cmd - m.pos:+.1f}° 가 {P['press_s']:g}s — "
                                  f"목표를 지금 자리 {m.pos:+.1f}° 로 풀어 줌(힘 유지). 바닥·물체 확인")
                        self._log(f"{m.name} ⚠ {m.note}")
                        m.target = m.cmd = m.pos
                        m.vel, m.press_t, m.stall_t, m.pressed = 0.0, None, None, True
                        self._play_abort(m, m.note)
                else:
                    m.press_t = None
                if abs(m.cmd - m.pos) > P["stall_deg"]:
                    m.stall_t = m.stall_t or now
                    if now - m.stall_t > P["stall_s"]:
                        self._fault(m, f"막힘: 명령 {m.cmd:+.1f}° · 실제 {m.pos:+.1f}° 가 {P['stall_s']:g}s 넘게 벌어짐 (물체·바닥을 누름?)")
                        continue
                else:
                    m.stall_t = None
                moving = m.cmd != prev or m.cmd != m.target
                if moving or now - m.sent_t >= hold_dt - 1e-3:
                    send = m.cmd
            elif m.target is not None and m.cmd is not None and m.fb_t is not None and now - m.fb_t <= P["gap_keep_s"]:
                # 짧은 끊김: 마지막 명령 자리를 계속 보냄(램프 멈춤). robot_arm 2026-10-21 실기: 안 보내면 팔이 떨어짐
                m.vel, m.stall_t = 0.0, None
                if not m.stale_logged:
                    m.stale_logged = True
                    self._log(f"{m.name} ⚠ 피드백 끊김 — 마지막 명령 {m.cmd:+.1f}° 홀드 계속 보냄 (최대 {P['gap_keep_s']:g}s)")
                if now - m.sent_t >= hold_dt - 1e-3:
                    send = m.cmd
            else:
                if m.cmd is not None:
                    age = None if m.fb_t is None else now - m.fb_t
                    self._log(f"{m.name} ✖ 피드백 {age if age is not None else 0:.1f}s 끊김 — 홀드 프레임 멈춤(전원 꺼짐?), 원점 지움. "
                              "다시 들어오면 지금 자리에서 홀드")
                    self._unhome(m.name, f"피드백 {P['gap_keep_s']:g}s 넘게 끊김 — 원점 다시 잡기")
                m.cmd, m.vel, m.stall_t = None, 0.0, None
            if send is not None:
                if abs(send) > P["max_command_deg"]:
                    self._fault(m, f"명령 {send:.1f}° 가 상한 ±{P['max_command_deg']:g}° 밖")
                    continue
                if self._send(ak45_pos_frame(m.id, send)):
                    m.sent_t, m.last_sent = now, send
                    m.sent_n += 1
        # 서보
        for n in self.servo_mover.needs_start():
            t = self.servo_mover.target[n]
            self.servo_mover.start(n, t)       # 위치를 몰라 첫 명령은 바로 그 각(arm_control_node 와 같음)
            self._servo_send(self.joints[n], t, now)
        for n, deg in self.servo_mover.step().items():
            self._servo_send(self.joints[n], deg, now)
        rh = P["servo_resend_hz"]
        if rh > 0 and self.servo_sent and now - self.servo_resend_t >= 1.0 / rh:
            self.servo_resend_t = now
            for j in self.cal.joints:
                if j.motor == "servo" and j.name in self.servo_sent:
                    self._send(servo_frame(j.id, self.servo_sent[j.name][0]))

    def _servo_send(self, j, deg, now):
        deg = min(max(deg, 0.0), 180.0)
        if self._send(servo_frame(j.id, deg)):
            self.servo_sent[j.name] = (deg, now)

    # ------------------------------------------------------------------ 명령 (대시보드가 부름)
    def _ready(self):
        if self._closing:
            return "연결 닫는 중"
        return None if self.running else "팔 연결 안 됨 (직접 CAN) — 연결 탭 ② 팔 연결"

    def send_joint_command(self, names, positions, instant=False):
        """sim 관절명, rad. arm_control_node + homing_guard 와 같은 변환: calibration(클립) → 원점 오프셋 → 램프.
        instant = 서보만 램프 없이 바로 그 각 (각도 제어 '서보 한 번에 보내기', 2026-11-25 사용자). AK45 는 늘 램프."""
        return self.joint_command(list(names), [float(p) for p in positions], instant=instant)

    def joint_command(self, names, positions, _source="화면", instant=False):
        why = self._ready()
        if why:
            return {"ok": False, "error": why}
        if len(names) != len(positions):
            return {"ok": False, "error": "name / position 길이가 다름"}
        for n, p in zip(names, positions):
            if n not in self.joints:
                return {"ok": False, "error": f"모르는 관절 {n}"}
            if not math.isfinite(p):
                return {"ok": False, "error": f"{n}: 유한값 아님"}
        now = time.monotonic()
        refused, done, clipped = [], [], []
        with self.lock:
            if self._closing:
                return {"ok": False, "error": "연결 닫는 중"}
            for n, p in zip(names, positions):
                j = self.joints[n]
                mdeg, cl = j.to_motor(math.degrees(p))
                if cl:
                    clipped.append(f"{n} → 모터 {mdeg:.1f}°")
                if j.motor == "servo":
                    self.servo_mover.set_target(n, mdeg)
                    if instant:     # 램프 없이: 지금 값 = 목표, 바로 한 번 보낸다 (그 뒤 재전송은 servo_resend_hz)
                        self.servo_mover.start(n, mdeg)
                        self._servo_send(j, mdeg, now)
                    done.append(n)
                    continue
                m = self.ak[f"ak45_{j.id}"]
                raw = self.homing.to_raw(m.name, mdeg)
                if m.fault:
                    refused.append(f"{n}: {m.fault} (원점을 다시 잡으면 풀림)")
                elif raw is None:
                    refused.append(f"{n}: 원점 없음")
                elif not m.fresh(now, self.p["fresh_s"]):
                    refused.append(f"{n}: 피드백 없음/끊김")
                elif abs(raw) > self.p["max_command_deg"]:
                    refused.append(f"{n}: 모터 {raw:.1f}° 가 ±{self.p['max_command_deg']:g}° 밖")
                else:
                    if m.target is None:
                        self._log(f"{m.name} 첫 명령({_source}): {raw:.2f}° (지금 {m.pos:+.2f}°에서 램프)")
                    if m.target is None or abs(raw - m.target) > 0.05:
                        m.pressed = False      # 새 목표: 누르기 멈춤 다시 허용
                    m.target = raw
                    m.note = None
                    done.append(n)
        if clipped:
            text = "calibration 범위로 잘림: " + ", ".join(clipped)
            if text != getattr(self, "_clip_text", None) or now - getattr(self, "_clip_t", 0.0) > 5.0:
                self._clip_text, self._clip_t = text, now
                self._log(text)
        if refused and not done:
            return {"ok": False, "error": "안 보냄: " + "; ".join(refused)}
        msg = f"{len(done)}개 관절 목표 보냄"
        if refused:
            msg += " · 안 보냄: " + "; ".join(refused)
        return {"ok": True, "msg": msg, "refused": refused}

    def hold_all(self, why="STOP"):
        """STOP: 모든 AK45 지금 측정 자리 홀드(힘 있는 것만), 서보 램프 멈춤, 재생 멈춤. 팔이 안 떨어진다."""
        self._play_stop.set()          # 잠금 밖에서 먼저: 재생 루프를 바로 깨움
        now = time.monotonic()
        held, stale = [], []
        with self.lock:
            self.stop_gen += 1         # 잠금 안에서 한 번 더: _play_begin 의 clear() 가 이 STOP 을 지우지 못하게
            self._play_stop.set()
            for m in self.ak.values():
                if m.target is None:
                    continue
                if m.fresh(now, self.p["fresh_s"]):
                    m.target = m.cmd = m.pos
                    m.vel = 0.0
                    held.append(m.name)
                else:
                    m.target = m.cmd       # 끊긴 모터: 마지막 명령 자리 그대로
                    stale.append(m.name)
            mv = self.servo_mover
            for n in list(mv.target):
                if n in mv.current:
                    mv.target[n] = mv.current[n]
        text = f"{why}: 지금 자리 홀드 {', '.join(held) or '없음 (홀드 중인 AK45 없음)'}" + \
               (f" · 피드백 끊긴 {', '.join(stale)} 는 마지막 명령 자리" if stale else "") + " · 서보 램프 멈춤"
        self._log(text)
        return {"ok": True, "msg": text}

    def limp(self, names=None):
        """힘 빼기: AK45 프레임 중단 + 브레이크 0 A. 팔이 떨어진다. 원점 지움."""
        self._play_stop.set()
        with self.lock:
            self.stop_gen += 1
            self._play_stop.set()
            ms = [self.ak[n] for n in (names or list(self.ak))]
            had = [m.name for m in ms if self._limp_motor(m, "힘 뺌(사용자)", brake=False)]
            for m in ms:
                m.note = None
                self._brake(m)          # 홀드 안 하던 모터에도 (3번, 2 ms 간격, 재시도). 실패면 m.note 에 남음
        text = f"힘 뺌: {', '.join(had) or '홀드 중인 AK45 없었음'} — 브레이크 0 A, 원점 지움. 서보는 마지막 각 유지(펌웨어에 끄기 없음)"
        self._log(text)
        return {"ok": True, "msg": text}

    def stop_all(self):            # 가상 팔과 같은 이름: 서버가 모드를 바꿀 때
        return self.hold_all("모드 바꿈")

    # ------------------------------------------------------------------ 원점 (homing_guard 서비스와 같은 이름)
    def call_trigger(self, service, timeout=2.0):
        prefix = "/arm_control/"
        why = self._ready()
        if why:
            return {"ok": False, "error": why}
        if not service.startswith(prefix):
            return {"ok": False, "error": f"{service}: 모르는 서비스"}
        act, _, joint = service[len(prefix):].partition("_")
        if act == "home" and self.play.get("running"):   # 2026-11-25 검토: 재생 중 원점을 다시 잡으면 남은 프레임이 그만큼 밀려 보정 범위 밖으로 갔다
            return {"ok": False, "error": "재생 중 — 멈춘 뒤 원점"}
        hm = self.homing
        try:
            if joint == "all":
                names = list(hm.m) if act == "release" else [n for n, h in hm.m.items() if not h.homed]
            else:
                names = [hm.by_joint(joint)]
        except KeyError:
            return {"ok": False, "error": f"{service} 없음"}
        if act == "home":
            return self.home(names)
        if act == "release":
            return self.release(names)
        return {"ok": False, "error": f"{service}: 모르는 서비스"}

    def home(self, names):
        now = time.monotonic()
        texts, ok = [], True
        with self.lock:
            for n in names:
                m = self.ak[n]
                if m.temp is not None and (m.temp >= self.p["temp_stop"] or (m.temp_fault and m.temp >= self.p["temp_warn"])):
                    texts.append(f"{m.joint.joint}: 거절, 온도 {m.temp} ℃ (온도로 힘 뺀 관절은 {self.p['temp_warn']:g} ℃ 아래로 식힌 뒤)")
                    ok = False
                    continue
                if m.err:
                    texts.append(f"{m.joint.joint}: 거절, 에러 코드 {m.err} ({AK45_ERRORS.get(m.err, '?')})")
                    ok = False
                    continue
                good, text = self.homing.home(n, now)
                texts.append(text)
                ok = ok and good
                if good:
                    m.fault = m.note = None
                    m.temp_fault = m.pressed = False
                    m.target = m.cmd = m.pos     # 원점 잡는 순간 지금 자리에서 홀드 시작(guard 와 같음)
                    m.vel = 0.0
        msg = "; ".join(texts) or "원점 없는 관절 없음 (전부 잡힘)"
        self._log("원점: " + msg)
        return {"ok": True, "msg": msg} if ok else {"ok": False, "error": msg}

    def release(self, names):
        """쉬게 하기: 원점을 지우고 명령을 막는다. 팔은 지금 자리에서 계속 홀드(힘을 빼려면 '힘 빼기')."""
        now = time.monotonic()
        with self.lock:
            rel = []
            for n in names:
                m = self.ak[n]
                if self._unhome(n, "쉬게 함: 다시 쓰기 전에 원점"):
                    rel.append(m.joint.joint)
                if m.target is not None and m.fresh(now, self.p["fresh_s"]):
                    m.target = m.cmd = m.pos
                    m.vel = 0.0
        text = f"쉬게 함(원점 지움): {', '.join(rel) or '원점 잡힌 관절 없었음'}. 지금 자리 홀드는 계속 — 힘을 빼려면 '힘 빼기'"
        self._log(text)
        return {"ok": True, "msg": text}

    # ------------------------------------------------------------------ 보정 (calibrate.py Session 의 m / z / f / k / v / w)
    def calib(self, action, joint, value=None):
        why = self._ready()
        if why and action != "save":
            return {"ok": False, "error": why}
        if action == "save":
            with self.lock:
                self.cal.save()
                self.calib_dirty = False
            self._log(f"보정 저장: {self.cal.path}")
            return {"ok": True, "msg": f"저장함: {self.cal.path}"}
        if self.play.get("running") and action != "verify":     # 재생 중 부호·영점·핀을 바꾸면 다음 프레임이 다른 곳으로 간다
            return {"ok": False, "error": "재생 중 — 멈춘 뒤 보정"}
        j = self.joints.get(joint)
        if j is None:
            return {"ok": False, "error": f"모르는 관절 {joint!r}"}
        now = time.monotonic()
        with self.lock:
            m = self.ak.get(f"ak45_{j.id}") if j.motor == "ak45" else None
            if action == "move":       # 모터 각을 지금 자리에서 ±value° (원점 없이도: 상대 이동이라)
                try:
                    dv = float(value)
                except (TypeError, ValueError):
                    return {"ok": False, "error": "움직일 각이 숫자가 아님"}
                if not math.isfinite(dv) or abs(dv) > self.p["calib_step_max"]:
                    return {"ok": False, "error": f"한 번에 ±{self.p['calib_step_max']:g}° 까지"}
                if m is not None:
                    if m.fault:
                        return {"ok": False, "error": f"{m.name}: {m.fault} — 원점을 다시 잡아 풀기"}
                    if not m.fresh(now, self.p["fresh_s"]):
                        return {"ok": False, "error": f"{m.name}: 피드백 없음/끊김 — 안 움직임"}
                    # 기준 = 측정 위치(목표가 아님): 막혀서 안 따라오는 관절에 클릭이 쌓여 목표가 ±10° 를 넘어가지 않게.
                    # 결과적으로 |목표 − 측정| ≤ calib_step_max
                    base = m.pos
                    new = base + dv
                    if abs(new) > self.p["max_command_deg"]:
                        return {"ok": False, "error": f"모터 {new:.1f}° 가 ±{self.p['max_command_deg']:g}° 밖"}
                    # 원점을 잡았으면 보정 범위 (zero ± limit) + 한 번 이동폭 까지만 (2026-11-25 검토: 클릭을 쌓으면 어디까지든 갔다 —
                    # calibrate.py move_motor 는 motor_bounds 밖을 거절). 원점 전에는 모터 각의 기준이 없어 상대 이동 제한만.
                    if self.homing.m[m.name].homed:
                        lo, hi = j.motor_bounds()
                        mg = float(self.p["calib_step_max"])
                        if not lo - mg <= new <= hi + mg:
                            return {"ok": False, "error": f"모터 {new:.1f}° 가 보정 범위 {lo:.0f}..{hi:.0f}° (+여유 {mg:g}°) 밖 — 영점·범위를 먼저 고치기"}
                    m.target = new
                    m.pressed = False
                    text = f"보정 {j.joint}({m.name}): 모터 {base:+.2f}° → {new:+.2f}° (지금 {m.pos:+.2f}°)"
                else:
                    cur = self.servo_mover.current.get(j.joint)
                    base = cur if cur is not None else j.zero_deg
                    new = min(max(base + dv, 0.0), 180.0)
                    self.servo_mover.set_target(j.joint, new)
                    text = f"보정 {j.joint}({j.name}): 서보 {base:.1f}° → {new:.1f}°" + ("" if cur is not None else " (첫 명령: 바로 감)")
                self._log(text)
                return {"ok": True, "msg": text}
            if action == "zero":       # 지금 모터 각 = sim 0
                if m is not None:
                    if not m.fresh(now, self.p["fresh_s"]):
                        return {"ok": False, "error": f"{m.name}: 피드백 없음 — 영점 못 잡음"}
                    old, j.zero_deg = j.zero_deg, round(m.pos, 2)
                    h = self.homing.m[m.name]       # 지금 자리가 sim 0 이 됐으니 세션 원점도 여기(오프셋 0)
                    h.homed, h.offset_deg = True, 0.0
                    h.reason = f"보정 영점(z): 모터 {j.zero_deg:+.2f}° = sim 0"
                else:
                    cur = self.servo_sent.get(j.name)
                    if cur is None:
                        return {"ok": False, "error": f"{j.name}: 아직 보낸 값이 없음 — 먼저 움직여서 sim 0 자세로"}
                    old, j.zero_deg = j.zero_deg, round(cur[0], 1)
                text = f"보정 {j.joint}: zero_deg {old:g} → {j.zero_deg:g} (저장 전)"
            elif action == "flip":
                j.sign = -j.sign
                text = f"보정 {j.joint}: sign → {j.sign:+d} (저장 전)"
                if m is not None and m.target is not None and m.fresh(now, self.p["fresh_s"]):
                    m.target = m.cmd = m.pos
            elif action == "scale":
                try:
                    k = float(value)
                except (TypeError, ValueError):
                    return {"ok": False, "error": "scale 이 숫자가 아님"}
                if not (math.isfinite(k) and 0.01 <= k <= 100):
                    return {"ok": False, "error": "scale 0.01~100"}
                j.scale = k
                text = f"보정 {j.joint}: scale → {k:g} (저장 전)"
            elif action == "verify":
                j.verified = bool(value)
                text = f"보정 {j.joint}: verified → {j.verified} (저장 전)"
            elif action == "pin":      # 서보 핀(STM32 채널 id) 바꾸기. 그 핀을 쓰던 관절과 맞바꿈 (2026-11-23 사용자: 실시간으로)
                if j.motor != "servo":
                    return {"ok": False, "error": "서보만 핀을 바꿀 수 있음"}
                try:
                    pin = int(value)
                except (TypeError, ValueError):
                    return {"ok": False, "error": "핀이 숫자가 아님"}
                if not 0 <= pin <= SERVO_PIN_MAX:
                    return {"ok": False, "error": f"핀 0~{SERVO_PIN_MAX}"}
                if pin == j.id:
                    return {"ok": True, "msg": f"{j.joint}: 이미 핀 {pin}"}
                other = next((o for o in self.cal.joints if o.motor == "servo" and o.id == pin and o is not j), None)
                # 위치를 모르는 채널이 되므로 두 관절 모두 '안 보낸 서보'로 (다음 명령은 램프 없이 바로, 재전송도 멈춤)
                for o in (j, other):
                    if o is None:
                        continue
                    self.servo_sent.pop(o.name, None)
                    self.servo_mover.target.pop(o.joint, None)
                    self.servo_mover.current.pop(o.joint, None)
                if other is not None:
                    j.id, other.id = other.id, j.id
                    j.name, other.name = other.name, j.name
                    text = f"서보 핀: {j.joint} ↔ {other.joint} 맞바꿈 → {j.joint} 핀 {j.id}, {other.joint} 핀 {other.id} (저장 전, 둘 다 '안 보냄')"
                else:
                    j.id = pin
                    text = f"서보 핀: {j.joint} → 핀 {pin} (저장 전, '안 보냄')"
            else:
                return {"ok": False, "error": f"모르는 보정 동작 {action}"}
            self.calib_dirty = True
        self._log(text)
        return {"ok": True, "msg": text}

    # ------------------------------------------------------------------ 궤적 재생 (실물)
    def play_trajectory(self, traj, max_speed, hold=5.0, max_error=15.0, stop_gen=None, start_tol=START_TOL_DEG):
        """play.play 로 프레임을 joint_command 에 넣고, TrackingWatch 로 AK45 추종을 본다. 벗어나면 그 자리 홀드.
        check_trajectory 는 서버가 먼저 한다(거절·--force 확인창).
        stop_gen: 호출한 쪽이 계획을 세우기 전에 읽은 self.stop_gen. 그 사이 STOP 이 눌렸으면(값이 바뀜) 재생하지 않는다.
        첫 프레임과 지금 자세가 start_tol° 넘게 다르면 거절 — 첫 프레임 홀드는 관절 직선이라 책상·기둥 검사가 없다."""
        with self.lock:      # 확인과 running 표시를 한 잠금 안에서: 두 요청이 같이 통과해 재생 스레드 둘이 뜨지 않게
            r = self._play_begin(traj, max_speed, hold, max_error, stop_gen, start_tol)
        return r

    def _play_begin(self, traj, max_speed, hold, max_error, stop_gen, start_tol):
        from arm_control import play as P
        why = self._ready()
        if why:
            return {"ok": False, "error": why}
        if self.play.get("running"):
            return {"ok": False, "error": "이미 재생 중"}
        if stop_gen is not None and stop_gen != self.stop_gen:
            return {"ok": False, "error": "계획을 세우는 동안 STOP(또는 힘 빼기·재생 멈춤)이 눌림 — 재생 안 함"}
        if self.calib_dirty:
            return {"ok": False, "error": "보정을 바꾸고 저장 안 함 — 계획은 저장된 calibration.json 으로 세워져 실제 팔과 다르다. "
                    "연결 탭에서 'calibration.json 저장' 뒤 다시"}
        names = traj["joint_names"]
        unknown = [n for n in names if n not in self.joints]
        if unknown:
            return {"ok": False, "error": "모르는 관절: " + ", ".join(unknown)}
        not_homed = [n for n in names if self.joints[n].motor == "ak45"
                     and not self.homing.m[f"ak45_{self.joints[n].id}"].homed]
        if not_homed:
            return {"ok": False, "error": "원점 없는 AK45: " + ", ".join(not_homed) + " — 각도 제어 탭에서 원점 먼저"}
        # 서보는 위치 피드백이 없어 한 번도 안 보낸 서보의 첫 명령은 램프 없이 바로 그 각으로 튄다(_tick). 재생이 그렇게 시작하지 않게
        # 궤적의 서보가 모두 한 번은 보내졌어야(servo_sent) 재생한다.
        unsent = [n for n in names if self.joints[n].motor == "servo" and self.joints[n].name not in self.servo_sent]
        if unsent:
            return {"ok": False, "error": "한 번도 안 보낸 서보: " + ", ".join(unsent) + " — 첫 명령은 램프 없이 바로 튄다. "
                    "각도 제어 탭에서 이 서보들을 지금 자리 근처로 먼저 보낸 뒤 재생"}
        far = self._start_far(names, traj["positions"][0], start_tol)
        if far:
            return {"ok": False, "error": f"지금 자세가 첫 프레임에서 {start_tol:g}° 넘게 떨어짐: " + ", ".join(far) +
                    " — 첫 프레임까지는 검사 없는 관절 직선이라 거절. 'HOME 으로' 버튼으로 원점에 간 뒤(또는 계획을 지금 자세에서 다시) 재생"}
        speed = min(float(max_speed), self.p["max_speed"])
        if not speed > 0:
            return {"ok": False, "error": f"속도 {max_speed} °/s — 0 보다 커야"}
        period, slow, peak = P.schedule(traj, speed)     # play.play 는 안 쓴다(아래 play_loop)
        n = len(traj["positions"])
        motors = [(k, self.joints[nm]) for k, nm in enumerate(names) if self.joints[nm].motor == "ak45"]
        tracker = TrackingWatch(max_error)
        self._play_stop.clear()
        gen0 = self.stop_gen           # 이 재생이 시작할 때의 STOP 번호. 바뀌면 멈춤(아래 step)
        self._play_bad = None
        import play_phase
        self.play = {"running": True, "frame": 0, "frames": n, "period": period, "slow": slow, "hold": hold,
                     "seconds": hold + (n - 1) * period, "started": time.time(), "reason": None, "real": True,
                     "phases": play_phase.phases(traj),   # 지금 무슨 동작 (snapshot 이 play_phase.view 로)
                     "holding": True}   # holding: 첫 자세를 hold 초 유지하는 동안 팔은 안 움직인다 — 화면이 '멈춘 것'으로 오해하지 않게
        state = {"k": 0}

        def publish(nm, positions):
            r = self.joint_command(list(nm), [float(p) for p in positions], _source="궤적")
            if not r.get("ok") or r.get("refused"):
                self._play_bad = self._play_bad or ("안 보낸 관절: " + "; ".join(r.get("refused") or [r.get("error", "?")]))
            state["k"] += 1
            self.play["frame"] = max(0, state["k"] - max(1, int(hold * 10)))

        def watch(frame):
            if self._play_bad:
                return self._play_bad
            now = time.monotonic()
            exp, fb = {}, {}
            with self.lock:
                for k, j in motors:
                    nm = f"ak45_{j.id}"
                    m = self.ak[nm]
                    exp[nm] = j.to_motor(math.degrees(frame[k]))[0]
                    fb[nm] = self.homing.from_raw(nm, m.pos) if m.fresh(now, self.p["fresh_s"]) and m.pos is not None else None
            return tracker.check(exp, fb, now)

        def step(frame):
            """멈춤 확인과 보내기를 한 잠금 안에서. STOP(hold_all·limp·stop_trajectory)은 잠금 안에서 stop_gen 을 올리고
            이벤트를 켠다 — 재생 시작(_play_begin, 잠금 안)의 clear() 와 겹쳐도 gen0 과 달라져 STOP 이 사라지지 않고,
            STOP 뒤에 프레임 하나가 목표를 덮어쓰지도 않는다."""
            with self.lock:
                if self._play_stop.is_set() or self.stop_gen != gen0:
                    return "stopped"
                if self._play_bad:
                    return self._play_bad
                publish(names, frame)
            return None

        def play_loop():
            # arm_control.play.play 와 같은 순서(첫 프레임 hold 초 동안 0.1 s 마다, 그 뒤 period 마다 + watch)인데,
            # 첫 프레임 홀드 동안에도 멈춤·고장을 본다(play.play 의 홀드 루프는 stop 을 안 봐서 STOP 뒤에도 첫 프레임으로 다시 감).
            # 홀드 동안 TrackingWatch 는 안 본다(첫 프레임으로 가는 중이라 오차가 큼 — play.play 와 같음).
            frames = traj["positions"]
            for _ in range(max(1, int(hold * 10))):
                r = step(frames[0])
                if r:
                    return r
                if self._play_stop.wait(0.1):
                    return "stopped"
            self.play["holding"] = False
            for frame in frames[1:]:
                r = step(frame)
                if r:
                    return r
                if self._play_stop.wait(period):
                    return "stopped"
                r = watch(frame)
                if r:
                    return r
            return None

        def run():
            try:
                reason = play_loop()
            except Exception as e:
                reason = f"{type(e).__name__}: {e}"
            if reason:
                # "stopped"(STOP·재생 멈춤 버튼) 포함: 마지막으로 보낸 프레임 쪽으로 계속 가지 않게 지금 자리 홀드
                self.hold_all(f"재생 멈춤({reason})")
            self.play.update(running=False, reason=reason or "끝까지 재생", ended=time.time())
            self._log(f"실물 궤적 재생 끝: {reason or '끝까지 (마지막 프레임 홀드)'}")

        threading.Thread(target=run, daemon=True, name="play").start()
        self._log(f"실물 궤적 재생 시작: {n} 프레임, {slow:.1f}배 느리게({speed:g} °/s), 약 {self.play['seconds']:.0f}초")
        return {"ok": True, "msg": f"실물 재생: {n} 프레임, {slow:.1f}배 느리게, 약 {self.play['seconds']:.0f}초. 멈춤 = STOP"}

    def _start_far(self, names, frame0, tol):
        """첫 프레임(rad)과 지금 자세의 차이가 tol° 넘는 관절 'joint 지금→첫 (°)' 목록. AK45 = 원점 기준 피드백(없거나 끊기면 넘는 것으로),
        서보 = 마지막으로 보낸 각(피드백이 없음). 잠금 안에서 부른다."""
        now = time.monotonic()
        out = []
        for k, n in enumerate(names):
            j = self.joints[n]
            want = math.degrees(frame0[k])
            if j.motor == "ak45":
                m = self.ak[f"ak45_{j.id}"]
                raw = self.homing.from_raw(m.name, m.pos) if m.pos is not None and m.fresh(now, self.p["fresh_s"]) else None
                if raw is None:
                    out.append(f"{n} 피드백 없음")
                    continue
                cur = j.to_sim(raw)
            else:
                sent = self.servo_sent.get(j.name)
                if sent is None:
                    continue          # 한 번도 안 보낸 서보는 위(unsent)에서 이미 거절
                cur = j.to_sim(sent[0])
            if abs(cur - want) > tol:
                out.append(f"{n} {cur:.0f}→{want:.0f}°")
        return out

    def stop_trajectory(self):
        self._play_stop.set()
        with self.lock:
            self.stop_gen += 1
            self._play_stop.set()
        return {"ok": True, "msg": "재생 멈춤 요청 (팔은 그 자리 홀드)"}

    # ------------------------------------------------------------------ 상태 (RosMonitor.snapshot 과 같은 모양)
    def snapshot(self):
        try:
            self._check_threads()      # 죽은 스레드 다시 띄우기(제어 스레드가 죽은 경우는 여기서만 잡힘)
        except Exception:
            pass
        now_m, now = time.monotonic(), time.time()
        P = self.p
        with self.lock:
            motors, joints, homed_fb = [], {}, {}
            for n, m in self.ak.items():
                age = None if m.fb_t is None else now_m - m.fb_t
                fresh = m.fresh(now_m, P["fresh_s"])
                if m.fault:
                    lv, msg = 2, f"힘 뺌: {m.fault}"
                elif m.fb_t is None:
                    lv, msg = 1, "피드백 수신 없음 (전원·배선·CubeMarsTool 'Send status over CAN' Rate 확인)"
                elif not fresh:
                    lv, msg = 2, f"피드백 끊김 {age:.1f}s" + (" — 마지막 자리 홀드 계속" if m.cmd is not None else "")
                elif m.err:
                    lv, msg = 2, f"에러 코드 {m.err} ({AK45_ERRORS.get(m.err, '?')})"
                elif m.note:
                    lv, msg = 1, m.note
                elif m.hot:
                    lv, msg = 1, f"온도 높음 {m.temp} ℃ (경고는 {P['temp_warn'] - P['temp_hyst']:g} ℃ 아래로 식으면 풀림)"
                else:
                    lv, msg = 0, "정상"
                vals = {"position_deg": "—" if m.pos is None else f"{m.pos:.2f}",
                        "speed_erpm": "—" if m.spd is None else f"{m.spd:.0f}",
                        "current_a": "—" if m.cur is None else f"{m.cur:.2f}",
                        "temperature_c": "—" if m.temp is None else str(m.temp),
                        "error_code": str(m.err), "watchdog_ok": "1" if fresh else "0",
                        "target_deg": "none" if m.target is None else f"{m.target:.2f}",
                        "command_deg": "none" if m.cmd is None else f"{m.cmd:.2f}",
                        "command_active": "1" if m.cmd is not None else "0",
                        "position_error_deg": "none" if m.cmd is None or m.pos is None else f"{m.cmd - m.pos:.2f}",
                        "frames_sent": str(m.sent_n), "feedback_count": str(m.fb_n)}
                motors.append({"name": f"ak45/{n}", "level": lv, "message": msg, "hardware_id": f"ak45_id_0x{m.id:02X}",
                               "values": vals, "t": now, "age": 0.0, "joint_deg": m.pos if fresh else None,
                               "joint_age": age, "joint": m.joint.joint, "fault": m.fault})
                if m.fb_t is not None:
                    joints[n] = {"deg": m.pos, "age": age}
                    d = self.homing.from_raw(n, m.pos)
                    if d is not None:
                        homed_fb[n] = {"deg": d, "age": age}
            b = dict(self.busst)
            blv = 0 if b["state"] == "ERROR-ACTIVE" else (2 if b["state"] == "BUS-OFF" else 1)
            recent_send_err = b["send_error_t"] is not None and now_m - b["send_error_t"] < 3.0
            recent_rx_err = b["rx_error_t"] is not None and now_m - b["rx_error_t"] < 5.0
            alive = {name: (i < len(self.threads) and self.threads[i].is_alive())
                     for i, name in enumerate(("rx", "ctrl"))}
            dead = [n for n, a in alive.items() if not a] if self.running else []
            problems = []
            if dead:
                problems.append("스레드 죽음: " + ", ".join({"rx": "수신(rx)", "ctrl": "제어"}[n] for n in dead)
                                + " — 피드백·홀드가 멈춤, 팔 받치기")
            if b["send_fail_streak"] >= 10:
                problems.append(f"송신 연속 실패 {b['send_fail_streak']}개: {b['last_send_error']}")
            elif recent_send_err:
                problems.append(f"송신 실패: {b['last_send_error']} (누적 {b['send_errors']})")
            if recent_rx_err:
                problems.append(f"수신 오류: {b['last_rx_error']} (누적 {b['rx_errors']})")
            if problems:
                blv = 2
            bus = {"name": "ak45/can_bus", "level": blv, "hardware_id": f"{self.channel} ({self.interface})",
                   "message": (" · ".join(problems) if problems else "정상" if blv == 0 else f"{b['state']}"),
                   "values": {k: str(v) for k, v in b.items() if k not in ("send_error_t", "rx_error_t")},
                   "threads": alive, "t": now, "age": 0.0}
            hs = self.homing.status(now_m)
            hj = [dict(j, joint=jn) for jn, j in hs["joints"].items()]
            for row in hj:
                m = self.ak[row["motor"]]
                row["command_active"] = m.cmd is not None
                row["fault"] = m.fault
            servos = {n: {"deg": d, "age": now_m - t} for n, (d, t) in self.servo_sent.items()}
            foreign = {k: {"count": c, "age": now_m - t} for k, (c, t) in self.foreign.items() if now_m - t < 30}
            cal_rows = [{"joint": j.joint, "motor": j.motor, "id": j.id, "name": j.name, "sign": j.sign, "scale": j.scale,
                         "zero_deg": j.zero_deg, "verified": j.verified} for j in self.cal.joints]
            return {
                "available": self.running, "reason": "" if self.running else "팔 연결 안 됨 (직접 CAN) — 연결 탭 ② 팔 연결",
                "virtual": False, "direct": True,
                "motors": motors, "bus": bus,
                "diag_age": 0.0 if self.running else None,
                "joint_age": min((now_m - m.fb_t for m in self.ak.values() if m.fb_t), default=None),
                "homing": {"available": True, "seen": self.running, "all_homed": hs["all_homed"], "age": 0.001,   # 0.0 이면 server.cmd_gate 의 (age or 99) 가 99 가 됨
                           "stamp": now, "joints": hj},
                "joints": joints, "joints_homed": homed_fb,
                "servos": servos,
                "servo_age": (now_m - max(t for _, t in self.servo_sent.values())) if self.servo_sent else None,
                "object_pose": None,
                "cmd_subs": 1 if self.running else 0,
                "conn": {"state": "direct" if self.running else "off", "reason": "직접 CAN", "pid": os.getpid()},
                "direct_state": {
                    "connected": self.running, "interface": self.interface, "channel": self.channel,
                    "lock": self.lock_path if self.lock_fd is not None else None, "started": self.started,
                    "params": dict(P), "playback": _play_view(self.play), "foreign": foreign,
                    "calib_dirty": self.calib_dirty, "calibration": cal_rows, "calibration_file": str(self.cal.path),
                    "servo_targets": dict(self.servo_mover.target),
                    "log": list(self.lines)[-40:],
                },
            }


class Disconnected:
    """팔 연결 전(직접 CAN 모드) 서버가 쓰는 빈 백엔드."""

    def __init__(self, reason="팔 연결 안 됨 (직접 CAN) — 연결 탭 ② 팔 연결", lines=()):
        self.reason, self.lines = reason, list(lines)
        self.play = {"running": False}

    def _no(self, *a, **k):
        return {"ok": False, "error": self.reason}

    send_joint_command = call_trigger = play_trajectory = calib = limp = _no

    def hold_all(self, *a, **k):
        return {"ok": True, "msg": "팔 연결 안 됨 — 홀드할 것 없음"}

    stop_trajectory = stop_all = hold_all

    def snapshot(self):
        return {"available": False, "reason": self.reason, "virtual": False, "direct": True, "motors": [], "bus": None,
                "diag_age": None, "joint_age": None, "homing": {"available": True, "seen": False, "joints": []},
                "joints": {}, "joints_homed": {}, "servos": {}, "servo_age": None, "object_pose": None, "cmd_subs": 0,
                "conn": {"state": "off", "reason": self.reason, "pid": None},
                "direct_state": {"connected": False, "log": self.lines[-40:], "playback": {"running": False},
                                 "params": dict(DEFAULTS), "foreign": {}}}
