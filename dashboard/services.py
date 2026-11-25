"""대시보드가 켜고 끄는 항목 목록. 새 항목은 SERVICES 에 Service(...) 하나를 더하면 된다.

Service 필드
  id        영문 짧은 이름(로그 파일 이름 dashboard/logs/<id>.log)
  label     화면 이름
  kind      "proc"  계속 도는 노드(켜기/끄기/재시작)
            "once"  한 번 돌고 끝나는 명령(실행 버튼, 끝나면 종료코드 표시)
  build     params(dict) -> 문자열(bash 본문, ROS 환경을 source 한 뒤 실행) 또는 list(argv, 그대로 실행)
            값이 틀리면 ValueError("이유") 를 던진다(화면에 그대로 뜸)
  ros       True 면 /opt/ros/<distro>/setup.bash 와 <ws>/install/setup.bash 를 먼저 source
  params    화면 입력칸 [Param(key, label, default, hint)]
  conflicts 동시에 켜면 안 되는 항목 id(켜기를 거절)
  procs     이 항목이 띄우는 실행 파일 이름(대시보드 밖에서 이미 떠 있는지 pgrep 으로 검사)
  stop_warn 끌 때 확인창 문구(비우면 확인 없이 끔)
  linux     True 면 Linux 에서만(Windows 에서는 '이 PC에서 못 씀'). 가상 모드에서는 전부 가상 노드로 켜진다
            build 의 p["_local"] 이 True 면(가상 모드 또는 Linux 아님) ROS 없이 이 PC 의 python 으로 돌리는 argv
  demo      True 면 JAX_DASH_DEMO=1 이거나 Linux 가 아닐 때만 보임(시험용 가짜 항목)
  tab       화면 탭: conn(연결) | ik(역기구학) (그 탭에 카드가 그려진다)
  tail      화면에 보이는 로그 줄 수
"""
import json
import math
import os
import re
import shlex
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, List, Union

REPO = Path(__file__).resolve().parent.parent
LOG_DIR = Path(os.environ.get("JAX_DASH_LOGS") or Path(__file__).resolve().parent / "logs")
PICK_TRAJ = LOG_DIR / "pick_dry_traj.json"   # pick_place --save: 마지막 dry-run 의 궤적(관절각 시계열)


def _default_ws():
    # README: ~/ros2_ws/src/<이 레포>. 레포가 어떤 src 폴더 안에 있으면 그 워크스페이스를 쓴다.
    if REPO.parent.name == "src":
        return str(REPO.parent.parent)
    return os.path.expanduser("~/ros2_ws")


WS = os.environ.get("JAX_WS") or _default_ws()
ROS_DISTRO = os.environ.get("JAX_ROS_DISTRO") or os.environ.get("ROS_DISTRO") or "jazzy"
CAN_IFACE = "can0"   # ak45_node 의 CAN 이름은 컴파일 타임 상수 can0 (README)
CAN_UP_CMD = f"sudo ip link set {CAN_IFACE} txqueuelen 1000 up type can bitrate 1000000"   # txqueuelen 1000: 기본 10 이면 한 제어 주기의 프레임 묶음에 송신 큐가 차 ENOBUFS 로 프레임이 빠졌다 (2026-11-25, 로그 1156번)


@dataclass
class Param:
    key: str
    label: str
    default: str = ""
    hint: str = ""


@dataclass
class Service:
    id: str
    label: str
    build: Callable[[dict], Union[str, List[str]]]
    kind: str = "proc"
    ros: bool = True
    group: str = "노드"
    help: str = ""
    params: List[Param] = field(default_factory=list)
    conflicts: tuple = ()
    procs: tuple = ()
    stop_warn: str = ""
    linux: bool = True
    demo: bool = False
    tab: str = "conn"
    tail: int = 30


# ---------- 값 검사 ----------
_NUM = r"\s*-?\d+(?:\.\d+)?\s*"


def _num(v, name, lo=None, hi=None):
    try:
        x = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: 숫자가 아님 ({v!r})")
    if (lo is not None and x < lo) or (hi is not None and x > hi):
        raise ValueError(f"{name}: {lo}~{hi} 범위 밖 ({x})")
    return x


def _iface(v, name):
    v = (v or "").strip()
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", v):
        raise ValueError(f"{name}: 인터페이스 이름이 이상함 ({v!r})")
    return v


# ---------- 명령 ----------
AK45_STOP_WARN = ("ak45_node 를 끄면 AK45 에 브레이크 0A 가 가서 홀드가 풀린다. 팔이 떨어질 수 있다.\n"
                  "팔을 받쳤나요?")


def _can_up(p):
    # 이미 UP 이면 아무것도 안 함. sudo -n(암호 없이 되는 경우) → pkexec(로봇 PC 화면에 암호창) 순서로 시도.
    up = f"ip link set {CAN_IFACE} up type can bitrate 1000000"
    return f"""
if ip link show {CAN_IFACE} 2>/dev/null | grep -qE '<([^>]*,)?UP(,[^>]*)?>'; then
  echo "{CAN_IFACE} 이미 UP"; ip -details -statistics link show {CAN_IFACE}; exit 0
fi
ip link show {CAN_IFACE} >/dev/null 2>&1 || {{ echo "{CAN_IFACE} 없음 (USB-CAN 연결 확인)"; exit 4; }}
if sudo -n {up} 2>/dev/null; then echo "sudo -n 으로 올림"
elif command -v pkexec >/dev/null && timeout 60 pkexec {up}; then echo "pkexec 으로 올림"
else
  echo "권한이 없어 못 올림. 터미널에서 직접:"; echo "  {CAN_UP_CMD}"; exit 5
fi
ip -details link show {CAN_IFACE}
"""


def _arm_launch(p):
    args = ["ros2", "launch", "arm_control", "arm.launch.py",
            f"servo_channel:={_iface(p.get('servo_channel'), 'servo_channel')}",
            f"max_speed_deg_s:={_num(p.get('max_speed_deg_s'), 'max_speed_deg_s', 0.1, 360)}"]
    cal = (p.get("calibration_file") or "").strip()
    if cal:
        args.append(f"calibration_file:={cal}")
    return "exec " + " ".join(shlex.quote(a) for a in args)


def _arm_homed(p):
    args = ["ros2", "launch", "arm_control", "arm_homed.launch.py",
            f"servo_channel:={_iface(p.get('servo_channel'), 'servo_channel')}",
            f"max_speed_deg_s:={_num(p.get('max_speed_deg_s'), 'max_speed_deg_s', 0.1, 360)}"]
    for k in ("calibration_file", "home_file"):
        v = (p.get(k) or "").strip()
        if v:
            args.append(f"{k}:={v}")
    return "exec " + " ".join(shlex.quote(a) for a in args)


def _stm_bridge(p):
    ch = _iface(p.get("servo_channel"), "servo_channel")
    return f"exec ros2 run stm_bridge stm_bridge_node --ros-args -p servo_channel:={ch}"


def _arm_control(p):
    a = ["ros2", "run", "arm_control", "arm_control_node", "--ros-args",
         "-p", f"max_speed_deg_s:={_num(p.get('max_speed_deg_s'), 'max_speed_deg_s', 0.1, 360)}"]
    cal = (p.get("calibration_file") or "").strip()
    if cal:
        a += ["-p", f"calibration_file:={cal}"]
    return "exec " + " ".join(shlex.quote(x) for x in a)


def _vision(p):
    fp = (p.get("fixed_pose") or "").strip()
    rj = (p.get("result_json") or "").strip()
    if bool(fp) == bool(rj):
        raise ValueError("fixed_pose 와 result_json 중 하나만 채운다")
    a = ["ros2", "run", "vision_node", "vision_node", "--ros-args"]
    if fp:
        if not re.fullmatch(rf"\[{_NUM},{_NUM},({_NUM},)?{_NUM}\]", fp):
            raise ValueError("fixed_pose 형식: [x_m, y_m, z_m, yaw_deg] 예 [0.10, -0.30, 0.05, 20.0] (z 빼면 0)")
        a += ["-p", f"fixed_pose:={fp}"]
    else:
        a += ["-p", f"result_json:={rj}"]
    return "exec " + " ".join(shlex.quote(x) for x in a)


def _stack_args(p, cx, cy, tx, ty):
    """탑 쌓기 탭이 주는 stack 칸 (JSON: {"for": [cube x, y, target x, y], "level": K, "target_yaw": °, "obstacles": [[x, y, yaw, 층], …]})
    → pick_plan.py --place-level/--target-yaw/--obstacle. 칸 값은 실행 사이에 남으므로, "for" 가 이번 큐브·놓을 자리와 같을 때만 쓴다
    (역기구학 탭·스크립트의 보통 계획이 지난 탑 쌓기 값으로 돌지 않게). 팀 시뮬 계획엔 이 옵션이 없다."""
    raw = (p.get("stack") or "").strip()
    if not raw or str(p.get("team_sim") or "").strip().lower() in ("1", "true", "on", "yes"):
        return []
    try:
        d = json.loads(raw)
        f = [float(v) for v in d["for"]]
        if len(f) != 4 or max(abs(u - v) for u, v in zip(f, (cx, cy, tx, ty))) > 5e-4:
            return []
        lv = int(d.get("level") or 0)
        obst = [[float(v) for v in o] for o in (d.get("obstacles") or [])]
        if not 0 <= lv <= 8 or len(obst) > 24 or any(len(o) != 4 or not all(math.isfinite(v) for v in o) or max(abs(o[0]), abs(o[1])) > 1.0
                                                     or not 0 <= o[3] <= 8 or o[3] != int(o[3]) for o in obst):
            raise ValueError
        out = ["--place-level", str(lv)]
        if d.get("target_yaw") is not None:
            out += ["--target-yaw", f"{_num(d['target_yaw'], 'target yaw', -360, 360):g}"]
        for o in obst:
            out += ["--obstacle", f"{o[0]:g}", f"{o[1]:g}", f"{o[2]:g}", str(int(o[3]))]
        return out
    except (KeyError, TypeError, ValueError):
        raise ValueError("stack: 탑 쌓기 값이 틀림 (탑 쌓기 탭에서 다시)")


def _pick_args(p):
    cx = _num(p.get("cube_x"), "cube x", -1.0, 1.0)
    cy = _num(p.get("cube_y"), "cube y", -1.0, 1.0)
    cyaw = _num(p.get("cube_yaw"), "cube yaw", -360, 360)
    tx = _num(p.get("target_x"), "target x", -1.0, 1.0)
    ty = _num(p.get("target_y"), "target y", -1.0, 1.0)
    ms = _num(p.get("max_speed"), "max_speed", 0.1, 360)
    a = ["--cube", f"{cx:g}", f"{cy:g}", f"{cyaw:g}", "--target", f"{tx:g}", f"{ty:g}", "--max-speed", f"{ms:g}"]
    d = (p.get("d") or "").strip()
    if d:
        a += ["--d", f"{_num(d, 'd', 0.0, 2.0):g}"]
    g = (p.get("grasp") or "").strip()   # 집는 방향 (대시보드 계획만 — 팀 시뮬 계획은 이 옵션이 없다)
    if g and g != "auto" and str(p.get("team_sim") or "").strip().lower() not in ("1", "true", "on", "yes"):
        if g not in ("vertical", "horizontal", "tilted"):
            raise ValueError(f"집는 방향: auto / vertical / horizontal / tilted ({g!r})")
        a += ["--grasp", g]
    if str(p.get("team_sim") or "").strip().lower() not in ("1", "true", "on", "yes"):
        import procman   # 순환 import 피하려고 여기서
        if procman.MODE.get("virtual"):   # 집기 8 cm 낮춤(pick_plan.PICK_DROP_M)은 실물 보정 — 가상 팔·3D 에선 손이 책상에 파묻히지 않게 0 (2026-11-25 검토)
            a += ["--pick-drop", "0"]
    a += _stack_args(p, cx, cy, tx, ty)
    ik = (p.get("ik") or "").strip()
    if ik:
        if not re.fullmatch(r"[A-Za-z0-9_./~:\\-]{1,200}", ik):
            raise ValueError(f"ik: 경로에 이상한 글자 ({ik!r})")
        a += ["--ik", os.path.expanduser(ik)]
    # --save: pick_place 가 계획한 궤적을 JSON 으로 쓴다(기존 옵션). 3D 뷰가 /api/ik/trajectory 로 읽는다.
    return a + ["--save", str(PICK_TRAJ), "--dry-run"]


# 로컬 실행(가상 모드 / Windows): ros2 run 대신 이 PC 의 python 으로 같은 pick_place.main 을 부른다.
# arm_control·stm_bridge 를 sys.path 에 넣는 것만 다르다(colcon 없이). IK 계획에는 mujoco·gymnasium·scipy 가 필요하다
# (없으면 출력에 ImportError). JAX_DASH_PYTHONPATH 에 경로를 주면 sys.path 뒤에 붙인다.
_LOCAL_PICK = "\n".join([
    "import os, sys",
    "sys.path[:0] = [{arm!r}, {stm!r}]",
    "sys.path += [x for x in os.environ.get('JAX_DASH_PYTHONPATH', '').split(os.pathsep) if x]",
    "from arm_control.pick_place import main",
    "main(sys.argv[1:])", ""])


def _pick_dry(p):
    # 계획 + 검사만(--dry-run 고정). 모터를 움직이는 실행(--dry-run 없이, --force)은 대시보드에 없다.
    # 기본 = 대시보드 계획(dashboard/pick_plan.py: '집을 수 있는 곳' 지도와 같은 해석 IK, 물리 없음, 2026-11-20).
    # team_sim 이 켜지면 팀 계획(pick_place: team-jax/IK motion1 MuJoCo 물리 시뮬).
    a = _pick_args(p)
    try:
        PICK_TRAJ.unlink()   # 이번 실행이 못 쓰면(팔이 못 닿음 등) 옛 궤적이 남아 보이지 않게
    except OSError:
        pass
    if str(p.get("team_sim") or "").strip().lower() not in ("1", "true", "on", "yes"):
        return [sys.executable, "-u", str(REPO / "dashboard" / "pick_plan.py")] + a
    if p.get("_local"):
        code = _LOCAL_PICK.format(arm=str(REPO / "arm_control"), stm=str(REPO / "stm_bridge"))
        return [sys.executable, "-u", "-c", code] + a
    return "exec " + " ".join(shlex.quote(x) for x in ["ros2", "run", "arm_control", "pick_place"] + a)


def _vcan(p):
    return f"exec bash {shlex.quote(str(REPO / 'arm_control' / 'tests' / 'launch_vcan.sh'))}"


PY = sys.executable

SERVICES = [
    Service("can_up", "can0 올리기", _can_up, kind="once", ros=False, group="준비", tab="conn",
            help=f"재부팅마다 1번. sudo 권한이 필요하다. 안 되면 터미널에서: {CAN_UP_CMD}"),
    Service("arm_launch", "arm.launch.py (3개 한 번에)", _arm_launch, group="노드",
            help="ak45_node + stm_bridge_node + arm_control_node. 보정(calibrate --ros) 중에는 켜지 않는다.",
            params=[Param("servo_channel", "servo_channel", "can0"),
                    Param("max_speed_deg_s", "max_speed_deg_s", "30.0", "°/s"),
                    Param("calibration_file", "calibration_file", "", "비우면 패키지 기본")],
            conflicts=("ak45", "stm_bridge", "arm_control", "arm_homed"),
            procs=("ak45_node", "stm_bridge_node", "arm_control_node"),
            stop_warn=AK45_STOP_WARN),
    Service("arm_homed", "arm_homed.launch.py (원점 guard 포함)", _arm_homed, group="노드",
            help="arm.launch.py 와 같고 homing_guard 가 arm_control_node 와 ak45_node 사이에 낀다. "
                 "원점을 잡기 전·전원 재투입 뒤에는 AK45 명령이 막힌다(각도 제어 탭 '원점' 칸).",
            params=[Param("servo_channel", "servo_channel", "can0"),
                    Param("max_speed_deg_s", "max_speed_deg_s", "30.0", "°/s"),
                    Param("calibration_file", "calibration_file", "", "비우면 패키지 기본"),
                    Param("home_file", "home_file", "", "비우면 패키지 homing.json")],
            conflicts=("arm_launch", "ak45", "stm_bridge", "arm_control"),
            procs=("ak45_node", "stm_bridge_node", "arm_control_node", "homing_guard"),
            stop_warn=AK45_STOP_WARN),
    Service("ak45", "ak45_node", lambda p: "exec ros2 launch ak45_ros2 ak45_node.launch.py", group="노드",
            help="AK45 4개(ID 1~4). /joint_states, /diagnostics 발행, /ak45/command 수신.",
            conflicts=("arm_launch", "arm_homed"), procs=("ak45_node",), stop_warn=AK45_STOP_WARN),
    Service("stm_bridge", "stm_bridge_node", _stm_bridge, group="노드",
            help="/servo/command → STM32 서보 CAN 프레임(StdId 5).",
            params=[Param("servo_channel", "servo_channel", "can0")],
            conflicts=("arm_launch", "arm_homed"), procs=("stm_bridge_node",)),
    Service("arm_control", "arm_control_node", _arm_control, group="노드",
            help="/arm_control/joint_command → /ak45/command, /servo/command. calibrate --ros 와 같이 켜지 않는다.",
            params=[Param("max_speed_deg_s", "max_speed_deg_s", "30.0", "°/s"),
                    Param("calibration_file", "calibration_file", "", "비우면 패키지 기본")],
            conflicts=("arm_launch", "arm_homed"), procs=("arm_control_node",)),
    Service("vision", "vision_node", _vision, group="노드",
            help="둘 중 하나만: fixed_pose(시험용) 또는 result_json(카메라 inference.json 경로).",
            params=[Param("fixed_pose", "fixed_pose", "[0.10, -0.30, 0.0, 20.0]", "[x m, y m, z m, yaw °]"),
                    Param("result_json", "result_json", "", "…/inference.json")],
            procs=("vision_node",)),
    Service("pick_dry", "pick_place --dry-run (계획·검사만)", _pick_dry, kind="once", group="역기구학", tab="ik", tail=200, linux=False,
            help="기본: 대시보드 계획(pick_plan.py) — '집을 수 있는 곳' 지도와 같은 해석 IK 로 경유점(큐브 위 → 잡기 → 들기 → "
                 "놓을 자리 위 → 놓기 → HOME)을 풀고 관절 보간 (물리·쥐기 시뮬 없음). '팀 시뮬 계획 (물리)' 를 켜면 "
                 "sim 에서 배달을 계획(planner.py, team-jax/IK motion1). 둘 다 검사(play.check_trajectory)만 한다. "
                 "--dry-run 고정: 모터에 아무것도 안 보낸다. 종료코드 0 통과, 1 거절, 3 팔이 못 닿음. "
                 "계획한 궤적은 --save 로 logs/pick_dry_traj.json 에 써서 오른쪽 3D 뷰에서 재생한다.",
            params=[Param("cube_x", "cube x", "-0.05", "m, +x = 로봇 왼쪽"),
                    Param("cube_y", "cube y", "-0.33", "m, 앞 = -y"),
                    Param("cube_yaw", "cube yaw", "20", "°, 위에서 반시계"),
                    Param("target_x", "target x", "0.0", "m"),
                    Param("target_y", "target y", "-0.33", "m"),
                    Param("max_speed", "max_speed", "30.0", "°/s, arm_control_node 의 max_speed_deg_s 이하"),
                    Param("d", "책상 높이 d (선택)", "", "어깨 축에서 책상 면까지 아래로 m, 비우면 실물 기본 0.5575 (2026-11-25 프로파일 8 cm 내림) · 0.44~0.60"),
                    Param("grasp", "집는 방향", "auto", "auto 자동(수직 → 수평 → 기울임) / vertical 수직(위에서) / horizontal 수평(옆에서 밀어 넣기)"),
                    Param("stack", "탑 쌓기 (탑 쌓기 탭이 채움)", "", "JSON: 놓는 층·다른 큐브 — 이번 큐브·놓을 자리와 같을 때만 쓴다"),
                    Param("ik", "ik (선택)", "", "team-jax/IK 경로, 비우면 $JAX_IK_DIR·~/IK"),
                    Param("team_sim", "팀 시뮬 계획 (물리)", "", "1 이면 팀 계획 (pick_place, MuJoCo 물리), 비우면 대시보드 해석 IK 계획")]),
    Service("vcan_test", "vcan 시험 (launch_vcan.sh)", _vcan, kind="once", group="시험",
            help="실물 없이 arm.launch.py 전체 시험. 별도 네트워크 공간의 가짜 can0 라 실제 버스·모터와 상관없다. 약 30초.",
            ),
    # ---- 시험용 가짜 항목 (Windows 또는 JAX_DASH_DEMO=1) ----
    Service("demo_counter", "가짜: 숫자 세기", lambda p: [PY, "-u", "-c",
            "import time\nfor i in range(100): print(i, flush=True); time.sleep(1)"],
            ros=False, linux=False, demo=True, group="시험(가짜)", help="켜기·로그·끄기 시험용."),
    Service("demo_stubborn", "가짜: SIGINT 무시", lambda p: [PY, "-u", "-c",
            "import signal,time\nfor s in ('SIGINT','SIGBREAK'):\n"
            "    getattr(signal,s,None) and signal.signal(getattr(signal,s), signal.SIG_IGN)\n"
            "i=0\nwhile True: print('stubborn', i, flush=True); i+=1; time.sleep(1)"],
            ros=False, linux=False, demo=True, group="시험(가짜)",
            help="ak45_node 가 SIGINT 로 안 끝나는 경우 흉내. 끄면 SIGTERM/SIGKILL 단계가 보여야 한다.",
            stop_warn="가짜 항목 확인창 시험"),
]

# 보정은 키보드 대화형(z/f/v/w)이라 웹에서 돌리지 않는다. 화면에는 명령 안내만.
CALIBRATE_GUIDE = {
    "needs": ["ak45", "stm_bridge"],
    "forbid": ["arm_launch", "arm_homed", "arm_control"],
    "commands": [
        f"source /opt/ros/{ROS_DISTRO}/setup.bash && source {WS}/install/setup.bash",
        "ros2 run arm_control calibrate --ros",
    ],
    "note": "ak45_node 와 stm_bridge_node 만 켜고(아래 버튼), arm_control_node·arm.launch.py·arm_homed.launch.py 는 끈 상태에서 터미널로 실행. "
            "키: z 영점 / f 방향 / v 확인 / w 저장.",
}

# 대시보드 밖(터미널)에서 떠 있는지 볼 실행 파일 이름
WATCH_PROCS = ("ak45_node", "stm_bridge_node", "arm_control_node", "vision_node", "calibrate", "homing_guard",
               "pick_place", "play_trajectory")


def visible_services():
    demo = os.environ.get("JAX_DASH_DEMO") == "1" or not sys.platform.startswith("linux")
    return [s for s in SERVICES if demo or not s.demo]
