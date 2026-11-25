"""강화학습 돌리기 페이지(/rl)의 서버 쪽. 역기구학 집기(pick_plan·hand_ik)와는 따로다 — 서로 안 부른다.

team-jax/IK 의 sim2sim 체인(정책 grasp_lift_23025 + 조회표)을 MuJoCo 로 한 번 돌려 궤적을 돌려준다 (rl_worker.py 를 서브프로세스로).
시뮬 전용: 실물 팔·CAN·ROS 를 건드리지 않는다. 서브프로세스라 서버가 MuJoCo 상태를 안 쥐고, 죽거나 오래 걸려도 서버는 산다.
"""
import importlib.util
import json
import math
import os
import secrets
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORKER = HERE / "rl_worker.py"
MARK = "@@RL@@"
TIMEOUT_S = 120
LIMITS = {"x_cm": (-60.0, 60.0), "y_cm": (-80.0, 80.0), "yaw_deg": (-180.0, 180.0), "mass_g": (20.0, 100.0),
          "target_x_cm": (-60.0, 60.0), "target_y_cm": (-80.0, 80.0), "insert_push_cm": (0.0, 5.0), "insert_side_cm": (-5.0, 5.0),
          "grip_force_x": (1.0, 6.0)}     # 2026-11-25: 이 줄이 없어 grip_force_x 를 넘기면 KeyError (rl_worker.GRIP_FORCE_RANGE 와 같게)
_lock = threading.Lock()     # 한 번에 하나만 (CPU 를 먹는다, 폰·PC 가 같이 눌러도 섞이지 않게)
_info_cache = {}
REF_X, REF_Y = -0.048, -0.285    # 기준 큐브 (위치 오프셋 0, 0) 의 로봇 틀 위치 m — rl.js·README 와 같은 값
TABLE_TOP_Z = -0.4775            # 로봇 틀에서 책상 윗면 (어깨 아래 47.75 cm). 큐브 중심 = +2.5 cm
RUNS_KEEP = 5
INSERT_SIDE_CM = -2.0            # 2026-11-25 사용자 '밀어 넣기 할 때 너무 왼쪽': 같은 구간에 로봇 옆으로 (cm, + 왼쪽 / − 오른쪽, −5~5)
INSERT_PUSH_CM = 2.0             # 2026-11-25 사용자 'INSERT 작업중 앞으로 더 땡겨': INSERT~들기 동안 쥐는 점을 로봇 앞(−y)으로 이만큼 더 (cm, 0~5)
_runs = {}                       # run_id -> 결과 (실물 재생이 '화면에 보인 그 궤적'만 보내게)


def _num(b, key, default):
    try:
        v = float(b.get(key, default))
    except (TypeError, ValueError):
        raise ValueError(f"{key} 가 숫자가 아님")
    lo, hi = LIMITS[key]
    if not math.isfinite(v) or not lo <= v <= hi:
        raise ValueError(f"{key} 는 {lo:g} ~ {hi:g} 사이여야 함 (받은 값 {b.get(key)!r})")
    return v


def _call(ik_dir, mode, arg=None):
    if ik_dir is None or not (Path(ik_dir) / "sim2sim" / "chain.py").is_file():
        return {"ok": False, "error": "team-jax/IK 의 sim2sim 을 못 찾음 (JAX_IK_DIR 또는 레포 옆 ../IK, sim-table-friction 브랜치)"}
    cmd = [sys.executable, "-I", str(WORKER), str(ik_dir), mode] + ([json.dumps(arg)] if arg is not None else [])
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=TIMEOUT_S, cwd=str(HERE))
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": f"{TIMEOUT_S} 초 안에 안 끝남"}
    for line in reversed(p.stdout.splitlines()):
        if line.startswith(MARK):
            try:
                return json.loads(line[len(MARK):])
            except ValueError:
                break
    tail = (p.stderr or p.stdout or "").strip().splitlines()[-3:]
    return {"ok": False, "error": "RL 워커가 결과를 안 줌 (exit %s): %s" % (p.returncode, " | ".join(tail)[:400])}


def info(ik_dir):
    key = str(ik_dir)
    if key not in _info_cache:
        r = _call(ik_dir, "info")
        if r.get("ok"):
            _info_cache[key] = r      # 실패는 담지 않는다 (IK 레포를 바꾸고 다시 누르면 되게)
        return r
    return _info_cache[key]


def run(ik_dir, body):
    b = body if isinstance(body, dict) else {}
    try:
        arg = {"x_cm": _num(b, "x_cm", 0.0), "y_cm": _num(b, "y_cm", 0.0),
               "yaw_deg": _num(b, "yaw_deg", 0.0), "mass_g": _num(b, "mass_g", 40.0),
               "home": b.get("home") is True}   # 기본 = 들고 ㄴ자 복귀(쥔 채) — 화면 기본과 같게 (2026-11-25)
        arg["insert_push_cm"] = _num(b, "insert_push_cm", INSERT_PUSH_CM)
        arg["insert_side_cm"] = _num(b, "insert_side_cm", INSERT_SIDE_CM)   # INSERT 중 앞으로 더 (rl_worker._insert_push)
        if b.get("grip_force_x") is not None:           # 정책 쥐는 힘 배율 (rl_worker.GRIP_FORCE_X, 기본 3)
            arg["grip_force_x"] = _num(b, "grip_force_x", 3.0)
        tx, ty = b.get("target_x_cm"), b.get("target_y_cm")     # 놓을 자리 (선택): 둘 다 있거나 둘 다 없어야 한다
        if (tx is None) != (ty is None):
            raise ValueError("놓을 자리는 target_x_cm 과 target_y_cm 를 같이 줘야 함")
        if tx is not None:
            arg["target_cm"] = [_num(b, "target_x_cm", 0.0), _num(b, "target_y_cm", 0.0)]
        lim = b.get("limits_deg")           # 서버가 실물 보정에서 계산해 넣는다 (옮기기 IK 의 관절 범위)
        if isinstance(lim, dict):
            arg["limits_deg"] = {str(k): [float(v[0]), float(v[1])] for k, v in lim.items() if isinstance(v, (list, tuple)) and len(v) == 2}
    except ValueError as e:
        return {"ok": False, "error": str(e)}
    if not _lock.acquire(blocking=False):
        return {"ok": False, "error": "다른 RL 실행이 아직 도는 중"}
    try:
        r = _call(ik_dir, "run", arg)
        if r.get("ok"):
            r["run_id"] = secrets.token_hex(6)
            r["params"] = arg
            _runs[r["run_id"]] = {"res": r, "t": time.time()}
            for k in sorted(_runs, key=lambda k: _runs[k]["t"])[:-RUNS_KEEP]:
                _runs.pop(k, None)
        return r
    finally:
        _lock.release()


# ---------------------------------------------------------------- 실물 재생용 궤적
def get_run(run_id):
    e = _runs.get(str(run_id))
    return e["res"] if e else None


GRIP_DEFAULT = {"gain": 1.8, "add_deg": 15.0}   # 2026-11-25 사용자: 실물 손가락이 덜 쥔다 → 실물로 보낼 때만 더 쥐게 (아래 squeeze_grip). 1.4·10° → '더 꽉' 1.8·15°
GRIP_GAIN_RANGE, GRIP_ADD_RANGE = (1.0, 2.5), (0.0, 25.0)
GRIP_JOINTS = ("finger1_joint1", "finger2_joint1", "finger3_joint1")   # 더 쥐는 마디 = 첫 마디만 (2026-11-25 사용자: 마디 2 가 바깥으로 간다 —
                         # 더 쥐기를 마디 2 에도 줘서 18.9°(손가락 1·2)·45.5°(손가락 3) 까지 갔다, 시뮬은 7.8°·30.8°). 마디 2 는 시뮬 그대로
GRIP_FULL_DEG = 2.0      # 이만큼 닫힌 마디부터 더하기 각을 다 준다 (5° → 2°: 시뮬 손가락 1·2 는 큐브에 닿으면 4.6° 갔다 0.3° 로 풀려 거의 안 더해졌다) (그 아래는 비례 — 펴진 마디·살짝 움직인 마디는 거의 그대로)


def grip_params(gain=None, add_deg=None):
    """(gain, add_deg) 검사·기본값. 틀리면 ValueError."""
    g = GRIP_DEFAULT["gain"] if gain in (None, "") else float(gain)
    a = GRIP_DEFAULT["add_deg"] if add_deg in (None, "") else float(add_deg)
    if not (math.isfinite(g) and GRIP_GAIN_RANGE[0] <= g <= GRIP_GAIN_RANGE[1]):
        raise ValueError(f"손가락 배율은 {GRIP_GAIN_RANGE[0]:g}~{GRIP_GAIN_RANGE[1]:g}")
    if not (math.isfinite(a) and GRIP_ADD_RANGE[0] <= a <= GRIP_ADD_RANGE[1]):
        raise ValueError(f"손가락 더하기 각은 {GRIP_ADD_RANGE[0]:g}~{GRIP_ADD_RANGE[1]:g}°")
    return g, a


def squeeze_grip(pos, names, res, gain, add_deg, limits=None):
    """실물용: 정책이 큐브를 쥐는 동안(GRASP_LIFT 시작 ~ HOME 전, 없으면 끝까지) 손가락 마디를 더 닫는다.
    시뮬은 큐브에 닿는 각(마디 1 약 8°, 엄지 마디 2 약 30°)에서 접촉 힘이 나지만 실물 서보는 그 각까지만 가서 거의 안 누른다.
    목표 = 잡기 시작 + gain·D + add_deg·min(1, D/GRIP_FULL_DEG)  (D = 잡기 시작부터 지금까지 가장 많이 닫힌 양 > 0 — 들고 가며 시뮬이 살짝 펴도 유지).
    놓기(RELEASE)·HOME 부터는 더하기를 빼서 시뮬처럼 펴진다.
    더 닫는 양은 프레임마다 '시뮬 손가락이 한 프레임에 가장 많이 도는 각 − 이 프레임에 시뮬이 도는 각' 까지만 바꾼다 →
    손가락 최고 속도가 시뮬과 같아 실물 재생이 느려지지 않는다 (재생 속도는 가장 빠른 관절에 맞춰 늦춘다). limits 위 끝 − 0.5° 까지.
    돌려줌: 시뮬보다 가장 많이 더 닫은 각 (°)."""
    ph = {p["name"]: float(p["t"]) for p in res.get("phases") or []}
    dt = float(res["dt"])
    if "GRASP_LIFT" not in ph or len(pos) < 2 or dt <= 0 or (gain == 1.0 and add_deg == 0.0):
        return 0.0
    k0 = min(len(pos) - 1, int(round(ph["GRASP_LIFT"] / dt)))
    ends = [ph[n] for n in ("RELEASE", "HOME") if n in ph and ph[n] > ph["GRASP_LIFT"]]     # 놓기·HOME 전까지 쥔다
    k1 = min(len(pos), int(round(min(ends) / dt))) if ends else len(pos)
    idx = [i for i, n in enumerate(names) if n in GRIP_JOINTS]
    if not idx:
        return 0.0
    cap = max(abs(b[i] - a[i]) for a, b in zip(pos, pos[1:]) for i in idx)    # 시뮬 손가락 최고 속도 (rad/프레임)
    ref = {i: pos[k0][i] for i in idx}
    full, add = math.radians(GRIP_FULL_DEG), math.radians(add_deg)
    extra = {i: 0.0 for i in idx}
    dmax = {i: 0.0 for i in idx}     # 쥐는 동안 가장 많이 닫힌 양 — 시뮬 손가락이 들고 가며 살짝 펴져도 실물은 그만큼 쥔 채 (2026-11-25 '더 꽉')
    prev = {i: pos[k0 - 1][i] if k0 > 0 else pos[k0][i] for i in idx}   # 바꾸기 전 시뮬 값 (속도 비교용)
    most = 0.0
    for k in range(k0, len(pos)):
        for i in idx:
            sim = pos[k][i]
            d = sim - ref[i]
            if k < k1:
                dmax[i] = max(dmax[i], d)
            want = 0.0
            if k < k1 and dmax[i] > 0.0:
                v = ref[i] + gain * dmax[i] + add * min(1.0, dmax[i] / full)
                lim = (limits or {}).get(names[i])
                if lim:
                    v = min(v, math.radians(float(lim[1]) - 0.5))
                want = max(0.0, v - sim)
            room = max(0.0, cap - abs(sim - prev[i]))
            prev[i] = sim
            extra[i] = min(max(want, extra[i] - room), extra[i] + room)
            lim = (limits or {}).get(names[i])
            if lim:     # 줄어드는 중이어도 보정 범위는 절대 넘지 않게
                extra[i] = max(0.0, min(extra[i], math.radians(float(lim[1]) - 0.5) - sim))
            pos[k][i] = sim + extra[i]
            most = max(most, math.degrees(extra[i]))
    return most


def real_traj(res, grip=None, limits=None):
    """RL 결과 → 대시보드 궤적 (역기구학 계획과 같은 모양: joint_names, positions(rad), dt, unit).
    RL 의 RotationBody 관절(rl_rotation) = 대시보드 finger_rotation − π (arm.xml 은 −π 를 0 으로 둔다, rl_worker 머리말) →
    finger_rotation = rl_rotation + π (0~180°, calibration sim 범위와 같다). 그 밖의 13 관절은 이름·부호 그대로."""
    names = [("finger_rotation" if n == "rl_rotation" else n) for n in res["joint_names"]]
    k = res["joint_names"].index("rl_rotation")
    pos = []
    for row in res["positions"]:
        r = list(row)
        r[k] = r[k] + math.pi
        pos.append(r)
    squeezed = squeeze_grip(pos, names, res, *grip, limits=limits) if grip else 0.0
    dt = float(res["dt"]) or 1.0 / 30
    phase_frames = {p["name"]: min(len(pos) - 1, int(round(float(p["t"]) / dt))) for p in (res.get("phases") or [])
                    if isinstance(p, dict) and p.get("t") is not None}
    return {"joint_names": names, "positions": pos, "dt": res["dt"], "unit": "rad", "planner": "rl", "grip_extra_deg": round(squeezed, 1),
            "phase_frames": phase_frames,
            "policy": "grasp_lift_23025",   # run_record.kind_of 가 '강화학습' 기록으로 분류하는 키
            "reason": "강화학습 정책 grasp_lift_23025 시뮬 궤적 재생 (열린 고리: 실물 피드백으로 다시 계산하지 않음)",
            "summary": res.get("summary")}


RL_SPEED_DEFAULT = 30.0   # 2026-11-25 사용자 '강화학습 좀 더 빠르게': 실물 재생 기본 °/s (대시보드 상한 30, 예전 15)
RL_HOLD_S = 2.0           # 실물 재생 첫 자세 유지 (예전 5 초 — 시작 자세는 이미 4° 안으로 검사함)
RL_INSERT_SPEED = 10.0    # 2026-11-25: 시뮬 INSERT 는 0.75 초라 실물 모터가 못 따라감 (어깨 6° 늦음 ≈ 손끝 3 cm) → INSERT 구간만 이 °/s 로
RL_GRASP_DWELL_S = 0.8    # 손가락이 닫히기 직전(GRASP_LIFT 시작)에 멈춰 기다리는 시간 — 팔이 INSERT 끝까지 실제로 간 뒤 쥐게


def retime(traj, speed):
    """빠른 구간만 늦춘다: 프레임 사이 어느 관절이든 speed(°/s) 를 넘으면 그 구간만 사이 프레임을 끼워 넣는다 (dt 그대로, 선형 보간).
    예전엔 play.schedule 이 궤적에서 가장 빠른 한 프레임(손가락 57°/s)에 맞춰 전체를 늦췄다 → 15°/s 에서 64 초 (이 함수로 30°/s 약 19 초).
    결과의 src_map[k] = 새 프레임 k 가 온 원래 프레임 번호 (RL 탭 3D 가 실물 재생 프레임을 따라가게)."""
    q, dt = traj["positions"], float(traj["dt"])
    if not q or not speed > 0:
        return dict(traj, src_map=list(range(len(q))))
    lim = math.radians(speed) * dt * 0.999           # 한 프레임에 허용하는 각 (rad) — 반올림으로 schedule 이 다시 늦추지 않게 조금 덜
    pf = traj.get("phase_frames") or {}
    fi, fg = pf.get("INSERT"), pf.get("GRASP_LIFT")   # INSERT 구간은 RL_INSERT_SPEED 로, GRASP_LIFT 시작에서 RL_GRASP_DWELL_S 멈춤
    lim_ins = math.radians(min(speed, RL_INSERT_SPEED)) * dt * 0.999
    dwell = int(round(RL_GRASP_DWELL_S / dt)) if fg is not None else 0
    pos, src = [list(q[0])], [0]
    for i in range(1, len(q)):
        a, b = q[i - 1], q[i]
        li = lim_ins if fi is not None and fg is not None and fi < i <= fg else lim
        m = max(1, math.ceil(max(abs(y - x) for x, y in zip(a, b)) / li))
        for s in range(1, m + 1):
            f = s / m
            pos.append([x + (y - x) * f for x, y in zip(a, b)])
            src.append(i if s == m else i - 1)
        if i == fg:
            pos += [list(b) for _ in range(dwell)]
            src += [i] * dwell
    return dict(traj, positions=pos, src_map=src)


# ---------------------------------------------------------------- 카메라 (비전) → 큐브 위치
def _camera_result_module():
    f = HERE.parent / "vision_node" / "vision_node" / "camera_result.py"   # ROS 없는 순수 함수 (rclpy 를 안 부름)
    spec = importlib.util.spec_from_file_location("jax_camera_result", f)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _floats(v, n):
    if not (isinstance(v, (list, tuple)) and len(v) == n):
        return None
    try:
        out = [float(a) for a in v]
    except (TypeError, ValueError):
        return None
    return out if all(math.isfinite(a) for a in out) else None


def spot_from_robot(x, y, yaw_deg):
    """로봇 틀 큐브 (x, y m, yaw °) → RL 위치 오프셋 (cm) + 각도. 큐브는 90° 마다 같아 (−45, 45] 로."""
    yaw = (float(yaw_deg) + 45.0) % 90.0 - 45.0
    return {"x_cm": round((y - REF_Y) * 100.0, 1), "y_cm": round(-(x - REF_X) * 100.0, 1), "yaw_deg": round(yaw, 1)}


def read_vision(b):
    """카메라 프로그램(SteroDepthSeg)이 쓰는 inference.json 에서 큐브 하나를 골라 로봇 틀 위치와 RL 위치 오프셋으로.
    vision_node 의 순수 함수(camera_result)를 그대로 쓴다 — 선택 규칙(후보 정확히 1 개), 오래된 파일 거절, camera_to_robot 같음.
    camera_to_robot = [tx, ty, tz, roll°, pitch°, yaw°] (임시값, '폰 고정 방법 미정' — NODES.md). 없으면 카메라 좌표만 돌려준다."""
    b = b if isinstance(b, dict) else {}
    path = os.path.expanduser(str(b.get("path") or "").strip())
    if not path or not path.lower().endswith(".json"):
        return {"ok": False, "error": "inference.json 경로(.json)를 넣어 주세요"}
    try:
        if os.path.getsize(path) > 2 << 20:
            return {"ok": False, "error": "파일이 너무 큼 (2 MB 넘음) — inference.json 이 맞나요?"}
    except OSError as e:
        return {"ok": False, "error": f"파일을 못 읽음: {e.strerror or e}"}
    try:
        max_age = float(b.get("max_age_s", 3.0))
    except (TypeError, ValueError):
        return {"ok": False, "error": "max_age_s 가 숫자가 아님"}
    ctr = _floats(b.get("camera_to_robot"), 6) if b.get("camera_to_robot") not in (None, [], "") else None
    if b.get("camera_to_robot") not in (None, [], "") and ctr is None:
        return {"ok": False, "error": "camera_to_robot 은 숫자 6개 [x, y, z, roll°, pitch°, yaw°]"}
    cr = _camera_result_module()
    raw, why = cr.read_result(path, time.time(), max_age)
    if raw is None:
        return {"ok": False, "error": why}
    cube, why = cr.select_cube(raw, (40.0, 80.0), (0.3, 1.0), ("*",) if b.get("any_label") else cr.DEFAULT_LABELS)
    if cube is None:
        return {"ok": False, "error": why}
    out = {"ok": True, "instance": cube["instance"], "camera_xyz": [round(v, 4) for v in cube["xyz"]],
           "yaw_cam_deg": round(cube["yaw_deg"], 1), "frame": "camera", "robot": None, "spot": None}
    if ctr is None:
        out["note"] = "camera_to_robot 이 없어 카메라 좌표만 — 로봇 위치로 못 바꿈 (카메라 카드에 위치·자세를 넣으세요)"
        return out
    xyz = cr.camera_to_robot(cube["xyz"], ctr)
    # 큐브 윗면의 x 축 방향(카메라 틀 yaw)을 같은 변환으로 로봇 틀로 옮겨 로봇 yaw 를 얻는다 (윗면이 카메라 영상과 나란하다는 가정 — 근사)
    ya = math.radians(cube["yaw_deg"])
    tip = cr.camera_to_robot((cube["xyz"][0] + 0.05 * math.cos(ya), cube["xyz"][1] + 0.05 * math.sin(ya), cube["xyz"][2]), ctr)
    yaw_robot = math.degrees(math.atan2(tip[1] - xyz[1], tip[0] - xyz[0]))
    cube_z = TABLE_TOP_Z + 0.025
    out.update({"frame": "shoulder_roll_axis", "robot": {"x": round(xyz[0], 4), "y": round(xyz[1], 4), "z": round(xyz[2], 4), "yaw_deg": round(yaw_robot, 1)},
                "spot": spot_from_robot(xyz[0], xyz[1], yaw_robot), "z_err_mm": round((xyz[2] - cube_z) * 1000.0, 1)})
    if abs(xyz[2] - cube_z) > 0.03:
        out["warn"] = (f"변환한 큐브 높이가 책상 위 큐브 중심(어깨 아래 {-cube_z * 100:.1f} cm)과 {abs(xyz[2] - cube_z) * 1000:.0f} mm 다름 — "
                       "camera_to_robot(위치·자세)이나 책상 높이가 틀렸을 수 있음")
    out["yaw_note"] = "yaw 는 카메라 영상 기준을 로봇 틀로 옮긴 근사 — 3D 의 보라 상자 방향을 실물과 비교해 확인"
    return out
