"""강화학습 정책(team-jax/IK sim2sim 체인) 한 번 돌리기 — 서버가 서브프로세스로 부른다 (rl_run.py).

    python rl_worker.py <IK 레포 경로> run  '{"x_cm":0,"y_cm":0,"yaw_deg":0,"mass_g":40}'
    python rl_worker.py <IK 레포 경로> info

결과는 stdout 마지막 줄 '@@RL@@<json>' 한 줄. 시뮬 전용 — CAN·모터·ROS 를 건드리지 않는다.
sim2sim/chain.py 가 늘어뜨린 팔 → 들어올리기(조회표) → 밀어 넣기 → grasp_lift 정책(45 → 256 → 256 → 11) → 복귀를 MuJoCo 에서 돌린다.

3D 뷰어(viewer.js, arm.xml)에 그리려고 궤적을 바꾼다. 둘은 같은 로봇이라 몸체 이름이 같고, 관절은 몸체로 짝지으면 부호·영점이 같다
(2026-11-24 FK 로 확인: 팔 7관절 + 손가락 1·2 의 몸체 위치 오차 0.00 mm). 다른 점 둘:
  · 손가락 3 의 RotationBody 관절(hand_base_to_rotation_base)을 정책이 0 → −172° 로 돌리는데 arm.xml 은 −π 로 굳어 있다.
    arm.xml 쪽 자세 = 정적 자세 · Rot_z(θ + π) (몸체 틀의 z 축). 뷰어에 'rl_rotation' 키로 넘긴다 (손가락 3 오차 0.0 mm 로 확인).
  · 월드 좌표: 뷰어는 어깨 축 원점의 로봇 틀(x 로봇 왼쪽, y 뒤, z 위). 같은 관절값을 두 모델에 넣은 몸체 위치로 강체 변환을 구한다.
"""
import json
import math
import os
import sys

import numpy as np

MARK = "@@RL@@"
# RL 관절(chain.ISAAC 순서) → 뷰어(arm.xml) 관절 이름. 9 번(RotationBody)은 따로 'rl_rotation'.
DASH = ["shoulder_roll", "shoulder_yaw", "upper_arm_roll", "elbow", "wrist_pitch", "wrist_yaw", "hand_roll",
        "finger1_joint1", "finger2_joint1", None, "finger1_joint2", "finger2_joint2", "finger3_joint1", "finger3_joint2"]
FIT_BODIES = ["Upper_Arm", "Under_Arm", "Wrist_Pitch", "Wrist_yaw", "MG996R", "Finger_1_1", "Finger_1_2", "Finger_2_1", "Finger_2_2"]
VIEW_HZ = 30.0     # 뷰어로 보낼 프레임 수 (시뮬은 60 Hz)
ARM_SHIFT_X = 0.052   # viewer.js REAL_FROM_BASE 와 같은 값


def out(obj):
    sys.stdout.write("\n" + MARK + json.dumps(obj, ensure_ascii=False, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def mat(pos, quat):
    w, x, y, z = np.asarray(quat, dtype=float) / np.linalg.norm(quat)
    M = np.eye(4)
    M[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    M[:3, 3] = pos
    return M


def quat_from_rot(R):
    """3x3 회전 → (w, x, y, z)"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return np.array([s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) * 2
    q = np.zeros(4)
    q[1 + i] = s / 4
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return q


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def kabsch(P, Q):
    pc, qc = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - pc).T @ (Q - qc))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = qc - R @ pc
    return R, t, float(np.sqrt((((P @ R.T + t) - Q) ** 2).sum(1).mean()))


def world_to_real(C, mujoco, ik):
    """RL 월드 → 뷰어 로봇 틀 4x4 + 맞춤 오차(m). 같은 관절값을 두 모델에 넣고 몸체 위치를 맞춘다."""
    A = mujoco.MjModel.from_xml_path(C.SCENE)
    B = mujoco.MjModel.from_xml_path(os.path.join(ik, "motion2", "arm", "arm.xml"))
    Ad, Bd = mujoco.MjData(A), mujoco.MjData(B)
    jid = mujoco.mjtObj.mjOBJ_JOINT
    qa = [A.jnt_qposadr[mujoco.mj_name2id(A, jid, n)] for n in C.ISAAC]
    qb = {n: B.jnt_qposadr[mujoco.mj_name2id(B, jid, n)] for n in DASH if n}
    ba = [mujoco.mj_name2id(A, mujoco.mjtObj.mjOBJ_BODY, b) for b in FIT_BODIES]
    bb = [mujoco.mj_name2id(B, mujoco.mjtObj.mjOBJ_BODY, b) for b in FIT_BODIES]
    rng = np.random.default_rng(0)
    P, Q = [], []
    for _ in range(12):
        v = rng.uniform(-0.6, 0.6, 14)
        v[9] = 0.0
        Ad.qpos[:] = A.qpos0
        Bd.qpos[:] = B.qpos0
        for i, a in enumerate(qa):
            Ad.qpos[a] = v[i]
            if DASH[i]:
                Bd.qpos[qb[DASH[i]]] = v[i]
        mujoco.mj_kinematics(A, Ad)
        mujoco.mj_kinematics(B, Bd)
        P.append(Bd.xpos[bb].copy())
        Q.append(Ad.xpos[ba].copy())
    R, t, err = kabsch(np.array(P).reshape(-1, 3), np.array(Q).reshape(-1, 3))
    BA = np.eye(4)
    BA[:3, :3], BA[:3, 3] = R, t                                   # arm.xml 월드 → RL 월드
    col = mujoco.mj_name2id(B, mujoco.mjtObj.mjOBJ_BODY, "column")
    bl = mujoco.mj_name2id(B, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    wb = mat(B.body_pos[col], B.body_quat[col]) @ mat(B.body_pos[bl], B.body_quat[bl])
    T = np.eye(4)
    T[0, 3] = ARM_SHIFT_X
    return (T @ np.linalg.inv(wb)) @ np.linalg.inv(BA), err


# ---------------------------------------------------------------- 놓을 자리까지 옮기기 (RL 집기 + 손 방향 유지 평행이동)
# 정책(grasp_lift)은 집어서 들고 복귀할 뿐 놓을 곳을 모른다. 정책의 집기는 '옆에서 쥐기'(손 접근축이 수평)라 역기구학 계획기의 '위에서 쥐기'와
# 자세가 달라 둘을 이을 수 없다(쥔 채 손을 90° 돌리면 큐브를 놓친다). 그래서 쥔 손 방향을 그대로 두고 큐브만 놓을 자리 위로 평행이동한다:
#   ① 1차 실행으로 '들어올린 순간'의 상태를 얻는다  ② 그 방향 그대로 큐브 위치만 목표로 옮긴 관절해(q_dest)를 IK 로 푼다
#   ③ 2차 실행에서 정책의 '복귀' 목표(Q_L, ㄴ자)를 q_dest 로 바꿔 큐브를 쥔 채 옮긴다(손가락 힘 제어는 정책 그대로)
#   ④ 내리기 → 손가락 열기 → 접근 반대로 손 빼기. 전부 같은 MuJoCo 물리 시뮬이라 큐브를 놓치거나 비뚤게 놓이면 그대로 실패로 드러난다.
# 2026-11-24 시험: 5~10 cm 이동에서 큐브가 책상 위에 똑바로(기울기 0.0°) 놓이고 목표와 1.4~2.4 cm 차이. 먼 목표(36 cm)는 방향 고정 IK 가 안 풀림.
PLACE_TOL_CM = 3.0      # 놓은 자리가 목표에서 이만큼(cm) 안이어야 '놓음'
PLACE_TILT_DEG = 10.0   # 큐브 기울기 상한
# 손 빼기: 2026-11-24 시뮬 비교 — 예전(7 cm 뒤·1 cm 위·안 기다림)은 제자리 내려놓기(-8,-18)에서 손가락이 큐브를 8 cm 끌고 감.
# 0.3 s 기다린 뒤 5 cm 뒤·3 cm 위로 빼면 4 경우 모두 0.96~1.27 cm. 손가락을 0° 로 더 펴면 오히려 5~8 cm 끌려 HAND_SHAPE 유지.
RETREAT_BACK_M = 0.05   # 놓은 뒤 접근 반대 방향(수평)으로 손 빼는 거리
RETREAT_UP_M = 0.03     # 손 빼며 같이 올리는 높이
RELEASE_OPEN = "shape"  # 손가락 열기 목표: "shape" = 정책 처음 손 모양(HAND_SHAPE), "zero" = 모든 손가락 0°
RELEASE_WAIT_S = 0.3    # 손가락을 연 뒤 빼기 전에 기다리는 시간


def _grav(C, mujoco, c, q7):
    s = c.scratch
    s.qpos[:] = c.d.qpos
    s.qpos[c.qadr[c.arm[:7]]] = q7
    mujoco.mj_kinematics(c.m, s)
    mujoco.mj_comPos(c.m, s)
    out = np.zeros(c.m.nv)
    mujoco.mj_rne(c.m, s, 0, out)
    return out[c.vadr[c.arm[:7]]] / C.ARM_KP


def _ik_translate(C, mujoco, c, q14, hand_q, tip0, delta, iters=400, wr=2.0, q_init=None):
    """손 방향 hand_q 를 고정하고 쥐는 점(tip_mid)만 delta 만큼 옮기는 팔 7관절 해 (감쇠 최소제곱, 한계 안). (q7, 위치오차 m, 방향오차 °)"""
    m, s = c.m, c.scratch
    s.qpos[:] = c.d.qpos
    s.qpos[c.qadr] = q14
    dof, qa = c.vadr[c.arm[:7]], c.qadr[c.arm[:7]]
    lo, hi = c.lo[c.arm[:7]].copy(), c.hi[c.arm[:7]].copy()
    if REAL_LIM:                                                   # 실물이 못 따라가는 각도(예: 손목 비틀기 −74°)로는 풀지 않는다
        for i, nm in enumerate(DASH[:7]):
            if nm in REAL_LIM:
                a, b = REAL_LIM[nm]
                lo[i] = max(lo[i], math.radians(a + REAL_LIM_MARGIN_DEG))
                hi[i] = min(hi[i], math.radians(b - REAL_LIM_MARGIN_DEG))
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    target = tip0 + delta
    ep = er = None
    for _ in range(iters):
        mujoco.mj_kinematics(m, s)
        mujoco.mj_comPos(m, s)
        tip = np.mean([s.xpos[b] for b in c.tip_body], axis=0)
        ep = target - tip
        hq = s.xquat[c.hand_body]
        er = C.axis_angle(C.quat_mul(hand_q, hq * np.array([1, -1, -1, -1])))
        if np.linalg.norm(ep) < 5e-4 and np.linalg.norm(er) < math.radians(0.3):
            break
        Jp = np.zeros((3, 7))
        for b in c.tip_body:
            mujoco.mj_jacBody(m, s, jp, jr, b)
            Jp += jp[:, dof] / len(c.tip_body)
        mujoco.mj_jacBody(m, s, jp, jr, c.hand_body)
        J = np.vstack([Jp, jr[:, dof]])
        e = np.concatenate([np.clip(ep, -0.01, 0.01), np.clip(wr * er, -0.05, 0.05)])
        s.qpos[qa] = np.clip(s.qpos[qa] + J.T @ np.linalg.solve(J @ J.T + 0.01 ** 2 * np.eye(6), e), lo, hi)
    mujoco.mj_kinematics(m, s)
    return s.qpos[qa].copy(), float(np.linalg.norm(ep)), float(math.degrees(np.linalg.norm(er)))


REAL_LIM = None   # 실물 보정이 허용하는 팔 7관절 범위 {이름: [lo°, hi°]} — 옮기기 IK 가 이 안에서만 풀게 (cmd_run 이 채운다)
REAL_LIM_MARGIN_DEG = 2.0
PLACE_ORI_MAX_DEG = 8.0  # 방향을 완화해서라도 풀 때 허용하는 손 방향 오차(큐브는 손가락이 쥐고 있어 몇 도는 괜찮다)


def _solve_dest(C, mujoco, c, q14, hq, tip0, delta):
    """방향 고정 IK 가 안 풀리면 ① 이동을 여러 단계로 쪼개 이어 풀고 ② 방향 가중치를 낮춰 다시 푼다. 가장 좋은 해를 돌려준다."""
    best = None
    for wr, steps in ((2.0, 1), (2.0, 4), (0.7, 4), (0.3, 4)):
        q = np.array(q14, dtype=float)
        for i in range(1, steps + 1):
            q7, ep, er = _ik_translate(C, mujoco, c, q, hq, tip0, delta * i / steps, wr=wr)
            q[c.arm[:7]] = q7
        if ep <= 2e-3 and er <= 2.0:
            return q7, ep, er
        if ep <= 3e-3 and er <= PLACE_ORI_MAX_DEG and (best is None or er < best[2]):
            best = (q7, ep, er)
    return best if best else (q7, ep, er)


def _move(C, mujoco, c, q_to, secs, fing=None):
    """팔 7관절을 지금 측정값에서 q_to 로 smoothstep 보간 (중력 처짐 보정 포함), 손가락 목표는 fing 으로 (없으면 그대로)."""
    q_from = c.q()[c.arm[:7]].copy()
    n = max(1, round(secs / C.DT))
    f_from = c.target[c.fing].copy()
    for i in range(n):
        w = C.smoothstep((i + 1) / n)
        q = q_from + w * (q_to - q_from)
        c.target[c.arm[:7]] = np.clip(q + _grav(C, mujoco, c, q), c.lo[c.arm[:7]], c.hi[c.arm[:7]])
        c.target[c.arm[7]] = C.ROT_BODY
        if fing is not None:
            c.target[c.fing] = f_from + w * (fing - f_from)
        c.step()


HOME_SPEED_DEG_S = 30.0   # HOME 으로 돌아가는 속도 (가장 많이 도는 관절 기준)


HOME_TWIST_DEG = 90.0      # 팔뚝을 옆으로 돌리는 upper_arm_roll 각 (부호는 HOME_TWIST_SIGN)
HOME_TWIST_SIGN = 1.0      # +1: 시뮬에서 놓기 뒤 책상 안 닿음 (−1 은 손가락이 책상을 침, 2026-11-24)
HOME_LIFT_M = 0.12        # 책상 윗면에서 쥐는 점을 이만큼 위로 올린 뒤 돌린다


def _table_hits(C, mujoco, c):
    """지금 책상에 닿은 몸체 이름들 (큐브 제외 — 큐브가 책상에 닿는 건 '놓음')."""
    tb = mujoco.mj_name2id(c.m, mujoco.mjtObj.mjOBJ_GEOM, "table")
    out = set()
    for k in range(c.d.ncon):
        g1, g2 = c.d.contact[k].geom1, c.d.contact[k].geom2
        if tb in (g1, g2):
            o = g2 if g1 == tb else g1
            if c.m.geom_bodyid[o] != c.cube_body:
                out.add(mujoco.mj_id2name(c.m, mujoco.mjtObj.mjOBJ_BODY, c.m.geom_bodyid[o]))
    return out


def _joint_move(C, mujoco, c, q_to7, keep_grip, q0, hits, rot_fing=False):
    a7, rot = c.arm[:7], c.arm[7]
    q_from = c.q()[a7].copy()
    rot_from, f_from = float(c.target[rot]), c.target[c.fing].copy()
    span = float(np.abs(q_to7 - q_from).max())
    if rot_fing and not keep_grip:
        span = max(span, abs(float(q0[rot]) - rot_from))
    n = max(1, round(max(1.5, math.degrees(span) / HOME_SPEED_DEG_S) / C.DT))
    for i in range(n):
        w = C.smoothstep((i + 1) / n)
        q = q_from + w * (q_to7 - q_from)
        c.target[a7] = np.clip(q + _grav(C, mujoco, c, q), c.lo[a7], c.hi[a7])
        if rot_fing and not keep_grip:
            c.target[rot] = rot_from + w * (float(q0[rot]) - rot_from)
            c.target[c.fing] = f_from + w * (q0[c.fing] - f_from)
        c.step()
        hits |= _table_hits(C, mujoco, c)


def _go_home(C, mujoco, c, q0, keep_grip):
    """시작 자세(q0 = 궤적 첫 프레임 = HOME, 팔 모든 관절 0°)로 돌아간다. ㄴ자에서 팔꿈치를 바로 펴면 팔뚝이 앞(책상 쪽)으로
    원을 그리며 내려가 손이 책상을 친다 (시뮬로 확인). 그래서 ① 쥐는 점을 책상 위로 올리고 ② upper_arm_roll 을 90° 돌려
    팔뚝을 옆(책상 밖)으로 향하게 한 뒤 ③ 팔꿈치·손목을 펴고 ④ upper_arm_roll 을 0 으로 (늘어진 팔이라 제자리 회전).
    keep_grip 이면 손가락·회전몸체는 그대로(큐브를 쥔 채), 아니면 ④ 에서 시작 값으로. (도착 오차 °, 책상에 닿은 몸체들)"""
    q0 = np.asarray(q0, dtype=float)
    a7 = c.arm[:7]
    hits = set()
    tb = mujoco.mj_name2id(c.m, mujoco.mjtObj.mjOBJ_GEOM, "table")
    top = float(c.d.geom_xpos[tb][2] + c.m.geom_size[tb][2])
    up = top + HOME_LIFT_M - c.tip_mid()[2]
    if up > 0.0:                                                     # ① 들어 올리기 (손 방향 유지)
        q_up, _, _ = _ik_translate(C, mujoco, c, c.q(), c.d.xquat[c.hand_body].copy(), c.tip_mid(), np.array([0.0, 0.0, up]))
        _joint_move(C, mujoco, c, q_up, keep_grip, q0, hits)
    i_roll, i_elb = 2, 3                                             # DASH: upper_arm_roll, elbow
    q = c.q()[a7].copy()
    q[i_roll] = q0[a7][i_roll] + HOME_TWIST_SIGN * math.radians(HOME_TWIST_DEG)
    _joint_move(C, mujoco, c, q, keep_grip, q0, hits)                # ② 팔뚝을 옆으로
    q2 = q0[a7].copy()
    q2[i_roll] = q[i_roll]
    _joint_move(C, mujoco, c, q2, keep_grip, q0, hits)               # ③ 펴기
    _joint_move(C, mujoco, c, q0[a7].copy(), keep_grip, q0, hits, rot_fing=True)   # ④ 되돌리기 (+ 손 시작 모양)
    for _ in range(round(1.0 / C.DT)):
        c.step()
        hits |= _table_hits(C, mujoco, c)
    return float(np.degrees(np.abs(c.q()[a7] - q0[a7]).max())), sorted(hits)


def run_with_target(C, mujoco, spot, target_cm, mass, home=True, put_back=False):
    """(summary, trace) — 집기(정책) + 옮기기 + 놓기. 놓지 못해도 궤적은 돌려주고 summary['done']=False·fail_reason 에 이유."""
    dx, dy, yaw = spot
    s1, tr1 = C.run(dx, dy, yaw, mass=mass)
    if not s1.get("done"):
        s1["place"] = {"placed": False, "why": "집기 단계에서 시뮬이 실패"}
        return s1, tr1
    k = int(np.argmin(np.abs(tr1["t"] - s1["phase_t_s"]["RETURN"])))
    base = tr1["cube_pos"][0][:2] - np.array([dx, dy])         # 기준 큐브 월드 xy (오프셋 0, 0)
    tbl_cube_z = float(tr1["cube_pos"][0][2])                  # 책상 위 큐브 중심 높이
    want = base + np.asarray(target_cm, dtype=float) / 100.0
    c0 = C.Chain(dx, dy, yaw, mass=mass)
    hq, tip0, cube0 = tr1["hand_quat"][k], tr1["tip_mid"][k], tr1["cube_pos"][k]
    delta = np.array([want[0] - cube0[0], want[1] - cube0[1], 0.0])
    q_dest, ep, er = _solve_dest(C, mujoco, c0, tr1["q"][k], hq, tip0, delta)
    if ep > 3e-3 or er > PLACE_ORI_MAX_DEG:
        lo_f, hi_f = 0.0, 1.0                                    # 그 방향으로 어디까지는 되는지 이분탐색
        for _ in range(5):
            mid = (lo_f + hi_f) / 2
            _, e1, e2 = _solve_dest(C, mujoco, c0, tr1["q"][k], hq, tip0, delta * mid)
            lo_f, hi_f = (mid, hi_f) if (e1 <= 3e-3 and e2 <= PLACE_ORI_MAX_DEG) else (lo_f, mid)
        reach_cm = float(np.linalg.norm(delta[:2]) * lo_f * 100.0)
        s1["done"], s1["fail_reason"] = False, (f"놓을 자리까지 쥔 손 방향 그대로 못 옮김 (IK 오차 위치 {ep * 1000:.0f} mm, 방향 {er:.1f}°). "
                                                f"이 큐브에서 그쪽으로는 약 {reach_cm:.0f} cm 까지만 옮길 수 있음 — 더 가까운 자리나 다른 방향으로")
        s1["place"] = {"placed": False, "why": s1["fail_reason"]}
        return s1, tr1
    orig = C.Q_L
    C.Q_L = q_dest + _grav(C, mujoco, c0, q_dest)               # 복귀 목표를 '놓을 자리 위' 로 (처짐 보정 포함)
    try:
        c = C.Chain(dx, dy, yaw, mass=mass)
        c.raise_and_settle()
        c.insert()
        c.grasp_lift(after_done_s=0.5)
    finally:
        C.Q_L = orig
    marks = {}
    if c.done_t is None:
        summary, tr = C.summarize(c), {kk: np.asarray(v) for kk, v in c.trace.items()}
        summary["done"], summary["fail_reason"] = False, "큐브를 쥔 채 옮기는 중 시뮬이 실패 (큐브를 놓침)"
        summary["place"] = {"placed": False, "why": summary["fail_reason"]}
        return summary, tr
    marks["LOWER"] = c.t
    dz = -(c.cube_pos()[2] - tbl_cube_z - 0.002)
    q_low, ep2, _ = _ik_translate(C, mujoco, c, c.q(), hq, c.tip_mid(), np.array([0.0, 0.0, dz]))
    _move(C, mujoco, c, q_low, 2.0)
    for _ in range(round(0.5 / C.DT)):
        c.step()
    marks["RELEASE"] = c.t
    open_q = C.HAND_SHAPE.copy() if RELEASE_OPEN == "shape" else np.zeros_like(C.HAND_SHAPE)
    _move(C, mujoco, c, q_low, 1.0, fing=open_q)                  # 손가락 열기
    for _ in range(round(RELEASE_WAIT_S / C.DT)):
        c.step()
    marks["RETREAT"] = c.t
    z_ax = c.d.xmat[c.hand_body].reshape(3, 3)[:, 2].copy()      # 접근 방향(수평 성분)의 반대로 7 cm 빼기
    z_ax[2] = 0.0
    z_ax /= np.linalg.norm(z_ax)
    q_ret, ep3, _ = _ik_translate(C, mujoco, c, c.q(), hq, c.tip_mid(), -RETREAT_BACK_M * z_ax + np.array([0.0, 0.0, RETREAT_UP_M]))
    _move(C, mujoco, c, q_ret, 1.5)
    for _ in range(round(0.7 / C.DT)):
        c.step()
    home_err, home_hits = 0.0, []
    if home:
        marks["HOME"] = c.t
        home_err, home_hits = _go_home(C, mujoco, c, tr1["q"][0], keep_grip=False)   # 손 빼고 시작 자세(HOME)로
    end, tilt = c.cube_pos(), float(C.quat_tilt_deg(c.cube_quat()))
    xy_err = float(np.linalg.norm(end[:2] - want) * 100.0)
    on_table = bool(abs(end[2] - tbl_cube_z) < 0.006)
    placed = bool(on_table and xy_err <= PLACE_TOL_CM and tilt <= PLACE_TILT_DEG)
    why = "" if placed else (f"놓은 큐브가 목표에서 {xy_err:.1f} cm (허용 {PLACE_TOL_CM:g})" if xy_err > PLACE_TOL_CM else
                             "놓은 큐브가 책상 위에 안 있음" if not on_table else f"놓은 큐브가 {tilt:.0f}° 기울어짐")
    summary, tr = C.summarize(c), {kk: np.asarray(v) for kk, v in c.trace.items()}
    summary["phase_t_s"] = {**summary.get("phase_t_s", {}), **marks}
    summary["place"] = {"placed": placed, "xy_err_cm": round(xy_err, 2), "tilt_deg": round(tilt, 2), "on_table": on_table, "why": why,
                        "target_cm": [float(target_cm[0]), float(target_cm[1])]}
    summary["place"]["put_back"] = bool(put_back)
    summary["home"] = {"err_deg": round(home_err, 2), "cube_held": False, "table_hits": home_hits} if home else None
    if home and placed and (home_hits or home_err > 3.0):
        placed = False
        summary["place"]["placed"] = False
        why = summary["place"]["why"] = (f"놓은 뒤 HOME 으로 오다가 책상에 닿음 ({', '.join(home_hits)})" if home_hits else
                                         f"놓은 뒤 HOME 에 못 닿음 (오차 {home_err:.0f}°)")
    summary["done"] = bool(summary.get("done") and placed)
    if not placed:
        summary["fail_reason"] = why
    return summary, tr


DROP_OPEN_S = 0.5      # 놓기: 들어 올린 자리에서 손가락을 펴는 시간 (힘 빼기)
DROP_SETTLE_S = 1.0    # 편 뒤 큐브가 떨어져 멈출 때까지


def run_lift_drop(C, mujoco, spot, mass, home=True):
    """(summary, trace) — 2026-11-25 사용자: 강화학습은 드는 것만. 정책으로 집어 들고(RETURN 자세까지) → 그 자리에서 손가락 힘을 빼
    큐브를 그대로 떨어뜨림 → HOME. 옮기거나 내려놓지 않는다. 성공 = 들기 성공 + (home 이면) HOME 오는 길에 책상 안 닿고 3° 안에 도착."""
    dx, dy, yaw = spot
    c = C.Chain(dx, dy, yaw, mass=mass)
    c.raise_and_settle()
    c.insert()
    c.grasp_lift(after_done_s=0.5)
    if c.done_t is None:
        summary, tr = C.summarize(c), {kk: np.asarray(v) for kk, v in c.trace.items()}
        summary["done"] = False
        summary.setdefault("fail_reason", "집어 들기 단계에서 시뮬이 실패")
        return summary, tr
    q0 = np.asarray(c.trace["q"][0], dtype=float)                 # 시작 자세 = HOME
    lift_z = float(c.cube_pos()[2] - c.trace["cube_pos"][0][2])   # 큐브를 든 높이 (시작보다)
    marks = {"RELEASE": c.t}
    open_q = C.HAND_SHAPE.copy() if RELEASE_OPEN == "shape" else np.zeros_like(C.HAND_SHAPE)
    _move(C, mujoco, c, c.q()[c.arm[:7]].copy(), DROP_OPEN_S, fing=open_q)   # 팔은 그 자리, 손가락만 펴기
    for _ in range(round(DROP_SETTLE_S / C.DT)):
        c.step()
    tbl_cube_z = float(c.trace["cube_pos"][0][2])
    end, tilt = c.cube_pos(), float(C.quat_tilt_deg(c.cube_quat()))
    dropped = bool(abs(end[2] - tbl_cube_z) < 0.03)                # 책상 근처로 떨어졌나 (기울어 누워도 됨)
    home_err, home_hits = 0.0, []
    if home:
        marks["HOME"] = c.t
        home_err, home_hits = _go_home(C, mujoco, c, q0, keep_grip=False)
    summary, tr = C.summarize(c), {kk: np.asarray(v) for kk, v in c.trace.items()}
    summary["phase_t_s"] = {**summary.get("phase_t_s", {}), **marks}
    summary["drop"] = {"lift_cm": round(lift_z * 100, 1), "dropped": dropped, "tilt_deg": round(tilt, 1),
                       "moved_cm": round(float(np.linalg.norm(end[:2] - c.trace["cube_pos"][0][:2]) * 100), 1)}
    summary["home"] = {"err_deg": round(home_err, 2), "cube_held": False, "table_hits": home_hits} if home else None
    ok = bool(summary.get("done"))
    why = ""
    if ok and not dropped:
        ok, why = False, "손가락을 폈는데 큐브가 안 떨어짐 (손에 걸림)"
    elif ok and home and home_hits:
        ok, why = False, f"놓은 뒤 HOME 으로 오다가 책상에 닿음 ({', '.join(home_hits)})"
    elif ok and home and home_err > 3.0:
        ok, why = False, f"놓은 뒤 HOME 에 못 닿음 (오차 {home_err:.0f}°)"
    summary["done"] = ok
    if why:
        summary["fail_reason"] = why
    return summary, tr


GRIP_FORCE_X = 5.0     # (3 → 5: 2026-11-25 '손가락 행동 스케일 크게, 더 꽉') — 정책 손가락 행동의 힘 목표 배율 (2026-11-25 사용자: 잡을 때 더 오므리게 — 행동 스케일). 정책이 고른 쥐는 힘
                       # (손가락 1·2 = 0.5·(5 + 4·a7), 엄지 = 5 + 4·a9) 에 곱한다. 12 자리 시뮬: 1× = 11/12 성공 · 첫 마디 6.9/4.3°,
                       # 3× = 11/12 · 8.5/8.9° (5× 부터 한 자리가 큐브를 놓침). IK 레포 파일은 안 고치고 grasp_lift 사본만 바꿔 끼운다
GRIP_FORCE_RANGE = (1.0, 6.0)


def patch_grip(C, x):
    """chain.Chain.grasp_lift 를 '쥐는 힘 목표 × x' 사본으로 바꾼다 (원본은 C._jax_grasp_lift_orig 에 둔다). 못 바꾸면 False (원본 그대로)."""
    orig = getattr(C, "_jax_grasp_lift_orig", None) or C.Chain.grasp_lift
    C._jax_grasp_lift_orig = orig
    if abs(x - 1.0) < 1e-9:
        C.Chain.grasp_lift = orig
        return True
    import textwrap
    lines = open(C.__file__, encoding="utf-8").read().split("\n")
    try:
        i0 = next(i for i, l in enumerate(lines) if l.startswith("    def grasp_lift("))
        i1 = next(i for i in range(i0 + 1, len(lines)) if lines[i].startswith("    def ") or (lines[i] and not lines[i].startswith(" ")))
    except StopIteration:
        return False
    src = textwrap.dedent("\n".join(lines[i0:i1]))
    a = "want = np.array([0.5 * sf, 0.5 * sf, st]) if on.all() else np.full(3, 0.3)"
    if src.count(a) != 1:          # IK 레포가 바뀌어 그 줄이 없으면 원본 그대로 (배율 없이)
        return False
    src = src.replace(a, "want = (_JAX_GRIP_X * np.array([0.5 * sf, 0.5 * sf, st])) if on.all() else np.full(3, 0.3)")
    ns = dict(vars(C))
    ns["_JAX_GRIP_X"] = float(x)
    exec(compile(src, "<grasp_lift grip x%g>" % x, "exec"), ns)
    C.Chain.grasp_lift = ns["grasp_lift"]
    return True


INSERT_PUSH_FWD = (0.0, -1.0, 0.0)   # 로봇 틀 '앞' (−y)
INSERT_PUSH_LEFT = (1.0, 0.0, 0.0)   # 로봇 틀 '왼쪽' (+x) — side_m 이 − 면 오른쪽


def _insert_push(C, mujoco, spot, mass, tr, ph, Rm, push_m, frames, side_m=0.0):
    """2026-11-25 사용자 'INSERT 작업중 앞으로 더 땡겨': 재생 궤적의 frames 에서 쥐는 점을 로봇 앞(−y)으로 push_m 더 (손 방향 그대로, 실물 범위 안).
    INSERT 동안 0 → push_m (smoothstep), 잡고 드는 동안 push_m, RETURN 동안 → 0. 물리 시뮬(성공 판정·큐브 움직임)은 고치기 전 궤적 그대로.
    → (프레임 → 고친 14관절, 정보)"""
    ti, tg, tret = ph.get("INSERT"), ph.get("GRASP_LIFT"), ph.get("RETURN")
    tdone = ph.get("DONE") or float(tr["t"][-1])
    if ti is None or tg is None or tret is None or not tg > ti or not tdone > tret:
        return {}, {"applied": False, "why": "단계 시각이 없음"}
    c = C.Chain(*spot, mass=mass)            # 운동학만 (이 Chain 은 돌리지 않는다)
    fw_robot = push_m * np.array(INSERT_PUSH_FWD) + side_m * np.array(INSERT_PUSH_LEFT)
    fw = Rm.T @ (push_m * np.array(INSERT_PUSH_FWD) + side_m * np.array(INSERT_PUSH_LEFT))   # 로봇 틀 앞·옆 → 시뮬 world (m)
    fw[2] = 0.0
    push_m = 1.0                             # 아래 w * push_m * fw = w * (앞·옆 이동)
    out, worst = {}, 0.0
    for k in frames:
        t = float(tr["t"][k])
        if t < ti or t > tdone:
            continue
        w = C.smoothstep((t - ti) / (tg - ti)) if t < tg else 1.0 if t < tret else 1.0 - C.smoothstep((t - tret) / (tdone - tret))
        if w <= 1e-4:
            continue
        q14 = np.array(tr["q"][k], dtype=float)
        s = c.scratch
        s.qpos[:] = c.d.qpos
        s.qpos[c.qadr] = q14
        mujoco.mj_kinematics(c.m, s)
        tip0 = np.mean([s.xpos[b] for b in c.tip_body], axis=0)
        hq = s.xquat[c.hand_body].copy()
        q7, ep, er = _ik_translate(C, mujoco, c, q14, hq, tip0, w * push_m * fw, iters=300)
        q14[c.arm[:7]] = q7
        out[k] = q14
        worst = max(worst, ep)
    return out, {"applied": bool(out), "push_cm": round(float(-fw_robot[1]) * 100, 1), "side_cm": round(float(fw_robot[0]) * 100, 1),
                 "frames": len(out), "worst_err_mm": round(worst * 1000, 1)}


def cmd_run(C, mujoco, ik, a):
    x_cm, y_cm = float(a.get("x_cm", 0.0)), float(a.get("y_cm", 0.0))
    yaw, mass = float(a.get("yaw_deg", 0.0)), float(a.get("mass_g", 40.0)) / 1000.0
    tgt = a.get("target_cm")
    global REAL_LIM
    REAL_LIM = {k: [float(v[0]), float(v[1])] for k, v in (a.get("limits_deg") or {}).items()}
    home = bool(a.get("home", True))
    try:
        gx = float(a.get("grip_force_x", GRIP_FORCE_X))
    except (TypeError, ValueError):
        gx = GRIP_FORCE_X
    gx = min(max(gx, GRIP_FORCE_RANGE[0]), GRIP_FORCE_RANGE[1])
    grip_ok = patch_grip(C, gx)
    spot = (x_cm / 100.0, y_cm / 100.0, yaw)
    if tgt:
        summary, tr = run_with_target(C, mujoco, spot, tgt, mass, home=home)
    elif home:   # 2026-11-25 사용자: 드는 것만 — 들어 올린 자리에서 손가락 힘을 빼 그대로 놓고 HOME (예전: 제자리에 내려놓기 put_back)
        summary, tr = run_lift_drop(C, mujoco, spot, mass, home=True)
    else:
        summary, tr = C.run(*spot, mass=mass)
    M, fit_err = world_to_real(C, mujoco, ik)
    if fit_err > 1e-4:
        return {"ok": False, "error": f"RL 모델과 뷰어 모델의 관절 짝이 안 맞음 (몸체 위치 오차 {fit_err * 1000:.2f} mm) — IK 레포 버전 확인"}
    Rm = M[:3, :3]
    target_robot = None
    if tgt:   # 놓을 자리(큐브 중심, 책상 위)를 뷰어 로봇 틀로: 3D 의 초록 상자
        c_start = tr["cube_pos"][0]
        want_w = np.array([c_start[0] - x_cm / 100.0 + tgt[0] / 100.0, c_start[1] - y_cm / 100.0 + tgt[1] / 100.0, c_start[2], 1.0])
        target_robot = [round(float(v), 5) for v in (M @ want_w)[:3]]
    n = len(tr["t"])
    stride = max(1, int(round((1.0 / C.DT) / VIEW_HZ)))
    idx = list(range(0, n, stride))
    if idx[-1] != n - 1:
        idx.append(n - 1)
    push_q, push_info = {}, None
    try:
        push_m = float(a.get("insert_push_cm") or 0.0) / 100.0
    except (TypeError, ValueError):
        push_m = 0.0
    try:
        side_m = float(a.get("insert_side_cm") or 0.0) / 100.0
    except (TypeError, ValueError):
        side_m = 0.0
    if (push_m > 0 or side_m) and not tgt:
        push_q, push_info = _insert_push(C, mujoco, spot, mass, tr, summary.get("phase_t_s") or {}, Rm, min(push_m, 0.05), idx,
                                         side_m=max(-0.05, min(0.05, side_m)))
    names = [d for d in DASH if d] + ["rl_rotation"]
    cols = [i for i, d in enumerate(DASH) if d] + [9]
    pos, quat, cube, cube_q = [], [], [], []
    for k in idx:
        qk = push_q.get(k, tr["q"][k])
        pos.append([round(float(qk[i]), 5) for i in cols])
        p = M @ np.append(tr["cube_pos"][k], 1.0)
        cube.append([round(float(v), 5) for v in p[:3]])
        w, x, y, z = qmul(quat_from_rot(Rm), tr["cube_quat"][k])
        cube_q.append([round(float(x), 5), round(float(y), 5), round(float(z), 5), round(float(w), 5)])   # three.js 순서 x y z w
    # 단계: 시작 시각이 있는 것만 (RAISE 가 0, 그 뒤 …)
    ph = summary.get("phase_t_s") or {}
    phases = sorted(({"name": k, "t": float(v)} for k, v in ph.items() if v is not None), key=lambda d: d["t"])
    # 책상 (장면의 world 박스): 뷰어 틀로. 로봇 틀 회전이 90° 배수라 크기는 축을 바꿔 쓴다.
    table = None
    A = mujoco.MjModel.from_xml_path(C.SCENE)
    for g in range(A.ngeom):
        if A.geom_bodyid[g] == 0 and A.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and \
                (mujoco.mj_id2name(A, mujoco.mjtObj.mjOBJ_GEOM, g) or "") == "table":
            c = M @ np.append(A.geom_pos[g], 1.0)
            size = (np.abs(np.round(Rm)) @ (2 * A.geom_size[g])).tolist()
            table = {"center": [round(float(v), 4) for v in c[:3]], "size": [round(float(v), 4) for v in size]}
    keep = ("spot_cm", "cube_yaw_deg", "cube_mass_kg", "theta_deg", "valid_spot", "done", "fail_reason", "max_rise_m",
            "handover_err_deg", "first_touch_after_close_s", "all_touch_after_close_s", "cube_shift_before_grasp_mm", "place", "home", "drop")
    sm = {k: summary[k] for k in keep if k in summary}
    for k, v in list(sm.items()):
        if isinstance(v, (np.floating, np.integer)):
            sm[k] = float(v)
        elif isinstance(v, np.bool_):
            sm[k] = bool(v)
    sm["duration_s"] = float(tr["t"][-1])
    sm["grip_force_x"] = gx if grip_ok else 1.0
    if push_info is not None:
        sm["insert_push"] = push_info       # 재생 궤적만 고침 (큐브 3D·성공 판정은 고치기 전 시뮬)
    try:   # 손가락이 닫히기 직전(GRASP_LIFT 시작) 손가락 끝 가운데 − 큐브 중심, 로봇 틀 cm (+x 왼쪽, −y 앞) — 밀어 넣기 고치기 전 시뮬
        tg = (summary.get("phase_t_s") or {}).get("GRASP_LIFT")
        if tg is not None and "tip_mid" in tr:
            kg = int(np.searchsorted(tr["t"], tg))
            d = Rm @ (np.asarray(tr["tip_mid"][kg]) - np.asarray(tr["cube_pos"][kg]))
            sm["grasp_offset_cm"] = [round(float(v) * 100, 1) for v in d]
    except Exception:
        pass
    return {"ok": True, "dt": float(tr["t"][idx[1]] - tr["t"][idx[0]]) if len(idx) > 1 else 0.0,
            "joint_names": names, "positions": pos, "cube": cube, "cube_quat": cube_q, "phases": phases,
            "phase_idx": [int(tr["phase"][k]) for k in idx], "table": table, "target_robot": target_robot, "summary": sm, "fit_err_mm": fit_err * 1000}


def cmd_info(C, ik):
    T = C.Tables()
    gx, gy = T.gx, T.gy
    cells = [[bool(T.valid(float(x), float(y))) for x in gx] for y in gy]    # 행 = y, 열 = x
    meta = {}
    try:
        with open(os.path.join(C.DEPLOY, "grasp_lift_23025_meta.json"), encoding="utf-8") as f:
            m = json.load(f)
        meta = {"checkpoint": m.get("checkpoint"), "network": m.get("network")}
    except Exception:
        pass
    return {"ok": True, "xs_cm": [round(float(v) * 100, 2) for v in gx], "ys_cm": [round(float(v) * 100, 2) for v in gy],
            "valid": cells, "policy": "grasp_lift_23025", **meta}


def main():
    ik, mode = sys.argv[1], sys.argv[2]
    arg = json.loads(sys.argv[3]) if len(sys.argv) > 3 else {}
    sys.path.insert(0, os.path.join(ik, "sim2sim"))
    try:
        import mujoco
        import chain as C
        out(cmd_run(C, mujoco, ik, arg) if mode == "run" else cmd_info(C, ik))
    except Exception as e:
        import traceback
        out({"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-1500:]})


if __name__ == "__main__":
    main()
