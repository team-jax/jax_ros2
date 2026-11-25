"""대시보드 집어 옮기기 계획 (해석 IK) — '집을 수 있는 곳' 지도와 같은 IK 로 (2026-11-20).

    python dashboard/pick_plan.py --cube X Y YAW --target X Y --max-speed 30 --save logs/pick_dry_traj.json --dry-run

[계획 돌려 보기] 의 기본. 팀 계획(arm_control/pick_place.py: team-jax/IK motion1 MuJoCo 물리 시뮬, 자기 수치 IK)은
지도(hand_ik.grasp_eval, 해석 IK)와 판정이 달라 초록 칸에서도 실패했다. 여기는 지도와 같은 hand_ik._Solver 로 푼다:
  - 집는 점(end_effector site, 손끝 사이) = 큐브 중심 + GRIP_ABOVE_CENTER 1 cm (책상 위 3.5 cm, hand_ik 한 곳), shoulder_yaw 0 고정, 손목 중앙으로 해석 IK
    + 손목 2 관절로 손 최대한 아래 + 집는 점 오차만큼 손목을 옮겨 다시 (analytic_tip) — 지도와 같은 풀이.
  - 손 돌림: 손가락이 닫히는 축(site x)을 위에서 본 각(손 yaw)을 FK 로 재고, hand_roll(±88°)로 큐브 yaw 에 맞춘다
    (큐브는 90° 마다 같으니 yaw mod 90). 손목 관절이 이미 손을 돌린 만큼은 FK 로 재서 빼므로 자동으로 들어간다.
  - 경유점: HOME(열림) → 큐브 위(집는 점 9 cm 위, 안 되면 1 cm 씩 낮춤) → 내려감(집는 점 = 큐브 중심 + 1 cm) → 손가락 닫기 → 들기(큐브 위)
    → 놓을 자리 위 → 내려감(큐브 바닥 = 책상) → 손가락 열기 → 들기 → HOME.
  - 가지 고르기: 경유점마다 해석 IK 해(가지) × hand_roll 후보 중, 이웃 경유점 사이 관절 최대 이동각의 합이 가장 작은 줄
    (손 IK 의 '최소 회전' 규칙을 경로 전체로).
  - 경유점 사이는 관절 공간 보간, 사다리꼴 시간(가장 빠른 관절이 max_speed °/s, 실물 램프 가속 max_acc °/s² 를 안 넘게), 프레임 간격 dt 0.02 s.
  - 프레임마다 손가락 충돌 상자 가장 낮은 점이 책상 위인지 (lowest_finger_z) 검사, 아니면 상태 'finger below table'.
물리·쥐기 시뮬은 없다: 큐브가 실제로 잡히고 미끄러지지 않는지는 이 계획이 모른다 (팀 시뮬 계획은 '팀 시뮬 계획 (물리)').

출력 JSON 은 pick_place --save 와 같은 꼴 (joint_names 팔 7 + 손가락 6, dt, positions rad, status ...) + planner/waypoints.
종료코드: pick_place 와 같다 — 0 통과, 1 거절(검사), 3 팔이 못 닿음.
"""
import argparse
import contextlib
import json
import math
import threading
import os
import sys
import time
import warnings
from pathlib import Path

warnings.filterwarnings("ignore", message=".*reachmap.*")   # team env 가 import 때 내는 도달 맵 경고 (이 계획은 도달 맵을 안 씀)

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
sys.path[:0] = [str(HERE), str(REPO / "arm_control"), str(REPO / "stm_bridge")]   # pick_place 로컬 실행과 같게
from hand_ik import GRIP_ABOVE_CENTER, D_RANGE, d_range_text   # noqa: E402  집는 점 = 큐브 중심 + 이것 (지도·계획·IK 탭 한 값), 책상 높이 범위

ABOVE_CM = (9.0, 7.0, 5.0, 4.0, 3.0)   # 큐브·놓을 자리 위 경유점: 집는 점을 이만큼 (3.25 cm 미만이면 벌린 손가락 끝이 큐브 윗면보다 낮다:
                                  # 그때는 손가락이 큐브 양옆으로 곧게 내려가야 한다 — 손가락↔큐브 검사가 본다)
                                  # 4.0 (2026-11-24): 3층 놓을 자리는 위로 4~5.5 cm 까지만 빠진다 — 5 가 안 되면 3 (손끝이 놓은 큐브 윗면 아래) 대신 4
APPROACH_TILTS = (0, 15, 30, 45, 60)   # 그 경유점 방향: 수직에서 기울기(°) × 방위 45° 마다
APPROACH_AZ = 45
STEP_TRANSIT = 0.01         # 옮겨 가는 길: 손목 중앙 1 cm 마다 IK
STEP_DESCENT = 0.005        # 내려가기·들기: 집는 점 0.5 cm 마다 IK (그 사이는 관절 보간). 1 cm 이면 손목 2 관절이 한계에 닿는 곳에서
                            # 한 걸음에 팔이 15° 넘게 움직여 '가지 튐' 으로 버려졌다 (2026-11-21 지도 판정과 같이 0.5)
FALLBACK_COST = 10.0        # 길을 못 찾아 관절 직선으로 가는 구간의 벌점 (rad, 고르기에서 뒤로)
TRANSIT_LIFTS = (0.0, 0.03, 0.06, None)   # 옮겨 가는 길의 높이: max(두 끝, 하한) + 이만큼 (None = 곧은 선)
CLEAR_CM = 3.0               # 옮겨 갈 때 손가락 끝이 큐브 윗면보다 이만큼 위 (손목 높이 하한)
JUMP_DEG = 15.0             # 그 사이 팔 관절이 이보다 크게 튀면 가지가 끊긴 것 (그 길 버림)
SAME_BRANCH_DEG = 2.0       # 길 끝 해가 목표 자세와 같은 가지인가 (팔 3 관절 차이)
MAX_TRIES = 100               # 손가락이 큐브에 닿지 않는 접근을 찾을 때 볼 후보 수 (집기·놓기 각각)
DT = 0.02                    # 프레임 간격 (팀 계획과 같은 50 Hz)
GRIP_S = 0.8                 # 손가락 닫기·열기 시간 (팀: 한 스텝 0.025 씩 → 40 스텝 = 0.8 s)
GRIP_HOLD_S = 0.2            # 닫은/연 뒤 기다림
YAW_TOL = 2.0                # 손 yaw 가 큐브 면에서 이 각(°) 안
TIP_TOL = 0.005              # 집는 점 오차 5 mm (지도와 같은 TOL_POS)
TRACK_LAG_DEG = 2.0          # 실물 램프(real_arm)가 순항 중 늦게 따라가는 각 상한 → 순항 속도 √(2·max_acc·이것) = 19 °/s (max_acc 90)
try:   # 실물 램프 가속(°/s²) — real_arm.DEFAULTS 한 곳에서 읽는다 (계획 프레임을 실물이 따라갈 수 있게)
    from real_arm import DEFAULTS as _REAL
    MAX_ACC = float(_REAL["max_acc"])
except Exception:   # real_arm 을 못 읽는 곳(팀 PC 등)에선 보수적으로 같은 기본값 — 경고를 남긴다
    MAX_ACC = 90.0
    print("pick_plan: real_arm.DEFAULTS 를 못 읽어 MAX_ACC 90 °/s² 가정", file=sys.stderr)
PICK_DROP_M = 0.0            # 0 (2026-11-25 오후): '손이 8 cm 위에서 멈춤' 의 원인 = 실물 책상이 어깨 아래 0.557 m (기본 0.4775 보다 8 cm 낮음, 폰 IMU·큐브
                             # 마커로 잼) → 책상 높이 d 를 실제 값으로 넣으면 집기·놓기·HOME 이 모두 맞는다. 둘을 같이 쓰면 16 cm 내려간다.
                             # (예전 0.08: 실물에서 손이 큐브보다 약 8 cm 위에서 멈춤 → 집는 자리의 큐브를 모델 책상보다 이만큼 아래로 보고
                             # 집기·들기(큐브 위 → 내려감 → 잡기 → 들기)만 그만큼 낮게 계획한다. 놓기·옮겨 가는 높이(z_clear)·HOME 은 그대로.
                             # 책상 충돌 검사 바닥도 이만큼 내린다 (안 그러면 낮춘 잡는 자세가 '책상 아래'로 거절). --pick-drop 으로 바꿈 (0 = 예전)
TABLE_MARGIN = 0.008         # 손가락 충돌 상자·들고 가는 큐브 바닥이 책상 면보다 이만큼(m) 위여야 (실물 책상 높이 MAP_D 0.4775 는 가정 —
                             # 실측 전엔 오차가 이 여유를 먹는다. README). 집는 점 = 큐브 중심이던 때는 수직 손에서 쥔 손가락 8.5 mm 위라
                             # 손이 조금만 기울어도 못 지켰다 → 2026-11-21 집는 점 +1 cm (GRIP_ABOVE_CENTER, 수직이면 18.5 mm 위)
FINGER_OPEN_J1_DEG = -60.0  # 벌린 손가락 첫 마디 각 (°). None = env FINGER_OPEN (−15°, 쥠 −11° 와 4° 차이). 2026-11-23 사용자: 손이 뒤집힌 채 내려가 손가락이 걸리면 더 벌림 → plan_check 25·27/30 (−15° 는 24·23, −30·−45 는 25·24). 실물 손가락이 −60° 까지 가는지 미확인
YAW_SAMPLES = tuple(range(0, 90, 15))   # 지도 판정에서 볼 큐브 yaw (sim °, 90° 마다 같음)
VERT_TILT = 90.0            # ° 지도: 잡는 자세 손이 이보다 기울면 '못 집음' — 2026-11-21 집는 점 +1 cm 부터 사실상 끔 (손가락 책상 여유
                            # _roll_best 가 판정). 예전(집는 점 = 큐브 중심) 0.5: 수직이어도 쥔 손가락 8.5 mm 라 1° 만 기울어도 8 mm 를 못 지켰다.
                            # +1 cm 면 수직 18.5 mm 위라 8~15° 기운 칸도 통과 (plan_check 로 확인, README)
ELBOW_BRANCH = 1            # 계획·지도가 쓰는 팔꿈치 가지 (elbow 부호). 2026-11-23 사용자: 위팔 비틀기 ±90 (calibration upper_arm_roll [-90, 90])
                            # → 위팔 0 근처 + elbow > 0. 예전(−1) 은 위팔 ≈180 가지라 HOME 에서 매번 곧은 팔로 0→180 먼저 돌렸다
# HOME = 모든 관절 0 (팔꿈치가 이 각 안 = 팔이 곧게 늘어짐) 이면 HOME 에서 나가고 들어오는 관절 경유점 (Planner._home_exit, 2026-11-21).
# 값은 경유점 격자 검색으로 고름 (shoulder_roll −45..0, elbow −90..−135, wrist ±45·90 중 관절 이동 합이 작고 손가락이 책상 발자국 위에서
# 책상 면 + 41.6 mm 이상): elbow −120 이면 이동이 10° 짧지만 여유 21~25 mm 라 이것. elbow −130 은 한계 ±135 에 5° 남음 (실물 끝 간섭 미확인, calibration note).
CUBE_EDGE_IN = 0.01          # 큐브(놓을 자리 포함)는 책상 발자국 안쪽으로 반 변 + 이만큼 (env CUBE_X_MAX 와 같은 1 cm, 2026-11-21)
HOME_STRAIGHT_DEG = 5.0
HOME_EXIT_MID_DEG = {"shoulder_roll": 0.0, "upper_arm_roll": 0.0, "elbow": 90.0, "wrist_pitch": 0.0, "wrist_yaw": 0.0, "hand_roll": 0.0}   # 2026-11-24 사용자: 손목 U자(90) 말고 ㄴ자(0) — 책상 여유 63 → 43 mm, 기둥 52 mm 그대로
# ↑ 2026-11-23 사용자: 어깨만 먼저 젖히지 말고 위팔 비틀기로 — ① HOME_EXIT_PRE_DEG 위팔 +90·팔꿈치 90·어깨 −30 을 한꺼번에 (팔꿈치가 옆으로 접혀
#   손이 책상 위를 안 지남, 어깨 부담 작음) → ② 여기: 위팔 0 으로 되돌려 손을 책상 위 172 mm 에서 앞으로 → ③ 책상 위 손목 자리 (79 mm). 기둥 52 mm
# ↑ 2026-11-23 사용자 자세 (팔꿈치 ±100): 어깨 뒤로 −45 + 팔꿈치 90 + 손목 굽힘 90 = 손이 책상 위 224 mm·모서리 바깥. 그 전에 곧은 팔로 어깨만 −45 (HOME_EXIT_PRE_DEG)
#   — 한꺼번에 가면 손이 책상 높이 아래일 때 팔꿈치가 접혀 아래팔이 책상 밑으로 (−201 mm). 검색: 책상 여유 34.5 mm, 기둥 52 mm
# ↑ 2026-11-23 팔꿈치 ±100 (사용자, JAX 초기값 CAD V3): 2026-11-21 ±100 격자 검색 값(−30/180/−95/85/85)을 위팔 ±90 가지로 거울
# ↑ 2026-11-23: 위팔 ±90 가지(ELBOW_BRANCH +1) 로 아래 값을 거울로 (위팔 −180, elbow·wrist_yaw 부호) — 격자 검색에서 책상 여유 47.4 mm, 기둥 52.3 mm. 아래 설명은 예전 −1 가지
# ↑ 2026-11-21: 위팔을 곧은 팔로 먼저 180° 돌린 뒤(HOME_EXIT_TURN_FIRST) 어깨 젖히고 팔꿈치 접기. 기둥 검사(COLUMN_BOXES)를 넣고 격자 검색
#   (shoulder_roll −40..40, elbow −130..−60, wrist ±85; 관절 한계에서 5° 이상) — 책상 여유 46.3 mm, 기둥 여유 52.3 mm (HOME 자세 그대로), HOME→책상 위 19.6 s.
#   위팔을 안 먼저 돌리는 길(팔꿈치를 굽힌 채 위팔 0→180)은 앞팔이 기둥 쪽으로 휩쓸어 전부 기둥을 판다 (잠깐 쓴 elbow −40·wrist −30/−30 은 서보가 기둥 40 mm 안,
#   그 전 값 elbow −130·wrist −45/90 은 손가락이 기둥 6 mm 안).
HOME_EXIT_PRE_DEG = {"upper_arm_roll": 90.0, "elbow": 90.0, "shoulder_roll": -30.0}   # 첫 경유점 (HOME 에서 이 관절들을 한꺼번에)
HOME_EXIT_TURN_FIRST = False   # True: 먼저 upper_arm_roll 만 180 (곧은 팔 제자리 돌림) 경유점 — 위팔 ±90 가지(2026-11-23)에선 필요 없음
# 기둥 (IK arm.xml 의 column 메시를 상자로, sim 월드 m): 세로 기둥과 위 가로대, 기둥에 붙은 Holder. 바닥 받침(높이 8 cm)은 팔이 못 닿아 뺌.
COLUMN_BOXES = (((0.30, -0.19, 0.0), (0.38, -0.11, 1.13)),
                ((0.30, -0.29, 1.13), (0.38, 0.0, 1.17)),
                ((0.194, -0.198, 1.027), (0.30, -0.10, 1.292)))
COLUMN_MARGIN = 0.02           # 팔·손 메시 꼭짓점이 기둥 상자에서 이만큼(m) 떨어져야 (HOME 자세가 52 mm)
COLUMN_VOXEL = 0.008           # 기둥 검사 꼭짓점을 이 격자(m)로 솎음 (한 자세 12 ms → 약 1 ms)
HOME_EXIT_SEED_DEG = {"shoulder_roll": -45.0, "upper_arm_roll": 0.0, "elbow": 60.0, "wrist_pitch": 0.0, "wrist_yaw": 90.0, "hand_roll": 0.0}   # 책상 위 손목 자리 해 고르기 기준
# ↑ 책상 위 손목 자리 해를 고를 때 가까운 쪽 기준 (예전 경유점 값 그대로 → 그 자리 해 = 10.4/0/180/−133/0/90/0 이 예전과 같다)
HOME_EXIT_WRIST_X = 0.0      # 그다음 경유점: 손목 중앙 sim x (책상 위), 높이 z_clear + HOME_EXIT_LIFT (2026-11-23 검색 값)
HOME_EXIT_WRIST_DY = -0.05   # 그 경유점 sim y = HOME 손목 y + 이만큼
HOME_EXIT_LIFT = 0.0        # 그 경유점 손목 높이 = z_clear + 이만큼. 0 이면 책상 앞 끝 가까이(실좌표 y −0.18) 놓은 큐브를 빠질 때 ③→② 에서
                             # 손가락이 건드렸다 (plan_check 3/30). 0.07 은 손목이 안 닿음. 0.04 면 그 해 elbow −133° (한계 135 에 2° 남음)
EXIT_SHIFTS = tuple((dx, dy) for dx in (0.0, 0.03, 0.06) for dy in (-0.09, -0.06, -0.03, 0.0, 0.03, 0.06, 0.09) if (dx, dy) != (0.0, 0.0))
# ↑ 탑 쌓기(다른 큐브 위에 놓기)가 기본 경유점으로 안 될 때만: 경유점 손목 sim (x, y) 를 이만큼 옮겨 본다 (2026-11-24, 3층. 풀리는 것만, exit_ok 검사 그대로)
EXIT_SHIFT_GAIN = 0.02      # 그중 탑 자리에서 지금보다 이만큼(m) 넘게 먼 것만
MAP_SLACK = 0.0003       # m 지도 판정의 책상 여유 덤: hand_roll 만 돌린 근사·중간 프레임 차이 (계획은 덤 없이 TABLE_MARGIN)
MAP_TILTS = (15, 30)        # 지도가 보는 비스듬한 내려가기 (수직에서 °, APPROACH_TILTS 의 앞쪽). 계획은 45·60 도 보지만 지도는 시간 때문에 여기까지
# 집는 방향 (2026-11-24): 자동 = 수직 → 수평 → 기울임(예전 방식: 손목이 되는 만큼 아래) 순서로 되는 것, 수직·수평 = 그것만
GRASP_MODES = ("auto", "vertical", "horizontal", "tilted")   # tilted = 기울임(어제 방식, 2026-11-24 사용자): 손목 ±90 안에서 예전 기울여 집기만
TILT_WRIST = ("wrist_pitch", "wrist_yaw", "hand_roll")   # 기울임 모드에서 ±90 으로 묶는 관절 (11-23 실물에서 성공한 계획과 같은 한계)


def tilt_ranges(ranges):
    """기울임(어제 방식) 모드의 관절 한계: 손목 3 관절을 ±90 안으로 (보낼 수 있는 범위와 겹치는 부분). 나머지는 그대로."""
    out = {k: list(v) for k, v in (ranges or {}).items()}
    for n in TILT_WRIST:
        lo, hi = out.get(n, [-90.0, 90.0])
        out[n] = [max(float(lo), -90.0), min(float(hi), 90.0)]
    return out
STYLE_KO = {"vertical": "수직", "horizontal": "수평", "tilted": "기울임", "auto": "자동"}
VERT_MAX_TILT = 5.0         # ° 수직: 잡는 자세와 내려가기·들기 내내 손 축이 수직에서 이 안
H_TOL = 5.0                 # ° 수평: 손 축이 밀어 넣는 방향(큐브 면 법선, 수평)에서 이 안 + 손가락 닫히는 축이 수평에서 이 안
H_PUSH_CM = (5.0, 6.0, 4.0)   # 수평: 큐브 옆 같은 높이에서 손 축 따라 이만큼 곧게 밀어 넣음 (팀 RL 정책 4.5 cm)
H_LIFT_CM = (9.0, 7.0, 5.0, 3.0)   # 수평: 잡은 뒤 곧게 위로 (손 수평 그대로), 그 위는 손목 길(transit)
H_GRIP_ABOVE = 0.018        # 수평: 집는 점 = 큐브 중심 + 이만큼 (m) = 책상 위 4.3 cm. 누운 손은 손가락 상자가 집는 점 아래로 35.8 mm(손가락 1 이 아래)·39 mm
                            # (손가락 2 가 아래) 내려온다 (손 세로 폭 75 mm > 큐브 50 mm) → 큐브 중심 높이(2.5 cm)로는 손가락이 책상 속. 4.3 cm 면 4~7 mm 위.
                            # 손목도 같은 높이라 팔 길이(어깨↔손목 46 cm)가 빠듯: 기본 책상(d 0.4775)에서 손목이 어깨 바로 아래 반지름 약 15 cm 안일 때만 닿는다
H_TABLE_MARGIN = 0.003      # 수평: 손가락 충돌 상자가 책상 면보다 이만큼(m) 위 — 책상 검사(hand_ik.NO_TABLE)를 꺼도 지킨다
STACK_GAP = 0.002           # 탑 쌓기(--place-level ≥ 1): 큐브 바닥을 아래 큐브 윗면보다 이만큼(m) 위에서 놓는다 (IK 오차로 누르지 않게)
OBST_MARGIN = 0.008         # 다른 큐브(--obstacle)와 팔·손 메시 꼭짓점 사이 여유 (m, 책상 여유 TABLE_MARGIN 과 같은 값)
OBST_CARRY_MARGIN = 0.005   # 들고 가는 큐브 ↔ 다른 큐브 여유 (m). 내려놓는 구간(놓을 자리 위 → 놓기)은 파고들지만 않으면 됨
REASON = {1: "손목은 닿지만 손끝(집는 점)을 큐브에 못 맞춤", 0: "팔이 안 닿음"}   # hand_ik.graspcheck 와 같은 말


def _trap(t, T, L, v, acc):
    """사다리꼴 속도(최대 v, 가속 acc, 총 시간 T, 거리 L)로 t 초에 간 거리. 짧으면 삼각형(최대 속도 √(L·acc))."""
    vp = min(v, math.sqrt(L * acc))
    ta = vp / acc
    if t <= 0:
        return 0.0
    if t >= T:
        return L
    if t < ta:
        return 0.5 * acc * t * t
    if t > T - ta:
        r = T - t
        return L - 0.5 * acc * r * r
    return 0.5 * acc * ta * ta + vp * (t - ta)


def _cmd_ranges(cal_file):
    """calibration.json -> {관절: [lo, hi] sim °} 보낼 수 있는 범위 (server._cmd_range 와 같은 계산)."""
    d = json.loads(Path(cal_file).read_text(encoding="utf-8"))
    out = {}
    for j in d.get("joints") or []:
        try:
            lo, hi = (float(v) for v in j["sim_range_deg"])
            if j["motor"] == "servo":
                mlo, mhi = 0.0, 180.0
            else:
                mlo, mhi = j["zero_deg"] - j["limit_deg"], j["zero_deg"] + j["limit_deg"]
            a, b = sorted(((mlo - j["zero_deg"]) / (j["sign"] * j["scale"]), (mhi - j["zero_deg"]) / (j["sign"] * j["scale"])))
            r = [max(lo, a), min(hi, b)]
            if r[0] <= r[1]:
                out[j["joint"]] = r
        except Exception:
            pass
    return out


def _find_ik(path):
    for c in (path, os.environ.get("JAX_IK_DIR"), REPO.parent / "IK", Path.home() / "IK"):
        if c and (Path(c) / "motion1" / "src" / "humanoid_grasp_rl").is_dir():
            return Path(c).resolve()
    raise FileNotFoundError("team-jax/IK 를 못 찾음 (--ik 또는 JAX_IK_DIR)")


class Planner:
    def __init__(self, ik_dir, ranges, solver=None, setup=None, d=None, grasp="auto"):
        """solver/setup: 지도(hand_ik worker)가 자기 _Solver 와 grasp_setup 결과로 이 계획기의 집기 조건(point_check)을 쓴다.
        d: 책상 높이 (어깨 축 아래 m, None = 기본 MAP_D; hand_ik.D_RANGE 밖이면 ValueError). solver 를 받으면 그 solver 의 지금 높이.
        책상 기준 높이(z_clear, HOME 나가기 경유점)를 여기서 정하므로 계획기 하나 = 책상 높이 하나 (self.table_d)."""
        if solver is None:
            import hand_ik
            solver = hand_ik._Solver(str(ik_dir))
            solver.set_table_d(d)
        if grasp not in GRASP_MODES:
            raise ValueError(f"집는 방향은 {' / '.join(GRASP_MODES)} 중 하나 ({grasp!r})")
        self.grasp = grasp   # 집는 방향 (auto / vertical / horizontal): point_check 가 이 방식으로 본다. 계획기의 다른 상태는 방식과 상관없다
        self.pick_drop = 0.0   # 집기·들기만 낮춤 (m): plan() 이 PICK_DROP_M 으로 바꾼다. 지도·탑 쌓기 판정(이 계획기를 직접 쓰는 곳)은 0
        self.s = s = solver
        self.table_d = float(s.table_d)
        self.np = np = s.np
        self.S = setup if setup is not None else s.grasp_setup(ranges, 0.0)   # 지도와 같은 한계 (shoulder_yaw 0 고정)
        self.lim = self.S["lim"]
        self.hr = s.arm.index("hand_roll")
        self.open = np.asarray(s.env.finger_targets(0.0), dtype=float)
        self.open_exit = self.open.copy()    # HOME 나가기·들어오기 경유점 길의 벌림 = env 그대로(좁게): 손이 책상 앞 모서리 옆을 지날 때 덜 튀어나옴 (2026-11-23)
        if FINGER_OPEN_J1_DEG is not None:   # 내려가고 빠질 때 손가락을 더 벌림 (첫 마디 = 각 손가락 joint1, 6 개 중 0·2·4)
            self.open[[0, 2, 4]] = math.radians(FINGER_OPEN_J1_DEG)
        self.closed = np.asarray(s.env.finger_targets(1.0), dtype=float)   # 팀 계획이 들고 갈 때 쓰는 값 (grip 1.0)
        # HOME = 모든 관절 0 (실물 원점 = 팔이 곧게 늘어진 자세, 대시보드 'HOME 으로 가기'·README). IK 레포의 env HOME_ARM_QPOS 는
        # sim 의 손바닥-아래 자세라 0 이 아니다(팀 최신 IK 에선 0) — 그걸 그대로 쓰면 HOME 이 위팔 143°·손목 −90° 가 되어
        # 관절 한계·기둥 검사에 걸리고 지도 초록 칸이 0 이 된다 (2026-11-24). HOME_FROM_IK=True 면 예전처럼 env 값.
        self.home = np.asarray(s.home_arm, dtype=float)   # hand_ik._Solver.home_arm (JAX_HOME_FROM_IK=1 이면 env 값)
        self.home0 = self.home.copy()
        self.home0[s.arm.index("shoulder_yaw")] = 0.0   # 해석 IK 는 shoulder_yaw 0 고정 → 계획은 HOME 에서 먼저 그것만 0 으로
        self.cube_grip = float(getattr(s.E, "CUBE_GRIP", 1.0))
        # 옮겨 갈 때 손목 높이 하한: 손을 수직으로 내려도 벌린 손가락 끝이 큐브 윗면(5 cm) + CLEAR_CM 위
        self.z_clear = s.table_z + 0.05 + CLEAR_CM / 100 + (s.wrist_above_site - s.finger_below_site)
        self.obj_body = int(s.env.object_body_ids[0])
        self.cube_geom = [g for g in range(s.m.ngeom) if s.m.geom_bodyid[g] == self.obj_body][0]
        self.fgeoms = set(s.finger_geoms)   # 충돌하는 손 상자는 손가락 6 개뿐 (손바닥·서보 메시는 충돌 없음)
        self.arc_c = self.wrist(self.home0)[:2].copy()   # 호 길(_arc_pts) 중심 = 곧게 늘어진 팔의 손목 = 어깨 바로 아래
        self._column_init()
        self._home_exit()

    def _home_exit(self):
        """HOME 에서 옮겨 가는 길의 시작 (self.exit = HOME0 부터의 관절 경유점, 끝 = 손목 길(transit)의 시작 자세).
        예전 HOME(손이 책상 앞 끝 위)은 HOME0 에서 바로 transit (손목 2 관절을 HOME 값에서 섞음, exit_fh True).
        HOME = 모든 관절 0 (2026-11-21, 팔이 책상 옆에 곧게 늘어짐 = 팔꿈치 0 특이점) 이면 손목 길로는 못 나간다:
        그 자리에서 손목을 곧게 올리면 shoulder_roll 한계(45°)로 약 20 cm 까지만 오르고(손가락은 아직 책상 면 아래),
        팔꿈치를 곧장 구부려 계획 가지(upper_arm_roll ≈ 180, elbow < 0)로 가면 앞팔이 책상 쪽으로 돌며 손가락이 책상 모서리를
        23 cm 파고든다 (관절 직선 검사). 그래서 관절 경유점 셋 (HOME_EXIT):
          1) upper_arm_roll 0 → 180 만 (팔이 곧아 손이 제자리에서 돈다, 책상 옆)
          2) HOME_EXIT_MID: 어깨를 뒤로(shoulder_roll −40) 젖히며 팔꿈치 −130 + 손목으로 손을 아래로 → 손이 책상 옆에서 위로
          3) 손목 중앙 (HOME_EXIT_WRIST_X, HOME 손목 y, z_clear) 의 해석 IK 해 (계획 가지, 손 최대한 아래) → 거기서 손목 길
        각 구간은 따로 사다리꼴 (모서리에서 멈춤). 이 경유점은 계획기를 만들 때 프레임처럼 촘촘히 검사 (exit_clear_mm)."""
        np, s = self.np, self.s
        ie = s.arm.index("elbow")
        self.exit, self.exit_fh, self.exit_ok, self.exit_clear_mm, self.exit_why = [self.home0.copy()], True, True, None, None
        if abs(float(self.home0[ie])) > math.radians(HOME_STRAIGHT_DEG):
            return
        h1 = self.home0.copy()
        if HOME_EXIT_PRE_DEG:
            for n, v in HOME_EXIT_PRE_DEG.items():
                h1[s.arm.index(n)] = math.radians(v)
        else:
            h1[s.arm.index("upper_arm_roll")] = math.radians(HOME_EXIT_MID_DEG["upper_arm_roll"])
        mid = self.home0.copy()
        for n, v in HOME_EXIT_MID_DEG.items():
            mid[s.arm.index(n)] = math.radians(v)
        seed = self.home0.copy()
        for n, v in HOME_EXIT_SEED_DEG.items():
            seed[s.arm.index(n)] = math.radians(v)
        w = self.wrist(self.home0)
        dx, dy = getattr(self, "exit_shift", (0.0, 0.0))   # 탑 쌓기만 (plan 이 줌): 경유점을 탑에서 먼 쪽으로
        sols, _ = s.analytic_position(np.array([HOME_EXIT_WRIST_X + dx, w[1] + HOME_EXIT_WRIST_DY + dy, self.z_clear + HOME_EXIT_LIFT]), seed, self.lim)
        sols = [c for c in sols if (c[ie] < 0) == (ELBOW_BRANCH < 0)]
        if not sols:
            self.exit_ok = False
            self.exit_why = "HOME 에서 나가는 경유점(책상 위 손목 자리)을 못 풂"
            return
        e = min(sols, key=lambda c: float(np.abs(c - seed).max()))
        self.exit, self.exit_fh = ([self.home0.copy(), h1, mid, e] if (HOME_EXIT_TURN_FIRST or HOME_EXIT_PRE_DEG) else [self.home0.copy(), mid, e]), False
        low = 9.0
        for a, b in zip(self.exit, self.exit[1:]):
            for k in range(81):
                q = a + (b - a) * (k / 80)
                low = min(low, self.finger_low(q, self.open_exit) - s.table_z, self.arm_low(q) - s.table_z + TABLE_MARGIN)
        col = min(self.column_low(self.exit[i] + (self.exit[i + 1] - self.exit[i]) * (k / 80), self.open_exit)
                  for i in range(len(self.exit) - 1) for k in range(81))
        self.exit_clear_mm = round(low * 1000, 1)
        self.exit_column_mm = round(col * 1000, 1)
        self.exit_ok = low >= TABLE_MARGIN and col >= COLUMN_MARGIN
        self.exit_why = (None if self.exit_ok else
                         f"HOME 에서 나가는 경유점 길에서 손가락이 책상 위 {low * 1000:+.1f} mm (여유 {TABLE_MARGIN * 1000:.0f} mm)" if low < TABLE_MARGIN else
                         f"HOME 에서 나가는 경유점 길에서 팔이 기둥에서 {col * 1000:+.1f} mm (여유 {COLUMN_MARGIN * 1000:.0f} mm)")

    def exit_shifts_away(self, target_sim, z, n=2):
        """탑 쌓기: HOME 나가기 경유점(손목 sim x, y)을 EXIT_SHIFTS 만큼 옮긴 것 중 풀리고(exit_ok) 탑 자리(target_sim xy)에서 지금보다
        EXIT_SHIFT_GAIN 넘게 먼 것, 먼 순서로 n 개. 옮겨 가는 높이 z (z_clear) 기준으로 보고, 끝나면 경유점·z_clear 를 원래대로 돌려 놓는다."""
        np = self.np
        old, oz = getattr(self, "exit_shift", (0.0, 0.0)), self.z_clear
        self.z_clear = z
        dist = lambda: float(np.hypot(*(self.grip(self.exit[-1])[:2] - np.asarray(target_sim)[:2])))   # noqa: E731
        self.exit_shift = (0.0, 0.0)
        self._home_exit()
        d0 = dist() if self.exit_ok else 0.0
        out = []
        for sh in EXIT_SHIFTS:
            self.exit_shift = sh
            self._home_exit()
            if self.exit_ok and dist() > d0 + EXIT_SHIFT_GAIN:
                out.append((dist(), sh))
        self.exit_shift, self.z_clear = old, oz
        self._home_exit()
        return [sh for _, sh in sorted(out, reverse=True)[:n]]

    def _column_init(self):
        """기둥 검사용: 관절에 따라 움직이는 팔·손 메시 geom 과 그 꼭짓점 (geom 틀). 기둥·Holder·base_link 처럼 안 움직이는 것은 뺀다."""
        s, np = self.s, self.np
        m, d = s.m, s.d
        mesh = [g for g in range(m.ngeom) if int(m.geom_type[g]) == 7]
        s.fk(self.home0)
        a = {g: d.geom_xpos[g].copy() for g in mesh}
        q = self.home0.copy() + 0.3
        s.fk(q)
        self.col_geoms = [g for g in mesh if float(np.linalg.norm(d.geom_xpos[g] - a[g])) > 1e-6]
        self.col_verts = {}
        for g in self.col_geoms:
            mid = int(m.geom_dataid[g])
            a0, n = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
            v = m.mesh_vert[a0:a0 + n].copy()
            _, keep = np.unique(np.round(v / COLUMN_VOXEL), axis=0, return_index=True)   # 꼭짓점을 격자로 솎음 (빠르게; 여유 20 mm 가 덮음)
            self.col_verts[g] = v[np.sort(keep)]
        self.col_lo = [np.array(lo) for lo, _ in COLUMN_BOXES]
        self.col_hi = [np.array(hi) for _, hi in COLUMN_BOXES]

    def column_low(self, q, f=None):
        """팔·손 메시 꼭짓점과 기둥 상자 사이 가장 짧은 거리 (m, 안에 들어가면 음수)."""
        s, np = self.s, self.np
        d = s.d
        if f is not None:
            d.qpos[s.fadr] = f
        s.fk(q)
        V = np.concatenate([self.col_verts[g] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g] for g in self.col_geoms])
        best = 9.0
        for lo, hi in zip(self.col_lo, self.col_hi):
            out = np.maximum(np.maximum(lo - V, V - hi), 0.0)
            dist = np.sqrt((out * out).sum(axis=1))
            inside = (dist == 0.0)
            if inside.any():
                pen = np.minimum(V[inside] - lo, hi - V[inside]).min(axis=1)
                best = min(best, -float(pen.max()))
            best = min(best, float(dist.min()))
        return best

    def exit_frames(self, fa, sp, back=False):
        """HOME0 ↔ 손목 길 시작 경유점 프레임 (구간마다 따로 사다리꼴). back: 거꾸로 (손목 길 끝 → HOME0)."""
        qs = self.exit[::-1] if back else self.exit
        out = []
        for a, b in zip(qs, qs[1:]):
            out += self.frames([a, b], fa, fa, sp)
        return out

    # ---- 지도(집을 수 있는 곳)의 판정 = 이 계획기가 그 자리에서 집을 수 있나 (2026-11-21, 두 판정이 어긋나지 않게 여기 한 곳)
    def point_check(self, pc, yaws=None, style=None):
        """큐브 중심 pc (sim) 를 집을 수 있나 — (됨?, 이유, 잡는 자세 손 기울기 °). style: "vertical"(손 축 수직 VERT_MAX_TILT 안) /
        "horizontal"(옆에서 밀어 넣기) / "tilted"(예전 방식: 손목이 되는 만큼 아래), None = 이 계획기의 집는 방향(self.grasp):
        auto = 기울임(수직을 포함한다)이 되면 됨, 안 되고 yaws 를 받았으면 그 방향의 수평도 본다 (지도처럼 yaws 가 없으면 수평은 안 본다:
        수평은 큐브 면이 어깨 쪽을 볼 때만 되어 '모든 yaw' 로는 초록이 없다)."""
        if style is None:
            if self.grasp != "auto":
                style = self.grasp
            else:
                r = self._pc_down(pc, yaws)
                if r[0] or yaws is None:
                    return r
                rh = self._pc_h(pc, yaws)
                return rh if rh[0] else r
        if style == "horizontal":
            return self._pc_h(pc, yaws)
        return self._pc_down(pc, yaws, vertical=style == "vertical")

    def _pc_h(self, pc, yaws=None):
        """수평 집기 판정 (point_check): 큐브 yaw 마다 candidates_h 의 자세 하나라도 — 옆 H_PUSH_CM 에서 곧게 밀어 넣는 길 + 곧게 드는 길
        (손 수평, 손가락 책상 위 H_TABLE_MARGIN) + 벌린 손가락이 큐브에 안 닿음 + HOME 에서 큐브 옆으로 옮겨 오는 길.
        yaws 를 받으면 그 방향 모두, None(지도) 이면 YAW_SAMPLES 중 하나라도 되면 됨 ('큐브를 알맞게 돌려 두면 집을 수 있는 자리')."""
        if not self.on_table(pc):
            return False, self.off_table_why(), None
        if not self.exit_ok:
            return False, self.exit_why, None
        any_ok = yaws is None
        samples = tuple(YAW_SAMPLES) if any_ok else tuple(sorted({round(float(y) % 90.0, 3) for y in yaws}))
        why = None
        for yaw in samples:
            ok, w = self._h_yaw_ok(pc, yaw)
            if ok and any_ok:
                return True, "집을 수 있음 (수평, 큐브 면이 팔 쪽을 볼 때)", 90.0
            if not ok and not any_ok:
                return False, w, None
            why = why or w
        return (False, why or "수평으로 못 집음", None) if any_ok else (True, "집을 수 있음 (수평)", 90.0)

    def _h_yaw_ok(self, pc, yaw):
        C = self.candidates_h(pc, yaw)
        if not C:
            return False, ("수평: 이 큐브 방향에선 옆에서 손을 댈 수 있는 면이 없음 (손목이 집는 점 높이까지 내려가야 해서 어깨 바로 아래 "
                           "가까이에서만 — 큐브 면이 어깨 쪽을 보는 자리만 된다)")
        why = None
        for qg, a in C:
            for cm, lift in ((c, h) for c in H_PUSH_CM for h in H_LIFT_CM):
                up, w = self.h_lift(pc, qg, a, lift)
                A, w = (None, w) if up is None else self.h_push(pc, qg, a, cm, lift)
                if A is None:
                    why = why or w
                    continue
                t1 = self.transit(self.exit[-1], A[0], f_home=self.exit_fh, t_home=True)
                if t1 is None:
                    why = why or "수평: HOME 에서 큐브 옆 위로 옮겨 가는 길이 없음 (팔 가지·도달 범위)"
                    continue
                t3 = self.transit(A[0], self.exit[-1], f_home=True, t_home=self.exit_fh)   # 놓고 빠져 돌아가는 길 (놓을 자리로 쓸 때)
                if t3 is None:
                    why = why or "수평: 큐브 옆 위에서 HOME 쪽으로 돌아가는 길이 없음"
                    continue
                mid = lambda qs: [qs[0]] + [x + (y - x) * u for x, y in zip(qs, qs[1:]) for u in (0.5, 1.0)]   # noqa: E731  프레임처럼 사이도
                dense = mid(t1) + mid(A) + mid(t3)
                if self.cube_hits([(q, self.open) for q in dense], range(len(dense)), pc, yaw):
                    why = why or "수평: 큐브 옆으로 가서 밀어 넣는(빠지는) 길에서 벌린 손가락이 큐브에 닿음"
                    continue
                if min(self.hand_floor(q, self.open) for q in dense) < H_TABLE_MARGIN:
                    why = why or "수평: 큐브 옆 위로 옮겨 오는 길에서 손가락이 책상에 닿음"
                    continue
                return True, None
        return False, why or "수평으로 못 집음"

    def _pc_down(self, pc, yaws=None, vertical=False):
        """위에서 집기 판정 (point_check 의 수직·기울임). vertical: 잡는 자세와 곧게 올리는 길 내내 손 축이 수직에서 VERT_MAX_TILT 안,
        내려가는 방향도 수직만. 아래는 예전 point_check 설명 그대로 —
        큐브 중심 pc (sim) 에서 이 계획기의 집기(=놓기·빠지기) 조건을 싸게: (됨?, 이유, 잡는 자세 손 기울기 °).
        1) 집는 점 = 큐브 중심 + GRIP_ABOVE_CENTER 인 해 (analytic_tip, 모든 가지 → HOME 팔꿈치 가지 우선 = family).
        2) 그 해에서 집는 점을 곧게 위로 min(ABOVE_CM) 까지 (descent 와 같은 한 걸음 _tip_step, 손 최대한 아래, 같은 가지),
           그 위는 손목을 옮겨 갈 높이 z_clear 까지 (transit 의 마지막 다리, 손목 2 cm 마다).
        3) 큐브 yaw 를 YAW_SAMPLES 로 돌려 가며 모두: hand_roll(±88°, 큐브는 90° 마다 같으니 roll 후보 1~2 개)을 맞춘 그 길에서
           잡는 자세 손가락(벌림·쥠) 책상 위 TABLE_MARGIN 이상, 벌린 손가락이 길 내내 책상 여유 위·큐브에 안 닿음.
           hand_roll 만 바꾸고 다시 풀지 않는다 (손 축 둘레라 집는 점이 3 mm 안으로만 움직임 — 계획은 다시 풀어 정확히 검사).
        큐브는 손이 수직이면 손 축 둘레 대칭이라 yaw 와 상관없고, 기울면 yaw 에 따라 아래쪽 손가락이 달라 모든 yaw 를 본다.
        yaws: 볼 큐브 yaw (sim °) — 계획은 실제 큐브·놓을 방향 하나만 (2026-11-23), 지도는 None = YAW_SAMPLES 전부."""
        samples = tuple(sorted({round(float(y) % 90.0, 3) for y in yaws})) if yaws is not None else tuple(YAW_SAMPLES)
        np, s, lim = self.np, self.s, self.lim
        if not self.on_table(pc):
            return False, self.off_table_why(), None
        pg = self.grip_target(pc)
        qs, _ = s.analytic_tip(pg, self.S["q0"], lim, "position", tol=0.002)
        qs = [q for q in qs if float(np.linalg.norm(self.grip(q) - pg)) <= TIP_TOL]
        if not qs:
            return False, f"손끝(집는 점)을 큐브 중심 +{GRIP_ABOVE_CENTER * 100:g} cm 에 못 맞춤", None
        qs = self.family(qs)
        tilts = [s.axis_tilt(s.fk(q)[1]) for q in qs]
        why = None
        n_tip = int(round(min(ABOVE_CM) / 100 / STEP_DESCENT))
        for q, t in sorted(zip(qs, tilts), key=lambda c: c[1]):   # 손이 수직에 가까운 해부터
            if vertical and t > VERT_MAX_TILT:
                why = why or f"수직: 손을 수직으로 못 세움 (가장 곧은 해가 {t:.0f}° 기울어짐, 수직 = {VERT_MAX_TILT:g}° 안)"
                continue
            if t > VERT_TILT or self._roll_best(q) < TABLE_MARGIN + MAP_SLACK:   # 손이 기울면 손가락이 여유 아래 (아래 참고)
                why = why or f"손이 {t:.0f}° 기울어 손가락이 책상 위 {TABLE_MARGIN * 1000:.0f} mm 를 못 지킴"
                continue
            need = set(samples)
            for v in (self._dirs()[:1] if vertical else self._dirs()):
                path, w = self._rise(pg, q, v)
                if path is None:
                    why = why or w
                    continue
                if vertical and max(s.axis_tilt(s.fk(x)[1]) for x in path[:n_tip + 1]) > VERT_MAX_TILT:
                    why = why or f"수직: 곧게 내려가는 동안 손이 {VERT_MAX_TILT:g}° 넘게 기울어짐 (손목 한계)"
                    continue
                got = self._yaws_pass(pc, path, need)
                if not got:
                    continue
                if not self.exit_ok:
                    return False, self.exit_why, min(tilts)
                t1 = self.transit(self.exit[-1], path[min(len(path) - 1, int(round(min(ABOVE_CM) / 100 / STEP_DESCENT)))], f_home=self.exit_fh)
                if t1 is None:
                    why = why or "HOME 에서 이 자리 위로 옮겨 가는 길이 없음 (팔 가지·도달 범위)"
                    continue
                if any(self.finger_low(q, self.open) < self.s.table_z + TABLE_MARGIN + MAP_SLACK or self.arm_low(q) < self.s.table_z for q in t1):
                    why = why or "HOME 에서 옮겨 오는 길에 손가락이 책상 여유 아래 (어깨 가까운 곳: 손을 아래로 못 돌림)"
                    continue
                need -= got
                if not need:
                    return True, "집을 수 있음", t   # 고른 해(마지막으로 yaw 를 채운 해)의 손 기울기 — 지도 색 (2026-11-24)
            if need != set(samples):
                why = ("큐브 yaw " + ", ".join(f"{y - s.R.SIM_FROM_REAL_YAW:+.0f}°" for y in sorted(need))
                       + " (실좌표) 에선 곧게 내려가는 길에서 손가락이 큐브·책상 여유에 걸림")
        return False, why or "곧게 내려가는 길 없음", min(tilts)

    def on_table(self, pc):
        """큐브 중심 pc (sim) 가 책상 위에 설 수 있나: 책상 발자국 안쪽으로 반 변 + CUBE_EDGE_IN (env CUBE_X_MAX 와 같은 1 cm).
        2026-11-21 책상을 발자국으로 본 뒤: 예전 지도는 책상 앞 끝(어깨 축 앞 14 cm) 너머 칸도 초록이었다."""
        x0, x1, y0, y1 = self.s.table_xy
        e = float(self.s.E.OBJECT_HALF_SIZE) + CUBE_EDGE_IN
        return bool(x0 + e <= pc[0] <= x1 - e and y0 + e <= pc[1] <= y1 - e)

    def off_table_why(self):
        x1 = self.s.table_xy[1]
        e = float(self.s.E.OBJECT_HALF_SIZE) + CUBE_EDGE_IN
        return (f"큐브가 책상 밖: 책상 앞 끝(어깨 축 앞 {self.s.R.ORIGIN_XY[0] - x1:.2f} m, 가정)에서 {e * 100:.1f} cm 안쪽 = "
                f"실좌표 y ≤ {x1 - e - self.s.R.ORIGIN_XY[0]:+.3f} 이어야")

    def grip_target(self, pc):
        """큐브 중심 pc (sim) → 잡을(놓을) 때 집는 점 목표 = 위로 GRIP_ABOVE_CENTER (sim z = 위)."""
        pg = self.np.array(pc, dtype=float)
        pg[2] += GRIP_ABOVE_CENTER
        return pg

    def _dirs(self):
        """지도가 보는 내려가기 방향: 수직, 그다음 MAP_TILTS 기울기 × 방위 APPROACH_AZ 마다 (계획 approaches 의 앞쪽과 같은 순서)."""
        np = self.np
        out = []
        for al in (0,) + MAP_TILTS:
            for be in (range(0, 360, APPROACH_AZ) if al else (0,)):
                a_, b_ = math.radians(al), math.radians(be)
                out.append(np.array([math.sin(a_) * math.cos(b_), math.sin(a_) * math.sin(b_), math.cos(a_)]))
        return out

    def _roll_best(self, q):
        hr, best = self.hr, -1.0
        for r in self.np.linspace(self.lim[hr, 0], self.lim[hr, 1], 24):
            qq = q.copy(); qq[hr] = r
            best = max(best, self.grasp_clearance(qq))
        return best

    def _rise(self, pg, qg, v):
        """qg 에서 집는 점을 v 방향 곧게 min(ABOVE_CM) 까지 STEP_DESCENT 마다 (descent 의 거꾸로), 그 위 손목을 z_clear 까지
        2 cm 마다 (transit 의 마지막 다리). (자세 목록, None) 또는 (None, 이유)."""
        np, s = self.np, self.s
        path, why = self._tip_path(pg, qg, v, min(ABOVE_CM) / 100)
        if path is None:
            return None, why
        prev = path[-1]
        w = self.wrist(prev)
        n = int(math.ceil((self.z_clear - w[2]) / 0.02))
        for i in range(1, n + 1):
            sols, _ = s.analytic_position(np.array([w[0], w[1], min(self.z_clear, w[2] + i * 0.02)]), prev, self.lim)
            if not sols:
                return None, "큐브 위로 손목을 옮겨 갈 높이까지 못 올림"
            q = self._near(sols, prev)
            if self._jump(q, prev):
                return None, "큐브 위로 손목을 올리면 팔 가지가 바뀜"
            path.append(q)
            prev = q
        return path, None

    def _yaws_pass(self, pc, path, yaws):
        """yaws 중 이 길(path)로 되는 것 (집합)."""
        np, hr, lim = self.np, self.hr, self.lim
        qg = path[0]
        # 이웃 자세 사이 중간점도 (계획의 프레임은 그 사이 관절 보간)
        poses = [qg] + [a + (b - a) * u for a, b in zip(path, path[1:]) for u in (0.5, 1.0)]
        phi = self.hand_yaw(qg)
        good = set()
        for yaw in yaws:
            d = ((yaw - phi + 45) % 90) - 45
            ok = False
            for k in (-2, -1, 0, 1, 2):
                r = qg[hr] + math.radians(d + 90 * k)
                if not (lim[hr, 0] - 1e-9 <= r <= lim[hr, 1] + 1e-9):
                    continue
                qs = []
                for q in poses:
                    qq = q.copy(); qq[hr] = r + (q[hr] - qg[hr])
                    qs.append(qq)
                if self.grasp_clearance(qs[0]) < TABLE_MARGIN + MAP_SLACK:
                    continue
                if any(self.finger_low(q, self.open) < self.s.table_z + TABLE_MARGIN + MAP_SLACK for q in qs):
                    continue
                if self.cube_hits([(q, self.open) for q in qs], range(len(qs)), pc, yaw):
                    continue
                ok = True
                break
            if ok:
                good.add(yaw)
        return good

    # ---- 손 yaw (위에서 본 손가락 닫히는 축 = site x 의 각, sim °)
    def hand_yaw(self, q):
        _, rot = self.s.fk(q)
        return math.degrees(math.atan2(rot[1, 0], rot[0, 0]))

    def grip(self, q):
        return self.s.fk(q)[0]

    def candidates(self, p, yaw_sim):
        """집는 점 = p (sim) 이고 손 yaw ≡ yaw_sim (mod 90) 인 해 모두: 해석 IK 가지 × hand_roll 후보."""
        np, s, lim, hr = self.np, self.s, self.lim, self.hr
        hold = self.S["q0"].copy()
        hold[hr] = 0.0
        base, _ = s.analytic_tip(p, hold, lim, "position")
        out = []
        for q in base:
            phi = self.hand_yaw(q)
            for k in range(-3, 4):
                r = q[hr] + math.radians(yaw_sim - phi + 90 * k)
                if not (lim[hr, 0] - 1e-9 <= r <= lim[hr, 1] + 1e-9):
                    continue
                qq = q.copy()
                for _ in range(3):    # hand_roll 을 바꿔 다시 풀고(같은 가지), 손 yaw 를 다시 재서 맞춘다
                    h = hold.copy(); h[hr] = float(np.clip(r, lim[hr, 0], lim[hr, 1]))
                    sols, _ = s.analytic_tip(p, h, lim, "position")
                    if not sols:
                        qq = None
                        break
                    qq = min(sols, key=lambda c: float(np.abs(c - qq).max()))
                    e = ((yaw_sim - self.hand_yaw(qq) + 45) % 90) - 45
                    if abs(e) < 0.05:
                        break
                    r = qq[hr] + math.radians(e)
                if qq is None:
                    continue
                e = ((yaw_sim - self.hand_yaw(qq) + 45) % 90) - 45
                tip = float(np.linalg.norm(self.grip(qq) - p))
                inside = bool(np.all(qq >= lim[:, 0] - 1e-6) and np.all(qq <= lim[:, 1] + 1e-6))
                if abs(e) <= YAW_TOL and tip <= TIP_TOL and inside and not any(float(np.abs(qq - o).max()) < 1e-3 for o in out):
                    out.append(qq)
        return out

    # ---- 집는 방향 (2026-11-24 사용자: "집는 각도 수직이랑 수평 전부 가능하게"). 수평 = 손 축이 수평, 옆에서 밀어 넣어 큐브의 세로 두 면을 쥠
    @contextlib.contextmanager
    def aiming(self, u):
        """이 안에서는 해석 IK 의 '손 최대한 아래'(hand_ik.point_down) 가 '손 축(site z)을 u 쪽으로' 가 된다 (u = None 이면 예전대로 아래).
        수평 집기: u = −(밀어 넣는 방향). 제너레이터 안에서 yield 를 이 안에 두지 말 것 (밖의 풀이까지 옆을 봄)."""
        old = getattr(self.s, "aim", None)
        self.s.aim = None if u is None else self.np.asarray(u, dtype=float)
        try:
            yield
        finally:
            self.s.aim = old

    def h_err(self, q, a):
        """수평 집기 자세 오차 (°): (손 축 −site z 와 밀어 넣는 방향 a 사이 각, 손가락 닫히는 축 site x 가 수평에서 벗어난 각)."""
        rot = self.s.fk(q)[1]
        return (math.degrees(math.acos(max(-1.0, min(1.0, float(-rot[:, 2] @ a))))),
                abs(math.degrees(math.asin(max(-1.0, min(1.0, float(rot[2, 0])))))))

    def h_ok(self, q, a):
        e = self.h_err(q, a)
        return e[0] <= H_TOL and e[1] <= H_TOL

    def hand_floor(self, q, f):
        """손가락 충돌 상자의 가장 낮은 점 − 책상 면 (m). 책상 발자국·NO_TABLE 과 상관없이 잰다 — 수평 집기는 손가락이 책상 바로 위를
        누워서 지나가므로 책상 충돌 검사를 꺼도(hand_ik.NO_TABLE) 이 높이는 지킨다 (H_TABLE_MARGIN)."""
        s, np = self.s, self.np
        s.d.qpos[s.fadr] = self._f_eff(f)
        s.fk(q)
        low = 9.0
        for g in s.finger_geoms:
            c, R, h = s.d.geom_xpos[g], s.d.geom_xmat[g].reshape(3, 3), s.m.geom_size[g]
            low = min(low, float(c[2] - (np.abs(R) @ h)[2]))
        return low - s.table_z

    def _f_eff(self, f):
        """닫는 값이 CUBE_GRIP 을 넘으면 큐브가 막는 자세 (finger_low 와 같은 규칙)."""
        s, np = self.s, self.np
        g = float(np.clip((f[0] - self.open[0]) / (self.closed[0] - self.open[0]), 0.0, 1.0)) if self.closed[0] != self.open[0] else 0.0
        return np.asarray(s.env.finger_targets(self.cube_grip), dtype=float) if g > self.cube_grip else f

    def h_grip_target(self, pc):
        pg = self.np.array(pc, dtype=float)
        pg[2] += H_GRIP_ABOVE
        return pg

    def candidates_h(self, pc, yaw_sim):
        """수평 집기 자세 모두: [(qg, a)] — a = 밀어 넣는 방향(수평 단위 벡터, 큐브 면 법선 = yaw + 90°·k 네 방향), 집는 점 = 큐브 중심 높이,
        손 축이 a 와 H_TOL 안, 손가락 닫히는 축이 수평(hand_roll 로 맞춤, 반 바퀴 다른 두 해 중 한계 안인 것)."""
        np, s, lim, hr = self.np, self.s, self.lim, self.hr
        p = self.h_grip_target(pc)
        out = []
        for k in range(4):
            az = math.radians(yaw_sim + 90.0 * k)
            a = np.array([math.cos(az), math.sin(az), 0.0])
            if float(np.linalg.norm(p - a * s.wrist_above_site - self.S["g"][0])) > self.S["reach"] + 0.02:
                continue   # 손목(집는 점에서 a 반대쪽 24 cm, 같은 높이)이 어깨에서 팔 길이 밖
            with self.aiming(-a):
                hold = self.S["q0"].copy()
                hold[hr] = 0.0
                base, _ = s.analytic_tip(p, hold, lim, "position")
                for q in self.family(base) if base else []:
                    if self.h_err(q, a)[0] > H_TOL:
                        continue
                    for qq in self._level_rolls(p, q, hold):
                        tip = float(np.linalg.norm(self.grip(qq) - p))
                        inside = bool(np.all(qq >= lim[:, 0] - 1e-6) and np.all(qq <= lim[:, 1] + 1e-6))
                        if tip <= TIP_TOL and inside and self.h_ok(qq, a) and not any(float(np.abs(qq - o).max()) < 1e-3 for o, _ in out):
                            out.append((qq, a))
        return out

    def _level_rolls(self, p, q, hold):
        """q 에서 hand_roll 만 돌려 손가락 닫히는 축(site x)을 수평으로 (반 바퀴 다른 두 해, 한계 안). hand_roll 을 바꾸면 집는 점이 조금
        움직이므로 다시 풀고(같은 가지) 다시 잰다 (candidates 와 같은 방식). 지금 self.s.aim 그대로 쓴다."""
        np, s, lim, hr = self.np, self.s, self.lim, self.hr

        def delta(qq):   # site x 를 수평으로 만드는 hand_roll 변화 (rad, 두 해 중 작은 쪽)
            rot = s.fk(qq)[1]
            q1 = qq.copy(); q1[hr] += 0.2
            sg = 1.0 if float(s.fk(q1)[1][:, 0] @ rot[:, 1]) > 0 else -1.0
            d = math.atan2(-float(rot[2, 0]), sg * float(rot[2, 1]))
            return (d + math.pi / 2) % math.pi - math.pi / 2
        out = []
        d0 = delta(q)
        for n in (-2, -1, 0, 1, 2):
            r = q[hr] + d0 + math.pi * n
            if not (lim[hr, 0] - 1e-9 <= r <= lim[hr, 1] + 1e-9):
                continue
            qq = q.copy()
            for _ in range(3):
                h = hold.copy(); h[hr] = float(np.clip(r, lim[hr, 0], lim[hr, 1]))
                sols, _ = s.analytic_tip(p, h, lim, "position")
                if not sols:
                    qq = None
                    break
                qq = min(sols, key=lambda c: float(np.abs(c - qq).max()))
                e = delta(qq)
                if abs(e) < math.radians(0.05):
                    break
                r = qq[hr] + e
            if qq is not None:
                out.append(qq)
        return out

    def h_push(self, pc, qg, a, cm, rise_cm):
        """수평 집기의 들어가는 길 [큐브 옆 위 … 큐브 옆 … qg]: 큐브 옆(집는 점에서 a 반대쪽 cm) 위 rise_cm 에서 손을 수평으로 둔 채 곧게 내려와,
        같은 높이에서 a 방향 cm 곧게 밀어 넣는다 (손 축 수평 H_TOL, 손가락 책상 위 H_TABLE_MARGIN, 팔·손 메시가 책상 면 위).
        옆 위에서 내려오는 구간이 없으면 손목 길(transit)의 끝에서 손이 누우며 벌린 손가락이 큐브를 쓸고 지나간다. (길, None) 또는 (None, 이유)."""
        p = self.h_grip_target(pc)
        with self.aiming(-a):
            back, _ = self._tip_path(p, qg, -a, cm / 100)
            rise = None if back is None else self._tip_path(p - a * (cm / 100), back[-1], self.np.array([0.0, 0.0, 1.0]), rise_cm / 100)[0]
        if back is None:
            return None, f"수평: 큐브 옆 {cm:g} cm 에서 곧게 밀어 넣는 길이 없음"
        if rise is None:
            return None, f"수평: 큐브 옆 위 {rise_cm:g} cm 에서 손을 수평으로 둔 채 곧게 못 내려옴"
        if not all(self.h_ok(q, a) for q in back + rise):
            return None, "수평: 밀어 넣는 동안 손을 수평으로 못 둠 (손목 한계)"
        if min(min(self.hand_floor(q, self.open), self.hand_floor(q, self.closed)) for q in back) < H_TABLE_MARGIN:
            return None, f"수평: 누운 손가락이 책상 위 {H_TABLE_MARGIN * 1000:.0f} mm 를 못 지킴"
        if min(self.mesh_floor(back[0]), self.mesh_floor(back[-1])) < 0.0:
            return None, "수평: 손·손목이 책상 면 아래"
        return rise[::-1] + back[::-1][1:], None

    def h_lift(self, pc, qg, a, cm):
        """수평 집기의 드는 길 [qg … 위]: 집는 점을 곧게 위로 cm, 손 축 수평 그대로. (길, None) 또는 (None, 이유)."""
        with self.aiming(-a):
            up, _ = self._tip_path(self.h_grip_target(pc), qg, self.np.array([0.0, 0.0, 1.0]), cm / 100)
        if up is None:
            return None, f"수평: 손을 수평으로 둔 채 {cm:g} cm 곧게 못 들어 올림"
        if not all(self.h_ok(q, a) for q in up):
            return None, "수평: 드는 동안 손을 수평으로 못 둠 (손목 한계)"
        return up, None

    def mesh_floor(self, q):
        """팔·손 메시(기둥 검사용으로 솎은 꼭짓점)의 가장 낮은 점 − 책상 면 (m). 수평 집기에서 손바닥·손목이 책상 속이 아닌지."""
        s = self.s
        s.fk(q)
        return min(float((self.col_verts[g] @ s.d.geom_xmat[g].reshape(3, 3).T + s.d.geom_xpos[g])[:, 2].min()) for g in self.col_geoms) - s.table_z

    def approaches_h(self, pc, Gh):
        """수평 집기의 길 후보 (게으르게, approaches 와 같은 꼴): Gh = candidates_h 의 [(qg, a)]. 밀어 넣기 H_PUSH_CM × 들기 H_LIFT_CM.
        pairs = [(큐브 옆 위 자세, qg, 들어가는 길(내려와 밀어 넣기), 드는 길)]. 옆 위 높이 = 드는 높이."""
        for push in H_PUSH_CM:
            for lift in H_LIFT_CM:
                pairs = []
                for qg, a in Gh:
                    A, B = self.h_push(pc, qg, a, push, lift)[0], self.h_lift(pc, qg, a, lift)[0]
                    if A is not None and B is not None:
                        pairs.append((A[0], qg, A, B))
                if pairs:
                    yield {"cm": lift, "push_cm": push, "tilt_deg": 90, "azimuth_deg": None, "v": None, "pairs": pairs}

    def approaches(self, p, G, axis, vertical=False):
        """큐브 위 경유점 후보를 좋은 순서로 (게으르게): 잡는 자세 qg(G) 마다 집는 점을 v 방향 곧게 h cm 올린 길 (_tip_path,
        hand_roll 은 qg 그대로 — 지도 point_check 와 같은 걸음). yield {cm, 방향, pairs: [(위 자세 qa, qg, 내려가기 qa→qg)]}.
        순서 점수 = 3 × (v 와 잡는 자세 손 축(axis) 사이 각 °) + 5 × (9 − h cm): 손가락 방향 그대로 위로 빠지는 것을 모든 높이에서
        먼저, 그다음 높은 것. 2026-11-21: 예전엔 위 경유점을 따로(처음부터) 풀어 잡는 자세로 내려갔는데, 손목 관절이 한계에 닿는
        곳에선 따로 푼 해가 없거나 다른 가지라 지도(이어서 올림)와 판정이 달랐다."""
        np = self.np
        opts = []
        for al in ((0,) if vertical else APPROACH_TILTS):   # 수직: 곧게 수직으로만, 길 내내 손 축 VERT_MAX_TILT 안
            for be in (range(0, 360, APPROACH_AZ) if al else (0,)):
                a_, b_ = math.radians(al), math.radians(be)
                v = np.array([math.sin(a_) * math.cos(b_), math.sin(a_) * math.sin(b_), math.cos(a_)])
                ang = math.degrees(math.acos(max(-1.0, min(1.0, float(v @ axis)))))
                for cm in ABOVE_CM:
                    opts.append((3.0 * ang + 5.0 * (ABOVE_CM[0] - cm), cm, al, be, v))   # 손 축 방향(수직) 먼저, 그다음 높은 것
        opts.sort(key=lambda o: o[0])
        for _, cm, al, be, v in opts:
            pairs = []
            for qg in G:
                path, _ = self._tip_path(p, qg, v, cm / 100)
                if path is not None and not (vertical and max(self.s.axis_tilt(self.s.fk(x)[1]) for x in path) > VERT_MAX_TILT):
                    pairs.append((path[-1], qg, path[::-1], path))   # (위 자세, 잡는 자세, 내려가는 길, 드는 길)
            if pairs:
                yield {"cm": cm, "tilt_deg": al, "azimuth_deg": be, "v": v, "pairs": pairs}

    def _tip_path(self, p, qg, v, top):
        """qg(집는 점 = p)에서 집는 점을 v 방향 곧게 top(m) 까지 STEP_DESCENT 마다 (_tip_step, hand_roll 고정). (자세 목록, None) 또는 (None, 이유)."""
        path, prev = [qg], qg
        n = max(1, int(round(top / STEP_DESCENT)))
        for k in range(1, n + 1):
            q = self._tip_step(prev, p + v * (top * k / n), qg[self.hr])
            if q is None:
                return None, f"큐브 위 {top * k / n * 100:.1f} cm 까지 곧게 못 올라감 (같은 팔 가지)"
            path.append(q)
            prev = q
        return path, None

    def hand_axis(self, q):
        return self.s.fk(q)[1][:, 2].copy()

    def cube_hits(self, frames, idx, pos, yaw_deg):
        """frames[idx] 중 손가락 충돌 상자가 큐브(5 cm, pos sim, yaw)에 0.5 mm 넘게 파고드는 프레임 번호."""
        mj, s = self.s.mj, self.s
        m, d = s.m, s.d
        a = m.jnt_qposadr[m.body_jntadr[self.obj_body]]
        h = math.radians(yaw_deg) / 2
        d.qpos[a:a + 7] = [pos[0], pos[1], pos[2], math.cos(h), 0.0, 0.0, math.sin(h)]
        out = []
        for i in idx:
            q, f = frames[i]
            d.qpos[s.qadr] = q
            d.qpos[s.fadr] = f
            mj.mj_kinematics(m, d)
            mj.mj_collision(m, d)
            for c in d.contact[:d.ncon]:
                if self.cube_geom in (c.geom1, c.geom2) and (c.geom1 in self.fgeoms or c.geom2 in self.fgeoms) and c.dist < -5e-4:
                    out.append(i)
                    break
        return out

    # ---- 경로 (관절 목록)
    A3I = (0, 2, 3)   # shoulder_roll, upper_arm_roll, elbow: 해석 IK 가 푸는 팔 관절 (가지 이어 붙이기는 이것으로)

    def wrist(self, q):
        self.s.fk(q)
        return self.s.wrist_pos().copy()

    def _near(self, sols, prev):
        i = list(self.A3I)
        return min(sols, key=lambda c: float(self.np.abs(c[i] - prev[i]).max()))

    def _jump(self, q, prev):
        """이웃 IK 점 사이 팔 관절이 JUMP_DEG 넘게 튀면 (가지가 끊겨 다른 가지로 넘어감) 그 길은 버린다."""
        i = list(self.A3I)
        return float(self.np.degrees(self.np.abs(q[i] - prev[i])).max()) > JUMP_DEG

    def _same(self, q, qt):
        i = list(self.A3I)
        return float(self.np.degrees(self.np.abs(q[i] - qt[i])).max()) <= SAME_BRANCH_DEG

    def transit(self, qf, qt, f_home=False, t_home=False):
        """손목 중앙 길: 위로 → 높이 유지하며 옆으로 → 아래로 (높이 = 두 끝과 z_clear 중 높은 쪽, 안 되면 +3, +6 cm, 그래도 안 되면 곧은 선),
        1 cm 마다 해석 IK(앞 가지 이어서, 튀면 버림). 손목 2 관절은 손이 최대한 아래 (point_down). 끝이 HOME(손이 옆으로 누운 자세)이면
        그 손목 값에서/으로 옆 이동 동안 섞는다. hand_roll 은 옆 이동 동안 qf → qt. 끝 해가 qt 와 다른 가지면 None."""
        np = self.np
        w0, w1 = self.wrist(qf), self.wrist(qt)
        for dz in TRANSIT_LIFTS:
            if dz is None:   # 탑 쌓기만 (plan 이 stack_standoff 를 줌): 곧은 선 전에 '높은 쪽 끝 가까이에서는 그 높이 그대로' 길
                for sd in (getattr(self, "stack_standoff", None) or ()):
                    pts = self._standoff_pts(w0, w1, sd)
                    r = self._track(pts, qf, qt, f_home, t_home) if pts else None
                    if r is not None:
                        return r
            for arc in ((False, True) if dz is not None else (False,)):
                if dz is None:
                    legs = ((w0, w1, 1),)
                else:
                    zt = max(w0[2], w1[2], self.z_clear) + dz
                    u0, u1 = np.array([w0[0], w0[1], zt]), np.array([w1[0], w1[1], zt])
                    legs = ((w0, u0, 0), (u0, u1, 1), (u1, w1, 2))
                pts, empty_arc = [], False
                for a, b, leg in legs:
                    if arc and leg == 1:
                        ap = self._arc_pts(a, b)
                        empty_arc = not ap
                        pts += ap
                        continue
                    n = int(math.ceil(float(np.linalg.norm(b - a)) / STEP_TRANSIT))
                    for k in range(1, n + 1):
                        pts.append((a + (b - a) * (k / n), leg, k / n))
                if empty_arc:
                    continue
                r = self._track(pts, qf, qt, f_home, t_home)
                if r is not None:
                    return r
        return None

    def _standoff_pts(self, w0, w1, sd):
        """탑 쌓기: 두 끝 중 높은 쪽(탑 위)에서 수평으로 sd (m) 까지는 그 높이를 지키고, 나머지를 곧은 선으로 (손목 중앙 길).
        위로 먼저 올라가는 길(TRANSIT_LIFTS)이 팔 길이로 안 될 때, 곧은 선은 탑 옆을 비스듬히 스친다 — 탑 가까이에서만 높이를 지킨다."""
        np = self.np
        h = w1[:2] - w0[:2]
        L = float(np.linalg.norm(h))
        if L < 0.03 or abs(float(w1[2] - w0[2])) < 0.01:
            return None
        f = min(sd, 0.85 * L) / L
        if w0[2] >= w1[2]:   # 탑에서 떠남: 수평으로 먼저, 그다음 비스듬히 아래로
            m = np.array([w0[0] + h[0] * f, w0[1] + h[1] * f, w0[2]])
            legs = ((w0, m, 0), (m, w1, 1))
        else:                # 탑으로 감: 비스듬히 위로 먼저, 마지막은 수평으로
            m = np.array([w1[0] - h[0] * f, w1[1] - h[1] * f, w1[2]])
            legs = ((w0, m, 1), (m, w1, 2))
        pts = []
        for a, b, leg in legs:
            n = max(1, int(math.ceil(float(np.linalg.norm(b - a)) / STEP_TRANSIT)))
            pts += [(a + (b - a) * (k / n), leg, k / n) for k in range(1, n + 1)]
        return pts

    def _arc_pts(self, a, b):
        """높이 유지 옆 이동을 어깨 아래 세로축(ARC 중심 = HOME 손목 xy) 둘레로: 반지름·각을 선형으로 (2026-11-21, 팔꿈치 ±100).
        팔꿈치를 100° 까지만 접으면 손목이 어깨에서 약 0.29 m 안으로 못 들어와, 곧은 선은 두 끝이 닿아도 가운데가 안쪽 원을 지나 끊긴다.
        두 끝 반지름 사이로만 지나는 호는 그 원에 안 들어간다. 두 끝이 거의 같은 각·반지름이면 빈 목록 (곧은 선과 같음)."""
        np = self.np
        c = self.arc_c
        ra, rb = float(np.hypot(*(a[:2] - c))), float(np.hypot(*(b[:2] - c)))
        ta, tb = math.atan2(a[1] - c[1], a[0] - c[0]), math.atan2(b[1] - c[1], b[0] - c[0])
        dt = (tb - ta + math.pi) % (2 * math.pi) - math.pi
        L = math.hypot(abs(rb - ra), max(ra, rb) * abs(dt))
        if abs(dt) < math.radians(2.0):
            return []
        n = max(1, int(math.ceil(L / STEP_TRANSIT)))
        out = []
        for k in range(1, n + 1):
            u = k / n
            r, t = ra + (rb - ra) * u, ta + dt * u
            out.append((np.array([c[0] + r * math.cos(t), c[1] + r * math.sin(t), a[2] + (b[2] - a[2]) * u]), 1, u))
        return out

    def _track(self, pts, qf, qt, f_home, t_home):
        s = self.s
        prev, out = qf.copy(), [qf.copy()]
        hr, i56 = self.hr, [4, 5]
        ie = s.arm.index("elbow")
        for w, leg, u in pts:
            b = 0.0 if leg == 0 else 1.0 if leg == 2 else u * u * (3 - 2 * u)
            hold = prev.copy()
            hold[hr] = qf[hr] + (qt[hr] - qf[hr]) * b
            sols, _ = s.analytic_position(w, hold, self.lim)
            # 끝(qt)과 같은 팔꿈치 가지만: HOME(팔꿈치 0, 팔을 쭉 편 특이점)에서 나갈 때 두 가지가 똑같이 가까워 엉뚱한 쪽을 탈 수 있다
            sols = [c for c in sols if (c[ie] < 0) == (qt[ie] < 0)] or sols
            if not sols:
                return None
            q = self._near(sols, prev).copy()
            if self._jump(q, prev):
                return None
            a56 = qf[i56] if f_home else q[i56]
            b56 = qt[i56] if t_home else q[i56]
            q[i56] = a56 + (b56 - a56) * b
            out.append(q)
            prev = q
        if not out or not self._same(out[-1], qt):
            return None
        out[-1] = qt.copy()
        return out

    def via_home(self, qa, qb, home0=None, blend=False):
        """곧장 옮겨 갈 길이 없으면 (두 자리가 팔 가지·도달 범위로 갈라져 있음) HOME 쪽을 거쳐서: qa → (손목 길 시작 경유점 self.exit[-1],
        예전 HOME 이면 HOME0) → qb. 없으면 None."""
        hx, fh = self.exit[-1], self.exit_fh
        t1 = self.transit(qa, hx, f_home=blend, t_home=fh)   # blend (수평 집기): 두 끝의 손목 값을 옆 이동 동안 섞음
        t2 = self.transit(hx, qb, f_home=fh, t_home=blend) if t1 else None
        return t1 + t2[1:] if t1 and t2 else None

    def family(self, cands):
        """ELBOW_BRANCH 쪽 팔꿈치 가지만 (예전 HOME 의 가지 = elbow < 0). 반대 가지로 넘어가려면 팔을 쭉 펴야 해서(특이점)
        공중에서 크게 휘두르게 된다. HOME 이 모든 관절 0 (팔꿈치 0 = 두 가지의 경계) 이 된 2026-11-21 부터 HOME 이 아니라 상수로.
        같은 가지 해가 없으면 그대로 (그때는 길을 못 찾아 관절 직선 → 검사에 걸림)."""
        e = self.s.arm.index("elbow")
        same = [q for q in cands if (q[e] < 0) == (ELBOW_BRANCH < 0)]
        return same or cands

    def near_branch(self, qa, qg):
        i = list(self.A3I)
        return float(self.np.degrees(self.np.abs(qa[i] - qg[i])).max()) <= 45.0

    def descent(self, qa, qg):
        """집는 점을 qa 의 집는 점에서 qg 의 집는 점까지 직선으로 (STEP_DESCENT 마다, 앞 가지 이어서). hand_roll 은 qa → qg.
        각 점: 손목 목표 = 앞 손목 + (집는 점 목표 − 앞 집는 점) 에서 시작해 해석 IK(손 최대한 아래) → 집는 점 오차만큼 손목을 옮겨
        다시 (analytic_tip 과 같은 방식, 앞 점에서 이어 시작해 빠르다)."""
        np, s = self.np, self.s
        pa, pg = self.grip(qa), self.grip(qg)
        n = max(1, int(math.ceil(float(np.linalg.norm(pg - pa)) / STEP_DESCENT)))
        prev, out = qa.copy(), [qa.copy()]
        for k in range(1, n + 1):
            p = pa + (pg - pa) * (k / n)
            q = self._tip_step(prev, p, qa[self.hr] + (qg[self.hr] - qa[self.hr]) * (k / n))
            if q is None:
                return None
            out.append(q)
            prev = q
        if not self._same(out[-1], qg) or float(np.linalg.norm(self.grip(out[-1]) - pg)) > TIP_TOL:
            return None
        out[-1] = qg.copy()
        return out

    def _tip_step(self, prev, p, roll):
        """한 걸음: 집는 점을 p 로 (hand_roll = roll). 손목 목표 = 앞 손목 + (p − 앞 집는 점) 에서 해석 IK(손 최대한 아래) →
        집는 점 오차만큼 손목을 옮겨 다시. 앞 가지에 가장 가까운 해, 튀면 None. descent·지도(_rise) 가 같이 쓴다."""
        np, s = self.np, self.s
        hold = prev.copy()
        hold[self.hr] = roll
        w = self.wrist(prev) + (p - self.grip(prev))
        q = None
        for _ in range(8):
            sols, _ = s.analytic_position(w, hold, self.lim)
            if not sols:
                return None
            q = self._near(sols, prev)
            e = p - self.grip(q)
            if float(np.linalg.norm(e)) < 3e-4:
                break
            w = w + e
        # 2026-11-21: 8 번 안에 집는 점이 p 에 안 오면(손목 한계) 버린다. 예전엔 오차를 안 봐서 '큐브 위 9 cm' 가 19 cm 옆인 길을 받아들였다
        if float(np.linalg.norm(p - self.grip(q))) > TIP_TOL:
            return None
        return None if self._jump(q, prev) else q

    def arc(self, qs):
        np = self.np
        return float(sum(np.abs(b - a).max() for a, b in zip(qs, qs[1:])))

    def frames(self, qs, fa, fb, speed_deg_s, min_s=0.0):
        """관절 목록 qs (꺾은선) 를 따라, 손가락 fa → fb. 길이 L = 이웃 사이 관절 최대 이동각의 합(손가락 포함).
        시간은 사다리꼴 (2026-11-21): 속도 ≤ min(speed, √(2·MAX_ACC·TRACK_LAG_DEG)), 가속 ≤ MAX_ACC (실물 real_arm 램프의 max_acc) — 가장 빠른 관절도
        둘 다 안 넘는다 (꺾은선 모서리에선 관절별 방향이 바뀌어 순간 가속이 더 클 수 있다: 1 cm 마다라 작다). min_s 보다 짧으면 늘인다.
        첫 프레임 뺀 (q, f) 목록."""
        np = self.np
        fmove = float(np.abs(fb - fa).max())
        segs = [float(np.abs(b - a).max()) for a, b in zip(qs, qs[1:])]
        L = sum(segs)
        Lt = math.degrees(max(L, fmove))
        acc = MAX_ACC
        v = min(float(speed_deg_s), math.sqrt(2.0 * acc * TRACK_LAG_DEG))   # 실물 램프는 목표에서 멈추게 감속하므로 움직이는 목표를
        #   v²/(2·acc) 만큼 늦게 따라간다 (30 °/s 면 5°) — 늦음을 TRACK_LAG_DEG 안으로: 순항 속도 √(2·acc·lag)
        if Lt >= v * v / acc:
            T = Lt / v + v / acc
        else:
            T = 2.0 * math.sqrt(Lt / acc) if Lt > 0 else 0.0
        k_ = max(1.0, (min_s / T) if T > 0 else 1.0)      # min_s 로 늘이기: 시간 축만 늘림 (속도 1/k, 가속 1/k²)
        Ts = max(T * k_, min_s)
        n = max(1, int(math.ceil(Ts / DT - 1e-9)))
        cum = np.concatenate([[0.0], np.cumsum(segs)])
        out = []
        for k in range(1, n + 1):
            u = _trap(k / n * T, T, Lt, v, acc) / Lt if Lt > 0 and T > 0 else 1.0
            if L <= 0 or len(qs) == 1:
                q = qs[-1].copy()
            else:
                x = u * L
                j = min(int(np.searchsorted(cum, x, side="right")) - 1, len(segs) - 1)
                r = 0.0 if segs[j] <= 0 else (x - cum[j]) / segs[j]
                q = qs[j] + (qs[j + 1] - qs[j]) * min(1.0, max(0.0, r))
            out.append((q, fa + (fb - fa) * u))
        return out

    def table_penalty(self, q):
        """잡는·놓는 자세에서 손가락(벌림·쥠)이 책상 아래면 벌점. 손이 기울면 아래쪽 손가락 끝이 내려간다 —
        hand_roll 90° 다른 해(큐브는 90° 마다 같음)는 기울기 축이 달라 손가락 높이가 다르다: 닿지 않는 쪽을 고르게."""
        return 0.0 if self.grasp_clearance(q) >= TABLE_MARGIN else FALLBACK_COST

    def grasp_clearance(self, q):
        """잡는·놓는 자세 q 에서 손가락(벌림·쥠 중 낮은 쪽) 가장 낮은 점의 책상 위 높이 (m)."""
        return min(self.finger_low(q, self.open), self.finger_low(q, self.closed)) - self.s.table_z

    def below_table(self, fr):
        return any(self.finger_low(q, f) < self.s.table_z + TABLE_MARGIN or self.arm_low(q) < self.s.table_z for q, f in fr)

    def arm_low(self, q):
        """팔 기준점(팔꿈치·손목, 링크 반지름 hand_ik.ARM_RADIUS 뺌) 중 책상 발자국 위에 있는 것의 가장 낮은 z (sim). 없으면 9.0."""
        self.s.fk(q)
        return self.s.arm_low_z()

    def carried_cube_low(self, qg, pc, yaw_deg, qs):
        """잡는 자세 qg 에서 큐브(중심 pc, yaw, 수평)를 손에 붙여 qs 자세들로 옮길 때 큐브 8 꼭짓점 중 가장 낮은 z (sim).
        손이 기울면 큐브도 같이 기운다 (바닥 중심이 아니라 가장 낮은 모서리)."""
        np = self.np
        ps, rs = self.s.fk(qg)
        h = math.radians(yaw_deg)
        rc = np.array([[math.cos(h), -math.sin(h), 0.0], [math.sin(h), math.cos(h), 0.0], [0.0, 0.0, 1.0]])
        rel_r, rel_p = rs.T @ rc, rs.T @ (np.asarray(pc) - ps)
        half = float(self.s.E.OBJECT_HALF_SIZE)
        low = 9.0
        for q in qs:
            p, r = self.s.fk(q)
            c, rr = p + r @ rel_p, r @ rel_r
            low = min(low, float(c[2] - half * np.abs(rr[2]).sum()))
        return low

    # ---- 다른 큐브 (탑 쌓기, 2026-11-24): 상자 = (중심 sim, 회전 3x3, 반 변). plan() 의 --obstacle 만 쓴다
    @staticmethod
    def _box_gap(np, V, c, R, half):
        """점들 V 와 상자 사이 가장 짧은 거리 (m, 점이 상자 안이면 음수 = 파고든 깊이)."""
        a = np.abs((V - c) @ R) - half
        inside = (a < 0).all(axis=1)
        if inside.any():
            return -float((-a[inside]).min(axis=1).max())
        o = np.maximum(a, 0.0)
        return float(np.sqrt((o * o).sum(axis=1)).min())

    def obstacle_gap(self, q, f, boxes):
        """팔·손 메시 꼭짓점(기둥 검사와 같은 솎은 꼭짓점)과 상자들 사이 가장 짧은 거리 (m)."""
        if not boxes:
            return 9.0
        s, np = self.s, self.np
        d = s.d
        d.qpos[s.fadr] = f
        s.fk(q)
        V = np.concatenate([self.col_verts[g] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g] for g in self.col_geoms])
        return min(self._box_gap(np, V, c, R, h) for c, R, h in boxes)

    def carried_cube_gap(self, qg, pc, yaw_deg, qs, boxes):
        """잡는 자세 qg 에서 손에 붙인 큐브(carried_cube_low 와 같은 가정)와 상자들 사이 가장 짧은 거리 (m, 겹치면 음수).
        두 상자의 겉면 점(5x5x5 격자)을 서로의 상자에 대 본다."""
        if not boxes:
            return 9.0
        np = self.np
        ps, rs = self.s.fk(qg)
        h = math.radians(yaw_deg)
        rc = np.array([[math.cos(h), -math.sin(h), 0.0], [math.sin(h), math.cos(h), 0.0], [0.0, 0.0, 1.0]])
        rel_r, rel_p = rs.T @ rc, rs.T @ (np.asarray(pc) - ps)
        half = float(self.s.E.OBJECT_HALF_SIZE)
        g = np.linspace(-1.0, 1.0, 5)
        pts = np.array([(x, y, z) for x in g for y in g for z in g if max(abs(x), abs(y), abs(z)) == 1.0])
        best = 9.0
        for q in qs:
            p, r = self.s.fk(q)
            c, rr = p + r @ rel_p, r @ rel_r
            mine = c + (pts * half) @ rr.T
            for bc, bR, bh in boxes:
                best = min(best, self._box_gap(np, mine, bc, bR, bh), self._box_gap(np, bc + (pts * bh) @ bR.T, c, rr, half))
        return best

    def finger_low(self, q, f):
        """손가락 충돌 상자 가장 낮은 z. 닫는 값이 CUBE_GRIP(패드 사이 = 큐브 50 mm, env) 을 넘는 프레임은 큐브가 손가락을
        거기서 막으므로 CUBE_GRIP 자세로 잰다 (명령은 팀처럼 1.0 — 남는 만큼이 쥐는 힘)."""
        s, np = self.s, self.np
        g = float(np.clip((f[0] - self.open[0]) / (self.closed[0] - self.open[0]), 0.0, 1.0)) if self.closed[0] != self.open[0] else 0.0
        if g > self.cube_grip:
            f = np.asarray(s.env.finger_targets(self.cube_grip), dtype=float)
        s.d.qpos[s.fadr] = f
        s.fk(q)
        return s.lowest_finger_z()


def plan(a):
    """집는 방향 a.grasp (없으면 auto): vertical / horizontal = 그 방식만. auto = 수직 → 수평 → 기울임(예전 방식) 순서로 검사를 다 통과하는
    첫 계획 (2026-11-24 사용자: 둘 다 되면 수직, 수직이 안 되는 자리만 수평, 그다음 되는 만큼 기울여서). 어느 것도 통과 못 하면 기울임의 결과.
    결과 "grasp" = {mode: 고른 값, style: 실제 쓴 방식, tilt_deg: 잡는 자세 손 기울기, text: "집는 방향: 수직 (기울기 3°)", tried: 먼저 해 본 방식의 이유}."""
    t0 = time.time()
    grasp = getattr(a, "grasp", None) or "auto"
    if grasp not in GRASP_MODES:
        return {"planner": "analytic", "status": "unreachable", "reason": f"집는 방향은 {' / '.join(GRASP_MODES)} 중 하나 ({grasp!r})"}
    cal_file = a.calibration or (REPO / "arm_control" / "calibration.json")
    ik_dir = _find_ik(a.ik)
    if a.d is not None and not (math.isfinite(a.d) and D_RANGE[0] - 1e-9 <= a.d <= D_RANGE[1] + 1e-9):   # 모델을 읽기 전에 거절
        import hand_ik
        return {"planner": "analytic", "real": {"cube": [float(v) for v in a.cube], "target": [float(v) for v in a.target], "d": float(a.d)},
                "status": "unreachable", "reason": d_range_text(a.d, hand_ik.TABLE_D_DEFAULT)}
    rng = _cmd_ranges(cal_file)
    if grasp == "tilted":   # 어제(11-23 20:22) 실물에서 성공한 계획과 같게: 손목 ±90, 기울여 집기만
        rng = tilt_ranges(rng)
    P = Planner(ik_dir, rng, d=a.d, grasp=grasp)
    drop = getattr(a, "pick_drop", None)
    drop = PICK_DROP_M if drop is None else float(drop)
    if not (math.isfinite(drop) and 0.0 <= drop <= 0.15):
        return {"planner": "analytic", "status": "unreachable", "reason": f"집기 낮춤은 0 ~ 15 cm ({drop * 100:g} cm)"}
    P.pick_drop = drop
    P.s.table_z -= drop   # 책상 충돌 검사 바닥 = 낮춘 집는 자리 (z_clear·HOME 경유점은 위 Planner 에서 이미 실제 책상으로 정함)
    out = {"planner": "analytic",
           "real": {"cube": [float(v) for v in a.cube], "target": [float(v) for v in a.target], "d": P.table_d, "grasp": grasp,
                    "pick_drop_m": drop}}
    level, obst_real, _ = _stack_args(a, P)   # 탑 쌓기 (--place-level, --obstacle): 안 주면 0, [] — 예전과 똑같다
    lifts = [0.0]
    if level or obst_real:
        side_m = 2.0 * float(P.s.E.OBJECT_HALF_SIZE)
        out["real"].update(place_level=level, obstacles=obst_real, target_yaw=float(a.cube[2] if a.target_yaw is None else a.target_yaw))
        # 옮겨 가는 높이 하한(z_clear)은 큐브 한 층 기준. 그 높이로 탑에 걸리면 한 번 더 (올리면 팔이 못 닿는 자리가 늘어 먼저 낮게 해 본다):
        # ① 다른 큐브들(이미 있는 탑) 윗면 기준 ② 이번에 놓고 난 탑 윗면 기준. 2026-11-24: 예전엔 ② 만 — 3층(놓고 난 탑 15 cm)이면 +10 cm 라
        # 큐브 위로 손목을 못 올려 거절되고, 낮은 길(+0)의 결과(HOME 경유점 옆에서 든 큐브가 2층 탑을 −15 mm 파고듦)가 나왔다
        for top in (max([(o[3] + 1) * side_m for o in obst_real], default=0.0), (level + 1) * side_m):
            if top > side_m + 1e-9 and all(abs(top - side_m - v) > 1e-9 for v in lifts):
                lifts.append(top - side_m)
        P.stack_standoff = (0.20, 0.16, 0.12, 0.08)
    z0 = P.z_clear
    tried, r, first = {}, None, None
    tries = [((0.0, 0.0), lift) for lift in lifts]
    k = 0
    while k < len(tries):
        shift, lift = tries[k]
        k += 1
        P.z_clear = z0 + lift
        P.exit_shift = shift
        if len(tries) > 1:
            # HOME 나가기·들어가기 경유점(손목 높이 z_clear + HOME_EXIT_LIFT)도 같이 올린다. 예전엔 계획기를 만들 때의 높이 그대로라
            # z_clear 를 올려도 오가는 길이 그 낮은 경유점(탑 자리 바로 앞)을 지나 탑에 걸렸다. 못 풀면 exit_ok False → 그 높이는 거절
            P._home_exit()
        tried = {}
        for style in (("vertical", "horizontal", "tilted") if grasp == "auto" else (grasp,)):
            r = _plan_style(a, P, style, out)
            g = r.get("grasp") or {"style": style, "tilt_deg": None, "text": f"집는 방향: {STYLE_KO[style]}"}
            r["grasp"] = {"mode": grasp, **g, "tried": dict(tried)}
            if r["status"] == "playable":
                break
            tried[style] = r.get("reason")
        first = first or (r, tried)
        if r["status"] == "playable":
            if shift != (0.0, 0.0):
                r.setdefault("checks", {})["home_exit_shift_sim_m"] = list(shift)
            break
        if k == len(tries) and level and shift == (0.0, 0.0):
            # 다른 큐브 위에 놓는데 안 됨 (2026-11-24, 3층): HOME 나가기·들어가기 경유점이 탑 자리 바로 앞(기본 자리에서 6 cm)이고 탑 윗면보다 낮다.
            # 3층은 위로 4~5.5 cm 밖에 못 빠지고 그 높이로는 경유점 쪽으로 못 가서 (어깨 가까이는 그 높이에 손목이 못 감) 돌아가는 길이 비스듬히
            # 내려가며 놓은 큐브를 스친다 → 탑 쌓기일 때만 경유점을 탑에서 먼 쪽으로 옮겨 다시 (검사·여유는 그대로)
            for sh in P.exit_shifts_away(P.s.to_sim([float(a.target[0]), float(a.target[1]), 0.0]), z0):
                tries += [(sh, lift) for lift in lifts]
    P.exit_shift = (0.0, 0.0)
    if r["status"] != "playable":
        r, tried = first   # 둘 다 안 되면 낮은 길의 결과 (이유가 더 알기 쉽다)
    if r["status"] != "playable" and grasp == "auto":
        r["grasp"]["tried"] = tried
        r["reason"] = f"{r.get('reason')} · 자동: 수직·수평도 안 됨 (수직: {tried.get('vertical')} / 수평: {tried.get('horizontal')})"
    r["plan_s"] = round(time.time() - t0, 2)
    return r


def _stack_args(a, P):
    """탑 쌓기 인자 (2026-11-24, 덧붙임): (놓는 층 K, 다른 큐브 [[x, y, yaw°, 층] 실좌표], 그 상자들 (중심 sim, 회전, 반 변)).
    --place-level K = 큐브 바닥을 책상 위 K·5 cm (+ STACK_GAP) 에 놓는다. --obstacle 의 층 0 = 책상 위."""
    s, np = P.s, P.np
    level = int(getattr(a, "place_level", 0) or 0)
    obst = [[float(v) for v in o] for o in (getattr(a, "obstacle", None) or [])]
    side_m = 2.0 * float(s.E.OBJECT_HALF_SIZE)
    boxes = []
    for ox, oy, oyaw, olv in obst:
        h = math.radians(oyaw + s.R.SIM_FROM_REAL_YAW)
        boxes.append((np.asarray(s.to_sim([ox, oy, (olv + 0.5) * side_m - P.table_d]), dtype=float),
                      np.array([[math.cos(h), -math.sin(h), 0.0], [math.sin(h), math.cos(h), 0.0], [0.0, 0.0, 1.0]]), side_m / 2))
    return level, obst, boxes


def _plan_style(a, P, style, out):
    """한 방식(style = vertical / horizontal / tilted)으로 계획. plan 이 부른다."""
    hz, vt = style == "horizontal", style == "vertical"
    ko = STYLE_KO[style]
    s, np = P.s, P.np
    R = s.R
    D = P.table_d   # 책상 높이 (어깨 축 아래 m): 기본 MAP_D, --d 로 바꿈
    cx, cy, cyaw = a.cube
    tx, ty = a.target
    tyaw = cyaw if a.target_yaw is None else a.target_yaw
    level, _, boxes = _stack_args(a, P)
    lift_z = level * 2.0 * float(s.E.OBJECT_HALF_SIZE) + (STACK_GAP if level else 0.0)   # 놓을 때 큐브 중심이 책상 위 큐브보다 이만큼 위
    if hz and (level or boxes):
        return {**out, "status": "unreachable", "reason": "탑 쌓기(다른 큐브 위·옆)는 수평 집기로 계획하지 않음 — 수직 또는 자동"}
    # 1) 지도와 같은 판정 (그 점 그대로, MuJoCo FK 검증 포함)
    for what, x, y in (("큐브", cx, cy), ("놓을 자리", tx, ty)):
        up = lift_z if what == "놓을 자리" else 0.0
        if up:
            what = f"놓을 자리 {level + 1}층 (책상 위 {up * 100:.1f} cm)"
        if not P.on_table(s.to_sim([x, y, 0.0])):
            return {**out, "status": "unreachable", "reason": f"{what} ({x:+.3f}, {y:+.3f}): {P.off_table_why()}"}
        code = 2 if (hz or up or (what == "큐브" and P.pick_drop)) else s.grasp_eval(P.S, x, y, verify=True, check=False)[0]   # 팔이 닿나만 (2026-11-23: 집기 판정은 아래 실제 방향으로). 수평은 손목이 큐브 위가 아니라 옆
        if code == 0:
            return {**out, "status": "unreachable", "reason": f"{what} ({x:+.3f}, {y:+.3f}): {REASON[code]}"}
        down = P.pick_drop if what == "큐브" else 0.0   # 집는 자리만 PICK_DROP_M 아래
        ok, why, _ = P.point_check(np.asarray(s.to_sim([x, y, float(s.E.OBJECT_HALF_SIZE) - D + up - down])),
                                   yaws=[(cyaw if what == "큐브" else tyaw) + R.SIM_FROM_REAL_YAW], style=style)
        if not ok:
            note = f" (집는 자리를 모델 책상보다 {P.pick_drop * 100:g} cm 아래로 계획 중 — --pick-drop)" if (down and what == "큐브") else ""
            return {**out, "status": "unreachable", "reason": f"{what} ({x:+.3f}, {y:+.3f}): {why}{note}"}
    zc = float(s.E.OBJECT_HALF_SIZE) - D   # 큐브 중심 높이 (실좌표, 어깨 축 기준). 놓을 때도 큐브 바닥 = 책상
    pc = np.asarray(s.to_sim([cx, cy, zc - P.pick_drop]))      # 큐브 중심 (sim) — 집는 자리는 PICK_DROP_M 아래 — 손가락↔큐브 검사·들고 가는 큐브
    pt = np.asarray(s.to_sim([tx, ty, zc + lift_z]))   # 탑 쌓기면 그 층 높이
    gc, gt = (P.h_grip_target(pc), P.h_grip_target(pt)) if hz else (P.grip_target(pc), P.grip_target(pt))   # 집는 점 목표 = 큐브 중심 + GRIP_ABOVE_CENTER (수평은 H_GRIP_ABOVE)
    ycube = cyaw + R.SIM_FROM_REAL_YAW        # sim yaw (mod 90 만 쓴다)
    yplace = tyaw + R.SIM_FROM_REAL_YAW
    # 2) 경유점 후보
    if hz:   # 수평: (자세, 밀어 넣는 방향) — 큐브 면 네 방향 중 팔이 닿는 것
        Gh, Plh = P.candidates_h(pc, ycube), P.candidates_h(pt, yplace)
        G, Pl = [q for q, _ in Gh], [q for q, _ in Plh]
    else:
        G = P.family(P.candidates(gc, ycube))
        Pl = P.family(P.candidates(gt, yplace))
    for name, c in (("큐브를 잡는 자세", G), ("놓는 자세", Pl)):
        if not c:
            return {**out, "status": "unreachable", "reason": f"{name}: " + ("수평으로 손을 댈 수 있는 큐브 면이 없음" if hz else "손 방향(yaw)까지 맞는 해 없음")}
    tilt_of = lambda q: s.axis_tilt(s.fk(q)[1])   # noqa: E731  손 축이 수직에서 기운 각 (°)
    if vt:   # 수직: 손 축이 수직에서 VERT_MAX_TILT 안인 해만
        for name, c in (("큐브를 잡는 자세", G), ("놓는 자세", Pl)):
            if not any(tilt_of(q) <= VERT_MAX_TILT for q in c):
                return {**out, "status": "unreachable",
                        "reason": f"{name}: 손을 수직({VERT_MAX_TILT:g}° 안)으로 못 세움 (가장 곧은 해 {min(tilt_of(q) for q in c):.0f}°)"}
        G = [q for q in G if tilt_of(q) <= VERT_MAX_TILT]
        Pl = [q for q in Pl if tilt_of(q) <= VERT_MAX_TILT]
    # 집는 점은 그대로(큐브 중심 + GRIP_ABOVE_CENTER), 손가락이 책상 위 TABLE_MARGIN 을 지키는 해만. 없으면 그 자리는 집을 수 없음
    for name, c in ((("큐브를 잡는 자세", G), ("놓는 자세", Pl)) if not hz else ()):
        if not any(P.grasp_clearance(q) >= TABLE_MARGIN for q in c):
            best = max(c, key=P.grasp_clearance)
            return {**out, "status": "unreachable",
                    "reason": f"{name}: 손가락이 책상 위 {TABLE_MARGIN * 1000:.0f} mm 를 못 지킴 (가장 좋은 해 {P.grasp_clearance(best) * 1000:+.1f} mm, "
                              f"손 기울기 {s.axis_tilt(s.fk(best)[1]):.0f}°)"}
    if not hz:
        G = [q for q in G if P.grasp_clearance(q) >= TABLE_MARGIN]
        Pl = [q for q in Pl if P.grasp_clearance(q) >= TABLE_MARGIN]

    def tilt_groups(c):
        """손 기울기가 작은 것부터 1° 묶음 (2026-11-24 사용자: 여러 자세가 되면 수직에 가장 가까운 것 — 기울기가 첫째, 관절 이동은 그다음).
        hand_roll 만 다른 해는 기울기가 같아 한 묶음, 다른 팔 가지는 다른 묶음."""
        c = sorted(c, key=tilt_of)
        out_ = []
        for q in c:
            if out_ and tilt_of(q) - tilt_of(out_[-1][0]) <= 1.0:
                out_[-1].append(q)
            else:
                out_.append([q])
        return out_
    G_groups = [Gh] if hz else tilt_groups(G)
    Pl_groups = [Plh] if hz else tilt_groups(Pl)

    def floor_bad(fr):
        """수평: 누운 손가락이 책상 위 H_TABLE_MARGIN 아래인 프레임이 있나 (책상 위에 있는 구간만 넘긴다)."""
        return hz and any(P.hand_floor(q, f) < H_TABLE_MARGIN - 1e-4 for q, f in fr)
    home = P.home    # 팀 계획과 같은 시작·끝 (env HOME_ARM_QPOS, shoulder_yaw 6.3°)
    home0 = home.copy()
    home0[s.arm.index("shoulder_yaw")] = 0.0   # 해석 IK 는 shoulder_yaw 0 고정 → HOME 에서 먼저 그것만 0 으로
    hx = P.exit[-1]   # 손목 길(transit)이 시작·끝나는 HOME 쪽 자세 (HOME = 0 이면 책상 위 경유점, 예전 HOME 이면 HOME0)
    if not P.exit_ok:
        return {**out, "status": "unreachable", "reason": P.exit_why}
    sp = float(a.max_speed)
    O, C = P.open, P.closed
    OX = P.open_exit      # HOME 쪽 경유점 길 (좁게 벌림). 손목 길(HOME→큐브 위) 동안 OX → O 로 넓힘, 돌아올 땐 O → OX

    def leg(path, qf, qt, what, fb):
        if path is None:   # 손목 높이 유지 길을 못 찾음: 관절 공간 직선 (검사가 책상·큐브 닿음을 잡는다)
            fb.append(what)
            return [qf.copy(), qt.copy()]
        return path

    # 2) 큐브 위·놓을 자리 위 경유점: 손가락 방향 그대로 위로 빠지는 길부터. 고르는 순서 (욕심쟁이, 집기 먼저):
    #    집기 = HOME→큐브 위 길 + 내려가기 의 관절 이동 합이 가장 작은 (큐브 위, 잡는 자세) 가지, 벌린 손가락이 큐브에 안 닿으면 확정
    #    놓기 = 그 큐브 위 자세에서 놓을 자리 위로 + 내려가기 + HOME 으로 의 합이 가장 작은 것, 빠질 때 놓은 큐브에 안 닿으면 확정
    ref_axis = lambda c: P.hand_axis(min(c, key=lambda q: float(np.abs(q - hx).max())))   # noqa: E731  손목 길 시작(HOME 쪽) 자세에 가장 가까운 잡는 자세의 손 축
    target_pos = np.array([pt[0], pt[1], pt[2]])   # 놓인 큐브 중심 (탑 쌓기면 그 층)

    def obst_low(fr):   # 다른 큐브와 팔·손 사이 가장 짧은 거리 (없으면 9)
        return min((P.obstacle_gap(q, f, boxes) for q, f in fr), default=9.0) if boxes else 9.0

    def obst_pen(q):   # 잡는·놓는 자세가 다른 큐브에 닿으면 벌점 (table_penalty 처럼 — 닿지 않는 손 방향을 고르게)
        # 닫는 중간이 가장 넓다 (벌린 손가락 −60° 가 수평을 지나며 닫힘) → 벌림~쥠 사이 5 단계를 본다
        return FALLBACK_COST if boxes and min(P.obstacle_gap(q, O + (C - O) * u, boxes) for u in (0.0, 0.25, 0.5, 0.75, 1.0)) < OBST_MARGIN else 0.0

    stack = bool(level or boxes)

    def split(path, at):   # 탑 쌓기: 손목 길을 둘로 (손가락을 탑에서 먼 쪽에서만 벌리고·좁히게). 길이 짧거나 없으면 None
        k = int(round((len(path) - 1) * at)) if path is not None else 0
        return (path[:k + 1], path[k:]) if 1 <= k < (len(path) - 1 if path is not None else 0) else None

    def pick_part(ap, clean_only):
        """clean_only: 이미 대체(안 깨끗한) 계획이 있으면 길을 못 찾은 후보는 아예 안 푼다 (시간)."""
        best = None
        # 수평(hz): 손목 길 끝에서 손목 두 관절을 큐브 옆 자세(누운 손) 값으로 섞는다 (t_home) — 길 동안은 손이 아래
        t1s = [(qa, qg, d1, up, P.transit(hx, qa, f_home=P.exit_fh, t_home=hz)) for qa, qg, d1, up in ap["pairs"]]
        if any(x[4] for x in t1s):
            t1s = [x for x in t1s if x[4]]
        elif clean_only:
            return None
        for qa, qg, d1, up, t1 in t1s:
            c = P.arc(t1 or [hx, qa]) + P.arc(d1) + (0.0 if t1 else FALLBACK_COST) + (0.0 if hz else P.table_penalty(qg)) + obst_pen(qg)
            if best is None or c < best[0]:
                best = (c, qa, qg, t1, d1, up)
        if best is None:
            return None
        c, qa, qg, t1, d1, up = best
        fr, fb = [(home, OX)], []
        if float(np.abs(home - home0).max()) > 0:   # 예전 HOME(shoulder_yaw 6.3°) 만. HOME = 모든 관절 0 이면 같은 자세
            fr += P.frames([home, home0], OX, OX, sp)
        fr += P.exit_frames(OX, sp)   # HOME = 0 이면 관절 경유점 3 개 (Planner._home_exit), 예전 HOME 이면 없음
        i_exit = len(fr) - 1
        if hz:   # 수평: 손목 길 동안 손이 누우며 큐브 가까이 내려온다 → 떠나기 전에 다 벌리고 간다 (지도 _h_yaw_ok 도 다 벌린 손으로 본다)
            fr += P.frames([hx, hx], OX, O, sp, GRIP_S)
        sp1 = split(t1, 0.6) if stack else None
        if sp1:   # 탑 쌓기: 가는 길 앞 60 % 는 좁게 (탑 옆을 지날 수 있다), 큐브 가까이서 벌림
            fr += P.frames(sp1[0], OX, OX, sp)
            fr += P.frames(sp1[1], OX, O, sp)
        else:
            fr += P.frames(leg(t1, hx, qa, "HOME→큐브 위", fb), O if hz else OX, O, sp)
        i_above = len(fr) - 1
        fr += P.frames(d1, O, O, sp)
        hit = P.cube_hits(fr, range(len(fr)), pc, ycube)
        bad = bool(hit or fb or P.below_table(fr) or floor_bad(fr[i_exit:]) or obst_low(fr[i_exit:]) < OBST_MARGIN)
        return {"bad": bad, "c": c, "qa": qa, "qg": qg, "d1": d1, "up": up, "frames": fr, "above": i_above, "exit": i_exit, "hit": hit, "ap": ap, "fb": fb}

    def place_part(pk, bp, clean_only):
        best = None
        qa = pk["up"][-1]   # 든 뒤 자세 (위에서 집기 = 큐브 위 자세 그대로, 수평 = 곧게 든 자세)
        # 놓기는 집기의 거울: 드는 길을 거꾸로 내려가 놓고(qb = 그 길의 위 끝), 내려가는(밀어 넣는) 길을 거꾸로 빠진다(qo = 그 길의 시작)
        ts = [(up2[-1], qp, up2[::-1], P.transit(qa, up2[-1], f_home=hz, t_home=hz) or P.via_home(qa, up2[-1], blend=hz),
               P.transit(qo, hx, f_home=hz, t_home=P.exit_fh), d2[::-1], qo) for qo, qp, d2, up2 in bp["pairs"]]
        if any(x[3] and x[4] for x in ts):
            ts = [x for x in ts if x[3] and x[4]]
        elif clean_only:
            return None
        for qb, qp, d2, t2, t3, out2, qo in ts:
            c = (P.arc(t2 or [qa, qb]) + P.arc(d2) + P.arc(out2) + P.arc(t3 or [qo, hx])
                 + (0.0 if t2 else FALLBACK_COST) + (0.0 if t3 else FALLBACK_COST) + (0.0 if hz else P.table_penalty(qp)) + obst_pen(qp))
            if best is None or c < best[0]:
                best = (c, qb, qp, t2, d2, t3, out2, qo)
        if best is None:
            return None
        c, qb, qp, t2, d2, t3, out2, qo = best
        qg = pk["qg"]
        fr, mk = list(pk["frames"]), {"start": 0, "above_pick": pk["above"], "grasp": len(pk["frames"]) - 1}
        if len(P.exit) > 1:
            mk = {"start": 0, "exit_out": pk["exit"], **{k: v for k, v in mk.items() if k != "start"}}
        fb = list(pk["fb"])

        def add(qs, fa, fz, key, min_s=0.0):
            fr.extend(P.frames(qs, fa, fz, sp, min_s))
            mk[key] = len(fr) - 1
        add([qg, qg], O, C, "close", GRIP_S)
        add([qg, qg], C, C, "closed", GRIP_HOLD_S)
        add(pk["up"], C, C, "lift")
        add(leg(t2, qa, qb, "큐브 위→놓을 자리 위", fb), C, C, "above_place")
        add(d2, C, C, "place")
        add([qp, qp], C, O, "open", GRIP_S)
        add([qp, qp], O, O, "opened", GRIP_HOLD_S)
        add(out2, O, O, "lift2")
        n_tail, mk_tail = len(fr), dict(mk)
        # 탑 쌓기: 돌아가는 길에서 손가락을 어디서 좁히나 — (길의 몇 할 지점, 그 자리에 서서 좁히나) 후보 중 탑·놓은 큐브에 안 닿는 첫 것
        opts = [(at, stand) for stand in (False, True) for at in (0.4, 0.5, 0.6, 0.7, 0.8, 0.9)] + [(None, False)]
        for at, stand in (opts if stack and t3 is not None and len(P.exit) > 1 else ((None, False),)):
            del fr[n_tail:]
            mk.clear(); mk.update(mk_tail)
            sp3 = split(t3, at) if at else None
            if len(P.exit) > 1:   # HOME = 0: 손목 길로 나감 경유점까지, 그다음 관절 경유점을 거꾸로 HOME 까지
                if sp3:   # 탑 쌓기: 돌아가는 길을 둘로 — 앞쪽은 벌린 채 (낮게 든 손은 좁히면 뒤따르는 손가락이 놓은 큐브를 스친다), 뒤쪽에서 좁힘
                    #       (HOME 쪽 경유점으로 내려갈 때 벌린 손가락이 탑에 닿지 않게). 나누는 자리는 탑·놓은 큐브에 안 닿는 첫 값
                    add(sp3[0], O, O, "away")
                    if stand:
                        add([sp3[0][-1], sp3[0][-1]], O, OX, "narrow", GRIP_S)
                    add(sp3[1], OX if stand else O, OX, "exit_back")
                else:
                    add(leg(t3, qo, hx, "놓을 자리 위→HOME", fb), O, O if hz else OX, "exit_back")
                if hz:   # 수평: 다 벌린 채 돌아와 책상 위 경유점에서 좁힌다
                    add([hx, hx], O, OX, "exit_back", GRIP_S)
                fr.extend(P.exit_frames(OX, sp, back=True))
                mk["home0"] = len(fr) - 1
            else:
                add(leg(t3, qo, home0, "놓을 자리 위→HOME", fb), O, OX, "home0")
            if float(np.abs(home - home0).max()) > 0:
                add([home0, home], OX, OX, "home")
            if at is None or not (P.cube_hits(fr, range(n_tail, mk["exit_back"] + 1), target_pos, yplace)
                                  or obst_low(fr[n_tail:mk["exit_back"] + 1]) < OBST_MARGIN):
                break
        hit = P.cube_hits(fr, range(mk["opened"], len(fr)), target_pos, yplace)
        bad = bool(hit or len(fb) > len(pk["fb"]) or P.below_table(fr[mk["grasp"]:]) or floor_bad(fr[mk["grasp"]:mk.get("exit_back", mk["lift2"]) + 1]))
        if boxes and not bad:   # 다른 큐브 (HOME 나가기 경유점 길은 집기 쪽에서 이미 봄 — 같은 길 거꾸로)
            bad = bool(obst_low(fr[mk["grasp"]:mk.get("exit_back", len(fr) - 1) + 1]) < OBST_MARGIN
                       or P.carried_cube_gap(qg, pc, ycube, [fr[i][0] for i in range(mk["closed"], mk["above_place"] + 1)], boxes) < OBST_CARRY_MARGIN
                       or P.carried_cube_gap(qg, pc, ycube, [fr[i][0] for i in range(mk["above_place"], mk["place"] + 1)], boxes) < -5e-4)
        return {"bad": bad, "c": c + pk["c"], "q": (qa, qg, qb, qp), "frames": fr, "marks": mk, "hit": hit, "bp": bp, "fb": fb}

    ie = s.arm.index("elbow")
    # 손 기울기가 작은 묶음부터 (수직에 가까운 자세가 첫째): 그 묶음에서 검사를 다 통과하는 길이 있으면 확정, 없으면 다음 묶음
    pk = first_pk = None
    tries_a = 0
    for grp in G_groups:
        n = 0
        for ap in (P.approaches_h(pc, grp) if hz else P.approaches(gc, grp, ref_axis(grp), vertical=vt)):
            tries_a += 1
            n += 1
            r = pick_part(ap, first_pk is not None)
            if r is not None:
                first_pk = first_pk or r
                if not r["bad"]:
                    pk = r
                    break
            if n >= MAX_TRIES:
                break
        if pk is not None:
            break
    pk = pk or first_pk
    if pk is None:
        return {**out, "status": "unreachable", "reason": (f"큐브 옆 {min(H_PUSH_CM):g}~{max(H_PUSH_CM):g} cm 에서 손을 수평으로 둔 채 곧게 밀어 넣고 드는 길이 없음" if hz else
                                                           f"큐브 위 {min(ABOVE_CM):g}~{max(ABOVE_CM):g} cm 에서 손을 {'수직으로' if vt else '아래로'} 둔 채 큐브까지 곧게 내려가는 길이 없음")}
    plan_ = first_pl = None
    tries_b = 0
    for grp in Pl_groups:
        n = 0
        for bp in (P.approaches_h(pt, grp) if hz else P.approaches(gt, grp, ref_axis(grp), vertical=vt)):
            tries_b += 1
            n += 1
            r = place_part(pk, bp, first_pl is not None)
            if r is not None:
                first_pl = first_pl or r
                if not r["bad"]:
                    plan_ = r
                    break
            if n >= MAX_TRIES:
                break
        if plan_ is not None:
            break
    plan_ = plan_ or first_pl
    if plan_ is None:
        return {**out, "status": "unreachable", "reason": (f"놓을 자리에서 손을 수평으로 둔 채 곧게 내려놓고 옆으로 {min(H_PUSH_CM):g}~{max(H_PUSH_CM):g} cm 빠지는 길이 없음" if hz else
                                                           f"놓을 자리 위 {min(ABOVE_CM):g}~{max(ABOVE_CM):g} cm 에서 손을 {'수직으로' if vt else '아래로'} 둔 채 곧게 내려가는 길이 없음")}
    fallback = plan_["fb"]
    ap, bp = pk["ap"], plan_["bp"]
    frames, mk = plan_["frames"], plan_["marks"]
    qa, qg, qb, qp = plan_["q"]
    ha, hb = ap["cm"], bp["cm"]
    g_tilt = tilt_of(qg)
    g_text = f"집는 방향: {ko} (기울기 {g_tilt:.0f}°)"
    if style == "tilted" and g_tilt <= VERT_MAX_TILT:   # 자동에서 수직 조건(곧게 수직으로 내려가기·놓는 손도 수직)은 못 채웠지만 잡는 손은 곧다
        g_text = f"집는 방향: {ko} (잡는 손 기울기 {g_tilt:.0f}°, 놓는 손 {tilt_of(qp):.0f}° · 비스듬히 내려가거나 놓는 손이 기울어 '수직' 조건 밖)"
    labels = {"start": "start (HOME)", "above_pick": f"큐브 위 (+{ha:g} cm, 수직에서 {ap['tilt_deg']}°)", "grasp": "내려감: 큐브 잡는 자세" + (f" (모델 책상보다 {P.pick_drop * 100:g} cm 아래)" if P.pick_drop else ""),
              "close": "손가락 닫기", "closed": "기다림", "lift": "들기", "above_place": f"놓을 자리 위 (+{hb:g} cm, 수직에서 {bp['tilt_deg']}°)",
              "place": "내려감: 놓는 자세", "open": "손가락 열기", "opened": "기다림", "lift2": "들기",
              "home0": "HOME (shoulder_yaw 만 0)" if "home" in mk else "HOME", "home": "HOME",
              "exit_out": "HOME 나감 경유점 (팔 돌림 → 어깨 젖히고 팔꿈치 접기 → 책상 위)", "exit_back": "HOME 들어가기 전 경유점 (책상 위)",
              "away": "탑에서 멀어짐 (손가락 벌린 채)", "narrow": "서서 손가락 좁힘"}
    if hz:
        labels.update({"above_pick": f"큐브 옆 {ap['push_cm']:g} cm 의 위 (+{ha:g} cm, 손 수평)", "grasp": f"내려와 {ap['push_cm']:g} cm 밀어 넣음: 큐브 잡는 자세", "lift": f"들기 (곧게 위로 {ha:g} cm, 손 수평)",
                       "above_place": f"놓을 자리 위 (+{hb:g} cm, 손 수평)", "lift2": f"옆으로 {bp['push_cm']:g} cm 빠진 뒤 위로"})
    # 3) 검사: 손가락↔큐브, 손가락↔책상, 관절 한계, 잡는 점 오차, 패드↔큐브 면 각, 들고 갈 때 큐브 바닥 높이
    hit_pick = P.cube_hits(frames, range(0, mk["grasp"] + 1), pc, ycube)
    hit_place = P.cube_hits(frames, range(mk["opened"], len(frames)), target_pos, yplace)
    lim = P.lim
    low = [P.finger_low(q, f) for q, f in frames]
    arm_lo = [P.arm_low(q) for q, _ in frames]   # 팔 기준점(팔꿈치·손목)도 책상 발자국 위에선 책상 면 위 (2026-11-21)
    below = [i for i, z in enumerate(low) if z < s.table_z + TABLE_MARGIN or arm_lo[i] < s.table_z]
    col = [P.column_low(q, f) for q, f in frames]   # 팔·손 ↔ 기둥 (2026-11-21)
    near_col = [i for i, c in enumerate(col) if c < COLUMN_MARGIN]
    names = list(s.arm) + list(s.fingers)
    traj_q = [list(map(float, q)) + list(map(float, f)) for q, f in frames]
    sy = s.arm.index("shoulder_yaw")   # 0 고정 (HOME 만 6.3°: 그 사이 보간은 범위 안)
    over = [n for k, n in enumerate(s.arm) if k != sy and any(not (lim[k, 0] - 1e-4 <= q[k] <= lim[k, 1] + 1e-4) for q, _ in frames)]
    grip_err = float(np.linalg.norm(P.grip(qg) - gc))
    place_err = float(np.linalg.norm(P.grip(qp) - gt))

    def pad_face(q, yaw):
        """손가락이 닫히는 축(site x)과 가장 가까운 큐브 옆면 법선 사이 3D 각 (°). 손이 기울면 0 이 아닐 수 있다."""
        x = s.fk(q)[1][:, 0]
        return min(math.degrees(math.acos(min(1.0, abs(float(x @ [math.cos(math.radians(yaw + k)), math.sin(math.radians(yaw + k)), 0.0])))))
                   for k in (0, 90))

    def side(i0, i1, p):
        return max(float(np.linalg.norm((P.grip(frames[i][0]) - p)[:2])) for i in range(i0, i1 + 1))
    carry_low = P.carried_cube_low(qg, pc, ycube, [frames[i][0] for i in range(mk["lift"], mk["above_place"] + 1)]) - s.table_z
    spd = np.degrees(np.abs(np.diff(np.array(traj_q), axis=0)).max(axis=0)) / DT
    problems = []
    obst = None
    if boxes:   # 다른 큐브: 팔·손(모든 프레임), 들고 가는 큐브(옮길 때 여유, 내려놓을 때 안 파고듦)
        og = [P.obstacle_gap(q, f, boxes) for q, f in frames]
        near_ob = [i for i, g in enumerate(og) if g < OBST_MARGIN]
        cg = P.carried_cube_gap(qg, pc, ycube, [frames[i][0] for i in range(mk["closed"], mk["above_place"] + 1)], boxes)
        dg = P.carried_cube_gap(qg, pc, ycube, [frames[i][0] for i in range(mk["above_place"], mk["place"] + 1)], boxes)
        obst = {"arm_min_mm": round(min(og) * 1000, 1), "carry_cube_min_mm": round(cg * 1000, 1), "lower_cube_min_mm": round(dg * 1000, 1)}
        if near_ob:
            problems.append(f"팔·손이 다른 큐브에 {OBST_MARGIN * 1000:.0f} mm 보다 가까이 감: 프레임 {near_ob[0]}~{near_ob[-1]} ({len(near_ob)}개), 가장 가까이 {min(og) * 1000:+.1f} mm")
        if cg < OBST_CARRY_MARGIN:
            problems.append(f"들고 가는 큐브가 다른 큐브에 {cg * 1000:+.1f} mm (여유 {OBST_CARRY_MARGIN * 1000:.0f} mm 아래)")
        if dg < -5e-4:
            problems.append(f"내려놓는 길에서 든 큐브가 다른 큐브를 {-dg * 1000:.1f} mm 파고듦")
    i_tab0, i_tab1 = mk.get("exit_out", 0), mk.get("exit_back", mk["lift2"])   # 손이 책상 위에 있는 구간 (HOME 나가기 경유점 뒤 ~ 들어가기 전)
    if hz:
        fl = [P.hand_floor(q, f) for q, f in frames[i_tab0:i_tab1 + 1]]
        if min(fl) < H_TABLE_MARGIN - 1e-4:
            problems.append(f"수평: 누운 손가락이 책상 위 {min(fl) * 1000:+.1f} mm (여유 {H_TABLE_MARGIN * 1000:.0f} mm 아래)")
        he = max(max(P.h_err(frames[i][0], a_)) for i0, i1, a_ in ((mk["above_pick"], mk["lift"], -P.hand_axis(qg)), (mk["above_place"], mk["lift2"], -P.hand_axis(qp)))
                 for i in range(i0, i1 + 1))
        if he > H_TOL + 1.0:
            problems.append(f"수평: 밀어 넣고 드는 동안 손이 수평에서 {he:.1f}° 벗어남 (한계 {H_TOL:g}°)")
    if vt:
        vmax = max(tilt_of(frames[i][0]) for i0, i1 in ((mk["above_pick"], mk["lift"]), (mk["above_place"], mk["lift2"])) for i in range(i0, i1 + 1))
        if vmax > VERT_MAX_TILT + 1.0:
            problems.append(f"수직: 내려가고 드는 동안 손이 수직에서 {vmax:.1f}° 기울어짐 (한계 {VERT_MAX_TILT:g}°)")
    if hit_pick:
        problems.append(f"벌린 손가락이 큐브에 닿음 (집으러 갈 때): 프레임 {hit_pick[0]}~{hit_pick[-1]} ({len(hit_pick)}개)")
    if hit_place:
        problems.append(f"손가락이 놓은 큐브에 닿음 (빠질 때): 프레임 {hit_place[0]}~{hit_place[-1]} ({len(hit_place)}개)")
    if below:
        problems.append(f"손가락이 책상 위 여유 {TABLE_MARGIN * 1000:.0f} mm 보다 낮게 감: 프레임 {below[0]}~{below[-1]} ({len(below)}개), "
                        f"가장 낮게 책상 위 {(min(low) - s.table_z) * 1000:+.1f} mm")
    if over:
        problems.append("관절 한계 밖: " + ", ".join(over))
    if near_col:
        problems.append(f"팔·손이 기둥에 {COLUMN_MARGIN * 1000:.0f} mm 보다 가까이 감: 프레임 {near_col[0]}~{near_col[-1]} ({len(near_col)}개), "
                        f"가장 가까이 {min(col) * 1000:+.1f} mm")
    if carry_low < TABLE_MARGIN:
        problems.append(f"들고 가는 큐브의 가장 낮은 모서리가 책상 위 {carry_low * 1000:+.1f} mm (여유 {TABLE_MARGIN * 1000:.0f} mm 아래)")
    if fallback:
        problems.append("손목 높이 유지 길을 못 찾아 관절 공간 직선으로 감: " + ", ".join(fallback))
    if grip_err > TIP_TOL or place_err > TIP_TOL:
        problems.append(f"집는 점이 목표(큐브 중심 +{GRIP_ABOVE_CENTER * 100:g} cm)에서 잡을 때 {grip_err * 1000:.1f} mm, 놓을 때 {place_err * 1000:.1f} mm")
    status = "playable" if not problems else ("finger below table" if below else "finger hits cube" if (hit_pick or hit_place) else "check failed")
    wp = [{"label": labels[k], "frame": i, "t_s": round(i * DT, 2),
           "joints_deg": {n: round(math.degrees(float(v)), 2) for n, v in zip(s.arm, frames[i][0])},
           "grip_real": [round(float(v), 4) for v in s.to_real(P.grip(frames[i][0]))],
           "hand_yaw_real_deg": round(P.hand_yaw(frames[i][0]) - R.SIM_FROM_REAL_YAW, 1),
           "hand_tilt_deg": round(s.axis_tilt(s.fk(frames[i][0])[1]), 1),
           "fingers": "closed" if np.allclose(frames[i][1], C) else "open"} for k, i in mk.items()]
    out["sim"] = {"cube": pc.tolist(), "target": pt[:2].tolist(), "cube_yaw_deg": ycube,
                  "hand_yaw_deg": round(P.hand_yaw(qg), 2), "hand_tilt_deg": round(s.axis_tilt(s.fk(qg)[1]), 2)}
    return {**out, "status": status,
            "grasp": {"style": style, "tilt_deg": round(g_tilt, 1), "place_tilt_deg": round(tilt_of(qp), 1), "text": g_text},
            "reason": f"해석 IK 계획 (지도와 같은 IK, 물리·쥐기 시뮬 없음) · {g_text}" if not problems else "; ".join(problems),
            "playable": status == "playable", "delivered_in_sim": None, "retries": [], "final_error_m": None,
            "joint_names": names, "dt": DT, "positions": traj_q, "unit": "rad",
            "peak_speed_deg_s": {"joint": names[int(spd.argmax())], "value": float(spd.max())},
            "waypoints": wp, "marks": dict(mk), "problems": problems,   # marks = {경유점: 닿는 프레임} → 재생 중 '지금 무슨 동작' (play_phase)
            "checks": {"grasp_grip_error_mm": round(grip_err * 1000, 2), "place_grip_error_mm": round(place_err * 1000, 2),
                       "pad_face_deg": {"pick": round(pad_face(qg, ycube), 1), "place": round(pad_face(qp, yplace), 1)},
                       "lowest_finger_above_table_mm": round((min(low) - s.table_z) * 1000, 1),
                       "column_min_mm": round(min(col) * 1000, 1),
                       **({"obstacles": obst} if obst else {}),
                       "finger_cube_hits": {"pick": len(hit_pick), "place": len(hit_place)},
                       "descent_side_max_mm": {"pick": round(side(mk["above_pick"], mk["grasp"], pc) * 1000, 1),
                                               "place": round(side(mk["above_place"], mk["place"], pt) * 1000, 1)},
                       "carry_cube_bottom_min_above_table_mm": round(carry_low * 1000, 1),
                       "approach": {"pick": {"cm": ha, "tilt_deg": ap["tilt_deg"], "azimuth_deg": ap["azimuth_deg"], "tried": tries_a},
                                    "place": {"cm": hb, "tilt_deg": bp["tilt_deg"], "azimuth_deg": bp["azimuth_deg"], "tried": tries_b}},
                       "path_joint_move_deg": round(math.degrees(plan_["c"]), 1),
                       "home_exit": {"waypoints_deg": [[round(math.degrees(float(v)), 1) for v in q] for q in P.exit[1:]],
                                     "lowest_finger_above_table_mm": P.exit_clear_mm},
                       "joint_space_fallback": fallback,
                       "n_candidates": {"above_pick": len(ap["pairs"]), "grasp": len(G), "above_place": len(bp["pairs"]), "place": len(Pl)}}}


HOME_AT_DEG = 1.0        # 팔 관절이 모두 HOME 에서 이 안이면 '이미 HOME'
HOME_NEAR_EXIT_DEG = 5.0 # 지금 자세가 HOME 나가기 경유점 하나에서 이 안이면 손목 길 없이 그 경유점부터 거꾸로
_HOME_PLANNER = {}
_HOME_LOCK = threading.Lock()   # 캐시한 Planner 하나가 MuJoCo MjData 를 같이 쓴다 → 동시에 두 계획이면 서로의 검사(qpos)를 덮어씀


def plan_home(now_deg, ik=None, cal_file=None, max_speed=30.0, d=None):
    """_plan_home 을 한 번에 하나씩 (폰·PC 가 거의 같이 HOME 을 눌러도 검사가 섞이지 않게). d = 책상 높이 (None = 기본)."""
    with _HOME_LOCK:
        return _plan_home(now_deg, ik, cal_file, max_speed, d)


def _plan_home(now_deg, ik=None, cal_file=None, max_speed=30.0, d=None):
    """지금 자세 → HOME(모든 관절 0) 궤적 (2026-11-21 '원점 자세로 가기' 버튼). now_deg = {관절: sim °}.
    계획(pick_plan)이 HOME 으로 돌아오는 길과 같게: (shoulder_yaw 만 0) → 손목 길(transit)로 HOME 나가기 경유점의 책상 위 자리
    → 나가기 경유점을 거꾸로 → HOME. 손가락은 첫 구간 동안 벌림. 곧장 관절 직선으로 HOME 에 가면 앞팔이 책상 모서리를 파고들어서
    (_home_exit 설명) 쓰지 않는다 — 길을 못 찾으면 거절. 결과는 pick_plan 궤적과 같은 모양 (재생·검사 코드를 그대로 씀)."""
    cal_file = cal_file or (REPO / "arm_control" / "calibration.json")
    key = (str(ik), str(cal_file), None if d is None else round(float(d), 4))   # 계획기 하나 = 책상 높이 하나
    if key not in _HOME_PLANNER:
        if len(_HOME_PLANNER) >= 3:   # 계획기마다 MuJoCo 모델 하나 — 높이를 여러 번 바꿔도 쌓이지 않게
            _HOME_PLANNER.clear()
        _HOME_PLANNER[key] = Planner(_find_ik(ik), _cmd_ranges(cal_file), d=d)
    P = _HOME_PLANNER[key]
    s, np = P.s, P.np
    names = list(s.arm) + list(s.fingers)
    out = {"planner": "analytic", "kind": "home", "joint_names": names, "dt": DT, "unit": "rad"}
    miss = [n for n in s.arm if n not in now_deg]
    if miss:
        return {**out, "status": "unreachable", "reason": "지금 각을 모르는 관절: " + ", ".join(miss) + " (AK45 피드백이 없음)"}
    q0 = np.radians([float(now_deg[n]) for n in s.arm])
    f0 = np.array([math.radians(float(now_deg[n])) if n in now_deg else float(o) for n, o in zip(s.fingers, P.open_exit)])
    home = P.exit[0]
    if float(np.degrees(np.abs(q0 - home)).max()) <= HOME_AT_DEG and float(np.degrees(np.abs(f0 - P.open_exit)).max()) <= HOME_AT_DEG:
        return {**out, "status": "already", "reason": "이미 HOME (모든 관절 0)"}
    if not P.exit_ok:
        return {**out, "status": "unreachable", "reason": P.exit_why}
    sy = s.arm.index("shoulder_yaw")
    pre = []                        # 관절 꺾은선 목록 (구간마다 따로 사다리꼴 = 모서리에서 멈춤)
    q1 = q0.copy()
    if abs(q1[sy]) > math.radians(0.5):
        q1[sy] = 0.0
        pre.append([q0, q1])        # 해석 IK 는 shoulder_yaw 0 고정 → 먼저 그것만 0
    ex = P.exit                     # [HOME, h1, mid, 책상 위 손목 자리]

    def back_from(k):               # ex[k] → … → HOME
        return [[a, b] for a, b in zip(ex[k::-1], ex[k - 1::-1])] if k > 0 else []

    cands = []                      # (길 이름, 꺾은선 목록) — 차례대로, 검사를 통과한 첫 길
    # ① 나가기 경유점 꺾은선(HOME 나가는·들어오는 중) 위나 가까이: 거기서 꺾은선을 거꾸로 (손목 길은 이 자세들에서 책상을 판다)
    best = None                     # (관절 최대 차 °, 구간 j, 그 구간 위 점)
    for j in range(len(ex) - 1):
        a, b = ex[j], ex[j + 1]
        for u in np.linspace(0.0, 1.0, 201):
            p = a + (b - a) * u
            d = float(np.degrees(np.abs(q1 - p)).max())
            if best is None or d < best[0] - 1e-9 or (abs(d - best[0]) <= 1e-9 and j > best[1]):
                best = (d, j, p)
    if best and best[0] <= HOME_NEAR_EXIT_DEG:
        _, j, p = best
        cands.append((f"HOME 나가기 경유점 길 위 (구간 {j + 1}) → 거꾸로",
                      [[q1, p, ex[j]] if best[0] > 1e-6 else [q1, ex[j]]] + back_from(j)))
    # ② 손목 길(계획이 HOME 으로 돌아올 때와 같은 길)로 책상 위 손목 자리 → 나가기 경유점 거꾸로
    path = P.transit(q1, ex[-1], t_home=P.exit_fh)
    if path is not None:
        cands.append(("손목 길 → HOME 나가기 경유점 거꾸로", [path] + back_from(len(ex) - 1)))
    # ③ 관절 직선 (팔이 책상 옆에 늘어져 있을 때 등). 아래 책상 검사를 통과할 때만
    cands.append(("관절 직선 (책상·기둥 검사 통과)", [[q1, ex[0]]]))
    sp = float(max_speed)
    lim = P.lim
    first = None
    for route, legs in cands:
        legs = pre + legs
        frames = [(q0.copy(), f0.copy())]
        ends = []                   # 구간마다 끝 프레임 → marks (재생 중 '지금 무슨 동작', play_phase.HOME_TEXT)
        for i, qs in enumerate(legs):
            frames += P.frames(qs, f0 if i == 0 else P.open_exit, P.open_exit, sp)
            ends.append(len(frames) - 1)
        mk = {"start": 0}
        if pre:
            mk["yaw0"] = ends[0]
        if len(ends) > len(pre) + 1:
            mk["exit_back"] = ends[len(pre)]   # 첫 길(손목 길·경유점 길 위)의 끝 = 나가기 경유점, 그 뒤는 경유점을 거꾸로
        mk["home"] = ends[-1]
        low = [P.finger_low(q, f) for q, f in frames]
        arm_lo = [P.arm_low(q) for q, _ in frames]
        below = [i for i, z in enumerate(low) if z < s.table_z + TABLE_MARGIN or arm_lo[i] < s.table_z]
        over = [n for j, n in enumerate(s.arm) if j != sy and any(not (lim[j, 0] - 1e-4 <= q[j] <= lim[j, 1] + 1e-4) for q, _ in frames)]
        col = [P.column_low(q, f) for q, f in frames]
        near_col = [i for i, c in enumerate(col) if c < COLUMN_MARGIN]
        problems = []
        if near_col:
            problems.append(f"팔·손이 기둥에 {COLUMN_MARGIN * 1000:.0f} mm 보다 가까이 감: 프레임 {near_col[0]}~{near_col[-1]}, 가장 가까이 {min(col) * 1000:+.1f} mm"
                            + (" — 지금 자세부터 이미 가까움" if near_col[0] == 0 else ""))
        if below:
            problems.append(f"손가락·팔이 책상 위 여유 {TABLE_MARGIN * 1000:.0f} mm 보다 낮게 감: 프레임 {below[0]}~{below[-1]} ({len(below)}개), "
                            f"가장 낮게 책상 위 {(min(low) - s.table_z) * 1000:+.1f} mm" + (" — 지금 자세부터 이미 낮음" if below[0] == 0 else ""))
        if over:
            problems.append("관절 한계 밖: " + ", ".join(over))
        r = {"route": route, "legs": legs, "frames": frames, "low": low, "below": below, "problems": problems, "col": col, "marks": mk}
        first = first or r
        if not problems:
            break
    else:
        r = first
    frames, problems, route = r["frames"], r["problems"], r["route"]
    if problems:
        tried = ", ".join(c[0] for c in cands)
        problems = [f"[{route}] " + p for p in problems] + [f"해 본 길: {tried} — 모두 검사에 걸림. 각도 제어로 손을 책상 위로 옮긴 뒤 다시"]
    traj_q = [list(map(float, q)) + list(map(float, f)) for q, f in frames]
    status = "playable" if not problems else ("finger below table" if r["below"] else "check failed")
    return {**out, "status": status, "playable": status == "playable",
            "reason": f"HOME 으로 ({route}, {len(frames) * DT:.1f}초)" if not problems else "; ".join(problems),
            "positions": traj_q, "problems": problems, "marks": r["marks"],
            "checks": {"joint_space_fallback": [], "lowest_finger_above_table_mm": round((min(r["low"]) - s.table_z) * 1000, 1),
                       "column_min_mm": round(min(r["col"]) * 1000, 1), "route": route, "legs": len(r["legs"])},
            "seconds": round(len(frames) * DT, 1)}


PLACE_FROM_PULL_DEG = 5.0   # plan_place_from: 시작 자세가 우리 관절 한계를 이 각(°) 안으로 넘으면 먼저 한계 안으로 (검사함), 더 넘으면 거절
PLACE_FROM_GAP = 0.002      # 놓을 때 손에 든 큐브의 가장 낮은 모서리가 책상 위 이만큼 (m) 에서 손을 편다
PLACE_FROM_HAND_M = 0.10    # 큐브 중심이 집는 점에서 이보다 멀면 '손에 없음'


def plan_place_from(q_start_deg, fingers_start_deg, cube_real, target_xy, target_yaw=0.0, d=None, ik=None, cal_file=None, max_speed=30.0):
    """손에 큐브를 든 아무 자세(강화학습 '들기' 가 끝난 ㄴ자 자세)에서 이어서: 놓을 자리 위로 → 내려감 → 손가락 열기 → 들기 → HOME.
    plan() 의 놓기 쪽과 같은 조각(candidates·approaches·transit·descent·exit_frames)을 쓴다. plan()·plan_home() 은 그대로다 (2026-11-24).

    q_start_deg {팔 관절: sim °}, fingers_start_deg {손가락 관절: °} — 들고 가는 동안 손가락은 이 값 그대로(정책이 쥔 값), 놓을 때 우리 벌림 값으로.
    cube_real = 지금 큐브 [x, y, z, qx, qy, qz, qw] (실좌표 = 3D 뷰 틀). 큐브는 시작 자세의 손(집는 점 틀)에 굳게 붙었다고 본다 (물리 없음):
    손이 돌면 같이 돈다. 놓는 자세 = 그 큐브의 중심이 target_xy(실좌표 m) 위, 가장 낮은 모서리가 책상 위 PLACE_FROM_GAP, 수평으로 눕혔을 때
    yaw ≡ target_yaw (mod 90). 손 안에서 기운 큐브(정책이 쥐면 10~15°)는 그 기울기로 놓인다 → release_tilt_deg 로 알린다.
    돌려주는 것은 plan() 과 같은 꼴(status·reason·joint_names·positions rad·dt) + cube(프레임마다 큐브 실좌표 자세) + marks."""
    t0 = time.time()
    cal_file = cal_file or (REPO / "arm_control" / "calibration.json")
    ranges = _cmd_ranges(cal_file)
    tx, ty = (float(v) for v in target_xy)
    tyaw = float(target_yaw or 0.0)
    out = {"planner": "analytic", "kind": "place_from", "real": {"target": [tx, ty], "target_yaw": tyaw}}

    def no(why):
        return {**out, "status": "unreachable", "reason": why}
    ik_dir = _find_ik(ik)
    if d is not None and not (math.isfinite(d) and D_RANGE[0] - 1e-9 <= d <= D_RANGE[1] + 1e-9):
        import hand_ik
        return no(d_range_text(d, hand_ik.TABLE_D_DEFAULT))
    P = Planner(ik_dir, ranges, d=d)
    s, np = P.s, P.np
    R = s.R
    out["real"]["d"] = P.table_d
    names = list(s.arm) + list(s.fingers)
    miss = [n for n in names if n not in {**q_start_deg, **fingers_start_deg}]
    if miss:
        return no("시작 각을 모르는 관절: " + ", ".join(miss))
    q0 = np.radians([float(q_start_deg[n]) for n in s.arm])
    F = np.radians([float(fingers_start_deg[n]) for n in s.fingers])
    # 1) 시작 자세 ↔ 우리 관절 한계 (calibration). shoulder_yaw 는 계획이 0 에 고정 → 먼저 그것만 0 으로
    sy = s.arm.index("shoulder_yaw")
    lim = P.lim.copy()
    if "shoulder_yaw" in ranges:
        lim[sy] = np.radians(ranges["shoulder_yaw"])
    tol = math.radians(0.5)
    over = [(n, math.degrees(float(q0[k])), math.degrees(float(lim[k, 0])), math.degrees(float(lim[k, 1])),
             math.degrees(float(max(lim[k, 0] - q0[k], q0[k] - lim[k, 1]))))
            for k, n in enumerate(s.arm) if q0[k] < lim[k, 0] - tol or q0[k] > lim[k, 1] + tol]
    out["start_over_limit"] = [{"joint": n, "deg": round(v, 2), "range_deg": [round(lo, 1), round(hi, 1)], "over_deg": round(e, 2)} for n, v, lo, hi, e in over]
    if any(e > PLACE_FROM_PULL_DEG for *_, e in over):
        return no("들기 끝 자세가 우리 관절 한계 밖: " + ", ".join(f"{n} {v:+.1f}° (한계 {lo:+.0f}~{hi:+.0f}°)" for n, v, lo, hi, e in over)
                  + f" — {PLACE_FROM_PULL_DEG:g}° 넘게 벗어나 이어서 계획하지 않음")
    q1 = np.clip(q0, lim[:, 0], lim[:, 1])
    q1[sy] = 0.0
    pre = [[q0, q1]] if float(np.degrees(np.abs(q1 - q0)).max()) > 0.05 else []   # 0.05° 안이면 그대로 시작 (정책 끝 shoulder_yaw −0.003° 같은 것)
    if not P.exit_ok:
        return no(P.exit_why)
    # 2) 손에 든 큐브: 시작 자세의 집는 점 틀에 붙인다
    a = math.radians(R.SIM_FROM_REAL_YAW)
    Rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])   # sim = Rz · real
    cq = np.array([cube_real[6], cube_real[3], cube_real[4], cube_real[5]], dtype=float)
    cq /= float(np.linalg.norm(cq)) or 1.0
    m9 = np.zeros(9)
    s.mj.mju_quat2Mat(m9, cq)
    pc0, rc0 = np.asarray(s.to_sim([float(v) for v in cube_real[:3]]), dtype=float), Rz @ m9.reshape(3, 3)
    ps0, rs0 = s.fk(q0)
    rel_p, rel_r = rs0.T @ (pc0 - ps0), rs0.T @ rc0
    half = float(s.E.OBJECT_HALF_SIZE)
    if float(np.linalg.norm(rel_p)) > PLACE_FROM_HAND_M:
        return no(f"큐브가 손에 없음 (집는 점에서 {np.linalg.norm(rel_p) * 100:.1f} cm)")

    def cube_at(q):
        p, r = s.fk(q)
        return p + r @ rel_p, r @ rel_r

    def low_ext(rc):   # 큐브 중심에서 가장 낮은 모서리까지 (m)
        return half * float(np.abs(rc[2]).sum())

    def flat(rc):      # 기운 큐브를 눕혔을 때: (수직에서 기울기 °, yaw ° — 90° 마다 같음)
        k = int(np.abs(rc[2]).argmax())
        c = rc[:, (k + 1) % 3]
        return math.degrees(math.acos(min(1.0, abs(float(rc[2, k]))))), math.degrees(math.atan2(c[1], c[0]))
    # 3) 놓을 자리
    pt = np.asarray(s.to_sim([tx, ty, half - P.table_d]), dtype=float)   # 놓인 뒤 큐브 중심 (sim)
    if not P.on_table(pt):
        return no(f"놓을 자리 ({tx:+.3f}, {ty:+.3f}): {P.off_table_why()}")
    yplace = tyaw + R.SIM_FROM_REAL_YAW
    w90 = lambda v: ((v + 45.0) % 90.0) - 45.0

    def err(q):
        c, rc = cube_at(q)
        want = np.array([pt[0], pt[1], s.table_z + low_ext(rc) + PLACE_FROM_GAP])
        return want - c, w90(yplace - flat(rc)[1])

    def refine(q, p, yaw):
        """놓는 자세: 집는 점 p·손 yaw 를 큐브 오차만큼 고쳐 가며 다시 푼다 (손이 기운 만큼 큐브 자리가 달라져서)."""
        for _ in range(8):
            ep, ey = err(q)
            if float(np.linalg.norm(ep)) < 5e-4 and abs(ey) < 0.3:
                return q
            p, yaw = p + ep, yaw + ey
            cs = P.candidates(p, yaw)
            if not cs:
                return None
            q = min(cs, key=lambda c: float(np.abs(c - q).max()))
        ep, ey = err(q)
        return q if float(np.linalg.norm(ep)) <= TIP_TOL and abs(ey) <= YAW_TOL else None
    p_first = P.grip_target(pt)
    Pl = []
    for q in P.family(P.candidates(p_first, yplace)):
        q = refine(q, p_first.copy(), yplace)
        if q is not None and not any(float(np.abs(q - o).max()) < 1e-3 for o in Pl):
            Pl.append(q)
    if not Pl:
        return no(f"놓을 자리 ({tx:+.3f}, {ty:+.3f}): 손에 든 큐브를 그 자리·각도로 놓는 자세가 없음 (팔이 안 닿거나 손 방향을 못 맞춤)")
    Pl.sort(key=lambda q: float(np.abs(q - q1).max()))
    hx = P.exit[-1]
    sp = float(max_speed)
    O, OX = P.open, P.open_exit
    home_exit = len(P.exit) > 1

    def cube_low(qs):
        return min(c[2] - low_ext(rc) for c, rc in (cube_at(q) for q in qs)) - s.table_z

    def build(qb, qp, d2):
        t2 = P.transit(q1, qb, f_home=True) or P.via_home(q1, qb)   # f_home: 시작(ㄴ자, 손이 앞을 봄) 손목 값에서 옆으로 가는 동안 손 아래로
        t3 = P.transit(qb, hx if home_exit else P.home0, t_home=P.exit_fh if home_exit else True)
        fb = ([] if t2 else ["들기 끝 자세→놓을 자리 위"]) + ([] if t3 else ["놓을 자리 위→HOME"])
        fr, mk = [(q0.copy(), F.copy())], {"start": 0}

        def add(qs, fa, fz, key, min_s=0.0):
            fr.extend(P.frames(qs, fa, fz, sp, min_s))
            mk[key] = len(fr) - 1
        for qs in pre:
            add(qs, F, F, "into_limits")
        add(t2 or [q1, qb], F, F, "above_place")
        add(d2, F, F, "place")
        add([qp, qp], F, O, "open", GRIP_S)
        add([qp, qp], O, O, "opened", GRIP_HOLD_S)
        add(d2[::-1], O, O, "lift2")
        if home_exit:
            add(t3 or [qb, hx], O, OX, "exit_back")
            fr.extend(P.exit_frames(OX, sp, back=True))
            mk["home0"] = len(fr) - 1
        else:
            add(t3 or [qb, P.home0], O, OX, "home0")
        _, yaw_flat = flat(cube_at(qp)[1])
        hit = P.cube_hits(fr, range(mk["opened"], len(fr)), pt, yaw_flat)
        carry = cube_low([q for q, _ in fr[:mk["above_place"] + 1]])
        down = cube_low([q for q, _ in fr[mk["above_place"]:mk["place"] + 1]])
        bad = bool(hit or fb or P.below_table(fr) or carry < TABLE_MARGIN or down < 0.0)
        return {"bad": bad, "frames": fr, "marks": mk, "hit": hit, "fb": fb, "carry": carry, "down": down, "qp": qp, "qb": qb, "yaw_flat": yaw_flat}
    best = first = None
    tries = 0
    for qp in Pl:
        for bp in P.approaches(P.grip(qp), [qp], P.hand_axis(qp)):
            tries += 1
            qb, _, d2, _ = bp["pairs"][0]   # (위 자세, 잡는 자세, 내려가는 길, 드는 길)
            r = build(qb, qp, d2)
            r["bp"] = bp
            first = first or r
            if not r["bad"]:
                best = r
                break
            if tries >= MAX_TRIES:
                break
        if best or tries >= MAX_TRIES:
            break
    r = best or first
    if r is None:
        return no(f"놓을 자리 위 {min(ABOVE_CM):g}~{max(ABOVE_CM):g} cm 에서 손을 아래로 둔 채 곧게 내려가는 길이 없음")
    frames, mk, qp, bp = r["frames"], r["marks"], r["qp"], r["bp"]
    # 4) 검사 (plan() 과 같은 것 + 손에 든 큐브 높이)
    low = [P.finger_low(q, f) for q, f in frames]
    arm_lo = [P.arm_low(q) for q, _ in frames]
    below = [i for i, z in enumerate(low) if z < s.table_z + TABLE_MARGIN or arm_lo[i] < s.table_z]
    col = [P.column_low(q, f) for q, f in frames]
    near_col = [i for i, c in enumerate(col) if c < COLUMN_MARGIN]
    i0 = mk.get("into_limits", 0)   # 한계 안으로 들어온 뒤부터 한계 검사
    over_j = [n for k, n in enumerate(s.arm) if any(not (P.lim[k, 0] - 1e-4 <= q[k] <= P.lim[k, 1] + 1e-4) for q, _ in frames[i0:])]
    ep, ey = err(qp)
    tilt, yaw_flat = flat(cube_at(qp)[1])
    problems = []
    if r["hit"]:
        problems.append(f"손가락이 놓은 큐브에 닿음 (빠질 때): 프레임 {r['hit'][0]}~{r['hit'][-1]} ({len(r['hit'])}개)")
    if below:
        problems.append(f"손가락·팔이 책상 위 여유 {TABLE_MARGIN * 1000:.0f} mm 보다 낮게 감: 프레임 {below[0]}~{below[-1]} ({len(below)}개)")
    if over_j:
        problems.append("관절 한계 밖: " + ", ".join(over_j))
    if near_col:
        problems.append(f"팔·손이 기둥에 {COLUMN_MARGIN * 1000:.0f} mm 보다 가까이 감: 프레임 {near_col[0]}~{near_col[-1]} ({len(near_col)}개), 가장 가까이 {min(col) * 1000:+.1f} mm")
    if r["carry"] < TABLE_MARGIN or r["down"] < 0.0:
        problems.append(f"들고 가는 큐브의 가장 낮은 모서리가 책상 위 {min(r['carry'], r['down']) * 1000:+.1f} mm")
    if r["fb"]:
        problems.append("손목 높이 유지 길을 못 찾아 관절 공간 직선으로 감: " + ", ".join(r["fb"]))
    if float(np.linalg.norm(ep)) > TIP_TOL or abs(ey) > YAW_TOL:
        problems.append(f"놓는 자세의 큐브가 목표에서 {np.linalg.norm(ep) * 1000:.1f} mm, {ey:+.1f}°")
    # 5) 큐브 자세 (실좌표): 손 펴기 전까지 손에 붙음 → 펴는 동안 책상에 눕힘 → 그 자리
    mj = s.mj

    def pose(c, rc):
        qq = np.zeros(4)
        mj.mju_mat2Quat(qq, np.ascontiguousarray(Rz.T @ rc).reshape(-1))
        return np.array(list(s.to_real(c)) + [qq[1], qq[2], qq[3], qq[0]], dtype=float)
    held = pose(*cube_at(qp))
    # 눕힌 자세와 같은 큐브(24 가지 회전) 중 놓는 순간 자세에 가장 가까운 것으로 — 화면에서 큐브가 휙 돌지 않게
    c_, rc_ = cube_at(qp)
    k = int(np.abs(rc_[2]).argmax())
    up = np.zeros(3); up[2] = 1.0 if rc_[2, k] > 0 else -1.0
    ax = np.cross(rc_[:, k], up)
    sn, cs_ = float(np.linalg.norm(ax)), float(rc_[:, k] @ up)
    if sn > 1e-9:
        ax /= sn
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        rot_flat = (np.eye(3) + sn * K + (1 - cs_) * K @ K) @ rc_
    else:
        rot_flat = rc_
    rest = pose(pt, rot_flat)
    if float(rest[3:] @ held[3:]) < 0:
        rest[3:] *= -1
    cube = []
    for i, (q, _) in enumerate(frames):
        if i <= mk["place"]:
            cube.append(pose(*cube_at(q)))
        elif i < mk["open"]:
            u = (i - mk["place"]) / max(1, mk["open"] - mk["place"])
            u = min(1.0, 2.5 * u)   # 손가락이 벌어지기 시작하면 곧 눕는다
            v = held + (rest - held) * u
            v[3:] /= float(np.linalg.norm(v[3:]))
            cube.append(v)
        else:
            cube.append(rest)
    traj_q = [list(map(float, q)) + list(map(float, f)) for q, f in frames]
    spd = np.degrees(np.abs(np.diff(np.array(traj_q), axis=0)).max(axis=0)) / DT
    status = "playable" if not problems else ("finger below table" if below else "finger hits cube" if r["hit"] else "check failed")
    labels = {"start": "들기 끝 자세 (큐브 쥠)", "into_limits": "관절 한계 안으로", "above_place": f"놓을 자리 위 (+{bp['cm']:g} cm, 수직에서 {bp['tilt_deg']}°)",
              "place": "내려감: 놓는 자세", "open": "손가락 열기", "opened": "기다림", "lift2": "들기", "exit_back": "HOME 들어가기 전 경유점 (책상 위)", "home0": "HOME"}
    return {**out, "status": status, "playable": status == "playable",
            "reason": "해석 IK 계획 (손에 든 큐브는 붙었다고 봄, 물리·쥐기 시뮬 없음)" if not problems else "; ".join(problems),
            "joint_names": names, "dt": DT, "unit": "rad", "positions": traj_q, "problems": problems,
            "cube": [[round(float(x), 5) for x in v] for v in cube], "marks": mk,
            "waypoints": [{"label": labels[k_], "frame": i, "t_s": round(i * DT, 2)} for k_, i in mk.items()],
            "peak_speed_deg_s": {"joint": names[int(spd.argmax())], "value": float(spd.max())},
            "seconds": round((len(frames) - 1) * DT, 1),
            "checks": {"cube_in_hand_mm": [round(float(v) * 1000, 1) for v in rel_p], "release_tilt_deg": round(tilt, 1),
                       "release_cube_error_mm": round(float(np.linalg.norm(ep)) * 1000, 2), "release_yaw_error_deg": round(ey, 2),
                       "carry_cube_bottom_min_above_table_mm": round(r["carry"] * 1000, 1),
                       "lowest_finger_above_table_mm": round((min(low) - s.table_z) * 1000, 1), "column_min_mm": round(min(col) * 1000, 1),
                       "finger_cube_hits": len(r["hit"]), "joint_space_fallback": r["fb"],
                       "approach": {"cm": bp["cm"], "tilt_deg": bp["tilt_deg"], "azimuth_deg": bp["azimuth_deg"], "tried": tries},
                       "n_candidates": len(Pl), "start_into_limits": bool(pre and over)},
            "plan_s": round(time.time() - t0, 2)}


def main(argv=None):
    try:
        sys.stdout.reconfigure(encoding="utf-8")   # 로그(procman)는 utf-8 로 읽는다
    except Exception:
        pass
    from arm_control.play import add_run_args, checked_run
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cube", type=float, nargs=3, required=True, metavar=("X", "Y", "YAW"))
    ap.add_argument("--target", type=float, nargs=2, required=True, metavar=("X", "Y"))
    ap.add_argument("--target-yaw", type=float, default=None, help="놓을 때 큐브 yaw (°, 실좌표). 비우면 집을 때와 같게")
    ap.add_argument("--d", type=float, default=None,
                    help=f"책상 높이 = 어깨 축에서 책상 면까지 아래로 (m). 비우면 IK 기본 0.4775, {D_RANGE[0]:g} ~ {D_RANGE[1]:g} 만")
    ap.add_argument("--grasp", choices=GRASP_MODES, default="auto",
                    help="집는 방향: auto = 수직 → 수평 → 기울임 순서로 되는 것, vertical = 손 축 수직(위에서), horizontal = 손 축 수평(옆에서 밀어 넣기)")
    ap.add_argument("--place-level", type=int, default=0,
                    help="탑 쌓기: 큐브 바닥을 책상 위 K·5 cm 에 놓는다 (0 = 책상, 기본)")
    ap.add_argument("--obstacle", type=float, nargs=4, action="append", default=None, metavar=("X", "Y", "YAW", "LEVEL"),
                    help="다른 큐브 (실좌표 m·°, LEVEL 0 = 책상 위, 1 = 그 위 …). 여러 번 줄 수 있다")
    ap.add_argument("--pick-drop", type=float, default=None,
                    help=f"집기·들기만 이만큼(m) 낮게 (기본 PICK_DROP_M {PICK_DROP_M:g}, 0 = 낮추지 않음)")
    ap.add_argument("--ik", default=None)
    ap.add_argument("--save", default=None)
    add_run_args(ap)
    a, _ = ap.parse_known_args(argv)
    print("planning (analytic IK, same as the graspable map; no physics) ...", flush=True)
    r = plan(a)
    if r["status"] == "unreachable":
        print(f"refused: {r['reason']}")
        raise SystemExit(3)
    c = r["checks"]
    print(f"grasp: {r['grasp']['text']}")
    print(f"plan: {r['status']} — grip {c['grasp_grip_error_mm']} mm from the cube centre at the grasp, place "
          f"{c['place_grip_error_mm']} mm, hand turned {r['sim']['hand_yaw_deg']:+.1f} deg (sim), tilt {r['sim']['hand_tilt_deg']:.1f} deg, "
          f"lowest finger {c['lowest_finger_above_table_mm']:+.1f} mm above the table, {r['plan_s']} s")
    for w in r["waypoints"]:
        print(f"  {w['t_s']:6.2f}s  {w['label']}: grip {w['grip_real']}, hand yaw {w['hand_yaw_real_deg']:+.1f}, fingers {w['fingers']}")
    for p in r["problems"]:
        print(f"check: {p}")
    if a.save:
        with open(a.save, "w", encoding="utf-8") as f:
            json.dump(r, f, indent=1, ensure_ascii=False)
        print(f"wrote {a.save}")
    raise SystemExit(checked_run(r, a))


if __name__ == "__main__":
    main()
