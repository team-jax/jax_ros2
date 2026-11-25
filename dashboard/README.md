# dashboard — 실물 팔 제어(ROS 없이) + 각도 제어 + 팔 3D (웹 화면)

브라우저 한 화면에서 `can0` 올리기, **팔 연결(이 서버가 can0 을 직접: `real_arm.py`)**, AK45 상태, 원점(homing), 보정,
**관절 각도 보내기(모터가 실제로 움직임)**, `pick_place --dry-run` 궤적 3D 재생과 실물 재생을 한다. **ROS 설치가 필요 없다.**
모드 2개: **실물(직접 CAN, 기본)** / **가상**(Windows 에서도 끝까지 돈다). `--ros` 를 주면 '실물'이 팀 ROS 노드 경로가 된다(아래 'ROS 모드').

## 실행

```bash
sudo apt install -y can-utils python3-can python3-fastapi python3-uvicorn python3-websockets   # 또는 pip install python-can fastapi uvicorn websockets
python3 dashboard/server.py              # 실물(직접 CAN) 모드: ① CAN 켜기 → ② 팔 연결
python3 dashboard/server.py --virtual    # 가상 팔 모드 (실물·CAN·ROS 없이, Windows 도)
python3 dashboard/server.py --fake-hw    # 직접 CAN 을 가짜 AK45 4개 + 가짜 STM32 (python-can virtual) 에 — 화면 시험
python3 dashboard/server.py --ros        # 팀 ROS 버전 (ROS 2 + colcon build 필요)
```

브라우저에서 **http://localhost:8080**. 기본은 **이 PC에서만**(127.0.0.1). 같은 네트워크에서 보려면 `--host 0.0.0.0`
(암호가 없어 같은 네트워크 누구나 노드를 켜고 끄고 **모터 명령을 보낼 수 있다**. 필요할 때만).

| 변수 | 기본 | 뜻 |
|---|---|---|
| `JAX_WS` | 레포가 `…/src/` 안이면 그 워크스페이스, 아니면 `~/ros2_ws` | `install/setup.bash` 를 찾는 곳 |
| `JAX_ROS_DISTRO` | `$ROS_DISTRO`, 없으면 `jazzy` | `/opt/ros/<distro>/setup.bash` |
| `JAX_DASH_PORT` / `--port`, `JAX_DASH_HOST` / `--host` | 8080, 127.0.0.1 | |
| `JAX_DASH_VIRTUAL=1` / `--virtual` | | 가상 모드로 시작 (화면 위 스위치로도 바꿈) |
| `JAX_DASH_ROS=1` / `--ros` | | '실물' = ROS 노드 (없으면 직접 CAN) |
| `JAX_CAN_INTERFACE` / `--can-interface`, `JAX_CAN_CHANNEL` / `--can-channel` | socketcan, can0 | 직접 CAN 의 python-can 버스 |
| `JAX_IK_DIR` | 없으면 레포 옆 `../IK`, 그다음 `<ws>/src/IK` | team-jax/IK 체크아웃. 3D 메시(`motion2/arm/meshes`)와 pick_place 계획에 쓴다 |
| `JAX_HOME_FROM_IK=1` | 없음 (HOME = 모든 관절 0) | `pick_plan.Planner` 의 HOME 을 IK 레포 `env.HOME_ARM_QPOS`(sim 의 손바닥-아래 자세)로 (예전 동작). 기본은 0 — IK 값을 쓰면 `sim-table-friction` 브랜치에서 HOME 이 위팔 143°·손목 −90° 가 되어 'HOME 으로 못 감'·지도 초록 칸 0 이 된다 (2026-11-24) |
| `JAX_DASH_PYTHONPATH` | | 로컬 pick_place(가상 모드·Windows)에서 sys.path 뒤에 붙일 경로(mujoco·gymnasium·scipy 가 따로 있을 때) |
| `JAX_DASH_SIGINT_WAIT`, `JAX_DASH_SIGTERM_WAIT` | 5, 3 초 | 끄기 단계 사이 대기 |
| `JAX_DASH_DEMO=1` | | Linux 에서도 가짜 시험 항목을 보인다 |
| `JAX_DASH_LOGS` | `dashboard/logs` | 로그·계획 궤적(`pick_dry_traj.json`) 폴더. 안 주면 먼저 뜬 서버가 `logs/`(잠금 `logs/.server.lock`), 같은 PC 의 다른 서버는 `logs/other_<포트>` — 시험 서버의 계획이 실물 서버 계획을 덮어쓰지 않게 (2026-11-23) |
| `JAX_DASH_MAP_PARTS` | CPU 수 ÷ 2 (최대 8) | '집을 수 있는 곳' 지도를 나눠 푸는 프로세스 수 (PC 가 바쁘면 2~3) |

## 설치 치수 — 어깨 축 기준 (2026-11-25)

로봇·책상·폰·큐브를 이 치수로 놓아야 계획·인식이 맞는다. **좌표는 어깨 축(shoulder_roll 축) 기준**이다:
원점 = 어깨 축, **+x = 로봇 왼쪽**, **−y = 앞 (책상 쪽)**, +z = 위. 각도 제어·역기구학·휴대폰 탭의 x, y 가 모두 이 틀이다.

| 항목 | 값 | 어디서 쓰나 · 비고 |
|---|---|---|
| **책상 높이 d** (어깨 축 → 책상 윗면, 아래로) | **기본 0.5575 m** (역기구학 탭 칸을 비우면 이 값 — 2026-11-25 프로파일을 8 cm 더 아래로 달음. 실측 0.555~0.560, 폰 중력 센서 + 큐브 마커). IK 모델 장면은 0.4775 | 역기구학 탭 '책상 높이' 칸. 받는 범위 **0.44 ~ 0.60 m**. 휴대폰 탭 **'자동 보정'** 이 재서 넣는다. 틀리면 손이 그만큼 위·아래에서 멈추고 ArUco·뎁스가 안 맞는다 (0.4775 로 두면 실물에서 8 cm 위를 집었다) |
| **책상 앞 끝** (어깨 축에서 앞으로, 수평) | **0.14 m** (y = −0.14) | 역기구학 계획의 가정. 큐브는 앞 끝에서 **3.5 cm 이상 안쪽** = **y ≤ −0.175** |
| **큐브** | 한 변 **5 cm**, 윗면에 ArUco **4x4 ID 0~4, 40 mm** | 마커는 `phone_markers/phone_markers.pdf` 2쪽을 100% 로 인쇄. 큐브는 책상 위에 한 층만 (탑 쌓기 없앰) |
| **큐브를 놓을 곳** (계획이 집기·놓기 되는 칸, 1 cm 격자, 모든 yaw) | **d 0.555**: y −0.18 ~ −0.34. 줄마다 x: y −0.18~−0.20 → x −0.22 ~ +0.12 · y −0.24 → −0.20 ~ +0.10 · y −0.28 → −0.18 ~ +0.08 · y −0.32 → −0.14 ~ +0.04 · y −0.34 → −0.10 ~ 0.00 | 가운데는 **x ≈ −0.05** (팔이 어깨 축보다 오른쪽으로 약 5 cm). 앞으로 멀어질수록 좁다. **가장 쉬운 자리: x −0.05, y −0.22 ~ −0.26**. 정확한 칸은 역기구학 탭 지도(초록) |
| (참고) d 0.4775 일 때 | y −0.18 ~ −0.37, y −0.26 줄에서 x −0.22 ~ +0.12 | 책상이 높을수록(d 작을수록) 가까운 줄이 좁아진다 |
| **폰 (카메라)** 자리 | 왼쪽 렌즈가 **x +0.174, y −0.118, z −0.012 m** (어깨 축에서 왼쪽 17.4 cm, 앞 11.8 cm, 아래 1.2 cm) | CAD 값 (`camera_to_robot` 앞 셋). 자리·yaw 는 중력으로 못 재서 CAD 그대로 — 대신 **자리 고침** (휴대폰 탭, `pos_fix_m`) 기본 **x −3 cm, y +1 cm**: 실물로 집어 보며 맞춘 값 (2026-11-25, 손이 큐브보다 3 cm 앞 → 왼쪽 2 cm·앞 1 cm 더 → 23:50 로봇을 옮긴 뒤 손이 덜 가서 y +4 → +2 → 23:58 집은 뒤 큐브가 오른쪽·앞으로 밀려 x −1·y −1 더). 더 정확히는 휴대폰 탭 ③ 카메라 보정 |
| 손가락 마디 2 서보 (`finger1/2/3_joint2`) | **서보 90° = 곧게 편 자세 (원점)**, 서보 각이 작아질수록 안으로 굽힘 (`sign −1`) | 2026-11-25: 예전 '0° = 곧게, 커지면 굽힘' 은 실물에서 바깥으로 꺾였다 — 혼을 90° 에서 곧게 맞춰 끼움 |
| 폰 기울기 | **아래로 약 46°** (CAD 45°, 실측 46.3°), **거꾸로 끼움** (광축 둘레 180°) | 실시간 인식은 매 프레임 **폰 중력 센서로 기울기를 자동 보정** (저장 안 함, 8° 넘게 다르면 안 고침) |
| 폰 → 큐브 거리 | 0.4 ~ 0.8 m 쯤 | 지금 자리에서 큐브까지 약 0.58 m (마커 PnP·스테레오가 0.8 cm 안으로 같음) |
| **강화학습 정책 장면** | 책상 앞 끝 **0.225 m**, 윗면 어깨 아래 **0.4775 m** (고정, 학습값) | 실물 책상이 0.555 면 정책은 **8 cm 위를 집는다** → 강화학습을 실물로 쓸 땐 큐브를 **약 8 cm 받침대** 위에 (정책은 d 를 못 바꾼다). 위치는 강화학습 탭 지도(파란 칸) |

**맞추는 순서**: ① 폰을 거치대에 끼우고 폰 프로그램 켜기 → ② 큐브 하나를 '가장 쉬운 자리' 에 평평하게 → ③ 휴대폰 탭 **'자동 보정'**
(결과 카드가 막혔을 때 그 안의 버튼, 또는 뎁스 카드) → 확인하면 거치 기울기 저장 + 역기구학 탭 d → ④ 결과 줄이 **'ArUco + 뎁스 … 맞음'**,
뎁스 '책상 위 0 ~ 5 cm' 인지 확인 (2026-11-25 실측: d 0.555 에서 ArUco·뎁스 7 mm 차) → ⑤ 가상 팔로 먼저, 그다음 실물.
자동 보정은 2~3 번 해서 d 가 1 cm 안으로 같은지 본다 (큐브가 기울면 흔들린다).

## 화면: 탭 (연결 · 각도 제어 · 역기구학 · 강화학습 · 휴대폰 카메라)

위 띠(모든 탭 공통): 연결·팔 연결·can0·실행 수 칩, **모드 스위치 실물 / 가상**, 큰 **■ STOP**(Esc, 직접 CAN = 지금 자리 홀드; 가상·ROS = 전부 정지), 경고 띠. 고른 탭은 주소 해시(`#conn` `#angle` `#ik`, 예전 `#ak` 등도 옮겨 감)와 localStorage 에 기억.

| 탭 | 내용 |
|---|---|
| **연결** | 직접 CAN: ① CAN 켜기 ② 팔 연결(속도 °/s, 힘 빼기, 연결 끊기), AK45 4개 표, 고급: 버스 상태(에러 프레임·송신 실패)·안전 기본값·로그·**관절 하나씩 보정**. ROS 모드면 아래 옛 화면: ROS 연결 카드(상태 + 'ROS 연결(source)') 또는 가상 팔 카드(가상 전원 끄기/켜기), 노드 켜기/끄기 전부(can0 올리기, arm.launch / arm_homed.launch / ak45 / stm_bridge / arm_control / vision, vcan 시험), AK45 진단 카드 4개 + CAN 버스, 로그, 보정 안내 |
| **각도 제어** | 원점(homing) 패널, 관절 14개 슬라이더 + 숫자칸(**시뮬 관절각 °**) + 관절마다 '보내기' / '바뀐 것 전부 보내기' / '현재 자세로 슬라이더 맞추기' / '바꾼 것 되돌리기'. 손가락은 묶어서(손가락 1·2·3 같이 움직이기 체크). 오른쪽 3D: **불투명 = 실제 현재 자세**, **파란 반투명 = 슬라이더 목표** |
| **역기구학** | **2D 지도**(위에서 본 책상, 로봇 위·앞 아래·로봇 왼쪽 = 화면 오른쪽): [큐브 놓기]·[놓을 자리 고르기] 로 눌러 고르고 yaw 슬라이더, 칸 색 = 그 yaw 의 도달 맵(team-jax/IK `reachmap_real_yaw.npz`), 고를 때마다 `realcoords.plan` 판정 한 줄(안 돼도 계획은 돌릴 수 있음). 숫자칸은 '숫자로 입력' 안(양방향). `pick_place --dry-run` 폼(+ `/object_pose` 값을 cube 칸에 넣기, vision_node 상태 작게, 가상 모드면 가상 큐브 칸). 오른쪽 3D: 계획한 궤적 재생(재생/일시정지/프레임 슬라이더/속도) + 큐브(주황)·놓을 자리(초록)·/object_pose(보라). 가상 모드면 **가상 팔로 재생** |

### 역기구학 — 갈 수 있는 곳 (2026-11-21)

지도 위 **'갈 수 있는 곳 보기 (손 위치 높이에서 IK 풀림)'** 를 켜면 집기 조건 없이, 아래 ③ 손 위치로 보내기의 **높이·기준점(손끝 사이 / 손목 중앙)·어깨 옆 고정값**으로 격자(2 cm) x, y 마다 손 패널과 **같은 풀이**(`_Solver.solve`, 위치만 모드)가 풀리면 초록. 높이·기준점을 바꾸면 다시 계산(`GET /api/ik/reachxyz?z_cm=&ref=&shoulder_yaw=&step_cm=`, `hand_ik._Solver.reachxyz`, 지도 도우미 worker 로 나눠 높이 하나 약 7 s, 높이마다 서버 캐시). 켜 있는 동안 '집을 수 있는 곳'(파랑)은 숨김. 책상·기둥·경로 충돌은 안 본다(그 자리에 IK 해가 있나만).
예: 손끝 책상 위 3.5 cm 416 칸, 15 cm 500 칸, 25 cm 459 칸, 40 cm 385 칸 (팔꿈치 ±135, 어깨 옆 0°).

### 역기구학 — 집는 방향: 자동 / 수직 / 수평 (2026-11-24)

IK 탭 ① 카드의 "집는 방향" 칸 하나. 지도(집을 수 있는 곳)·IK 확인(`/api/ik/pair`)·계획(`pick_plan.py --grasp`)이 모두 이 값을 쓴다.
API: `GET /api/ik/graspmap?grasp=auto|vertical|horizontal&yaw=°`, `POST /api/ik/graspable {grasp, yaw}`, `POST /api/ik/pair {grasp}`, pick_dry 파라미터 `grasp`.

- **수직** (`vertical`): 잡는·놓는 자세와 곧게 내려가고 드는 길 내내 손 축이 수직에서 5° 안 (`VERT_MAX_TILT`), 내려가는 방향도 수직만. 안 되면 이유와 함께 거절.
- **수평** (`horizontal`): 손을 눕혀 큐브 옆 4~6 cm(`H_PUSH_CM`) 의 위에서 손 수평 그대로 곧게 내려와 → 손 축 따라 밀어 넣고 → 세로 두 면을 쥐고 → 곧게 위로. 놓기는 거울(곧게 내려놓고 → 옆으로 빠져 → 위로).
  밀어 넣는 방향은 큐브 면 네 방향(yaw + 90°·k) 중 팔이 닿는 것, 손가락 닫히는 축은 hand_roll 로 수평에 맞춘다 (손 축·닫히는 축 5° 안, `H_TOL`).
  - 집는 점은 큐브 중심이 아니라 **책상 위 4.3 cm**(`H_GRIP_ABOVE` 1.8 cm): 누운 손은 손가락 상자가 집는 점 아래로 36~39 mm 내려와(손 세로 폭 75 mm > 큐브 50 mm) 중심 높이로는 손가락이 책상 속이다.
    책상 충돌 검사를 꺼도(`hand_ik.NO_TABLE`) 수평은 손가락이 책상 위 3 mm(`H_TABLE_MARGIN`)를 지키는지 따로 본다.
  - 되는 자리가 좁다: 손목이 집는 점과 같은 높이(어깨 아래 43 cm)라 팔 길이(46 cm)로는 손목이 어깨 바로 아래 반지름 약 15 cm 안일 때만 닿는다 → **큐브 면이 어깨 쪽을 보는 자리만**.
    그래서 수평 지도는 지금 큐브 yaw 기준(화면이 `yaw` 를 같이 보냄, yaw 를 바꾸면 다시 계산 약 10~50 초). yaw 0° 지도 88 칸 (x −0.08~−0.02, y −0.35~−0.18), plan_check 10/10.
  - 손목 길(transit)은 끝에서 손목 두 관절을 누운 값으로 섞는다. 그동안 손이 큐브 가까이 내려오므로 떠나기 전에 손가락을 다 벌린다.
- **자동** (`auto`, 기본): 수직 → 수평 → 기울임(예전 방식: 손목이 되는 만큼 아래 + 비스듬한 내려가기) 순서로, 검사를 다 통과하는 첫 계획. 결과 `grasp.text` = "집는 방향: 수직 (기울기 0°)" (화면 결과 줄에도).
  지도는 예전과 같다(모든 큐브 yaw 로 기울임이 되는 칸 — 수직 칸을 포함, 수평은 yaw 에 달려 지도에 안 넣음): 439 칸, 수직만은 349 칸.
- 여러 자세가 되면 **손이 수직에 가장 가까운 것**: 잡는 자세를 손 기울기 1° 묶음으로 나눠 작은 묶음부터 길을 찾고, 그 안에서만 관절 이동 합으로 고른다 (`_plan_style` 의 `tilt_groups`).
- `plan_check.py --make-map f.json --grasp vertical|horizontal [--yaw 0]` 로 방향별 지도, 그 지도로 계획 검사. 2026-11-24: 자동 439 칸 10/10, 수직 349 칸 10/10, 수평(yaw 0) 88 칸 10/10. `try_direct_fake.py` 39/39.
- 안 한 것: 수평으로 쥐었을 때 큐브가 실제로 잡히는지(물리 없음 — 아래 손가락 하나와 반대쪽 손가락이 높이가 달라 기울 수 있다), 실물. IK 탭 ③ 손 위치 카드의 후보 고르기(최소 회전 → 2° 안이면 수직)는 그대로.

### 각도 제어 — HOME 자세로 가기 (2026-11-21)

'움직이기' 맨 위 **⌂ HOME(원점 자세)으로 가기**: 지금 자세 → HOME(모든 관절 0) 궤적을 `pick_plan.plan_home` 이 계획하고 지금 모드의 팔로 재생한다(`POST /api/arm/go_home`, 가상 = 가상 팔 재생, 실물 직접 CAN = `RealArm.play_trajectory` + `analytic_fatal`·`check_trajectory`, 경고는 확인창 뒤 force. ROS 모드는 아직 없음). 계획 궤적 파일(`logs/pick_dry_traj.json`)은 건드리지 않는다.
길은 차례대로 검사(손가락·팔 책상 여유 `TABLE_MARGIN`, 관절 한계)를 통과한 첫 것: ① 지금 자세가 HOME 나가기 경유점 꺾은선(나가는·들어오는 중) 5° 안이면 거기서 꺾은선을 거꾸로 ② 손목 길(`transit`)로 책상 위 손목 자리(`exit[-1]`) → 나가기 경유점 거꾸로 (계획이 HOME 으로 돌아오는 길과 같음) ③ 관절 직선(팔이 책상 옆에 늘어져 있을 때). shoulder_yaw 가 0 이 아니면 먼저 그것만 0. 손가락은 첫 구간 동안 벌림. 셋 다 걸리면 거절.
서보는 피드백이 없어 한 번도 안 보낸 서보는 0° 로 보고 계획한다(실물은 real_arm 이 '한 번도 안 보낸 서보' 재생을 따로 거절). 책상 옆 기둥·받침은 검사하지 않는다.
시험: 저장된 계획 궤적 50 프레임마다(60 자세) 모두 HOME 까지 playable, 끝 = 정확히 0. 가상 팔로 늘어진 팔(관절 직선 5 s)·계획 중간(손목 길 23.5 s)에서 HOME 도착 확인.

### 재생 중 '지금 하는 동작' 한 줄 (2026-11-24)

팔 재생(가상·실물, 역기구학 계획·⌂ HOME·탑 쌓기 걸음·강화학습 팔로 재생) 동안 머리띠 안 파란 줄에 지금 동작이 보인다 — 어느 탭에서든. 예: `▶ 큐브 위로 가는 중 · 2/12단계 · 남은 35초 · 프레임 807/2565`. 끝나면 6초 동안 `끝`(초록) 또는 `멈춤 (STOP) · 그때: 큐브 위로 가는 중`(빨강). 역기구학·HOME·실물·강화학습 칸의 재생 줄에도 같은 글이 붙는다.
- 서버 `play_phase.py`: 재생 시작 때 `phases(traj)` 로 단계표 [(시작 프레임, 글)] → `ros.virtual_state.playback` / `ros.direct_state.playback` 에 `phase`·`phase_k`·`phase_n`·`left_s`. 첫 프레임 홀드 동안은 '첫 자세 잡는 중'.
- 단계 출처: `pick_plan.plan()`·`plan_place_from()` = `marks`(경유점 → 닿는 프레임, plan() 출력에도 이제 넣음), 경유점 k 에 닿기까지가 'k 로 가는 중' (HOME 나가는 중 → 큐브 위로 → 내려가는 중 → 손가락 닫는 중 → 들어 올리는 중 → 놓을 자리로 옮기는 중(탑이면 'N층 자리로') → 내려놓는 중 → 손 펴는 중 → 손 빼고 올라가는 중 → HOME 으로 돌아가는 중). `plan_home()` = `marks` (yaw0 / exit_back / home). 강화학습 `rl_api.build_traj` = `phase_marks`(체인 RAISE·SETTLE·INSERT·GRASP_LIFT·RETURN·DONE + 내리기 CARRY·LOWER·OPEN·RETREAT·HOME, BUILD 3 → 옛 rl_traj.json 다시 만듦). 그 밖 옛 궤적 파일은 waypoints label.
- 확인: 가상 8093 IK 계획 재생 12단계·HOME 2단계 글이 차례로 바뀜(CDP), `--fake-hw` HOME + 실물 재생 중 STOP → '멈춤 (STOP) · 그때: 큐브 위로 가는 중', `try_direct_fake.py` 39/39.

### 검토 뒤 고친 것 (2026-11-23 밤)

- **STOP 이 재생 시작과 겹쳐도 안 사라진다**: STOP·힘 빼기·재생 멈춤은 잠금 안에서 `stop_gen` 을 올리고, 재생은 시작할 때의 값을 기억해 바뀌면 멈춘다.
- **재생 중 보정 거절**: `calib()` 은 재생 중 verify·save 말고 "재생 중 — 멈춘 뒤 보정".
- **화면이 보여 준 계획만 재생**: `POST /api/ik/play_real {force, mtime}` — mtime 이 지금 궤적 파일과 다르면 "계획이 바뀜" 거절(없으면 검사 안 함). 칸 값(큐브·놓을 자리·책상 높이)이 계획에 쓴 값과 다르면 결과 줄이 "값이 바뀜 — '계획 돌려 보기'를 다시", 재생 버튼(실물·가상) 꺼짐.
- **다른 사이트의 POST 거절**: `Origin` 헤더가 이 서버 주소(`Host`)와 다르면 403. Origin 없는 요청(curl·스크립트)은 그대로. 암호는 여전히 없다.
- HOME 카드: 진행 `가는 중 n/N` → `마지막: …`, 끝나면 슬라이더가 지금 자세로 맞춰진다. 폰 탭: 집을 수 있는지 확인에 책상 높이를 같이 보내고 '놓을 자리' 줄에 책상 높이 표시.

### 각도 제어 — 안전장치 (모터가 실제로 움직인다)

- 슬라이더·숫자칸은 3D 의 파란 목표만 바꾼다. **'보내기' 버튼을 눌러야** 나간다(관절 하나 / 바뀐 것 전부).
- 보내는 곳: `/arm_control/joint_command` (sensor_msgs/JointState, 이름 = sim 관절명, position = rad). ROS 자식 프로세스의 stdin 명령 → 자식이 발행.
  arm_control_node 가 calibration.json 변환·클립과 램프(`max_speed_deg_s`)를 하고, arm_homed 면 homing_guard 가 원점 없는 AK45 명령을 버린다(둘 다 기존 코드 그대로).
- 범위: 관절마다 `sim_range_deg` ∩ 모터 범위(서보 0~180°, AK45 `zero_deg ± limit_deg`)를 sim 각으로 바꾼 것(`cmd_range_deg`). 지금 calibration 이면 hand_roll −90~90, finger*_joint2 0~90, finger_rotation 0 만.
- 서버(`POST /api/joint/send {"joints": {관절: 도}}`)가 다시 검사: 아는 관절명, 유한한 숫자(bool·문자열 거절), 범위 안(잘라 보내지 않고 거절), 아래 게이트.
- 게이트(`state.gate`, 화면과 서버가 같은 판정): ROS(또는 가상) 연결 + `/arm_control/joint_command` 구독자 ≥ 1(arm_control_node / arm.launch / arm_homed 가 켜짐) 이 아니면 전부 꺼짐 + 이유.
  arm_homed 가 켜져 있거나 homing_guard 상태가 3초 안에 왔으면 '원점 모드': guard 상태가 없으면 AK45 전부, 원점 없는 AK45 는 그 관절만 꺼짐.
  알림(막지는 않음): AK45 피드백 없음(arm_control_node 가 기다림), 서보 첫 명령(위치를 몰라 램프 없이 바로 감 — arm_control_node 규칙).
- 첫 보내기 전에 확인창 1회(페이지마다): `verified=false` 관절 목록 — 방향·영점이 가정, 팔 주변 비우기, max_speed, 서보 첫 명령은 바로 감, 멈추는 법.
- '현재 자세로 슬라이더 맞추기': AK45 = 피드백(arm_homed 면 guard 가 오프셋을 뺀 `/ak45/joint_states_homed`) → sim 환산, 서보 = `/servo/state`(마지막으로 보낸 값). 서보는 피드백이 없다.
- 직접 CAN 모드: 보내는 곳은 `real_arm.RealArm.joint_command`(같은 calibration 변환·클립 → 원점 오프셋 → 램프). 멈춤 = ■ STOP(지금 자리 홀드, 팔 안 떨어짐).
- 가상·ROS 모드: 멈추는 것은 ■ 전부 정지(노드 종료 → AK45 브레이크 0A → **팔이 떨어질 수 있음**).

### 3D 뷰

- 모델: `static/robot/arm.xml` — team-jax/IK `motion2/arm/arm.xml`(브랜치 sim-table-friction, 커밋 925e023, 출처·해시는 `static/robot/SOURCE.txt`). IK 시뮬(motion1 pick_place 계획 포함)과 같은 생성기의 모델이라 관절명·축·영점이 시뮬 각과 같다.
- **arm.urdf 대신 arm.xml 을 쓰는 이유**: `arm.urdf` 는 ikpy 용 7관절 체인뿐이라 `<visual>`(메시)도 손가락도 없다. arm.xml 은 같은 7관절(관절 원점·축이 arm.urdf 와 같음을 확인: 위치 차 0, 회전 차 < 1e-7°) + 손가락 6관절 + 메시 배치를 갖고 있다. 그래서 urdf-loader 는 쓰지 않고, arm.xml 의 body/joint/geom 을 읽는 작은 로더(`static/viewer.js`)를 썼다. `arm.urdf` 도 참고로 같이 복사해 두었다(쓰지 않음).
- 좌표: 로봇 기준(vision_node·pick_place 와 같음) — 원점 = shoulder_roll 관절 원점, x 로봇 왼쪽+, y 뒤+(앞 = −y), z 위+. arm.xml base_link 틀을 x 로 +0.052 m 옮긴 것과 같다(IK `realcoords.py` + `env.py` robot_yaw −90° 로 계산. pick_place 궤적을 이 틀로 FK 하면 손끝이 큐브 중심 2.1 cm 안을 지나고 놓을 자리 2.5 cm 안에 간다 — grasp_dz 21.5 mm 를 생각하면 맞음). 격자 = 책상 높이(어깨 축 아래 d, 기본 0.4775 m).
- 메시(OBJ 약 8.7 MB)는 레포에 넣지 않는다. 서버가 `JAX_IK_DIR`(없으면 `../IK`, `<ws>/src/IK`)의 `motion2/arm/meshes/*.obj` 를 `/robot/meshes/<파일>` 로 준다(파일 이름 검사). 없으면 **막대 모델**(관절 원점을 잇는 막대 + 주황 관절 구 + 초록 손끝)로 그린다.
- 라이브러리: three.js r160 (`three.module.js`, `OrbitControls.js`, `OBJLoader.js`, LICENSE) 을 `static/vendor/three-0.160.0/` 에 저장(로봇 PC 가 오프라인이어도 됨, importmap). 마우스: 왼쪽 회전, 오른쪽 이동, 휠 확대.
- 현재 자세 = `state.pose`(서버가 계산): AK45 = 피드백(guard 가 있으면 원점 보정 피드백), 서보 = 마지막으로 보낸 값. 값 없는 관절은 0°. finger_rotation 은 sim 에서 고정이라 안 움직인다.
- [계획 돌려 보기] 기본 = **대시보드 계획** `dashboard/pick_plan.py` (2026-11-20): '집을 수 있는 곳' 지도와 **같은 해석 IK**(hand_ik._Solver: 집는 점 = 큐브 중심 + 1 cm (`hand_ik.GRIP_ABOVE_CENTER`, 책상 위 3.5 cm, 놓을 때도 — 큐브 바닥 = 책상), shoulder_yaw 0, 손 최대한 아래)로 경유점 HOME → 큐브 위(집는 점 9/7/5/3 cm, 바로 위가 안 되면 비스듬히 — 손 축에 가까운 방향 먼저) → 곧게 내려가 잡기 → 손가락 닫기(팀과 같은 값, grip 1.0) → 들기 → 놓을 자리 위 → 내려놓기(큐브 바닥 = 책상) → 열기 → 들기 → HOME. 손 yaw(손가락 닫히는 축을 위에서 본 각)를 FK 로 재서 hand_roll(±88°)로 큐브 yaw(mod 90)에 맞춘다. 옮겨 갈 때는 손목 중앙을 일정 높이(손가락 끝이 큐브 윗면 + 3 cm 위)로 1 cm 마다 해석 IK, 내려가기는 집는 점을 1 cm 마다 직선으로 — 둘 다 HOME 과 같은 팔꿈치 가지를 이어 간다(가지가 튀면 그 길 버림). 가지·경유점 고르기 = 관절 최대 이동각 합 최소. 프레임 0.02 s, 가장 빠른 관절 max_speed(30 °/s). 검사: 벌린 손가락↔큐브(MuJoCo 충돌), 손가락↔책상(lowest_finger_z, 닫힘은 큐브가 막는 CUBE_GRIP 0.86 자세로), 관절 한계, 집는 점 오차, 들고 가는 큐브의 가장 낮은 모서리(손 기울기 포함) — 손가락·큐브 바닥은 책상 위 **여유 8 mm**(`TABLE_MARGIN`, 한 곳) 이상이어야 한다. 집는 점은 큐브 중심 그대로 두고 그 여유를 지키는 해만 쓴다(손이 수직이면 벌린 손가락 9.7·쥔 손가락 8.5 mm 위, 손이 12° 쯤 기울면 4 mm 아래쪽 → 그 자리는 집을 수 없음). 걸리면 status 가 playable 이 아니고 출력에 check: 로 나온다. 실물 재생(직접 CAN)은 planner "analytic" 계획이 검사에 걸리거나 관절 직선 대체가 있으면 **force 로도 거절**한다. **책상 높이 MAP_D 0.4775 m(어깨 축 아래)는 가정 — 실물 셋업에서 재야 한다**(틀리면 8 mm 여유를 먹는다). **물리·쥐기 시뮬은 없다**(큐브가 실제로 잡히는지 모름). 지도에서 빨간 점이면 지도와 같은 말로 거절(종료코드 3). '팀 시뮬 계획 (물리)' 를 켜면 예전처럼 `pick_place`(team-jax/IK motion1 MuJoCo, 자기 수치 IK)를 돌린다.
- **위팔 비틀기 ±90° (2026-11-23 사용자)**: calibration upper_arm_roll [−90, 90], 계획 가지 `ELBOW_BRANCH = +1`(위팔 0 근처, elbow > 0) — 예전 −1 가지(위팔 ≈180)는 HOME 에서 매번 곧은 팔로 0→180 을 먼저 돌렸다. 손이 그 가지와 반 바퀴 뒤집혀 있어(손가락 3개, hand_roll ±88) 기운 손으로 곧게 내려갈 때 손가락이 걸리는 큐브 방향이 생긴다 → 계획은 실제 큐브·놓을 방향 하나로만 판정(`point_check(yaws=)`), 지도는 모든 방향 기준 그대로. plan_check(예전 지도 초록 칸) 24/30·23/30 (예전 30/30·29/30, 실패는 모두 '곧게 내려가는 길 없음' = 계획 거절), 지도 초록 276 칸 (예전 387).
- **HOME = 모든 관절 0** (2026-11-21 사용자, homing.json 원점 자세 = 팔이 곧게 아래로, 책상 옆에 늘어짐): team-jax/IK `env.py` SETUPS["real"] home 이 0 → 계획 시작·끝, 손 IK 시작점, 가상 팔 모두 0. 그 자세의 집는 점은 실좌표 (−0.048, 0.002, −0.692 site) = 책상 면 **23 cm 아래지만 책상 옆**이라
  **책상은 무한 평면이 아니라 앞 끝이 있는 반평면**으로 검사한다: **앞 끝 = 실좌표 y −0.14 (어깨 축 앞 14 cm, sim x 0.20 = `TABLE_NEAR_X` = 기둥 앞면 0.30 − STAND_GAP 0.10, env 장면 table_top 의 앞 끝)**, 그보다 앞(−y)은 옆·먼 쪽 끝없이 책상
  (env 장면 책상의 옆·먼 끝 — 실좌표 x −0.228~+0.332, y −0.42 — 은 도달 맵으로 정한 크기라 실물이 아니어서 안 쓴다, 보수적).
  손가락 충돌 상자가 발자국 + 1 cm(`hand_ik.TABLE_XY_MARGIN`) 에 걸치면 그 상자의 가장 낮은 점이 책상 면 + `TABLE_MARGIN` 위여야 하고(`lowest_finger_z`), 팔 기준점(팔꿈치·손목, 반지름 3 cm 가정 `ARM_RADIUS`)도 발자국 위에선 책상 면 위(`arm_low_z`, 계획 검사).
  **책상 앞 끝 자리(어깨 축 앞 14 cm)는 장면 값 = 가정, MAP_D 처럼 실물 셋업에서 재야 한다.** 책상 옆(발자국 밖)의 기둥·받침은 아직 검사하지 않는다.
- **책상 높이** (2026-11-23): ① 카드의 '책상 높이 — 어깨 축 아래 m' 칸 (pick_dry 의 `d`, 비우면 기본 `realcoords.MAP_D` 0.4775). `hand_ik.D_RANGE` **0.44 ~ 0.53 m** 만 받고 밖은 거절한다. IK 모델은 어깨가 고정이라 d 가 다르면 책상 면을 옮긴다 (`_Solver.set_table_d`: `table_z = TABLE_SURFACE_Z + (MAP_D − d)`): 큐브 중심·집는 점·옮겨 가는 높이(`z_clear`)·HOME 나가기 경유점·들고 가는 큐브 검사·손 위치의 '책상 위 cm' 가 모두 따라간다. 쓰는 곳: 계획(`pick_plan.py --d`), 집을 수 있는 곳 지도(`/api/ik/graspmap?d=`, 캐시 키에 d — 높이 3 개까지 기억), 한 점 판정·IK 판정·갈 수 있는 곳·손 위치(`d`), HOME 으로 가기(`/api/arm/go_home {d}`, 계획기 캐시 키에 d). 요청에 d 가 없으면 기본. 지도 초록 칸(1 cm): 0.44 = 20, 0.45 = 45, 0.4775 = 124, 0.50 = 223, 0.53 = 360 — 책상이 높을수록 좁다 (0.43 은 0 칸). 회귀: `tools/plan_check.py --make-map gm.json --d 0.45 [--parts 3]` → `plan_check.py gm.json 10 1` (지도 파일의 d 로 계획). 팀 시뮬 계획(pick_place)·옛 도달 맵은 예전처럼 MAP_D ± 5 mm 만.
  큐브·놓을 자리는 발자국 안쪽 반 변 + 1 cm(`pick_plan.CUBE_EDGE_IN`, env CUBE_X_MAX 와 같은 규칙) 안이어야 한다 = 실좌표 y ≤ −0.175 (지도·계획 모두, 예전 지도는 책상 앞 끝 너머 칸도 초록).
  `_tip_step`(곧게 내려가기 한 걸음)은 8 번 안에 집는 점이 5 mm 안으로 안 오면 버린다 (예전엔 오차를 안 봐서 '큐브 위 9 cm' 가 19 cm 옆인 길도 받았다).
  HOME 에서 나가기 (`pick_plan.Planner._home_exit`): 곧게 늘어진 팔(팔꿈치 0 특이점)은 손목을 곧게 올리면 shoulder_roll 한계로 20 cm 쯤만 오르고(손가락은 아직 책상 면 아래), 계획 가지(upper_arm_roll ≈ 180°, elbow < 0)의 책상 위 손목 자리로 관절 직선으로 가면 손가락이 책상 모서리를 파고든다(−80 mm) → 경유점 1 개를 거친다.
  **기둥 검사 (2026-11-21)**: `COLUMN_BOXES` = IK arm.xml column 메시를 상자 3 개로 (세로 기둥 x 0.30~0.38·y −0.19~−0.11, 위 가로대, 기둥에 붙은 Holder; sim 월드). 움직이는 팔·손 메시 꼭짓점(8 mm 격자로 솎음, 한 자세 0.4 ms)이 상자에서 `COLUMN_MARGIN` 20 mm 이상 (`Planner.column_low`). HOME 나가기 경유점·계획 전 프레임(`checks.column_min_mm`, 걸리면 check failed)·HOME 버튼 길에 쓴다. HOME 자세가 52.3 mm. **'집을 수 있는 곳' 지도는 아직 기둥을 안 본다** (지도 초록인데 계획이 기둥으로 거절될 수 있음: 책상 앞 끝 기둥 쪽 구석, plan_check 60 번 중 1 번 8.9 mm).
  **HOME 나가기 경유점**: ① upper_arm_roll 0 → 180 만 (곧은 팔이 제자리에서 돎 — 팔꿈치를 굽힌 채 돌리면 앞팔이 기둥 쪽으로 휩쓸어 기둥을 판다) ② `HOME_EXIT_MID_DEG` shoulder_roll −40, elbow −120, wrist_pitch 60, wrist_yaw 85 ③ 책상 위 손목 자리 (sim x 0.18, HOME 손목 y, z_clear + 4 cm `HOME_EXIT_LIFT` 의 해석 IK 해 10.4/0/180/−133/0/90/0 — `HOME_EXIT_SEED_DEG` 로 고름). 책상 여유 46.3 mm, 기둥 여유 52.3 mm, 편도 19.6 s.
  ②는 기둥·책상 검사로 격자 검색(shoulder_roll −40..40, elbow −130..−60, wrist ±85, 관절 한계에서 5° 이상)해 고름. ①을 빼는 길은 모두 기둥을 판다 (잠깐 쓴 elbow −40·wrist −30/−30 12 s 길은 서보가 기둥 40 mm 안, 그 전 값 shoulder_roll −40·elbow −130·wrist −45/90 은 손가락이 기둥 6 mm 안).
  plan_check N=30: seed 1 30/30, seed 2 29/30 (실패 1 = 기둥 8.9 mm, 위).
- '집을 수 있는 곳' 지도(초록, `GET /api/ik/graspmap`, 한 점 `POST /api/ik/graspable`) = **이 계획기가 그 자리에서 집기·놓기를 할 수 있음** (2026-11-21). 판정은 `pick_plan.Planner.point_check` 한 곳(지도·계획이 어긋나지 않게): 손끝을 큐브 중심 + 1 cm 에 두는 해 → 손가락 책상 여유 8 mm → 집는 점을 곧게(수직, 안 되면 15·30° 비스듬히) 3 cm 올리는 길(계획의 내려가기와 같은 걸음 `_tip_step`, 0.5 cm 마다) + 그 위 손목을 옮겨 갈 높이까지 → 큐브 yaw 0~75°(15° 마다, 큐브는 90° 마다 같고 hand_roll ±88° 로 모두 맞춤) 모두에서 벌린 손가락이 큐브·책상 여유에 안 걸림 → HOME 에서 그 위까지 옮겨 오는 길. 칸마다 수십 ms 라 y 줄을 도우미 worker 여러 개(`MAP_PARTS`)로 나눠 약 65 초(이 PC, 서버 켤 때 미리 계산). 검사 `dashboard/tools/plan_check.py gm.json 30 1`(초록 칸 둘 무작위로 계획 30 번).
  - **지도가 많이 줄었다**: 예전 초록 1211 칸 중 797 칸은 잡을 때 손이 8~13° 기울 수밖에 없어(손목 관절 한계) 손가락이 책상 위 0.2~5 mm — 8 mm 여유를 못 지킨다. 271 칸은 큐브 위로 곧게 못 올라감(팔꿈치 ±135°·손목 한계). 남는 곳은 대부분 팔 끝 쪽 띠(y ≈ −0.30~−0.37 m)와 어깨 옆 몇 칸 (99 칸).
  - **집는 점 +1 cm (2026-11-21 사용자 결정)**: `GRIP_ABOVE_CENTER = 0.010` (hand_ik, 지도·계획·IK 탭이 같이 씀) → 수직 손에서 쥔 손가락이 책상 위 18.5 mm. 손 기울기 문(`VERT_TILT` 0.5°, 여유 대신이던 것)은 끔 — 손가락 여유 검사가 판정. 초록 **99 → 432 칸**(기울기 문을 그대로 두면 50 칸). 지도 계산은 약 35 → **약 400 초**(이 PC, 지나는 칸이 많아 느려짐 — 서버 켤 때 미리 계산, 시간 상한 `MAP_TIMEOUT_S` 1500 s; 예전 120 s 상한이면 지도가 시간 초과로 안 나옴). plan_check N=30: seed 1 **29/30**, seed 2 **29/30** playable(잡을 때 손 기울기 최대 15°). 실패 2: 옮겨 가는 길 손가락 7.9 mm(여유 8 바로 아래), 놓고 빠질 때 손가락↔큐브. 기록 `dashboard/logs/grip35_20261121/`(git 밖).
  - 패드: 손가락 끝 판 안쪽 면(MuJoCo 메시)은 집는 점 기준 손 축으로 −16 ~ +34 mm, 그중 큐브 면 1 mm 안에 드는(먼저 닿는) 곳은 판 끝 −16 ~ +2 mm. 집는 점 3.5 cm 면 닿는 곳 = 책상 위 19~37 mm 로 큐브 옆면(0~50) 안. 판 전체(50 mm)는 2.5 cm 때도 윗면 위로 9 mm 나와 있었고 3.5 cm 면 19 mm — 판이 큐브 높이만큼 길어 '판 전체가 면 안' 은 어떤 높이로도 안 됨(책상 여유 8 mm 때문). 손이 기울 때 패드가 면에 붙는지·실제로 쥐어지는지는 물리 시뮬이 없어 모름.
- 궤적: `pick_place` 의 기존 `--save` 옵션으로 `logs/pick_dry_traj.json`(joint_names, dt, positions rad, real/sim 좌표, status)을 쓰게 하고 `GET /api/ik/trajectory` 로 읽는다(arm_control 코드는 안 고침). 팔이 못 닿으면(종료코드 3) 계획이 없어 파일을 안 쓰므로 큐브·놓을 자리만 보인다(실행마다 옛 파일은 지움).

## 강화학습 탭 (`#rl`, `/rl`, 2026-11-24) — 역기구학 집기와 따로 (시뮬 + 실물 재생 + 카메라)

대시보드 **'강화학습' 탭**(역기구학 탭 바로 옆, `#rl`)이고, 같은 화면이 별도 페이지 `http://localhost:8080/rl` 로도 열린다(같은 `static/rl.js` 를 `JaxRL.mount(컨테이너)` 로 붙임 — 탭은 처음 열 때만 만든다. 요소 id 는 전부 `rl-` 접두사). team-jax/IK `sim2sim` 의 체인(정책 `grasp_lift_23025`, 45 → 256 → 256 → 11, 조회표 `raise_grid`·`pregrasp_grid`)을 MuJoCo 로 한 번 돌려 궤적을 3D 로 재생한다. **시뮬 부분은 실물 팔·CAN·ROS 에 아무것도 보내지 않는다**(실물 재생은 ③ 카드, 아래). 역기구학 탭·`pick_plan`·`hand_ik` 와 서로 안 부른다.

- 서버: `rl_run.py`(검증·한 번에 하나·120 초 제한) → 서브프로세스 `rl_worker.py <IK> run|info`. `GET /api/rl/info`(학습 범위 격자·정책 정보·메시 유무), `POST /api/rl/run {x_cm,y_cm,yaw_deg,mass_g}`(범위: x ±60, y ±80, yaw ±180, 무게 20~100 g). IK 레포는 `JAX_IK_DIR`/`../IK` 의 `sim2sim`(`sim-table-friction` 브랜치에 있음).
- 화면: `static/rl.html`·`rl.js`·`rl.css`(대시보드 `app.css` 그대로). 위에서 본 지도(로봇 틀, 초록 = 정책이 학습한 범위)를 눌러 큐브 위치를 고르고, x·y·큐브 각도·무게를 숫자로도 넣는다. 오른쪽 3D 는 재생·슬라이더·속도·단계(들어올리기→자세 안정→밀어 넣기→쥐고 들기→복귀) 버튼.
- 위치 오프셋 ↔ 로봇 틀: 기준 큐브 (0, 0) = 로봇 틀 (−0.048, −0.285) m, `로봇 틀 x = −0.048 − y/100`, `로봇 틀 y = −0.285 + x/100`. 책상은 정책이 본 장면(어깨에서 22.5 cm 앞, 윗면 어깨 아래 47.75 cm).
- 3D 는 기존 `viewer.js`(arm.xml)를 쓴다. 두 모델(sim2sim `robot_v3` 장면, arm.xml)은 몸체 이름이 같고 **관절 짝·부호가 같다** — 같은 관절값을 넣은 몸체 위치가 팔 7관절·손가락 1·2 에서 0.00 mm 로 일치(`rl_worker.world_to_real` 이 실행마다 다시 맞추고 0.1 mm 넘으면 거절). 한 가지 다른 점: 손가락 3 의 `RotationBody` 관절을 정책은 0 → −172° 로 돌리는데 arm.xml 은 −π 로 굳혀 있다 → 뷰어에 `setPose` 키 `rl_rotation` 을 추가(`q0 · Rot_z(θ+π)`, 오차 0.0 mm). 다른 탭은 이 키를 안 줘서 그대로.
- 확인(2026-11-24): `rl_worker` 4 지점(유효 3 = 성공, 학습 범위 밖 (30, −40) = 'not all three digits touched the cube' 실패), 잘못된 입력 거절, `/rl`·정적 파일 200, 헤드리스 Firefox 로 페이지·지도·초록 범위 렌더, `rl.js` 를 실제 서버에 붙여 실행→재생(517 프레임)→끝에서 정지→지도 클릭 좌표 변환(가짜 DOM·뷰어 하네스). **3D 캔버스는 실제 브라우저에서 못 봤다**(헤드리스 스크린샷이 메시 로딩 전에 찍힘).

**실물 재생 (③ 카드, 직접 CAN 모드에서만)** — `POST /api/rl/play_real {run_id, ack_table, max_speed, force?}`, 상태 `GET /api/rl/real_state?run_id=`(읽기만). 시뮬 궤적을 `rl_run.real_traj` 로 대시보드 궤적(관절 14개, `finger_rotation = rl_rotation + π`)으로 바꿔 **역기구학 실물 재생과 같은 경로**(`check_trajectory`·`drop_unverified`·`RealArm.play_trajectory`: 원점 4개, 서보 한 번은 보냄, 시작 자세 5° 안, 추종 감시 15°/0.5 s, STOP)로 보낸다. 그 앞에 RL 전용 거절: 서버가 방금 돌린 `run_id` 만, 시뮬에서 성공(`done`)·학습 범위 안(`valid_spot`)만, 책상 확인(`ack_table`: 어깨 앞 22.5 cm·윗면 어깨 아래 47.75 cm), 실물 보정 범위에 1° 넘게 잘리는 궤적은 force 로도 거절(예: 큐브 각도 45° 는 `wrist_pitch` −74° 가 필요한데 허용 −20~90°).
- **열린 고리**: 시뮬 관절 궤적을 그대로 따라갈 뿐 실물 관측으로 정책을 다시 돌리지 않는다. 폐루프(실시간 관측으로 60 Hz 정책)는 만들지 않았다.
- 시작 자세: AK45 = HOME(모든 관절 0°), 서보 = 손목 0°·**손가락 마디1 0°**·`finger_rotation` 180° (정책이 RotationBody 를 0 → −172° 로 돌린다). **역기구학 HOME 의 손가락(마디1 −14.9°)·`finger_rotation` 0° 와 다르다** → HOME 에 있어도 '시작 자세와 다름'이 뜬다. 카드의 **'서보를 시작 자세로 보내기'**(이 궤적의 첫 프레임 서보 값, 확인창 1번)로 맞춘다 — 서보는 램프로 도는 중엔 몇 초 걸린다. 역기구학으로 돌아갈 땐 각도 제어 탭 **'서보 한 번에 보내기'**(이제 `finger_rotation` 0° 포함, 서보 10개).
- **실물 팔로는 시험하지 않았다** (가짜 AK45·STM32 `--fake-hw` 로만: 거절 경로 6개, 안전 조건을 맞춘 뒤 재생 시작·중복 거절·멈춤).
- 책상: 이 정책은 책상 앞 끝 22.5 cm 장면(정우 deploy)용이고 동작 1 실물 배치(14 cm)와 다르다 — 확인란이 그 때문.

**카메라 (카메라 카드)** — 3D 에 카메라 본체·축(x 빨강·y 초록·z 파랑 = 보는 방향)·시야 원뿔(깊이 0.3~1.0 m = `vision_node` `depth_range_m`)을 그린다(`viewer.setCamera`, 변환 = `vision_node.camera_to_robot` 과 같은 `Rz·Ry·Rx·p + t`, 수치로 확인). 위치·자세는 **레포에 확정값이 없다**('폰 고정 방법 미정') → 직접 넣는다(브라우저에 저장, '예시 값'은 실측 아님). 고른 큐브 자리가 시야 안인지 알려준다. `POST /api/rl/vision {path, camera_to_robot[6]}`: 카메라 프로그램의 `inference.json` 을 ROS 없이 `vision_node/camera_result.py` 순수 함수로 읽어(후보 정확히 1개, 3초 넘게 안 바뀐 파일 거절) 로봇 틀 위치 → RL 위치 오프셋을 칸에 채우고 3D 에 보라 상자로 보인다. 변환한 큐브 높이가 책상 위 큐브 중심과 3 cm 넘게 다르면 경고. yaw 는 근사.

**지도 모양 (2026-11-24)** — 강화학습 탭 지도는 역기구학 탭 지도와 **같은 SVG·CSS 클래스·좌표식**(1 m = 1000 단위, 로봇 = 위, 앞(책상) = 아래, 로봇 왼쪽 = 오른쪽)이다: 로봇 상자·어깨 (0,0)·'앞 (책상)' 화살표, cm 눈금, 파란 칸(정책이 학습한 범위) / 회색(못 집음), 회전하는 주황 큐브(+ 방향선), 초록 점선 놓을 자리 + 큐브에서 화살표, 분홍 십자 손 위치, 보라 사각형(카메라가 본 큐브), 책상 앞 끝(어깨에서 22.5 cm) 선. 위에 모드 버튼 3개, **'칸 중심에 붙이기'**(정책 조회표 격자 2 cm 의 칸 중심 — 끄면 0.5 cm, 손 위치는 1 cm), **큐브 yaw 슬라이더**(숫자칸과 양방향), **'3D 에 초록으로'**(`viewer.setGraspArea` 로 3D 책상에 반투명 초록, 역기구학 탭과 같은 함수), **'갈 수 있는 곳 보기'**(`GET /api/ik/reachxyz`, 손 위치 높이에서 IK 가 풀리는 곳 — 켜면 파랑 대신 초록, 처음 한 높이는 몇 초). 정책 격자(`/api/rl/info` 의 `valid`)를 `{xs, ys, rows, step}` 로 바꿔 2D 와 3D 가 같은 데이터를 쓴다. 끌기 동안은 누른 순간의 모드를 유지하고 놓을 때 다음 모드로 넘어간다.

**역기구학 탭 함께 고친 것 (2026-11-24)**: ① `hand_ik.home_arm` — 수치 IK(`pair`·손 위치) 시작·끌어당김 자세를 모든 관절 0 으로(IK 레포의 sim HOME 은 위팔 143° 가지라 '어깨 옆 0° 고정'에서 20건 중 18건이 '둘 다 안 풀림', 고친 뒤 0건·12건 둘 다 풀림). `JAX_HOME_FROM_IK=1` 이면 예전 값. ② 지도에서 큐브 → 놓을 자리를 고르면 자동으로 '손 위치' 모드로 넘어간다(버튼 순서대로).

**HOME 으로 돌아오기·실물 3D (강화학습 탭, 2026-11-24)** — '끝나면 HOME 으로 돌아오기'(기본 켬)면 놓기 → 손 빼기 뒤 시작 자세(팔 모든 관절 0°, 손가락·회전몸체도 시작 값)로 돌아와 궤적 끝 = 첫 프레임이라 바로 다시 실물 재생할 수 있다. ㄴ자에서 팔꿈치를 바로 펴면 손이 책상을 치므로(시뮬 확인) `rl_worker._go_home` 은 ① 쥐는 점을 책상 윗면 + 12 cm 로 ② upper_arm_roll +90°(팔뚝을 옆으로 — −90° 는 손가락이 책상을 침) ③ 팔꿈치·손목 펴기 ④ upper_arm_roll 0. 놓을 자리가 없으면 **집었던 제자리에 내려놓고** 온다(쥔 채 팔을 늘어뜨리면 옆으로 쥔 큐브가 미끄러져 빠진다, 시뮬 3/3). 손 빼기는 손가락을 연 뒤 0.3 s 기다리고 5 cm 뒤·3 cm 위(예전 7 cm·1 cm 는 손가락이 큐브를 8 cm 끌고 간 경우가 있었다). 학습 범위 안 7 경우 모두 HOME 오차 0.0°·책상 닿음 없음·놓은 자리 0.25~2.4 cm. 놓기·HOME 단계 계산(IK)은 실물 보정 범위(`server._rl_real_limits`, 연결돼 있으면 그 팔의 메모리 보정) 안에서만 푼다. 실물 재생 중엔 3D 가 서버 `play.frame`(같은 `run_id`)을 0.25 s 마다 따라간다.

**놓을 자리·손 위치 (강화학습 탭, 2026-11-24)** — 지도에 역기구학 탭처럼 세 모드 버튼(큐브 놓기 / 놓을 자리 고르기 / 손 위치)이 있고 한 번 고르면 다음 모드로 자동으로 넘어간다(끌기 동안은 누른 모드 그대로).
- **놓을 자리 = 정책이 집고, 쥔 손 방향 그대로 평행이동해서 놓는다.** 정책(`grasp_lift`)은 집어서 들고 복귀할 뿐 놓을 곳을 모르고, 정책의 집기는 '옆에서 쥐기'(손 접근축이 수평)라 역기구학 계획기의 '위에서 쥐기'와 이을 수 없다(쥔 채 손을 90° 돌리면 큐브를 놓친다). `rl_worker.run_with_target`: ① 들어올린 순간 상태를 얻고 ② 손 방향 고정·큐브 위치만 옮긴 관절해 `q_dest` 를 IK 로 풀고 ③ 정책의 복귀 목표 `Q_L`(ㄴ자)을 `q_dest` 로 바꿔(팀 코드는 안 고침, 모듈 상수만 임시로) 쥔 채 옮기고 ④ 내리기 → 손가락 열기(정책의 처음 손 모양) → 접근 반대로 7 cm 빼기. **전부 같은 MuJoCo 물리 시뮬**이라 놓치거나 비뚤게 놓이면 실패로 드러난다. 성공 = 책상 위(높이 6 mm 안)·목표에서 3 cm 안·기울기 10° 안, 아니면 `fail_reason` 에 이유 (`summary.place`).
- **한계 (시험 결과):** 큐브 가까이(대략 15 cm 안)만 된다 — 5·10 cm 이동은 큐브가 똑바로(기울기 0.0°) 놓이고 목표와 1.4~2.4 cm 차이, 36 cm 는 '쥔 손 방향 그대로' IK 가 안 풀려(위치 오차 46 mm) 이유와 함께 거절. 같은 자리로 옮기면 3.4 cm 밀려 실패. 놓은 뒤 팔은 빼낸 자세에서 끝난다(HOME 은 각도 제어 탭 'HOME 으로 가기').
- 실물 재생은 같은 안전 경로를 거치고 `done`(= 놓음 성공)이어야 한다. 실물 시험은 안 했다.
- **손 위치(④ 카드)**: 지도에서 '손 위치' 모드로 누르면 x, y 가 채워지고 바로 `/api/ik/hand` 로 푼다(파란 유령 + 분홍 점) → [보내기] = 역기구학 탭 '손 위치로 보내기' 와 같은 `/api/joint/send`. 집지는 않는다.
- API: `POST /api/rl/run` 에 `target_x_cm`·`target_y_cm`(둘 다 또는 둘 다 없음, 큐브 위치 오프셋과 같은 기준). 응답 `target_robot`(3D 초록 상자), `summary.place`.

## 실물(직접 CAN) 모드 (`real_arm.py`, `direct_api.py`) — 기본

한 프로세스(대시보드 서버)가 can0 을 연다(python-can socketcan). ak45_node + stm_bridge_node + arm_control_node + homing_guard 가 하던 일을
스레드 2개로 한다(robot_arm v2 `class AK` 의 구조를 CAN 프레임으로):

- **rx**: `bus.recv` → AK45 피드백 0x29 파싱(`arm_control.protocol.parse_ak45_feedback`), CAN 에러 프레임(python-can socketcan 이 에러 프레임을 켠다) 집계 → bus-off·passive·warning, bit-stuff·no-ack 수. 우리가 안 보낸 AK45/서보 명령 프레임이 보이면 '다른 프로그램이 명령 중' 경고.
- **control 50 Hz**: AK45 마다 목표 → 사다리꼴 램프(30 °/s, 90 °/s², 한 스텝 3°) → carrot(측정 ±30°) → 위치 프레임(`ak45_pos_frame`), 서 있으면 20 Hz 재송신. 끊김·튐·막힘·온도·에러코드 처리. 서보는 `arm_control.motion.Mover` 램프 → `servo_frame`, 1 Hz 재전송.
- 원점: `arm_control.homing.Homing`(gap_s = 5 s). 원점을 잡는 순간 그 AK45 가 지금 자리에서 홀드를 시작한다(homing_guard 와 같음). '쉬게 하기' = 원점 지움 + 명령 막음, 홀드는 계속(힘을 빼려면 '힘 빼기').
- 보정: 연결 탭 고급의 '관절 하나씩 보정' = `calibrate.py` 의 m ±N(한 번에 ±10° 까지, 원점 없이 상대 이동) / z(지금 = sim 0; AK45 는 세션 원점도 여기로) / f / k / v / w(calibration.json 저장).
- 궤적 실물 재생: 역기구학 탭 '실물로 재생'(확인창 2번) → `play.check_trajectory`(치명 = 거절, 검사 문제 = 세 번째 확인창 = `--force`) → `play.play` + `watch.TrackingWatch`(15°, 0.5 s) → 벗어나면 멈춤·그 자리 홀드. 원점 없는 AK45 가 있으면 시작 안 함.
- 연결 거부: `/tmp/ak45_ctrl.lock` 이 잠김(ak45_node 가 떠 있음), 대시보드 밖 ROS 노드(ak45_node·stm_bridge_node·arm_control_node·homing_guard·calibrate), 대시보드가 켠 ROS 항목, can0 DOWN/없음.
- 실물 모드에서 가상으로 바꾸려면 팔 연결을 먼저 끊는다. 서버 종료(Ctrl+C) = 브레이크 0 A 후 닫음. 로그 `logs/direct.log`.
- API: `POST /api/direct/connect {max_speed}`, `/api/direct/disconnect`, `/api/direct/hold`, `/api/direct/limp {joint|"all"}`, `/api/direct/calib {action: move|zero|flip|scale|verify|save, joint, value}`, `/api/ik/play_real {force}`, `/api/ik/stop_real`, `GET /api/direct/log`. `/api/stop` = STOP(홀드), 원점·보내기는 다른 모드와 같은 API.
- 시험: `python dashboard/tools/try_direct_fake.py` (가짜 AK45 4개 + 가짜 STM32, `tools/fake_can_arm.py`). 아래 '확인한 것'.
- 검사 지적 반영(2026-11-20): 재생은 `play.play` 대신 자체 루프(첫 프레임 홀드 동안에도 STOP·고장 확인, 멈추면 지금 자리 홀드, 멈춤 확인과 보내기를 한 잠금 안에서),
  짧은 끊김 뒤 위치가 3° 넘게 다르면 원점 지움(전원 재투입·영점 변화), rx 스레드 보호·버스 다시 열기·죽은 스레드 다시 띄움(화면 CAN 버스 빨강),
  보정 ±N° 의 기준 = 측정 위치, 브레이크 2 ms 간격·재시도·연속 송신 실패 경고, 누르기 멈춤(3 A·2°·1 s, 가정 — `press_amps` 0 = 끔), 온도 경고 5 ℃ 히스테리시스, 닫는 중엔 명령·위치 프레임 막음.

### 실물 첫 시동 체크리스트 (직접 CAN)

1. 대시보드를 켜기 전, 팔을 손으로 받치고 한 모터의 지금 각(candump 의 0x29 피드백)을 읽은 뒤 **그 각으로 위치 프레임 1개만** 보낸다
   (예: elbow ID 4 가 지금 0.0° 면 `cansend can0 00000404#00000000`, 값 = int32 BE deg×10000). 1 s 뒤 살짝 밀어 홀드 토크가 남는지 본다.
   real_arm 은 "프레임이 끊기면 AK45 힘이 빠진다"(0.25 s 정도)를 **가정**한다(실측 없음). 힘이 남으면 close()·힘 빼기의 브레이크가 꼭 필요하고, 빠지면 짧은 끊김 홀드(5 s)가 팔을 붙잡는 근거가 맞다.
2. `ip -details link show can0` 로 txqueuelen 확인(기본 10). 브레이크·홀드 프레임이 ENOBUFS 로 버려지지 않게 `sudo ip link set can0 txqueuelen 100` 권장. 화면 CAN 버스에 '송신 실패'가 뜨면 멈춘다.
3. 무부하 홀드 전류와 팔 무게 홀드 전류를 기록 → `press_amps`(기본 3 A, 가정)를 그보다 넉넉히 크게. 바닥을 살짝 눌러 '누르기 멈춤'이 뜨는지 확인.
4. 피드백 끊김 시험: 한 모터 CAN 선을 1 s 뽑았다 꽂기 → 원점 유지·홀드 확인. 전원만 1 s 껐다 켜기 → 원점 지움·지금 자리 홀드 확인(엔코더 영점이 바뀌는지도 기록).

## 가상 팔 모드 (`virtual_arm.py`)

화면 위 스위치(또는 `--virtual`)로 바꾼다. 가상 모드에서는 ROS 자식 프로세스 대신 `VirtualArm` 이 같은 모양의 상태를 채우고, 노드 켜기/끄기·각도 보내기·원점·쉬게 하기·궤적 재생이 다 돈다. 화면 곳곳에 보라색 **가상** 배지·띠가 붙는다. 실물 노드가 켜져 있으면 가상으로 못 바꾼다. 로그는 `logs/virtual/<id>.log`.

| 가상 노드 | 흉내 내는 규칙 |
|---|---|
| ak45 | 첫 명령 전엔 프레임 없음, \|목표\| > 360° 거부, 마지막 목표 홀딩(timeout 없음), 피드백·워치독 없으면 안 보냄, 60 ℃ 이상 잠금, `/joint_states` 50 Hz(0.1° 양자화), `/diagnostics` 2 Hz(같은 키). 모터는 목표를 1차 지연(τ 0.08 s) + 60 °/s 로 따라감. 힘이 없으면(노드 꺼짐·전원 꺼짐) 0° 쪽으로 40 °/s 로 떨어짐. 온도는 전류²로 오르고 식음(가상 숫자) |
| stm_bridge | `/servo/command` 를 0~180° 로 잘라 `/servo/state`(보낸 값)로. 서보는 즉시 그 각(가정) |
| arm_control | `arm_control/arm_control/calibration.py`(변환·클립)와 `motion.py`(Mover 램프)를 **import 해서 그대로** 씀. 서보 첫 명령은 바로 감 |
| homing_guard | `arm_control/arm_control/homing.py`(Homing)를 import, homing_guard.py 의 on_state / on_diagnostics / on_command / on_timer / home·release 서비스 순서를 따라 함(arm_homed 일 때만) |
| vision | fixed_pose 파라미터 또는 역기구학 탭의 가상 큐브 칸 → `/object_pose`(shoulder_roll_axis) 1 Hz |

- **가상 전원 끄기/켜기**(연결 탭): 피드백이 끊기고 힘이 빠짐 → 다시 켜면 엔코더 영점이 15~40° 틀어진다. guard 가 원점을 지우고 제자리 홀드를 보내고, 그 관절 명령이 막히는지 볼 수 있다.
- 한 번 실행형: can0 올리기·vcan 시험은 '가상: 할 일 없음'(종료코드 0). `pick_place --dry-run` 은 가상 모드·Windows 에서도 **진짜로** 이 PC 의 python 으로 돈다(arm_control·stm_bridge 를 sys.path 에 넣고 `pick_place.main`; mujoco·gymnasium·scipy 와 team-jax/IK 필요).
- 역기구학 탭 **가상 팔로 재생**: `arm_control/arm_control/play.py` 의 `schedule`·`play` 를 그대로 써서 궤적 프레임을 가상 arm_control_node 로 보낸다(실물 재생은 대시보드에 없음). `check_trajectory` 의 치명 거절(not delivered 등)은 막고, --force 가 필요한 검사는 알림만.
- 가상 숫자(모터 속도·온도·떨어지는 속도·영점 틀어짐)는 가정이다. 실측 아님.

## ROS 모드 (`--ros`, 팀 ROS 버전) — ROS 연결 (자식 프로세스)

`--ros` 일 때만 쓰인다. 이때 연결 탭은 ① can0 ② ROS 연결 ③ 팔 켜기(arm_homed.launch.py), 고급에 노드 따로 켜기·보정 안내가 보인다.

서버는 rclpy 를 import 하지 않는다. 구독·발행(`ros_monitor.RosMonitor`)은 `ros_bridge.py` 가 띄우는 자식에서 돈다:
`bash -c "source /opt/ros/$JAX_ROS_DISTRO/setup.bash; source $JAX_WS/install/setup.bash(있으면); exec python3 -u dashboard/ros_monitor_proc.py"`.
자식은 0.25초마다 상태 JSON 줄을 stdout 에, 원점 서비스 호출과 관절 명령은 stdin JSON 명령으로 받는다. stdin 이 닫히면 끝난다. stderr 는 `logs/ros_monitor.log`.
구독: `/diagnostics`, `/joint_states`, `/ak45/joint_states_homed`, `/arm_control/homing`(transient local), `/servo/state`, `/object_pose`. 발행: `/arm_control/joint_command` 하나(보내기 버튼일 때만, 처음 보낼 때 publisher 를 만든다). 서비스: homing_guard 의 Trigger.

## 동작 규칙 (노드 켜기/끄기)

- 항목마다 새 프로세스 그룹: `bash -lc "source …; exec stdbuf -oL -eL ros2 …"`. 로그 `dashboard/logs/<id>.log`(git 무시).
- 끄기: 그룹에 SIGINT → 5초 → SIGTERM → 3초 → SIGKILL. 어느 단계에서 끝났는지 카드에 표시.
- 같이 켜면 안 되는 것은 거절: `arm.launch.py` ↔ `arm_homed.launch.py` ↔ 개별 `ak45`/`stm_bridge`/`arm_control`. 터미널에서 같은 노드가 떠 있어도 거절.
- `can0 올리기`: 이미 UP 이면 그대로, 아니면 `sudo -n` → `pkexec` 순서. 안 되면 로그에 명령.
- 대시보드 서버를 꺼도 노드는 그대로 둔다(끄면 팔이 떨어질 수 있음). 다시 켜면 `logs/pids.json` 으로 다시 붙잡는다(Linux).
- 보정(`calibrate --ros`)은 키보드 대화형이라 웹에서 안 돌린다(안내 + 준비 버튼만).
- 원점: `POST /api/homing/home|release {"joint": "<관절>" | "all"}` → homing_guard 서비스(가상 모드면 가상 guard).
- 기타 API: `POST /api/mode {"virtual": bool}`, `POST /api/virtual/power {"on": bool}`, `POST /api/virtual/cube {x,y,z,yaw}`, `GET /api/reachmap?yaw=<실 yaw °>`(칸 중심 실좌표 + grasp/place/feasible/되는 손 돌림, `JAX_IK_DIR` 의 motion1/src/humanoid_grasp_rl), `POST /api/reachmap/check {"cube":[x,y,yaw],"target":[x,y]}`(realcoords.plan 결과 + 큐브·놓을 자리 따로 판정), `GET /api/ik/trajectory`, `POST /api/ik/play_virtual`, `POST /api/ik/stop_virtual`.

## 항목 추가

`dashboard/services.py` 의 `SERVICES` 에 `Service(...)` 한 줄(필드 설명은 파일 맨 위). 가상 모드에서 노드로 흉내 내려면 `virtual_arm.SERVICE_NODES` 에도 넣는다(아니면 한 번 실행형은 '할 일 없음', argv 항목은 그대로 실행).

## 확인한 것

### 역기구학 탭 ③ 손 위치로 보내기 (`hand_ik.py`, `POST /api/ik/hand`, `POST /api/ik/fk`)

- 손이 쥐는 점(큐브 중심) x, y(지도 [손 위치] 모드로 누르거나 숫자) + 높이(책상 위 cm, 책상 = 어깨 아래 0.4775 m, 기본·프리셋 3.5 cm = 큐브 중심 + 1 cm, 2026-11-21) + 손 방향(잡을 큐브의 yaw) → [풀기] → 3D 에 파란 유령 + 분홍 목표점 → [보내기].
- 풀이: team-jax/IK `env.py`·`ik.py`(solve_site_pose) 를 서버의 자식 프로세스(`hand_ik.py --worker`, JAX_SETUP=real, 첫 호출 때 모델 읽기 약 1초)에서 그대로 쓴다. 큐브 세 면(+0, ±90°) × 손 그대로/반 바퀴 × 시작점 5개(지금 자세·home 등)를 풀고, 관절 한계 = 모델(손 비틀기 ±88°) ∩ calibration 의 보낼 수 있는 범위, 위치 5 mm·방향 1.5° 안이면 풀림, 그중 손가락이 책상에 안 묻히고 지금 자세에서 가장 가까운 해.
- 손 자세: **기본 = 큐브 잡는 자세**(실물 셋업과 같은 손가락 아래 + −40° 기울임, 파지 중심은 손 축 따라 21.5 mm 위 = env.pick_pose). **손끝 수직(0°)** 은 옵션: 큐브 높이에선 어깨 가까운 일부(격자 48점 중 15점, 기울인 손은 39점)만 되고 책상 위 10 cm 부터는 0점.
- 실패 이유: 책상 아래 / 팔이 안 닿음(손 방향 빼고도 n mm) / 손 비틀기 서보 범위 밖(±120 이면 풀림) / 그 손 방향으로는 안 됨.
- [보내기] = 각도 제어 탭과 같은 `/api/joint/send`(같은 막힘 규칙, 같은 첫 확인창) + 첫 사용 때 "관절 공간 직선 이동, 중간 경로 충돌 검사 없음" 확인창. 손가락은 그대로(체크하면 벌림). 팔이 목표에 닿으면 FK 로 손 위치를 다시 재서 목표와의 mm 차이를 보인다.
- 자동 풀기: 슬라이더(x, y, 높이, 손 방향)·지도 끌기 중에는 빠른 풀이(`fast: true`, 직전 해에서 한 번 80 반복, 같은 손 모양, 요청은 최신 것만), 손을 떼거나 숫자를 바꾸면 300 ms 뒤 전체 풀이. 빠른 풀이가 못 따라가면 유령이 빨강(마지막 좋은 해)이고 바로 전체 풀이 한 번. [다시 풀기] 는 작은 버튼.
- 어깨 옆으로 벌리기 고정(`fix_deg: {shoulder_yaw: 0}`, 2026-11-20 부터 기본 끔): 7 관절 → 6 관절이라 해가 거의 하나. 대신 닿는 곳이 크게 줄어든다(큐브 높이 격자 24점: 고정 없음 22, 0° 고정 4, 10° 1, 20°·30° 3). 안 풀리면 고정 없이 다시 풀어 "고정 때문 — 풀면 풀림(shoulder_yaw +19°)" 처럼 알린다.
- 손 자세(2026-11-20, 사용자 "−40° 고정 빼"): 기본 **손가락 아래 (기울기 자동)** — 손가락이 닫히는 축(손 x, 손가락 1·2 ↔ 3) 둘레 기울기는 ±상한(기본 60°) 안에서 자유(그 축 둘레로 돌려도 패드가 같은 두 면에 닿음). 풀이: 쥐는 점 3 + 손 x 축 방향 2 를 주 과제, 남는 자유도로 기울기 → 0·HOME 쪽, 시작 기울기 0/−40/+40 세 번(가지 고르기). "수직만", "−40° 고정(팀 IK 원래 값)" 도 고를 수 있다. 큐브 높이 격자 24점(x −0.10…0.15 × y −0.35…−0.20, 책상 위 2.5 cm): 어깨 옆 0° 고정 켬 **−40 고정 4점 → 기울기 자동 24점**, 끔 23 → 24. 사용자 점(−0.03, −0.36, 2.5 cm) 고정 켬 −40 안 됨 → 자동 됨(기울기 −2°). 높이 10/20 cm 는 고정 켬이면 대부분 안 됨(6점 중 1). 전체 풀이 약 1.4~1.8 초(기존 0.6~1.3), 빠른 풀이는 직전 해에서 한 번. 기록: `logs/tilt_20261120/`.
- 지도 판정(2026-11-20, 사용자 "도달 지도 만들지 마라"): 칸 색칠 기본 끔(격자만, 어디든 클릭). 큐브·놓을 자리를 찍으면 `POST /api/ik/pair` 가 실제 IK 로 같은 파지(손 돌림·반 바퀴·기울기)로 집기·놓기가 둘 다 풀리는지 판정(약 0.06~0.6 초, 손 위치 카드의 어깨 옆 고정·기울기 상한을 따름). 큐브 중심 = 책상 위 2.5 cm, 손가락–책상 충돌·책상 모서리(큐브가 책상 위에 서나)는 안 본다. 옛 −40 도달 맵 색칠·판정은 "옛 도달 지도 보기" 체크로만.
- 팔도 바로 따라가기(기본 끔): 0.2초마다 같은 `/api/joint/send` 로, 한 번에 20° 넘게 바뀌는 해는 안 보냄(그 점은 건너뛰고 경고, 이어지는 점에서 다시 따라감). 켤 때 팔이 유령에서 20° 넘게 떨어져 있으면 먼저 [보내기] 로 옮길지 묻는다.
- 해 고르기(2026-11-20, 사용자): 벌린 손가락이 책상에 닿는 해는 **탈락**(빠른 풀이도) → 지금 자세(AK45 피드백, 서보는 보낸 값, 없으면 직전 보낸 해 `near_deg`, 그것도 없으면 HOME)에서 **최소 회전**(관절 최대 이동각, 관절각은 실제로 도는 각이라 ±180 감김 안 함) → 그 최소에서 2° 안이면 손 기울기 수직에 가까운 것. 전체 풀이 후보에 직전 보낸 해에서 기울기를 묶지도 당기지도 않고 이어 푼 후보를 더한다. 빠른 풀이는 기울기·HOME 당김 없이 직전 해에서만. 결과 줄 첫머리에 "지금 자세에서 최대 N° 회전".
- 기준점(2026-11-20, 사용자): 기본 **손목 중앙** = 손목 굽힘(wrist_yaw) 축과 손목 비틀기(wrist_pitch) 축의 교점(arm.xml 에서 두 축 직교·교차, wrist_yaw 기준점에서 그 축 따라 20 mm, Wrist_yaw 몸통 고정점 — 앞 4 관절로만 정해짐). `ref: "wrist" | "grip"`, 높이 칸 = 기준점의 책상 위 높이. 손 자세 기본 **수직**(기울기 자동·−40 은 '고급'). 수직 손이면 손목 중앙은 집는 점보다 24.35 cm 위(옆으로 약 4 cm) → 기본 높이 27 cm, 슬라이더 25~65 cm, 기준점을 바꾸면 높이도 그만큼 바뀜.
- 시험(가상 8098, 수직·손목·손목 높이 26.85 cm = 집는 점 2.5 cm, `tools/wrist_grid.py`): 어깨 옆 고정 켬 24점 중 5 + 사용자 점(−0.03, −0.36) 됨(안 되는 19점 = 고정 때문 14점: 가장 가까운 해 6.7~29.6 mm · 1~5°, 팔이 안 닿음 5점: x 0.10~0.15), 끔 19/24 + 사용자 점. 풀린 해 FK 손목 위치 오차 최대 2.0 mm. 기록 `logs/wrist_grid_vertical.txt`.
- 손 자세 기본(2026-11-20, 사용자 "손 각도는 무시"): **위치만** (`mode: "position"`) — 기준점 위치 3 만 맞추고 손목 3 관절(wrist_pitch, wrist_yaw, hand_roll)은 지금 값(서보 보낸 값, 없으면 직전 보낸 해·HOME) 그대로 보낸다(손목 안 움직임). 앞 4 관절만 풀고 해 고르기·손가락–책상 탈락·빠른 풀이·따라가기 같음. 수직/기울기 자동/−40 은 '고급'. 손목 높이 27 cm 격자 24점+사용자 점: 고정 끔 20/25, 켬 19/25 (`logs/wrist_grid_position.txt`).
- **따라가기 진단 로그** `dashboard/logs/follow.log`(4 MB 넘으면 `follow.log.1` 로 돌림, git 무시): 따라가기가 켜져 있는 동안 브라우저가 1초마다 `POST /api/log/follow` 로 보낸 JSON 한 줄씩. `ev` = `tick`(0.2초 틱: `sent`, `why` = 보냄 / 풀이 없음 / 풀이 실패 / 오래된 풀이 / 보내기 막힘 / 변화 없음 / 20° 초과 / 보내기 거절: 서버 문구, `d`·`dj` = 직전 보낸 해 대비 관절 최대 변화와 그 관절, `sol` = 그 풀이), `solve`(빠른/전체 풀이 응답: `kind` fast/full, `ok`, `pos_err`, `flip`, `turn`, `tilt`, `q` 7관절, `seed_from`, `d_sent`), `solve_drop`, `send_manual`, `follow_on`/`follow_off`(+이유), `mark`(재현 스크립트 구간). 모든 줄에 `ts`(서버 시각), `t`(페이지 ms), `tgt`(x, y, zc 책상 위 cm, yaw, mode, fix), `arm`(지금 팔 관절), `sent_ref`(직전 보낸 해), `n_sent`. 요약: `python dashboard/tools/follow_log_summary.py [로그]`. 재현(가상 서버, headless Edge): `python dashboard/tools/follow_repro.py --port 8098 --fix 1 --yaw 20`.
- 속도(가상 8098): 서버 빠른 풀이 0.3~7 ms, HTTP 왕복 2.5 ms(3D 숨김) — headless Edge(소프트웨어 WebGL)에선 3D 그리기가 느려 화면 갱신 약 3 회/초, 왕복 35~47 ms. GPU 있는 실제 브라우저는 안 재 봄.
- 확인(2026-11-20, 가상 8098): 큐브 높이 3점 + 15 cm + 수직 손 1점 풀림·보냄 → FK 손 위치 목표와 0.6~1.2 mm; 30 cm·안 닿는 곳·책상 아래·수직 (0.10, −0.30) 은 이유와 함께 거절.

### 실물(직접 CAN) — 가짜 하드웨어 (2026-11-20)

- Windows 11, Python 3.13, python-can 4.6.1 `virtual` 버스, `tools/try_direct_fake.py` 34/34 통과(약 50 초, 2번): 첫 명령 전 무송신(1 s 동안 AK45 프레임 0, 피드백 50개씩), 원점 없으면 거절, 원점 → elbow 0→20° 0.91 s(최대 ~30 °/s), 피드백 1.5 s 끊김 = 홀드 프레임 계속(1 s 에 17개)·팔 안 떨어짐·원점 유지, 7 s 끊김 = 5 s 에 송신 멈춤·원점 지움·돌아오면 지금 자리 홀드, 엔코더 +70° 튐 = 지금 자리 홀드·원점 지움(축 안 움직임), 붙잡기 = 2.5 s 뒤 힘 뺌(브레이크 3개), 62 ℃ 경고 / 76 ℃ 힘 뺌·원점 거절, 에러 코드 2 힘 뺌, STOP = 그 자리 홀드, 서보 첫 명령 바로·다음 램프, 궤적 재생 끝까지 / elbow 붙잡으면 TrackingWatch 로 멈춤, 보정 ±5°·영점·방향·확인·저장(임시 복사본), 힘 빼기 = 브레이크 12개 후 위치 프레임 0, 브레이크 프레임 `00000203#00000000`.
  검사 지적 시험 14개(시험마다 새 가짜 팔): 재생 첫 프레임 홀드 중 STOP → 그 자리(90° 로 안 감) / 재생 멈춤 버튼 → 지금 자리 홀드 / 프레임 재생 중 STOP → 목표 안 덮어씀, 전원 1 s 껐다 켜며 엔코더 영점 30° → 원점 지움·축 그대로(전엔 축 50° 로 감), 5.05 s 경계 피드백 → 지금 자리, rx 처리 예외 60개에도 rx 삶·화면 빨강, rx 스레드 죽음 → 다시 띄움, recv 실패 → 버스 다시 엶·'수신 오류' 로그 한 줄, 막힌 관절 보정 +10° ×10 → 목표 +10°(전엔 +100°), 송신 3번 중 2번 실패에도 모터마다 브레이크 3개 / 계속 실패 → 화면 경고, 누르기(전류 4.3 A·오차 8°) → 목표 풀어 줌·힘·원점 유지 / 끄면 안 함, 온도 62/57/54 ℃ 경고 켜짐/유지/풀림·76 ℃ 뒤 65 ℃ 원점 거절·58 ℃ 허용, 재생 중 닫기 → 브레이크 뒤 위치 프레임 0.
- WSL Ubuntu 22.04: `/tmp/…lock` 을 `flock` 이 쥐고 있으면 `take_lock` 거절, 우리가 쥐면 `flock -n` 이 막힘(ak45_node 쪽 거부). **WSL 커널에 vcan 이 없어(CONFIG_CAN_VCAN 꺼짐) SocketCAN 경로(에러 프레임 포함)는 못 돌렸다.**
- `server.py --port 8099 --fake-hw` + curl: 연결 전 보내기 거절 → 연결 → 원점 없음 거절 → 원점 → elbow 30·shoulder_yaw 20·wrist_pitch 15 → STOP → 현재 자세 표, 연결 중 가상 모드 거절, 보정 서보 이동, 관절 힘 빼기, 연결 끊기 → 가상 → 실물. headless Edge 스크린샷(연결 탭·각도 제어 탭).

### 탭 3개 + 3D + 각도 제어 + 가상 팔 (2026-11-20)

- Windows 11, Python 3.13, `--port 8097 --virtual`, `JAX_IK_DIR` = team-jax/IK 클론(925e023): `/robot/meshes/Upper_Arm_1.obj` 200(933942 B), `..%2Fik.py` 404, vendor three 200. `node --check app.js`, viewer.js(모듈) 문법 통과.
- 가상 모드 curl: arm_homed(가상) 켬 → `elbow 40` 거절 '원점 없음' → 서보 `wrist_pitch 30` 은 보내짐 → 전부 원점 → `elbow 40` 보냄 → 피드백 0.5초 간격 14.4 → 34.8 → 40.0°(30 °/s 램프) → 가상 전원 끄기/켜기 → 4관절 '원점 없음'(feedback back after 2 s), `elbow 10` 거절 → elbow 다시 원점(오프셋 −26.8°, 20° 넘음 경고 그대로) → 전부 쉬게. 범위 밖(`wrist_pitch 120`), 모르는 이름, `"NaN"` 문자열 거절.
- headless Edge(swiftshader) 스크린샷: 연결 탭, 각도 제어 탭(메시 / 막대 모델), 각도 보내기 3초 뒤·12초 뒤 두 장(3D 자세가 램프로 움직임). 역기구학 탭은 headless 스크린샷에 3D 캔버스가 빈칸으로 찍혔다 — 같은 페이지에서 `canvas.toDataURL()` 로 뽑으면 팔·큐브·놓을 자리가 제대로 그려져 있어(그리기 호출 27개) headless 합성 문제로 본다(실제 브라우저에서 못 봄).
- 로컬 pick_place --dry-run(가상 모드, `JAX_DASH_PYTHONPATH` 에 mujoco 4.0.1): 계획 playable, 놓은 오차 2.5 mm, 1601 프레임, 검사에서 거절(종료코드 1: hand_roll 이 −113.4° 까지 가는데 서보 범위로는 −90° 까지 → 잘림, calibration 미확인). 궤적 파일 → 3D 재생, 가상 팔로 재생(3.2배 느리게 약 106초) 돌아감.
- 모드 바꾸기: 가상 → 실물(Windows: '이 PC엔 ROS 없음', 보내기 거절, 가상 전원 API 거절) → 다시 가상. 가상 vision_node(fixed_pose) → `/object_pose` 값.
- **WSL Ubuntu 22.04 + ROS Humble**(로봇 PC 는 Jazzy), colcon 없이 arm_control_node + homing_guard 를 python 으로 직접(arm_homed 와 같은 리매핑, max_speed 30) + 가짜 ak45(받은 `/ak45/command` 를 60 °/s 로 따라가는 `/joint_states`) + 가짜 stm(`/servo/command` → `/servo/state`). 대시보드는 ROS source 없이(`env -i`, venv fastapi 0.162) `JAX_ROS_DISTRO=humble`:
  구독자 1·guard 감지 → elbow 거절(원점 없음) → 전부 원점 → `elbow 30, wrist_pitch 20` 보냄 → `/ak45/command` ak45_4 가 …29.4, 30.0 으로 램프, `/servo/command` W_PITCH 110°(= 90 + 20) → 대시보드 현재 자세 elbow 9.0 → 30.0°(3D 가 쓰는 값). 스택을 끄면 구독자 0 → 보내기 거절. 시험 프로세스 모두 끔.

### 이전 (탭 6개 시절, 2026-11-20)

- 노드 켜기/끄기·가짜 항목·SIGINT/SIGTERM 단계·파라미터 검사·WS 스냅샷(Windows), 원점 패널(WSL Humble + homing_guard 직접 실행), ROS 자식 프로세스 연결·죽음·다시 연결(WSL). 자세한 기록은 git 이력(c319bc1 이전 README).

## 확인 안 한 것

- 손 위치로 보내기의 어깨 옆 고정은 이 카드에만 적용된다. 큐브 집기 계획(①②, pick_place → team-jax/IK)에는 안 들어간다(IK 레포 수정이 필요).

- **직접 CAN 실물 전부**: 실제 can0(SocketCAN)·CANable 에서 real_arm, 에러 프레임 해석, bus-off 때 송신 실패 처리, 실제 AK45 가 위치 프레임이 끊기면 힘을 놓는지(가정), 20 Hz 홀드로 충분한지, carrot·막힘 기준이 실물 무게에서 맞는지, 실제 STM32 가 1 Hz 재전송에 문제 없는지. Linux vcan 으로도 못 돌림(WSL 에 vcan 없음).
- **ROS 모드 실물 전부**: 로봇 PC(Jazzy)에서 실제 `ros2 launch/run`, 실제 모터로 '보내기'(방향·영점 미확인 상태), can0, 진단 카드, pids.json 다시 붙잡기.
- 실제(headless 아닌) 브라우저·폰에서 3D 조작감과 성능(메시 8.7 MB 를 한 번 읽음).
- Ubuntu apt 의 fastapi 0.101 에서 도는지.
- 가상 팔의 숫자(속도·온도·중력)는 실물과 대조 안 함.

### 직접 CAN 남은 문제 (2026-11-20 검사)

- 전원 재투입 직후 **첫 피드백 전에** 나간 홀드 프레임은 막지 못한다(그때는 위치를 모름). 첫 피드백이 오면 3° 검사로 원점 지우고 지금 자리 홀드 — 그 사이(부팅 → 첫 피드백, 실물 미측정) 모터가 옛 명령 쪽으로 조금 움직일 수 있다. 가짜 모터에선 축 움직임 < 1°.
- 3° 기준(`resume_rehome_deg`)·누르기 3 A 는 실물 홀드 오차·전류를 재기 전의 가정. 실물 홀드 오차가 3° 를 넘으면 짧은 끊김마다 원점이 지워진다(안전 쪽 실패).
- 누르기 멈춤은 명령 하나에 한 번만: 계속 누르면 막힘(12°/2 s)·온도(75 ℃)가 마지막 방어선. 전류 부호·단위(0.01 A)는 매뉴얼 기준, 실물 미확인.
- 버스 다시 열기는 python-can virtual 에서만 확인(실제 USB-CAN 빼고 꽂기·`ip link set can0 down/up` 은 안 해 봄).

## 다음 단계

1. 로봇 PC 에서 가상 모드 → vcan 시험 → 실물 순서로 확인. 실물 첫 보내기는 작은 각(2~5°) 한 관절, 팔 받치고.
2. calibration 확인(verified=true) 전에는 AK45 를 크게 움직이지 않는다. hand_roll 범위(서보 0~180 → sim ±90 vs 시뮬 ±120) 정리 필요 — pick_place 궤적이 −113° 를 쓴다.
3. (직접 CAN 에서 됨) STOP = 지금 자리 홀드, 힘 빼기는 따로. ROS 모드의 정지는 여전히 노드 종료(팔 떨어짐).

## 큐브 실시간 인식 + 가상 큐브 장면 (`live_api.py`, `static/live.js`) (2026-11-24)

휴대폰 카메라 · 탑 쌓기 · 강화학습 탭이 **같은 '지금 책상 위의 큐브들'** 을 쓴다. 사진 올리기는 어디에도 없다.
**가상 모드는 카메라를 아예 안 읽는다** — 큐브는 서버의 가상 장면뿐이다.

**실물 모드 = 영상** — 서버 스레드(`live_api.Live`)가 폰 스테레오 프로그램(SteroDepthSeg)의 결과 폴더를 20 Hz 로 보고, 새 프레임마다
ArUco (`phone_detect.detect_cubes`: 4x4 ID 0~4, 층, 기울기) + 그때의 뎁스 결과 (`phone_depth.evaluate`) 를 계산해 두고 시각과 함께 남긴다.
휴대폰 탭이 집을 큐브 하나는 예전과 같은 `phone_fuse.find` (ArUco 4x4 ID 0 + 뎁스, 안 맞으면 막음) 결과다.

| 프레임 | 빠르기 | 내용 |
|---|---|---|
| `<결과 폴더>/live.jpg` (있고 3초 안에 바뀌었으면 이것) | 폰 프로그램이 쓰는 만큼 (아래 고침: 10 Hz) | 손대지 않은 렌즈 그림 |
| `<결과 폴더>/inference.png` (지금 팀 프로그램 그대로) | **1초쯤** (추론이 끝날 때 1초 넘었으면 저장) | 정류한 왼쪽 그림 + 객체 테두리·글자가 덧그려짐 (마커를 가릴 수 있음). 오른쪽 뎁스 색 칸(+ 눈금 70 px)은 잘라 낸다 |

- **영상 없음**: 마지막 프레임이 **1.5초** 넘게 오래되면 `stale` → 큐브 목록을 비우고 휴대폰 탭 '집으러 가기'·탑 쌓기·강화학습 '영상 인식' 들기를 막는다.
  inference.png 만 있을 때는 저장 간격 자체가 1~1.8초라 1.5초로는 계속 깜박인다 → 그때만 한계를 **저장 간격 중앙값 × 1.6 (1.5 ~ 3초)** 로 늘린다
  (화면 출처 줄에 지금 한계가 보인다). **진짜 실시간(≥ 5 Hz)은 아래 SteroDepthSeg 고침이 있어야 한다.**
- 책상 높이 d 는 화면이 보낸 값(역기구학 탭 칸)으로 뎁스 높이 검사를 하고, 바뀌면 마지막 프레임을 다시 계산한다.

**SteroDepthSeg 에 필요한 작은 고침 (팀 저장소에 안 올림 — 거기서 할 일)**: `z_phone_stereo.py` 에 옵션 하나와 루프 몇 줄.

```python
ap.add_argument("--live-jpg", type=float, default=0, metavar="HZ", help="손대지 않은 첫 렌즈 프레임을 <out>/live.jpg 로 HZ 번/초 (대시보드 실시간 인식)")
# main() 루프 안, frames, pair, ... = receiver.snapshot() 바로 뒤:
if a.live_jpg and frames and frames[0] is not None and now - last_live >= 1.0 / a.live_jpg:
    cv2.imwrite(str(out / "live.tmp.jpg"), frames[0].bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
    os.replace(out / "live.tmp.jpg", out / "live.jpg")      # 반쯤 쓴 그림을 읽지 않게
    last_live = now
# (루프 전에 last_live = 0)
```

대시보드의 '폰 프로그램 켜기' 는 그 파일에 `--live-jpg` 가 보이면 `--live-jpg 10` 을 붙여 띄운다 (없으면 안 붙임 — 옛 판은 그대로 inference.png).

**API**

- `GET /api/live/cubes?d=<책상 높이>&pick=<ID>` → `{ok, mode 'camera'|'virtual', stale, reason, age_s, stale_s, src 'live.jpg'|'inference.png'|'virtual', rate_hz,
  t_frame, frame_n, cubes: [{id, x, y, yaw_deg, level, source 'aruco'|'aruco+depth'|'depth'|'virtual', age_s, tilt_deg?}], refs, warn,
  table {d, measured (마지막 '책상 높이 재기')}, pick {휴대폰 탭이 집을 큐브 — 예전 /api/phone/live 모양}, pipeline, scene {jitter, last_move}}`.
  뎁스만 보인 큐브는 `id: null` (탑 쌓기는 안 씀). 화면(`live.js`)은 이 세 탭이나 역기구학 탭이 보일 때 0.5초마다 읽는다.
- `GET /api/live/frame.jpg` 마지막 프레임 (마커를 그린 것). `GET /api/phone/live` 는 남김 (= 위 pick + fresh, pipeline).
- `POST /api/live/scene {op: add|move|yaw|delete|clear|jitter, …}` — 가상 모드에서만.

**가상 장면** (`live_api.Scene`, 파일 `logs/virtual_scene.json` — 서버를 다시 켜도 남음)

- 지도 (휴대폰 탭 ① · 탑 쌓기 탭은 지도 위 '가상 큐브' 스위치 · 강화학습 탭 '영상 인식'): **빈 곳 = 큐브 놓기** (다음 빈 번호 0~4, 5개까지), **끌기 = 옮기기**
  (다른 큐브 위에 놓으면 그 위에 쌓이고 자리·각을 맞춤), **누르기 = 고르기** → 아래 줄의 각 칸 · 지우기. 밑에 깔린 큐브는 못 옮기고 못 지움.
- **가상 팔 재생이 끝까지 가면** (`VirtualArm.play_trajectory(on_end)`) 그 계획의 `real` (집은 자리 → 놓을 자리, 놓는 각 `target_yaw`, 없으면 큐브 각)
  대로 집은 자리의 맨 위 큐브를 놓을 자리로 옮긴다 (그 자리에 큐브가 있으면 그 위). 중간에 멈추면 안 옮긴다.
- **놓을 때 어긋남** (기본 끔): 옮길 때 자리 ±1 cm · 각 ±5° 를 무작위로 더한다.
- 3D: 역기구학·탑 쌓기·강화학습 탭 3D 에 가상 큐브 상자들 (계획이 따로 그리는 큐브는 뺌).

**휴대폰 탭**: 결과는 `pick` (실물 = 예전 합치기 결과). 영상이면 프레임 그림 + 출처 줄 (어느 그림, Hz, 몇 초 전), 영상 없음이면 '집으러 가기' 막음.
지도에서 다른 번호의 큐브를 누르면 그 ArUco 큐브를 집는다 (쌓인 큐브는 막음). 가상 모드는 폰 프로그램·책상 높이 카드를 숨기고 가상 큐브만.

**탑 쌓기 탭 (실시간, 한 걸음씩 — 2026-11-24 저녁 바뀜)**: 순서·탑 자리 입력 밑에 **걸음 목록** — `1층: 10번 옮기기` · `2층: 11번 옮기기` · `3층: 12번 옮기기`,
걸음마다 버튼 하나와 한 줄 이유, 그 밑에 상태 한 줄. **잠금·기다리는 고리는 없앴다**: 서버 `stack_api.steps(cubes, base, order, placed)` 가 **지금 실시간 인식 + 순서·탑 자리만으로**
완료·다음·막음을 정한다. 그래서 1층을 놓고 다른 일을 하다 와서, 페이지를 다시 열거나 서버를 다시 켜도, 카메라가 탑을 알아보면 2층을 누른다.
순서·탑 자리는 서버 `logs/stack_plan.json` (`GET/POST /api/stack/plan`, 칸을 바꾸거나 걸음을 누를 때 저장). 그림은 `/api/live/stream.mjpg`.

- **책상 위에는 큐브 하나씩이면 된다** (사용자: 여러 개면 손이 부딪힌다) — 순서의 다른 큐브가 안 보여도, 탑 속 큐브 수가 순서와 안 맞아도 막지 않는다.
  **탑 높이를 믿는다**: 탑 = 탑 자리 2.5 cm 안의 바닥 큐브 + 8 cm 안의 쌓인 큐브, 높이 = 보이는 맨 위 큐브의 층 + 1 (아래 층은 가려져도 됨).
  층 L < 높이 → **완료** (보이면 그 번호, 가려졌으면 이 화면이 놓은 기록의 번호). 층 L = 높이 → 다음 걸음.
- **다음 걸음의 큐브** = 순서에서 아직 탑에 없는 첫 번호가 책상 위에 보이면 그것, 없으면 **책상 위에 보이는 다른 큐브** (`2층: 보이는 큐브 13번 옮기기`).
  **순서는 비워도 된다** (= 보이는 큐브 아무거나, 3층까지). 이미 있는 큐브 위에 쌓으려면 탑 자리를 그 큐브로 (그 큐브가 1층 = 완료).
- 놓은 기록 (`stack_plan.json` 의 `placed {층: ID}`, 끝까지 재생하면 화면이 `POST /api/stack/placed`) 은 **힌트만**: 가려진 층의 번호 표시와
  '그 큐브가 다른 층으로 보임 → 막음' 에만 쓴다. 탑 자리를 바꾸면 지운다.
- **다음 걸음 버튼이 켜지는 조건** (하나라도 아니면 꺼지고 이유):
  서버 — 탑 속 보이는 큐브가 **아래 큐브에서 2 cm 안** (아래가 가려졌으면 탑 자리에서 층마다 2 cm 씩) · **기울기 15° 안** · **기록과 같은 층**
  (예전 `check_placed` 의 판정; 걸린 층은 '✗ 문제' 로 보임), 옮길 큐브가 책상 위에 보임 (다른 큐브 위면 막음), 탑 자리 8 cm 안에 다른 바닥 큐브 없음, 3층까지.
  화면 — **팔이 HOME** (AK45 4개 피드백 + 보낸 서보 값이 0° ±5°; 서보는 보낸 값만 안다), **영상 있음** (stale 아님), 재생·계획 중 아님,
  실물은 이 화면에서 누른 재생이 끝난 뒤의 새 프레임, 그 큐브가 **집을 수 있음** (`/api/ik/graspable`, 수직), 탑 자리에 그 층까지 놓을 수 있음.
- 누르면 예전과 같다: `pick_dry` 계획 (stack 칸 = 놓을 층·다른 큐브, 놓을 자리 = **지금 보이는 맨 위 큐브 자리·각 (90° 마다 맞춤)**, 없으면 탑 자리) → 계획 결과가 그 걸음인지 확인 →
  지금 모드의 원래 재생 버튼 (확인창·검사 그대로). 가상 모드 = 가상 장면 (끝까지 재생하면 큐브가 옮겨짐), 실물 = 카메라.
- 2층 큐브가 책상으로 떨어지면 (탑에서 8 cm 밖) 2층 걸음이 다시 켜진다. 탑 옆 8 cm 안이면 '겹침' 으로 막는다. '처음부터' 버튼은 없앴다.
- `POST /api/stack/steps {cubes, base, order}` → `{ok, text, bad?, done?, height, steps: [{n, id, level, state done|bad|next|blocked|wait, text, why}], next: {cube, level, target, obstacles, params}}`.
  `POST /api/stack/next` · `check_placed` 는 없앴다.

**강화학습 탭** (※ 팀 강화학습 탭 `rl_api.py` 기준 — 2026-11-24 우리 강화학습 탭으로 바꿔 이 스위치·영상 인식 들기는 지금 없다. 우리 탭은 카메라 카드에서 `inference.json` 을 읽는다): 카드 위 스위치 **영상 인식 / 지도로 고르기**. 영상 인식이면 큐브 칸 (x, y cm, 각) 이 지금 큐브를 따라간다 — 실좌표 → 정우 님 틀은 지도와 같은
변환 (`info.map` 의 ref·ax·ay; 각은 틀이 90° 돌아 있어 큐브에선 같은 값, −45~45). 여럿이면 지도에서 눌러 고른다 (기본 = 책상 위 가장 작은 번호),
가상이면 그 지도에서 가상 큐브를 놓고 옮길 수 있다. 영상 없음 · 큐브 없음이면 들기 막음. 실물·가상 팔로 강화학습 결과를 재생하는 것은 이 작업에 없다.

**시험** — `tools/test_live_cubes.py` (폰 없이): 그린 프레임을 inference.png (뎁스 칸 붙인 모양) 로 → 큐브 10 (1층)·12 (2층) 1 cm 안 · pick · 쌓인 큐브 막음,
live.jpg 5 Hz 로 바뀜, 1.5초 끊기면 영상 없음 → 다시 오면 풀림, 가상 장면 규칙 · 끝까지 재생 때만 옮김 · 어긋남 범위 · 파일에 남음, steps 걸음 목록 13가지 (완료·가려짐·어긋남·기울기·기록과 다른 층·떨어짐·겹침·큐브 하나씩·보이는 큐브로 대신·순서 비움).
**확인 (2026-11-24, 가상 8097, `JAX_DASH_LOGS` 따로, `JAX_DASH_MAP_PARTS=2`)**

- `tools/test_live_cubes.py` 통과 (inference.png 에서 큐브 1 cm 안·기울기 0.9~1.1°, live.jpg 5.0 Hz, 끊김 2.3초 → 영상 없음 → 풀림).
- 탑 쌓기 끝까지 (헤드리스 Chrome, 진짜 마우스로 지도에 가상 큐브 3개 (0.02, −0.27)·(0.10, −0.21)·(−0.03, −0.36) 놓기 → 확인창 → 가상 팔 재생 3번):
  어긋남 끔 — 탑 (−0.16, −0.28) 에 10·11·12 가 1·2·3층, 걸음마다 '놓인 큐브 확인'. 어긋남 켬 — 탑 (−0.15, −0.29): 놓일 때 −9·−7 mm / +9·+2 mm / −8·+1 mm 어긋났고
  다음 큐브는 매번 실제 맨 위 큐브 자리·각에 계획돼 3층 완성 (JS 오류 0). 어긋남 켬 + 탑 (−0.16, −0.28) 에서는 2층이 (−0.167, −0.282) 에 놓여 3층째를
  계획기가 '팔이 못 닿음' 으로 거절했다 (3층은 x −0.14~+0.01 의 좁은 띠 — 탑 자리는 그 안쪽으로).
- 강화학습 영상 인식: 가상 큐브 (−0.07, −0.30) 12° → 칸 (−1.5, 2.2) cm 12° → 들기 성공 (15.9 cm), 시뮬 큐브 첫 자리 (−0.070, −0.300)·각 12° (mod 90) 로 같음
  → 내리기 (−0.12, −0.33) 계획 OK (총 51초). 가상·실물 팔 재생은 이 작업에 없다.
- `test_phone_depth/detect/fuse/table.py` 통과, `try_direct_fake.py` 39/39. 화면: 컴퓨터 폭·390 px (휴대폰·탑 쌓기·강화학습, 가로 넘침 없음).
- **확인 안 한 것**: 실제 폰·SteroDepthSeg 로 돈 적 없다 (실물 경로는 그린 프레임으로만), `--live-jpg` 고침은 그 저장소에 안 넣었다,
  기울기 15° 판정의 실제 마커 흔들림, 실물 탑 쌓기.

## 휴대폰 카메라 탭 (거치한 폰의 화면 → 큐브 집기)

로봇에 거치한 폰(Galaxy S25)의 화면에서 서버가 큐브 자리 (x, y, yaw) 를 찾고, 역기구학 탭과 **같은 계획·재생**으로 집으러 간다.
**찾는 흐름은 하나다 (2026-11-24): ArUco / 세그먼트 모드 스위치도, 사진 올리기도 없다.** 폰 스테레오 프로그램이 켜져 있으면 화면이 1초마다
`GET /api/phone/live` 를 읽고, 서버는 그 프로그램의 지금 프레임에서 ArUco 를, 지금 `inference.json` 에서 뎁스 큐브를 찾아 합친다.
결과 줄에 어느 것을 썼는지 적힌다 (`출처: ArUco + 뎁스 (…4 mm · 0° 차이로 맞음)` / `출처: ArUco 만` / `출처: 뎁스 만`).

파일: `phone_fuse.py`(합치기) · `phone_detect.py`(프레임의 ArUco → 자리) · `phone_depth.py`(뎁스 결과·프레임 읽기·좌표 변환) · `phone_table.py`(책상 높이)
· `phone_api.py`(`/api/phone/*`, 폰 프로그램 켜기·끄기) · `static/phone.js`·`phone.css`(탭 화면). `server.py`·`index.html` 에는 연결 한 줄씩만.

**합치는 규칙** (`phone_fuse.fuse`, 시험 `tools/test_phone_fuse.py`):

| 있는 것 | 쓰는 것 | 집으러 가기 |
|---|---|---|
| ArUco + 뎁스, 서로 맞음 (자리 2 cm · 각 15° 안) | ArUco 의 x·y·yaw (더 정확, 각이 정확) | 됨 |
| ArUco + 뎁스, 안 맞음 | ArUco 값을 보여 주되 경고 (얼마나 다른지, 뎁스 값) | **막음** — 다음 프레임에서 맞으면 풀림 |
| ArUco 만 (뎁스가 큐브를 못 찾음·결과 없음) | ArUco | 됨 |
| 뎁스만 (프레임에 기준 종이·큐브 마커가 없음) | 뎁스 (각은 근삿값) | 됨 |
| 둘 다 없음 | — | 무엇을 할지 안내 (폰 프로그램 켜기 / 큐브가 보이게) |

- 그대로 둔 검사: **뎁스가 큐브 높이를 책상 위 0~5 cm 밖으로 재면 막는다** — ArUco 가 같이 있어도 막는다 (거치 자세나 책상 높이 d 가 틀렸다는 뜻이고,
  d 가 틀리면 ArUco 자리로 가도 손이 책상을 누른다). 인식 경고 (기준 마커 1개, 큐브 마커 크기, 거치 자세 미보정, 폰 프로그램 경고) 는 결과 줄에 그대로,
  그 자리를 집을 수 있는지 (`/api/ik/graspable`, 지금 책상 높이 d·집는 방향) 도 그대로.
- 각 비교는 90° 마다 같은 것으로 보고, 뎁스 각이 믿을 만할 때만 (옆면·윗면으로 추정) 한다 — 못 구했거나 두 면이 섞였거나 직접 넣은 값이면 자리만 본다.
- 뎁스의 자리는 큐브 중심이 아니라 **보이는 면의 가운데**다 (옆면이면 중심에서 2.5 cm) → 거치 자세를 큐브 중심 좌표로 보정해 두지 않으면 2 cm 검사에 걸린다.
- 3초 넘게 안 바뀐 결과·프레임은 안 쓴다. 뎁스 후보가 여럿이고 점수가 비슷하면 '헷갈림' 으로 안 고른다 (큐브만 보이게 치우기).
- **프레임 = 지금은 폰 프로그램이 1초마다 저장하는 `inference.png`** (`phone_depth.read_frame`). 이 그림에는 그 프로그램이 객체 테두리·글자를 덧그려서
  마커가 가려질 수 있다 (**실제 폰 화면으로는 확인 안 함**). 합치기·찾기 함수는 전부 '프레임 배열 + 그때의 뎁스 결과' 를 받는다
  (`phone_fuse.find(image, cam_cfg, depth)`, `phone_detect.detect / detect_cubes`, `phone_table.measure_aruco`, `stack_api.detect_frame`) —
  영상 스트림을 직접 받게 되면 `read_frame` 만 바꾸면 된다.

화면: ① 큐브 찾기 — 폰 프로그램이 꺼져 있으면 큰 [폰 프로그램 켜기] 하나, 켜져 있으면 그림 (마커가 보이면 마커를 그린 프레임, 아니면 폰 프로그램 그림) +
결과 한 줄 (x·y·yaw, 출처, 세부·경고) + 집을 수 있는지. ② 집으러 가기 — 역기구학 탭의 큐브 칸을 채우고 ▶ 계획 돌려 보기(`pick_dry`)를 돌린 뒤 지금 모드의
원래 재생 버튼을 누른다 (확인창·검사·force 확인은 전부 원래 것, 놓을 자리는 역기구학 탭의 값). 책상 높이 카드. 접힌 **준비** — 인쇄물·기준 종이 자리,
폰 프로그램 켜기·끄기, '뎁스 맵 저장', 거치 자세, 큐브 각 직접 넣기, 폴더.

없앤 것: `POST /api/phone/detect`(사진 올리기), 모드 스위치(`ph_mode`), 책상 높이의 사진 올리기, 탑 쌓기 탭의 `POST /api/stack/detect`.
`GET /api/phone/depth`(뎁스만의 결과) 는 남겼다 (화면은 안 씀).

### ArUco 준비 (기준 종이 · 큐브 마커)

1. **인쇄** `dashboard/phone_markers/phone_markers.pdf` (A4 가로 2쪽, **실제 크기 100%** — 1쪽의 100 mm 선을 자로 확인).
   다시 만들기: `python3 dashboard/tools/make_phone_markers.py`.
   - 1쪽 = 기준 종이 (ArUco 5x5 `DICT_5X5_50` ID 0~5, 32 mm). 책상에 평평하게 붙인다: '로봇 쪽' 글자가 로봇을 향하고 긴 변이 로봇 좌우와 나란하게,
     **가운데 십자가 어깨 축 바로 아래에서 앞으로 32 cm · 로봇 오른쪽으로 5 cm** (실좌표 x −0.05, y −0.32 = 집을 수 있는 곳 한가운데).
     다른 자리에 붙였으면 탭 아래 '준비' 에서 종이 가운데 (x, y)·돌아간 각을 넣고 저장 (`logs/phone_cam.json`).
   - 2쪽 = 큐브 윗면 마커 (ArUco 4x4 `DICT_4X4_50` ID 0, 40 mm). 50 mm 선으로 오려 큐브 윗면에 붙인다.
   - 큐브(4x4)와 기준 종이(5x5)는 ID 0~4 가 겹치지만 사전이 달라 사전마다 따로 찾는다 (`phone_detect.split_markers`, 한 자리를 두 사전이 다 읽으면 둘 다 버림).
2. **폰에서 열기**: 서버를 `--host 0.0.0.0` 으로 띄우고 폰 브라우저에서 `http://<노트북 IP>:<포트>/#phone`.
   (암호가 없다 — 같은 네트워크 누구나 팔을 움직일 수 있으니 시험할 때만.)
3. 거치한 폰의 화면에 기준 종이 마커 2개 이상 + 큐브 마커가 보이면 ArUco 자리가 나온다.

큐브 윗면은 책상보다 5 cm 높아서, 기준 마커로 카메라 자세를 구해 그 높이 평면에 쏜다 (비스듬한 화면에서 5~7 cm 보정).
초점거리는 모르므로 훑어서 고른다. 확인값 = '큐브 마커 한 변' 이 40 mm 근처로 나오는지 (10% 넘게 다르면 경고).
시험: `python3 dashboard/tools/test_phone_detect.py` (가짜 프레임 3장, 위치 1 cm·yaw 3° 안).

### 폰 뎁스 — 거치한 S25 의 두 렌즈 깊이 (2026-11-24)

팀 폰 스테레오 프로그램 ([team-jax/SteroDepthSeg](https://github.com/team-jax/SteroDepthSeg): Galaxy S25 렌즈 2·5 → FFS 깊이 → YOLO seg + `lib_blobs` 가 책상 위로
튀어나온 덩어리를 찾음) 이 1초쯤마다 덮어쓰는 `<결과 폴더>/inference.json`·`inference.png` 를 읽는다. 큐브 고르기와 카메라 → 로봇 좌표 변환은
팀 `jax_ros2` `vision-select` 의 `vision_node` (`camera_result.py` 의 `select_cube`·`camera_to_robot`, `tools/solve_camera_to_robot.py`) 를
ROS 없이 그대로 옮긴 것이다 (`phone_depth.py`). 거치 자세 풀기 `tools/solve_phone_mount.py`, 시험 `tools/test_phone_depth.py`.

- **각(yaw)은 근삿값**이다. 폰 프로그램의 yaw 는 '보이는 면이 카메라에 대해 돌아선 각'이라 그대로 못 쓰고, 그 면의 법선을 로봇 좌표로 돌려
  구한다 (옆면이 보이면 법선의 수평 방향, 윗면이면 면 안 모서리 방향). 큐브는 90° 마다 같아 −45~45° 로만 나온다. 작은 큐브에서는 10° 안팎
  틀릴 수 있고 두 면이 섞여 잡히면 더 틀린다 — ArUco 가 같이 보이면 ArUco 각을 쓰고, 뎁스만일 때 틀려 보이면 '준비' 에서 각을 직접 넣는다.

**실험실에서 할 일** (이 노트북에는 2026-11-24 기준 저장소만 받아 둠 — `venv_ffs`·FFS 가중치·`adb` 없음, GPU RTX 4060 은 켜져 있음):

1. 폰 스테레오 환경 (한 번): `sudo apt install adb python3.12-dev` (python3.12 필요), 그다음
   `cd ~/Documents/SteroDepthSeg && bash tools/setup_ffs.sh` (torch 2.6 cu124 + xformers + FFS 가중치, 몇 GB).
   YOLO seg 가중치는 처음 실행 때 내려받는다. 확인: `./venv_ffs/bin/python z_ffs_smoke.py`.
2. 폰: Galaxy S25 (SM-S931N, `calib_phone_pair.json` 이 이 폰 전용) 를 USB 로 연결, USB 디버깅 허용. 앱이 없으면 한 번
   `./venv/bin/python z_phone_stereo.py --install --list` (그 저장소 README; JDK 필요). 폰을 로봇 거치대에 끼우고 잠금(PIN)을 푼다.
3. 켜기: 탭의 **폰 프로그램 켜기** (꺼져 있으면 ① 카드에 크게, 끄기는 '준비'; 서버가 아래 명령을 `--headless` 로 띄움, 기록 `logs/phone_depth.log`), 또는 터미널에서 직접
   ```bash
   cd ~/Documents/SteroDepthSeg
   ./venv_ffs/bin/python z_phone_stereo.py --physical 2 5 --calib-stereo calib_phone_pair.json --yolo seg --out phone_stereo/runs/dash
   ```
   (`--headless` 를 빼면 창이 뜬다. 결과 폴더를 바꾸면 탭 '준비' 의 결과 폴더도 같게.) 시작 후 정류 검사가 끝날 때까지
   (정지한 쌍 3개, 0.6 m 이상 떨어진 질감 있는 장면) 객체에 `rectification_unverified` 경고가 붙는다 — 거치한 채로 경고가 안 없어지면
   폰을 잠깐 빼서 방을 비춘 뒤 다시 끼운다.
4. **거치 자세 보정 (꼭)**: 처음 값은 팀 `camera.yaml` 의 CAD 임시값이라 화면에 경고가 뜬다. 큐브를 로봇 좌표를 아는 자리 3곳 이상
   (일렬이 아니게, 집는 범위에 넓게) 에 놓고, 놓을 때마다 결과 줄의 '뎁스: … 카메라 좌표' 를 적어
   ```bash
   python3 dashboard/tools/solve_phone_mount.py --d 0.4775 --save \
       --point "cx,cy,cz:-0.05,-0.33" --point "cx,cy,cz:0.05,-0.30" --point "cx,cy,cz:-0.10,-0.38"
   ```
   (`--d` = 역기구학 탭의 책상 높이. `--save` 가 `dashboard/logs/phone_depth.json` 에 쓴다 — 서버 재시작 필요 없음, 화면 '준비' 칸은 새로고침.
   다른 logs 폴더를 쓰는 서버면 `--cfg`.) 잔차가 0 이어도 부호를 잘못 넣은 것은 못 잡으니, **큐브를 새 자리에 놓고 화면의 x·y 가 자로 잰 값과
   맞는지** 확인한 뒤 가상 팔 → 실물 순서로 집어 본다.

시험 (폰·GPU 없이): `python3 dashboard/tools/test_phone_depth.py` — 팀 `camera.yaml` 의 검증값 3개·손 계산값과 변환이 같은지, 실제 파일에서
따온 큐브 + 잡음에서 큐브만 고르는지, 로봇 좌표에 둔 큐브를 카메라 좌표로 거꾸로 옮겨 만든 inference.json 에서 자리(1 mm)·각(0.5°)을 되찾는지,
`solve_mount` 가 아는 자세를 되찾는지. **확인 안 한 것**: 실제 폰·FFS 로 돈 적 없다 (실제 큐브 기록 파일은 저장소에 없음), 거치 자세 미보정,
각 추정의 실제 정확도, 팔이 화면에 들어왔을 때 후보가 헷갈리는 정도. 팀 fixture 의 실제 큐브 값 (cam 0.0715, 0.0906, 0.4457) 은 CAD 임시
자세로 옮기면 책상 위 21 cm 로 나온다 — 거치대에서 찍은 값이 아니다.

### 책상 높이 재기 (휴대폰 카메라 탭, 2026-11-24)

탭 아래 **책상 높이** 카드의 [책상 높이 재기] → 결과 한 줄 (`책상 높이 d = 0.474 m (ArUco) · 뎁스 0.476 m · 차이 2 mm`) → [역기구학 탭에 넣기] 가
역기구학 탭의 '책상 높이' 칸을 채운다 (자동으로는 안 넣는다 — d 가 크게 틀리면 손이 책상을 누르므로 사람이 보고 누른다).
범위 0.44 ~ 0.53 m (`hand_ik.D_RANGE`) 밖이면 값만 보여 주고 넣지 않는다. 코드 `dashboard/phone_table.py`, `POST /api/phone/table_height` (본문 없음).

- **폰이 로봇 거치대(CAD 자리)에 끼워져 있어야 한다.** 보정 단계가 없고 정우 `CAMERA.md` (JaxIsaacLab `artifacts/deploy_2026-11-05`, CAD 2026-11-12) 의
  거치 사양을 그대로 믿는다: 카메라 = 프로파일 중심선 앞 117.7 mm, 어깨 피치축 − 7 mm (= 어깨 roll 축 − 12 mm), 앞을 보고 45° 아래.
  거치대에서 빠진 폰의 화면은 엉뚱한 값이 나온다.
- **모드·사진 올리기 없음**: 버튼을 누르면 거치한 폰의 지금 프레임 (`phone_depth.read_frame`) 과 뎁스 맵 중 있는 것으로 잰다 — 둘 다면 ArUco 값 + 뎁스 값 + 차이.
  ArUco: 기준 종이(인쇄물 1쪽, 100 % 인쇄)를 책상에 평평하게 두고 마커가 화면에 3개 이상 (6개 다 나오게). 종이 자리는 아무 데나 (평면만 쓴다).
  뎁스: 준비의 '뎁스 맵 저장' 을 켜고 폰 프로그램을 다시 켠다 (프로그램을 `--rgbd-viewer` 로 띄워 5초마다 `rgbd/rgbd.npz` 를 저장하게 함 — 그 점들에 평면을 맞춘다).
  프레임에 종이가 안 보이면 뎁스로만, 뎁스 맵이 없으면 ArUco 로만 잰다.
- **재는 법**: 광학 좌표에서 책상 평면을 구해 카메라~평면 **수직 거리** h 를 재고 d = h + 0.012 (카메라가 어깨 roll 축보다 12 mm 아래).
  책상이 수평이면 h 는 폰 기울기와 무관해서 거치 각 45° 가 틀려도 d 는 안 변한다. 대신 잰 평면이 CAD 자세의 '위' 와 이루는 각을 **기울기**로
  보여 주고 2° 넘으면 경고한다 (폰이 덜 끼워졌거나 거치 각이 다름). 사진이 90°·180° 돌아가 있어도 된다 (가장 가까운 방향으로 본다).
- **좌표**: 우리 (x 로봇 왼쪽, y 뒤, z 위, 원점 어깨 roll 축) = (−y_c, x_c, z_c) + (0.174, −0.1177, −0.012), (x_c, y_c, z_c) = CAMERA.md 카메라 좌표
  (x 뒤, y 로봇 오른쪽, z 위). 폰 뎁스의 기본 거치 자세 (`phone_depth.DEFAULT_C2R`, 팀 camera.yaml) 와 위치는 같은 점이고, 회전은
  '45° 아래 × 폰 뒤집힘 180°' 에서 1.4° 다르다 (IMU 로 잰 값) — 그 값은 안 바꿨다.
- **오차** (그래서 **처음 한 번은 줄자로 잰 어깨 축 ~ 책상 윗면과 맞춰 볼 것**):
  렌즈 자리 — CAD 점은 거치대 우측 아래 꼭짓점이고 렌즈는 거기서 경사를 따라 0~40 mm 위 (미반영) → d 가 **0~28 mm 크게** 나올 수 있다.
  초점거리 — 사진에서 추정 (기준 종이의 가까운 쪽·먼 쪽 크기 차이). 거리가 비례해 틀린다 (1 % ≈ 5 mm); 가짜 사진 15장에서 평균 1.1·최대 4.1 mm.
  실제 렌즈의 왜곡은 안 본다 (화면 가장자리의 종이는 더 틀림 → 종이를 화면 가운데쯤). 인쇄 배율 1 % = 약 5 mm.
  줄자와 일정하게 다르면 그 차이는 대부분 렌즈 자리다 — `phone_table.CAM_POS` 의 z 를 고친다.
- 시험 (폰 없이): `python3 dashboard/tools/test_phone_table.py [그림 폴더]` — CAD 값 맞춰 보기, CAD 자세로 그린 가짜 사진 (d 0.4725 → −1.1 mm,
  0.50 → −2.8 mm, 뒤집힌 사진, 거치 각 48° → d −0.7 mm·기울기 3.05° 경고), 가짜 뎁스 점 (오차 0.1 mm 안), 범위 밖. **확인 안 한 것**: 실제 폰 사진,
  실제 rgbd.npz (`--rgbd-viewer` 를 `--headless` 와 같이 켠 적 없음 — 코드로만 확인).

## 탑 쌓기 탭 (사진 한 장 → 큐브 여러 개 → 우리 IK 계획으로 한 큐브씩 쌓기) (2026-11-24) — **2026-11-25 없앰**

> **2026-11-25 사용자: 탑 쌓기는 없앤다.** 화면 탭과 서버 등록(`server.py` 의 `stack_api.install`)을 뺐다. 코드(`stack_api.py`, `static/stack.js`)는
> 남아 있어 되살릴 수 있다 (`server.py` 주석 두 줄 + `index.html` 탭·스크립트). 실시간 인식은 큐브를 늘 책상 위(0층)로 본다 (`live_api.LIVE_MAX_LEVELS = 1`).

> **2026-11-24 저녁: 실시간 인식으로 다시 쌓을 수 있다** — 위 '큐브 실시간 인식' 절. 아래 인용은 그 전 상태.
>
> **2026-11-24 바뀜: 사진 올리기를 없앴다.** 탭의 ① 카드는 지금 "실시간 인식 준비 중" 만 보여 주고 `POST /api/stack/detect` 는 없다 — 새 인식 결과가
> 들어올 길이 없어서 **지금은 이 탭으로 쌓을 수 없다** (거치한 폰 영상으로 계속 알아보는 기능이 다음 작업). 서버 쪽은 프레임 배열을 받는
> `stack_api.detect_frame(프레임, 종이 자리)` 로 남겼고 (결과는 `/api/stack/last` → 화면이 탭을 열 때 읽음), 계획·놓을 수 있는 곳·다음 한 걸음은 그대로다.
> 아래 '사진 찍기' 설명은 그 전 방식이다.

큐브마다 다른 ArUco 를 붙이고, 폰 사진으로 큐브들을 찾아, **역기구학 탭과 같은 계획기**(강화학습 아님)로 한 번에 한 큐브씩 탑 자리에 쌓는다.
파일: `stack_api.py`(`/api/stack/*`) · `static/stack.js`(탭 화면) · `phone_detect.detect_cubes` · `pick_plan.py --place-level/--obstacle`.
`server.py`·`index.html` 에는 연결 몇 줄만 있고 `app.js`·`phone.js` 는 안 고쳤다.

**쓰는 순서**

1. 인쇄: `dashboard/phone_markers/stack_markers.pdf` (A4 가로 1쪽, **실제 크기 100%**, ArUco 4x4 `DICT_4X4_50` ID 0~4, 40 mm, 50 mm 모서리 표시). 큐브마다 다른 번호를
   윗면에 붙인다. 기준 종이는 휴대폰 카메라 탭의 것(`phone_markers.pdf` 1쪽) 그대로, 종이 자리 설정도 그 탭. 다시 만들기: `python3 dashboard/tools/make_phone_markers.py --stack-only`.
2. **사진 찍기** — 기준 종이와 큐브 윗면이 다 나오게 **비스듬히**(30° 넘게 기울여) 한 장. 큐브 목록이 나온다: 번호, x, y, 각, 높이(층), 집을 수 있나(수직 손).
   폰으로 찍으면 PC 화면에서도 탭을 열 때 마지막 사진 결과가 보인다 (`/api/stack/last`).
3. **순서** — 옮길 큐브 번호를 차례대로 (기본 = 찾은 번호 오름차순, 예 `12, 10, 11`). 적힌 큐브만 옮긴다. 사진에 없는 번호·못 집는 자리의 큐브가 있으면 이유와 함께 막는다.
4. **탑 자리** — x, y (m, 역기구학 탭과 같은 실좌표: 어깨 축 아래가 0, +x 로봇 왼쪽, 앞 = −y) 를 넣거나 **작은 지도를 누른다**. 초록 = 그 자리에 놓을 수 있는 곳
   (진할수록 높이 놓을 수 있다: 1·2·3층). 고르면 "이 자리에 N층까지 놓을 수 있음 / 몇 층째를 못 놓음: 이유" 가 뜬다 (계획기와 같은 판정 `point_check`, 손 수직).
   순서의 첫 큐브가 그 자리 바닥(1층), 다음 큐브들이 그 위. 이미 놓인 큐브 위에 쌓으려면 지도에서 그 큐브를 누른다 (탑 자리 = 그 큐브 자리, 2.5 cm 안이면 그 큐브가 탑의 맨 위).
5. **탑 쌓기** — 한 번 누르면 큐브 하나: 계획(4~6초) → 통과하면 지금 모드의 **원래 재생 버튼**(가상 팔로 재생 / 실물로 재생)을 누른다 → 확인창·시작 자세 검사·
   stop_gen·mtime·보정 검사가 전부 원래 것. 놓고 HOME 으로 돌아오면 **사진을 다시 찍어야** 다음 큐브로 간다 (실제로 놓인 자리·탑 높이를 사진으로 확인,
   다음 큐브는 사진에 보이는 맨 위 큐브 위에 놓는다). 탑 자리·순서는 첫 큐브에서 고정되고 '처음부터' 로 푼다.

**계획** (`pick_plan.py`, 덧붙임 — 옵션을 안 주면 예전과 프레임까지 같다)

- `--place-level K`: 큐브 바닥을 책상 위 K·5 cm + 2 mm(`STACK_GAP`) 에 놓는다. 놓을 자리 판정·놓는 자세·놓인 큐브↔손가락 검사가 그 높이로.
- `--obstacle X Y YAW LEVEL`(여러 번): 다른 큐브 = 탑의 큐브들 + 책상 위 나머지 큐브. 검사: 팔·손 메시 꼭짓점(기둥 검사와 같은 것) ↔ 큐브 상자 8 mm(`OBST_MARGIN`),
  들고 가는 큐브 ↔ 큐브 5 mm(옮길 때), 내려놓는 길에선 안 파고듦. 걸리면 status 가 playable 이 아니다 (실물은 force 로도 재생 안 됨 — 기존 `analytic_fatal`).
  `checks.obstacles {arm_min_mm, carry_cube_min_mm, lower_cube_min_mm}`.
- 손은 **수직**(`grasp=vertical`, 탑 쌓기 탭이 역기구학 탭의 '집는 방향' 칸을 수직으로 바꾼다). 수평 집기는 탑 쌓기에 안 쓴다.
- 탑 가까이에서는 손목 높이를 지키는 길(`_standoff_pts`: 위로 먼저 올라가는 길이 팔 길이로 안 될 때), 가는 길 앞 60 % 는 손가락을 좁게,
  돌아오는 길은 벌린 채 멀어진 뒤 좁힘(나누는 자리·서서 좁히기 후보 중 탑에 안 닿는 첫 것).
- 화면 → 계획: `pick_dry` 의 `stack` 칸(JSON `{for, level, target_yaw, obstacles}`). 칸 값이 실행 사이에 남아도 `for`(큐브·놓을 자리)가 이번 값과 같을 때만 쓰고,
  화면은 계획 결과의 `real.place_level`·`obstacles` 가 이 걸음과 같은지 본 뒤에만 재생 버튼을 누른다.

**높이** — 기본 책상 높이(어깨 아래 0.4775 m)에서 **3층까지** (3층째 = 책상 위 10 cm 에 놓기). 4층째(15 cm)는 손목이 어깨 높이까지 올라가야 해서
(수직 손이면 손목 = 놓는 층 + 28 cm → 책상 위 43 cm + 위로 3 cm, 어깨는 47.75 cm) 어느 자리에서도 놓는 자세가 없다 (기울인 손도 0 칸).
놓을 수 있는 칸(3 cm 격자, 수직): 1층 39 · 2층 27 · 3층 11 — 3층은 x −0.14~+0.01, y −0.27~−0.33 의 좁은 띠. 책상이 낮으면(d 가 크면) 더 된다 (안 재 봄).

**자리 잡는 법 (계획이 거절하는 흔한 이유)**

- 큐브끼리 **15 cm 이상**: 벌린 손가락(첫 마디 −60°)이 닫히며 수평을 지날 때 큐브 중심에서 12 cm 쯤까지 나온다.
- 탑은 **팔이 드나드는 자리 (x 0, y −0.34: HOME 나가기 경유점)에서 15 cm 이상**: 3층 탑은 그 자리의 손보다 높아서, 가까우면 돌아가는 손가락이 놓은 큐브에 닿는다.
  3층까지 되는 곳은 (−0.15, −0.29)·(−0.17, −0.27) 근처 (기본 탑 자리 −0.16, −0.28). (−0.14, −0.30) 은 2층까지만 통과했다 (탑 자리 7곳 × 각 3개: 2층 21/21, 3층 9/21).
- 지도의 초록은 '그 높이에 놓는 자세가 있나'만 본다 (옮겨 오는 길·손가락 닿음은 계획이 본다).

**사진에서 층 알아내기** — 기준 마커로 카메라 자세를 구한 뒤, 큐브 마커 한 변이 40 mm 로 나오는 높이(5 cm 단위)를 고른다 (카메라 50 cm 위에서 한 층에 12 %).
초점거리를 모르므로 바로 위에서 찍으면 초점거리와 높이가 서로 바뀌어도 같은 사진이라 층이 불확실 → 경고를 띄운다. 아래 깔린 큐브는 가려서 안 보인다:
탑 높이 = 탑 자리에 보이는 맨 위 큐브의 층, 순서에 있는데 안 보이는 큐브 = 깔린 것으로 보고, 그 수가 탑 높이와 안 맞으면 멈춘다 (다시 찍기).

**확인** (2026-11-24, 가상 서버 8093, `JAX_DASH_LOGS` 따로)

- `tools/test_phone_detect.py`: 큐브 몸통까지 그린 가짜 사진 2장 (2층 탑 + 낱개 2, 3층 탑 + 낱개 1) — 보이는 큐브 모두 2 mm·1.4° 안, 층 맞음. 예전 3장도 통과.
- 계획: 탑 (−0.14, −0.30), 큐브 (0.02, −0.27)·(0.10, −0.21) → 2층·3층 계획 playable (다른 큐브와 팔 21.6 mm, 든 큐브 30.6 mm). 4층째는 이유와 함께 거절.
  옵션 없는 계획 4개는 고치기 전과 프레임까지 같음.
- 화면: 헤드리스 Chrome 을 CDP 로 몰아 (가상 arm_homed 켜고 원점) 가짜 사진 올리기 → 탑 쌓기 → 확인창 → 가상 팔 재생 끝까지 → 다음 사진 … 큐브 10·11·12 를
  탑 자리 (−0.16, −0.28) 에 1·2·3층 (계획 4~6초, 재생 65·74·68초, JS 오류 0). 팔이 꺼져 있으면 원래 재생 검사가 거절하고 탭은 "재생이 시작되지 않았다" 로 알린다.
- 찾기: `detect_cubes` 는 `minMarkerDistanceRate` 0.02 — 기본값이면 큐브 윗면 흰 네모(50 mm)가 후보로 잡혀 그 안의 마커가 버려진다 (가짜 사진 24장: 기본값 절반 놓침 → 24/24).
  `detect()`(휴대폰 카메라 탭, 4x4 ID 0 하나)는 안 고쳤다 — 같은 문제가 있을 수 있다.
- `tools/try_direct_fake.py` 39/39.

**안 한 것 / 주의**: 물리 없음 — 쥔 큐브가 미끄러지는지, 2 mm 위에서 놓은 큐브가 반듯이 앉는지, 탑이 쓰러지는지 모른다. 실물·실제 폰 사진으로 돈 적 없다.
놓인 큐브 자리는 사진이 정하므로 사진 오차(가짜 사진 2 mm, 실제는 더 큼)가 그대로 다음 층에 간다. 팔이 사진에 들어오면 마커가 가려진다 — HOME 으로 돌아온 뒤 찍는다.
기존 `phone_markers.pdf` 2쪽의 닫힌 50 mm 네모 선은 오린 뒤 남으면 ArUco 가 그 네모를 후보로 잡아 마커를 못 찾을 수 있다 (인쇄 그림 그대로는 못 찾음, 선을 지우면 찾음)
→ 탑 쌓기 인쇄물은 모서리 표시만 그렸다. '집을 수 있는 곳' 지도처럼 탑 자리 지도도 기둥은 안 본다.

## 실행 기록 (`run_record.py`, `static/record.js`) — 실물 재생마다 전부 남김 (2026-11-24)

사용자: "실물로 할 때 모든 정보 다 로깅되게, 영상이나 그런 것들". **실물 재생(역기구학 · HOME · 탑 쌓기 · 강화학습)이 시작되면 자동으로** 기록을 시작하고,
재생이 끝나면(끝까지 · STOP · 고장) 2초 더 찍고 닫는다. 가상 모드 재생도 기록한다(끌 수 있음, 영상 없음). 화면: 위 칩 `기록 중 ● runs/…`,
연결 탭 맨 아래 접힌 **기록** 칸 = 최근 20개(종류 · 길이 · 끝난 이유 · 크기 · **zip** 받기), '재생마다 자동 기록' · '가상 모드도' 체크, 손으로 **기록 시작 / 끝**
(재생 없이 원점 잡기 · 손으로 움직이기 같은 자유 시간; 그 사이 재생은 같은 폴더에 `plan_2.json` … 로 붙음, 30분이면 자동 끝). 주소 `/?rec=1#conn` = 기록 칸 펼침.

폴더 `logs/runs/<YYYYmmdd_HHMMSS>_<kind>/` (kind = ik · home · stack · rl · manual; `JAX_DASH_LOGS` 를 따름):

| 파일 | 내용 · 빈도 |
|---|---|
| `video_live.mp4` + `video_live_ts.csv` | 폰 프로그램 `live.jpg` (live_api `LIVE.raw_jpeg`, 약 10 Hz) — mp4v 10 fps. 프레임이 늦게 오면 앞 프레임을 되풀이해 **영상 시간 = 실제 시간** (3초 넘는 끊김은 메우지 않음). ts.csv = 새 프레임마다 영상 프레임 번호 · 받은 시각 · 파일 시각 · 되풀이 수 |
| `video_infer.mp4` + `_ts.csv` | `inference.png` (뎁스 · 세그, 1~3 Hz) 가 바뀔 때, 4 fps 같은 방식 |
| `joints.csv` | **제어 루프 50 Hz 한 줄씩**: 재생 프레임 번호, AK45 마다 명령 · 목표 · 측정 (모터 °) · 속도 (ERPM) · 전류 · 온도 · 에러 코드 · 원점 오프셋 · 이 틱에 보낸 값 · 관절 ° (sim), 서보마다 보낸 각 (서보 °) · 관절 °. 가상 = 20 Hz, 모터 °를 JSON 으로 |
| `cubes.csv` | 인식 결과 새 프레임마다 큐브 한 줄 (id · x · y · yaw · level · tilt · source · age · stale), 안 보이면 `(없음)` 한 줄. 가상 = 장면이 바뀔 때 |
| `plan.json` | 재생한 궤적 그대로 + 계획 칸 값 (큐브 · 놓을 자리 · d · grasp · stack · 강화학습 정보) + force · 검사 문제 · 속도 |
| `events.log` | `direct.log` 줄 (시작 직전 몇 줄부터: 재생 시작/끝 · STOP · 힘 빼기 · 고장 · 원점 · 보정), 기록 시작/끝/이유, **화면에서 누른 POST 전부** (경로 + 몸통 300자 + 응답 코드: 보정 · 핀 · 보내기 …), CAN 버스 상태 바뀜 |
| `meta.json` · `config/` | 시작/끝 · 모드 · git 커밋 (+dirty) · 버전 · 팔 설정 · 원점 오프셋 · 책상 높이 · 줄 수 · 영상 장수 · 크기. `config/` = calibration.json · homing.json · phone_cam/phone_depth/phone_table.json 사본 |

- **제어 루프 영향**: `real_arm._ctrl_loop` 에 한 줄 (`rec = self.rec; if rec: rec(self, now)`, 잠금 안) — 숫자 약 40개를 리스트로 deque 에 넣을 뿐, 변환 · 파일 쓰기 · 영상은
  `run-record` 스레드 (20 Hz) 가 한다. 플랜 저장은 줄마다 dumps (큰 JSON 한 번이면 GIL 을 수십 ms 잡음), git 해시는 서버 시작 때 한 번.
- **보관**: 최근 50개 또는 5 GB 중 작은 쪽 (`JAX_DASH_RUNS_KEEP`, `JAX_DASH_RUNS_MAX_GB`), 기록이 끝날 때마다 오래된 것부터 지움. 화면에 전체 크기 · 디스크 남은 곳.
  크기: 실물 1분 ≈ 관절 1 MB + 영상 3.5 MB + 뎁스 영상 1 MB.
- API: `GET /api/record/status`, `POST /api/record/start {kind?}`, `POST /api/record/stop`, `POST /api/record/settings {auto?, virtual?}`, `GET /api/record/zip/<폴더>`.
- 훅: `direct_api.play_real` · `server._go_home` · `server._play_virtual` 에서 재생이 시작되면 `run_record.REC.on_play(traj, params, arm, …)` 한 줄. 끝은 기록 스레드가
  `arm.play["running"]` 을 보고 안다. ROS 모드 재생은 기록 안 함.

**확인** (2026-11-24, `--fake-hw` / `--virtual` 서버 8197, `JAX_DASH_LOGS` 따로)

- `tools/try_record_fake.py` (가짜 AK45 · STM32, 팔꿈치 왕복 8초씩 기록 끔/켬): 틱 50.0 Hz · 간격 평균 20.0 ms · p99 20.04 ms 둘 다 같음, 잠금 안 시간 평균 166 → 216 µs
  (p99 502 → 519 µs), joints.csv 50.0 줄/초. `tools/try_direct_fake.py` 39/39.
- fake-hw 서버: 역기구학 계획 (2490 프레임) 실물 재생 → `…_ik/` 57.3초, joints 2865줄 (50.0 Hz), 폰 영상 553장 (9.7 Hz, 실행 중인 폰 프로그램 live.jpg 읽기만),
  뎁스 영상 17장, cubes 553줄, events 에 재생 시작/끝 · POST. 재생 중 STOP → 끝난 이유 `stopped`, 2초 뒤 닫힘, zip 내려받기 (mp4 는 압축 안 함).
- 가상 서버: HOME 재생 → `…_home/` (virtual, 20 Hz, 가상 장면 큐브).
- 영상: 10초 손 기록 → `video_live.mp4` 100 프레임 (10 fps = 10초, 새 프레임 53장 — 그때 폰이 5 Hz) cv2.VideoCapture 로 끝까지 읽힘. 프레임당 디코드 + mp4v 인코드 3.3 ms
  (640×480) → 10 Hz 에서 한 코어의 약 3 %, inference.png 읽기 8 ms.
- 보관: 개수 5 · 크기 3 MB 로 낮춰 오래된 것부터 지워지는 것.

**안 한 것**: 실물 can0 · 실제 폰 큐브 장면으로 긴 시간 (수십 분) 은 안 돌렸다. 서버 안에서는 다른 스레드 (인식 등) 때문에 2865 틱 중 1번 31 ms 늦은 틱이 있었다
(기록 켬/끔을 서버 안에서 비교하지는 못함 — 따로 잰 try_record_fake 에서는 차이 없음). 영상은 mp4v 라 브라우저에서 바로 안 열릴 수 있다 (VLC · cv2 는 됨).
