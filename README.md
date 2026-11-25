# jax_ros2 (포크 `gyumingim/jax_ros2`, 브랜치 `noros`) — JAX 팔을 ROS 없이 실물 제어

이 브랜치는 **ROS 설치 없이** 웹 대시보드 하나로 실물 팔을 움직인다. 대시보드 서버(파이썬)가 `can0` 을 직접 열어(python-can, SocketCAN)
AK45 4개와 STM32 서보 노드에 CAN 프레임을 보낸다. 팀 레포의 ROS 노드 코드는 그대로 남아 있다(맨 아래 '팀 ROS 버전').

```
브라우저 ── dashboard/server.py ── direct_api.py ── real_arm.py ──(python-can, can0, 1 Mbps)──┬─▶ AK45 4개 (ID 1~4, 확장 ID, 위치 모드 4 / 피드백 0x29)
            (탭: 연결 · 각도 제어 · 역기구학)                                                  └─▶ STM32 (표준 ID 5) ─▶ 서보 10개
```

`real_arm.py` 는 `arm_control` 의 순수 파이썬 모듈(calibration·homing·motion·play·watch·protocol)과 `stm_bridge/protocol.py` 를
import 해서 쓴다(rclpy 없음). 역기구학 계획은 `arm_control/pick_place.py --dry-run` 을 이 PC 의 python 으로 돌린다.

## 우분투 노트북에서 실물 제어 — 빠른 시작 (2026-11-20)

### 1. 받기
```bash
git clone -b noros https://github.com/gyumingim/jax_ros2
git clone -b handroll-limit https://github.com/gyumingim/IK     # 역기구학: hand_roll 을 서보 범위 ±88° 로 묶은 판
```
대시보드는 IK 레포를 `jax_ros2` 옆(`../IK`)에서 찾는다(다른 곳이면 `JAX_IK_DIR`).

### 2. 설치 (Ubuntu 22.04 / 24.04, ROS 필요 없음)
```bash
sudo apt install -y can-utils python3-can python3-fastapi python3-uvicorn python3-websockets
pip install --user --break-system-packages mujoco gymnasium scipy   # 역기구학 계획(시뮬)용. 22.04 면 --break-system-packages 빼기
```

### 3. 실제 연결 순서 (이 순서대로)

> 2026-11-20 기준 **실물에서 한 번도 안 돌려 봤다**(가짜 CAN 시험만 통과). 첫 연결은 한 단계씩, 전부 정지(Esc)에 손 올리고.

| # | 할 일 | 확인 |
|---|---|---|
| 1 | **팔을 받친다** (기준 자세 = 팔을 수직으로 내린 자세 근처). 힘 빼기·연결 끊기·대시보드 종료·피드백 5 초 넘는 끊김이면 AK45 홀드가 풀려 팔이 떨어진다 | — |
| 2 | 로봇 전원 켜기 → USB-CAN(CANable)을 노트북에 꽂기 | 터미널 `ip link show can0` 에 can0 이 보임 |
| 3 | 대시보드 켜기: `cd jax_ros2 && python3 dashboard/server.py` | 터미널에 서버 시작 문구 |
| 4 | 브라우저 **http://localhost:8080** → 위쪽 스위치 **실물**(기본) | 위 칩 "직접 CAN" |
| 5 | **연결** 탭 ① **CAN 켜기** (= `sudo ip link set can0 up type can bitrate 1000000`, 암호창이 뜰 수 있음. 안 되면 터미널에서 그 명령) | 칩 "can0 UP" |
| 6 | **연결** 탭 ② **▶ 팔 연결** (can0 을 연다. AK45 에는 아직 아무것도 안 보낸다) | AK45 4개 카드에 위치·온도가 뜸. 안 뜨는 모터 = 전원·배선·ID 확인 |
| 7 | **방향·원점 보정** (`calibration.json` 은 전부 `verified: false` = 가정): 연결 탭 → 고급 → **관절 하나씩 보정** → 관절 고르기 → `+ 움직이기`(2°)로 도는 방향을 눈으로 → 시뮬 0° 자세에서 `지금 = sim 0` → 반대면 `방향 뒤집기` → `확인됨` → **저장**. AK45 4개 다 | 4개 `확인됨` |
| 8 | **손목 서보**: 보정 카드에서 `손목 굽힘 (wrist_yaw) — 서보 W_PITCH` 를 `+`. 손목이 **굽어야** 맞다(비틀리면 `calibration.json` 의 wrist_pitch / wrist_yaw 서보 `name`·`id` 를 되돌림). 서보 첫 명령은 위치를 몰라 **바로 그 각으로 튄다** | 굽힘 / 비틀기 맞음 |
| 9 | **원점**: 각도 제어 탭 → 팔을 기준 자세(시뮬 0°, `arm_control/homing.json`)에 두고 [**전부 원점 잡기**]. 잡는 순간 그 자리에서 힘을 준다. 연결할 때마다·전원 재투입·긴 끊김·힘 뺀 뒤마다 다시 | 관절마다 "원점 있음" |
| 10 | **첫 움직임**: 각도 제어 탭 슬라이더로 **한 관절 2~5°** → [보내기]. AK45 위치가 출력축 기준인지 로터 기준인지 모른다 — 2° 보내 출력축이 실제로 몇 ° 도는지 본다 | 보낸 각 = 실제 각 |
| 11 | 관절마다 조금씩 넓혀 범위 확인 (어깨 앞뒤 ±45°, 어깨 옆 0~45°, 위팔 비틀기 −90~240°, 팔꿈치 ±135°, 2026-11-20 넓힘 — 끝에서 아래팔·위팔이 부딪히는지 먼저 확인) | 부딪힘 없음 |
| 12 | **역기구학**(보정 다 확인한 뒤만): 역기구학 탭 → 지도의 **파란 칸**(집을 수 있음)에 큐브 → 놓을 자리 → [계획 돌려 보기](모터 안 움직임) → **가상 팔로 재생**으로 3D 확인 → **실물로 재생**(확인창 2번) | 3D 와 같은 동작 |
| 끝 | 팔 받치고 → 연결 탭 **힘 빼기** 또는 **연결 끊기** → 서버 Ctrl+C | — |

실물 없이 같은 화면을 시험: `--virtual` (가상 팔) / `--fake-hw` (가짜 모터에 직접 CAN).

| 탭 | 하는 일 |
|---|---|
| 연결 | ① CAN 켜기 ② 팔 연결, AK45 4개 상태(위치·온도·홀드), 힘 빼기·연결 끊기, 고급: 버스 상태·로그·**관절 하나씩 보정** |
| 각도 제어 | 원점 잡기, 관절 슬라이더 → [보내기] (30 °/s 램프, 원점 없는 AK45 는 막힘), 오른쪽 3D |
| 역기구학 | 지도(파랑 = 집을 수 있음, 해석 IK: 손목 중앙이 책상 위 27 cm 로 갈 수 있나), 3D 초록 영역, 비전 큐브 배지, 손 위치 보내기, 계획·3D 재생·**실물로 재생** |

### 4. 멈추기 — 두 가지
| | 하는 일 | 팔 |
|---|---|---|
| **■ STOP** (위 빨간 버튼, Esc) | 모든 AK45 를 지금 측정 자리에 다시 홀드, 서보 램프 멈춤, 궤적 재생 멈춤 | **안 떨어짐** |
| **힘 빼기** (연결 탭 ②, 확인창) | AK45 위치 프레임 중단 + 브레이크(모드 2, 0 A) | **떨어짐** |

대시보드를 끄면(Ctrl+C) 브레이크 0 A 를 보내고 닫는다(프로세스가 끝나면 홀드 프레임도 끝나므로 어차피 힘이 빠진다). 서보는 펌웨어에 '끄기'가 없어 마지막 각을 유지한다.

### 안전장치 (real_arm.py, 기본값)
| 장치 | 기본 |
|---|---|
| 첫 명령 전 무송신, 피드백 없는 모터엔 송신 안 함 | — |
| 원점 없는 AK45 관절 명령 거절 (`arm_control.homing.Homing` 그대로, 세션 원점) | — |
| 램프: 속도 · 가속 · 한 스텝 최대 | 30 °/s · 90 °/s² · 3° |
| carrot: 명령을 측정 위치 ± 안으로 | 30° |
| 홀드 재송신 (움직이는 동안 50 Hz) | 20 Hz |
| 피드백 짧은 끊김: 마지막 명령 홀드 유지, 돌아오면 2° 넘게 벌어졌을 때 지금 자리에서 다시 램프 | ≤ 5 s |
| 긴 끊김: 그 모터 송신 멈춤 + 원점 지움, 돌아오면 지금 자리 홀드 | > 5 s |
| 위치 튐 → 지금 자리 홀드 + 원점 지움 | 60° |
| 막힘(명령 − 실제) → 그 관절 힘 뺌 + 원점 지움 | 12° 가 2 s |
| 온도 경고 / 힘 뺌 (이 모터 무부하 53 ℃ 기록) | 60 / 75 ℃ |
| AK45 에러 코드 ≠ 0 → 그 관절 힘 뺌 | — |
| \|모터 명령\| 상한 | 360° |
| 같은 can0 을 쓰는 ak45_node 가 떠 있으면 연결 거부 (`/tmp/ak45_ctrl.lock` flock, ak45_can 과 같은 파일), 대시보드 밖 ROS 노드 감지 | — |
| 다른 프로그램이 같은 모터 ID 에 보내는 명령 프레임 감지 → 경고 띠 | — |
| CAN 에러 프레임(bus-off·passive·warning, bit-stuff·no-ack 수) 표시 | — |
| 궤적 재생: `play.check_trajectory` 통과(검사 문제는 확인창 = `--force`), AK45 가 명령에서 15° 넘게 0.5 s → 멈춤·그 자리 홀드 | 15° |
| 서보: 30 °/s 램프(첫 명령은 바로), 마지막 값 재전송 | 1 Hz |

AK45 피드백(0x29)에는 전압이 없어 전압 경고는 없다(robot_arm 의 UART 판과 다름).

### 알아 둘 것
- **손목 이름**: CAD/URDF 의 손목 이름이 실제 움직임과 바뀌어 있다(시뮬 0° 에서 `wrist_pitch` 는 아래팔 축 비틀기, `wrist_yaw` 는 굽힘, `hand_roll` 도 아래팔 축 = yaw). 그래서 `calibration.json` 에서 wrist_pitch → W_YAW(8), wrist_yaw → W_PITCH(9) 로 바꿔 연결했다. 대시보드 표시는 "손목 비틀기 / 손목 굽힘 / 손 yaw".
- **hand_roll ±88°**: 손 yaw 서보(W_X, MG996R)가 0~180° 라 시뮬 ±90° 만 된다. IK `handroll-limit` 은 이 범위 안에서만 계획한다(집을 수 있는 칸 60 → 36).
- **반 바퀴 잡기**: 집기와 놓기는 같은 손 모양이어야 한다. 지도에서 둘 다 파란 칸이어도 손 모양이 다르면 `pick_place` 가 거절한다.
- **손가락 회전 F_ROT(6번)**: 3번 손가락이 1·2번과 마주 보는 서보 각도(`zero_deg`)가 아직 정해지지 않았다.
- **서보 펌웨어(team-jax/CAN_STM)**: StdId 5 만 받음, 피드백 없음(화면의 서보 값 = 보낸 값), 명령이 끊겨도 마지막 펄스 유지, 0~1800 클립, 첫 명령 전엔 펄스 없음. 500~2500 µs 가 실제 0~180° 인지는 서보마다 다를 수 있음(`scale` 1.0 은 가정).
- **AK45 명령이 끊기면 모터가 힘을 놓는다**는 것은 ak45_node 주석의 근거(가정)다. 실물에서 확인 안 함.

## 시험 (실물 없이)

| 명령 | 확인하는 것 |
|---|---|
| `python3 dashboard/tools/try_direct_fake.py` | real_arm 을 가짜 AK45 4개 + 가짜 STM32(`dashboard/tools/fake_can_arm.py`, python-can virtual 버스)에 붙여 안전 규칙 20개(첫 명령 전 무송신 … 힘 빼기). Windows 도 됨. Linux vcan: `--interface socketcan --channel vcan0` |
| `python3 dashboard/server.py --fake-hw` | 화면 전체를 가짜 모터에 직접 CAN 으로 |
| `python3 dashboard/server.py --virtual` | 가상 팔(규칙을 흉내 낸 시뮬) |

## 팀 ROS 버전

팀 레포(team-jax/jax_ros2)의 구조 — `ak45_node`(C++, ak45_can) · `stm_bridge_node` · `arm_control_node` · `homing_guard` · `vision_node`, `arm.launch.py` / `arm_homed.launch.py`, 토픽 표, colcon 빌드, `calibrate --ros`, vcan 시험 — 은 **`ROS_TEAM.md`** 로 옮겼다(코드는 그대로).
같은 대시보드를 ROS 노드로 쓰려면 `python3 dashboard/server.py --ros` (ROS 2 설치·`colcon build` 필요). 문서: `STATUS.md`, `NODES.md`.

## ROS 시험 돌리는 법 (2026-11-24, 이 PC = Ubuntu 24.04 + ROS Jazzy 에서 확인)

레포를 colcon 작업공간에 빌드·설치해야 `ros2 run vision_node …` 가 된다 (안 하면 vision_node 시험이 'published 0 messages' 로 전부 실패하는데 노드 문제가 아니다). 레포를 더럽히지 않게 임시 작업공간에 복사해서 빌드한다:

```bash
mkdir -p /tmp/ws/src/repo && for p in arm_control stm_bridge vision_node ak45_ros2 ak45_can; do rsync -a --exclude=__pycache__ $p /tmp/ws/src/repo/; done
cd /tmp/ws && source /opt/ros/jazzy/setup.bash && colcon build            # 가상 CAN 스크립트가 <작업공간>/src/<레포>/arm_control/tests 구조를 기대한다
source /tmp/ws/install/setup.bash && cd arm_control && python3 tests/check_node.py        # python3 -I 로 돌리면 PYTHONPATH 를 무시해 rclpy 를 못 찾는다
```

- 노드 시험: `arm_control/tests/check_node.py`·`check_calibrate_wait.py`, `stm_bridge/tests/check_node.py`, `vision_node/tests/check_node.py`·`check_camera_mode.py`.
- 가상 CAN 시험(`check_topics.sh`·`launch_vcan.sh`·`chain_vcan.sh`·`calibrate_vcan.sh`): `sudo` 없이 `unshare -Urn`(개인 네트워크 네임스페이스)에서 자기 `can0`/`vcan0` 를 만든다 — 실물 can0 와 분리. 단 `ak45_node` 는 `/tmp/ak45_ctrl.lock` 이 잡혀 있으면(대시보드가 실물 팔에 연결 중) 시작을 거부한다 → 시험은 `unshare -Urnm` 으로 그 파일을 더미로 덮어씌워 돌린다 (실물 잠금은 그대로).
- 기대값은 `calibration.json` 에서 읽는다 (서보 이름·핀·영점·범위를 사용자가 실물에서 고친다).
- `arm_control/tests/check_split.py --original <team-jax/SimToReal 경로>` 는 외부 저장소가 필요해서 이 점검에서는 못 돌렸다.
