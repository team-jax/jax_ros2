# jax_ros — JAX 팔 통합 ROS 2 레포 (Jazzy)

흩어져 있던 ROS 2 코드를 한 레포에 모았다. 시뮬 관절각을 실물 모터 명령으로 바꾸던 `sim2real_node`(team-jax/SimToReal)는 노드 4개로 쪼갰고, **동작은 바뀌지 않았다**(같은 입력에서 `/ak45/command`와 서보 프레임이 같은 값으로 나가는 것을 숫자로 비교했다. `STATUS.md`).

```
vision_node ─/object_pose─▶ arm_control ─┬─/ak45/command──▶ ak45_node ─CAN(확장 ID)─▶ AK45 4개 (ID 1~4)
                                          └─/servo/command─▶ stm_bridge ─CAN(StdId 5)─▶ STM32 ─▶ 서보 10개
```

| 폴더 | 내용 | 원본 |
|---|---|---|
| `ak45_can/` | AK45 CAN 라이브러리(ROS 패키지 아님. `ak45_ros2`가 `../ak45_can`을 찾는다) | team-jax/ak45_can `6db174f` |
| `ak45_ros2/` | `ak45_node`: AK45 **4개**(ID 0x01~0x04) | team-jax/ak45_ros2 `5495298` (6개→4개로 변경) |
| `stm_bridge/` | `/servo/command` → CAN_STM 서보 프레임, `/servo/state` | 새로 만듦 (SimToReal의 서보 CAN 부분) |
| `arm_control/` | 보정, 속도 제한, 감시, 계획, 재생, 보정 CLI. `arm.launch.py` | SimToReal의 나머지 |
| `vision_node/` | `fixed_pose`(시험용) 또는 `result_json`(카메라 프로그램의 `inference.json`)일 때만 `/object_pose`를 발행 | 새로 만듦 |

각 복사본 폴더의 `SOURCE.md`에 원본 URL과 커밋 SHA가 있다. 원본 레포(`SimToReal`, `ak45_ros2`, `ak45_can`)는 건드리지 않았다.

## 토픽

| 토픽 | 형식 | 방향 | 규칙 |
|---|---|---|---|
| `/arm_control/joint_command` | JointState, **시뮬 관절 이름**(`elbow`, `wrist_pitch` …), rad | pick_place·play_trajectory·사람 → arm_control | 기존 `/sim2real/joint_command`의 새 이름 |
| `/ak45/command` | JointState, 이름 `ak45_1..4`, position = **모터 각도(rad)** | arm_control → ak45_node | 기존 규칙 그대로 |
| `/joint_states` | JointState (AK45 실측, rad) | ak45_node → 모두 | 기존 그대로 |
| `/servo/command` | JointState, 이름 = 서보 이름(`W_PITCH`, `F1_1` … `calibration.json`의 `name`), position = **서보 모터 각도(rad, 0~π)** | arm_control → stm_bridge | `/ak45/command`와 같은 규칙. 시뮬 각도 → 모터 각도 변환은 **arm_control만** 한다 |
| `/servo/state` | 위와 같은 형식, position = **마지막으로 보낸 값**(측정값 아님) | stm_bridge → 모두 | 서보는 피드백이 없다. `/joint_states`에 섞지 않는다 |
| `/object_pose` | PoseStamped, `frame_id = shoulder_roll_axis`(카메라 모드에서 `camera_to_robot`이 비면 `camera`), 위치 m | vision_node → arm_control | 아래 |

`/object_pose`: 원점 = 오른쪽 어깨 롤 모터 축 중심, x = 로봇 왼쪽(+), y = 뒤(+, 앞은 −), z = 위. **회전은 z축 yaw만**(쿼터니언의 x·y는 0). 단위: `vision_node`의 `fixed_pose:=[x, y, yaw]`는 x·y가 m, **yaw가 도(°)**(반시계, `pick_place --cube x y yaw`와 같다). **position.z는 사용하지 않는다**(`fixed_pose`에 높이가 없어 0.0). 카메라 모드(`result_json`)에서는 변환한 z가 실린다(`arm_control`은 쓰지 않는다). `arm_control_node`는 지금 이 토픽을 로그로만 찍고 로봇을 움직이지 않는다(카메라 좌표 변환이 아직 없다).

서보 프레임: 표준 ID 5, `[motor_id 0..9][각도×10 H][L]`, 0~1800 (team-jax/CAN_STM `src/main.c`의 `CAN_SERVO_STDID 5U`). stm_bridge는 `/servo/command`에서 0~180°를 벗어난 값이 하나라도 있거나 이름·길이·NaN이 틀린 메시지를 **통째로 버린다**.

## 설치 (Ubuntu 24.04, ROS 2 Jazzy)

```bash
sudo apt install -y can-utils python3-can python3-colcon-common-extensions
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && git clone <이 레포> jax_ros      # 또는 폴더를 그대로 둔다
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
sudo ip link set can0 up type can bitrate 1000000       # 재부팅마다 1회
candump can0                                            # 00002901~00002904 확장 프레임(AK45 피드백)이 보여야 한다
```

`--symlink-install`이면 `calibrate`가 저장하는 `arm_control/calibration.json`이 저장소 파일 그대로다. `pick_place`까지 쓰려면 IK(`sim-table-friction` 브랜치)와 MuJoCo가 필요하다. 설치는 `arm_control/arm_control/planner.py` 맨 위 설명을 본다.

`ak45_node`의 CAN 인터페이스 이름은 컴파일 타임 상수 `can0`이다. `servo_channel`만 바꿀 수 있다(`ak45_ros2/README.md` §2).

## 실행

```bash
ros2 launch arm_control arm.launch.py            # ak45_node + stm_bridge_node + arm_control_node
#   servo_channel:=can0  max_speed_deg_s:=30.0  calibration_file:=<경로>
ros2 topic pub -t 3 -r 10 /arm_control/joint_command sensor_msgs/msg/JointState \
  "{name: ['elbow', 'wrist_pitch'], position: [0.0, 0.349]}"
ros2 run arm_control pick_place --cube 0.10 -0.30 20 --target 0.05 -0.27 --dry-run    # 계획과 검사만
ros2 run vision_node vision_node --ros-args -p fixed_pose:="[0.10, -0.30, 20.0]"      # 시험용 /object_pose
ros2 run vision_node vision_node --ros-args -p result_json:=<runs 폴더>/cube_test/inference.json   # 카메라 결과(camera 프레임)
```

### vision_node 카메라 모드 (`result_json`)
카메라 프로그램(SteroDepthSeg, 자체 venv)은 `--out` 폴더의 `inference.json`을 약 1초마다 덮어쓴다. `vision_node`는 그 파일만 읽는다(카메라 코드·torch를 import하지 않는다).
- 파라미터: `result_json`(SteroDepthSeg `--out` 폴더의 `inference.json` 경로, 기본 빈 문자열), `poll_hz`(2), `max_age_s`(3), `cube_dim_mm`([40, 80]), `depth_range_m`([0.3, 1.0]), `camera_to_robot`([tx, ty, tz, roll_deg, pitch_deg, yaw_deg], 기본 빈 목록). `fixed_pose`와 같이 주면 오류, 발행 없음.
- 후보: `label == "blob"`, `status == "ok"`, `dims_mm`의 가로·세로가 `cube_dim_mm` 안, `depth_rect_m`가 `depth_range_m` 안. **정확히 1개일 때만** 발행(0개·2개 이상이면 이유만 로그). 파일 수정 시각이 `max_age_s`보다 오래됐거나 JSON이 깨졌으면 발행하지 않는다(로그). 같은 파일 내용은 다시 발행하지 않는다.
- 좌표: `camera_to_robot`이 비면 `frame_id = camera`(카메라 좌표 그대로, OpenCV: x 오른쪽·y 아래·z 앞, **로봇 좌표 아님**), 있으면 `p_robot = R·p_cam + t`(R = Rz(yaw)·Ry(pitch)·Rx(roll))로 바꿔 `frame_id = shoulder_roll_axis`. 둘 다 **임시**(폰 고정 방법 미정). **yaw는 카메라 기준**(`rotation.yaw_deg` 그대로, 로봇 좌표 yaw로 바꾸지 않음). 이 경우 시작할 때 "임시: 카메라 좌표, 로봇 좌표 아님"을 로그로 남긴다.

### 보정 (calibrate)

```bash
ros2 run ak45_ros2 ak45_node &                       # 또는 ros2 launch ak45_ros2 ak45_node.launch.py
ros2 run stm_bridge stm_bridge_node
ros2 run arm_control calibrate --ros                 # ROS가 없는 곳: python3 arm_control/calibrate.py (dry-run)
```

> ⚠️ **`calibrate --ros`를 쓸 때는 `arm_control_node`를 띄우지 않는다**(그래서 `arm.launch.py`도 쓰지 않는다). 둘 다 같은 모터에 명령을 보내게 된다. `ak45_node`와 `stm_bridge_node`만 띄운다.
> 서보 명령은 `/servo/command`를 받는 `stm_bridge`가 보여야 나간다. 5초 안에 안 보이면 "no subscriber on /servo/command … nothing sent"로 거절한다. 명령은 `arm_control/arm_control/calibrate.py` 맨 위에 있다. `z`/`f`/`v`/`w`로 영점·방향·확인·저장.

### 주의

- **종료하면 AK45가 힘을 놓는다.** `ak45_node`가 끝날 때 `ak45_can`이 Current Brake 0A 프레임을 ID 1~**6**에 보낸다(`NUM_MOTORS=6`이라 5·6은 받는 모터가 없다). 먼저 팔을 받친다. AK45는 확장 29비트 ID, STM 서보 보드는 표준 11비트 프레임만 받으므로(`main.c`의 IDE 검사) 지금은 충돌하지 않지만, **나중에 ID 5·6 AK45를 추가하면 이 브레이크 프레임이 그 모터에 간다.**
- **`ak45_node`는 CAN 프레임이 더 안 들어오는 상태에서는 SIGINT/SIGTERM으로 끝나지 않을 수 있다**(`ak45_close()`가 소켓을 닫아 블로킹 읽기를 깨우려 하는데 깨어나지 않는다. 프레임이 오면 1초에 끝난다). 모터 전원이 꺼졌을 때는 `kill -KILL`이 필요할 수 있다. **실물 시험 때는 두 번째 터미널을 열어 두고 `pkill -KILL -f ak45_node`를 바로 칠 수 있게 한다.**
- `calibration.json`의 값은 전부 `verified: false`(영점·방향·관절↔모터 대응이 가정)다. `pick_place`/`play_trajectory`는 `verified`가 아닌 관절이 있으면 거절한다(`--force`로 넘길 수 있다).
- AK45 위치가 출력축 기준인지 로터 기준인지(V3)는 문서마다 다르다. **첫 실제 명령은 2° 정도로 보내 출력축이 얼마나 도는지 눈으로 확인한 뒤** 진행한다.

## 시험 (실물 없이)

CAN은 `vcan`으로 시험한다(`sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`). **실제 모터로는 시험하지 않았다.**

| 명령 | 확인하는 것 |
|---|---|
| `python3 -I stm_bridge/tests/check_protocol.py` | 서보 프레임 바이트, 이름→id 표 = `calibration.json` |
| `python3 stm_bridge/tests/check_node.py` | stm_bridge_node (python-can virtual 버스) |
| `python3 -I arm_control/tests/check_protocol.py --ik ~/IK` | 원본 SimToReal과 같은 58개 검사 |
| `python3 arm_control/tests/check_node.py` | arm_control_node → stm_bridge_node 전체 체인(원본 기대값 그대로) |
| `python3 arm_control/tests/check_split.py --original <SimToReal>` | 쪼개기 전/후 같은 입력, 같은 출력(틱 단위), 서보 0~180° 왕복 |
| `python3 arm_control/tests/check_calibrate_wait.py`, `bash arm_control/tests/calibrate_vcan.sh` | calibrate가 stm_bridge를 기다림 |
| `bash arm_control/tests/chain_vcan.sh` | vcan0에서 50 Hz, 발행한 서보 수 = candump 프레임 수 |
| `bash arm_control/tests/launch_vcan.sh` | `arm.launch.py` 전체(별도 네트워크 공간에 `can0`를 만들어 sudo가 필요 없다) |
| `bash arm_control/tests/check_topics.sh` | NODES.md §4 토픽 표(타입, 발행·구독 노드, 메시지 내용)를 실행 중인 시스템과 대조 |
| `python3 vision_node/tests/check_node.py` | `fixed_pose`가 없거나 틀리면 발행 없음, 있으면 1 Hz |
| `python3 vision_node/tests/check_camera_result.py` | ROS 없이 `inference.json` 파싱·큐브 선택·오래된 파일·임시 변환(손계산 값) |
| `python3 vision_node/tests/check_camera_mode.py` | `result_json` 모드 실제 `ros2 run`(파일 변경·깨진 JSON·후보 2개·오래된 파일·잘못된 파라미터) |

시험 스크립트를 쓸 때 `pkill -f`·`pgrep -f` 패턴이 같은 셸 명령 본문에 들어 있으면 자기 자신이 매칭된다. 스크립트 파일로 실행한다.

## 문서

- `STATUS.md` — 했던 일, 하고 있는 일, 할 일, 목표, 기술, 문제점과 해결방안, 단계별 검증 결과.
- `NODES.md` — 노드 구성과 토픽 표(2026-11-07에 이 구현에 맞춰 고침). 표는 `arm_control/tests/check_topics.sh`로 실행 중인 시스템과 대조한다.
- `ARCHITECTURE.md` — **아직 이 폴더에 없다.**
