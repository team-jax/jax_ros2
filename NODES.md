# ROS2 노드 구성 (최소안) — Team JAX 단일 팔 Pick & Place

> 기준: 2026-10-22. 2026-11-07에 `jax_ros` 구현(1~5단계)에 맞춰 고쳤다. 원칙은 "간단하게". 노드는 담당자가 다르거나, 다른 기기에서 돌거나, 주기가 다를 때만 나눈다. 그 외는 한 노드 안의 함수로 둔다. [확정] 합치기로 정함 / [제안] 아직 승인 전 / [미확정] 정해지지 않음.

## 1. 구성 한눈에

```
vision_node ──/object_pose──▶ arm_control ─┬─/ak45/command──▶ ak45_node ──CAN──▶ AK45 4개 (ID 0x01~0x04)
 (재준)                      (재준)       └─/servo/command─▶ stm_bridge ─CAN──▶ STM32 ─PWM─▶ 서보 10개
                                                              (재준)
```

실행 위치: 빌려 온 노트북 한 대에 모든 노드와 USB-CAN(CANable)을 둔다. [제안] 이 PC의 실제 환경은 **Ubuntu 24.04 + ROS 2 Jazzy**(`/opt/ros/jazzy`로 확인, 2026-11-07)이고 jax_ros는 이 환경에서 빌드·시험했다. 학교에서 빌린 노트북의 OS 설치 방법과 학교 허락은 [미확정].

## 2. 노드 (직접 만드는 것 3개 + 기존 1개)

| 노드 | 하는 일 | 구독 | 발행 | 담당 | 상태 |
|---|---|---|---|---|---|
| `stm_bridge` | `/servo/command`를 CAN 프레임으로 STM32에 보낸다. 마지막으로 보낸 각도를 `/servo/state`로 발행한다(측정값이 아님). 나중에 STM의 센서 프레임을 받아 토픽으로 올린다. | `/servo/command` | `/servo/state`, (10/29 이후) `/pressure` | 재준 | 구현, vcan 시험 완료(2026-11-07). 실물 전. `/pressure`는 아직 없음 |
| `ak45_node` | AK45 4개(ID 0x01~0x04) 제어와 상태 발행 | `/ak45/command` | `/joint_states`, `/diagnostics` | 재준 | 6개→4개로 변경, vcan 시뮬레이터로 시험. 실물 검증은 한 대(ID 0x02)만(ID 1·3·4는 ID 설정만 했고 통신 미확인) |
| `arm_control` | 순서 관리(집기→놓기)와 IK 계산을 한 노드에서 한다. IK 계산은 노드와 분리된 함수로 짜서 단독 시험이 가능하게 한다. | `/object_pose`, `/arm_control/joint_command`, `/joint_states` | `/ak45/command`, `/servo/command` | 재준 | 일부 구현: 시뮬 관절각 → 모터 각도 변환, 속도 제한, AK45 추종 감시, `pick_place`·`play_trajectory`·`calibrate`(SimToReal에서 이전, 같은 출력임을 숫자로 확인). `/object_pose`는 로그만 찍고 움직이지 않음. 상태머신과 IK의 노드화는 미구현 |
| `vision_node` | 카메라 입력, 물체 탐지, 깊이 계산, 로봇 좌표계 변환까지. 카메라 영상은 `/camera/*/image_raw`로도 발행한다. [제안] | (카메라) | `/object_pose` | 재준 | 일부 구현: `fixed_pose:=[x,y,yaw]`일 때 1 Hz 발행(시험용) 또는 `result_json`(카메라 프로그램의 `inference.json`)을 읽어 큐브 1개를 골라(구역·점수·마진·이전 선택) 발행. 파라미터: `result_json`(SteroDepthSeg `--out` 폴더의 `inference.json` 경로, 기본 빈 문자열), `poll_hz`(2), `max_age_s`(3), `cube_labels`(["blob", "kite"], `["*"]`이면 모든 라벨), `cube_dim_mm`([40, 80]), `depth_range_m`([0.3, 1.0]), `camera_to_robot`([tx, ty, tz, roll_deg, pitch_deg, yaw_deg], 기본 빈 목록), `work_x_m`·`work_y_m`·`work_z_m`(각 [최소, 최대], **로봇 좌표**, 기본 꺼짐), `score_weights`([1.0, 1.0] = [정사각 가중치, 중앙 가중치]), `min_margin`(0.05), `track_tol_m`(0.03 m). 후보 선택: 후보 조건(label·status·크기·깊이·유효한 중심/yaw)을 통과한 객체 중 (1) `camera_to_robot`이 있을 때만 작업 구역 `work_*_m` 밖의 객체를 버린다(이유를 로그에 남김; `camera_to_robot`이 비어 있으면 구역은 적용하지 않고 시작할 때 한 번 경고). (2) 직전에 발행한 위치에서 `track_tol_m` 이내인 후보가 있으면 그것을 계속 쓴다(마진 검사 없음, 직전 위치는 3초 넘게 갱신 안 되면 폐기). (3) 아니면 점수 순위: `점수 = (w0·(1 − |w−h|/max(w,h)) + w1·(1 − mean(|w−c|, |h−c|)/half)) / (w0+w1)` (w, h = `dims_mm[0]`, `dims_mm[1]`, c = `cube_dim_mm` 중앙, half = 그 폭의 절반). 후보가 1개면 그대로 발행, 2개 이상이면 1등과 2등의 점수 차가 `min_margin` 이상일 때만 1등을 발행하고 아니면 `ambiguous: 1등/2등 점수와 위치`를 로그에 남기고 발행하지 않는다. 손계산: `cube_dim_mm` [40, 80]에서 50×40 → (0.8 + 0.25)/2 = 0.525. 0개일 때는 이전처럼 이유만 로그. `ros2 launch vision_node vision.launch.py result_json:=<inference.json 경로>`는 `vision_node/config/camera.yaml`의 `camera_to_robot` `[0.174, -0.146, 0.016, 135.129, 0.988, 0.992]`을 노드에 넘긴다. **미검증 임시값**: 위치는 CAD 어깨 롤축 점 (-202, 0, 850)과 거치대 꼭짓점 (-28, -145.998, 866.304)의 차이 (+174, -146, +16.3) mm(CAD X = 로봇 왼쪽, Y = 뒤, Z = 위), 회전은 IMU(pitch -45.1, 폰 가로 장착, 뒤집힘). 렌즈-꼭짓점 오프셋은 반영하지 않았고 실물 보정 전까지 임시. 이전 값들은 폐기. 다른 파일은 `config:=...`. launch를 쓰지 않는 `ros2 run vision_node vision_node`는 그대로 `camera_to_robot` 빈 값(camera 프레임)으로 시작한다. `camera_to_robot`이 비면 `frame_id = camera`(카메라 좌표 그대로, OpenCV: x 오른쪽·y 아래·z 앞, **로봇 좌표 아님**), 있으면 `p_robot = R·p_cam + t`(R = Rz(yaw)·Ry(pitch)·Rx(roll))로 바꿔 `frame_id = shoulder_roll_axis`. 둘 다 **임시**(폰 고정 방법 미정). **yaw는 카메라 기준**(`rotation.yaw_deg` 그대로, 로봇 좌표 yaw로 바꾸지 않음). 실제 카메라로는 아직 시험하지 않음 |

## 3. 나중에 / 합친 것

| 항목 | 처리 |
|---|---|
| `grasp_node` (RL 파지) | 처음에는 규칙 기반 파지를 `arm_control` 안의 함수로 둔다. RL이 준비되면 그때 노드로 뺀다. |
| `mimic_node` (사람 팔 따라하기) | 동작 1(집어서 옮기기)이 끝난 뒤에 만든다. |
| `robot_state_publisher` | 직접 만들지 않고 ROS 표준 패키지를 실행만 한다. URDF 7DOF/8DOF 불일치가 정리된 뒤에 RViz용으로 쓴다. |

## 4. 토픽

| 토픽 | 메시지 | 보내는 쪽 → 받는 쪽 | 비고 |
|---|---|---|---|
| `/object_pose` | PoseStamped, `frame_id = shoulder_roll_axis`(카메라 모드에서 `camera_to_robot`이 비면 `camera`) | `vision_node` → `arm_control` | 위치 m. 원점 = 오른쪽 어깨 롤 모터 축 중심, x = 로봇 왼쪽(+), y = 뒤(+, 앞은 −), z = 위. **회전은 z축 yaw만**(쿼터니언 x·y는 0). **yaw 단위는 도(°)**(`vision_node`의 `fixed_pose:=[x, y, yaw]`, 반시계, `pick_place --cube x y yaw`와 같다. PoseStamped의 쿼터니언 자체에는 단위가 없다). **position.z는 사용하지 않는다**(`fixed_pose`에 높이가 없어 0.0). 카메라 모드에서는 변환한 z가 실린다. `frame_id = camera`이면 좌표는 카메라 좌표(로봇 좌표 아님), yaw는 항상 카메라 기준(로봇 좌표로 변환 전). `arm_control`은 지금 로그만 찍는다(카메라 좌표 변환 없음) |
| `/arm_control/joint_command` | JointState, **시뮬 관절 이름**(`elbow`, `wrist_pitch` …), position = 시뮬 관절각(rad) | `pick_place`·`play_trajectory`·사람 → `arm_control` | 기존 `/sim2real/joint_command`의 새 이름. 이름·길이·NaN이 틀린 메시지는 통째로 버림 |
| `/ak45/command` | JointState, 이름 `ak45_1..4`, position = **모터 각도(rad)** | `arm_control` → `ak45_node` | 기존 규칙. 시뮬 각도 → 모터 각도 변환은 `arm_control`(`calibration.json`)이 한다 |
| `/servo/command` | JointState, 이름 = 서보 이름(`W_PITCH`, `F1_1` …, `calibration.json`의 `name`), position = **서보 모터 각도(rad, 0~π)** [확정, 구현·시험 2026-11-07] | `arm_control` → `stm_bridge` | 아래 이름 표 사용. `/ak45/command`와 같은 규칙("모터 각도를 rad로"). **시뮬 각도 → 모터 각도 변환(오프셋 포함)은 `arm_control`이 하고, `stm_bridge`는 프레임 송신만 한다**(기존 "단위와 오프셋은 stm_bridge에서 변환"을 바꿈). `stm_bridge`는 0~180°를 벗어난 값이 하나라도 있거나 이름·길이·NaN이 틀린 메시지를 통째로 버린다 |
| `/joint_states` | JointState | `ak45_node` → 모두 | AK45만 들어간다. 서보는 피드백이 없다. |
| `/servo/state` | JointState (관절 이름, position = **마지막으로 보낸 값**, rad) [확정, 구현·시험 2026-11-07] | `stm_bridge` → 모두 | 서보는 각도 피드백이 없어서 **측정값이 아니라 보낸 값**이다(0.1° 단위 정수로 반올림해 보낸 값을 되돌린 것). 명령을 받을 때마다 한 번 발행하고, 첫 명령 전에는 발행하지 않는다. `/joint_states`(AK45 실측)에 섞지 않고 따로 둔다 |
| `/pressure` | 미정 | `stm_bridge` → `arm_control` | 10/29 이후. 아직 없음 |

### 서보 관절 이름 → motor_id (STM32_PINMAP.md B안 기준) [확정]

| 이름 | motor_id | 이름 | motor_id |
|---|---|---|---|
| F1_1 | 0 | F_ROT | 6 |
| F1_2 | 1 | W_X (MG996R) | 7 |
| F2_1 | 2 | W_YAW | 8 |
| F2_2 | 3 | W_PITCH | 9 |
| F3_1 | 4 | | |
| F3_2 | 5 | | |

`stm_bridge`가 이름을 motor_id로, 각도(rad → 도)를 `×10` 정수로 바꿔 CAN StdId 5로 보낸다(`stm_bridge/servo_names.py`, 이 표와 `arm_control/calibration.json`의 서보 `name`·`id`가 같음을 시험이 확인한다). 프레임 형식과 변환식은 `STM32_SERVO_SPEC.md`를 따른다. **마지막 목표를 10Hz로 재송신하는 것은 구현하지 않았다**: 서보 프레임은 명령 1개당 1번만 나가고, 프레임이 사라지면 다음 명령까지 그대로다. `arm_control`도 목표에 도달하면 그 서보에는 더 보내지 않는다. (AK45의 10 Hz 재송신은 `ak45_node`가 한다.) 서보 재송신 여부는 [제안, 미구현].

## 4-1. `arm_control` 내부 구조 [제안]

```
입력: /object_pose   (나중에) /joint_states, /servo/state, /pressure
  ① 상태머신: 대기 → 물체 위로 이동 → 접근 → 잡기 → 들기 → 이동 → 놓기 → 복귀
  ② IK 함수: 목표 위치 → 관절 각도 (노드와 분리해 단독 시험 가능)
  ③ 안전 제한: 관절 한계 초과 거부, 속도·변화량 제한
출력: /ak45/command (어깨 3 + 팔꿈치 1), /servo/command (손목 + 손가락)
```

- 파지는 처음에 규칙 기반 함수(정해진 각도로 쥠)로 두고, RL이 준비되면 그 부분만 교체한다.
- 서보 피드백이 없어서 "잡았는지"는 압력센서(10/29 이후) 전까지 확인할 수 없다. 그때까지는 정해진 시간만큼 쥐는 방식이다.
- 도달할 수 없는 목표를 받으면 거부하고 대기할지 정해야 한다. [미확정]
- IK 체인의 관절 수는 URDF 7DOF/8DOF 불일치가 정리되어야 확정된다. [미해결]

## 5. 구현 순서

최소로 시작하고 나중에 확장한다. [확정, 2026-10-22] 노드끼리는 토픽 이름과 메시지 형식으로만 연결되므로, 뒤 단계의 노드를 붙여도 앞 단계 노드를 고칠 필요가 없다.

| 단계 | 만들 것 | 목표 | 입력 | 상태 (2026-11-07) |
|---|---|---|---|---|
| **1** | `stm_bridge` (+ 기존 `ak45_node`) | motor_id + 각도 → 서보가 움직인다 | `ros2 topic pub`으로 손으로. 시험은 `vcan0` 먼저, STM 쪽은 `STM32_SERVO_SPEC.md` 8장 순서 | **vcan 시험 완료**(`005#090384` 등 프레임 확인). 실물(STM 보드) 시험 전 |
| 2 | `/servo/state` 발행 (A안) | 마지막으로 보낸 각도 확인 | 위와 같음 | **구현·vcan 시험 완료** |
| 3 | `arm_control` | 순서 관리 + 팔 IK + 파지(규칙 기반) | 물체 위치를 손으로 입력 | **일부**: 시뮬 관절각 변환·속도 제한·감시·`pick_place`·`calibrate`는 구현. 상태머신·IK 노드화·파지 함수는 미구현 |
| 4 | `vision_node` (카메라 입력과 탐지·좌표 변환을 한 노드에서) | 카메라로 물체 위치를 얻어 `/object_pose`로 발행 | 폰/웹캠 영상 | **일부**: `fixed_pose` 시험용 발행, `result_json`으로 카메라 프로그램의 `inference.json`을 읽어 발행(카메라→로봇 변환은 임시 `camera_to_robot`, yaw는 카메라 기준). 파일로만 시험, **실제 카메라 미시험**, 폰 고정 위치와 정확한 변환 미정 |
| 5 | STM 생존 신호(B: StdId 0x107, 1초, `[seq][rx_count]` 제안), `/pressure`(10/29 이후), RL 파지, 따라하기, 주먹인사, RViz | 확장 | | 미착수 |

jax_ros 통합 작업의 1~5단계(위 표의 단계와는 별개, `STATUS.md`)는 2026-11-07에 가상 CAN에서 끝났다: ① 레포 뼈대와 `ak45_can`·`ak45_ros2` 복사, AK45 4개로 변경 ② `stm_bridge` ③ `arm_control`(SimToReal의 `sim2real_node`를 쪼갬, 같은 입력에서 같은 모터 명령) ④ `vision_node` 뼈대 ⑤ `arm.launch.py`, README, 문서. **실물 시험은 아직 하지 않았다.**

- B(STM 생존 신호)는 STM 펌웨어 변경이 필요해서 5단계로 미뤘다. 미루는 동안 연결이 끊겨도 알려 주는 것이 없지만, 1단계는 서보를 눈으로 보며 시험하므로 위험은 작다. [제안]
- 팔 움직임은 IK, 파지만 RL이다. 파지는 처음에 규칙 기반 함수로 두고, RL이 준비되면 교체한다.

## 6. 미결 사항

1. **카메라 방식**: 폰 스테레오(S25) 또는 웹캠 2대. 폰은 제조사 카메라 구현에 따라 동시 스트림이 보장되지 않고(Android Multi-camera API 문서), 렌즈 종류가 달라 보정이 필요하다. 영상을 PC로 보내는 방법도 정해야 한다. [미확정]
2. **GPU**: 학교에서 빌린 노트북. GPU 모델, OS 설치 방법(외장 SSD / 듀얼 부팅 / 교체), 학교 허락 확인 필요. 대여 기간도 확인. [미확정]
3. **카메라→로봇 좌표 변환(tf)**: 누가 보정하는지 [미확정].
4. **AK45 관절 이름과 개수**: `ak45_node`는 2026-11-07부터 `joint_names`를 정확히 **4개(ID 0x01~0x04)** 요구한다(`kNumMotors=4`, `ak45_node.cpp`의 생성자 `Ak45Node::Ak45Node()`에서 `joint_names_.size() != kNumMotors`이면 `invalid joint_names size`로 종료, 기본 이름 `ak45_1`~`ak45_4`). 피드백 없는 모터에는 명령을 안 보내므로(`Ak45Node::onCommandTimer()`의 S3 검사 `st.valid == 0 || !wd_ok`) 4대 중 일부만 있어도 동작한다. 실제 관절 이름으로 바꿀지는 [미확정]. (`ak45_can` 라이브러리의 `NUM_MOTORS=6`은 그대로라 종료 시 브레이크 프레임이 ID 5·6에도 나간다. 받는 모터는 없다.)
5. **CAN ID 충돌**: 없음. AK45는 확장 29비트 ID(`ak45_can`의 `can_transmit_eid()`가 `CAN_EFF_FLAG`를 붙여 보내고, `rx_thread()`가 `CAN_EFF_FLAG`가 없는 프레임을 버린다), STM 서보는 표준 StdId 5(CAN_STM `main.c`의 `CAN_SERVO_STDID 5U`, 수신 콜백 `HAL_CAN_RxFifo0MsgPendingCallback()`이 `rxHeader.IDE != CAN_ID_STD` 또는 `rxHeader.StdId != CAN_SERVO_STDID`인 프레임을 버린다)이다. (2026-11-04 코드 확인, 이름 기준 재확인 2026-11-07: ak45_can `6db174f`, CAN_STM `2c9c45f`)
6. **서보 각도 오프셋**(상위 제어기에서 −90~+90 → 0~180 변환 규칙)을 관절마다 정해야 한다. [미확정] (변환 자체는 `arm_control`의 `calibration.json`(`zero_deg`, `sign`, `scale`)이 한다. 값은 전부 `verified:false`다.)
7. URDF 7DOF/8DOF 불일치(강준영과 조율), 온도 이상 원인. [미해결] AK45 위치 피드백 기준축은 출력축으로 판별됨(2026-08-27, `ak45_node.yaml` 주석). (단 `ak45_ros2/docs/ros2.md`와 README §9는 "V3 미실시"라고 적어 서로 다르다. `STATUS.md` 참조. 첫 실제 명령은 2° 정도로 출력축 회전을 눈으로 확인한 뒤 진행.)
