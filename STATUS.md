# STATUS — Team JAX 단일 팔 Pick & Place (재준 담당 파트)

> 기준: 2026-11-04. [확정] 사용자가 정함 / [제안] 아직 승인 전 / [미확정] 정해지지 않음 / [검증필요] 코드나 실물로 확인 못 함.
> 관련 문서: `NODES.md`(ROS2 노드), `STM32_SERVO_SPEC.md`(서보 CAN 명세), `STM32_PINMAP.md`(핀맵), `README.md`(STM 펌웨어), `roles/00_INDEX.md`~`08_SYSTEMD.md`(역할별 필요한 값, 2026-11-04).

## 1. 전체 목표

단일 팔 로봇이 카메라로 물체를 찾아 집어서 옮기는 Pick & Place. 마지막에 재미 기능으로 사람 얼굴과 주먹이 보이면 주먹인사를 한다(안전 설계 필요).

## 2. 역할 분담

| 담당 | 맡은 일 |
|---|---|
| 재준 | 통신·모터 제어(STM32 서보, AK45 CAN), 역기구학, 카메라·비전, ROS 구조, 대시보드 |
| 정우 | RL(파지)만 |

## 3. 세부 목표와 완료 기준

완료 기준은 전부 [제안]이다. 승인하면 이 기준으로 완료 여부를 판단한다.

| # | 항목 | 완료 기준 |
|---|---|---|
| 1 | STM 펌웨어 마무리 + 파지센서(ADC) | PlatformIO로 빌드되어 보드에 올라가고, `cansend can0 005#050384`로 motor 5가 90.0°로 간다. 서보 10개 전부 확인. 파지센서는 10/29 이후 6채널 값이 읽힌다. |
| 2 | ROS 구조 짜기 | `stm_bridge`가 `/servo/command`를 받아 서보를 움직이고, `vcan0`에서 먼저 시험한다. 이후 `arm_control`, `vision_node`가 같은 토픽 규칙으로 붙는다. |
| 3 | 아루코 마커 | 필수 아님 [제안, 11/4]: 카메라 위치·방향을 보정하는 도구 중 하나(체커보드·실측도 가능). 카메라 위치(몸통·스탠드 / 손목)가 정해지면 쓸지 결정한다. 완료 기준은 알려진 위치에 놓은 큐브의 x, y 오차를 mm로 기록하는 것. |
| 4 | 카메라 분석 | SteroDepthSeg를 감싼 `vision_node`가 `/object_pose`(베이스 좌표)를 발행한다. |
| 5 | 역기구학(아루코 포함) | 목표 위치 → 관절각 함수가 단독으로 시험되고, 도달 불가능한 목표는 거부한다. |
| 6 | MuJoCo → 현실, 파라미터 조정 | 1단계: 시뮬레이션 관절각과 실제 서보 각도의 대응표(부호·오프셋·한계각). 2단계 이후는 1단계로 부족할 때만. |
| 7 | 웹 대시보드(CAN 모터) | ROS 없이 CAN 직접 [확정]. 기존 `ak45_can/tools/ak45_dashboard.py`(AK45 상태·제어)에 서보 패널을 더할지(A) 새로 만들지(B) [미확정, A 제안]. |
| 8 | systemd 자동 실행 | 전원을 켜면 CAN 설정과 노드가 자동으로 뜬다. 수동 실행 순서가 확정된 뒤에 만든다. |
| 9 | 통합 시험 | 물체 인식부터 집기까지 한 번에 돈다. |

권장 순서 [제안]: 1 → 2(`stm_bridge`) → 6의 대응표 → 3 → 4 → 5 → `arm_control` 순서 관리 → 6의 이후 단계 → 7 → 8 → 9.

## 4. 했던 일 (완료)

- STM 서보 펌웨어 `main.c` 작성: CAN 1 Mbps 수신, 서보 10채널 PWM(500~2500 µs), 잘못된 프레임 무시. 로컬 브랜치 `servo-pwm-can`에 커밋 2개(`6d6cbf9`, `d29e230`). 호스트에서 수식만 시험했고 **ARM 빌드는 못 해 봤다**. [검증필요]
- 문서 작성: `STM32_SERVO_SPEC.md`, `STM32_PINMAP.md`, `README.md`, `NODES.md`.
- ADC 핀 6개(PC1, PC2, PC4, PC5, PA2, PA3) 매핑을 Zephyr pinctrl 및 `hal_msp.c`와 대조해 확인.
- SteroDepthSeg 소스 읽기 완료(아래 §7).
- PlatformIO + CubeMX 가져오기 방법을 메모리에 기록.
- 2026-11-01~02: MuJoCo 헤드리스 검증(`team-jax/IK`) — 동작2 IK 0.00 mm·파지 9/9, 동작1 smoke test 통과·배달 11/12(시드 0~11. README의 100/100과 차이 나는 원인은 미확인, MuJoCo 버전이 다름).
- 2026-11-04: Fusion 파일 `profile (Assembly)`로 실물 구성 확인 — 스탠드에 팔이 매달린 구조, 바닥→어깨 모터 꼭짓점 1207.5 mm(CAD). 시뮬 q=0 FK 계산(팔이 곧게 늘어짐, 손 기준점이 어깨 아래 696 mm).
- 2026-11-04: 압력센서 = 마블덱스 FSR **RA12P** (데이터시트 J200712 7쪽: 14.15 × 12 mm, 주 감지 지름 4.8 mm, 단단한 PCB 기판, 최대 하중에서도 약 2.5 kΩ).
- 2026-11-04: 책상 높이 73.5 cm(사용자 측정) → D ≈ 472.5 mm로 다시 계산(`roles/03`, `05`). FSR 분압 전원을 3.3 V에서 보드 3V로, 분압 예시를 RA12P 그래프 기준으로 바로잡음(`roles/02`).
- 2026-11-04: 역할별 필요한 값 문서 9개(`roles/`) 작성. 근거 파일·행을 다시 대조하면서 이전 답변의 틀린 값 10개를 바로잡음(`roles/00_INDEX.md` 표).

## 5. 하고 있는 일

- MuJoCo → 실물 1단계(잡기 없이 큐브 위 접근 위치까지 팔 보내기) 준비: 영점 자세 결정, 책상 높이(D) 측정, 카메라 위치 결정 (`roles/03`, `05`, `04`).

## 6. 할 일 (다음 작업)

1. 영점 자세 결정 → 관절 ↔ 모터·ID 대응 → 영점 표 채우기 (`roles/03_IK_ZERO.md` 6절).
2. D 직접 측정(책상 73.5 cm 확인, CAD 기준 D ≈ 472.5 mm) → 책상 높이 차이 대응 선택 (`roles/05_SIM_TO_REAL.md` 4절).
3. 카메라 위치·종류 결정 (`roles/04_CAMERA_VISION.md`).
4. AK45 1~4번 ID가 실물과 맞는지 확인 (대시보드 `--read-only`). `ak45_node`가 6개로 고정이라 4개 구성에 맞게 고칠지 결정.
5. FSR(RA12P) 부착 위치와 Rc 시험 → ADC 6채널 DMA 설정, 센서 CAN 프레임 정의 (`roles/02_STM32_FIRMWARE_FSR.md`).
6. 보드에 올린 펌웨어가 `servo-pwm-can` 브랜치인지 확인하고 원격에 반영.
7. `stm_bridge` 1단계 설계 승인 후 구현: 10 Hz 재전송 여부와 `/servo/command` 메시지 타입(JointState)을 정해야 한다.
8. `vision_node`, `arm_control` 상태머신, IK 함수.
9. 맨 끝에 대시보드 서보 패널, systemd, 주먹인사.

## 7. 사용 기술

| 영역 | 내용 |
|---|---|
| MCU | STM32F407G-DISC1, HAL, PlatformIO(`disco_f407vg`, `stm32cube`), HSE+PLL 168 MHz, 타이머 클럭 84 MHz, PSC 83, ARR 19999(20 ms) |
| CAN | 1 Mbps, SN65HVD230, SocketCAN. 서보 노드 StdId 5, 프레임 `[motor_id][angle_H][angle_L]`(각도×10, 0~1800) |
| 서보 | PWM 10개. 위치 피드백 없음. 핀 배치 B안(`STM32_PINMAP.md`) |
| PC | ROS2 Humble, Ubuntu 22.04(노트북, 학교에서 대여 — OS 설치 방법 [미확정]) |
| 모터 | AK45 4개(CAN), 서보 10개(PWM) |
| 비전 | 갤럭시 S25 후면 두 렌즈 스테레오, USB ADB 스트림(TCP 8765), Fast-FoundationStereo 깊이 + YOLO11 seg. 레포에 ROS·AprilTag 코드는 없음(검색 0건). |
| 시뮬레이션 | MuJoCo (`team-jax/IK`: 동작1 스크립트 컨트롤러, 동작2 웹캠 텔레옵) |

SteroDepthSeg 수치(레포 문서 기준, 직접 측정하지 않음): 처리 시간은 S24 시험 중앙값 276.8 ms(레포 STATUS 623행. 이전에 적은 240 ms는 근거를 다시 찾지 못함), 이론 깊이 오차 ≈ Z²×0.070(Z가 m이면 오차도 m, 0.5 m에서 18 mm), 0.4~0.6 m에서 실측 약 1%, baseline 16.64 mm(README)와 15.76 mm(레포 STATUS)가 서로 다름, 보정은 이 S25 전용, 초점 1 m 고정, 폰 화면을 켜 둬야 함. 출력 `center_world_m`의 원점은 왼쪽 렌즈이고 방향만 중력 기준이라 로봇 베이스 좌표가 아니다.

## 8. 문제점 및 해결방안

| 문제 | 상태 | 해결방안 |
|---|---|---|
| `team-jax/can_ioc` 푸시 403 | 막힘 | 로컬 `main.c`를 사용자가 직접 커밋하거나, 저장소 관리자가 Claude GitHub App을 설치 |
| ARM 빌드 미확인 | 열림 | PlatformIO에서 실제 빌드(§6-1) |
| 카메라→베이스 변환 없음 | 열림 | 아루코 또는 손 실측(§3-3) |
| 지연 약 240 ms | 열림 | 움직이는 중 보정 대신 "한 번 보고 → 확정 → 이동" 구조 [제안] |
| `ak45_node` joint_names 6개 vs AK45 4개 | 해결(11/4) | 코드가 6개를 강제하고, 피드백 없는 모터는 송신에서 빠지므로 4대로도 동작. 실제 관절 이름으로 바꿀지는 [미확정] |
| AK45 CAN ID가 StdId 5과 충돌하는지 | 해결(11/4) | 충돌 없음: AK45는 확장 29비트, STM 서보는 표준 11비트 |
| URDF 7DOF/8DOF 불일치, 온도 이상 | 열림 | 강준영과 조율. AK45 피드백 기준축은 출력축으로 판별됨(2026-08-27, `ak45_node.yaml` 주석) |
| 실물 D(어깨→책상) 약 472.5 mm vs 시뮬 600 mm | 열림 | 받침(A: 약 12.75 cm, B) / 시뮬 책상 높이 변경(C) / 1단계는 IK만 사용(D). 책상 73.5 cm에서는 동작1의 접근·운반 높이 0.4675가 상한 0.42에서 잘림 (`roles/05` 4절) |
| 책상 가장자리 수평 거리 300 mm의 측정 기준 | 열림 | 기둥 중심 기준이면 도달 칸 중 월드 x ≤ 0.20(기둥에서 0.30~0.50 m)만 책상 위. 어깨 팔 선 기준이면 책상이 너무 멀다 (`roles/03` 5절) |
| 카메라 설치 위치·종류 미정 | 열림 | 몸통·스탠드 고정이면 보정 한 번으로 끝남 (`roles/04`) |
| 기존 대시보드의 임의 프레임 기본값이 확장 프레임 | 주의 | STM으로 보낼 때는 체크를 끈다 (`roles/07`) |
| 대시보드 "전체 정지" = 브레이크 0 A | 주의 | 팔을 달면 팔이 떨어짐. "정지"의 뜻을 정해야 함 |
| FSR 부착 위치·Rc 미정 | 열림 | 모델 RA12P 확인(11/4). 최대 하중에서도 약 2.5 kΩ라 Rc 1 kΩ면 ADC 범위의 약 29 %만 씀 → 4.7·10 kΩ 시험 (`roles/02`) |
| 서보 각도 오프셋 | 열림 | 대응표 측정(§3-6) |
| 학교 노트북 GPU 모델·OS 설치·대여 기간 | [미확정] | 학교에 확인. SteroDepthSeg가 `cuda:0` 고정이라 NVIDIA GPU 필요 |
| 폰 교체 시 재보정 | 열림 | `doc_CALIBRATION.md` 절차 |

## 9. 파라미터 변경 이력

없음(펌웨어 파라미터는 처음 작성한 값 그대로: PSC 83, ARR 19999, CCR=500+각도×10×2000/1800 µs). 2026-11-04 기준 시뮬 파라미터(`TABLE_SURFACE_Z` 등)도 아직 바꾸지 않았다.

예정된 시뮬 변경(`PROMPT_SIM_ADAPT.md`, 실행 전): `TABLE_SURFACE_Z` 0.25 → 환경변수(기본 0.25, 실물 0.3775), `WORKSPACE_LOW/HIGH` z 절대값 0.24/0.42 → 책상 기준 상대값(−0.01/+0.17), `HOME_ARM_QPOS`·`reachmap.npz` 높이별 재계산(기본 파일은 유지), `CUBE_FRICTION` 2.0 → 0.4, 패드 충돌 마찰 1.0 → 0.4(motion1에서만). 영향 파일: `env.py`, `make_arm_urdf.py`(생성기 인자 추가), `build_reachmap.py`, 문서. 실행하면 이 표를 이전값 → 새값으로 갱신한다.

## 10. 결정 기록

- ROS는 최소 구성으로 시작하고 나중에 확장한다. [확정]
- `vision_node` = 카메라 입력 + 탐지 + 깊이 + 좌표 변환을 한 노드에서(담당: 재준). [확정]
- `/servo/state`는 마지막 명령값(A안). STM 생존 신호(B)는 5단계로 미룸. [확정]
- ADC는 10/29 이후로 미룸. [확정]
- FreeRTOS는 쓰지 않는다. [확정]
- 주먹인사는 가장 마지막 재미 기능. [확정]
- 웹 대시보드는 ROS 없이 CAN으로 직접 통신한다. [확정]
- MuJoCo → 실물 1단계는 잡기 없이 큐브 위 접근 위치까지 팔만 보낸다. [확정, 11/3]
- 압력센서가 오기 전 접촉 판정은 AK45 전류(토크)로 먼저 시도한다. [제안, 시험 승인 대기]
- 아루코 마커는 필수가 아니다(보정 도구 중 하나). [제안]
- 책상 높이 차이는 시뮬 쪽을 바꾸는 방법(C)으로 간다. 단, 먼저 복사본에서 높이별 도달 가능 칸 수를 실험하고 승인한 뒤 본 코드를 바꾼다. [확정, 11/4]
- 시뮬 마찰을 0.4로 바꾼다(요청). 접촉 마찰이 두 지오메트리 중 큰 값이라 큐브(2.0)와 패드(1.0)를 **둘 다** 내려야 0.4가 되고, 큐브–책상 마찰이 2.0 → 1.0으로 바뀌는 부작용이 있다(MuJoCo 3.17.0에서 직접 확인). 0.4의 근거(실측/추정)는 [미확정]. [확정: 변경 요청, 11/4]
- 2026-11-05: 좌표 원점은 어깨 모터 회전축으로 결정(사용자). 관절 영점 자세(q=0 vs HOME)는 아직 미정. D, 책상 수평거리 30 cm 모두 이 축 기준으로 다시 재야 한다 (`roles/03` 1절).
- 2026-11-05: 실물 좌표계(x 왼쪽 +, y 뒤 +, z 위 +)와 시뮬 변환(월드 = (0.34 + y, −x, 0.85 + z)) 승인, `roles/03` 1-1절에 반영. 사용자 측정 대략값: 책상 수평거리 10 cm, 큐브 위치 10~38 cm [기준·방향 확인 필요]. 도달 범위와의 비교는 시뮬 실험 필요.
- 2026-11-05: 좌표 변환 가정 오류 가능성 발견 — 변환이 "앞 = 팔이 뻗는 쪽"을 가정했으나 사용자 정의는 FRONT 글자 기준일 수 있음. 책상 위치(FRONT 정면 / 팔 매달린 쪽 옆) 확인 전까지 `roles/03` 1-1절 변환은 [재검토 필요].
- 2026-11-05: 사용자 확정 — 책상 수평 거리 10 cm(거치대 표면 기준), 큐브 50 mm. 원점 점(롤 관절 원점 0.288 / 롤·요 교점 0.238 / base_link 0.34)은 미정.
- 2026-11-05: 사용자 정의 — 모터 위에서부터 AK45는 CAN ID 1~4, STM 서보 노드는 표준 StdId 5. 관절 대응(shoulder_roll=1, shoulder_yaw=2, upper_arm_roll=3, elbow=4)은 순서 기준 추정이라 [검증필요]. 서보: 펌웨어 주석 기준 W_PITCH=9, W_YAW=8, W_X=7(hand_roll인지 미확인).
- 2026-11-05: STM 서보 노드 CAN 표준 ID 7 → 5 (사용자 변경). 이유: 사용자 결정(AK45 1~4, STM 5). 펌웨어: 사용자의 `main.c` 38행 `CAN_SERVO_STDID 5U`. 문서 12개의 StdId·`007#` 예시를 5로 맞춤(NODES, ARCHITECTURE, PROMPT_DASHBOARD, README, STM32_SERVO_SPEC, STATUS, roles/00·01·02·03·06·07). 아직 7인 곳: 저장소 `can_ioc/Core/Src/main.c`(86행 `SERVO_CAN_ID 7`, 사용자 로컬 파일만 5), 대시보드 `ak45_dashboard.py`의 `--raw-id` 기본값 7과 주석(`roles/07` 16행은 코드 기준이라 7 유지). 보드에서 `005#030384` 동작 확인 전.
- 2026-11-05: `ak45_ros2`를 AK45 4대(ID 0x01~0x04)로 변경(사용자 승인). `ak45_node.cpp` kNumMotors 6→4·ID 목록·기본 joint_names·에러 문구·진단 주석, `config/ak45_node.yaml`(joint_names 4개), `scripts/ak45_deg`(VALID_JOINTS), README·AGENTS들 동기화. `docs/ros2.md`는 6개 시절 기록이라 상단에 변경 안내만 추가. 라이브러리 `ak45_can`(NUM_MOTORS 6)은 그대로라 종료 시 브레이크 프레임이 ID 5·6에도 나감(받는 모터 없음). **빌드·실행 미검증** (이 환경에 ROS 없음) — Mac/Linux에서 `colcon build` 필요.
- 2026-11-05: 사용자 결정 — 원점 = 오른쪽 어깨 모터(shoulder_roll) 회전축 중심, 영점 자세 = q=0(팔 내림), 실물 HOME = 팔꿈치만 접은 자세(각도 미정). D는 "50 cm대"라고 함(정확한 mm 필요, CAD 추정 472.5 mm). "팔이 내가 보는 기준으로 오른쪽": 어느 쪽에서 본 기준인지 불명확해 [확인필요] — 책상 위치(FRONT 정면 / 팔 옆) 답은 아직 없음.
- 2026-11-05: 사용자 정정 — FRONT 글자 앞에서 보면 팔은 **왼쪽**(시뮬과 같음, 거울상 아님). 실물 HOME 팔꿈치 각도 90° 확정. D = 롤 모터 축 중심에서 책상 윗면까지 수직 거리(대략 50 cm대, 정확한 mm 필요). 책상 위치(팔 옆 / FRONT 정면)는 아직 답 없음 → 좌표 변환 [재검토 필요] 유지.
- 2026-11-05 22:31: 책상 위치 질문에 "FRONT 글자 정면입니까?"라고만 답함 — 책상이 FRONT 정면이라는 답으로 읽었으나 [확인 대기]. 맞다면 좌표 변환은 축 교환 없음(월드 = (0.288 + x, y, z))이고, 시뮬 도달 지도(어깨 기준 x −0.34~−0.08, y −0.30~+0.05)가 책상 정면 영역을 덮는지 다시 봐야 함.
- 2026-11-05 22:50: 책상은 FRONT 글자 앞이라고 확정(사용자). 좌표 변환 정정 — 실물 축 = 시뮬 월드 축(축 교환 없음), 월드 = (0.288 + x, y, z0 + z). 이전 `x_real = −y_sim` 변환은 폐기, `roles/03` 1-1절·`PROMPT_SIM_ADAPT.md` 갱신. 시뮬 도달 칸(책상 0.25): base_link 기준 x −0.38~−0.10, y −0.275~+0.025. 실물 책상이 이 영역을 덮는지 줄자 확인 필요. 사용자의 "큐브 앞 10~38 cm"는 이 x 범위(팔 쪽 10~38 cm)와 같은 값일 가능성이 큼 [확인 대기].
- 2026-11-05 22:50: 사용자 지시 "대충 하고 나중에 고치자" — `PROMPT_SIM_ADAPT.md`를 임시값(D 0.50 m → 시뮬 책상 0.35, 롤 원점 0.288/z0 0.85 가정, 마찰 0.4 패드–큐브만, 큐브 50 mm/0.05 kg, 도달 칸 전체)으로 바로 진행하도록 바꿈. 전부 파라미터로 두어 나중에 교체.
- 2026-11-06: 사용자 서보 영점 표 접수 — 서보 id/핀/타이머는 펌웨어 `servo[]`와 일치(확인함). 영점(°): 0:90, 1:0, 2:90, 3:0, 4:90, 5:0, 6:?, 7(W_X):0, 8(W_YAW):0, 9(W_PITCH):0. 손목 3개의 영점이 0°라 한쪽 방향으로만 움직임 → 부호(−1 필요)와 시뮬 IK의 손목 값 부호 확인 필요. F_ROT(6) 영점 미정. `roles/03` §6에 반영.

## 2026-11-06 sim-to-real(정우) 비교 검토
- 읽은 것: README, DEPLOYMENT_SPEC, ARCHITECTURE, deploy/{controller,run,observations,policies,trajectory}.py, sim_validation/scene.py (미확인: tests, ISAAC_COMPARISON, MODEL_VALIDATION, .pt 조회표 - JaxIsaacLab/ 는 레포에 없음)
- 결론: MuJoCo 전용(CAN/실물 어댑터 없음), 파지 완료 미달성(DEPLOYMENT_SPEC 명시). 1단계(큐브 위 2~3cm 접근)는 기존 방침 A(IK 웨이포인트) 유지, 이쪽은 3단계 이후 후보.
- 불일치 항목: 큐브 4cm(scene.py) / 5.5cm(학습) / 50mm(우리), 월드·베이스 좌표(robot_mount z=0.305, yaw -90°), 책상 73.5cm, "10cm" 정의(center10/edge10).
- 2026-11-06: 정우 쪽도 큐브 50mm로 가기로 함(사용자 전언). 큐브 크기 불일치(4/5.5/50mm)는 50mm로 통일. 남은 확인: 기존 정책(5.5cm 학습)이 50mm에서 유효한지/재학습 여부, sim-to-real scene.py CUBE_SIDE_M(0.04) 수정 필요.

## 2026-11-06 1단계 IK 드라이런 (샌드박스에서 ikpy 실행, 임시값 D=0.50 z0=0.85)
- 스크립트: /home/claude/stage1/dryrun.py (전송 없음, motion2/arm/ik.py의 build_chain/world_to_local 사용). 목표 월드 z = 0.85 + (-D+0.05+0.025) = 0.425
- 결과: 실제 (x,y) = (-0.20,-0.15),(-0.30,-0.10),(-0.10,-0.20),(-0.20,0) 모두 위치만 풀면 오차 0mm 도달. palm-down(손바닥 아래) 제약은 앞 3점 0~5mm, (-0.20,0)은 42.8mm 오차로 불가.
- 문제: 손목 값이 음수(wrist_pitch -41~-87°, wrist_yaw -90° 한계값) -> 손목 서보 영점 0° 한방향이라 부호 -1/오프셋 필요. 관절↔모터 대응, 실제 모터 회전 부호 미검증.
- 미검증: D, z0, 관절 부호/영점, AK45 ID 1,3,4. 샌드박스의 ik.py는 원격 8d8b157 기준(사용자 Mac 최신 커밋 아님).

## 2026-11-06 motion1 IK 재검증 (user: "지금 motion1만 하면 됨")
- 스크립트: /home/claude/stage1/dryrun_m1.py, sweep_m1.py (motion1 solve_site_pose, palm-down, HOME_ARM_QPOS 시드, 전송 없음). 앞서의 ikpy(motion2) 결과는 solver가 달라 참고용.
- 결과: D=0.50(목표 월드 z=0.425)에서 palm-down 도달 격자 거의 전멸(x=-0.30~-0.40에 띄엄띄엄). D=0.60(z=0.325)은 x -0.35~-0.05, y -0.30~+0.05에서 넓게 도달. tilt 허용 0.20rad로 완화해도 거의 안 늘어남 -> 원인은 높이.
- 결론: 실물 책상이 시뮬 책상(roll축 아래 0.60m)보다 높으면 motion1 palm-down 영역이 사라짐. D(정확한 mm)가 핵심 입력. 선택지: 1단계는 palm-down 완화(위치만), 또는 책상/로봇 높이 조정, 또는 sim TABLE_SURFACE_Z 변경 후 재맵(PROMPT_SIM_ADAPT).

## 2026-11-06 D 확정, 손목 서보 영점 정정
- D = 0.4725 m (사용자 확인, CAD 472.5mm와 일치). 앞서 "50cm대"/임시 0.50은 폐기. 목표 월드 z = 0.85 + (-0.4725+0.075) = 0.4525 (z0=0.85 미검증 유지)
- motion1 palm-down 도달 격자 재계산(D=0.4725): 거의 없음(x=-0.30~-0.40 몇 점뿐). 시뮬 책상(D=0.60)보다 약 12.75cm 높아서 도달 영역이 사라짐 -> 1단계는 palm-down 완화(위치만) 필요, 또는 시뮬 책상 높이 변경+재맵.
- 서보 영점 정정(사용자): 9번 W_PITCH 기본 90°(양방향). 7번 W_X, 8번 W_YAW는 직접 측정 필요(전선 걸림 방지 위치, 정확값 모름). 영점표(roles/03_IK_ZERO.md §6) 갱신.

## 2026-11-06 오후 AK45 ID 설정
- 사용자: AK45 ID 1·3·4 "설정 다 함". 통신(피드백 수신) 확인은 아직 -> 다음: 4모터 패치 적용·빌드 후 `ros2 run ak45_ros2 ak45_deg --list`로 ak45_1~4 응답 확인. (관절↔모터 대응은 여전히 [추정])

## 2026-11-07 team-jax 조직 전체 확인 (15개 저장소)
- 새로 확인: SimToReal(재준 본인, ROS2 Jazzy 패키지 sim2real: calibration.json 관절→모터 매핑, calibrate, sim2real_node, pick_place=시뮬 궤적 계획→검사→감시 재생, 2026-11-05), CAN_STM(펌웨어, main.c:38 CAN_SERVO_STDID 5U 확인), jaxMotorDashboard(웹 대시보드, 11/2), SteroDepthSeg(스테레오 깊이, 10/25), JaxIsaacLab, CAN_IOC, Can(분할 안내), URDF, UART_STM, STM32_TEST, CV_TEST.
- 불일치 발견: (1) SimToReal calibration.json 서보 zero_deg 전부 90, 전부 verified=false — 사용자 영점표(손가락 joint2 0°, 7·8번 미측정)와 다름 (2) GitHub ak45_ros2(10/22)는 6모터 — 4모터 변경 미푸시 (3) ROS 배포판: SimToReal README는 Jazzy(Ubuntu 24.04), 내가 안내한 명령은 humble이었음 -> /opt/ros 확인 필요 (4) hand_roll 시뮬 ±120° vs 서보 ±90°.
- 미확인(코드 안 읽음): jaxMotorDashboard, SteroDepthSeg, JaxIsaacLab, URDF, SimToReal sim2real/*.py 상세.

## 2026-11-07 jax_ros 통합 ROS2 레포 계획
- 사용자 결정: 새 레포 하나(jax_ros)에 ak45_can, ak45_ros2(복사), stm_bridge, arm_control, vision_node(뼈대) 패키지로 통합. 원본 레포는 유지.
- 설계 결정(제안, 사용자 확인 대기): /servo/command = JointState, 서보 이름, position=모터 각도(rad). 변환은 arm_control. NODES.md의 "노드(stm_bridge)에서 변환"과 다름 -> 승인 시 NODES.md 수정.
- 산출물: /home/claude/PROMPT_JAX_ROS.md (Claude Code용). SimToReal 소스(node.py, ros_io.py, protocol.py, calibration.py, motion.py, watch.py, planner.py, pick_place.py, play.py 앞부분, launch)를 읽고 파일 단위 분할표 작성. play.py 뒷부분(run_on_robot)과 calibrate.py, tests는 미확인 -> 프롬프트 0단계에서 Claude Code가 확인.

## 2026-11-07 RL·압력센서 검토 (근거 정리)
- 읽은 것: sim-to-real ISAAC_COMPARISON.md, MODEL_VALIDATION.md, DEPLOYMENT_SPEC.md, deploy/observations.py(obs=14q+14qd+3+3+3+6ADC=43), JaxIsaacLab/artifacts SPEC_pressure_sensors·SPEC_joints_sensors(앞부분).
- 정우 RL 현황: MuJoCo에서 4cm 큐브 30초 제한 내 들기 미완료(압력>0 프레임 7/1800). Isaac 99% 유지(5.5cm, 마찰0.4)는 모델 요약 보고치이며 정우가 재실행하지 않음. 원 큐브 5.5cm, 조회표도 그 조건.
- 실물에 없는 관측: 서보 9개(손목3+손가락6) 위치/속도 피드백 없음, 큐브 위치/속도(비전 변환 없음), ADC 6채널(펌웨어 미읽음).
- 계산: RL 로봇 URDF(q=0, robot_mount z=0.305, yaw -90) 첫 어깨 관절(shoulder_pitch_to_shoulder_roll) anchor z=1.155, 책상 윗면 0.735 -> D_RL≈0.420 m. 실물 D=0.4725 -> 책상이 약 5.25cm '낮음'. Grid.__init__는 |dz|>0.03이면 ValueError -> 조회표 범위(±3cm) 밖(가정: 실물 롤축=Isaac 관절0 축).
- 두 시뮬 설정 충돌: motion1 palm-down 도달은 D≈0.60 근처에서 넓고, RL 조회표는 D≈0.42±0.03. 실물 0.4725는 둘 다 가장자리.
- 팔 7관절 범위: calibration.json sim_range_deg = Isaac 한계(7/7 일치) -> 같은 로봇 모델.
## jax_ros 통합 레포 진행 (2026-11-07~)

**목표:** 흩어진 ROS2 코드를 `jax_ros` 한 레포로 모으고 `sim2real_node`를 노드 4개(`ak45_node`, `stm_bridge`, `arm_control`, `vision_node`)로 쪼갠다. 동작은 바뀌면 안 된다.
**환경:** Ubuntu 24.04, ROS 2 **Jazzy**(`/opt/ros/jazzy`), 작업 폴더 `~/ros2_ws/src/jax_ros`. push 안 함.
**사용 기술:** ROS 2(rclcpp, rclpy, ament_cmake/ament_python), SocketCAN(vcan으로 시험), python-can, CAN_STM 서보 프레임(StdId 5).

### 단계 현황
| 단계 | 상태 |
|---|---|
| 0 조사·계획 | 완료(승인 받음) |
| 1 뼈대 + ak45_can/ak45_ros2 복사 + 4모터 | **완료** (아래) |
| 2 stm_bridge | **완료** (프레임·노드·vcan0 실측 확인, 아래) |
| 3 arm_control | **완료** (쪼개기 전/후 숫자 비교 통과, 아래) |
| 4 vision_node 뼈대 | **완료** (아래) + calibrate 대기 수정 |
| 5 launch·README·NODES.md·package.xml 점검 | 할 일 |

### 1단계에서 한 일
- 원본 복사(`.git` 제외, 각 `SOURCE.md`에 URL·SHA): ak45_can `6db174f`, ak45_ros2 `5495298`. 원본 그대로 한 번 커밋한 뒤 변경을 따로 커밋.
- `ak45_node.cpp`: `kNumMotors=4`, `kMotorIds={1,2,3,4}`, 기본 이름 `ak45_1..4`, 오류 문구 "정확히 4개", 주석·진단 로그의 6→4. `ak45_node.yaml`(joint_names 4개), `scripts/ak45_deg`(VALID_JOINTS), README·AGENTS 3개 동기화. `docs/ros2.md`는 상단 안내만 추가(6개 시절 본문은 이력).
- 검증: `colcon build --packages-select ak45_ros2` 경고 0. `joint_names` 6개를 주면 FATAL + 종료 코드 1, yaml(4개)은 검증을 통과해 `ak45_init`(can0 없음)에서만 실패. `ak45_deg`는 `ak45_1~4`만 안내.

### 문제점과 해결방안
| 문제 | 상태 | 해결방안 |
|---|---|---|
| ak45_ros2에 기존 테스트가 없음 | 열림 | 1단계 검증은 빌드·파라미터 검증·종료 코드로 대체. 노드 실행 시험은 vcan 준비 후 |
| `ak45_can`의 `NUM_MOTORS=6` | 기록만 | 종료 시 브레이크 0A 프레임이 ID 5·6에도 나감(받는 모터 없음). 라이브러리는 수정하지 않음 |
| `max_command_deg` 주석("출력축 기준 판별 완료", 2026-08-27)과 `docs/ros2.md` §13.9·README §9("V3 미실시") 불일치 | 기록만 | 값은 바꾸지 않는다. 팀 확인 전까지 `calibration.json`의 `scale`=1 유지(로터 기준이면 출력축은 1/36만 돈다). **첫 실제 명령은 2° 소량으로 보내 출력축이 실제로 얼마나 도는지 육안으로 확인한 뒤** 진행 |
| `python3-can` 미설치, vcan0 없음 | **해결(2026-11-07)** | 사용자가 설치. python-can 4.3.1, vcan0 UP 확인 |
| NODES.md·ARCHITECTURE.md가 `jax_ros/`에 없음(`~/Desktop/ros2/`에는 0바이트 파일) | 열림 | 사용자가 내용을 넣으면 5단계에서 반영 |
| `~/Desktop/IK`는 `main` 브랜치, `real_task.py` 없음 | 기록만 | `pick_place` 끝까지 시험은 `sim-table-friction` 브랜치 필요. IK는 수정하지 않음 |
| `~/Desktop/ak45_ros2`에 커밋 안 된 이전 시도(arm_control 등) | 읽기만 | 수정·삭제 금지, 복사 전 질문 |
| 서보 영점(사용자 표: 손가락 joint2 0°, 7·8번 미측정) vs `calibration.json`(전부 90, 전부 `verified:false`) | 열림 | 값 변경하지 않음. 측정 후 사용자가 결정 |
| `hand_roll` 시뮬 ±120° vs 서보 ±90° | 열림 | 변경하지 않음. `check_trajectory`가 잘림을 거절 |
| 관절↔모터 대응 미검증, AK45 ID 1·3·4 통신 미확인 | 열림 | 변경하지 않음. 실물 확인 필요 |

### 2단계에서 한 일 (stm_bridge)
- `stm_bridge/protocol.py`: `Frame`, `servo_frame`, `SERVO_STDID=5`, `SERVO_COUNT=10`. SimToReal `protocol.py`의 서보 부분을 바이트 배치 그대로 옮김.
- `stm_bridge/servo_names.py`: 서보 이름→motor_id 표(`F1_1`=0 … `W_PITCH`=9). 근거 CAN_STM `Stm32pinmap.md` §3. 원본 `calibration.json`의 서보 `name`·`id`와 일치함을 직접 비교(True).
- `stm_bridge/stm_bridge_node.py`: `/servo/command`(JointState, 서보 이름, **모터 각도 rad**) 구독 → 이름·길이·NaN이 틀린 메시지는 통째로 버림 → `degrees()` → `servo_frame` → CAN 송신(파라미터 `servo_interface=socketcan`, `servo_channel=can0`) → `/servo/state`(마지막으로 보낸 각도, rad, motor_id 순) 발행. 첫 명령 전에는 아무것도 보내지 않음. 0~180°를 벗어나면 펌웨어처럼 자르고 WARN.
- 검증(통과): `tests/check_protocol.py` 12개(`005#000000`, `005#030384`, `005#090708`, 클립, 반올림, `005#09044C` 등), `tests/check_node.py` 10개(FakeCan: 첫 명령 전 무송신, 이름·길이·NaN 통째 버림, 서보 2개 → 프레임 2개, 200°→`005#090708`, 상태는 보낸 값). `colcon build --packages-select stm_bridge` 성공.
- vcan 검증 절차(준비 후): `ros2 run stm_bridge stm_bridge_node --ros-args -p servo_channel:=vcan0` → 다른 터미널 `candump vcan0` → `ros2 topic pub -1 /servo/command sensor_msgs/msg/JointState "{name: [W_PITCH], position: [1.5708]}"` 이면 `005#090384`가 보여야 한다. `python3 tests/check_node.py`는 python-can이 있으면 virtual 버스로 돈다.
- 범위 밖 각도: 처음에는 0~180°로 자르고 WARN이었으나, 사용자 기준(잘못된 메시지는 통째로 버림)에 맞춰 **0~180°(±0.05° 여유) 밖이 하나라도 있으면 메시지 전체를 버리도록** 바꿈. 원본 흐름에서는 arm_control이 이미 서보 범위로 자른 값만 보내므로 정상 경로 동작은 같다.

### 2단계 마무리 기준 확인 결과 (2026-11-07, python-can 4.3.1, vcan0)
| 기준 | 결과 |
|---|---|
| 프레임 바이트 테스트(`SERVO_STDID=5`, W_PITCH 90° → `005#090384`) | 통과. `tests/check_protocol.py` 12개 + 이름 표 2개 |
| vcan0에서 `/servo/command` 발행 → `candump vcan0` | W_PITCH 1.5707963 rad → `005#090384`. W_PITCH 1.0 rad → `005#09023D`, F1_1 2.0 rad → `005#00047A`(손계산: 57.3°→573, 114.6°→1146과 일치). 첫 명령 전 프레임 0개. 명령 1번에 프레임 1개씩 |
| `/servo/state`는 마지막으로 보낸 값 | 첫 명령 전 0개, 이후 명령마다 1개. 마지막 메시지 `[F1_1, W_PITCH]` = `[2.00015, 1.00007]` rad. 보낸 값(1146, 573 count)을 되돌린 값이라 입력 2.0/1.0 rad와 0.0001 rad 정도 다르다 → 측정값이 아니라 보낸 값임을 확인 |
| 이름→id 표 = calibration.json의 name/id | 통과. 원본 SimToReal `calibration.json`으로 `--calibration` 실행(arm_control이 생기면 기본 경로로 3단계에서 다시 실행) |
| 잘못된 메시지는 통째로 버림 | vcan0 실측: 모르는 이름(BAD), 길이 불일치, NaN, 3.5 rad(200.5°) → 프레임·상태 증가 0, WARN 4개. 두 관절 중 하나만 −5°인 메시지는 `tests/check_node.py`(python-can virtual 버스)에서 첫 관절(F1_1)도 안 나가는 것을 확인. 한 곳의 WARN이 5초 스로틀이라 같은 종류의 연속 WARN은 하나만 보인다 |
- 측정 중 프레임·상태가 2배로 나온 적이 있었다. 이전 시도에서 `kill`이 `ros2 run` 래퍼만 죽여 남은 `stm_bridge_node`가 하나 더 있었기 때문이다. 정리 후에는 1배. 시험할 때 노드 중복 실행에 주의(`ros2 run`은 PID로 자식까지 죽일 것).
- 설계 메모: CAN 송신 오류(`can.CanError`)는 잡지 않는다(원본 `sim2real_node`와 같음). 필요하면 별도로 결정.

### 3단계에서 한 일 (arm_control)
- 파일 이동(원본 `sim2real/`): `calibration.py`, `motion.py`, `watch.py`, `play.py`, `planner.py`, `pick_place.py`, `calibrate.py`, `calibration.json`(**값 그대로**, `cmp`로 원본과 동일 확인), 루트 `calibrate.py`(ROS 없는 dry-run). `protocol.py`는 AK45 부분만 남기고 서보 프레임(`Frame`, `servo_frame`)은 `stm_bridge.protocol`에서 import(`exec_depend stm_bridge`).
- `node.py` → `arm_control_node.py`: 서보 직접 CAN 송신을 없애고 `/servo/command`(이름=`calibration.json`의 `name`, 모터 각도 rad) 발행. 한 틱의 서보 전부를 메시지 1개에 담는다(시작 점프 먼저, 그다음 속도제한 스텝: 원본과 같은 순서). `rate_hz`, `max_speed_deg_s`, `calibration_file` 파라미터 유지, `servo_interface`·`servo_channel`은 stm_bridge로 이동. `/object_pose`(PoseStamped) 구독은 **로그만**(yaw 계산해 출력, 로봇은 안 움직임).
- `ros_io.py`: `Ak45Topics` 유지, `ServoCan` 삭제, `ServoTopic`(/servo/command 발행) 추가. 이름 변경: `sim2real` → `arm_control`, `/sim2real/joint_command` → `/arm_control/joint_command`, `sim2real_node` → `arm_control_node`. 옛 이름은 비교용 `tests/check_split.py`와 그 주석에만 남김(의도).
- `calibrate --ros`: 서보를 CAN 직접 송신 대신 `/servo/command`로 발행, `--servo-interface`·`--servo-channel` 삭제. **calibrate를 쓸 때는 `arm_control_node`를 띄우지 않는다**(ak45_node와 stm_bridge만). README에는 5단계에서 명시.
- 테스트: `check_protocol.py`(원본과 출력이 줄 단위로 동일, 58 PASS), `check_node.py`(arm_control_node → stm_bridge_node → virtual CAN 전체 체인, 원본의 기대값 그대로: `005#09044C`, `005#0903FC`, 팔꿈치 rad), `check_split.py`, `chain_vcan.sh`+`chain_load.py`(vcan0 50 Hz 부하), `fake_ak45.py`.

### 3단계 검증 결과 (2026-11-07)
| 항목 | 결과 |
|---|---|
| 쪼개기 전/후 비교 (`tests/check_split.py --original SimToReal`) | 같은 명령 7개(유효·클립·부분·이름/길이/NaN 오류)를 900틱. **`/ak45/command` 326개 메시지 전부 이름·위치 동일**, 서보 프레임 493개 **id·순서·count 전부 동일(차이 0)** |
| 서보 왕복(deg → rad → deg → count) 0~180° | 23,601개(0.05° 간격 3601 + 무작위 20000) **버려진 값 0개**. 원본 `servo_frame` 대비 count 차이 0: 23,514개, +1: 81개, −1: 6개(0.05° 경계의 반올림). **±1 count는 기록하고 통과**(사용자 기준), 그 이상은 없음 |
| 0° / 180°(π rad) 경계 | 0 rad → `…0000`, math.pi → `…0708`, radians(180.0) → `…0708`, degrees(π)=180.0 정확히. −0.049°·180.049°는 보냄(0/180으로), −0.06°·180.06°·−1°·181°는 버림, π+4.5e-16 rad는 180으로 보냄 |
| 이름→id 표 (arm_control 기본 `calibration.json`) | 통과(`stm_bridge/tests/check_protocol.py`, 인자 없이 실행) |
| 50 Hz 부하 (`tests/chain_vcan.sh`, 실제 프로세스 3개 + vcan0) | 서보 6개·AK45 1개 이동. `/servo/command`로 발행한 서보 **686개(메시지 152개) = candump 프레임 686개**. 서보 6개의 마지막 프레임이 전부 목표값. `/servo/state` 메시지 152개. 2회 실행 결과 동일 |
| 프로세스 정리 | `pgrep -fa 'stm_bridge_node|ak45_node|arm_control_node'` 시험 전 `(none)`, 후 `(none)`, 전체 시험 뒤 count=0. (주의: pgrep 패턴 문자열이 Bash 도구의 명령 본문에 들어가면 그 래퍼가 매칭되어 1줄이 보임. 스크립트 파일로 실행하면 0) |
| 기타 | `/object_pose` 발행 → 로그 1줄, 버스 프레임 0. `calibrate --ros`(arm_control_node 없이): `005#0903E8` 점프 후 50 Hz로 `005#09044C`까지. `play_trajectory --dry-run`은 원본처럼 "calibration not verified"로 거절(종료 1). `pick_place --dry-run`은 `~/IK`가 없어 안내 메시지(원본과 같음) |
| 빌드 | `colcon build` 전체(ak45_ros2, stm_bridge, arm_control) 경고 0 |

### 3단계의 문제점·메모
| 문제 | 상태 | 해결방안 |
|---|---|---|
| `calibrate --ros` 종료 시 `terminate called without an active exception`(core dump) | **원본 SimToReal에서도 같음**(원본 `calibrate --ros --servo-interface virtual`로 확인). 쪼개기와 무관 | 고치지 않음. 원인은 spin 스레드(daemon)가 도는 채로 종료하는 것으로 추정(미확인) |
| `calibrate --ros`의 첫 `/servo/command`는 발행자 생성 직후 보내면 디스커버리 전이라 사라질 수 있음(원본은 직접 CAN이라 없던 문제) | 기록 | 사람이 `j`, `m`을 입력하는 속도에서는 문제 없음을 확인(3초 뒤 입력). 스크립트로 즉시 보낼 때만 주의 |
| 원본 `docker_check.sh`, `in_container.sh` 미이전 | 열림 | 이 PC에 docker 없음(검증 불가). 같은 검사는 `check_protocol/check_node/check_split/chain_vcan.sh`로 대체. 필요하면 5단계에서 jax_ros용으로 다시 쓴다 |
| `check_protocol.py`의 IK 경로 기본값 `~/IK`(없으면 SKIP) | 기록 | `--ik ~/Desktop/IK`로 돌리면 시뮬 범위 비교까지 통과(58 PASS) |
| `ak45_ros2`와 `ak45_node`가 `can0` 전용 | 기록 | 실제 노드는 vcan으로 못 시험(fake_ak45로 대체) |

### 4단계에서 한 일 (vision_node 뼈대)
- `vision_node/vision_node.py`: 파라미터 `fixed_pose:=[x, y, yaw]`(x·y는 m, **yaw는 도(°)**: `pick_place --cube x y yaw`와 같은 단위 선택, 반시계 방향)가 있을 때만 1 Hz로 `/object_pose`(PoseStamped, `frame_id=shoulder_roll_axis`)를 발행. 회전은 z축 yaw만(`x=y=0`, `z=sin(yaw/2)`, `w=cos(yaw/2)`). `fixed_pose`에는 높이가 없어 `position.z=0.0`(의미 없음, 코드·docstring에 명시). 파라미터가 없거나 숫자 3개가 아니거나(길이 2·4, NaN, inf, 문자열) 하면 **발행자도 만들지 않고** 로그만 남긴다. 정수 목록 `[1, 2, -90]`은 숫자이므로 받는다. 카메라(SteroDepthSeg)는 연결하지 않음. `dynamic_typing`으로 선언(기본값 `[]`은 BYTE_ARRAY로 추정되어 `[0.1, -0.3, 20.0]`도 거절되던 것을 시험에서 발견해 고침).
- 검증 `tests/check_node.py`(실제 `ros2 run`, 17개 통과): 파라미터 없음 → 4초간 0개, 발행자도 0개 / 잘못된 값 5종 → 0개 / `[0.10, -0.30, 20.0]` → 6.5초에 6개, 간격 1.0/0.999/1.0/1.0/1.0 s, `frame_id`, x·y, z=0, yaw 20.000000°, 단위 쿼터니언, stamp. `arm_control_node`의 `/object_pose` 로그 수신은 3단계에서 확인.
- 의존성: `rclpy`, `geometry_msgs`.

### calibrate --ros 변경 (원본 SimToReal 대비, 사용자 요청)
- **이유:** 원본은 서보 프레임을 CAN에 직접 썼지만, 쪼갠 뒤에는 `/servo/command`로 발행한다. 발행자를 만든 직후(ROS 디스커버리 전)에 보낸 메시지는 구독자에게 도착하지 않고, 서보의 첫 명령(점프)은 한 번만 가므로 **첫 명령이 사라진다.** 수정 전 코드로 재현: `j W_PITCH / m 100 / m +10`을 즉시 파이프하면 첫 프레임 `005#0903E8`이 사라지고 램프의 둘째 프레임 `005#0903EE`부터 보였다(17프레임). 사람이 천천히 입력하면 드러나지 않던 문제.
- **변경:** `RosOutput.wait_for_stm_bridge()` — 서보 명령을 보내기 전에 `/servo/command` 구독자가 1개 이상 보일 때까지 기다린다(최대 `SERVO_WAIT_S=5` 초, 상수이며 CLI 옵션은 추가하지 않음). 안 보이면 `RuntimeError("no subscriber on /servo/command after 5 s (is stm_bridge running?); nothing sent")`로 거절하고 아무것도 보내지 않으며, 그 관절의 마지막 명령으로도 기록하지 않는다(`main`의 예외 처리에 `RuntimeError` 추가). AK45 관절은 기다리지 않는다(피드백이 없으면 원래대로 "not moving").
- **검증:** `tests/check_calibrate_wait.py`(구독자 없음 → 1.00 s 뒤 거절·기록 안 됨, 0.6 s 뒤 구독자가 생기면 첫 명령이 도착, 이미 있으면 대기 없음). `tests/calibrate_vcan.sh`(실제 프로세스, vcan0): 수정 후 즉시 파이프해도 첫 프레임 `005#0903E8`, 마지막 `005#09044C`, 18프레임. stm_bridge 없음 → 5초 뒤 거절, 프레임 0, 이동으로 표시 안 됨. stm_bridge 없이 AK45(elbow)는 정상 이동.

### 4단계의 문제점·메모
| 문제 | 상태 | 해결방안 |
|---|---|---|
| `calibrate --ros` 종료 시 `terminate called without an active exception`(core dump) | **원인 확인**(원본에도 있음) | spin 스레드가 도는 채로 `rclpy.shutdown()`을 부르면 abort. `check_calibrate_wait.py`에서 `executor.shutdown()` + 스레드 join을 먼저 하니 종료 코드 0으로 사라짐. `calibrate.py` 본체는 쪼개기와 무관해 고치지 않음 — 원하면 같은 방식으로 고친다 |
| `stm_bridge_node`, `arm_control_node`가 SIGTERM으로 끝날 때 `ExternalShutdownException` traceback | 기록만(`sim2real_node`와 같은 코드) | 실측: 두 노드 2건, `vision_node` 0건(새 코드라 처리함). Ctrl+C·launch의 SIGINT에서는 `KeyboardInterrupt`로 정상. 필요하면 두 노드도 같은 한 줄로 고친다 |
| `fixed_pose` yaw 단위(도)와 `position.z=0.0` | 결정 필요 | NODES.md에 단위가 있으면 5단계에서 맞춘다 |
| 테스트 중 `kill -- -PID`·`pkill -f`·`pgrep -f` 주의 | 메모 | 패턴 문자열이 같은 Bash 호출 본문에 있으면 도구 래퍼가 매칭됨. 시험은 스크립트 파일로 실행 |

### 전체 회귀 (2026-11-07, 4개 패키지 새로 빌드: 경고 0)
stm_bridge `check_protocol`·`check_node`, arm_control `check_protocol`(58 PASS, 원본과 출력 동일)·`check_node`·`check_split`·`check_calibrate_wait`·`chain_vcan.sh`(686 = 686)·`calibrate_vcan.sh`, vision_node `check_node` 전부 통과. `pgrep -fa 'stm_bridge_node|ak45_node|arm_control_node|vision_node'`는 `(none)`.
