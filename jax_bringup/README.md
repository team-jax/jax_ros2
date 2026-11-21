# jax_bringup — 팔 전체 켜기

명령 4개와 설정 파일 1개.

```bash
# 처음 한 번, 그리고 코드나 jax.yaml을 고친 뒤
cd ~/ros2_ws && colcon build && source install/setup.bash

ros2 run jax_bringup up                          # 1. 켜기 (카메라). 이전 노드 정리 + can0 재시작
ros2 run jax_bringup up "[0.10, -0.30, 20.0]"    #    또는 카메라 없이 고정 위치 [x m, y m, yaw 도]
ros2 run jax_bringup zero                        # 2. 다른 터미널: 시작 자세(시뮬 0)  ← 팔이 움직임
ros2 run jax_bringup start                       # 3. 접근 → 쥐기 → 들기 → ㄴ자      ← 팔이 움직임
ros2 run jax_bringup stop                        #    언제든 멈춤
```

- `up` 뒤에 `force:=true`(보정 미검증이어도 시작), `grasp:=false`(접근만), `result_json:=...`(카메라 파일)를 붙일 수 있다.
- **손가락 시험** (압력센서 없이, 고정 위치): `ros2 run jax_bringup up "[0.10, -0.30, 20.0]" force:=true fingers:=true` → `zero` → `start`.
  접근이 끝나면 팔은 그대로 두고 손가락만 닫고(약 3.5 s, 첫 관절 30°·엄지 0°·끝마디 60°까지, 압력이 있으면 닿는 곳에서 멈춤) 2 s 쥔 뒤 다시 편다.
  단계: `INSERT FINGERS CLOSED OPEN DONE`. 손가락 서보는 `calibration.json`에서 영점 90°, 부호 +1이 **미검증**이니, 처음에는 손을 가까이 두고 방향을 본다.
- 다시 켜려면 `up`을 한 번 더 실행한다. 앞의 것은 알아서 꺼진다(이때 토크가 풀리니 팔을 받친다).
- 카메라 프로그램(SteroDepthSeg)은 따로 켠다:
  `cd ~/Desktop/StereoDepthSeg && ./venv_ffs/bin/python z_phone_stereo.py --physical 2 5 --calib-stereo calib_phone_pair.json --yolo seg --rgbd-viewer --out phone_stereo/runs/my_view`

## 설정 위치

| 무엇 | 파일 |
|---|---|
| 카메라 결과 파일, 속도 상한, 쥐기 시간 등 자주 바꾸는 값 | `jax_bringup/config/jax.yaml` |
| 카메라 → 로봇 변환 `camera_to_robot` | `vision_node/config/camera.yaml` |
| AK45 온도·각도 상한 | `ak45_ros2/config/ak45_node.yaml` |
| 관절 영점·부호·범위 | `arm_control/calibration.json` |

## 뜨는 노드 (5개)

`ak45_node`, `stm_bridge`, `arm_control_node`, `vision_node`, `approach_node`. 확인: `ros2 node list`.

## 진행 단계 (`start`가 출력)

`HOLD_FIRST RAISE SETTLE INSERT GRASP RETURN DONE`. 끝이 `HOLD`면 압력 토픽(`/pressure`)이 없어 쥐지 않은 것, `GRASP_TIMEOUT`은 10 s 안에 못 든 것, `STOPPED`는 멈춘 것(이유는 `up` 터미널 로그).

예전 방식(`tools/bringup.sh`, `tools/go_zero.sh`)은 이 명령들로 넘겨준다. 자세한 설명은 레포 루트 `BRINGUP.md`.
