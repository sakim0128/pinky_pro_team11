# 실물 없이 돌려 보는 모의 테스트 (도커, 3단계)

로봇 · 카메라 · 태블릿이 없어도 노트북의 도커만으로 비전 미션 시나리오, 태블릿 좌표 계산, 중계 시나리오 시작까지 볼 수 있다.
아래 명령은 저장소 루트에서 실행한다. 각 단계의 통과 수치는 2026-09-30 에 ROS 2 Jazzy 컨테이너에서 직접 돌려 얻은 값이다.

| 단계 | 무엇을 보나 | 필요한 것 | 결과 |
| :-- | :-- | :-- | :-- |
| 1 | 비전 미션 시나리오 폐루프 — 가상 로봇 둘이 교차로 통행권을 주고받는다 | 도커 + `ros:jazzy-ros-base` | 34 통과 |
| 2 | 태블릿 가상 카메라 — 마커를 그려 좌표로 되돌린다 | 위 이미지 + OpenCV | 태블릿 시험 43 통과 |
| 3 | 중계 게이트웨이 `--vision` — 시나리오 시작이 코디네이터에 닿는다 | ROS 2 Jazzy + `cv_bridge` + OpenCV | 시나리오 `s1` → `RUNNING` |

## 1단계 — 비전 미션 시나리오 폐루프 (ROS 없이)

관제 코디네이터(`VisionFleetCoordinator`)와 가상 로봇 둘(`LaneDriver`)이 토픽 없이 메시지를 직접 주고받으며
교차로 진입, 빨간 선 정지, 통행권 부여(순차와 경합), 고정 동작, 차선 복귀, 흰 정지선 도착을 끝까지 돈다.

```bash
docker run --rm --entrypoint /bin/bash -v "$(pwd):/workspace" -w /workspace ros:jazzy-ros-base -c \
  "pytest relay_station/tests/test_vision_mode.py pinky_lane_station/test/test_vision_mission.py \
          pinky_fleet_agent/test/test_vision_driver.py -q -p no:cacheprovider"
```

기대: `34 passed`, skip 없음.

**`--entrypoint /bin/bash` 가 핵심이다.** 이 이미지는 기본으로 ROS 를 불러온다. ROS 가 불러와져 있으면 시나리오 폐루프 시험 둘
(`test_vision_coordinator_scenario1_*` · `scenario2_*`)은 가짜 노드 전용이라 **건너뛴다**(32 통과, 2 skip). 폐루프를 실제로 돌리려면
ROS 를 불러오지 않는다. 그래서 이 단계에는 `colcon build` 가 필요 없다.

## 2단계 — 태블릿 가상 카메라

`tablet/vision/vision_core/synthetic_camera.py` 가 모서리 마커 40–43 과 로봇 마커 30 · 31 을 도로망 노드 위에 그리고,
`OverheadLocalizer` 가 그 그림에서 마커를 찾아 좌표로 되돌린다. 카메라도 로봇도 필요 없다.

```bash
# OpenCV(aruco 포함)가 있는 이미지에서. pinky1 이 BL → TR 을 0.15 m/s 로 가는 모의 프레임 한 장
docker run --rm --entrypoint /bin/bash -v "$(pwd):/repo" -w /repo <OpenCV 가 있는 ROS 이미지> -c \
  "python3 -m tablet.vision.runtime.entrypoint --mock --mock-motion 0.15 --mock-path BL,TR --once"

# 태블릿 시험 전체
docker run --rm --entrypoint /bin/bash -v "$(pwd):/repo" -w /repo <OpenCV 가 있는 ROS 이미지> -c \
  "python3 -m pytest tablet/vision/tests -q -p no:cacheprovider"
```

기대: 종료 코드 0, 로그에 `Mock poses from road graph: start={'pinky1': (0.2, 0.2, 0.0), 'pinky2': (2.15, 0.2, 0.0)} ... path=BL,TR`,
시험은 `43 passed`. 시작 좌표 BL · BR 은 팀 도로망 정본 `pinky_lane_station/config/road_graph.yaml` 에서 온다.

**OpenCV.** 순정 `ros:jazzy-ros-base` 에는 `cv2` 가 없다. `opencv-python-headless` 나 `python3-opencv` 로 aruco 가 들어 있는 OpenCV 를 넣는다.
이 확인은 `opencv-python-headless` 4.10 휠로 했다. `apt-get install python3-opencv` 는 확인하지 못했다.

**모의 좌표는 중계로 나가지 않는다.** 설정 세 파일이 `FIELD_CONFIG_PENDING` 인 동안은 송신이 막혀 있고
(`Relay egress is DISABLED (fail-closed)`), 모의 모드에서는 그것과 별개로 `--mock-egress` 를 줘야 나간다. 실수로 가짜 좌표가
실제 중계에 들어가지 않게 한 안전장치다. 진짜로 보내 보려면 `--allow-pending-config --mock-egress --relay-url http://<중계 PC>:8889` 를 함께 준다.

## 3단계 — 중계 게이트웨이 `--vision`

```bash
colcon build --packages-select pinky_fleet_msgs pinky_lane_msgs && source install/setup.bash
python3 relay_station/gateway_web/gateway_web_server.py --port 8889 --no-camera --vision
```

- 키(`RELAY_VISION_API_KEY`)는 필요 없다. 태블릿 좌표 입구(`/api/vision/pose_fix`)에만 쓰이고, 키 없이도 게이트웨이와 시나리오 시작은 된다.
- 브라우저: `http://localhost:8889/fleet_control_v2.html` → **미션 / 시나리오** 탭 → "시나리오 1 시작" · "시나리오 2 시작".
  `/` 는 옛 화면(`index.html`)이다 — V2 가 아니다.
- 화면 대신 명령줄로:

  ```bash
  curl -s -X POST -H 'Content-Type: application/json' -d '{"name":"s1"}' http://127.0.0.1:8889/api/fleet/scenario
  # → {"applied": true, "mission_state": "RUNNING", "success": true, ...}   모르는 이름은 UNKNOWN_SCENARIO (있는 것: s1, s2)
  curl -s http://127.0.0.1:8889/api/fleet/status     # mode: vision, vision.active: s1
  ```

- **로봇이 없으면 `RUNNING` 이 되어도 진행하지 않는다.** 코디네이터는 로봇의 응답(`LaneStatus`)을 기다리며 START 를 다시 보낼 뿐이다.
  시나리오가 끝까지 도는 모습은 1단계가 보여 준다. 3단계는 화면 · 요청 · 상태 표시가 이어지는지를 본다.
- 필요한 것: ROS 2 Jazzy, `cv_bridge`(순정 `ros:jazzy-ros-base` 에는 없다 — 게이트웨이가 시작할 때 불러온다), OpenCV, numpy, psutil, PyYAML.
  `cv_bridge` 는 `ros-jazzy-cv-bridge` 로 넣는다. 이 저장소를 확인한 환경에서는 `cv_bridge` 를 대체 모듈로 두고 돌렸고, apt 설치는 확인하지 못했다.
- 컨테이너에서 띄우고 호스트 브라우저로 접속하면(`-p 8889:8889`) 게이트웨이는 컨테이너 기본 경로의 게이트웨이 주소를 로컬로 본다
  (`gateway_web_server.py` 의 `_container_gateway_ips`). 그래서 시나리오 시작이 통과하게 되어 있지만, 브라우저 포트 포워딩으로는 확인하지 못했다.
  403 이 나오면 [`control_policy.md`](control_policy.md) 를 본다.

## 자주 걸리는 곳

| 증상 | 원인 | 할 일 |
| :-- | :-- | :-- |
| 1단계가 `32 passed, 2 skipped` | ROS 가 불러와져 있다 | `--entrypoint /bin/bash` 로 ROS 를 불러오지 않는다 |
| 2단계 `No module named 'cv2'` | 이미지에 OpenCV 가 없다 | OpenCV(aruco 포함)를 넣는다 |
| 3단계 `No module named 'cv_bridge'` | 이미지에 없다 | `ros-jazzy-cv-bridge` 를 넣는다 |
| 3단계 `/` 에 V2 가 아닌 옛 화면 | 주소 | `/fleet_control_v2.html` |
| 시나리오 시작이 `NOT_VISION_MODE` | `--vision` 없이 띄움 | 게이트웨이를 `--vision` 으로 다시 띄운다 |
| 시나리오 시작이 403 | 제어권 정책 — 로컬이 아니거나 허용 목록 밖 | [`control_policy.md`](control_policy.md) |
| 시험이 `JunctionPlan` 을 못 찾는다 | 메시지 빌드가 옛것 | `build/ install/ log/` 를 지우고 `colcon build` 를 다시 |
