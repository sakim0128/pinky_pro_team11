# Live 웹 상태 조회 (첫 PR)

이 화면은 ROS 상태를 구독만 합니다. 브라우저를 열거나 닫아도 주행 명령,
heartbeat, 제어 세션을 발행하지 않습니다. bridge/coordinator/카메라도 실행하지 않습니다.
모터 제어와 연결 감시는 기존 백엔드가 담당합니다.

## 빌드와 실행

ROS Jazzy 및 colcon 환경에서, 이 저장소 루트에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths pinky_fleet_msgs pinky_lane_msgs pinky_fleet_station --packages-select pinky_fleet_msgs pinky_lane_msgs pinky_fleet_station
source install/setup.bash
ROS_DOMAIN_ID=0 ros2 launch pinky_fleet_station live_web.launch.xml
```

같은 PC에서 http://127.0.0.1:8080 에 접속합니다. 다른 PC에 공개하려면
`host:=0.0.0.0`을 launch 인수로 주고 `http://관제PC주소:8080`으로 접속합니다.
포트는 `port:=8081` 등으로 변경합니다. 사설 네트워크 조회용이며 인증은 제공하지 않습니다.
이미 실행 중인 실기 bridge와 lane coordinator를 사용합니다. 이 launch에는 fake 노드가 없습니다.
Ctrl+C로 웹만 종료하며 로봇 제어 상태를 변경하지 않습니다.

로봇 이름/timeout 또는 토픽 remap을 바꿀 때:

```bash
ROS_DOMAIN_ID=0 ros2 run pinky_fleet_station live_web_node --ros-args -p robot_names:="['pinky1', 'pinky2']" -p state_timeout:=2.0 -p port:=8080
```

## API와 화면 매핑

`GET /api/state`: schema_version=1, mode=live, read_only=true.
각 스트림은 `{status, age_seconds, data}` 형식이며 status는 missing/live/stale입니다.
`age_seconds`는 서버의 monotonic 수신 시간 기준입니다. ROS stamp를 PC 시계에서 빼지 않습니다.
원본 header stamp/frame_id는 data.header에 보존합니다. NaN/Infinity는 재귀적으로 null 처리합니다.

| 화면/API | ROS 원본 | 의미 |
|---|---|---|
| robots[].state | /<robot>/state, RobotState | 원본 필드와 header |
| 현재 위치/방향 | x,y,yaw + localized | TF 기반 위치. localized=false면 위치는 — |
| AMCL 위치/불확실성 | /<robot>/amcl_pose, PoseWithCovarianceStamped | x/y/yaw와 covariance 대각 성분의 표준편차. 정지 중 stale일 수 있음 |
| 실제 속도 | RobotState.linear_velocity/angular_velocity | odom 기반 측정; LaneStatus 제어 출력과 구분 |
| robots[].lane | /<robot>/lane_status, LaneStatus | 원본 필드와 header |
| 주행 상태/이유 | drive_state/state_reason | 차선 주행 상태, stale이면 현재 상태로 표시하지 않음 |
| mission | /fleet/lane/status, std_msgs/String | JSON mission, warning, robots |
| health | 전체 스트림 수신 경과 시간 | 모두 live: online, 모두 missing: waiting, 기타: degraded |

QoS는 기존 상태 발행자와 맞춘 RELIABLE/VOLATILE, depth=10입니다.
원본 JSON 미션에 mission 문자열이 없거나 잘못된 JSON이면 샘플을 무시하고 마지막 정상 수신의 age를 유지합니다.
로봇 state, lane status, 미션은 각각 독립적으로 timeout(기본 2초)을 적용합니다.
AMCL은 정지 중 새 추정치를 발행하지 않을 수 있으므로 전체 health에는 포함하지 않고 AMCL 행에서만
독립 freshness를 표시합니다.
화면은 500ms 간격으로 요청하며 요청을 겹치지 않습니다. HTTP timeout은 2.5초입니다.
서버 연결 실패 시 이전 카드를 흐리게 하고 마지막 수신값임을 알립니다.
누락값을 가짜 데이터로 채우지 않으며 제어 API는 없습니다. POST 요청은 405입니다.

`GET /api/map/metadata`와 `GET /api/map/image.png`는 launch의 `map_yaml`(기본 map5)에서
읽은 Nav2 지도만 제공합니다. PGM은 브라우저 호환 PNG로 손실 없이 변환합니다. map5의
world 좌표는 `origin + resolution × cell`이고 이미지 Y축은 아래 방향이므로, 화면의 Y는
`100% - world Y`로 변환합니다. live `RobotState`가 `localized=true` 및 `header.frame_id=map`
이고 map 규격(width/height/resolution/origin)이 map5와 전부 일치할 때에만 map5에 마커를
표시합니다. 같은 조건에서 live AMCL covariance의 XY 2σ 타원과 `goal_valid` Nav2 목표(G)를
표시합니다. Figma 차선 레이어는 별도로 전환할 수 있으나 좌표 정렬이 완료될 때까지 로봇
마커를 표시하지 않습니다.

전방 카메라는 `/pinky1/camera/image/compressed`, `/pinky2/camera/image/compressed`의 JPEG만
구독합니다. 매 로봇의 최신 유효 JPEG 한 장(최대 2 MB)만 유지하며,
`GET /api/camera/<robot>.jpg`로 제공합니다. `/api/state`의 `cameras.<robot>`은 frame의
`missing/live/stale` 상태와 수신 경과 시간을 제공합니다. 이 노드는 카메라를 실행하거나
카메라 토픽을 발행하지 않습니다.

상부 카메라 프레임은 기본 `/overhead/camera/image/compressed` JPEG 토픽을 사용하며 launch의
`overhead_camera_topic`으로 변경할 수 있습니다. 외부 카메라 기반 위치는 로봇별
`/<robot>/overhead_pose` `geometry_msgs/PoseStamped`를 사용합니다. `frame_id=map`이고 live인
값만 `P′`로 표시합니다.

상부 카메라에서 직접 P′를 만들려면 [상부 ArUco 추적 연결](overhead_tracker.md)을 사용합니다.

전방 카메라 카드의 보조 정보는 별도 영상 추론 결과를 만들지 않고 기존
`/<robot>/lane_status`의 `lane_quality`, `error_x_norm`, `drive_state`, `lidar_min_range`,
`state_reason`을 그대로 표시합니다. 따라서 카드의 장애물·횡단보도·차선 미검출 표시는
lane driver가 실제로 보고한 상태입니다.

## 제어 활성화

제어 기능은 기본으로 꺼져 있습니다. 실기에서만 다음처럼 명시해 활성화합니다.

```bash
ROS_DOMAIN_ID=0 ros2 launch pinky_fleet_station live_web.launch.xml \
  enable_control:=true
```

주행 시작·일시정지·재시작은 lane coordinator의 `/fleet/lane/control`에 `start`·`stop`·`resume`을
전달합니다. 초기화는 주행을 stop한 다음 각 로봇의 AMCL `initialpose`를 설정합니다. 기본 초기
위치는 map5에서 Pinky1 `(0.11, 1.08, 0°)`, Pinky2 `(0.16, 0.74, -90°)`입니다. 따라서 로봇을
그 물리 위치와 방향에 직접 놓은 뒤에만 초기화를 사용합니다.

## 재사용과 범위

team10 웹의 Python 표준 HTTP + ROS 상태 직렬화 접근을 사용하되, 기존 web_node의
명령/세션/heartbeat 동작은 이식하지 않았습니다. HTTP는 별도 스레드, 상태 저장소는
lock과 snapshot 복사로 ROS와 분리했습니다. 정적 리소스는 패키지에 포함됩니다.
참고 저장소의 영상 코드는 다음 카메라 PR에서 검토합니다. 이번에는 복사하지 않았습니다.
Figma 최종 디자인, 지도, 공분산, 영상, 제어는 다음 PR 범위입니다.
