# relay_station — 중계 관제국 (rkd1rjs2 팀원 몫)

`relay_station/` 은 rkd1rjs2/robot_mini_project_pinky 의 중계 관제국을 이 저장소 구조로 옮긴 것이다.
**colcon 패키지가 아니다**(`relay_station/COLCON_IGNORE`) — `colcon build` 는 건너뛴다. 순수 Python·셸이고,
실행에는 ROS 2 Jazzy 와 이 저장소의 `pinky_fleet_msgs` · `pinky_lane_msgs` · `pinky_lane_station`(도로망 `road_graph`
구현은 그 패키지 하나다)이 필요하다. 파일 안내와 옮기며 바꾼 것은 [`relay_station/README.md`](../../relay_station/README.md).

## 2026-09-29 팀 결정 — 코디네이터 하나 · 도로망 하나 · 이름 하나 · 지도 map5

| | 전 | 후 |
| :-- | :-- | :-- |
| 코디네이터(경로 배정·구간 예약·`LaneCommand`) | 관제 PC `lane_coordinator_node` 와 중계 `fleet_coordinator` 둘 | **중계 `relay_station/fleet/fleet_coordinator.py` 하나** — `pinky_lane_station` 의 코디네이터·`reservation.py` 는 지웠다 |
| 도로망 구현 · 파일 | `road_graph.py` 두 벌, `road_graph.yaml` 셋(팀11·relay legacy·relay map4) | **`pinky_lane_station.road_graph` 하나**, 파일은 `pinky_lane_station/config/road_graph.yaml` 하나 — relay 프로파일이 상대경로로 읽는다 |
| 로봇 이름 | 토픽 `/robotN/*` · id `robotN` · API `/api/robotN/*` 와 `pinkyN` 이 섞임 | **`pinky1` · `pinky2`** 하나(별칭 표 없음) |
| 로봇 쪽 | Nav2 하이브리드(`hybrid_agent_node` · `/cmd_vel` 게이트)와 레인 로봇 | **`lane_agent_node` 만**(`lane_robot.launch.xml`) — Nav2 직접 경로(`goal_pose` · `mission_cmd` · `nav_status` · 관제 teleop `cmd_vel`)는 브리지·게이트웨이·화면에서 지웠다 |
| 지도 | live 웹·상부 추적기 map5, 중계 프로파일 map4/legacy | **map5** 하나 — 프로파일 `team11_map5`(로봇 지도 이름 `map5`) |

같은 로봇에 코디네이터가 하나뿐이니 "둘 중 하나만 띄운다" 는 규칙은 사라졌다. 관제 PC 의 `lane_station.launch.xml` 은
브릿지 + `lane_pipeline_node`(차선 인식)만 띄운다.

## 도로망·미션 (임시, 2026-09-29)

`pinky_lane_station/config/road_graph.yaml` — map5 좌표(원점 좌하단 −0.01/−0.01, 2.36 × 1.28 m)의 **자리표시 4노드**:
BL(0.20, 0.20) · BR(2.15, 0.20) · TR(2.15, 1.08) endpoint, J(1.18, 0.64) junction, 엣지 BL_J · J_TR · J_BR(직선).
**새 맵 위에서 `graph_editor` 로 다시 찍는다**(파일 머리 주석). 임시 미션은 pinky1 BL→TR, pinky2 BR→BL
(`relay_station/fleet/config/profiles/team11_map5/lane_mission.yaml`, 관제 PC 의 `pinky_lane_station/config/lane_mission.yaml` 도 같은 값 —
후자는 파이프라인·가짜 로봇의 로봇 목록일 뿐, 배정은 중계가 한다).

시작·목적지는 **중계 웹에서 고른다**: V2 설정 탭의 "미션 배정" 카드 → `POST /api/fleet/assign {robot, start, goal}`
(후보는 도로망의 endpoint 노드 — `/api/fleet/profiles` 의 `graph.endpoints`). 규칙(L3): 두 목표가 다르고, 어느 목표도
다른 로봇 경로 **안**(중간 노드)에 있지 않아야 한다. 출발 노드를 다른 로봇이 아직 쥐고 있는 것은 거절 사유가 아니다 —
그 로봇이 떠나면 놓는다(`Reservation.assign_conflict`, `relay_station/tests/test_assign_map5.py`).

## 실행 (중계 PC, 도메인 8)

```bash
# 이 저장소 루트
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pinky_fleet_msgs pinky_lane_msgs pinky_lane_station pinky_fleet_agent pinky_fleet_station
source install/setup.bash

# 0) 값 채우기 — 주소·NIC 는 전부 자리표시자다 (relay_station/README.md "실행 전에 바꿔야 하는 것")
#    relay_station/configs/{fleet_domains.env,cyclonedds.xml,video_sources.json} · FIELD_NIC
#    팀원 노트북에서 움직이는 명령을 내리려면 relay_station/configs/control_allow.json 에 그 노트북 주소를 enabled:true 로 (재기동 없음)
export RELAY_VISION_API_KEY=<공유 키>          # 비전 수신 API 를 쓸 때만. 없으면 그 API 는 전부 401

# 1) 도메인 브리지 — 로봇 10·11 <-> 관제 8 (pinky1_control · pinky2_control · team_mirror · watchdog)
relay_station/domain_bridge/install_bridge.sh          # 또는 launch_master_gateway.sh 가 같이 띄운다

# 2) 게이트웨이 웹 :8889 — 플릿 코디네이터를 안에 품는다(프로파일 team11_map5 → 도로망·미션 로드, 경로 배정)
ROS_DOMAIN_ID=8 python3 relay_station/gateway_web/gateway_web_server.py --port 8889
#    (relay_station/launch_master_gateway.sh = DDS 프로파일 선택 → 브리지 → 웹 서버)

# 3) 차선 인식 — 로봇 카메라를 받아 LanePath 를 낸다 (pinky_lane_station, 같은 도메인 8 에서)
ROS_DOMAIN_ID=8 ros2 launch pinky_lane_station lane_station.launch.xml use_bridge:=False \
    mission:=$PWD/pinky_lane_station/config/lane_mission.yaml
#    (브릿지는 1) 의 domain_bridge 가 나른다 — pinkyN_control.yaml 이 /pinkyN/camera/image/compressed(업)·/pinkyN/lane_path(다운)를
#     싣는다(2026-09-29 추가). pinky_lane_station 의 lane_bridge(도메인 0 기준)는 띄우지 않는다)

# 4) 항공뷰 — 상부 카메라 ArUco → /pinkyN/overhead_pose (pinky_fleet_station, docs/integration/overhead_tracker.md)
ROS_DOMAIN_ID=8 ros2 launch pinky_fleet_station overhead_tracker.launch.xml
#    또는 태블릿이 로봇 좌표를 HTTP 로 보낸다 → 중계가 같은 /pinkyN/overhead_pose 로 낸다 (docs/integration/tablet_pose.md).
#    한 로봇에는 한 출처만 — 둘을 같은 로봇에 쓰지 않는다

# 로봇 (도메인 10 · 11) — lane_agent_node 만
ros2 launch pinky_fleet_agent lane_robot.launch.xml robot_name:=pinky1 domain_id:=10 map:=$HOME/map/map5.yaml
```

브라우저: `http://<중계 PC>:8889/fleet_control_v2.html` — 설정 탭에서 **① 좌표 프로파일(team11_map5) → ② 로봇 지도 전환(map5) →
③ 초기 위치 → ④ "미션 배정"(시작·목적지)** 뒤 주행 시작. 로봇별 정지·재개는 로봇 카드(`/api/pinkyN/stop` · `/api/pinkyN/resume`).
`http://<중계 PC>:18081` 로 열면 **보기 전용**(움직이는 조작이 숨는다 · `/api/status` 의 `view_only`).

**팀원 노트북에서 움직이는 명령**(2026-09-28 저녁): 기본은 중계 PC 자신만 움직이는 명령(배정·재개·좌표 전환·플릿 start/resume)을
낸다. `relay_station/configs/control_allow.json` 에 노트북 LAN 주소를 `enabled: true` 로 넣으면 그 노트북의 `:8889` 화면에서도 낼 수 있다 —
**한 번에 한 사람**(첫 명령이 제어권, 다른 허용 노트북은 409 와 노랑 알약, 30 s 무응답이면 만료) · 중계 PC 콘솔은 언제나 제어권을 가져온다 ·
멈추는 명령(일시정지·비상정지·로봇 정지)은 누구든. 규칙·접점·시험: [`control_policy.md`](control_policy.md).

## 영상 공유

중계는 카메라 소스를 모아 브라우저에 다시 내보낸다(MJPEG). 폰·태블릿의 카메라 앱, 로컬 웹캠, 로봇 카메라 토픽(`/pinkyN/camera/image_raw/compressed`)이 소스가 된다.

| | |
| :-- | :-- |
| `GET /api/sources` | 소스 목록(id · 종류 · 최근 프레임 시각 · 수신 fps) — 로봇 소스 id 는 `pinky1` · `pinky2` |
| `GET /video_feed?src=<id>` | 그 소스의 MJPEG 스트림 (`/video_feed` 만 주면 기본 소스) |
| `GET /robot_camera_feed?id=pinky1` | 로봇 온보드 카메라 |
| `GET /` (`index.html`) | 멀티뷰 — 소스 타일을 사람이 연다(기본은 전부 닫힘) |
| 소스 설정 | `relay_station/configs/video_sources.json` (당겨올 URL, 후보 주소, 발견 규칙) · `MCV_HOST_CAMERA_URL` |

폰 카메라 앱은 이 저장소에 없다(원 저장소 `packaging/android-app`, APK 는 손으로 전달). 앱은 `:18086` 에서 MJPEG 를 내고
중계가 당겨온다 — 폰과 중계 PC 가 **같은 네트워크**에 있어야 한다.

## 시험

```bash
cd relay_station && python3 -m pytest tests -q        # numpy · opencv-python · psutil · pyyaml 필요, install/setup.bash 뒤
```

rclpy 없이도 도는 것: 도로망·배정(`test_assign_map5.py` · `test_assign_ui.py` · `test_junction_first_come.py`), 브리지 설정
(`test_bridge_two_sets.py` · `test_topic_wrap.py` · `test_control_topic_naming.py`), 화면 정직성·제어권 정책 등. 코디네이터·게이트웨이를
실제로 부르는 것(`test_d7_robot_stop.py` · `test_fleet_profiles.py` · `test_review_0926_*.py`)은 ROS 2 Jazzy 컨테이너에서.
`tests/test_team11_export.py` 가 공개 규칙(키 기본값 없음 · 키 없으면 거절 · 사설 주소는 자리표시자뿐 · 개인 경로 없음 · 외부 사이트 낱말 없음 ·
COLCON_IGNORE)을 잠근다. 옛 map4 도로망은 예약 규칙 회귀 시험의 **픽스처**(`tests/fixtures/map4_road_graph.yaml`)로만 남아 있다.

## 다시 옮길 때

원 저장소 main 이 바뀌면 같은 규칙으로 다시 내보낸다(내보내기 도구는 원 저장소에 있다 — 실제 주소 치환표를 담아 공개하지 않는다).
기준 커밋은 `relay_station/README.md` 첫 줄과 [`source_versions.md`](source_versions.md) 에 적는다. 2026-09-29 의 통일(코디네이터·도로망·
이름·map5)은 이 저장소 쪽 변경이라 다시 옮길 때 원 저장소의 `robotN` 이름·Nav2 경로를 되살리지 않도록 본다.
