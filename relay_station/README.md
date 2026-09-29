# relay_station — 중계 관제국 (웹 관제 · 플릿 코디네이터 · 도메인 브리지)

rkd1rjs2/robot_mini_project_pinky 의 `relay_station/` 을 이 레포 구조로 옮긴 것이다(원 저장소 main `f8fc97d` + 제어권 정책 `1eed3f8`, 2026-09-28 저녁).
**colcon 패키지가 아니다** — `COLCON_IGNORE` 가 있어 `colcon build` 는 이 폴더를 건너뛴다. 순수 Python 과 셸이며,
실행에는 ROS 2 Jazzy 와 이 레포의 `pinky_fleet_msgs` · `pinky_lane_msgs` · **`pinky_lane_station`**(도로망 `road_graph` 구현은
그 패키지 하나다 — `fleet/` 가 `pinky_lane_station.road_graph` 를 import 한다)이 필요하다. 셋 다 `colcon build` 뒤 `install/setup.bash` 로 준다
(소스 체크아웃만 있으면 게이트웨이가 형제 폴더 `../pinky_lane_station` 을 `sys.path` 뒤에 붙여 도로망은 찾는다 — 메시지 패키지는 빌드가 필요하다).

## 2026-09-29 — 코디네이터 하나 · 도로망 하나 · 이름 하나 · 지도 map5

- **코디네이터는 이것 하나다.** `fleet/fleet_coordinator.py`(`RelayFleetCoordinator`)가 경로 배정·구간 예약·`LaneCommand`·`Route` 를 낸다.
  관제 PC 의 `pinky_lane_station` 에는 코디네이터가 없다(`lane_coordinator_node`·`reservation.py` 삭제) — 그쪽은 차선 인식(`lane_pipeline_node`)뿐.
- **도로망 구현·파일은 하나다.** `pinky_lane_station.road_graph`(`RoadGraph`·`Route`·`summary()`)와 `pinky_lane_station/config/road_graph.yaml`.
  `fleet/config/profiles/team11_map5/lane_mission.yaml` 이 그 파일을 상대경로로 읽는다 — 복사본이 없다.
- **로봇 이름은 `pinky1`·`pinky2` 하나다.** 토픽 `/pinkyN/*`, API `/api/pinkyN/stop|resume`, 화면 키, 브리지 설정 `pinkyN_control.yaml`,
  `fleet_domains.env` 의 `PINKYN_DOMAIN_ID`. `robotN` 별칭 표는 없다.
- **로봇은 `lane_agent_node` 만이다**(`pinky_fleet_agent/launch/lane_robot.launch.xml`). Nav2 직접 경로 — `/robotN/goal_pose`·`mission_cmd`·
  `nav_status`·관제 teleop `cmd_vel`, `/api/robotN/goal`·`/api/goal`·`/api/robotN/mission`, 클릭 목표 데스크톱 창, `scripts/control_robot1.sh` — 는
  받는 쪽이 없어 지웠다. 코디네이터의 `drive_mode: nav2` 코드는 남아 있지만 프로파일은 전부 `lane` 이다.
- **지도는 map5** (`pinky_fleet_station/config/map5.yaml`, 236×128 셀, 0.01 m/셀, 원점 −0.01/−0.01 → x 0..2.36, y 0..1.28 m).
  프로파일 `team11_map5`(frame `team11_map5`, 로봇 지도 이름 `map5`) 하나 — map4 프로파일과 옛 5노드 가상 경기장(legacy)은 지웠다.
- **시작·목적지는 웹에서 고른다.** V2 설정 탭 "미션 배정" 카드 → `POST /api/fleet/assign {robot, start, goal}`. 후보는 도로망의 endpoint
  노드(`/api/fleet/profiles` 의 `graph.endpoints`, 지금은 BL·BR·TR). 임시 미션 기본값은 pinky1 BL→TR, pinky2 BR→BL.
  배정 규칙(L3)은 `Reservation.assign_conflict` 한 곳 — ROS 없이 `tests/test_assign_map5.py` 가 잰다.

## 무엇인가

| 폴더 | 역할 |
| :--- | :--- |
| `gateway_web/` | 웹 관제 서버 `:8889` (`gateway_web_server.py`) — 로봇 상태·지도·플릿 API, 플릿 제어 화면(`static/fleet_control_v2.*`), 영상 중계 |
| `fleet/` | 플릿 코디네이터 — 도로망(`pinky_lane_station.road_graph`) 위 경로 배정, 구간 예약(`reservation`), `LaneCommand`(START·CLEARANCE·STOP·ESTOP·RESUME) · `Route` 발행, 좌표 프로파일(`profiles`) |
| `domain_bridge/` | 관제 도메인(8) ↔ 로봇 도메인(10·11) 토픽 브리지 설정·생성기·systemd 유닛 (`pinkyN_control.yaml` · `team_mirror.yaml` · 팀원 teleop 벌) |
| `configs/` | DDS 프로파일(`cyclonedds*.xml`) · 로봇 도메인 표(`fleet_domains.env`) · 영상 소스 · 경기장 · 영상 가림 정책 · **제어권 허용 목록(`control_allow.json`)** |
| `network/` · `scripts/` · `systemd/` | 현장 운용 스크립트(방화벽·SSH 터널·게이트웨이 유닛 설치·SLAM 매핑 `map_pinkyN.sh` 등) — **예시**다. 주소·경로는 자기 현장 값으로 바꿔 쓴다 |
| `tests/` | 시험(아래) |

**이 레포의 기존 관제와의 관계**: `pinky_lane_station`(차선 인식) · `pinky_fleet_station`(live 웹 `:8080`, 상부 추적기)과 **같은 메시지 계약**
(`FleetCommand` · `RobotState` · `LaneCommand` · `LaneStatus` · `Route`)을 쓰고, 셋이 **같이** 돈다 — 코디네이터는 이 중계뿐이다.
도메인 배치: 관제 = 8, 로봇 = 10·11, 사이를 `domain_bridge` 가 잇는다(`docs/integration/relay_station.md` 의 실행 순서).

## 실행 전에 바꿔야 하는 것

- **`RELAY_VISION_API_KEY`(필수, 비전 API 를 쓸 때)** — 비전 수신 API(`/api/vision/pose_fix` · `/api/vision/zone_event`)의
  공유 키. 코드에 기본값이 **없다**. 안 주면 비전 API 는 전부 401 이다(fail-closed).
- **주소는 전부 자리표시자다** — `configs/*.json|xml|env`, 스크립트, 화면 안내 문구의 IP 는 문서용 대역
  (`198.51.100.x` = 현장 LAN, `203.0.113.x` = 기타 LAN, `100.64.0.x` = tailnet)으로 바꿔 두었다. 자기 현장 값으로 채운다.
- **`FIELD_NIC`** — 현장 유선 어댑터 이름. 기본값 `enx001122334455` 는 자리표시자이고, `configs/cyclonedds.xml` 의 인터페이스
  이름도 같이 바꾼다(`tests/test_bridge_env.py` · `test_dds_profiles.py` 가 둘이 같은지 본다). 없으면 런처가 offsite 프로파일로 떨어진다.
- **`configs/control_allow.json`(선택)** — 팀원 노트북에서 움직이는 명령(배정·로봇 재개·좌표 전환·플릿 start/resume)을 내게 하려면
  그 노트북의 LAN 주소를 `enabled: true` 로 넣는다(재기동 없음). 기본 파일은 닫혀 있어 중계 PC 자신만 낸다. 한 번에 한 사람 · 30 s 무응답 만료 ·
  중계 PC 콘솔 우선 · 멈추는 명령은 누구든. `docs/integration/control_policy.md`.
- **경로** — 스크립트는 체크아웃을 `$HOME/pinky_pro/src/pinky_pro_team11`, 로봇 워크스페이스를 `$HOME/pinky_pro` 로 가정한다
  (`REPO_ROOT` 로 바꿀 수 있다).

기동 순서(중계 PC, 도메인 8): `domain_bridge/install_bridge.sh`(브리지) → `gateway_web/gateway_web_server.py --port 8889`(게이트웨이 = 코디네이터) →
`ros2 launch pinky_lane_station lane_station.launch.xml use_bridge:=False`(차선 인식) → `ros2 launch pinky_fleet_station overhead_tracker.launch.xml`(항공뷰).
`relay_station/launch_master_gateway.sh` 가 앞의 둘(DDS 프로파일 선택 → 브리지 → 웹 서버)을 한 번에 띄운다.

## 원 저장소에서 옮기며 바꾼 것

- **비전 API 키**: 원 저장소는 기본 키를 코드에 두고, 키가 비면 인증을 **통과**시켰다. 여기서는 기본값을 없애고 키가 비면 **거절**한다.
- **신원**: 현장·tailnet IP → 위 자리표시자, 개인 홈 경로 → `$HOME` 기준(systemd 는 `%h`), 어댑터 MAC 이 든 NIC 이름 → 자리표시자,
  원 저장소 운영 환경을 가리키는 낱말은 중립어("외부 사이트" · "다른 배포 체계")로. 화면(`index.html`)의 접속 주소 안내는 박힌 주소 대신
  **지금 연 주소**(`location.origin`)를 보이고 "원격(`:18081`)은 보기 전용" 한 줄만 남겼다.
- **경로**: 에이전트 `robot_onboard/pinky_fleet_agent` → `pinky_fleet_agent`, 메시지 `shared_msgs/pinky_*_msgs` → `pinky_*_msgs`,
  레포 루트 `configs/` → `relay_station/configs/`. 2026-09-29 부터 도로망은 `pinky_lane_station.road_graph`(위).
- **싣지 않은 것**: 원 저장소의 도커 복제본(외부 사이트용 운영 환경, `docker/` — 카메라 발행기 `docker/host_camera_publisher.py` 하나만 남겼다, 시험이 대사한다),
  게이트웨이 실행본 심링크 검사, 쓰지 않는 스크린샷, 에이전트 시험(→ `pinky_fleet_agent/test/`), 원 저장소 인프라·문서를 대사하는 시험,
  태블릿 비전 코드(용도가 바뀌어 싣지 않는다 — 비전 수신 API 는 남아 있지만 키 없이는 닫혀 있다).
- **시험**: 원 저장소의 다른 폴더(`robots/` · `tablet/` · `robot_onboard/pinky_navigation`)를 대사하는 것은 사유를 적어 `skip`.
  하이브리드 에이전트(`route_chain` · `drive_command_gate` · `hybrid_agent_node`)에 기대던 시험은 그 코드와 함께 지웠다(2026-09-29).
  `tests/test_team11_export.py` 가 위 규칙(키 · 주소 · 경로 · 낱말 · COLCON_IGNORE)을 잠근다.
- 주석의 `docs/*.md` 는 **원 저장소**의 설계·검수 문서를 가리킨다(이 레포에는 없다).

## 시험

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pinky_fleet_msgs pinky_lane_msgs pinky_lane_station pinky_fleet_agent && source install/setup.bash
cd <이 레포>/relay_station && python3 -m pytest tests -q        # numpy · opencv-python · psutil · pyyaml 필요
```

rclpy 없이 도는 것(이 레포 컨테이너에서 확인, 2026-09-29): 도로망·배정(`test_assign_map5.py` · `test_assign_ui.py` · `test_junction_first_come.py`),
브리지 설정(`test_bridge_two_sets.py` · `test_topic_wrap.py` · `test_control_topic_naming.py` · `test_bridge_unit_overlay.py`), 화면 정직성·제어권 정책
(`test_page_honesty.py` · `test_v2_front_honesty.py` · `test_control_policy.py` · `test_control_0928_*.py`), 내보내기 규칙(`test_team11_export.py` · `test_no_baked_addresses.py`) 등.
코디네이터·게이트웨이 객체를 실제로 부르는 것(`test_d7_robot_stop.py` · `test_fleet_profiles.py` · `test_review_0926_gw/res/ui2.py` ·
`test_review_0927_resume_retry.py` · `test_relay_contract_delta.py`)은 ROS 2 Jazzy 컨테이너(`install/setup.bash` 뒤)에서 돈다 — skip 은 통과가 아니다.
옛 map4 도로망은 예약 규칙 회귀 시험(`test_review_0926_res.py`)의 **픽스처**(`tests/fixtures/map4_road_graph.yaml`)로만 남아 있다.
