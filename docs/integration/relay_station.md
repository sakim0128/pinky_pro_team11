# relay_station — 중계 브리지 (rkd1rjs2 팀원 몫)

`relay_station/` 은 rkd1rjs2/robot_mini_project_pinky 의 중계 관제국을 이 저장소 구조로 옮긴 것이다.
**colcon 패키지가 아니다**(`relay_station/COLCON_IGNORE`) — `colcon build` 는 건너뛴다. 순수 Python·셸이고,
실행에는 ROS 2 Jazzy 와 이 저장소의 `pinky_fleet_msgs` · `pinky_lane_msgs` 가 필요하다. 파일 안내와 옮기며 바꾼 것은
[`relay_station/README.md`](../../relay_station/README.md).

**2026-09-29 개편**: 주 대시보드는 팀11 live 웹(`pinky_fleet_station/live_web_node`, `:8080`)이다. 중계는 화면을 버리고
**도메인 브리지 · 플릿 코디네이터 · 제어 문(제어권) · 영상 중계** 네 가지만 한다. 중계 PC 에서 live 웹을 **도메인 8** 로 띄우면
live 웹의 지도·로봇·카메라·미션 버튼이 그대로 중계 코디네이터와 이어진다(같은 토픽 계약).

## 역할

| 역할 | 누가 | 어디 |
| :-- | :-- | :-- |
| 주 대시보드 — 지도 · 로봇 위치 · 전방/상부 카메라 · 미션 버튼 · 속도 상한 | 팀11 live 웹 | `:8080` · `relay_station/launch_live_web.sh` |
| 도메인 브리지 — 로봇 10·11 ↔ 관제 8 | 중계 | `relay_station/domain_bridge/` |
| 플릿 코디네이터 — 도로망 · 구간 예약 · `LaneCommand` · `Route` | 중계 | `relay_station/fleet/` (`/fleet/lane/control` · `/fleet/lane/status`) |
| 제어 문 — 누가 움직이는 명령을 낼 수 있나 | 중계 | `:8889` · [`control_policy.md`](control_policy.md) |
| 중계 콘솔 — 버스 상태 · 제어권 · 좌표 프로파일 ①②③ · 멈춤 · 폰 영상 | 중계 | `http://<중계 PC>:8889/` (`relay_console.html`) |
| 영상 중계 — 폰 카메라 앱 · 로컬 캠 | 중계 | `/api/sources` · `/video_feed?src=` |

팀11 레인 관제(`pinky_lane_station`)와는 **같은 메시지 계약**을 쓰는 다른 코디네이터다. **같은 로봇에 두 코디네이터를 동시에
붙이지 않는다**(둘 다 `/pinkyN/lane_command` · `/pinkyN/route` 를 낸다). live 웹은 어느 쪽과도 같이 뜬다.

## 실행

```bash
# 이 저장소 루트
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pinky_fleet_msgs pinky_lane_msgs pinky_fleet_agent pinky_fleet_station && source install/setup.bash

# 1) 값 채우기 — 주소·NIC 는 전부 자리표시자다 (relay_station/README.md "실행 전에 바꿔야 하는 것")
#    relay_station/configs/{fleet_domains.env,cyclonedds.xml,video_sources.json} · FIELD_NIC
#    팀원 노트북에서 움직이는 명령을 내리려면 relay_station/configs/control_allow.json 에 그 노트북 주소를 enabled:true 로 (재기동 없음)
export RELAY_VISION_API_KEY=<공유 키>          # 외부 비전 PoseFix 를 받을 때만. 없으면 그 API 는 401

# 2) 중계 PC (도메인 8)
relay_station/launch_master_gateway.sh          # DDS 프로파일 선택 → 브리지 → 게이트웨이 :8889 (코디네이터 포함)
relay_station/launch_live_web.sh                # 주 대시보드 :8080 — 지도 = 지금 고른 중계 좌표 프로파일의 지도

# 3) 로봇 (도메인 10 · 11)
ros2 launch pinky_fleet_agent hybrid_robot.launch.xml robot_name:=pinky1 domain_id:=10 map_name:=map4
#    전방 카메라(camera_node)는 use_camera:=True 가 기본 — live 웹 전방 카메라 타일이 이것을 본다
```

| 누가 | 여는 곳 | 할 수 있는 것 |
| :-- | :-- | :-- |
| 누구나(같은 Wi-Fi) | `http://<중계 PC>:8080/` | 보기 — 지도 · 로봇 · 카메라 · 미션 상태 |
| 중계 PC | `http://localhost:8889/` (콘솔) | 전부 — 주행 시작/재시작 · ①②③ · 로봇 재개 · 멈춤 · 제어권 |
| 허용된 팀원 노트북 | `http://<중계 PC>:8889/` | 움직이는 명령(한 번에 한 사람) + 멈춤 |
| 그 밖의 주소 · `:18081` | `:8889` · `:18081` | 보기 전용 콘솔 — 멈춤만 |

live 웹의 미션 버튼은 ROS 로 바로 발행해 제어 문을 지나지 않는다. 그래서 `launch_live_web.sh` 는 기본으로 버튼을 끄고
(`LIVE_WEB_CONTROL=true` 로 켬), 켜는 것은 중계 PC 자신이 쓸 때만이다 — 팀원의 움직이는 명령은 `:8889` 로.

## 계약 (live 웹 ↔ 중계)

| 토픽 | 방향 | 누가 내나 |
| :-- | :-- | :-- |
| `/pinkyN/state` · `/pinkyN/lane_status` | 로봇 → 8 | 에이전트 (브리지) |
| `/pinkyN/amcl_pose` (PoseWithCovarianceStamped) | 로봇 → 8 | `hybrid_agent_node` 가 `/amcl_pose` 를 다시 낸다 (브리지) |
| `/pinkyN/camera/image/compressed` | 로봇 → 8 | `camera_node` (브리지, best_effort depth 1) |
| `/pinkyN/overhead_pose` · 상부 카메라 | 중계 PC | 상부 추적기(`overhead_tracker_node`)를 도메인 8 에서 |
| `/fleet/lane/control` `{cmd: start·stop·resume·estop·assign·reset}` | 8 | live 웹 버튼 · 게이트웨이 → 중계 코디네이터 |
| `/fleet/lane/status` (`mission` · `warning` · `robots` …) | 8 | 중계 코디네이터 |
| `/pinkyN/command` (FleetCommand) | 8 → 로봇 | live 웹 `SET_SPEED` · `SET_INITIAL_POSE`, 코디네이터 (브리지) |

`relay_station/tests/test_live_web_contract.py` 가 live 웹 소스를 직접 읽어 이 표를 대조한다 — 이름이 바뀌면 먼저 빨개진다.
live 웹은 로봇이 보고한 지도가 자기 `map_yaml` 과 **원점·크기까지 같을 때만** 로봇을 그린다. 그래서 좌표 프로파일을 바꾸면
(콘솔 ① → ②) live 웹도 `launch_live_web.sh` 로 다시 띄운다.

## 중계 상태 — `GET /api/relay/health`

콘솔과 보기 전용 화면이 이것 하나를 2 초마다 본다: 도메인 · DDS 프로파일 · 코디네이터(미션 · 비상정지 래치 · 경고 · 프로파일) ·
제어권 · live 웹이 떠 있나 · 로봇마다 [브리지 발행자 수(state · lane_status · diag · amcl_pose · camera) · 최근 수신 · 지도 대조 ·
세움 이유 · 온보드 진단(Nav2 활성 수 · `/estop` · 게이트 출처 · 지도 적재 · 링크유실 래치)]. 못 잰 값은 `null` 이다.

## 영상 공유

| | |
| :-- | :-- |
| `GET /api/sources` | 소스 목록(id · 종류 · 최근 프레임 시각 · 수신 fps) |
| `GET /video_feed?src=<id>` | 그 소스의 MJPEG 스트림 (`/video_feed` 만 주면 기본 소스) — 콘솔의 영상 타일 |
| 소스 설정 | `relay_station/configs/video_sources.json` (당겨올 URL, 후보 주소, 발견 규칙) · `MCV_HOST_CAMERA_URL` |

폰 카메라 앱은 이 저장소에 없다(원 저장소 `packaging/android-app`, APK 는 손으로 전달). 앱은 `:18086` 에서 MJPEG 를 내고
중계가 당겨온다 — 폰과 중계 PC 가 **같은 네트워크**에 있어야 한다. 로봇 전방 카메라는 live 웹이 ROS 토픽으로 직접 본다.

## 개편에서 지운 것 (2026-09-29)

중계 화면 V2(`fleet_control_v2.*`) · 옛 멀티뷰 `index.html` · 데스크톱 GUI(`relay_controller_gui.py`) · 태블릿 비전 월드와 캘리브레이션
(`vision_world` · `calibration` · `drift` · `framing` · `masks` · `censorship`) · 원 저장소 단일 로봇 목표/미션 API(`/api/robot1/goal` ·
`/api/robot1/mission`) · `/api/ops/*` · Jenkins · 로그 API. 남은 HTTP 경로는 `relay_station/tests/test_relay_surface.py` 가 목록으로 잠근다.
`relay_station/scripts/control_robot1.sh` 의 목표·미션 메뉴는 이제 404 를 받아 "거절" 로 끝난다(직접 발행으로 우회하지 않는다) —
긴급 정지 메뉴(`/api/robot1/stop` + 0 속도 직접 발행)는 그대로다. 기록: [`status.md`](status.md) 09-29 절.

## map4 ↔ map5

live 웹의 기본 지도·차선 도면(`fleet-lanes.svg`)은 **map5**(236×128 셀, 0.01 m/셀)에 맞춰 그려져 있고, 중계의 좌표 프로파일
(`relay_station/fleet/config/profiles/`)은 **map4**(47×25 셀, 0.05 m/셀, 가운데 원점)다. `launch_live_web.sh` 는 프로파일 지도(map4)를
live 웹에 넘기므로 로봇 위치는 맞게 찍히지만, map5 에 맞춘 차선 도면은 조금 어긋나 보인다. 로봇 Nav2 지도를 무엇으로 통일할지는
팀 결정이고, **map5 프로파일은 그 뒤에 넣는다**.

## 시험

```bash
cd relay_station && python3 -m pytest tests -q        # numpy · opencv-python · psutil · pyyaml 필요, install/setup.bash 뒤
```

기대(ROS 2 Jazzy 컨테이너): `relay_station/README.md` "시험" 절의 수치. skip 은 통과가 아니다(환경 4 + 대사 대상 없음 2).
`tests/test_team11_export.py` 가 공개 규칙(키 기본값 없음 · 키 없으면 거절 · 사설 주소는 자리표시자뿐 · 개인 경로 없음 ·
외부 사이트 낱말 없음 · COLCON_IGNORE)을 잠근다.

## 다시 옮길 때

원 저장소 main 이 바뀌면 같은 규칙으로 다시 내보낸다(내보내기 도구는 원 저장소에 있다 — 실제 주소 치환표를 담아 공개하지 않는다).
기준 커밋은 `relay_station/README.md` 첫 줄과 [`source_versions.md`](source_versions.md) 에 적는다. 개편으로 지운 파일은 다시 옮길 때
제외 목록에 넣는다(되살아나면 `test_relay_surface.py` 가 잡는다).
