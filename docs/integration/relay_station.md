# relay_station — 중계 관제국 (rkd1rjs2 팀원 몫)

`relay_station/` 은 rkd1rjs2/robot_mini_project_pinky 의 중계 관제국을 이 저장소 구조로 옮긴 것이다.
**colcon 패키지가 아니다**(`relay_station/COLCON_IGNORE`) — `colcon build` 는 건너뛴다. 순수 Python·셸이고,
실행에는 ROS 2 Jazzy 와 이 저장소의 `pinky_fleet_msgs` · `pinky_lane_msgs` 가 필요하다. 파일 안내와 옮기며 바꾼 것은
[`relay_station/README.md`](../../relay_station/README.md).

## 이 저장소의 관제와 어떤 관계인가

| | `pinky_lane_station` · `pinky_fleet_station` (팀11) | `relay_station` (중계) |
| :-- | :-- | :-- |
| 무엇 | 레인 코디네이터 · GUI · live 웹(`:8080`, 조회 전용) · 상부 추적기 | 웹 관제(`:8889`) · 플릿 코디네이터 · 도메인 브리지 · 영상 공유 |
| 메시지 | `FleetCommand` `RobotState` `LaneCommand` `LaneStatus` `Route` | **같다** (바이트 동일한 `.msg`) |
| 도메인 | 관제 PC 도메인 0, 로봇 10·11 을 `bridges.launch.xml` 로 | 관제 8, 로봇 10·11 을 `relay_station/domain_bridge/` 로 |
| 로봇 쪽 짝 | `lane_robot.launch.xml`(레인) · `robot.launch.xml`(Nav2) | `hybrid_robot.launch.xml`(Nav2 + `/cmd_vel` 게이트 + 레인 관제 수신, [`docs/hybrid_nav2_gate.md`](../hybrid_nav2_gate.md)) — 레인 로봇도 같은 `LaneCommand` 로 중재된다 |
| 지도 | live 웹·상부 추적기는 **map5** | 좌표 프로파일은 **map4** (아래) |

**같은 로봇에 두 코디네이터를 동시에 붙이지 않는다.** 둘 다 `/pinkyN/lane_command` · `/pinkyN/route` 를 낸다.
조회 전용인 live 웹(`:8080`)은 어느 쪽 코디네이터와도 같이 띄울 수 있다.

## 실행

```bash
# 이 저장소 루트
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pinky_fleet_msgs pinky_lane_msgs pinky_fleet_agent && source install/setup.bash

# 1) 값 채우기 — 주소·NIC 는 전부 자리표시자다 (relay_station/README.md "실행 전에 바꿔야 하는 것")
#    relay_station/configs/{fleet_domains.env,cyclonedds.xml,video_sources.json} · FIELD_NIC
#    팀원 노트북에서 움직이는 명령을 내리려면 relay_station/configs/control_allow.json 에 그 노트북 주소를 enabled:true 로 (재기동 없음)
export RELAY_VISION_API_KEY=<공유 키>          # 비전 수신 API 를 쓸 때만. 없으면 그 API 는 전부 401

# 2) 중계 PC (도메인 8)
relay_station/launch_master_gateway.sh          # DDS 프로파일 선택 → 브리지 → 웹 서버 :8889
#    또는 따로: relay_station/domain_bridge/install_bridge.sh · python3 relay_station/gateway_web/gateway_web_server.py --port 8889

# 3) 로봇 (도메인 10 · 11)
ros2 launch pinky_fleet_agent hybrid_robot.launch.xml robot_name:=pinky1 domain_id:=10 map_name:=map4
```

브라우저: `http://<중계 PC>:8889/fleet_control_v2.html` — 설정 탭에서 **① 중계 좌표 전환(map4) → ② 로봇 지도 전환 → ③ 초기 위치**
뒤 주행 시작. `http://<중계 PC>:18081` 로 열면 **보기 전용**(움직이는 조작이 숨는다 · `/api/status` 의 `view_only`).

**팀원 노트북에서 움직이는 명령**(2026-09-28 저녁): 기본은 중계 PC 자신만 움직이는 명령(목표·미션·재개·좌표 전환·플릿 start/resume/assign)을
낸다. `relay_station/configs/control_allow.json` 에 노트북 LAN 주소를 `enabled: true` 로 넣으면 그 노트북의 `:8889` 화면에서도 낼 수 있다 —
**한 번에 한 사람**(첫 명령이 제어권, 다른 허용 노트북은 409 와 노랑 알약, 30 s 무응답이면 만료) · 중계 PC 콘솔은 언제나 제어권을 가져온다 ·
멈추는 명령(일시정지·비상정지·로봇 정지)은 누구든. 규칙·접점·시험: [`control_policy.md`](control_policy.md).

## 영상 공유

중계는 카메라 소스를 모아 브라우저에 다시 내보낸다(MJPEG). 폰·태블릿의 카메라 앱, 로컬 웹캠, 로봇 카메라 토픽이 소스가 된다.

| | |
| :-- | :-- |
| `GET /api/sources` | 소스 목록(id · 종류 · 최근 프레임 시각 · 수신 fps) |
| `GET /video_feed?src=<id>` | 그 소스의 MJPEG 스트림 (`/video_feed` 만 주면 기본 소스) |
| `GET /` (`index.html`) | 멀티뷰 — 소스 타일을 사람이 연다(기본은 전부 닫힘) |
| 소스 설정 | `relay_station/configs/video_sources.json` (당겨올 URL, 후보 주소, 발견 규칙) · `MCV_HOST_CAMERA_URL` |

폰 카메라 앱은 이 저장소에 없다(원 저장소 `packaging/android-app`, APK 는 손으로 전달). 앱은 `:18086` 에서 MJPEG 를 내고
중계가 당겨온다 — 폰과 중계 PC 가 **같은 네트워크**에 있어야 한다.

## map4 ↔ map5

이 저장소의 live 웹·상부 추적기는 **map5**(236×128 셀, 0.01 m/셀, 원점 −0.01/−0.01)를 쓰고, 중계의 좌표 프로파일
(`relay_station/fleet/config/profiles/`)은 **map4**(47×25 셀, 0.05 m/셀, 가운데 원점)다. 로봇 Nav2 지도를 무엇으로 통일할지는
팀 결정이고, **map5 프로파일은 그 뒤에 넣는다**(프로파일 파일에 같은 주석). 그때까지 두 화면의 좌표를 겹쳐 읽지 않는다.

## 시험

```bash
cd relay_station && python3 -m pytest tests -q        # numpy · opencv-python · psutil · pyyaml 필요, install/setup.bash 뒤
```

기대(ROS 2 Jazzy 컨테이너): `relay_station/README.md` "시험" 절의 수치. skip 22 는 통과가 아니다 — 환경(Chrome·socat·tailscale0·
격리 네트워크) 18 + 이 저장소에 대사 대상이 없어 사유를 적어 둔 4. `tests/test_team11_export.py` 가 공개 규칙(키 기본값 없음 ·
키 없으면 거절 · 사설 주소는 자리표시자뿐 · 개인 경로 없음 · 외부 사이트 낱말 없음 · COLCON_IGNORE)을 잠근다.

## 다시 옮길 때

원 저장소 main 이 바뀌면 같은 규칙으로 다시 내보낸다(내보내기 도구는 원 저장소에 있다 — 실제 주소 치환표를 담아 공개하지 않는다).
기준 커밋은 `relay_station/README.md` 첫 줄과 [`source_versions.md`](source_versions.md) 에 적는다.
