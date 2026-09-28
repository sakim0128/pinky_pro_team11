# relay_station — 중계 관제국 (웹 관제 · 플릿 코디네이터 · 도메인 브리지)

rkd1rjs2/robot_mini_project_pinky 의 `relay_station/` 을 이 레포 구조로 옮긴 것이다(원 저장소 main `c784497`).
**colcon 패키지가 아니다** — `COLCON_IGNORE` 가 있어 `colcon build` 는 이 폴더를 건너뛴다. 순수 Python 과 셸이며,
실행에는 ROS 2 Jazzy 와 이 레포의 `pinky_fleet_msgs` · `pinky_lane_msgs` 가 필요하다.

## 무엇인가

| 폴더 | 역할 |
| :--- | :--- |
| `gateway_web/` | 웹 관제 서버 `:8889` (`gateway_web_server.py`) — 로봇 상태·지도·목표 API, 플릿 제어 화면(`static/fleet_control_v2.*`), 영상 중계 |
| `fleet/` | 플릿 코디네이터 — 도로망(`road_graph`) 위 경로 배정, 구간 예약(`reservation`), `LaneCommand`(START·CLEARANCE·STOP·ESTOP·RESUME) · `Route` 발행 |
| `domain_bridge/` | 관제 도메인(8) ↔ 로봇 도메인(10·11) 토픽 브리지 설정·생성기·systemd 유닛 |
| `configs/` | DDS 프로파일(`cyclonedds*.xml`) · 로봇 도메인 표(`fleet_domains.env`) · 영상 소스 · 경기장 · 영상 가림 정책 |
| `network/` · `scripts/` · `systemd/` | 현장 운용 스크립트(방화벽·SSH 터널·게이트웨이 유닛 설치 등) — **예시**다. 주소·경로는 자기 현장 값으로 바꿔 쓴다 |
| `tests/` | 시험(아래) |

**이 레포의 기존 관제와의 관계**: `pinky_lane_station` · `pinky_fleet_station` 과 **같은 메시지 계약**
(`FleetCommand` · `RobotState` · `LaneCommand` · `LaneStatus` · `Route`)을 쓰는 **다른 관제 구현**이다. 둘 중 하나만 띄운다.
도메인 배치가 다르다 — 이 중계는 관제 = 8, 로봇 = 10·11 이고 사이를 `domain_bridge` 가 잇는다.
로봇 쪽은 `pinky_fleet_agent/launch/hybrid_robot.launch.xml`(Nav2 + 게이트 + `hybrid_agent_node`)과 짝이다
(`docs/hybrid_nav2_gate.md`). 레인 로봇(`lane_robot.launch.xml`)도 같은 `LaneCommand` 로 중재된다.

## 실행 전에 바꿔야 하는 것

- **`RELAY_VISION_API_KEY`(필수, 비전 API 를 쓸 때)** — 비전 수신 API(`/api/vision/pose_fix` · `/api/vision/zone_event`)의
  공유 키. 코드에 기본값이 **없다**. 안 주면 비전 API 는 전부 401 이다(fail-closed).
- **주소는 전부 자리표시자다** — `configs/*.json|xml|env`, 스크립트, 화면 안내 문구의 IP 는 문서용 대역
  (`198.51.100.x` = 현장 LAN, `203.0.113.x` = 기타 LAN, `100.64.0.x` = tailnet)으로 바꿔 두었다. 자기 현장 값으로 채운다.
- **`FIELD_NIC`** — 현장 유선 어댑터 이름. 기본값 `enx001122334455` 는 자리표시자이고, `configs/cyclonedds.xml` 의 인터페이스
  이름도 같이 바꾼다(`tests/test_bridge_env.py` · `test_dds_profiles.py` 가 둘이 같은지 본다). 없으면 런처가 offsite 프로파일로 떨어진다.
- **경로** — 스크립트는 체크아웃을 `$HOME/pinky_pro/src/pinky_pro_team11`, 로봇 워크스페이스를 `$HOME/pinky_pro` 로 가정한다
  (`REPO_ROOT` 로 바꿀 수 있다).

기동: `relay_station/launch_master_gateway.sh` (DDS 프로파일 선택 → 브리지 → 웹 서버 `:8889`).

## 원 저장소에서 옮기며 바꾼 것

- **비전 API 키**: 원 저장소는 기본 키를 코드에 두고, 키가 비면 인증을 **통과**시켰다. 여기서는 기본값을 없애고 키가 비면 **거절**한다.
- **신원**: 현장·tailnet IP → 위 자리표시자, 개인 홈 경로 → `$HOME` 기준(systemd 는 `%h`), 어댑터 MAC 이 든 NIC 이름 → 자리표시자.
- **경로**: 에이전트 `robot_onboard/pinky_fleet_agent` → `pinky_fleet_agent`(`agent_node` → `hybrid_agent_node`),
  메시지 `shared_msgs/pinky_*_msgs` → `pinky_*_msgs`, 레포 루트 `configs/` → `relay_station/configs/`.
- **싣지 않은 것**: 원 저장소의 도커 홈랩 복제본(`docker/`, 카메라 발행기 `docker/host_camera_publisher.py` 하나만 — 시험이 대사한다),
  게이트웨이 실행본 심링크 검사, 쓰지 않는 스크린샷, 에이전트 시험(→ `pinky_fleet_agent/test/`), 원 저장소 인프라·문서를 대사하는 시험,
  태블릿 비전 코드(용도가 바뀌어 싣지 않는다 — 비전 수신 API 는 남아 있지만 키 없이는 닫혀 있다).
- **시험**: 원 저장소의 다른 폴더(`robots/` · `tablet/` · `robot_onboard/pinky_navigation`)를 대사하는 4개는 사유를 적어 `skip`,
  팀11 구조에 맞춰 고친 것 3곳(게이트 위치 → `hybrid_robot.launch.xml`, Nav2 허용치, pipefail 규칙 문턱).
  `tests/test_team11_export.py` 가 위 규칙(키 · 주소 · 경로 · COLCON_IGNORE)을 잠근다.
- 주석의 `docs/*.md` 는 **원 저장소**의 설계·검수 문서를 가리킨다(이 레포에는 없다).

## 시험

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pinky_fleet_msgs pinky_lane_msgs pinky_fleet_agent && source install/setup.bash
cd <이 레포>/relay_station && python3 -m pytest tests -q        # numpy · opencv-python · psutil · pyyaml 필요
```

일부 시험은 Chrome · socat · tailscale0 · 격리 네트워크(unshare)가 없으면 skip 한다.
