# 팀 공유용 진행 현황

이 문서는 기능이 검증 가능한 단위에 도달할 때마다 갱신합니다. 구현 변경, 검증 결과,
다른 작업자가 확인하거나 결정해야 할 내용을 한곳에서 공유합니다.

## 2026-09-24 — map5 웹 지도 기반

완료한 변경:

- 사용자가 제공한 `map5.yaml`과 `map5.pgm`을 관제 패키지에 포함했다.
- 웹 노드가 Nav2 PGM을 PNG로 변환해 `GET /api/map/image.png`로 제공한다.
- `GET /api/map/metadata`는 지도 이름, 236×128 셀, 0.01 m/cell, origin,
  가로·세로 실제 길이를 제공한다.
- Fleet Map은 기본으로 map5를 표시하고, Figma 차선 도면은 별도 전환 레이어로 남겼다.
- `frame_id=map`이며 localized인 live 상태만 map5 위에 표시할 수 있도록 좌표 변환을 추가했다.

확인할 사항:

- Figma 차선 SVG와 map5 사이의 실제 기준점 대응은 아직 측정하지 않았다. 따라서 차선 도면
  화면에는 `map5 좌표 정렬 대기`를 표시하며, 해당 레이어 위에 로봇을 표시하지 않는다.
- 실기에서 `/pinky1/state`, `/pinky2/state`의 header.frame_id가 `map`이고 좌표가 map5를
  사용함을 팀원이 확인하면 map5의 로봇 마커가 바로 유효해진다.

다음 작업: `amcl_pose` 원본과 covariance를 웹 상태 API로 연결하고, 카드와 map5 오버레이에
불확실성 정보를 표시한다.

## 2026-09-24 — AMCL 및 covariance 조회

완료한 변경:

- domain bridge가 제공하는 `/pinky1/amcl_pose`, `/pinky2/amcl_pose`
  (`PoseWithCovarianceStamped`)를 조회 전용 웹 노드가 구독한다.
- API에는 ROS 메시지 객체 전체가 아니라 `header`, `position`, `orientation`, 36개 covariance
  숫자만 JSON으로 보존한다.
- 로봇 카드의 AMCL 행이 실제 x/y/yaw와 `σx`, `σy`, `σyaw`를 표시한다. 표준편차는 covariance의
  `[0]`, `[7]`, `[35]` 대각 성분의 제곱근으로 계산한다.
- AMCL은 정지 중 새 메시지가 없을 수 있어 시스템 health 판정에는 포함하지 않았다. AMCL 행의
  `수신 중/오래된 데이터/미수신`으로만 상태를 판단한다.
- map5 마커는 `localized`, `frame_id=map`, 그리고 RobotState의 map 규격(width/height,
  resolution, origin)이 map5와 전부 일치할 때만 표시하도록 제한했다.

검증:

- station 테스트 87개 통과.
- `pinky_fleet_msgs`, `pinky_lane_msgs`, `pinky_fleet_station` colcon 빌드 성공.
- map API는 map5 메타데이터를 실제로 반환하는 것을 localhost에서 확인했다.

추가한 지도 오버레이:

- map5 위의 노란 타원은 live AMCL의 XY covariance를 고유값/고유방향으로 변환한 2σ 영역이다.
- 녹색 `G`는 `RobotState.goal_valid=true`일 때의 실제 Nav2 goal이다.
- 두 오버레이와 로봇 마커 모두 map 규격 대조를 통과한 경우에만 표시한다.

다음 작업: 실기 `/pinky*/state`와 `/pinky*/amcl_pose` 샘플로 map5 규격 대조와 실제 표시 위치를
검증한다. 그 뒤에 상부 카메라 입력의 계약과 화면 연결을 진행한다.

## 2026-09-24 — Pinky 전방 카메라 실시간 조회

- `/pinky1/camera/image/compressed`, `/pinky2/camera/image/compressed`를 구독한다.
- JPEG 형식이며 2 MB 이하인 최신 프레임 한 장만 메모리에 보관한다. 다른 인코딩, 깨진 JPEG,
  과도하게 큰 프레임은 버린다.
- `GET /api/camera/<robot>.jpg`는 live 프레임만 제공하고, 화면은 `LIVE`/`STALE FEED`/
  `WAITING FEED`를 구분한다.
- 상부 카메라는 ROS 토픽 계약이 아직 없으므로 미연결로 유지했다.

검증: station 테스트 88개 통과, ROS 3개 패키지 colcon 빌드 성공.

## 2026-09-24 — 설계 차선 정렬 및 제어 UX

- Figma 차선 SVG의 원래 전체 기준 영역(5957×3194) 안에서의 vector 여백을 유지한 채 map5
  경계에 정규화했다. map5 기본 화면에서 점유 격자와 흰색 차선 도면을 함께 볼 수 있다.
- 이 정렬은 Figma 설계 영역과 map5 경계 비율을 맞춘 값이며, 실측 기준점 검증 전에는
  `설계 차선 정렬`로 표시한다.
- 제어 활성 상태와 요청 전송 결과를 toolbar에 표시한다.
- 초기화 버튼 설명에는 Pinky1 `(0.11, 1.08, 0°)`, Pinky2 `(0.16, 0.74, -90°)` 수동 배치
  좌표를 명시했다.
- 속도 값은 단위 문자 없이 입력 필드에 유지하고, 사용자가 수정 중인 값은 500 ms 상태 갱신이
  덮어쓰지 않는다.

검증: station 테스트 89개 통과, ROS 3개 패키지 colcon 빌드 성공.

## 2026-09-24 — 상부 ArUco 추적 노드

- `overhead_tracker_node`가 `/overhead/camera/image/compressed` JPEG에서 `DICT_4X4_50`
  ArUco ID 1(Pinky1), 2(Pinky2)를 검출한다.
- 보정된 image-pixel → map homography로 `/pinky*/overhead_pose` `PoseStamped`를 발행한다.
- 기본 homography는 비어 있으며 이 경우 pose를 절대 발행하지 않는다. 보정 전 가짜 P′ 위치는 없다.
- 보정 방법과 실행 명령은 `docs/integration/overhead_tracker.md`에 기록했다.

검증: station 테스트 90개 통과, ROS 3개 패키지 colcon 빌드 성공.

## 2026-09-24 — 카메라 상태·차선 정보 표시

- Overhead vision은 실제 상부 JPEG frame 상태와 `/pinky*/overhead_pose` 수신 로봇 수를
  `LIVE`, `POSE ONLY`, `NOT CONNECTED`로 표시한다.
- Overhead 카드 하단에는 live 외부 위치의 `P1′/P2′ x/y`를 표시한다.
- Pinky 전방 카메라 카드에는 `/pinky*/lane_status`의 차선 품질(양쪽·한쪽 추정·분기·이전·미검출),
  중심 오차, 장애물 거리, 횡단보도 상태를 표시한다.
- 모든 표시는 실제 live LaneStatus·camera·PoseStamped가 수신될 때만 채운다.

검증: station 테스트 89개 통과, ROS 3개 패키지 colcon 빌드 성공.

## 2026-09-24 — map5 호환성 진단 및 upstream 확인

- `GET /api/state`는 로봇별 `map_compatibility`를 반환한다.
- `state_missing`, `robot_map_unknown`, `mismatch:width|height|resolution|origin_x|origin_y`,
  `compatible` 중 하나로 map5 위 로봇 표시의 가능 여부를 설명한다.
- Fleet Map은 map5가 선택됐을 때 호환하지 않는 로봇과 원인을 표시한다.
- upstream `sakim0128/pinky_pro_team11:mini_project_2`를 fetch해 확인했다. 현재 시작점
  `1f505cb`와 같아 이 기능 브랜치에 반영할 새 upstream 커밋은 없었다.

검증: station 테스트 88개 통과, ROS 3개 패키지 colcon 빌드 성공.

## 2026-09-24 — 관제 제어 연결

- `enable_control:=true`일 때만 웹의 주행 시작, 일시정지, 재시작, 초기화, 로봇별 속도 적용을
  활성화한다. 기본값은 false다.
- 시작/일시정지/재시작은 기존 `/fleet/lane/control`에 각각 `start`/`stop`/`resume` JSON을
  전송한다. 차선 coordinator가 기존 예약·안전 규칙으로 처리한다.
- 초기화는 먼저 전체 차선 주행을 stop한 뒤 각 로봇의 `/pinky*/command`에
  `CMD_SET_INITIAL_POSE`를 발행한다. 위치는 map5 기준 Pinky1 `(0.11, 1.08, 0°)`,
  Pinky2 `(0.16, 0.74, -90°)`다. 로봇을 해당 실제 위치에 수동 배치한 경우에만 사용한다.
- 속도는 로봇별 `CMD_SET_SPEED`로 보내며 linear `0–0.3 m/s`, angular `0–2 rad/s`로
  서버에서 검증한다.

검증: station 테스트 88개 통과, ROS 3개 패키지 colcon 빌드 성공.

다음 작업: 실기 camera bridge 수신을 확인하고, 상부 카메라의 영상·좌표 토픽 계약을 정하면
Overhead 카드와 P′ 위치를 연결한다.

## 2026-09-24 — 상부 카메라·외부 위치 계약

- 상부 카메라 JPEG 기본 토픽은 `/overhead/camera/image/compressed`이며 launch 인수
  `overhead_camera_topic`으로 바꿀 수 있다.
- 외부 추적 위치는 `/pinky1/overhead_pose`, `/pinky2/overhead_pose`의
  `geometry_msgs/PoseStamped`를 사용한다. frame은 `map`이어야 한다.
- live 데이터만 Overhead 카드의 P′ 행과 map5의 `P1′`/`P2′` 표시에 반영한다.
- 아직 publisher가 없으므로 값이나 영상을 만들지 않는다.

검증: station 테스트 88개 통과, ROS 3개 패키지 colcon 빌드 성공.

## 2026-09-28 — relay_station(중계 관제국)·hybrid 로봇 스택 이식 (rkd1rjs2 팀원)

완료한 변경(브랜치 `mini_project_integration`, upstream `5faddf1` 위 → 09-28 저녁 `main` `094e5d7`(= 새 브랜치 `mini_project_integration`) 병합, 충돌 0 — PR 대상은 `mini_project_integration`):

- 로봇 온보드: `pinky_fleet_agent` 에 `hybrid_agent_node`(Nav2 + 레인 관제 수신) · `drive_command_gate`(`/cmd_vel` 단일 발행자) ·
  `pose_fuser_node`(aerial_view 원본) · `route_chain` 을 **추가**했다. 기존 `agent_node`·`lane_agent_node`·launch 는 바이트 그대로.
  새 최상위 launch `hybrid_robot.launch.xml`. `pinky_lane_msgs` 에 `PoseFix.msg`(aerial_view 와 바이트 동일)를 더했다.
  기록: `docs/hybrid_nav2_gate.md`(2026-09-29 삭제) (§6 "main 동기" — 원 저장소 에이전트 커밋을 따라온 표).
- 중계 관제국 `relay_station/`(웹 관제 `:8889` · 플릿 코디네이터 · 도메인 브리지 · 영상 공유). `COLCON_IGNORE` — 빌드 대상 아님.
  이 저장소 관제(`pinky_lane_station`)와 같은 메시지 계약을 쓰는 다른 구현이라 **같은 로봇에 둘을 동시에 붙이지 않는다**.
  안내: [`relay_station.md`](relay_station.md).
- 공개 규칙: 비전 API 키는 코드에 기본값이 없고 없으면 거절 · 사설/tailnet 주소는 자리표시자 · 개인 경로 없음 —
  `relay_station/tests/test_team11_export.py` 가 잠근다.

확인할 사항:

- **map4 ↔ map5**: live 웹·상부 추적기는 map5, 중계 좌표 프로파일은 map4. 로봇 Nav2 지도를 무엇으로 통일할지 팀 결정 뒤
  map5 프로파일을 넣는다(`relay_station/fleet/config/profiles/profiles.yaml` 주석).
- 실물 배포는 09-28 저녁 원 저장소 쪽(중계 노트북 + 실물 두 대)에서 했다. 이 저장소 체크아웃으로는 아직 돌려 보지 않았다.
- 상류 `mini_project_1`(`094e5d7`)과의 관계: `nav2_params_fleet.yaml` 튠은 하이브리드 런치가 같은 파일을 쓰므로 그대로 적용. `agent_node` 의
  Nav2 파라미터 감사(`param_audit`)는 `hybrid_agent_node` 에 아직 없다(후속). 하이브리드 스택은 절대 토픽(`/cmd_vel` · `/estop`)이라
  `pinky_fleet_sim` 의 네임스페이스 로봇에서는 뜨지 않는다(실기 전용). 표는 `docs/hybrid_nav2_gate.md` §8.

검증(ROS 2 Jazzy 컨테이너, `pinky_fleet_msgs pinky_lane_msgs pinky_fleet_agent` colcon build 뒤):
수치는 [`relay_station/README.md`](../../relay_station/README.md) "시험" 절과 `docs/hybrid_nav2_gate.md` §7(병합 전) · §8(`094e5d7` 병합 뒤: 에이전트 319 · 중계 1425/22 · station·lane 188/24). station·lane 시험은 그대로 초록.

## 2026-09-29 — 웹 배정 · 정지선 교차로 · 선착순 · 중복 정리 (브랜치 `mini_project_integration_stand`, 팀 동의 전)

`mini_project_integration`(e3ca6ed) 에서 갈라 만든 브랜치다. 팀 합의가 되면 `mini_project_integration` 에 얹고, 아니면 브랜치만 지운다.
결정(사용자, 2026-09-29): 코디네이터는 relay 하나 · 로봇은 `lane_agent_node` 하나 · 이름은 `pinkyN` · 지도는 map5 · 항공뷰는 relay 와 같은 도메인 8.

완료한 변경:

- **lane_rules 병합** (D14: 차선 ≥3 바깥 쌍 · LANE_SEARCH · BARRICADE_WAIT · JUNCTION_STOP/PASS · 항공뷰 pose_fuser). `lane_robot.launch.xml` 기본 위치추정 = 항공뷰.
- **웹 배정**: relay V2 설정 탭 "미션 배정" — 로봇별 시작/목적지 드롭다운(`/api/fleet/profiles.graph.endpoints`) → `/api/fleet/assign`.
- **정지선**: `LanePath.stop_line_*`, `SceneState.stop_line_*`, `lane_target` 디바운스, 오버레이. 모델 id 미정(`detector_yolo.yaml` 주석).
- **교차로 규칙**(로봇): 트리거 = 정지선(다음 분기 ≤ 0.6 m) 또는 분기 반경. `JUNCTION_STOP` 은 1 s + **관제 허가**(`clear_until` > 분기 idx) 뒤에만 `JUNCTION_PASS`.
- **선착순**(relay): `DRIVE_JUNCTION_STOP` 보고 → `reservation.request_next_now`(거리 무관) → 선착순(요청 틱 → domain_id). 상태 `junction_wait_sec`.
- **Nav2 하이브리드 스택 삭제**: `hybrid_agent_node` `drive_command_gate` `route_chain` `hybrid_link_watch` `diag` `pose_fix_fuser_node`, hybrid/nav2_gated launch, 관련 시험, `docs/hybrid_nav2_gate.md`(아래 옛 절의 링크는 죽었다). `agent_node`(mini_project_1)는 그대로.
- **항공뷰 웹캠**: `pinky_fleet_station/overhead_camera_node`(cv2 → CompressedImage, RELIABLE depth 1) + `overhead_tracker.launch.xml` 인수. 도메인 8 에서 relay 와 같이 띄운다 → 브리지 `/pinkyN/overhead_pose`(8→10/11).
- **코디네이터 단일화**: `pinky_lane_station` 의 `lane_coordinator_node`·`reservation.py` 삭제, `lane_station.launch.xml` = 브리지 + 인식. 도로망 구현은 `pinky_lane_station.road_graph` 하나(relay 포크 삭제, `summary()` 이식).
- **pinkyN 통일**: relay 의 `/robotN/*` · `robotN` id · `/api/robotN/*` → `pinkyN`. Nav2 직접 경로(goal_pose·mission_cmd·nav_status·teleop cmd_vel, Tk GUI, `/api/robotN/goal|mission`) 삭제. 브리지 설정 `pinkyN_control.yaml` 재생성(+ `camera/image/compressed` 업 · `lane_path` 다운).
- **map5**: 프로파일 `team11_map5` 하나(정본 파일을 상대경로로 참조, 복사 없음). `road_graph.yaml` 임시 4노드(BL·BR·TR·J), 미션 pinky1 BL→TR · pinky2 BR→BL. relay `assign_conflict` 는 도착해 선 로봇의 노드만 막는다(출발·분기의 일시 점유는 통과).
- live 웹: relay 의 `mission_state` 를 받는다, launch 가 `enable_control` 을 넘긴다.

검증(ROS 없는 컨테이너): 3개 패키지 pytest 348 passed / 24 skipped · relay 순수 시험(`test_assign_ui` `test_assign_map5` `test_junction_first_come` `test_bridge_*` `test_wiring` 등) 통과.
rclpy/ROS msgs 가 필요한 relay 시험은 이름 변경 뒤 컴파일만 했다 — 중계 PC 재측정 필요.

확인할 사항:

- 정지선 모델 id · 새 맵 `road_graph.yaml` 좌표 · 항공뷰 호모그래피(map5) · DDS Cyclone 통일(핑키 rmw 확인) — README "미결".
- `docs/integration/live_web.md`·`pr_live_web.md` 는 첫 PR 시점 문서라 relay 없는 도메인 0 배치를 말한다(이력).

## 2026-09-28 저녁 — 제어권 정책: 팀원 노트북도 중계를 거쳐 움직이는 명령을 낸다 (rkd1rjs2 팀원)

완료한 변경(원 저장소 main `1eed3f8` → 이 브랜치 `mini_project_integration`, `relay_station/` 안과 `docs/integration/` 만):

- `relay_station/gateway_web/control_policy.py`(순수 정책) + `gateway_web_server.py` 배선: 움직이는 다섯 경로(목표·미션·로봇 재개·좌표 전환·
  플릿 start/resume/assign)가 정책을 지난다. 기본은 예전과 같이 **중계 PC 자신만**. `relay_station/configs/control_allow.json` 에 팀원 노트북
  주소를 `enabled: true` 로 넣으면(재기동 없음) 그 노트북에서도 낸다 — 한 번에 한 사람(409 `CONTROL_HELD`) · 30 s 무응답 만료 · 중계 PC 콘솔 우선 ·
  멈추는 명령은 정책 밖. `GET /api/control` · `POST /api/control/{acquire,release}` · `/api/status` 에 `control` 블록.
- V2 화면 헤더에 제어권 알약 + 잡기/놓기. 남이 쥐면 움직이는 버튼을 잠근다(숨기지 않음).
- 기록: [`control_policy.md`](control_policy.md). 시험: `tests/test_control_policy.py`(순수 12) · `tests/test_control_0928_control_policy_wiring.py`(정적 8).

확인할 사항:

- 허용은 **주소 단위**이고 이름은 표시용이다(같은 Wi-Fi 안의 신뢰를 전제). 인증이 필요해지면 토큰을 따로 둔다.
- `:18081` 원격 경로는 여전히 보기 전용이다(socat 출처 127.0.0.2 는 허용 목록에 없다).
- 이 저장소 체크아웃에서의 실기 확인은 아직이다 — 팀원 노트북 한 대를 허용 목록에 넣고 `주행 시작` 이 먹는지, 두 번째 노트북이 409 를 받는지, 중계 PC 가 누르면 제어권이 넘어오는지 세 가지를 본다.

검증(rkd1rjs2 노트북, rclpy 없는 Windows — 정적·순수만): `test_team11_export.py` 7 · `test_no_baked_addresses.py` · `test_control_policy.py` 12 ·
`test_control_0928_control_policy_wiring.py` 8 · `test_control_0928_webui_p1.py` 11 · `test_v2_front_honesty.py` 6 통과. rclpy 가 필요한 나머지는
컨테이너 재측정 전이다(README "시험" 절의 1425/22 는 이 변경 **전** 수치).

## 2026-09-29 — 태블릿 좌표를 로봇 위치로 (rkd1rjs2 팀원, 문서만)

태블릿이 로봇 위 마커로 구한 map5 좌표를 중계 `POST /api/vision/pose_fix` 로 계속 보내면, 중계가 같은 좌표를
`/pinkyN/overhead_pose` 로 내고 브리지가 로봇으로 내린다 — 로봇 `pose_fuser_node` 가 천장 추적기 값과 똑같이 받는다.
마커 도착 판정과 갈림길 방향은 로봇과 코디네이터가 한다. 계약·규칙·확인 방법: [`tablet_pose.md`](tablet_pose.md).

- 게이트웨이 소스는 rkd1rjs2/robot_mini_project_pinky `e59d521` 에서 관리한다 — 이 브랜치 `relay_station/` 코드는 바꾸지 않았다.
- 이 브랜치의 브리지 설정에는 필요한 다운링크가 이미 있다.
- 확인할 사항: `road_graph.yaml` 노드 좌표는 임시값 — 바닥 마커 위치를 map5 로 재서 넣는다. 실물 확인 전이다.

## 2026-09-30 — 태블릿 비전 소스 합류 · 모서리 마커 각도 실측 (rkd1rjs2 팀원, 문서만)

`7c59c7c` 로 원 저장소 main 의 `tablet/vision/`(모서리 마커 40–43 으로 호모그래피를 스스로 맞춰 로봇 마커 30 · 31 의 x · y · yaw 를 중계
`POST /api/vision/pose_fix` 로 보낸다)가 이 브랜치에 들어왔다. 태블릿 시험 43개는 이 브랜치에서 통과한다(재현함).
안내·규칙·확인 방법: [`tablet_pose.md`](tablet_pose.md).

- 태블릿 세션이 현장 녹화 프레임에서 모서리 마커 부착 각도를 실측했다(40번 91.3° · 41번 0.6° · 42번 178.4° · 43번 −179.0°).
  yaw 를 전부 0 으로 두면 재투영 오차 20.36 px 로 송신이 막히고, 실측 각도를 넣으면 0.998 px 로 내려간다는 보고다 — 프레임 분석은 이 환경에서 재현하지 못했다.
- 아직 실측 전이다: 모서리 마커 중심 x · y 는 꼭짓점 가정값이고 세 설정은 `FIELD_CONFIG_PENDING` 이라 `--allow-pending-config` 로만 송신된다.
  보고된 샘플 로봇 좌표(x −0.667 · y −1.114)가 경기장 밖이라 원인 확인이 먼저다.
- 정리함: `7c59c7c` 가 다시 넣은 `relay_station/fleet/config/road_graph.yaml`(태블릿 mock 용 복사본)이 "도로망 하나" 결정과 충돌해
  `relay_station/tests/test_assign_map5.py` 의 `..._without_copies` 시험 하나가 실패했다. 복사본을 지우고 태블릿 mock 이 팀 정본
  `pinky_lane_station/config/road_graph.yaml`(노드 BL · BR · TR · J)을 읽게 바꿨다. 태블릿 코드의 `--relay-url` 사설 주소 기본값도 없앴다(빈 값 = dry-run).
  검증(ROS 2 Jazzy 컨테이너): 태블릿 시험 43 통과 · 팀 중계 시험 875 통과 / 5 skip, 실패 0.
- 게이트웨이 소스는 계속 rkd1rjs2/robot_mini_project_pinky `e59d521` 에서 관리한다.

## 2026-09-30 — 실물 없이 돌려 보는 도커 모의 테스트 안내 (rkd1rjs2 팀원, 문서만)

[`mock_test.md`](mock_test.md) 를 더했다. 로봇 · 카메라 · 태블릿 없이 노트북 도커만으로 세 단계를 돌린다.

- 1단계 비전 미션 시나리오 폐루프 34 통과 — **ROS 를 불러오지 않아야 돈다**(`--entrypoint /bin/bash`). ROS 가 불러와져 있으면 폐루프 시험 둘이 건너뛰어 32 통과 / 2 skip.
- 2단계 태블릿 가상 카메라 — 모의 프레임 실행 종료 코드 0, 태블릿 시험 43 통과. 순정 `ros:jazzy-ros-base` 에는 OpenCV 가 없다.
- 3단계 게이트웨이 `--vision` — 시나리오 `s1` 이 `RUNNING`, 모드 `vision`. 순정 이미지에는 `cv_bridge` 가 없다. V2 화면 주소는 `/fleet_control_v2.html` (`/` 는 옛 화면).
- 확인하지 못한 것: apt 로 OpenCV · `cv_bridge` 를 넣는 설치(확인 환경에서 apt 가 막힘), 컨테이너 포트 포워딩으로 브라우저에서 시나리오를 시작하는 것.

