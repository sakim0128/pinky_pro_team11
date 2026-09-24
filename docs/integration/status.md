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
