# 첫 live 상태 조회 변경 검증

검증일: 2026-09-23

- `python3 -m pytest pinky_fleet_station/test -q`: 85 passed.
- ROS Jazzy에서 pinky_fleet_msgs, pinky_lane_msgs, pinky_fleet_station colcon 빌드 성공.
- 별도 /tmp/pinky-live-build build/install 사용: 기존 실기 워크스페이스 설치본 보존.
- 설치된 live_web.launch.xml 실행 성공 (domain 0, localhost:8086).
- 브라우저에서 정적 리소스와 JS 실행 확인. 두 로봇 카드, 미션, ROS 미수신 표시 확인.
- 실시간 API가 초기 미수신을 null로 반환하며 데모 값은 없음.
- HTTP 테스트: 정적 화면 제공, nonfinite 정규화, 제어 POST 405, 임의 경로 404.
- 상태 테스트: 독립적인 freshness 및 snapshot 격리 확인.
- ROS 어댑터: 명령 publisher/client가 없음을 정적 검사.
- `git diff --check` 통과.

미실시: 실제 로봇 데이터 수신, 실기 네트워크 단절, 여러 브라우저 장시간 부하.
현재 실행 환경에서는 로봇/미션 데이터가 들어오지 않음. 정상 연결을 주장하지 않음.
Node.js CLI가 없어 별도 `node --check`는 실행하지 못했으나 실제 브라우저의 JS 렌더링은 확인함.

## PR 설명 초안

제목: feat: add read-only live fleet web telemetry

기존 차선 주행 시스템의 위치·odom 기반 속도·LaneStatus·미션 상태를 브라우저에서
조회하는 웹 노드를 추가합니다. 각 입력의 수신 경과 시간을 독립적으로 추적하고
미수신/지연/서버 단절을 구분합니다. 명령·heartbeat를 발행하지 않으며 기존
coordinator와 bridge 실행을 변경하지 않습니다.

Python 표준 HTTP 서버와 패키지에 포함된 정적 화면을 사용합니다.
빌드 3개 패키지 성공, station 테스트 85개 통과, 설치본 브라우저 실행 확인.
실제 로봇 수신 검증은 미실시입니다. 지도·카메라·제어는 후속 PR 범위입니다.

Base: sakim0128/pinky_pro_team11:mini_project_2
Head: 0gpublike/pinky_pro_team10:feat/live-web-state

원격 push/PR 생성은 아직 수행하지 않았습니다.

## Figma 화면 수정 (2026-09-23)

Figma 1:2 프레임의 디자인 컨텍스트와 스크린샷을 다시 확인해 임시 목록 화면을
대시보드로 교체했습니다. 상단 제목/조작부, 지도와 카메라 2열, 로봇 카드 2열,
색상·테두리·라운딩·기준 높이를 반영했습니다. 좁은 화면에서는 단일 열로 전환합니다.
제어 버튼과 속도 적용은 실제 disabled 상태이며 지도/카메라/AMCL은 미연결을 명시합니다.
현재 TF 위치는 AMCL로 잘못 표시하지 않고 보조 행으로 유지합니다.
설치 패키지 재빌드 성공, 기존 테스트 85개 통과, 실제 브라우저 재로드 및 화면 확인 완료.
Figma의 로봇 예시 점과 타원은 실제 데이터가 없어 렌더링하지 않습니다.

## 전방 카메라 조회 (2026-09-24)

- `FrameStore` 단위 테스트로 JPEG magic bytes, 최대 크기, 허용 로봇 이름, stale 전환을 확인했다.
- `python3 -m pytest pinky_fleet_station/test -q`: 88 passed.
- ROS 3개 패키지 colcon 빌드 성공.

## 설계 차선 정렬 및 제어 UX (2026-09-24)

- 정규화한 Figma 차선 SVG와 제어 상태 요소가 정적 화면에 포함되는 테스트를 추가했다.
- `python3 -m pytest pinky_fleet_station/test -q`: 89 passed.
- ROS 3개 패키지 colcon 빌드 성공.
- 실제 카메라 프레임/bridge 수신은 미실시다. 이 환경에는 실기 camera publisher가 없다.

## 제어 요청 검증 (2026-09-24)

- `enable_control=false`일 때 `/api/control` 요청이 HTTP 403으로 거부되는 것을 HTTP 테스트로
  확인했다.
- control queue는 start/pause/resume/reset/speed만 받고, speed는 로봇 이름과 범위 검증을 통과해야
  큐에 넣는다.
- `python3 -m pytest pinky_fleet_station/test -q`: 88 passed.
- ROS 3개 패키지 colcon 빌드 성공. 실제 로봇 명령 발행은 미실시다.

## map5 호환성 진단 (2026-09-24)

- map metadata와 RobotState의 width/height/resolution/origin 비교 단위 테스트를 추가했다.
- state 없음, robot map 미수신, 폭 불일치와 호환 상태를 검증했다.
- `python3 -m pytest pinky_fleet_station/test -q`: 88 passed.
- ROS 3개 패키지 colcon 빌드 성공.

## map5 및 AMCL 조회 (2026-09-24)

- 사용자 제공 map5의 236×128 PGM, 0.01 m/cell, origin을 API와 설치 패키지에서 검증했다.
- `GET /api/map/metadata`가 위 메타데이터를 반환하고, PNG 응답이 PNG signature를 반환하는 HTTP
  테스트를 추가했다.
- `/pinky*/amcl_pose` 구독에서 필요한 primitive JSON만 보존하는 정적 테스트를 추가했다.
- `python3 -m pytest pinky_fleet_station/test -q`: 87 passed.
- `pinky_fleet_msgs`, `pinky_lane_msgs`, `pinky_fleet_station` colcon 빌드 성공.
- localhost:8086의 map metadata 응답 확인. browser automation 프로세스에서는 localhost 격리로
  화면 확인을 재현할 수 없었으며, HTTP endpoint 확인으로 대체했다.
