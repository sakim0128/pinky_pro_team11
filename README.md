# pinky_pro_team11 — mini_project_2 · 도로망 위 2대 차선 추종 자율주행

[Pinky Pro](https://github.com/pinklab-art/pinky_pro) **2대**가 각자 시작점에서 목적지까지, 마스킹 테이프로 만든
**분기가 있는 도로망**을 카메라로 차선을 인식해 주행한다. 관제 PC 가 라이다 맵 위 도로망 그래프에서
경로를 계산해 로봇에 주고, 로봇은 그 경로를 따라가되 **좌우 위치는 카메라가 본 차선 중앙**으로 맞춘다.
두 대가 폭 20 cm 도로에서 마주치면 비켜갈 길이 없으므로, 관제가 구간을 **예약**해 마주침 자체를 막는다.

> mini_project_1(2 대 멀티로봇 관제) 문서는 [`docs/mini_project_1.md`](docs/mini_project_1.md).
> 이 브랜치는 그 코드에서 분기했고, AMCL·데드맨 스위치·브릿지·GUI 를 그대로 재사용한다.

---

## 과제 요구사항 ([`MVP_subject.html`](MVP_subject.html)) ↔ 구현

| # | 요구 | 구현 |
|---|---|---|
| 1 | **차선 추종** | YOLO-seg `lane` 인스턴스 중 화면 중앙에 가장 가까운 좌/우 쌍 → 중점 |
| 2 | **중앙 정렬** | `ErrorX = target_x − 화면중앙` 을 각속도 보정항으로, 20 Hz. **좌우 위치는 100 % 카메라** |
| 3 | **횡단보도 정지** 후 재주행 | `crosswalk` 인스턴스 하단 y 임계 → 3 s 정지 → 주행거리 기준 재래치 방지 |
| 4 | **장애물 정지**, 제거 시 **자동 재개** | 라이다 전방 섹터 + 초음파 → 1 s 비면 복귀 |
| + | 시작·목적지 지정, **2대 동시** | 도로망 그래프 경로 계획 + 구간 예약 (관제) |

**코스**: 회색 카펫 + 흰 테이프, 흰 폼보드 벽, 도로 폭 약 20~25 cm, 분기·루프·횡단보도 2곳, 직각에 가까운 코너 다수.

---

## 왜 카메라만으로는 안 되는가

분기가 있으면 차선 추종은 세 가지를 모른다: **갈림길에서 어느 쪽**, **여기가 목적지인지**, **교차로에서 어느 차선 쌍이 내 길인지**.
셋 다 "맵 위 내 위치"가 있어야 풀린다. 라이다 맵과 mini_project_1 의 AMCL 이 이미 있으므로 **위치 파악용으로만** 살린다
(`localization_launch.xml` — map_server + AMCL. planner/controller 는 띄우지 않는다).

AMCL 은 5 cm 쯤 흔들려서 차선 중앙 유지에는 못 쓴다. 그건 카메라만 할 수 있다. 역할이 나뉜다:
**맵 경로 = 어느 길로, 카메라 = 그 길의 어디에.**

---

## 아키텍처

```mermaid
flowchart LR
    subgraph PC["관제 PC · domain 0"]
        G[road_graph.yaml<br/>노드·엣지] --> RP[route_planner<br/>Dijkstra]
        RP -->|Route| B
        RS[reservation<br/>엣지 배타 점유] -->|clear_until| B
        P[lane_pipeline_node<br/>YOLO-seg → 좌/우 쌍 → ErrorX] -->|LanePath 0.3 s| B
        GUI[fleet_gui 확장<br/>그래프 편집 · 시작/목적지 · 예약 표시]
        B[domain_bridge]
    end
    subgraph R["핑키 N · domain 10/11"]
        AM[map_server + amcl<br/>위치만] --> LA
        CAM[camera_node<br/>picamera2 → 반전 보정 → JPEG] -->|image/compressed| B
        OG[obstacle_guard<br/>/scan + /us_sensor/range] --> LA
        LA[lane_agent_node<br/>경로 pure-pursuit + 카메라 보정<br/>FSM · /cmd_vel 유일 발행자]
        LA -->|LaneStatus · RobotState| B
    end
    B -->|Route · LanePath · LaneCommand| LA
```

### 원칙
1. `/cmd_vel` 발행자는 `lane_agent_node` 하나. Nav2 controller/planner 없음.
2. **정지 권한은 로봇.** 관제는 "가도 되는 지점"(`clear_until`)과 "무엇이 보이는지"만 준다. 관제가 죽으면 로봇은 `link_watch` 로 멈춘다.
3. 이미지 stamp 는 로봇이 찍고 이후 복사만. 두 기기 시계는 만나지 않는다.

---

## 설계

### 도로망 그래프 (`config/road_graph.yaml`)
노드(끝점·분기·횡단보도, map 좌표) + 엣지(구간 중심선 폴리라인, 0.10 m 간격). 기존 `MapCanvas` 클릭으로 편집하고,
라이다 맵 위에 항공 사진을 반투명으로 겹쳐 놓고 찍는다. 코스가 테이프라 바뀔 수 있으니 편집이 쉬워야 한다.
중심선 정확도는 ±5 cm 면 충분하다 — 나머지는 카메라가 메운다.

### 구간 예약 (관제)
```
매 틱, domain_id 오름차순:
  뒤 엣지 해제 — 끝 노드를 0.15 m 지나면
  앞 엣지 예약 — 시작까지 0.40 m 이내이고 비어 있으면 점유 → clear_until 연장
로봇은 clear_until 웨이포인트에서 정지(WAIT_CLEARANCE), 연장되면 재출발
```
같은 엣지 반대 방향 마주침은 배타 점유로 **구조적으로** 불가능하다. 2대 상호 대기 교착(서로의 다음 엣지를 서로가 점유)은
감지해 GUI 경고 + 수동 양보 버튼만 둔다 — 폭 20 cm 도로에서 자동 후진은 위험하다. 데모 시나리오는 시작·목적지 조합을
그래프로 미리 검토해 이 경우를 피한다.

### 인식 (관제 `lane_pipeline_node`)
```
lane 인스턴스마다 샘플 행 y = 0.72·H 에서 폴리곤 교차 x
  좌 = x < W/2 중 최대, 우 = x > W/2 중 최소            # 화면 중앙에 가장 가까운 쌍
  BOTH: 중점 · SINGLE: 보이는 쪽 ± half_lane_px(관측 이력 중앙값) · LOST
error_x = (target_x − W/2)/(W/2)
crosswalk 인스턴스 최하단 y ≥ stop_row_px → N 프레임 확정
```
**분기 모드**: 관제가 로봇 pose 로 분기 노드 반경 0.25 m 안임을 알면 `quality=JUNCTION` 을 보낸다. 로봇은 카메라 보정을
끄고 맵 경로만 따른다. 교차로에서 차선 쌍이 모호해지는 문제를 풀지 않고 **회피**한다.
샘플 행 방식을 쓰는 이유: 곡선 차선의 중심점(moments)은 화면 중앙을 넘어갈 수 있다.

### 제어 (로봇 `lane_agent_node`, 20 Hz)
```
P   = Route lookahead(0.25 m) 점 → TF 로 로봇 좌표
ω_route = 2·v·sin(atan2(P.y, P.x)) / L_d            # pure pursuit
ω_cam   = −(Kp·error_x + Kd·Δerror_x/Δt)
ω = ω_route + w_cam·ω_cam,   w_cam = BOTH 1.0 · SINGLE 0.5 · JUNCTION/LOST/STALE 0
v = v_max · min(1, 1 − k·|error_x|, 1 − k'·|ω|/ω_max),  clear_until 앞에서 감속·정지
```
카메라가 차선을 못 봐도 맵 경로가 끌고 간다 — v2 의 "블라인드 선회"가 필요 없어진다. 대신 `w_cam=0` 이 0.6 m 이상
이어지면 감속·경고(AMCL 만 믿고 오래 달리지 않는다).

### 주행 FSM (로봇)
```
ESTOP > LINK_LOST > OBSTACLE_WAIT > WAIT_CLEARANCE > CROSSWALK_STOP > CROSSWALK_CLEAR
      > ARRIVED > DRIVE(BOTH / SINGLE / JUNCTION / BLIND) > IDLE
```
- **OBSTACLE_WAIT** — 라이다(±35°, 0.18 m) 또는 초음파 ≤ 0.20 m, 2 연속 → 정지. 1 s 비면 **자동 복귀**. 상대 핑키인지 구분하지 않는다 — 예약이 제대로면 여기 오지 않고, 와도 동작은 같다.
- **WAIT_CLEARANCE** — `clear_until` 도달 후 관제 연장 대기. `state_reason = "J1→C1 대기 (pinky1 점유)"`.
- **CROSSWALK_STOP** — 확정 후 3.0 s → CLEAR. 재래치는 `travelled ≥ crosswalk_len + 0.25 m`. 그래프의 crosswalk 노드 반경 밖 트리거는 무시(오검출 방어).
- **ARRIVED** — 목적지 노드 0.10 m 이내 → 정지, 점유 전부 해제.
- 정지 중에도 zero Twist 20 Hz. 종료 시 작별 스핀. 영상·`LanePath` 는 BEST_EFFORT depth 1 — RELIABLE 은 재연결 시 낡은 값을 재생해 로봇을 혼자 움직인다.

### 카메라로 핑키를 인식하지 않는 이유
라이다(12.5 cm)가 핑키(14 cm)를 잡고, 관제가 두 로봇 위치를 안다. "전방에 뭔가 있음 + 상대가 그 근처" = 핑키, 아니면 장애물.
클래스 추가·라벨링 없이 구분되고, 어차피 둘 다 "일단 멈춤"이다.

---

## 메시지 (`pinky_lane_msgs`, 신규)

| 메시지 | 방향 | 핵심 필드 |
|---|---|---|
| `Route` | PC→로봇 (R, TL) | waypoints[] · edge_end_idx[] · edge_ids[] · goal_idx · crosswalk_idx[] · junction_idx[] |
| `LanePath` | PC→로봇 (BE/1) | source_stamp · quality(BOTH/SINGLE/JUNCTION/STALE/LOST) · error_x_norm · left/right_x · half_lane_px |
| `SceneState` | PC 내부 | crosswalk_detected · crosswalk_bottom_y · detector_name · infer_ms |
| `LaneCommand` | PC→로봇 (R/10) | HEARTBEAT/START/STOP/ESTOP/RESUME/SET_SPEED · **clear_until_idx** |
| `LaneStatus` | 로봇→PC (R/10) | state · state_reason · route_idx · edge_id · error_x_norm · path_age · lidar_min · us_range |

기존 `FleetCommand`(`CMD_SET_INITIAL_POSE`, `CMD_HEARTBEAT`)와 `RobotState`(pose 보고)는 그대로 쓴다. `pinky_fleet_msgs` 필드는 건드리지 않는다.

---

## 패키지

| 위치 | 파일 | 역할 |
|---|---|---|
| `pinky_fleet_agent/` (로봇, 확장) | `camera_node` `lane_agent_node` · ROS-free `route_follower` `lane_control` `drive_fsm` `obstacle_guard` · `launch/lane_robot.launch.xml` | bringup + AMCL + 카메라 + 주행 |
| `pinky_lane_station/` (PC, 신규) | `lane_pipeline_node` · ROS-free `lane_target` `road_graph` `reservation` · `detectors/` · `fake_camera_pub` `fake_lane_robot` · `config/road_graph.yaml` `bridge_lane.yaml` | 인식·경로·예약·시뮬 |
| `pinky_fleet_station/` (PC, 최소 수정) | `coordinator_node`(예약 루프) · GUI 그래프 편집 위젯 | 기존 관제 재사용 |
| `pinky_lane_msgs/` (신규) | 위 5 개 | |
| `training/` | 데이터셋·학습 스크립트, `COLCON_IGNORE` | 가중치는 git 에 넣지 않는다 |

**재사용**: `link_watch.py`(그대로) · `map_canvas` 좌표 변환 · `bridge_fleet.yaml` QoS 논리 · GUI QTimer 패턴 · 업스트림 `localization_launch.xml`.

---

## 데이터 수집

절차와 스크립트는 [`tools/README.md`](tools/README.md). `record_drive.py`(로봇, mp4 + stamp CSV, `--snap` 방향 확인) · `extract_frames.py`(PC, 프레임 솎기).
세션(`course_a`~`e`, `crosswalk`)을 나눠 찍어야 세션 단위 train/val 분할이 된다. 함께 `ros2 bag record /odom /scan /tf /tf_static /amcl_pose /cmd_vel`.

---

## 마일스톤

| | 내용 | 완료 기준 |
|---|---|---|
| M0 | 그래프 편집기 + `road_graph.yaml` | 라이다 맵에 사진 정합, 노드·엣지 완성, 경로가 GUI 에 그려짐 |
| M1 | 메시지 + ROS-free 코어 + 하드웨어 없는 폐루프 | 가짜 로봇 2대가 경로 추종·예약 대기·횡단보도 정지·도착. pytest 그린. 추론 벤치 |
| M2 | 카메라 노드 + 전송 | p95 < 250 ms, 좌/우 일치 육안 확인 |
| M3 | `best.pt` 연결, 쌍 선택·SINGLE·crosswalk 튜닝 (녹화 영상) | `target_x` 궤적이 중앙, 분기에서 JUNCTION 전환 |
| M4 | 실차 1대: AMCL + 경로 + 카메라 보정 + 장애물 | 시작→목적지 3/3, 분기 정확, 벽 접촉 0, 장애물 자동 재개, PC 종료 시 1 s 정지 |
| M5 | 횡단보도 | 3.0 ± 0.3 s, 통과당 1 회, 두 곳 모두 |
| M6 | 실차 2대: 예약 + 동시 출발 | 겹치는 경로에서 마주침 0, 대기 후 재출발, 둘 다 도착 |
| M7 | 문서 + 평가 리포트 | 인식률 · 중앙 정렬 RMS · 정지 성공률 · 완주율 |

---

## 확인이 필요한 것

- 도로 폭 실측 (20 cm 면 코너에서 차선이 시야를 벗어나는 구간이 길어진다)
- 라이다 맵이 현재 벽 배치와 맞는지 (테이프만 바뀌고 벽은 그대로인지)
- 횡단보도 2곳 모두 정지 대상인지, 3 s 유지인지
- 데모 시작·목적지 조합 — 반대 방향 공유 엣지가 있는지 그래프로 미리 확인
- `--snap` 으로 고른 보정값, 초음파 노드 동작 여부
- AMCL 초기 자세를 GUI 클릭으로 줄지, 시작 노드에 놓고 자동 세팅할지
