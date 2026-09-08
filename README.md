# pinky_pro_team11

현장에서 SLAM 으로 만든 맵 위에서 **Pinky Pro 2대를 하나의 PC 로 관제**하는 프로젝트.

미션: 목적지 좌표 하나를 지정하면 →
**1호기가 목적지를 찍고 출발지로 복귀 → 그 다음 2호기가 같은 목적지를 찍고 복귀**.

두 로봇은 같은 WiFi 에 있지만 `ROS_DOMAIN_ID` 가 각각 **10, 11** 로 분리되어 있다.
그래서 관제 쪽이 두 도메인에 동시에 접근해야 한다.

```
 [Pinky-1] DOMAIN 10                    [Pinky-2] DOMAIN 11
   │ ▲ NavigateToPose 액션 (직접)          │ ▲
   │ │ /map, /amcl_pose (브리지)           │ │
 ┌─┴─┴───────────── PC ───────────────────┴─┴──┐
 │ 제어 평면  fleet_master + 워커 프로세스 2개   │
 │ 관제 평면  domain_bridge × 2 → rviz2 (D20)  │
 └─────────────────────────────────────────────┘
```

- **제어 평면** — `multiprocessing` 으로 도메인마다 프로세스를 나누고 각자
  `rclpy.init(domain_id=N)` (19·20강). Nav2 는 `BasicNavigator` 액션으로 직접 지시한다.
- **관제 평면** — `domain_bridge` 로 `/map`, `/amcl_pose` 만 관제 도메인(20)으로 중계해
  **rviz2 한 화면**에서 두 대를 본다 (21강).

액션을 브리지에 태우지 않은 이유 등 설계 근거는 [`pinky_fleet/docs/ARCHITECTURE.md`](pinky_fleet/docs/ARCHITECTURE.md) 참고.

## 문서

| 문서 | 내용 |
|---|---|
| [ARCHITECTURE.md](pinky_fleet/docs/ARCHITECTURE.md) | 구조와 기술 선택 근거 (발표용) |
| [RUNBOOK.md](pinky_fleet/docs/RUNBOOK.md) | 현장 실행 순서, 문제 해결 |
| [VERIFICATION.md](pinky_fleet/docs/VERIFICATION.md) | L0~L4 단계별 검증 절차 |

## 빠른 시작

```bash
# 0) 빌드
cd ~/ros2_ws/src && ln -s <이 저장소>/pinky_fleet .
cd ~/ros2_ws && colcon build --packages-select pinky_fleet && source install/setup.bash

# 1) 로봇 준비 — 각 로봇에 SSH (도메인 10 / 11)
ros2 launch pinky_bringup bringup_robot.launch.xml
ros2 launch pinky_navigation bringup_launch.xml map:=<맵이름>.yaml

# 2) PC
ros2 launch pinky_fleet fleet_bridge.launch.py      # 관제 평면
ros2 launch pinky_fleet fleet_view.launch.py        # rviz2 (도메인 20)
ros2 run    pinky_fleet fleet_master                # 미션

# RViz 의 Publish Point 로 2번 클릭 (목적지 → 바라볼 방향) 후 Enter
```

로봇 없이 로직만 확인:

```bash
cd pinky_fleet && PYTHONPATH=. python3 -m pinky_fleet.fleet_master --dry-run
```

## 설정

전부 [`pinky_fleet/config/mission.yaml`](pinky_fleet/config/mission.yaml) 한 곳에 있다.
관제 도메인, 로봇 도메인, 출발지 좌표, 타임아웃을 여기서 바꾸면
브리지 설정은 launch 가 자동으로 다시 생성한다.

## 수업 자료와의 연결

| 강의 | 이 프로젝트에서 |
|---|---|
| 07강 통신 방식 선택 기준 | 목적지 지시에 **액션**을 고른 근거 |
| 08강 토픽 pub/sub | 관제 콘솔의 `/clicked_point` 구독, 마커 발행 |
| 19·20강 다중 도메인 | `multiprocessing` + `rclpy.init(domain_id=)` |
| 21강 domain_bridge | 관제 평면, QoS 선택 |
| 25강 상태기반 제어 | 미션 상태머신 |
| 29강 + Pinky Pro PDF | Nav2 / `BasicNavigator` / `/amcl_pose` / `/clicked_point` |
