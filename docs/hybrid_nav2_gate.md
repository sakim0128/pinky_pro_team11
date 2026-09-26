# Nav2 + 게이트 모드 (`hybrid_robot.launch.xml`) — 외부 비전 위치 · /cmd_vel 단일 발행자 · 레인 관제 수용

`mini_project_2` 위에 **새 파일만 얹은** 패치다. 기존 두 모드는 그대로다.

| 모드 | launch | /cmd_vel 발행자 | 위치 |
| :--- | :--- | :--- | :--- |
| 차선 주행 (기존) | `lane_robot.launch.xml` | `lane_agent_node` | AMCL |
| Nav2 단일 목표 (기존) | `robot.launch.xml` | Nav2 | AMCL |
| **Nav2 + 게이트 (이 패치)** | `hybrid_robot.launch.xml` | `drive_command_gate` | AMCL 또는 외부 비전(`use_pose_fuser:=True`) |

세 launch 는 동시에 띄우지 않는다 — 셋 다 `/<robot_name>/command` 를 받는다.

## 1. 무엇이 바뀌었나

**고친 팀11 파일은 세 개이고 모두 한두 줄 추가다.** 나머지는 새 파일이다.

| 파일 | 변경 |
| :--- | :--- |
| `pinky_lane_msgs/CMakeLists.txt` | `msg/PoseFix.msg` 한 줄 (mini_project_2_aerial_view 판과 같다) |
| `pinky_fleet_agent/setup.py` | 진입점 3개: `pose_fuser_node` · `drive_command_gate` · `hybrid_agent_node` |
| `pinky_fleet_agent/package.xml` | `lifecycle_msgs` (에이전트가 Nav2 수명주기를 묻는다) |

`agent_node.py` · `lane_agent_node.py` · `link_watch.py` · 기존 launch 는 바이트 그대로다
(`test_hybrid_launch.py::test_existing_launch_files_do_not_start_new_nodes` 가 launch 쪽을 지킨다).

새 파일

| 파일 | 역할 | 출처 |
| :--- | :--- | :--- |
| `pinky_lane_msgs/msg/PoseFix.msg` | 외부 절대 위치 계약 | `mini_project_2_aerial_view@21ebc31` 바이트 그대로 |
| `pinky_fleet_agent/pose_fuser.py` · `pose_fuser_node.py` · `test/test_pose_fuser.py` | PoseFix + odom → map→odom TF (AMCL 자리) | 같은 aerial_view 커밋, 바이트 그대로 |
| `pinky_fleet_agent/drive_command_gate.py` | ESTOP > TELEOP > MISSION > IDLE 중재, `/cmd_vel` 유일 발행자 | rkd1rjs2/robot_mini_project_pinky (아래 §5 수정 포함) |
| `pinky_fleet_agent/hybrid_agent_node.py` | FleetCommand 단일 목표 + Route/LaneCommand 레인 관제를 Nav2 목표 연쇄로 | 같은 저장소의 `agent_node.py` (아래 §5 수정 포함) |
| `pinky_fleet_agent/route_chain.py` | 레인 규약 상태 기계 (ROS 비의존). 진행 인덱스는 팀11 `route_follower.RouteFollower` | 같은 저장소 |
| `pinky_fleet_agent/hybrid_link_watch.py` | 데드맨 — **아무 명령이나** 생존 신호로 받는다 (`link_watch.py` 는 하트비트로만 무장) | 같은 저장소 (아래 §5 수정 포함) |
| `pinky_fleet_agent/diag.py` | `/<robot>/diag` JSON 스키마 (ROS 비의존) | 같은 저장소 |
| `launch/nav2_gated_navigation.launch.xml` | pinky_pro `navigation_launch.xml` 에서 cmd_vel remap 다섯 줄만 바꿈 | pinklab-art/pinky_pro (Apache-2.0) |
| `launch/nav2_gated_bringup.launch.xml` | pinky_pro `bringup_launch.xml` 구성 + 위 navigation. `use_pose_fuser` 면 AMCL 없이 map_server 만 | 같음 |
| `launch/hybrid_agent.launch.xml` · `launch/hybrid_robot.launch.xml` | 에이전트 단독 / 최상위 | 새로 |
| `test/test_drive_command_gate.py` · `test_hybrid_link_watch.py` · `test_hybrid_launch.py` | | 새로 |
| `test/test_route_chain.py` · `test_hybrid_agent_review.py` · `test_hybrid_agent_loopback.py` | | 같은 저장소에서 경로만 바꿔 옮김 (+ §5 수락 시험 추가) |

## 2. 실행

```bash
# 로봇 (pinky_pro 워크스페이스, 기존과 같은 빌드 패키지)
cd ~/pinky_pro && colcon build --packages-select pinky_lane_msgs pinky_fleet_agent && source install/setup.bash
export ROS_DOMAIN_ID=10   # pinky2 는 11
ros2 launch pinky_fleet_agent hybrid_robot.launch.xml robot_name:=pinky1 domain_id:=10 map_name:=map4
#   외부 비전으로 위치를 잡을 때: use_pose_fuser:=True  (관제가 /pinky1/pose_fix 를 브리지로 내려 줘야 한다)

# 관제 PC 쪽 PoseFix 를 쓰려면 pinky_lane_msgs 를 다시 빌드한다 (메시지가 하나 늘었다)
```

`/cmd_vel` 경로:

```text
controller_server > /cmd_vel_nav > velocity_smoother > /cmd_vel_mission ┐
behavior_server   > /cmd_vel_mission ─────────────────────────────────────┼> drive_command_gate > /cmd_vel
수동 조종         > /cmd_vel_teleop ──────────────────────────────────────┤
hybrid_agent_node > /estop (ESTOP · 링크 유실에서 true) ──────────────────┘
```

## 3. 레인 관제 규약 (hybrid_agent_node)

`pinky_lane_station` 코디네이터가 레인 로봇과 같은 방식으로 이 로봇을 중재할 수 있게 한다.

| 입력 | 동작 |
| :--- | :--- |
| Route(seq) | 새 경로. 진행도를 새로 잡고 IDLE (달리던 goal 취소) |
| START (seq 일치) | ack — LaneStatus 가 IDLE 을 벗어난다. STOP 래치도 푼다 |
| CLEARANCE (seq 일치) | `min(clear_until_idx, goal_idx)` waypoint 까지 NavigateToPose |
| 허가 지점 도달 · clear 0 | **HOLD = goal 취소.** `/estop` 은 건드리지 않는다 (게이트 MISSION 이 0.5 s 뒤 0) |
| STOP | HOLD 래치 (RESUME 으로 해제) |
| ESTOP · 링크 유실 | goal 취소 + `/estop true` 래치. **LaneCommand RESUME 으로만** 해제 |
| SET_SPEED | 속도 상한만 바꾼다 (lane_agent_node 와 같은 뜻) |

진행 인덱스는 TF `map→base_footprint` 를 Route 에 투영해 잰다 (`RouteFollower.update`). 링크가 끊겨도
로봇이 스스로 멈춰야 하므로 판단은 온보드에서 한다. 규약 전체는 `route_chain.py` 머리말에 있다.

## 4. 검증 (TESTED_SHA 영수증)

```text
TESTED_SHA = 9470111df43eb3da03299c5bebfd958a7c9648a4   (이 문서 커밋의 부모 = 코드 마지막 커밋)
git status --porcelain = (비어 있음)
base = mini_project_2 1f505cb88719c44391cc2781dcc3c5f09c9b7795
```

| 무엇 | 어디서 | 결과 |
| :--- | :--- | :--- |
| `python3 -m pytest pinky_lane_station/test pinky_fleet_agent/test pinky_fleet_station/test` | ROS 없는 Python 3.11 | **297 passed, 62 skipped** (skip = ROS 가 필요한 에이전트 시험). 패치 전 베이스 215 passed, 4 skipped |
| `colcon build --packages-select pinky_fleet_msgs pinky_lane_msgs pinky_fleet_agent` + `ros2 interface show pinky_lane_msgs/msg/PoseFix` | ROS 2 Jazzy 컨테이너 (`ros:jazzy-ros-base` + 소스 빌드 `nav2_msgs`) | 성공 |
| `python3 -m pytest pinky_fleet_agent/test` (ROS source 후) | 같은 컨테이너 | **221 passed, 0 skipped** (DDS loopback · 가짜 Nav2 액션 서버 포함) |
| `ros2 launch pinky_fleet_agent hybrid_robot.launch.xml launch_bringup:=False use_pose_fuser:=True` | 같은 컨테이너 | `/estop` 구독 = `drive_command_gate` · `/cmd_vel` 발행자 = `drive_command_gate` **1개** · `/cmd_vel_mission` 0.1 흘리는 중 `LaneCommand ESTOP` → `/estop true` → `/cmd_vel` 0.0 (5/5) · `drive_gate_status source=ESTOP` · `RESUME` → 0.1 |
| 같은 launch, `use_pose_fuser:=False` | 같은 컨테이너 | `amcl` 포함 localization 로드 시도, 진단 대상 9노드 · `True` 면 `amcl` 없이 8노드 + `pose_fuser` |
| 변경 파일 인코딩 | | 전부 UTF-8 (`git diff --numstat` 에 바이너리 행 없음, UTF-16 BOM 없음, `.py` 전부 compile) |

**아직 안 한 것 (정직하게):**

- **실제 Nav2 가 뜬 상태의 주행은 이 환경에서 못 쟀다.** 컨테이너에 Nav2 패키지가 없어 컴포저블 노드 10개가
  "Could not find requested resource" 로 로드되지 않았다 (로드 **시도** 목록이 설계와 같은 것까지만 확인).
  remap 이 맞는지는 `test_hybrid_launch.py` 가 launch 파일을 읽어 확인한다.
- **실물 핑키에서 돌려 보지 않았다.** 첫 기동 때 `ros2 topic info /cmd_vel` 발행자가 `drive_command_gate` 하나인지,
  `bt_navigator` 가 configure 를 통과하는지 먼저 본다.
- `use_pose_fuser:=True` 는 관제가 `/<robot>/pose_fix` 를 로봇 도메인으로 내려 줘야 한다. mini_project_2 의 브리지
  설정에는 아직 없다 (aerial_view 의 `pinky_lane_station/config/bridge_lane.yaml` 에만 있다 — 이 패치는 관제 쪽을
  건드리지 않았다). fix 가 `fix_timeout`(15 s) 동안 없으면 TF 가 끊겨 Nav2 가 멈춘다 — 의도된 동작이다.
- `test_hybrid_agent_loopback.py` 는 옮긴 그대로라 flake8 스타일 경고(한 줄 세미콜론 등)가 남아 있다. 동작과는 무관하다.

## 5. 가져오면서 고친 것 (원 저장소 대비)

| 파일 | 고친 것 | 왜 |
| :--- | :--- | :--- |
| `hybrid_agent_node.py` | FleetCommand `SET_SPEED` 분기 · 알 수 없는 명령 경고 되살림 | 원본에 없어 SET_SPEED 가 경고 없이 버려졌다 (`agent_node` 에는 있다) |
| `hybrid_agent_node.py` | 링크 복구 직후 은혜 기간에 **이동 명령만** 거른다 | 원본은 STOP · CANCEL 까지 버렸다 |
| `hybrid_agent_node.py` | LaneCommand `SET_SPEED` 처리 · `map_dir`/`map_name` 기본값과 보고 이름을 `agent_node` 와 같게 | |
| `hybrid_link_watch.py` | `command_timeout` 0 이면 LOST 를 내지 않는다 | 원본은 0(비활성)이어도 첫 poll 에 LOST 를 냈다 |
| `drive_command_gate.py` | 비상정지 값이 **바뀔 때만** 저장된 입력을 버린다 | 원본은 같은 값이 올 때마다 버려 `/estop false` 가 되풀이되면 주행 입력이 계속 지워졌다 |

수락 시험: `test_hybrid_agent_review.py` 의 `test_B2_*` · `test_LaneCommand_SET_SPEED_*` · `test_복구_직후에도_STOP_은_받고_GOTO_는_거른다`,
`test_hybrid_link_watch.py::test_zero_timeout_never_fires`, `test_drive_command_gate.py::test_repeated_same_estop_value_keeps_inputs`.
