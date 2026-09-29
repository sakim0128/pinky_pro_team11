# Nav2 + 게이트 모드 (`hybrid_robot.launch.xml`) — 외부 비전 위치 · /cmd_vel 단일 발행자 · 레인 관제 수용

`mini_project_2` 위에 **새 파일만 얹은** 패치다(09-28 저녁 팀11 `main` `094e5d7` = `mini_project_integration` 까지 병합, §8). 기존 두 모드는 그대로다.

> **배포 전에 §6 "main 동기" 를 본다** — 에이전트가 원 저장소 main 의 어느 커밋까지 따라왔는지, 무엇으로 검증했는지.

| 모드 | launch | /cmd_vel 발행자 | 위치 |
| :--- | :--- | :--- | :--- |
| 차선 주행 (기존) | `lane_robot.launch.xml` | `lane_agent_node` | AMCL |
| Nav2 단일 목표 (기존) | `robot.launch.xml` | Nav2 | AMCL |
| **Nav2 + 게이트 (이 패치)** | `hybrid_robot.launch.xml` | `drive_command_gate` | AMCL 또는 외부 비전(`use_pose_fuser:=True`) |

세 launch 는 동시에 띄우지 않는다 — 셋 다 `/<robot_name>/command` 를 받는다.

## 1. 무엇이 바뀌었나

**고친 팀11 코드·설정 파일은 세 개이고 모두 한두 줄 추가다.** 나머지는 새 파일이다(팀11 관례 문서 `docs/integration/{status,source_versions}.md` 에는 §7 E-5 로 절을 덧붙였다).

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
| `test/test_route_chain.py` · `test_hybrid_agent_review.py` · `test_hybrid_agent_loopback.py` · `test_hybrid_review_0926_agent.py` · `test_hybrid_review_0926_agent2.py` | | 같은 저장소에서 import·경로만 바꿔 옮김 (+ §5 수락 시험 추가). `test_hybrid_review_0926_agent2.py` 의 월요일 시나리오 경로는 이 저장소 `pinky_lane_station/config/road_graph.yaml` + `pinky_lane_station.road_graph.RoadGraph` 로 만든다 |

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
TESTED_SHA = 1a961ebfac613c72e8df66f3d8af0b4c139095b0   (코드 마지막 커밋 — §6 main 동기)
git status --porcelain = (비어 있음)
base = mini_project_2 1f505cb88719c44391cc2781dcc3c5f09c9b7795
```
(첫 판 `9470111` 의 수치는 §6 표의 "동기 전" 열에 남겼다.)

| 무엇 | 어디서 | 결과 |
| :--- | :--- | :--- |
| `python3 -m pytest pinky_lane_station/test pinky_fleet_agent/test pinky_fleet_station/test` | ROS 없는 Python 3.11 | **313 passed, 132 skipped** (skip = ROS 가 필요한 에이전트 시험). 패치 전 베이스 215 passed, 4 skipped |
| `colcon build --packages-select pinky_fleet_msgs pinky_lane_msgs pinky_fleet_agent` + `ros2 interface show pinky_lane_msgs/msg/PoseFix` | ROS 2 Jazzy 컨테이너 (`ros:jazzy-ros-base` + 소스 빌드 `nav2_msgs`) | 성공 |
| `python3 -m pytest pinky_fleet_agent/test` (ROS source 후) | 같은 컨테이너 | **307 passed, 0 skipped** (DDS loopback · 가짜 Nav2 액션 서버 포함) |
| `ros2 launch pinky_fleet_agent hybrid_robot.launch.xml launch_bringup:=False use_pose_fuser:=True` | 같은 컨테이너 | `/estop` 구독 = `drive_command_gate` · `/cmd_vel` 발행자 = `drive_command_gate` **1개** · `/cmd_vel_mission` 0.1 흘리는 중 `LaneCommand ESTOP` → `/estop true` → `/cmd_vel` 0.0 (5/5) · `drive_gate_status source=ESTOP` · `RESUME` → **0.0 유지**(Nav2 가 없어 취소를 확인할 수 없다 — §6 ⚠️; 첫 판 `9470111` 에서는 0.1 로 풀렸다) |
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
- `test_hybrid_review_0926_agent2.py` 의 ROS 부분은 옆 패키지 `pinky_lane_station` 을 import 한다 — 이 저장소 체크아웃 전체가 있어야 돈다(`pinky_fleet_agent` 만 떼어 내면 collection 에서 ImportError).

## 5. 가져오면서 고친 것 (원 저장소 대비)

| 파일 | 고친 것 | 왜 |
| :--- | :--- | :--- |
| `hybrid_agent_node.py` | FleetCommand `SET_SPEED` 분기 · 알 수 없는 명령 경고 되살림 | 원본에 없어 SET_SPEED 가 경고 없이 버려졌다 (`agent_node` 에는 있다) |
| `hybrid_agent_node.py` | 링크 복구 직후 은혜 기간에 **이동 명령만** 거른다 | 원본은 STOP · CANCEL 까지 버렸다 |
| `hybrid_agent_node.py` | LaneCommand `SET_SPEED` 처리(속도 상한만 바꾸고, 다른 LaneCommand 처럼 해제 문(§6)도 지난다) · `map_dir`/`map_name` 기본값과 보고 이름을 `agent_node` 와 같게 | |
| `hybrid_link_watch.py` | `command_timeout` 0 이면 LOST 를 내지 않는다 | 원본은 0(비활성)이어도 첫 poll 에 LOST 를 냈다 |
| `drive_command_gate.py` | 비상정지 값이 **바뀔 때만** 저장된 입력을 버린다 | 원본은 같은 값이 올 때마다 버려 `/estop false` 가 되풀이되면 주행 입력이 계속 지워졌다 |

수락 시험: `test_hybrid_agent_review.py` 의 `test_B2_*` · `test_LaneCommand_SET_SPEED_*` · `test_복구_직후에도_STOP_은_받고_GOTO_는_거른다`,
`test_hybrid_link_watch.py::test_zero_timeout_never_fires`, `test_drive_command_gate.py::test_repeated_same_estop_value_keeps_inputs`.

## 6. main 동기 — 원 저장소 에이전트 커밋을 따라온 기록 (배포 전 확인)

원 저장소(rkd1rjs2/robot_mini_project_pinky) main 의 `agent_node.py` · `route_chain.py` 가 고쳐지면 이 브랜치의
`hybrid_agent_node.py` · `route_chain.py` 도 같은 내용으로 맞춘다. 이 표의 마지막 행이 지금 상태다.

| 날짜 | 원 저장소 커밋 (옮긴 것) | 브랜치 커밋 | 무엇 |
| :--- | :--- | :--- | :--- |
| 2026-09-26 | main `dd3b4e0` 시점의 `agent_node.py` · `route_chain.py` | `457659e` (코드 `9470111` 까지) | 첫 이식 (§1) |
| 2026-09-27 | `6f28c63` · `d0352f4` · `2750fe0` · `46757f9` (+ 시험만 고친 `c541be9` · `6b3c3eb`) | `02c171d` + **`1a961eb`** | 아래 (§6.1) |

### 6.1 2026-09-27 · `02c171d` · `1a961eb` — 원 저장소 `REQ_20260927_CLOUD_SESSION_HYBRID_REPORT.md`

| 원 커밋 | 브랜치 파일 | 무엇이 바뀌나 (운영자가 알아야 할 것) |
| :--- | :--- | :--- |
| `6f28c63` | `route_chain.py` | **START 전 위치는 진행도를 밀지 않는다.** START 때 그 순간 위치로 경로 처음부터 다시 투영한다(1 s 넘은 위치는 안 쓴다). 좌표 전환 뒤 옛 좌표계 위치가 진행도를 앞으로 밀어 첫 허가보다 뒤가 되어 영영 대기하거나, 움직이지 않고 도착을 보고하던 것(원 저장소 E2E-1)을 고친다 |
| `d0352f4` | `route_chain.py` · `hybrid_agent_node.py` | **같은 Route 재전달은 무시**(번호·waypoint·goal·발행 시각이 전부 같을 때) — 도메인 브리지만 재기동해도 달리던 임무가 IDLE 로 서지 않는다. 발행 시각이 다르면(코디네이터 재시작) 새 경로 |
| `2750fe0` | `hybrid_agent_node.py` | **해제 문 하나**: 정지·ESTOP·링크유실 래치를 푸는 네 길(LaneCommand RESUME·START · FleetCommand RESUME · 링크 회복)이 Nav2 에 스탬프 CancelGoal 을 내고 **확인된 뒤에만** `/estop false`·goto 를 낸다. 확인이 없으면(시간초과·거부·서비스 없음) 래치를 쥔 채 1 Hz 재취소, LaneStatus 사유 `해제 보류 — Nav2 취소 미확인`, 진단 `agent.release_hold`. 우리 goal 은 id 를 기억하고 플릿 통제 중 남의 goal 은 id 로 취소. 정지 중 GOTO 는 거절(진단 `refused`). `navigate_through_poses` · `follow_waypoints` 도 본다. odom 1 s 끊김 · Nav2 상태 모름 = 움직이는 중으로 본다 |
| `46757f9` | `hybrid_agent_node.py` | 문이 선 채 다시 정지했다 풀리면 문을 **새로** 세운다(옛 스탬프의 확인으로 열지 않는다) |

⚠️ **운영에 바로 닿는 것**: 이제 RESUME 은 Nav2 가 취소를 확인해 줄 때만 로봇을 풀어 준다. **Nav2 가 죽어 있거나
아직 안 떴으면 RESUME 을 보내도 `/cmd_vel` 은 0 에 머문다**(사유 `해제 보류 — Nav2 취소 미확인`). Nav2 를 살리면
1 Hz 재취소가 확인되어 열린다. 아래 컨테이너 점검의 "RESUME 뒤 0.0" 이 그 동작이다.

**어떻게 옮겼나**: 3-way merge(공통 조상 = 원 저장소 `agent_node.py`@`dd3b4e0`, 이쪽 = `hybrid_agent_node.py`@`e6ee635`,
저쪽 = `agent_node.py`@main `b545c84`). 충돌 2곳(import 한 줄, 로그 문구 + 이 브랜치의 SET_SPEED 처리). 옮긴 뒤
`hybrid_agent_node.py` 와 main `agent_node.py` 의 차이는 §1·§5 의 브랜치 적응 115줄뿐이다.

**이식 뒤 적대 검토** (6 관점 × 독립 검토 → 발견마다 반박 검증 2명): 확정 1건 — 이 브랜치의 SET_SPEED 조기 `return` 이
새로 옮긴 해제 문(`_release_gate`)보다 위에 남아, 링크 유실만 걸린 로봇이 SET_SPEED 한 발로 취소 확인 없이 풀릴 수 있었다
(지금 코디네이터는 LaneCommand SET_SPEED 를 보내지 않아 잠재 결함). **`1a961eb`** 에서 return 을 없애고 시험
`test_LaneCommand_SET_SPEED_로_링크가_회복돼도_같은_해제_문을_지난다` 로 잠갔다(return 을 되살리면 빨강).
반박된 것: `unique_identifier_msgs` 미선언(main 과 같음) · 지도 이름 realpath 정규화(첫 이식부터 있던 의도) · 주석의 원 저장소
문서 이름(출처는 파일 머리에 밝혀 둠).
팀11 파일 변경은 여전히 세 개(`pinky_lane_msgs/CMakeLists.txt` · `pinky_fleet_agent/setup.py` · `package.xml`), 새 ROS 파라미터 없음.

**검증** (`TESTED_SHA = 1a961ebfac613c72e8df66f3d8af0b4c139095b0`, `git status --porcelain` 비어 있음)

| 무엇 | 명령 · 환경 | 동기 전 (`9470111`) | 동기 후 (`1a961eb`) |
| :--- | :--- | :--- | :--- |
| 팀11 전체 | `python3 -m pytest pinky_lane_station/test pinky_fleet_agent/test pinky_fleet_station/test -q` · ROS 없는 Python 3.11 | 297 passed, 62 skipped | **313 passed, 132 skipped** |
| 에이전트 (ROS) | `colcon build --packages-select pinky_fleet_msgs pinky_lane_msgs pinky_fleet_agent` 뒤 `python3 -m pytest pinky_fleet_agent/test -q` · ROS 2 Jazzy 컨테이너 | 221 passed, 0 skipped | **307 passed, 0 skipped** |
| B-4 런타임 | `hybrid_robot.launch.xml launch_bringup:=False use_pose_fuser:=True`, Nav2 실행 패키지 없는 컨테이너 | ESTOP → `/cmd_vel` 0.0, RESUME → 0.1 | ESTOP → `/cmd_vel` 0.0, **RESUME → 0.0 유지** + `해제 보류 — Nav2 취소 미확인` 1 Hz (위 ⚠️, Nav2 가 없어서 확인할 수 없다 — 설계대로) |
| 인코딩·구조 | `git diff --numstat team11/mini_project_2..HEAD` 바이너리 행 0 · 바뀐 `.py` 전부 `py_compile` 통과 · `_on_command`/`_on_lane_command` 는 각자 한 메서드 | | 통과 |

**되돌림 변이** (그 커밋을 되돌린 코드로 같은 시험을 돌리면 빨개져야 한다 — 시험이 규약을 잡고 있다는 증거)

| 되돌린 것 | 돌린 시험 | 결과 |
| :--- | :--- | :--- |
| `6f28c63` (route_chain) | `test_hybrid_review_0926_agent2.py` · `test_route_chain.py` (ROS 없음) | 8 failed |
| `d0352f4` (route_chain) | 같음 | 7 failed |
| `d0352f4` (hybrid_agent_node `_on_route` 부분) | `test_hybrid_review_0926_agent2.py` (ROS) | 1 failed (`F2_에이전트_브리지_재기동…`) |
| `46757f9` | `test_hybrid_review_0926_agent.py` · `test_hybrid_agent_review.py` (ROS) | 2 failed |
| `2750fe0` 이전 판(`e6ee635` 의 `hybrid_agent_node.py`) | `test_hybrid_review_0926_agent.py` (ROS) | 52 failed (전부) |
| `1a961eb` 되돌림 (SET_SPEED 조기 return 복원) | `test_hybrid_agent_review.py` (ROS) | 1 failed (새 시험만) |

**수락 기준 ↔ 시험** (원 저장소 REQ §3)

| 기준 | 브랜치 시험 |
| :--- | :--- |
| 1 E2E-1 | `test_hybrid_review_0926_agent2.py::test_E2E1_*` (RouteChain 9건 + 에이전트 월요일 순서 3건) |
| 2 F2 | `test_hybrid_review_0926_agent2.py::test_F2_*` (RouteChain 6건 + 에이전트 4건) |
| 3 A-1~A-4 · 46757f9 | `test_hybrid_review_0926_agent.py::test_A1_*` `test_A2_*` `test_A3_*` `test_A4_*` `test_A12_*` `test_재검_R2_*`, `test_hybrid_agent_review.py::test_S2_취소_응답이_1초_없어도_해제하지_않고_1Hz_로_다시_취소한다__Nav2_무응답` |
| 4 A-5 · A-8 | `test_hybrid_review_0926_agent.py::test_A5_*` `test_A8_*`, `test_hybrid_agent_loopback.py` `--scenario-gate0926` · `--scenario-foreign0926` |
| 5 기존 시험 | 위 표 307 / 313 |

**아직 안 한 것**: §4 와 같다 — 실제 Nav2 가 뜬 환경(가상 팜·실물)에서의 주행은 이 환경에서 못 쟀다. 실물 첫 기동 때
RESUME 뒤 `ros2 topic echo /pinky1/lane_status` 의 `state_reason` 이 `해제 보류 — Nav2 취소 미확인` 에 머물면 Nav2
(`bt_navigator` · `navigate_to_pose/_action/cancel_goal`)가 살아 있는지 먼저 본다.

## 7. export 정리 (E-1~E-7) — `relay_station/` PR 전 수정 (2026-09-28)

원 저장소 `docs/REQ_20260928_CLOUD_SESSION_EXPORT_FIXES.md` 의 E-1~E-7. 브랜치 커밋: 병합 **`e2ca87c`** + 정리 **`d31e75a`**.

| # | 무엇 | 어떻게 · 확인 |
| :-- | :-- | :-- |
| E-1 | upstream 최신에 다시 맞춤 | `git merge team11/mini_project_2`(`5faddf1`, PR #3 live 웹·상부 추적기·map5) → 충돌 0 · `git merge-base HEAD team11/mini_project_2` = `5faddf1` |
| E-2 | 외부 사이트를 가리키는 낱말 제거 | 19곳을 중립어로("외부 사이트" · "다른 배포 체계" · "외부 가공 단"). `relay_station/tests/test_team11_export.py::test_외부_사이트를_가리키는_낱말이_없다` 가 잠근다(낱말을 넣으면 빨강 — 확인) |
| E-3 | `index.html` 접속 안내 | 자리표시자 IP 링크 세 개를 지우고 **지금 연 주소**(`location.origin`) + "원격(`:18081`)은 보기 전용" 한 줄. `test_page_honesty` 의 단언은 그 뜻으로 |
| E-4 | 원 저장소 main 최신 | export 기준 `c784497` → **`f8fc97d`** (`8b68b6d`: `/api/status` 에 `view_only` + `tests/test_review_0928_view_only.py`). `relay_station/README.md` 첫 줄에 기준 커밋 |
| E-5 | 팀11 관례 문서 | `docs/integration/relay_station.md`(실행법 · 팀11 관제와의 관계 · 영상 공유 `/api/sources` · `/video_feed?src=` · map4/map5) · `status.md` 09-28 절 · `source_versions.md` 기준 커밋 |
| E-6 | map4 ↔ map5 | 코드 변경 없음. `relay_station/fleet/config/profiles/profiles.yaml` 주석 + `relay_station.md`·`README.md` 에 "map5 프로파일은 팀 결정 뒤" |
| E-7 | skip 4건 | 사유를 이 저장소 기준(대사 대상 부재 · `hybrid_robot.launch.xml` · `pinky_fleet_agent`)으로 다시 적고, README 에 skip 22 = 환경 18 + 사유 4 를 적어 통과로 세지 않게 |

**검증** (`TESTED_SHA = d31e75a`, `git status --porcelain` 비어 있음, ROS 2 Jazzy 컨테이너 · `pinky_fleet_msgs pinky_lane_msgs pinky_fleet_agent` colcon build 뒤 — **병합 전 기록**이다. 팀11 `main` `094e5d7` 병합 뒤 수치와 diff 기준은 §8)

| 대상 | 명령 | 결과 |
| :-- | :-- | :-- |
| 중계 | `cd relay_station && python3 -m pytest tests -q` | **1425 passed, 22 skipped** (동기 전 export `d32ff07`: 1419 / 22) |
| 에이전트 | `python3 -m pytest pinky_fleet_agent/test -q` | **307 passed, 0 skipped** (§6 과 같다) |
| 팀11 station·lane | `python3 -m pytest pinky_lane_station/test pinky_fleet_station/test -q` · ROS 없는 Python 3.11 | **153 passed, 4 skipped** (PyQt5 · ultralytics · mcap_ros2 없음 — 팀11 원본과 같은 skip) |
| 팀11 전체(ROS 없음) | `python3 -m pytest pinky_lane_station/test pinky_fleet_agent/test pinky_fleet_station/test -q` | 332 passed, 132 skipped (skip = ROS 필요 에이전트 시험 + 위 4) |
| 공개 규칙 | `python3 -m pytest relay_station/tests/test_team11_export.py -q` | 7 항목 (키 기본값 없음 · 키 없으면 거절(소스·메서드) · 사설 주소 자리표시자뿐 · 개인 경로 없음 · 낱말 없음 · COLCON_IGNORE) |
| 구조 | `git diff --name-status team11/mini_project_2..HEAD` | 팀11 코드·설정 변경은 여전히 `pinky_lane_msgs/CMakeLists.txt` · `pinky_fleet_agent/setup.py` · `package.xml` 세 개(M). 문서는 E-5 가 요구한 `docs/integration/{status,source_versions}.md` 두 개에 **절을 덧붙였다**(기존 내용 그대로). 나머지 전부 A. 바이너리 취급 행 0, 바뀐 `.py` 전부 compile |

⚠️ ROS 2 Jazzy 컨테이너의 pytest 7.4 로 `pinky_lane_station/test` · `pinky_fleet_station/test` 를 디렉터리로 주면 module 수준
`importorskip`(PyQt5 등)에서 수집이 멈춰 "1 skipped" 만 나온다 — 팀11 원본 `5faddf1` 에서도 같다(이 브랜치 탓이 아니다). 그래서 이 두 묶음의
수치는 ROS 없는 pytest 8 에서 잰 것이다(팀11 README 도 "테스트 (ROS 불필요)" 다).

## 8. 팀11 `main` 병합 — PR 대상 `mini_project_integration` (2026-09-28 저녁)

팀11 에 새 브랜치 **`mini_project_integration`** 이 생겼다(= `main` `094e5d7`). `094e5d7` 은 `mini_project_2` `5faddf1` 위에
`mini_project_1`(PR #2 · #4)을 얹은 것이다: `pinky_fleet_sim` 패키지(가제보 2대, `/pinky1/...` 네임스페이스) · `params/nav2_params_fleet.yaml`
실기 튠(`max_obstacle_height` 0.2 · `raytrace_max_range` 2.5) · `agent_node` 의 Nav2 파라미터 감사(`param_audit.py`, `GetParameters`) ·
`agent.launch.xml` 인자 7개(`map_server` `controller_server` `velocity_smoother` `nav2_params_file` `audit_*`) · `robot.launch.xml` 이
`nav2_params_file` 전달 · `test_param_audit.py` · `pinky_fleet_station/test/test_sim_*.py`. PR 대상을 `mini_project_2` → **`mini_project_integration`**
으로 바꾸고 병합했다: 커밋 **`f2d4e5b`**, 충돌 0, `git merge-base HEAD team11/mini_project_integration` = `094e5d7`.

상류 변경이 이 브랜치에 미치는 것(코드 변경 없음 — 전부 기록·후속):

| 상류 변경 | 하이브리드 스택 |
| :-- | :-- |
| `nav2_params_fleet.yaml` 튠 | `hybrid_robot.launch.xml` 의 `params_file` 기본값이 **같은 파일**이라 그대로 적용된다. `nav2_gated_navigation.launch.xml` 은 노드 이름(`controller_server` · `velocity_smoother` · `behavior_server` · `bt_navigator`)을 바꾸지 않고 `cmd_vel` 출력만 `cmd_vel_mission` 으로 돌린다 |
| `agent.launch.xml` 새 인자 | `hybrid_agent.launch.xml` 은 받지 않는다. `hybrid_agent_node` 는 `controller_server` · `velocity_smoother` · `follow_path_plugin` 을 파라미터로 선언하고(기본값 = 상대 이름) `_apply_speed` 가 그 이름으로 `SetParameters` 를 부른다 — 상류 `agent_node` 의 같은 부분은 바뀌지 않았다 |
| Nav2 파라미터 감사(`param_audit`) | 하이브리드에는 **없다**. 같은 params 파일·같은 노드 이름이라 감사 규칙은 그대로 옮길 수 있다 — **후속 PR** (이 PR 범위 밖) |
| `pinky_fleet_sim` 네임스페이스 로봇 | 하이브리드 스택은 `/cmd_vel` · `/estop` · `/drive_gate_status` 가 절대 토픽이다 — 시뮬의 `/pinky1/...` 로봇에서는 뜨지 않는다(실기 전용). 필요하면 후속 |
| `agent_node` 가 `import yaml` | `pinky_fleet_agent/package.xml` 에 `python3-yaml` 이 없다 — 상류 몫(PR 본문에 제안). 이 브랜치의 새 파일은 yaml 을 import 하지 않는다 |

**검증** (`TESTED_SHA = f2d4e5b`, `git status --porcelain` 비어 있음, ROS 2 Jazzy 컨테이너 · `pinky_fleet_msgs pinky_lane_msgs pinky_fleet_agent` colcon build 뒤)

| 대상 | 명령 | 결과 |
| :-- | :-- | :-- |
| 에이전트 | `python3 -m pytest pinky_fleet_agent/test -q` | **319 passed, 0 skipped** (§7 의 307 + 상류 `test_param_audit.py` 12) |
| 중계 | `cd relay_station && python3 -m pytest tests -q` | **1425 passed, 22 skipped** (§7 과 같다) |
| 팀11 station·lane | `python3 -m pytest pinky_lane_station/test pinky_fleet_station/test -q` · ROS 없는 Python 3.11 | **188 passed, 24 skipped** (§7 의 153/4 + 상류 `test_sim_*` — skip 은 PyQt5 · ultralytics · mcap_ros2 · `pinky_description`/xacro 없음, `main` 원본과 같다) |
| 팀11 전체(ROS 없음) | `python3 -m pytest pinky_lane_station/test pinky_fleet_agent/test pinky_fleet_station/test -q` | 379 passed, 152 skipped |
| 공개 규칙 | `python3 -m pytest relay_station/tests/test_team11_export.py -q` | 6 passed, 1 skipped (ROS 없음 → 진짜 메서드 시험 skip; 컨테이너에서 7) |
| 런치 수락 | `hybrid_robot.launch.xml launch_bringup:=False` 뒤 토픽 검사(§4 와 같은 절차) | `/cmd_vel` 발행자 = `drive_command_gate` 하나 · `LaneCommand ESTOP` 뒤 `/cmd_vel` 5표본 0.0 · `source=ESTOP` · RESUME 뒤 0.0(Nav2 없음 → 해제 문 fail-closed, §6 과 같다) |
| 구조 | `git diff --diff-filter=M --name-only 094e5d7 HEAD` | 팀11 파일 수정은 여전히 `pinky_lane_msgs/CMakeLists.txt` · `pinky_fleet_agent/setup.py` · `package.xml` + `docs/integration/{status,source_versions}.md` 절 추가 — 5개 |

**추가 (09-28 밤, 제어권 정책)** — 관제가 원 저장소 main `1eed3f8`(팀원 노트북도 중계를 거쳐 움직이는 명령을 낸다 · 기본 닫힘 · `configs/control_allow.json`)을
팀11 `mini_project_integration` 에 **`71faa4f`** 로 직접 push 했다(`relay_station/` 만 · 시험 +20). 재측정: `cd relay_station && python3 -m pytest tests -q` →
처음 **1 failed**(`test_calibration_http.py::test_게이트_없는_POST_경로가_새로_생기지_않았다` — 시험이 새 문 `_deny_if_cannot_move`/`CONTROL_POLICY` 를 몰라
`/api/fleet/*` 5 + `/api/control/*` 2 를 무게이트로 잡음, 원 저장소 main 에서도 같다) → 시험만 고쳐 **1445 passed, 22 skipped**. 에이전트·station 시험은 이 커밋이
건드리지 않는다(319 · 188/24 그대로).

## 9. 중계 브리지 개편과 로봇 쪽 (2026-09-29)

주 대시보드가 팀11 live 웹이 되면서 로봇 쪽에서 바뀐 것은 하나다: `hybrid_robot.launch.xml` 이 팀11 `camera_node` 를 띄운다
(`use_camera` 기본 True · `camera_orient` rot180 · `camera_fps` 10 — `lane_agent.launch.xml` 과 같은 값, `test_hybrid_launch.py` 가 대조).
`/pinkyN/amcl_pose` 는 원래 `hybrid_agent_node` 가 다시 내고 있었고, 중계 브리지가 이제 그것과 카메라를 관제 도메인에 올린다.
게이트·에이전트 코드는 그대로다(에이전트 시험 320 passed · 런치 수락 동일). 중계 쪽 기록은 `docs/integration/relay_station.md`.
