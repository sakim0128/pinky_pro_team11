# pinky_pro_team11 — 핑키 프로 2대 멀티로봇 관제

관제 PC 한 대에서 [Pinky Pro](https://github.com/pinklab-art/pinky_pro) 2대를 동시에 제어한다.
GUI 맵 위에 목표를 찍으면 두 대가 동시에 출발하고, 좁은 길에서 서로 막히면
도메인 ID가 작은 쪽이 먼저 통과한 뒤 나머지 한 대가 재출발한다.

**upstream `pinky_pro` 는 한 줄도 수정하지 않는다.** 로봇마다 `ROS_DOMAIN_ID` 를 분리해
namespace / TF frame prefix / `/scan` 충돌 문제를 원천적으로 없앴고, 멀티로봇 로직은
전부 이 저장소의 별도 패키지로 얹었다.

## 미션 요구사항 대응

| 요구사항 | 구현 |
|---|---|
| 1. 핑키가 자신의 위치를 확인 | 로봇의 TF `map → base_footprint` (AMCL) → `RobotState` 로 발행 |
| 2. 각자 가야할 위치를 전달받음 | GUI 클릭 / `mission.yaml` → `FleetCommand(CMD_GOTO)` → 로컬 Nav2 `navigate_to_pose` |
| 3. 동시 출발 | GUI `[동시 출발]` 이 두 로봇에 같은 틱에 `CMD_GOTO` 발행 |
| 4. 만나면 멈춘 후 순차 출발 | 평소엔 Nav2 로컬 플래너 회피. 교착이면 coordinator 가 개입 |
| 5. 도메인 ID 작은 쪽 우선 | coordinator 가 `domain_id` 오름차순으로 leader / yielder 결정 |
| 주의 1. YAML 로 좌표·속도 전달 | `config/mission.yaml` (GUI 로드·저장, coordinator 가 읽음) |

## 구조

```
┌── ROS_DOMAIN_ID=10 ───────────┐   ┌── ROS_DOMAIN_ID=11 ───────────┐
│ pinky1                        │   │ pinky2                        │
│  pinky_bringup / sllidar      │   │  (동일)                        │
│  Nav2 (amcl, controller, ...) │   │                               │
│    ▲ navigate_to_pose 액션     │   │                               │
│    │ initialpose/set_parameters│   │                               │
│  pinky_fleet_agent ───────────┤   │  pinky_fleet_agent ───────────┤
│   pub /pinky1/state   (10Hz)  │   │   pub /pinky2/state           │
│   sub /pinky1/command         │   │   sub /pinky2/command         │
└───────────────┬───────────────┘   └───────────────┬───────────────┘
                │        (같은 Wi-Fi LAN / DDS)      │
┌───────────────▼───────────────────────────────────▼───────────────┐
│ ROS_DOMAIN_ID=0  (관제 PC)                                         │
│   domain_bridge  (config/bridge_fleet.yaml)                        │
│   coordinator_node  — 교착 감지 / 양보 / 재출발                     │
│   fleet_gui_node    — PyQt5. 맵 pgm·png 로컬 로드, 클릭→목표         │
└────────────────────────────────────────────────────────────────────┘
```

### 왜 에이전트 노드가 필요한가

`domain_bridge` 는 **YAML 설정만으로는 토픽만 브리지할 수 있고 서비스·액션은 브리지하지
못한다.** Nav2 의 `navigate_to_pose` 는 액션이고 속도 변경은 `set_parameters` 서비스다.
또 `domain_bridge` 는 메시지 안의 `frame_id` 를 재작성하지 않는다.

그래서 각 로봇 도메인 안에 경량 에이전트를 두고, 관제 PC 와는 **토픽 2개**(`state` /
`command`)로만 대화한다. 액션·서비스·TF 는 전부 로봇 도메인 내부에서만 쓰이므로
관제 PC 에는 TF 트리가 아예 필요 없다.

## 패키지

| 패키지 | 설치 대상 | 내용 |
|---|---|---|
| `pinky_fleet_msgs` | 로봇 2대 + 관제 PC | `RobotState`, `FleetCommand` |
| `pinky_fleet_agent` | 로봇 2대 | `agent_node`, `robot.launch.xml`, `nav2_params_fleet.yaml` |
| `pinky_fleet_station` | 관제 PC | `coordinator_node`, `gui_node`, `fake_state_pub`, launch |

`pinky_fleet_msgs` 는 **관제 PC 에도 반드시 빌드·소싱**해야 한다. `domain_bridge` 가
런타임에 메시지 타입서포트를 로드하기 때문이다.

## 설치

세 대 모두 저장소를 받되, **빌드하는 패키지가 다르다.**

저장소는 반드시 colcon 워크스페이스의 `src/` 아래에 둔다. colcon 이 워크스페이스
루트의 `src/` 를 훑어 `package.xml` 을 찾기 때문에, 홈 디렉터리에 그냥 받아두면
빌드되지 않는다. 저장소 루트의 `config/` 를 `pinky_fleet_station/setup.py` 가
참조하므로 **패키지 디렉터리만 따로 옮기지 말고 저장소를 통째로** 넣는다.

### 로봇 2대 (SSH, 계정 `pinky`)

기존 `pinky_pro` 워크스페이스를 그대로 쓴다. `pinky_fleet_agent` 의 launch 가
`pinky_bringup` / `pinky_navigation` 을 include 하므로 같은 워크스페이스여야 한다.

```bash
git clone -b mini_project_1 \
    https://github.com/sakim0128/pinky_pro_team11.git ~/pinky_pro/src/pinky_pro_team11
# 이미 홈에 받아 뒀다면:  mv ~/pinky_pro_team11 ~/pinky_pro/src/

cd ~/pinky_pro
colcon build --packages-select pinky_fleet_msgs pinky_fleet_agent
source install/setup.bash
```

`--packages-select` 는 필수다. 빼면 `pinky_pro` 전체와, 로봇에는 필요 없는
`pinky_fleet_station`(PyQt5 의존)까지 다시 빌드한다.

`~/.bashrc` 에 아래를 넣어 두면 접속할 때마다 손이 덜 간다 (핑키 2호는 `11`).

```bash
export ROS_DOMAIN_ID=10
source ~/pinky_pro/install/setup.bash
```

### 관제 PC

`pinky_pro` 가 필요 없으므로 별도 워크스페이스를 만든다.

```bash
sudo apt install ros-jazzy-domain-bridge python3-pyqt5 python3-numpy

mkdir -p ~/fleet_ws/src
git clone -b mini_project_1 \
    https://github.com/sakim0128/pinky_pro_team11.git ~/fleet_ws/src/pinky_pro_team11
# 이미 홈에 받아 뒀다면:  mv ~/pinky_pro_team11 ~/fleet_ws/src/

cd ~/fleet_ws
colcon build --packages-select pinky_fleet_msgs pinky_fleet_station
source install/setup.bash
```

`pinky_fleet_agent` 는 관제 PC 에서 빌드하지 않는다 (`pinky_bringup`,
`pinky_navigation` 의존). 같은 이유로 이 워크스페이스에서 `rosdep install` 을
인자 없이 돌리면 안 된다 — 없는 패키지를 찾다 실패한다.

### 맵 파일 (관제 PC)

**GUI 가 맵 이미지를 로컬 파일에서 직접 로드**하므로, 로봇에 올린 것과 같은 맵이
관제 PC 에도 있어야 한다.

```bash
mkdir -p ~/maps
scp pinky@<핑키1_IP>:~/pinky_pro/src/pinky_pro/pinky_navigation/map/pinklab.* ~/maps/
```

그다음 `~/fleet_ws/src/pinky_pro_team11/config/mission.yaml` 의 `map.yaml_path` 를
복사한 경로(예: `/home/sungah/maps/pinklab.yaml`)로 고친다.
`config/mission_deadlock_test.yaml` 도 마찬가지.

### 네트워크 전제

- 세 대가 같은 서브넷.
- `ROS_AUTOMATIC_DISCOVERY_RANGE` 가 `LOCALHOST` / `OFF` 이면 안 된다 (Jazzy 기본값 `SUBNET` 유지).
- `RMW_IMPLEMENTATION` 이 셋 다 같아야 한다.
- 관제 PC 도메인 `0` 은 ROS 기본값이라 같은 랜의 다른 ROS 프로세스가 섞일 수 있다.
  공용 실습망이면 `config/bridge_fleet.yaml` 의 `to_domain: 0` / `from_domain: 0` 을
  다른 값(예: 20)으로 바꾸고 관제 PC 도 같은 값으로 띄운다.

## 실행

로봇 2대를 먼저 띄운다. `domain_bridge` 는 퍼블리셔가 이미 떠 있을 때 QoS 매칭이 가장 안정적이다.

```bash
# 로봇 pinky1 (SSH)
export ROS_DOMAIN_ID=10
ros2 launch pinky_fleet_agent robot.launch.xml \
    robot_name:=pinky1 domain_id:=10 \
    map:=$HOME/pinky_pro/src/pinky_pro/pinky_navigation/map/pinklab.yaml

# 로봇 pinky2 (SSH)
export ROS_DOMAIN_ID=11
ros2 launch pinky_fleet_agent robot.launch.xml \
    robot_name:=pinky2 domain_id:=11 \
    map:=$HOME/pinky_pro/src/pinky_pro/pinky_navigation/map/pinklab.yaml

# 관제 PC
export ROS_DOMAIN_ID=0
ros2 launch pinky_fleet_station fleet.launch.xml \
    mission:=$HOME/fleet_ws/src/pinky_pro_team11/config/mission.yaml
```

### GUI 조작 순서

1. 각 로봇 카드의 `[초기위치 지정]` → 맵에서 실제 시작 위치를 **클릭 후 드래그**
   (드래그 방향이 로봇이 바라보는 방향). 위치가 맞게 표시되는지 확인.
2. `[목표 지정]` → 목표를 클릭 후 드래그. (또는 `[불러오기]` 로 `mission.yaml` 적용)
3. 속도를 바꾸려면 `최대 직진 / 최대 회전` 값을 넣고 `[속도 적용]`.
4. `[동시 출발]`.
5. `coordinator [NORMAL/YIELD]` 표시와 두 로봇 사이 거리선으로 개입 상황을 확인.

맵 조작: 휠 = 확대·축소, 우클릭(또는 가운데 버튼) 드래그 = 이동, `[화면에 맞추기]`.

## 실기 없이 검증하기

`fake_state_pub` 이 `mission.yaml` 을 읽어 가짜 로봇 2대를 관제 도메인에 직접 띄운다
(브리지·Nav2·하드웨어 불필요). 상대가 주행 중이면 멈추고, 멈춰 있으면 비켜 가는
단순 모델이라 교착 → 양보 → 재출발 전 과정이 그대로 재현된다.

```bash
export ROS_DOMAIN_ID=0
ros2 launch pinky_fleet_station fake_fleet.launch.xml \
    mission:=$HOME/fleet_ws/src/pinky_pro_team11/config/mission_deadlock_test.yaml \
    auto_start:=True

# 다른 터미널
ros2 topic echo /fleet/coordinator_status
```

`mission.yaml` 의 `state_topic` / `command_topic` 을 노출해 둔 덕분에 coordinator 와
GUI 는 브리지를 쓰든 가짜 로봇을 쓰든 똑같이 동작한다.

단위 테스트:

```bash
cd ~/fleet_ws/src/pinky_pro_team11 && python3 -m pytest pinky_fleet_station/test -q
```

## `config/mission.yaml`

```yaml
map:
  yaml_path: "..."             # 로봇에 올린 것과 동일한 맵 yaml (GUI 가 직접 로드)

defaults:
  max_linear_vel:  0.20        # m/s   → FollowPath.desired_linear_vel
  max_angular_vel: 1.50        # rad/s → velocity_smoother max_velocity[2]

coordinator:
  conflict_distance: 0.70      # m   이 거리 이내면 "근접"
  clear_distance:    1.00      # m   이 이상 벌어지면 양보 해제 (히스테리시스)
  stall_speed:       0.03      # m/s 미만이면 "정지"로 간주
  stall_duration:    3.0       # s   근접+정지가 이만큼 지속되면 교착 판정
  resume_timeout:   30.0       # s   양보 대기 최대 시간 (안전장치)
  cooldown:          2.0       # s   해제 직후 재진입 방지
  state_timeout:     2.0       # s   state 가 끊기면 개입하지 않음

robots:
  - name: pinky1
    domain_id: 10              # 작을수록 먼저 출발
    state_topic:   /pinky1/state
    command_topic: /pinky1/command
    color: "#ff5a7a"
    initial_pose: {x: 0.0, y: 0.0, yaw: 0.0}
    goal:         {x: 1.5, y: 2.0, yaw: 0.0}
    max_linear_vel: 0.20       # 생략 시 defaults 사용
```

`conflict_distance` 는 **두 로봇이 실제로 서로 막혀 멈추는 거리보다 커야 한다.**
작게 잡으면 교착이 감지되지 않는다. footprint 12cm 정사각 + `inflation_radius` 0.15m
기준으로 두 로봇의 인플레이션이 겹치기 시작하는 거리가 약 0.47m 이라 0.70m 을 기본값으로 두었다.

## 교착 회피 동작

평소 회피는 각 로봇의 Nav2 로컬 플래너가 한다 (서로를 라이다 장애물로 인식).
coordinator 는 Nav2 가 스스로 못 빠져나가는 상황만 잡아낸다.

```
NORMAL
  두 로봇 모두 주행 중(NAV_ACTIVE) + 둘 다 정지(|v| < stall_speed)
  + 거리 < conflict_distance  가 stall_duration 동안 지속
      → domain_id 가 큰 쪽에 CMD_STOP, 작은 쪽이 먼저 통과 → YIELD

YIELD
  아래 중 하나면 양보 해제 (CMD_RESUME) → NORMAL (cooldown 동안 재진입 차단)
    · 거리 > clear_distance
    · leader 의 주행이 끝남 (SUCCEEDED / ABORTED / CANCELED)
    · resume_timeout 초과 (안전장치)
```

안전장치 두 겹:
- **coordinator**: 한쪽 `state` 가 `state_timeout` 넘게 끊기면 즉시 `CMD_RESUME` 후 개입 중단.
  오래된 데이터로 멀쩡한 로봇을 세우는 것이 가장 위험하다.
- **에이전트**: HOLD 상태에서 `hold_watchdog`(기본 45초) 동안 아무 명령도 못 받으면
  스스로 HOLD 를 푼다. 관제 PC 나 브리지가 죽어도 로봇이 영원히 서 있지 않는다.

**알려진 한계**: leader 가 좁은 통로 위에서 목표에 도달해 멈춰 버리면 `NAV_ACTIVE` 조건이
깨져 교착 판정이 되지 않는다. 이 경우 yielder 는 Nav2 자체 회피와 recovery behavior 에
의존한다. 두 로봇의 목표를 서로의 경로를 막지 않는 곳에 잡는 것이 전제다.

## upstream `pinky_pro` 와의 관계

코드는 수정하지 않고, `pinky_fleet_agent/params/nav2_params_fleet.yaml` 이
upstream `nav2_params.yaml` 의 사본 + 아래 변경만 담는다 (`[fleet]` 주석으로 표시).
`bringup_launch.xml` 이 `params_file` 을 통째로 한 번만 읽어서 부분 오버레이가 불가능하기 때문이다.

1. `amcl.set_initial_pose` `true` → `false` — 초기 위치는 GUI / `mission.yaml` 이 준다.
2. `amcl.initial_pose: [0, 0, 0]` 제거 — nav2 규격(`{x:, y:, z:, yaw:}`)과 형식이 달라 어차피 무시되던 값.
3. `controller_server.use_sim_time` 하드코딩 제거 — launch 인자로만 결정.
4. `bt_navigator.robot_base_frame` `base_link` → `base_footprint` — costmap / behavior_server 와 통일.

`lifecycle_nodes_nav` 에 `smoother_server` 를 넣는 것은 `robot.launch.xml` 이 인자로 처리한다
(`bringup_launch.xml` 의 기본값이 이를 빠뜨려 `smoother_server` 가 unconfigured 로 방치된다).

도메인 분리 덕분에 아래 upstream 이슈들은 **손댈 필요가 없다**: costmap 의 `topic: /scan`
절대경로, `bringup.py` 의 `odom` / `base_footprint` 하드코딩, `pinky_params.yaml` 의
최상위 키가 `/**` 가 아닌 점 — 전부 로봇마다 도메인이 격리되어 충돌하지 않는다.

## 토픽 / 메시지

| 토픽 | 타입 | 방향 |
|---|---|---|
| `/pinkyN/state` | `pinky_fleet_msgs/msg/RobotState` | 로봇(도메인 N) → 관제(0), 10Hz |
| `/pinkyN/command` | `pinky_fleet_msgs/msg/FleetCommand` | 관제(0) → 로봇(도메인 N) |
| `/pinkyN/plan` | `nav_msgs/msg/Path` | 로봇 → 관제 (GUI 경로 표시, 선택) |
| `/fleet/coordinator_status` | `std_msgs/msg/String` (JSON) | coordinator → GUI (도메인 0 내부) |

`FleetCommand.command`: `CMD_GOTO` / `CMD_STOP` / `CMD_RESUME` / `CMD_CANCEL` /
`CMD_SET_INITIAL_POSE` / `CMD_SET_SPEED`.

`/tf`, `/map`, `/odom`, `/scan` 은 브리지하지 않는다. 두 로봇의 `odom` / `base_footprint`
프레임 이름이 같아 합치면 충돌하고, 관제 PC 는 map frame 스칼라 좌표만 쓰므로 필요가 없다.

## 트러블슈팅

| 증상 | 확인 |
|---|---|
| 관제 PC 에서 `/pinky1/state` 가 안 보임 | 로봇에서 `ROS_DOMAIN_ID=10 ros2 topic hz /pinky1/state`. 관제에서 `ROS_DOMAIN_ID=10 ros2 topic list` (브리지 없이도 보여야 정상) |
| 브리지 노드가 시작하자마자 죽음 | 관제 PC 에 `pinky_fleet_msgs` 미설치. `ros2 interface show pinky_fleet_msgs/msg/RobotState` |
| 명령이 로봇에 안 감 | 로봇에서 `ROS_DOMAIN_ID=10 ros2 topic echo /pinky1/command` |
| state 는 오는데 GUI 에 로봇이 안 보임 | `localized` 가 `false` (AMCL 미수렴). 초기 위치를 다시 지정 |
| 로봇이 맵의 엉뚱한 자리에 표시됨 | 두 로봇과 GUI 가 같은 맵 yaml 을 쓰는지 확인 |
| 교착인데 coordinator 가 개입하지 않음 | `conflict_distance` 가 실제 멈추는 거리보다 작음. `/fleet/coordinator_status` 의 `distance` 확인 후 키울 것 |
| 불필요하게 자주 멈춤 | `stall_duration` 을 늘리거나 `conflict_distance` 를 줄인다 |
| 속도 적용이 안 됨 | 로봇에서 `ros2 param get /controller_server FollowPath.desired_linear_vel`. Nav2 가 아직 activate 되기 전이면 건너뛴다 (로그 확인) |
