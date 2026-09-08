# 검증 절차 (L0 → L4)

실물 두 대에서 바로 돌리면 실패했을 때 원인이 다섯 군데 중 어디인지 알 수 없다
(상태머신 / 도메인 배선 / 브리지 / Nav2 / 로봇). 그래서 **한 번에 하나씩만 새로 도입**한다.

| 레벨 | 새로 도입하는 것 | 실패하면 원인이 확정되는 것 |
|---|---|---|
| L0 | 상태머신 | 순서·타임아웃·취소 로직 |
| L1 | ROS + 다중 도메인 프로세스 | `rclpy.init(domain_id=)` 배선 |
| L2 | domain_bridge | 브리지 설정 / QoS |
| L3 | Nav2 (시뮬) | 액션·좌표·AMCL |
| L4 | 실물 | 하드웨어·네트워크 |

---

## L0 — 상태머신 (ROS 불필요)

```bash
cd pinky_fleet
PYTHONPATH=. python3 -m pytest test -q
PYTHONPATH=. python3 -m pinky_fleet.fleet_master --dry-run
```

**통과 기준**

```
[STATE] WAIT_NAV2 → GOING_TO_GOAL(pinky1) → GOING_HOME(pinky1) → SETTLING
      → GOING_TO_GOAL(pinky2) → GOING_HOME(pinky2) → SETTLING → DONE
[RESULT] 성공 — 두 로봇 모두 목적지 왕복 완료
```

**이상 경로도 반드시 확인한다** (여기서 안 잡으면 실물에서 잡게 된다):

```bash
# 1) 중간 실패 -> ABORT 로 빠지고 뒤 로봇은 출발하지 않아야 한다
PYTHONPATH=. python3 -m pinky_fleet.fleet_master --dry-run --inject-fail pinky1:home

# 2) 응답 없음 -> 타임아웃으로 취소되어야 한다 (짧은 타임아웃 설정으로)
sed -e 's/goal_timeout_sec: 90.0/goal_timeout_sec: 8.0/' \
    -e 's/home_timeout_sec: 90.0/home_timeout_sec: 8.0/' \
    config/mission.yaml > /tmp/mission_fast.yaml
PYTHONPATH=. python3 -m pinky_fleet.fleet_master --dry-run \
    --inject-hang pinky2:goal --config /tmp/mission_fast.yaml
echo "종료코드 $?"      # 실패 시 1
```

기대: 1)은 `pinky1 pinky1:home 실패`, 2)는 `pinky2 pinky2:goal 실패: 타임아웃 8s 초과`.
두 경우 모두 뒤 단계로 넘어가지 않고 종료코드가 1 이어야 한다.

---

## L1 — 다중 도메인 배선 (turtlesim)

PC 한 대에서 turtlesim 두 마리를 서로 다른 도메인에 띄운다.
**로봇도 Nav2 도 없이** "한 스크립트가 두 도메인을 순서대로 제어"만 확인한다 (20강 실습과 동일).

```bash
# 터미널1
export ROS_DOMAIN_ID=10 && export ROS_LOCALHOST_ONLY=0 && ros2 run turtlesim turtlesim_node
# 터미널2
export ROS_DOMAIN_ID=11 && export ROS_LOCALHOST_ONLY=0 && ros2 run turtlesim turtlesim_node
# 터미널3 (이 터미널의 ROS_DOMAIN_ID 는 무관하다 — 코드가 지정한다)
ros2 run pinky_fleet fleet_master \
    --config $(ros2 pkg prefix pinky_fleet)/share/pinky_fleet/config/mission_turtlesim.yaml \
    --backend turtlesim --no-console --goal 9.0 5.5 0 -y
```

**통과 기준**
- 1번 창의 거북이만 먼저 (9.0, 5.5) 로 갔다가 시작점으로 돌아온다
- 그 다음에야 2번 창의 거북이가 같은 동작을 한다
- **두 마리가 동시에 움직이면 실패** — 순차 제어가 안 되고 있는 것

**여기서 실패하면**: 도메인 분리(`rclpy.init(domain_id=)`)나 Queue 배선 문제다.
브리지/Nav2 는 아직 등장하지도 않았다.

---

## L2 — domain_bridge

### 2-1. 브리지 자체 (21강 재현)

```bash
# 터미널1 (도메인 10)
ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0 ros2 run demo_nodes_cpp talker
# 터미널2 (도메인 20)
ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0 ros2 run demo_nodes_py listener
# 터미널3
cat > /tmp/t.yaml <<'Y'
name: t
from_domain: 10
to_domain: 20
topics:
  chatter:
    type: std_msgs/msg/String
Y
ROS_LOCALHOST_ONLY=0 ros2 run domain_bridge domain_bridge /tmp/t.yaml
```

listener 에 `I heard: Hello World` 가 찍히면 브리지 자체는 정상이다.

### 2-2. 실제 설정 (로봇 또는 L3 시뮬이 떠 있는 상태에서)

```bash
ros2 launch pinky_fleet fleet_bridge.launch.py
```

```bash
ROS_DOMAIN_ID=20 ros2 topic list
#   /map
#   /pinky1/amcl_pose
#   /pinky2/amcl_pose      <- 이 3개가 보여야 한다

ROS_DOMAIN_ID=20 ros2 topic hz /pinky1/amcl_pose
ROS_DOMAIN_ID=20 ros2 topic echo /map --once | head -5
```

**통과 기준**: 세 토픽이 모두 보이고, `amcl_pose` 가 로봇이 움직일 때 갱신된다.

**여기서 실패하면**
- 토픽이 아예 안 보임 → `ROS_LOCALHOST_ONLY`, 도메인 번호, 로봇 연결
- 토픽 이름은 보이는데 데이터가 안 옴 → **QoS 불일치**.
  `ros2 topic info /amcl_pose --verbose` 를 소스 도메인에서 실행해 발행자 QoS 를 확인하고
  `mission.yaml` 재생성본과 비교한다

---

## L3 — Nav2 통합 (Gazebo 시뮬 2대)

PC 한 대에서 시뮬레이터를 **2개** 띄운다. 도메인만 나누면 안 된다 —
**Gazebo(gz) transport 는 ROS 도메인과 완전히 별개**라서, `GZ_PARTITION` 을 나누지 않으면
두 시뮬이 같은 월드를 공유해 서로의 로봇을 조종한다.

```bash
# 터미널1 — 시뮬 1
export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0 GZ_PARTITION=sim1
ros2 launch pinky_gz_sim launch_sim.launch.xml
# 터미널2 — 시뮬 1 의 Nav2
export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0 GZ_PARTITION=sim1
ros2 launch pinky_navigation bringup_launch.xml map:=my_map.yaml use_sim_time:=true

# 터미널3,4 — 시뮬 2 (ROS_DOMAIN_ID=11, GZ_PARTITION=sim2 로 동일하게)

# 터미널5 — 브리지
ros2 launch pinky_fleet fleet_bridge.launch.py
# 터미널6 — 관제 화면
ros2 launch pinky_fleet fleet_view.launch.py
# 터미널7 — 미션
ros2 run pinky_fleet fleet_master
```

**통과 기준**
- RViz 한 화면에 맵 + 로봇 2대 마커 + 상태 텍스트
- Publish Point 2회 클릭 → 목적지 마커(주황) 표시 → Enter → 1호기만 출발
- 1호기 복귀 후 2호기 출발, 최종 `DONE`

**PC 부하가 커서 gz 2개가 버거우면 (L3.5)**: 시뮬 1대(도메인 10) + 실물 1대(도메인 11) 혼합.
설계상 두 워커는 서로를 모르므로 섞어도 그대로 동작한다.

---

## L4 — 실물 2대

순서를 지킨다. 각 단계에서 멈출 수 있어야 다음으로 간다.

1. **바퀴를 띄운 채** 1~4단계(RUNBOOK)까지만 하고 `fleet_master` 는 실행하지 않는다.
   RViz 에 마커 2개가 뜨는지만 확인한다. → 관제 평면 확인 완료
2. 여전히 바퀴를 띄운 채 `fleet_master` 실행 → `WAIT_NAV2` 통과, `WAIT_GOAL` 도달 확인.
   여기서 Ctrl-C. → 준비 단계 확인 완료
3. 바닥에 내리고 **출발지에서 1 m 이내**의 안전한 지점을 목적지로 찍어 전체 미션 1회.
4. **취소 확인**: 3을 다시 돌리고 주행 중 Ctrl-C → **두 로봇이 실제로 멈추는지** 확인.
   이게 안 되면 실제 목적지로 넘어가지 않는다.
5. 실제 목적지로 본 미션.

**체크리스트**

- [ ] 두 로봇 배터리 7V 이상
- [ ] 두 로봇에 **같은 맵 파일**
- [ ] `mission.yaml` 의 home 이 실측값
- [ ] home 이 서로의 통행로를 막지 않음
- [ ] 경로상에 사람/장애물 없음
- [ ] Ctrl-C 로 멈추는 것을 4번에서 확인함
