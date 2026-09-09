# pinky_fleet 아키텍처와 기술 선택 근거

발표용 문서. "왜 이렇게 만들었는가"에 대한 답을 수업 자료와 소스 근거에 연결해 둔다.

---

## 1. 풀어야 하는 문제

목적지 좌표 하나를 지정하면

1. Pinky-1 이 목적지로 가고 → 출발지로 돌아온다
2. 그 다음 Pinky-2 가 같은 목적지로 가고 → 출발지로 돌아온다

제약이 하나 있다. **두 로봇의 `ROS_DOMAIN_ID` 가 각각 10, 11 로 분리되어 있다.**
`ROS_DOMAIN_ID` 는 DDS 레벨의 네트워크 파티션이라, 같은 WiFi 에 있어도 서로를 발견하지
못한다(19강 s3). 즉 **기본 상태로는 하나의 노드가 두 로봇을 동시에 볼 수 없다.**

여기서 갈림길이 생긴다.

- 도메인을 하나로 통일한다 → 가장 쉽지만, 팀별 도메인 분리라는 운용 규칙을 깬다.
  두 로봇의 토픽 이름이 전부 같아서(`/cmd_vel`, `/scan`, `/amcl_pose`) 한 도메인에 넣으면
  **그대로 충돌한다.** 네임스페이스를 새로 씌우려면 로봇 쪽 launch 를 전부 손봐야 한다.
- 도메인은 그대로 두고, 관제 쪽에서 두 도메인에 접근한다 → **이 프로젝트의 선택.**

---

## 2. 전체 구조 — 제어 평면과 관제 평면을 나눈다

```
 [Pinky-1] DOMAIN 10                       [Pinky-2] DOMAIN 11
 pinky_bringup + pinky_navigation(Nav2)    pinky_bringup + pinky_navigation(Nav2)
   │ ▲                                       │ ▲
   │ │ NavigateToPose 액션 (goal/feedback/result/cancel)
   │ │ /amcl_pose, /map                      │ │
───┼─┼──────────────── WiFi (같은 서브넷) ─────┼─┼──────────────
   │ │                                       │ │
 ┌─┴─┴───────────────────────────────────────┴─┴─────────────────┐
 │ PC                                                            │
 │                                                               │
 │  ● 제어 평면 :  fleet_master (부모, ROS 안 씀)                   │
 │      ├ 자식 A  rclpy.init(domain_id=10)  BasicNavigator        │
 │      ├ 자식 B  rclpy.init(domain_id=11)  BasicNavigator        │
 │      └ 자식 C  rclpy.init(domain_id=20)  관제 콘솔 노드          │
 │           ↕ multiprocessing.Queue                             │
 │                                                               │
 │  ● 관제 평면 :  domain_bridge × 2 (별도 프로세스)                │
 │      10 → 20 :  /map,  /amcl_pose → /pinky1/amcl_pose         │
 │      11 → 20 :         /amcl_pose → /pinky2/amcl_pose         │
 │                          ↓                                    │
 │      rviz2 (DOMAIN 20) — 맵 한 장 위에 로봇 2대                  │
 └───────────────────────────────────────────────────────────────┘
```

한 문장으로: **명령은 액션으로 직접 보내고, 관제는 브리지로 모은다.**

---

## 3. 기술 선택 근거

### 3.1 도메인 10·11 동시 접근 → `multiprocessing`

| 후보 | 가능? | 왜 아닌가 / 왜 맞나 |
|---|---|---|
| 터미널을 2개 연다 | 가능 | 사람이 눈으로 보고 다음 명령을 친다. **"A가 복귀했는지"를 코드가 판정할 수 없다** → 순차 미션 자동화 불가 |
| `threading` | **불가** | `rclpy.init()` 은 프로세스당 1회. 스레드는 같은 프로세스라 도메인을 못 바꾼다 (19강 s24) |
| `multiprocessing` | 가능 | 자식마다 `rclpy.init(args=[], domain_id=N)` (19강 s17). **선택** |

19강 `publisher_process`, 20강 `turtle_process` 가 정확히 이 패턴이다.
`domain_worker.worker_process()` 가 그 자리에 대응한다.

> **부모는 `rclpy.init()` 을 하지 않는다.** 관제 도메인(20) 노드조차 자식 프로세스 C 로 뺐다.
> 부모가 먼저 init 해버리면 fork 로 생긴 자식이 부모의 DDS 상태를 물려받아 꼬인다.
> 추가 안전장치로 `mp.set_start_method('spawn')` 을 명시했다 (`fleet_master.main()`).

### 3.2 목적지 지시 → `NavigateToPose` 액션 (`BasicNavigator`)

07강에서 정리한 통신 선택 기준을 그대로 적용한다.

| | 방식 | 응답 | 취소 | 이 미션에 |
|---|---|---|---|---|
| 토픽 | Pub/Sub | 없음 | 불가 | ✗ 도착 여부를 모른다 |
| 서비스 | Req/Res | 있음 | 불가 | ✗ 주행은 오래 걸린다 |
| **액션** | Goal/Feedback/Result | **있음** | **가능** | ✓ |

미션 정의 자체가 "**A가 복귀한 뒤에** B가 출발"이다. 즉 **완료 판정이 필수**다.
그리고 실물 로봇이라 **취소(비상 정지)도 필수**다. 이 둘을 동시에 주는 통신은 액션뿐이다.

구현은 `주피터로 내비게이션 주행하기` p8~p18 의 흐름 그대로:

```
BasicNavigator() → setInitialPose() → goToPose()
  → while not isTaskComplete():  getFeedback()   # 타임아웃이면 cancelTask()
  → getResult()                                   # SUCCEEDED / CANCELED / FAILED
```

### 3.3 액션을 브리지로 넘기지 않는 이유 (핵심)

브리지로 다 넘기면 도메인 20 하나에서 전부 처리할 수 있어 편해 보인다. 그런데:

- ROS2 액션은 내부적으로 **토픽 2개 + 서비스 3개**의 묶음이다.
  - 토픽 2 — `_action/feedback`, `_action/status`
  - 서비스 3 — `_action/send_goal`, `_action/cancel_goal`, `_action/get_result`
- `domain_bridge` 설정 파서(`parse_domain_bridge_yaml_config.cpp`)를 확인한 결과,
  **파싱하는 최상위 키는 `name`, `from_domain`, `to_domain`, `mode`, `topics` 뿐이다.
  `services` 도 `actions` 도 키 자체가 없다.**

즉 액션은 브리지로 넘어가지 않는다. 그래서 **검증된 것(토픽)만 브리지에 태우고,
액션은 도메인에 직접 붙는다**로 역할을 나눴다. 이 경계가 이 설계의 중심이다.

### 3.4 관제 화면 → `domain_bridge` + rviz2 1개

브리지의 공식 설명이 "특정 토픽만 선택적으로 공유"(21강 s3)다. 관제 화면은 정확히 그 용도다.
rviz2 를 도메인별로 2개 띄우면 "한 화면에서 두 대를 본다"는 관제의 정의가 깨진다.

### 3.5 `/tf` 를 브리지하지 않는 이유

두 로봇 모두 frame 이름이 `map` / `odom` / `base_link` 로 **같다.**
`remap` 은 **토픽 이름만** 바꾸고 메시지 안의 `frame_id` 는 못 바꾼다(21강 s29).
그래서 `/tf` 를 둘 다 중계하면 한 도메인에 `map→odom` 변환이 두 벌 들어와 트리가 충돌한다.

대신 `/amcl_pose` 만 받아서 관제 콘솔이 `Marker` 로 그린다.
Marker 는 `frame_id: map`, RViz Fixed Frame 이 `map` 이면 **TF 없이도 그려진다.**

### 3.6 브리지 대상 최소화

| 토픽 | 중계? | 이유 |
|---|---|---|
| `/map` | ✓ (1대에서만) | 관제 화면 배경. 두 로봇이 같은 맵 파일을 쓰므로 한 벌이면 충분 |
| `/amcl_pose` | ✓ (2대 모두) | 로봇 위치. 저빈도(수 Hz) |
| `/scan`, `/camera` | ✗ | 고빈도. WiFi 대역폭을 먹는데 관제 목적엔 불필요 |
| `/tf`, `/tf_static` | ✗ | 3.5 참고 |
| `/cmd_vel` | ✗ | 제어는 액션이 한다. 브리지로 속도를 흘리면 Nav2 와 명령이 충돌한다 |

### 3.7 QoS — 21강 s26 표 그대로

| 토픽 | 설정 | 근거 |
|---|---|---|
| `/map` | `reliable` + `transient_local` | 늦게 붙은 RViz 도 지도를 받아야 한다 |
| `/amcl_pose` | `reliable` + `transient_local` | **발행자와 정확히 일치시킨다** — 아래 참고 |

`/amcl_pose` 는 발행자에 맞추는 것이 필수다. nav2 amcl 의 발행 QoS 는

```cpp
pose_pub_ = create_publisher<PoseWithCovarianceStamped>(
    "amcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
```

이고, 더 중요한 것은 **amcl 이 로봇이 정지해 있으면 `/amcl_pose` 를 아예 발행하지 않는다**는
점이다 (`shouldUpdateFilter()` 가 `update_min_d` 0.25 m / `update_min_a` 0.2 rad 를
넘어야 필터를 갱신하고 발행한다). `volatile` 로 구독하면 래치된 마지막 값을 못 받으므로
**정지한 로봇은 관제 화면에 아예 나오지 않는다.** `transient_local` 로 구독해야 붙는 즉시
마지막 위치가 한 건 온다. 워커·콘솔·브리지 세 곳 모두 같은 프로파일을 쓴다
(`pinky_fleet/ros_qos.py`).

### 3.8 브리지를 로봇당 1 프로세스로 나눈 이유

`domain_bridge` YAML 은 `topics` 아래에 **소스 토픽 이름을 키로** 쓴다.
두 로봇 다 `/amcl_pose` 라서 한 파일에 넣으려면 같은 키를 두 번 써야 하는데,
그건 YAML 표준이 보장하지 않는다(상류 예제가 실제로 그렇게 쓰지만 위험하다).
그래서 **로봇당 YAML 1개 + 프로세스 1개**로 나눴다.
부수 효과로 한쪽 브리지가 죽어도 다른 로봇 관제는 살아 있다.

### 3.9 순차 실행 — 성능이 아니라 안전

Nav2 는 **멀티로봇 경로 조율을 하지 않는다.** 좁은 실내에서 두 대가 같은 목적지로 동시에
가면 서로를 동적 장애물로 인식해 교착하거나 회피 진동에 빠진다.
순차 실행은 미션 정의이면서 동시에 가장 단순한 충돌 회피 전략이다.

### 3.10 `ROS_LOCALHOST_ONLY=0` 이어야 한다

21강 실습은 한 PC 안이라 `ROS_LOCALHOST_ONLY=1` 을 썼다. 여기서는 로봇이 WiFi 너머에 있으므로
**1이면 브리지도 액션도 전부 불통이다.** 실수를 막으려고 `fleet_bridge.launch.py` 와
`fleet_view.launch.py` 가 자식 프로세스 환경변수로 `0` 을 직접 넣는다.

---

## 4. 미션 상태머신 (25강 상태기반 제어 응용)

```
INIT
 └ WAIT_NAV2       두 워커 병렬: setInitialPose(home) -> waitUntilNav2Active() -> 초기 위치 확인
    └ WAIT_GOAL    관제 콘솔이 /clicked_point 2회 수신 → 터미널 Enter 확인
       └ GOING_TO_GOAL(pinky1) ─ GOING_HOME(pinky1) ─ SETTLING
          └ GOING_TO_GOAL(pinky2) ─ GOING_HOME(pinky2) ─ SETTLING
             └ DONE

 임의 상태 ─┬ result != SUCCEEDED
            ├ navigation_time > timeout   → cancelTask()
            └ 사용자 Ctrl-C
                                          → ABORT (두 워커에 cancel 브로드캐스트)
```

`WAIT_NAV2` 만 두 로봇을 **병렬로** 처리한다. 이 단계에서는 로봇이 움직이지 않기 때문이다.

### 목적지 입력이 2-click 인 이유

RViz 의 `Publish Point` 는 `geometry_msgs/PointStamped` 라서 **각도 정보가 없다.**
Nav2 의 goal 은 자세(방향)까지 필요하다. 그래서

- 1번째 클릭 = 목적지 위치 `(x, y)`
- 2번째 클릭 = 바라볼 방향 기준점 → `yaw = atan2(y₂−y₁, x₂−x₁)`

로 각도를 만든다 (`mission.yaml` 의 `goal_input.mode: one_click` 이면 고정 각도 사용).
오클릭 방지를 위해 좌표를 출력하고 **Enter 확인 후** 미션을 시작한다.

### 초기 위치 — 순서와 판정 기준

**순서가 중요하다. `setInitialPose()` 를 먼저, `waitUntilNav2Active()` 를 나중에.**
`waitUntilNav2Active()` 는 내부에서 `_waitForInitialPose()` 를 부르고, 그건
`self.initial_pose` 를 AMCL 에 발행한다. `setInitialPose()` 를 아직 안 불렀으면 그 값은
`BasicNavigator` 의 기본 `PoseStamped` — **위치 (0,0,0) 에 쿼터니언이 전부 0 인
유효하지 않은 자세** 다. 즉 원점을 먼저 밀어 넣게 된다. nav2 공식 예제도 이 순서다.

그리고 우리가 확인하는 것은 **"수렴"이 아니라 "AMCL 이 우리가 준 초기 위치를 받아들였는가"** 다.
초기 위치가 unknown 이면 amcl 은 공분산을 `[0.5², 0.5², (π/12)²]` 로 시작한다 —
**xy 표준편차가 0.5 m** 이고, 정지 중에는 리샘플이 없어 줄지 않는다.
수렴을 기다리면 100% 타임아웃이다.

| | 방법 |
|---|---|
| 하드 조건 | `setInitialPose` **이후 새로 발행된** `/amcl_pose` 가 `home` 에서 `max_initial_offset`(0.5 m) 이내 |
| "새 발행" 판정 | `header.stamp` 변화 — **로봇 자기 시계끼리** 비교하므로 PC 와의 시계 오차와 무관하다 |
| 소프트 조건 | 공분산은 로그·경고로만. 실제 수렴은 주행하면서 이뤄진다 |

amcl 은 초기 위치를 받으면 **첫 라이다 스캔에서 `force_publication` 으로 반드시 한 번
발행**하므로 이 조건은 1~2초 안에 판정된다.

---

## 5. 코드 지도

| 파일 | 역할 | 대응하는 수업 자료 |
|---|---|---|
| `fleet_master.py` | 부모. 상태머신 + Queue 중계 | 25강 상태기반 제어 |
| `domain_worker.py` | 자식 A/B. 도메인별 `rclpy.init` + 주행 | 19·20강 multiprocessing, Nav2 PDF p8~18 |
| `console_node.py` | 자식 C. 관제 도메인 노드 (clicked_point, Marker) | 08강 pub/sub, Nav2 PDF p10~14 |
| `make_bridge_yaml.py` | `mission.yaml` → 브리지 설정 생성 | 21강 YAML 옵션 |
| `preflight.py` | 현장 사전 점검 — 노드·토픽·액션서버·시계 오차·home 실측 | — |
| `mission_config.py` / `pose_utils.py` / `ros_qos.py` | 설정 로더 / 쿼터니언 변환 / QoS | Nav2 PDF p10, 21강 s26 |
| `protocol.py` | 프로세스 간 메시지 규약 | — |

세 백엔드(`mock` / `turtlesim` / `nav2`)가 **같은 메시지 규약**을 쓰고 "목표까지 간다"의
구현만 다르다. 20강 s42 의 학습 포인트 —
*"세 방식 모두 동일한 multiprocessing 패턴을 따르며, 달라지는 것은 노드 클래스의 통신
인터페이스뿐"* — 을 그대로 구조에 반영한 것이다.

---

## 6. 생성되는 브리지 설정 예시

`mission.yaml` 로부터 자동 생성된다 (커밋하지 않는다).

```yaml
# fleet_bridge_pinky1.yaml
name: pinky_fleet_bridge_pinky1
from_domain: 10
to_domain: 20
topics:
  amcl_pose:
    type: geometry_msgs/msg/PoseWithCovarianceStamped
    remap: pinky1/amcl_pose
    qos: {reliability: reliable, durability: transient_local}
  map:
    type: nav_msgs/msg/OccupancyGrid
    qos: {reliability: reliable, durability: transient_local}
```

```yaml
# fleet_bridge_pinky2.yaml   (map 없음 — 같은 맵이므로 한 대에서만 가져온다)
name: pinky_fleet_bridge_pinky2
from_domain: 11
to_domain: 20
topics:
  amcl_pose:
    type: geometry_msgs/msg/PoseWithCovarianceStamped
    remap: pinky2/amcl_pose
    qos: {reliability: reliable, durability: transient_local}
```

---

## 7. 한계와 다음 단계

- **순차라서 느리다.** 동시 주행을 하려면 로봇 간 충돌 회피(예약 구역, 우선순위)를 따로 만들어야 한다.
- **브리지는 단방향 관제용이다.** 비상정지를 브리지로 전 로봇에 뿌리려면
  `/fleet/estop` 토픽을 관제→로봇 방향으로 추가하고 로봇 쪽에 수신 노드를 둬야 한다.
- **로봇이 늘어나면** `mission.yaml` 의 `robots` 에 항목만 추가하면 된다.
  워커 프로세스와 브리지 프로세스가 자동으로 그 수만큼 뜬다.
