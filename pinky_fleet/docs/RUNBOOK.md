# 현장 실행 순서 (RUNBOOK)

---

## 0. 사전 준비 — 한 번만

### 0.1 맵을 두 로봇에 동일하게 복사

두 로봇이 **완전히 같은 맵 파일**을 써야 한다. 좌표계가 다르면 목적지 좌표 하나로
두 대를 보낼 수 없다.

```bash
# [PC] 맵을 만든 쪽에서 가져온 뒤, 두 로봇에 같은 파일을 뿌린다
scp pinky@<로봇1_IP>:~/<맵이름>.pgm  ~/maps/
scp pinky@<로봇1_IP>:~/<맵이름>.yaml ~/maps/

scp ~/maps/<맵이름>.pgm  ~/maps/<맵이름>.yaml  pinky@<로봇1_IP>:~/
scp ~/maps/<맵이름>.pgm  ~/maps/<맵이름>.yaml  pinky@<로봇2_IP>:~/
```

확인: `.yaml` 안의 `image:` 가 **파일명만** 적혀 있거나 로봇 기준 경로여야 한다.
PC 절대경로가 박혀 있으면 로봇에서 지도를 못 연다.

```bash
ssh pinky@<로봇1_IP> "head -3 ~/<맵이름>.yaml"   # image: <맵이름>.pgm
```

### 0.2 로봇 도메인 고정

각 로봇 `~/.bashrc` 끝에 (초기설정 자료 p47):

```bash
# 로봇1
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
# 로봇2
export ROS_DOMAIN_ID=11
export ROS_LOCALHOST_ONLY=0
```

`ROS_LOCALHOST_ONLY=1` 이면 WiFi 너머 통신이 전부 막힌다. 반드시 0.

### 0.3 PC 패키지 빌드

```bash
cd ~/ros2_ws/src && ln -s <이 저장소>/pinky_fleet .
cd ~/ros2_ws && colcon build --packages-select pinky_fleet && source install/setup.bash
sudo apt install ros-jazzy-domain-bridge      # 없으면
```

### 0.4 출발지 좌표 실측 → `mission.yaml` 에 기입

1. 로봇을 각자의 출발 위치에 놓는다
2. 아래 1~3단계로 RViz 를 띄우고, 2D Pose Estimate 로 위치를 한 번 맞춘 뒤
3. `ros2 topic echo /amcl_pose` 로 좌표를 읽거나 `Publish Point` 로 클릭해 읽는다
4. `config/mission.yaml` 의 `robots[].home` 에 적는다

> **주의**: 두 로봇의 home 이 서로의 통행로를 막지 않는 위치여야 한다.
> 순차 주행이라도 대기 중인 로봇은 그 자리에 서 있는 장애물이다.

### 0.5 배터리 확인

```bash
ssh pinky@<로봇IP> "battery"    # 7V 이하면 충전 (스펙 자료 p10)
```

---

## 1. [로봇1] SSH 접속 후 bringup + Nav2

터미널 2개 필요 (또는 tmux).

```bash
ssh pinky@<로봇1_IP>            # 비밀번호 1
# 터미널 A
ros2 launch pinky_bringup bringup_robot.launch.xml
# 터미널 B
ros2 launch pinky_navigation bringup_launch.xml map:=<맵이름>.yaml
```

## 2. [로봇2] 동일

```bash
ssh pinky@<로봇2_IP>
ros2 launch pinky_bringup bringup_robot.launch.xml
ros2 launch pinky_navigation bringup_launch.xml map:=<맵이름>.yaml
```

## 3. [PC] 브리지

```bash
ros2 launch pinky_fleet fleet_bridge.launch.py
```

`mission.yaml` 을 읽어 브리지 설정을 새로 생성하고, 로봇 수만큼 `domain_bridge` 를 띄운다.

## 4. [PC] 관제 화면

```bash
ros2 launch pinky_fleet fleet_view.launch.py
```

`ROS_DOMAIN_ID` 를 따로 export 할 필요가 없다 — launch 가 `mission.yaml` 의
`control_domain_id` 를 rviz2 프로세스에 직접 넣는다.

**확인**: 맵이 보이고, 잠시 뒤 로봇 2대의 화살표 마커가 뜬다.
안 보이면 → [문제 해결](#문제-해결) 로.

## 5. [PC] 미션 실행

```bash
ros2 run pinky_fleet fleet_master
```

1. `WAIT_NAV2` — 두 로봇 Nav2 활성화 + 출발지 설정 + AMCL 수렴 대기
2. `WAIT_GOAL` — RViz 상단 **Publish Point** 를 누르고
   - 1번째 클릭: **목적지 위치**
   - 2번째 클릭: **도착했을 때 바라볼 방향** (목적지에서 그 점을 향하는 각도)
3. 터미널에 좌표가 뜨면 **Enter** → 순차 왕복 시작

좌표를 미리 아는 경우 클릭 없이:

```bash
ros2 run pinky_fleet fleet_master --goal 2.5 1.0 90 -y
```

## 6. 정지

- 정상 종료: 미션이 `DONE` 이 되면 자동 종료
- 중단: `fleet_master` 터미널에서 **Ctrl-C** → 두 로봇에 `cancelTask()` 브로드캐스트

---

## 문제 해결

| 증상 | 원인 후보 | 확인 / 조치 |
|---|---|---|
| RViz 에 맵이 안 뜬다 | 브리지가 로봇을 못 봄 | 로봇 터미널에서 `ros2 topic list` 에 `/map` 이 있는지 → PC 에서 `ROS_DOMAIN_ID=10 ros2 topic list` 로 로봇이 보이는지 |
| 맵은 뜨는데 마커가 없다 | `/amcl_pose` 미발행 | Nav2 가 떴는지, 2D Pose Estimate 를 한 번도 안 했는지 확인. `ROS_DOMAIN_ID=20 ros2 topic hz /pinky1/amcl_pose` |
| 아무것도 안 보인다 | `ROS_LOCALHOST_ONLY=1` | 로봇/PC 양쪽에서 `echo $ROS_LOCALHOST_ONLY` → 0 이어야 한다 |
| `Nav2 가 활성화되지 않았습니다` | 로봇 도메인 불일치 / Nav2 미기동 | 로봇에서 `echo $ROS_DOMAIN_ID` 가 `mission.yaml` 값과 같은지 |
| `AMCL 이 수렴하지 않았습니다` | home 좌표가 실제 위치와 다름 | `mission.yaml` 의 `home` 을 다시 실측. 임시로 `localization.wait_for_convergence: false` 로 우회 가능 |
| 로봇이 엉뚱한 데로 간다 | 두 로봇의 맵이 다름 | 0.1 로 돌아가 **같은 파일**을 다시 복사 |
| 클릭해도 반응 없다 | 상태가 `WAIT_GOAL` 이 아님 | `fleet_master` 터미널의 `[STATE]` 확인. `Publish Point` 도구를 눌렀는지 확인 |
| 브리지가 자기 메시지를 되받음 | 도메인 겹침 | `mission.yaml` 로더가 막지만, 수동 설정 시 주의 |
| 목표를 계속 거부(`goToPose` false) | costmap 상 도달 불가 지점 | 벽/inflation 안쪽을 찍었을 가능성. 통로 중앙을 다시 클릭 |

### 관제 도메인 ID 바꾸기

1. `config/mission.yaml` 의 `control_domain_id` 수정
2. `colcon build --packages-select pinky_fleet && source install/setup.bash`
3. 3~5단계 재실행 — 브리지 설정은 launch 가 자동 재생성한다

수동으로 확인하고 싶으면:

```bash
ros2 run pinky_fleet make_bridge_yaml --out-dir /tmp/x && cat /tmp/x/*.yaml
```

### RViz 없이 관제하기

```bash
ROS_DOMAIN_ID=20 ros2 topic echo /fleet/state          # 미션 상태
ROS_DOMAIN_ID=20 ros2 topic echo /pinky1/amcl_pose     # 로봇1 위치
```
