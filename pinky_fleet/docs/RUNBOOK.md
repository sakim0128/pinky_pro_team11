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

PC 쪽도 같다. 그리고 **Jazzy 에서는 `ROS_LOCALHOST_ONLY` 가 deprecated 이고
`ROS_AUTOMATIC_DISCOVERY_RANGE` 가 그 자리를 대신한다** — 이 값이 `LOCALHOST` 여도
똑같이 막힌다. PC 의 모든 터미널에서:

```bash
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

`ROS_DOMAIN_ID` 는 PC 의 `.bashrc` 에 넣지 않는다 — 도메인은 코드와 launch 가 정한다.
`preflight` 가 이 네 가지를 먼저 검사해 준다.

### 0.2-1 로봇의 IP 알아내기

LCD 에는 **AP 모드용 SSID/비번만** 나오고 강의장 네트워크 IP 는 안 나온다.
로봇에게 직접 물어봐야 한다.

1. PC 를 로봇 LCD 의 SSID(예: `pinky_e2a8`, 비번 `pinkypro`)에 연결
2. `ssh pinky@192.168.4.1` (비밀번호 `1`)
3. `./wifi_setup.sh` 로 강의장 공유기에 연결
4. **그 SSH 세션에서 바로** `hostname -I` → `192.168.4.1` 이 아닌 쪽이 강의장 IP
5. 나머지 로봇도 반복한 뒤, 마지막에 **PC 의 WiFi 를 강의장 공유기로 바꾼다**

> **가장 흔한 함정**: PC 가 핑키 AP(`192.168.4.1`)에 붙어 있으면 그 로봇 하나만 보이고
> 나머지는 절대 안 보인다. PC 와 두 로봇이 **모두 강의장 공유기**에 있어야 한다.

셋 다 이미 강의장 망에 있는데 IP 만 모르면:

```bash
ip -4 route | grep default        # PC 가 속한 대역 확인
nmap -sn 192.168.0.0/24           # 대역은 위에서 본 것으로
```

### 0.3 PC 패키지 빌드

```bash
cd ~/ros2_ws/src && ln -s <이 저장소>/pinky_fleet .
cd ~/ros2_ws && colcon build --packages-select pinky_fleet && source install/setup.bash
sudo apt install ros-jazzy-domain-bridge      # 없으면
```

### 0.4 출발지 좌표 실측 → `mission.yaml` 에 기입

이 절에서만 **로봇 도메인의 RViz** 를 쓴다. 이유는 아래 상자를 볼 것.

1. 로봇을 각자의 **실제 출발 위치**에 놓는다
2. 아래 1~2단계로 로봇의 bringup + Nav2 를 띄운다
3. 로봇 도메인에서 RViz 를 띄우고 **2D Pose Estimate 로 위치를 한 번 맞춘다**

   ```bash
   ROS_DOMAIN_ID=10 ros2 launch pinky_navigation nav2_view.launch.xml   # 로봇1
   ROS_DOMAIN_ID=11 ros2 launch pinky_navigation nav2_view.launch.xml   # 로봇2
   ```

   맵에서 로봇의 실제 위치를 클릭하고 바라보는 방향으로 드래그한다.
   **라이다 점이 벽과 겹치면** 성공. 확인되면 창을 닫는다.

   `pinky_navigation` 이 PC 에 없으면 `ROS_DOMAIN_ID=10 rviz2` 로 띄우고
   Fixed Frame `map`, Map(`/map`, Durability **Transient Local**),
   LaserScan(`/scan`, Reliability **Best Effort**) 을 추가한 뒤 같은 버튼을 쓴다.

4. 두 대 다 맞춘 뒤 PC 에서:

```bash
ros2 run pinky_fleet preflight --print-home
```

5. 출력된 `robots:` 블록을 **소스 트리의** `mission.yaml` 에 붙여넣고 재빌드한다.

```bash
nano ~/<워크스페이스>/src/pinky_fleet/config/mission.yaml
cd ~/<워크스페이스> && colcon build --packages-select pinky_fleet && source install/setup.bash
```

`preflight` 가 네임스페이스도 같이 채워 준다.

> **`2D Pose Estimate` 는 관제 도메인(20)의 RViz 에서는 먹지 않는다.**
> 브리지는 **로봇 → 관제 단방향**이라 관제 도메인에서 발행한 `/initialpose` 가
> 로봇으로 넘어가지 않는다. 그래서 위처럼 **로봇 도메인에서** RViz 를 따로 띄운다.
> 본 미션에서는 `fleet_master` 가 `setInitialPose()` 로 넣으므로 이 조작이 필요 없다 —
> home 좌표를 처음 재는 이 1회에만 필요하다.

> **설정은 `install/` 이 아니라 `src/` 를 고친다.**
> `install/pinky_fleet/share/.../mission.yaml` 을 고치면 다음 `colcon build` 때 덮어써진다.

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

## 3. [PC] 사전 점검 — 먼저 이것부터

```bash
ros2 run pinky_fleet preflight
```

한 번에 다음을 확인한다. **여기서 FAIL 이 나면 미션을 돌리지 말고 그것부터 해결한다.**

| 항목 | FAIL 이면 |
|---|---|
| PC 패키지 | `sudo apt install ros-jazzy-domain-bridge ros-jazzy-nav2-simple-commander ros-jazzy-turtlesim` |
| Nav2 노드 (`amcl`, `bt_navigator`) | 로봇에서 `pinky_navigation` 이 안 떴거나 도메인이 틀렸다 |
| 네임스페이스 | 나오면 `mission.yaml` 의 그 로봇에 `namespace:` 를 채운다 |
| `/map` `/amcl_pose` `/scan` | 위와 같은 원인 |
| `navigate_to_pose` 액션 서버 | Nav2 가 아직 활성화 전 |
| **시계 오차** | 1초 이상이면 TF 조회가 실패한다. 로봇에서 시각을 PC 에 맞춘다 |

## 4. [PC] 브리지

```bash
ros2 launch pinky_fleet fleet_bridge.launch.py
```

`mission.yaml` 을 읽어 브리지 설정을 새로 생성하고, 로봇 수만큼 `domain_bridge` 를 띄운다.

## 5. [PC] 관제 화면

```bash
ros2 launch pinky_fleet fleet_view.launch.py
```

`ROS_DOMAIN_ID` 를 따로 export 할 필요가 없다 — launch 가 `mission.yaml` 의
`control_domain_id` 를 rviz2 프로세스에 직접 넣는다.

**확인**: **맵이 보인다.** 이 단계에서는 그것뿐이다.

> **로봇 마커는 아직 안 뜬다.** `fleet_view.launch.py` 는 rviz2 만 띄우고,
> `/fleet/markers` 를 발행하는 관제 콘솔 노드는 `fleet_master` 의 자식 프로세스로 돈다.
> 미션을 돌리기 전에 로봇 위치를 화면으로 확인하고 싶으면 별도 터미널에서:
>
> ```bash
> ros2 run pinky_fleet fleet_console      # 맵 + 두 로봇 화살표. 미션은 안 돈다
> ```
>
> **`fleet_master` 를 실행하기 전에 이걸 Ctrl-C 로 끈다.** 둘 다 `/fleet/markers` 를
> 발행하면 상태 텍스트가 깜빡인다.

맵조차 안 보이면 → [문제 해결](#문제-해결) 로.

## 6. [PC] 미션 실행

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

## 7. 정지

- 정상 종료: 미션이 `DONE` 이 되면 자동 종료
- 중단: `fleet_master` 터미널에서 **Ctrl-C** → 두 로봇에 `cancelTask()` 브로드캐스트

---

## Nav2 파라미터 튜닝 결과 저장하기

rqt_reconfigure 로 `inflation_radius` 같은 값을 만졌다면 **그 값은 노드를 재시작하면
사라진다.** ROS2 의 rqt 에는 ROS1 의 dynamic_reconfigure 같은 save 버튼이 없다.
파일로 남겨야 다음에 로봇을 껐다 켜도 같은 주행이 나온다.

**로봇이 두 대이므로 양쪽에 똑같이 적용해야 한다.** 한쪽만 고치면 같은 목적지 좌표에
대해 두 대가 다르게 움직인다.

### 1. 지금 값을 덤프한다

파라미터는 서비스라 PC 에서 도메인만 맞추면 읽힌다. SSH 할 필요 없다.

```bash
tools/dump_nav2_params.sh 10          # 로봇1 → ./nav2_params_dump_domain10/
tools/dump_nav2_params.sh 11          # 로봇2
```

노드 하나만 보려면:

```bash
ROS_DOMAIN_ID=10 ros2 param dump /local_costmap/local_costmap > lc10.yaml
```

> `ros2 param dump` 는 Jazzy 에서 **stdout 으로만** 출력한다.
> 구버전 문서에 나오는 `--output-dir` / `--print` 옵션은 제거됐다. `>` 로 리다이렉트한다.
> 최상위 키는 `/local_costmap/local_costmap:` 처럼 노드의 완전한 이름이다.

값 하나만 확인할 거면 덤프도 필요 없다:

```bash
ROS_DOMAIN_ID=10 ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
```

### 2. 바꾼 항목만 골라낸다

```bash
grep -n "inflation_radius\|cost_scaling_factor" nav2_params_dump_domain10/*costmap*.yaml
```

> **덤프 파일을 그대로 params 파일로 쓰지 않는다.**
> 덤프에는 `use_sim_time`, `qos_overrides.*`, 플러그인이 런타임에 추가한 항목까지
> 전부 들어 있고, 원본의 주석과 구조도 사라진다. 런타임에만 존재하는 항목이 섞여
> 기동 때 문제를 만들 수도 있다.
> **덤프는 "내가 뭘 바꿨는지" 를 찾는 용도로 쓰고, 바뀐 항목만 원본에 옮긴다.**

### 3. 로봇의 params 파일에 옮긴다

```bash
ssh pinky@<로봇1_IP>
find "$(ros2 pkg prefix pinky_navigation)/share/pinky_navigation" -name '*.yaml'
ros2 launch pinky_navigation bringup_launch.xml --show-args    # params_file 인자가 있나?
```

- `params_file` 인자가 **있으면** — 원본을 홈에 복사해 고치고 그걸 넘긴다.
  설치 경로를 건드리지 않아 가장 안전하다.
  ```bash
  cp <원본>.yaml ~/my_nav2.yaml
  nano ~/my_nav2.yaml
  ros2 launch pinky_navigation bringup_launch.xml map:=<맵>.yaml params_file:=~/my_nav2.yaml
  ```
- **없으면** — 설치된 yaml 을 직접 고친다. **반드시 먼저 백업한다.**
  ```bash
  sudo cp <원본>.yaml <원본>.yaml.bak
  sudo nano <원본>.yaml
  ```

### 4. 두 로봇 다 재기동해서 확인

```bash
ROS_DOMAIN_ID=10 ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ROS_DOMAIN_ID=11 ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
```

두 값이 같고 내가 넣은 값이면 끝이다.

---

## 문제 해결

| 증상 | 원인 후보 | 확인 / 조치 |
|---|---|---|
| RViz 에 맵이 안 뜬다 | 브리지가 로봇을 못 봄 | 로봇 터미널에서 `ros2 topic list` 에 `/map` 이 있는지 → PC 에서 `ROS_DOMAIN_ID=10 ros2 topic list` 로 로봇이 보이는지 |
| 맵은 뜨는데 마커가 없다 | **`fleet_master` 도 `fleet_console` 도 안 떠 있음** | 마커는 관제 콘솔 노드가 그린다. 둘 중 하나를 띄운다 (동시에는 안 됨) |
| 〃 | `/amcl_pose` 미발행 | Nav2 가 떴는지, 2D Pose Estimate 를 한 번도 안 했는지 확인. `ROS_DOMAIN_ID=20 ros2 topic hz /pinky1/amcl_pose` |
| 상태 텍스트가 깜빡인다 | `fleet_console` 과 `fleet_master` 가 동시에 떠 있음 | `fleet_console` 을 끈다 |
| 로봇 도메인 RViz 에 한 대만 보인다 | **정상** — 그 RViz 는 그 도메인만 본다 | 두 대를 한 화면에서 보려면 브리지 + `fleet_view` + `fleet_console`/`fleet_master` |
| 아무것도 안 보인다 | `ROS_LOCALHOST_ONLY=1` | 로봇/PC 양쪽에서 `echo $ROS_LOCALHOST_ONLY` → 0 이어야 한다 |
| `Nav2 가 활성화되지 않았습니다` | 로봇 도메인 불일치 / Nav2 미기동 | 로봇에서 `echo $ROS_DOMAIN_ID` 가 `mission.yaml` 값과 같은지 |
| `AMCL 이 보고한 위치가 home 에서 … 떨어져` | `home` 좌표가 실제 로봇 위치와 다름 | `preflight --print-home` 으로 다시 뽑는다. 급하면 `localization.max_initial_offset` 를 키운다 |
| `setInitialPose 이후 새 /amcl_pose 가 오지 않았습니다` | 라이다가 안 나오거나 amcl 이 죽음 | `preflight` 로 `/scan` 과 `amcl` 노드 확인 |
| 액션·TF 가 이상하게 실패 | **PC 와 로봇의 시계 차이** | `preflight` 의 시계 오차 항목 확인. 로봇에서 시각을 맞춘다 |
| 2D Pose Estimate 가 안 먹음 | 관제 도메인(20)의 RViz 에서 눌렀음 | 0.4 처럼 **로봇 도메인**에서 RViz 를 띄운다 (브리지는 단방향) |
| 재빌드해도 설정이 그대로 | `install/` 안의 파일을 고쳤음 | `src/` 를 고치고 다시 `colcon build` |
| `Duplicate package names not supported` | `pinky_fleet` 사본이 여러 곳에 있음 | 워크스페이스 `src/` 에 하나만 남긴다. **홈에서 `colcon build` 하지 않는다** |
| 로봇이 엉뚱한 데로 간다 | 두 로봇의 맵이 다름 | 0.1 로 돌아가 **같은 파일**을 다시 복사 |
| 재시작하니 주행이 다시 이상해짐 | rqt 로 만진 파라미터는 재시작하면 사라진다 | [Nav2 파라미터 튜닝 결과 저장하기](#nav2-파라미터-튜닝-결과-저장하기) |
| 두 로봇의 주행 성향이 다르다 | 파라미터를 한쪽에만 적용했다 | 같은 절의 4단계로 양쪽 값을 대조 |
| 클릭해도 반응 없다 | 상태가 `WAIT_GOAL` 이 아님 | `fleet_master` 터미널의 `[STATE]` 확인. `Publish Point` 도구를 눌렀는지 확인 |
| 브리지가 자기 메시지를 되받음 | 도메인 겹침 | `mission.yaml` 로더가 막지만, 수동 설정 시 주의 |
| 목표를 계속 거부(`goToPose` false) | costmap 상 도달 불가 지점 | 벽/inflation 안쪽을 찍었을 가능성. 통로 중앙을 다시 클릭 |

### 진행 순서 요약 (오늘 현장용)

```
0. PC   colcon build && source install/setup.bash
1. PC   pytest + fleet_master --dry-run            (로봇 불필요)
2. PC   L1 turtlesim 2도메인                        (로봇 불필요, 로봇 부팅과 병행)
3. 로봇 bringup + pinky_navigation  ×2
4. PC   preflight                                   ← FAIL 있으면 여기서 해결
5. PC   preflight --print-home → mission.yaml → 재빌드
6. PC   fleet_bridge + fleet_view                   ← RViz 에 맵
        (선택) fleet_console 으로 로봇 마커 확인 후 끈다
7. 실물 바퀴 띄우고 준비만 → 1 m 미션 → 취소 확인 → 본 미션
```

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
