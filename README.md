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

## 설치 — PC 에만 한다

`pinky_fleet` 은 **PC 전용 패키지**다. 로봇에는 아무것도 넣지 않는다.
로봇에서 도는 것은 핑키 이미지에 이미 있는 `pinky_bringup` / `pinky_navigation` 뿐이고,
우리 코드는 DDS(토픽 + `NavigateToPose` 액션)로만 로봇과 대화한다.

### 1) 인증 — 저장소가 private 이라 필요하다

**Personal Access Token** (추가 설치 없음)

브라우저에서 GitHub → **Settings → Developer settings → Personal access tokens →
Fine-grained tokens → Generate new token**

- Repository access: **Only select repositories** → 이 저장소
- Permissions → Repository permissions → **Contents: Read and write**
  (받기만 할 거면 Read-only)

생성 직후 토큰 문자열을 복사해 둔다. 창을 닫으면 다시 볼 수 없다.

**또는 SSH 키** (만료 없음)

```bash
ssh-keygen -t ed25519 -C "<본인 이메일>"      # 엔터 3번
cat ~/.ssh/id_ed25519.pub                     # 출력을 GitHub → Settings → SSH and GPG keys 에 등록
ssh -T git@github.com                         # "Hi <계정>!" 이 나오면 성공
```

### 2) 워크스페이스 `src/` 아래에 clone

```bash
cd ~/<워크스페이스>/src

# PAT 방식
git config --global credential.helper store   # 한 번만 입력하면 기억한다
git clone -b dual-control https://github.com/sakim0128/pinky_pro_team11.git
#   Username: <계정>
#   Password: <토큰>          ← GitHub 비밀번호가 아니라 토큰

# 또는 SSH 방식
git clone -b dual-control git@github.com:sakim0128/pinky_pro_team11.git
```

저장소 루트가 패키지가 아니라 `pinky_fleet/` 을 담고 있는 구조라, 통째로 clone 해도
colcon 이 재귀 탐색으로 패키지를 찾는다.

```
~/<워크스페이스>/
├── src/
│   └── pinky_pro_team11/
│       ├── README.md
│       └── pinky_fleet/          ← colcon 이 찾는 패키지
├── build/  install/  log/
```

> `credential.helper store` 는 `~/.git-credentials` 에 **평문으로** 저장한다.
> 공용 PC 면 `store` 대신 `cache --timeout=28800` (8시간, 메모리에만) 을 쓴다.

### 3) 빌드

```bash
cd ~/<워크스페이스>

colcon list --packages-select pinky_fleet
#   pinky_fleet   src/pinky_pro_team11/pinky_fleet   (ament_python)
#   ← 반드시 한 줄만 나와야 한다

colcon build --packages-select pinky_fleet
source install/setup.bash
ros2 run pinky_fleet preflight --help
```

> **`src/` 에 `pinky_fleet` 사본이 둘 이상이면 colcon 이 빌드를 거부한다**
> (`Duplicate package names not supported`). 압축을 풀어 복사해 둔 게 남아 있으면 치운다.
>
> ```bash
> mkdir -p ~/_fleet_old && mv ~/<워크스페이스>/src/pinky_fleet ~/_fleet_old/copied
> ```
>
> **홈(`~`)에서 `colcon build` 하지 않는다.** 홈 전체를 훑어서 `~/venv/*` 안의
> numpy 테스트 폴더까지 패키지로 인식하려다 에러가 쏟아진다. 항상 워크스페이스 루트에서.

### 4) 필요한 ROS 패키지

```bash
sudo apt install -y ros-jazzy-domain-bridge ros-jazzy-nav2-simple-commander ros-jazzy-turtlesim
```

`preflight` 가 무엇이 없는지 알려준다.

### 5) 갱신 받기

```bash
cd ~/<워크스페이스>/src/pinky_pro_team11 && git pull
cd ~/<워크스페이스> && colcon build --packages-select pinky_fleet && source install/setup.bash
```

`config/mission.yaml` 은 git 이 추적하는 파일이다. 현장 실측 `home` 좌표를 넣었다면
`git pull` 때 충돌할 수 있다 — 커밋해서 남기거나(권장) `git stash` 로 잠시 치운다.

## 빠른 시작

```bash
# 0) PC 사전 점검 — 로봇·네트워크·시계까지 한 번에
ros2 run pinky_fleet preflight

# 1) 로봇 준비 — 각 로봇에 SSH (도메인 10 / 11), 세션마다 export 필요
export ROS_DOMAIN_ID=10 && export ROS_LOCALHOST_ONLY=0
ros2 launch pinky_bringup bringup_robot.launch.xml
ros2 launch pinky_navigation bringup_launch.xml map:=<맵이름>.yaml

# 2) 출발지 좌표 실측 (1회) — 로봇 도메인에서 RViz 를 띄워 2D Pose Estimate
ROS_DOMAIN_ID=10 ros2 launch pinky_navigation nav2_view.launch.xml
ros2 run pinky_fleet preflight --print-home     # 출력을 src 의 mission.yaml 에 붙여넣고 재빌드

# 3) PC
ros2 launch pinky_fleet fleet_bridge.launch.py      # 관제 평면
ros2 launch pinky_fleet fleet_view.launch.py        # rviz2 (도메인 20)
ros2 run    pinky_fleet fleet_master                # 미션

# RViz 의 Publish Point 로 2번 클릭 (목적지 → 바라볼 방향) 후 Enter
```

> `2D Pose Estimate` 는 **관제 도메인(20)의 RViz 에서는 먹지 않는다.** 브리지가
> 로봇 → 관제 단방향이라 `/initialpose` 가 로봇으로 넘어가지 않는다. 그래서 2)에서만
> 로봇 도메인의 RViz 를 쓴다. 본 미션은 `fleet_master` 가 `setInitialPose()` 로 넣는다.

### 어느 RViz 에서 무엇이 보이나

도메인이 논리적 칸막이라, **띄운 도메인에 있는 것만 보인다.**

| 띄운 것 | 보이는 것 |
|---|---|
| `ROS_DOMAIN_ID=10 ros2 launch pinky_navigation nav2_view.launch.xml` | **1호기만.** 로봇 모델 + 라이다 + 맵. `2D Pose Estimate` 가 먹는다 |
| `ROS_DOMAIN_ID=11 ...` (같은 명령) | **2호기만.** 위와 동일 |
| `ros2 launch pinky_fleet fleet_view.launch.py` (도메인 20) | **두 대 같이.** 단 맵 + 화살표 마커만 — 로봇 모델·라이다는 없다 (`/tf` 를 브리지하지 않기 때문) |

로봇 도메인 RViz 에 한 대만 보이는 것은 **정상**이다. 그래서 출발지 좌표 실측은
도메인을 바꿔 가며 **두 번** 한다.

관제 화면(도메인 20)의 화살표 마커는 관제 콘솔 노드가 그린다. 그 노드는
`fleet_master` 의 자식 프로세스이거나, 단독으로 띄운 `fleet_console` 이다.

```bash
ros2 run pinky_fleet fleet_console    # 미션 없이 관제 화면만 채운다
```

**`fleet_master` 를 실행하기 전에 `fleet_console` 은 끈다** — 둘 다 `/fleet/markers` 를
발행하면 상태 텍스트가 깜빡인다.

로봇 없이 로직만 확인:

```bash
cd pinky_fleet && PYTHONPATH=. python3 -m pinky_fleet.fleet_master --dry-run
```

자세한 현장 절차는 [RUNBOOK.md](pinky_fleet/docs/RUNBOOK.md) 에 있다.

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
