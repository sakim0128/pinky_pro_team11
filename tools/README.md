# 주행 데이터 수집 절차 (mini_project_2)

teleop 으로 핑키를 수동 주행시키면서 **카메라 영상 + rosbag** 을 동시에 기록한다.
영상은 YOLO 라벨링용, rosbag 은 시뮬레이션 재현·제어 검증용이다.

| 파일 | 실행 위치 | 역할 |
|---|---|---|
| `record_drive.py` | 로봇 | picamera2 로 mp4 저장 + 프레임별 촬영 시각 CSV |
| `extract_frames.py` | PC | 영상에서 라벨링용 프레임 솎아내기 (흔들린 프레임 제외) |

## 0. 스크립트를 로봇에 넣기

로봇에는 `~/pinky_pro/src/pinky_pro_team11` 이 `mini_project_1` 브랜치로 체크아웃되어 있다.
**브랜치를 바꾸거나 새로 clone 하지 않는다.** 파일 하나만 가져온다.

```bash
# 방법 1 — PC 에서 (mini_project_2 를 받아 둔 폴더에서)
scp tools/record_drive.py pinky@<핑키IP>:~/

# 방법 2 — 로봇에서, 기존 저장소에서 꺼내기 (작업 트리를 건드리지 않는다)
cd ~/pinky_pro/src/pinky_pro_team11
git fetch origin mini_project_2
git show origin/mini_project_2:tools/record_drive.py > ~/record_drive.py
```

## 1. 카메라 방향 확인 — 본 촬영 전에 반드시

카메라가 거꾸로 달려 있다. 참고 자료의 코드는 `ROTATE_180` 뒤에 `flip(1)` 을 하는데,
이 둘을 합치면 수직 뒤집기 하나와 같아서 뒤쪽 flip 이 LCD 미리보기용 좌우반전인지
실제 보정인지 코드만으로는 알 수 없다. **잘못 고르면 좌/우 차선이 뒤바뀌어 조향이
반대로 나간다.**

```bash
# 로봇에서. 로봇 "왼쪽" 에만 표식(펜 등)을 두고 찍는다
python3 ~/record_drive.py --snap
```

`~/pinky_data/snap_{none,rot180,rot180_mirror,vflip,hflip}.jpg` 다섯 장이 생긴다.
사진에서도 표식이 **왼쪽** 에 보이는 파일을 고르고, 그 이름을 아래 `--orient` 에 넣는다.
기하학적으로는 `rot180` 이 맞을 가능성이 가장 높다.

## 2. 터미널 구성

```bash
# [로봇 T1] 브링업
ros2 launch pinky_bringup bringup_robot.launch.xml

# [로봇 T2] 초음파·IR·배터리 — bringup 이 띄우지 않으므로 따로 실행 (선택)
ros2 run pinky_sensor_adc main_node

# [로봇 T3] 영상 녹화 — 주행 직전에 시작
python3 ~/record_drive.py --name course_a --orient rot180

# [PC T4] rosbag — 영상과 거의 동시에 시작
ros2 bag record -o ~/pinky_data/course_a_bag \
  /odom /scan /tf /tf_static /cmd_vel /us_sensor/range

# [PC T5] 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- 시작 전 `battery` 로 전압 확인. **7 V 이하면 충전기를 꽂아도 충전이 안 될 수 있다.**
- `pinky_sensor_adc` 가 `Package not found` 면 건너뛴다. rosbag 에서 `/us_sensor/range` 만 빼면 된다.
  (`~/pinky_pro` 가 fleet 패키지만 `--packages-select` 로 빌드했을 때 생기는 현상.
  `source ~/pinky_violet/install/setup.bash` 로 원본 워크스페이스에서 띄울 수도 있다.)
- teleop 은 `z` 로 속도를 낮춰 **0.15 m/s 정도** 로 천천히 돈다. 빠르면 모션블러로
  라벨링 가능한 프레임이 줄어든다.
- 녹화 종료는 `Ctrl+C`. `video.mp4` 와 `frames.csv` 가 `~/pinky_data/<세션>/` 에 남는다.

## 3. 세션 구성

세션을 나눠 찍어야 나중에 **세션 단위로 train / val 을 나눌 수 있다.**
연속 프레임은 거의 같은 그림이라 프레임 단위 랜덤 분할은 검증 점수를 부풀린다.

| 세션 | 내용 |
|---|---|
| `course_a` | 정방향 1바퀴, 차선 중앙 |
| `course_b` | 역방향 1바퀴 |
| `course_c` | 일부러 좌/우로 치우쳐 주행 — 중앙이 아닌 상황을 학습시킨다 |
| `course_d` | 코너·S자만 반복 — 가장 부족해질 구간 |
| `course_e` | 조명을 다르게 (형광등 일부 소등, 다른 시간대) |
| `crosswalk` | 횡단보도 접근을 여러 거리·각도에서 |

## 4. PC 로 가져와 프레임 추출

```bash
scp -r pinky@<핑키IP>:~/pinky_data ~/pinky_data
python3 tools/extract_frames.py ~/pinky_data/course_a --every 10 --max 60
```

`~/pinky_data/course_a/frames/` 를 Roboflow 에 올리고 Polygon Tool 로 `left` / `right` 를 그린다.
40~60 장이면 1차 모델은 돈다.

## 5. 시뮬레이션 재현용으로 함께 수집할 것

`pinky_gz_sim` 에 카메라 센서가 이미 정의되어 있어 Gazebo 에서 자율주행 시뮬레이션이 가능하다
(`params/pinky_bridge.yaml` 에 `camera/image_raw` 브릿지 항목만 추가하면 된다).
단, 시뮬 영상으로 학습한 YOLO 는 실물에 전이되지 않으므로 **시뮬은 제어·FSM·코스 기하
검증용** 이고, 그 안에서 차선은 HSV 임계값으로 뽑는다.

코스를 시뮬에 재현하려면 영상·rosbag 외에 아래가 필요하다.

**A. 코스 실측 (줄자)**
- 차선 폭(두 테이프 안쪽 거리), 테이프 자체 폭
- 직선 길이, 코너 반경(또는 진입·탈출점 간 거리), S자 두 원호의 반경과 변곡점
- 횡단보도 위치, 주행 방향 길이, 줄무늬 수·폭·간격
- 시작점·끝점, 전체 외곽 치수

**B. 부감 사진 — 가장 중요**
- 코스 전체를 위에서 수직에 가깝게 1장. 안 들어오면 겹치게 2~3장
- 사진 안에 **길이를 아는 자** 를 같이 놓는다 (스케일 복원용)
- 이것으로 바닥 텍스처 PNG 를 만들어 Gazebo 월드에 입힌다

**C. 벽** — 위치·높이·두께·색(흰색). 라이다 평면(바닥 12.5 cm)에 걸리는지

**D. 카메라 실측** — 지면 기준 렌즈 높이(URDF 6.5 cm), 틸트(URDF 8°), 각도 조절이 하드웨어적으로 되는지

**E. 로봇 거동** — teleop 최대 직진·회전 속도, 제자리 회전 가능 여부

**F. 조명** — 천장등 위치, 창문 유무, 바닥 광택, 시간대별 밝기 차이

**G. 장애물·신호등** — 쓸 물체의 크기·높이·색, 신호등 패널 크기·설치 높이

## 6. 도로망 그래프 편집기 (M0)

라이다 맵 위에 도로망(노드·엣지)을 찍어 `pinky_lane_station/config/road_graph.yaml` 을 만든다.
관제 PC 에서 실행하며 ROS 는 필요 없다 (PyQt5 · numpy · opencv · yaml).

```bash
cd ~/fleet_ws/src/pinky_pro_team11          # 또는 저장소 루트
PYTHONPATH=pinky_lane_station:pinky_fleet_station python3 -m pinky_lane_station.graph_editor \
    --map   pinky_fleet_station/config/map4.yaml \
    --graph pinky_lane_station/config/road_graph.yaml \
    --photo docs/course_aerial.jpg
```

| 키 | 동작 |
|---|---|
| `1` / `2` / `3` | 선택 / 노드 추가 / 엣지 추가 모드 |
| 노드 모드 좌클릭 | 노드 추가 (타입은 툴바 콤보박스, 라벨은 툴바 입력칸) |
| 엣지 모드 | 시작 노드 클릭 → 중간점 클릭… → 끝 노드 클릭. `Esc` 취소 |
| 선택 모드 | 노드·중간점 드래그 이동, `Del` 삭제, `O` 로 oneway 토글 |
| `Shift`+클릭 두 노드 | 경로 미리보기 (초록). `Ctrl+Shift` 로 두 번째 경로 (보라). **반대 방향 공유 엣지는 빨강** |
| `R 사진 정합` | 사진에서 외벽 안쪽 모서리 4점을 좌하→우하→우상→좌상 순으로 클릭 → 사진이 맵 위에 반투명으로 겹친다 |
| 휠 / 우클릭 드래그 / `F` | 줌 / 팬 / 화면 맞추기 |
| `Ctrl+S` | 저장 (검증 실패 시 저장하지 않는다) |

현재 커밋된 `road_graph.yaml` 은 **사진에서 읽은 토폴로지만 맞고 좌표는 초안**이다.
사진 정합 후 노드를 끌어 테이프 중심선에 맞춘다. 결과 확인용 렌더링: `docs/road_graph_overlay.png`.
