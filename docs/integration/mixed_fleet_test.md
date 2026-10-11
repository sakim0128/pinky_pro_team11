# 혼합 함대 시험 시나리오 — 핑키1 팀 방식 · 핑키2 Nav2 (도커 로봇 + 실제 중계 · 태블릿 + 녹화 영상)

> **도구의 위치.** 이 문서가 부르는 `tools/mixed_fleet/`(시험 트리 조립 · 재생 서버 · 가짜 베이스 · 도커 로봇 · 자동 판정)은 이 저장소에 없다.
> `rkd1rjs2/robot_mini_project_pinky` 의 `claude/zen-ptolemy-7xdrtp` (`08492cf`) `tools/mixed_fleet/` 에서 관리하고, 아래 명령은 그 체크아웃에서 실행한다.
> 팀 브랜치에는 이 문서만 둔다.

**한 줄.** 로봇은 도커의 가상 로봇 둘만 쓰고, 중계 장비와 태블릿 비전은 실제로 돌리며, 태블릿 카메라 입력만 어제 확보한 프레임의 재생으로 바꾼다.
핑키1 은 팀의 기존 주행 방식(`lane_agent_node` + 항공뷰 위치), 핑키2 는 Nav2 주행(`hybrid_agent_node` + 게이트 + Nav2)이다.
한 미션 안에서 **서로 다른 주행 방식의 두 로봇이 같은 교차로를 코디네이터 하나의 예약으로 주고받는지**를 사람이 화면으로 본다.

| | 실물 / 가상 | 무엇 |
| :-- | :-- | :-- |
| 중계 게이트웨이 · 코디네이터 · 도메인 브리지 | **실물** (중계 노트북, 도메인 8) | 팀 브랜치 + 우리 수정을 합친 시험 트리로 띄운다 |
| 태블릿 비전 (`tablet/vision`) | **실물 코드** | 카메라 주소만 재생 서버로 바꾼다. 마커 검출 · 호모그래피 · 송신은 그대로 |
| 카메라 영상 | **녹화 재생** | 어제 확보한 프레임을 MJPEG 로 튼다 (`replay_frames_mjpeg.py`) |
| 핑키1 | **가상** (도커, 도메인 10) | 가짜 베이스 + 실제 `pose_fuser_node` + `lane_agent_node` |
| 핑키2 | **가상** (도커, 도메인 11) | 가짜 베이스 + 실제 `drive_command_gate` + `hybrid_agent_node` + **가짜 Nav2** (또는 진짜, 아래) |

> **핑키1 은 열린 고리다.** 핑키1 의 위치는 녹화 영상 속 로봇이 만든다. 도커 핑키1 의 바퀴가 아니다. 그래서 영상 속 로봇이 움직이는 대로
> `lane_agent_node` 가 반응하고(교차로 정지 · 통과), 도커 핑키1 이 낸 속도 명령은 위치에 되돌아오지 않는다. 정지 명령에도 재생은 멈추지 않는다.
> 핑키2 는 닫힌 고리다 — 가짜 베이스가 속도를 적분해 자기 위치를 만든다.

## 0. 검증 상태 (먼저 읽는다)

| 구간 | 상태 |
| :-- | :-- |
| 재생 → 태블릿 비전 → 중계 게이트웨이 → 로봇 위치, 코디네이터 중재, 교차로 순서, 도착, DONE, 정지 · 재개 | **샌드박스에서 확인함** — 실제 중계 · 태블릿 코드를 **합성 프레임**과 가짜 베이스 로봇으로 돌려 `scenario_check.py` 10 항목 통과(정지 시험은 별도 실행) |
| 도메인 브리지 경유 (도메인 8 ↔ 10 · 11) | **확인 못 함** — 샌드박스는 한 도메인 안에서 돌렸다. 브리지 자체는 빌드까지만 |
| 컨테이너 ↔ 실제 중계 노트북 사이 DDS (유니캐스트) | **확인 못 함** |
| 어제 녹화 프레임으로의 실행 | **확인 못 함** — 녹화가 이 환경에 없다. 아래 §2 사전 점검이 그 대신이다 |
| 진짜 Nav2 · 가제보 · 실제 로봇 | **확인 못 함** |
| 태블릿 실기기(Termux) 에서 OpenCV · aruco 설치 | **확인 못 함** |

리허설 값(참고): 핑키2 가 `BL_J` 앞에서 핑키1 때문에 대기 → 교차로 점유가 핑키1 → 핑키2 로 넘어감 → 두 로봇 ARRIVED → 목표 구역 이벤트 뒤 DONE.
이 값은 합성 프레임에서 나온 것이므로, **실제 녹화에서는 시간 · 순서가 다를 수 있다.**

## 1. 준비물

- 중계 노트북: ROS 2 Jazzy, 이 저장소 체크아웃, 팀 브랜치 fetch (`git fetch team11 mini_project_integration_stand`), Docker.
- 태블릿 비전을 돌릴 곳: 중계 노트북 자신(가장 쉽다) 또는 태블릿. OpenCV(aruco 포함) 필요.
- 어제 녹화 프레임 폴더 또는 zip (JPEG). **연속 프레임이어야 하고**, 경기장 네 모서리 마커 40–43 과 로봇 마커 30 이 보여야 한다.
- 로봇 이미지: 기본 `pinky-robot-farm:jazzy` (Nav2 · `nav2_msgs` · CycloneDDS 가 들어 있다). 순정 `ros:jazzy-ros-base` 로는 핑키2 가 못 뜬다.
- 값: `RELAY_HOST_IP`(중계 노트북 주소), `RELAY_VISION_API_KEY`(공유 키). **둘 다 저장소에 적지 않는다.**

## 2. 사전 점검 — 녹화가 이 시나리오에 맞는가

이 시나리오는 녹화 속 로봇 30 이 **BL → 교차로 J → TR** 을 지나야 성립한다(도로망 `pinky_lane_station/config/road_graph.yaml`: BL(0.20,0.20) · J(1.18,0.64) · TR(2.15,1.08)).
녹화가 다른 곳을 지나면 핑키1 이 경로에서 벗어난 것으로 보고 멈춘다. 그래서 미션을 걸기 전에 본다.

```bash
# 재생만 띄우고, 태블릿이 낸 좌표를 눈으로 본다
python3 tools/mixed_fleet/replay_frames_mjpeg.py --source <프레임 폴더|zip> --fps 3.4 --port 18100
```

태블릿을 dry-run(`--relay-url` 없이)으로 같은 주소에 붙여 로그의 좌표를 읽는다. 확인할 것:

| 점검 | 통과 기준 | 어긋나면 |
| :-- | :-- | :-- |
| 모서리 마커 | 네 개가 연속 검출, 재투영 오차 < 4 px | 태블릿 세션 실측(`tablet_pose.md` §2)과 같은지 본다. 이 값이 크면 좌표는 나가지 않는다 |
| 로봇 30 궤적 | 처음 BL 근처(±0.15 m), 중간에 J 근처, 끝이 TR 근처 | `--start` · `--end` 로 맞는 구간만 자른다 |
| 좌표 범위 | 0..2.36 × 0..1.28 m 안 | 경기장 밖 좌표(`tablet_pose.md` §2 의 음수 샘플)가 나오면 그 구간은 자른다 |
| 프레임률 | 3 fps 안팎 | `--fps` 를 녹화 때 값으로 (기본 3.4) |
| 로봇 31 | 영상에 없어도 된다 | 있으면 핑키2 위치가 태블릿에서도 나가 **가짜 베이스와 두 출처가 된다** — 그 구간은 자른다 |

⚠️ 어제 녹화가 이 조건을 만족한다는 것은 **확인하지 못했다.** 만족하지 않으면 시나리오를 못 돌리는 것이지 코드 결함이 아니다.

## 3. 기동 순서

모든 명령은 **시험 트리**(§3-1)에서 실행한다. 시험 트리는 어느 브랜치에도 올리지 않는다.

### 3-1. 시험 트리

```bash
git fetch team11 mini_project_integration_stand
./tools/mixed_fleet/build_test_tree.sh ~/mixed_tree      # 팀 브랜치 + 우리 하이브리드 파일 + 게이트웨이 패치 2개 + 혼합 프로파일
```

팀 브랜치에는 핑키2 방식(Nav2 하이브리드)이 없고 우리 브랜치에는 팀 최신 주행이 없다. 어느 한쪽만으로는 이 조합이 안 돌아 합친다.

### 3-2. 중계 노트북 (도메인 8)

```bash
cd ~/mixed_tree
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pinky_fleet_msgs pinky_lane_msgs pinky_lane_station pinky_fleet_agent pinky_fleet_station && source install/setup.bash
export RELAY_VISION_API_KEY=<공유 키>

relay_station/domain_bridge/install_bridge.sh                                   # 도메인 브리지 (pinky1_control · pinky2_control)
ROS_DOMAIN_ID=8 python3 relay_station/gateway_web/gateway_web_server.py --port 8889 --no-camera
```

브라우저 `http://<중계 노트북>:8889/fleet_control_v2.html` (`/` 는 옛 화면). 설정 탭 **① 좌표 프로파일 → `team11_map5_mixed`**.
카드의 라벨: "혼합 시험 map5 — pinky1 팀 방식(lane) BL→TR · pinky2 Nav2(hybrid) BR→BL".

### 3-3. 도커 로봇 둘

```bash
MIXED_TREE=~/mixed_tree RELAY_HOST_IP=<중계 노트북 주소> \
  docker compose -f tools/mixed_fleet/compose.mixed-robots.yml up
```

핑키2 의 Nav2 는 기본이 **가짜**다(`ROBOT_NAV2=fake`, 직선 추종). 진짜 Nav2 를 쓰려면 `ROBOT_NAV2=real` 로 주고 그쪽 스택을 따로 띄운다 — 이 경우 지도 프레임(map5)과 농장 월드가 맞는지 먼저 본다(**미확인**).

### 3-4. 재생 + 태블릿 비전

```bash
python3 tools/mixed_fleet/replay_frames_mjpeg.py --source <프레임 폴더|zip> --fps 3.4 --port 18100
# 태블릿이 다른 기기면 --bind 0.0.0.0 (같은 LAN 안에서만)
```

`tablet/vision/config/cameras.yaml` 의 `cameras.overhead.device` 를 `http://<재생 서버>:18100/video` 로 바꾼 사본을 쓴다(원본은 두고 `--config-dir` 로 가리킨다).

```bash
python3 -m tablet.vision.runtime.entrypoint --config-dir <바꾼 설정 폴더> \
    --relay-url http://<중계 노트북>:8889 --allow-pending-config --api-key <공유 키>
```

`--allow-pending-config` 는 설정 세 파일이 `FIELD_CONFIG_PENDING` 인 동안 송신을 허용한다. 좌표는 **꼭짓점 가정값** 기준이다(`tablet_pose.md` §2).

### 3-5. 시작

V2: 설정 탭 **② 로봇 지도 전환(map5) → ③ 초기 위치 → 주행 시작.**
③ 은 출발 노드를 로봇 초기 위치로 보낸다. 핑키1 은 영상 속 로봇이 BL 근처에서 **움직이기 시작하는 시점**에 시작을 누른다.
(재생은 반복 재생이므로 한 바퀴가 도는 동안 타이밍을 잡는다. 반복이 끝나 처음으로 돌아가면 핑키1 위치가 순간이동한다 — `--once` 로 한 번만 틀면 끝난 뒤 좌표가 끊겨 3 s 뒤 핑키1 이 선다.)

## 4. 사람이 확인하는 것

| # | 언제 | 어디서 | 기대 |
| :-- | :-- | :-- | :-- |
| H1 | 기동 직후 | V2 로봇 카드 | 두 로봇이 응답하고 위치가 있다. 주행 방식(핑키1 `lane` · 핑키2 `nav2`)은 화면이 아니라 `GET /api/fleet/status` 또는 `scenario_check.py` P0 로 본다 |
| H2 | 기동 직후 | `http://<중계 노트북>:8080` live 웹 | 핑키1 P′(태블릿 좌표)가 영상 속 로봇을 따라 움직임. 핑키2 P′ 는 **없음**(정상 — 태블릿이 31 을 안 봄) |
| H3 | 시작 직후 | V2 미션 카드 | 미션 `RUNNING`, 두 로봇 경로: 핑키1 BL→J→TR, 핑키2 BR→J→BL |
| H4 | 핑키1 이 J 접근 | V2 로봇 카드 / `GET /api/fleet/status` | 핑키1 `JUNCTION_STOP` 약 1 s → 통과 허가 후 `JUNCTION_PASS` |
| H5 | 핑키2 가 J 에 접근 | 같은 곳 | 핑키1 이 J 를 쥐고 있는 동안 핑키2 는 `BL_J`(또는 J 진입 구간) 앞에서 **대기**: `waiting_for` 가 채워지고 `blocked_by` 가 `pinky1`. 화면에 대기 사유가 보여야 한다 |
| H6 | 핑키1 이 J 를 벗어남 | `node_holders` | J 의 점유가 `pinky1` → `pinky2` 로 넘어가고 핑키2 가 다시 출발(Nav2 목표 진행) |
| H7 | 진행 중 | 핑키2 카드 | 핑키2 의 속도 명령은 **게이트를 거친다**: 코디네이터가 멈추라 하면 Nav2 가 목표를 진행 중이어도 0 |
| H8 | 두 로봇 도착 | 로봇 카드 | 둘 다 `ARRIVED`. 이때 미션은 아직 `RUNNING` — 목표 구역 이벤트를 기다린다 |
| H9 | 구역 이벤트 뒤 | 미션 카드 | `DONE`. 구역 이벤트 없이는 DONE 이 되지 않는다(**정상 동작**) |
| H10 | (별도 실행) 주행 중 정지 | 정지 → 재개 | 정지 후 핑키2 실제 속도 0. 재개 뒤 미션 `RUNNING` 으로 복귀. 핑키1 은 위치가 재생을 따라가므로 **위치는 계속 바뀐다** |

### 목표 구역 이벤트 (H9)

실제 구역 카메라가 없다. 태블릿 `zones.yaml` 은 `PENDING` 이다. 로봇이 목표에 도착하면 사람이 대신 보낸다(또는 §5 의 도구가 보낸다):

```bash
curl -s -X POST http://<중계 노트북>:8889/api/vision/zone_event \
  -H 'Content-Type: application/json' -H 'X-API-Key: <공유 키>' \
  -d '{"robot_name":"pinky1","camera_id":"CAM_TR","zone_id":"TR","event_type":"PRESENT","confidence":0.95}'
```

`zone_id` 는 **목표 노드 이름**(핑키1 → `TR`, 핑키2 → `BL`)과 같아야 하고, 로봇이 `ARRIVED` 인 동안 보내야 한다(유효 시간 `goal_event_timeout` 3 s).

## 5. 자동 판정 (`scenario_check.py`)

사람이 화면으로 보는 것을 게이트웨이 상태 API(`/api/fleet/status` 등)로 같이 잰다. ROS 도 필요 없고 표준 라이브러리만 쓴다.

```bash
# 관찰만 (기본은 읽기 전용) — 사람이 V2 에서 ③ → 주행 시작
python3 tools/mixed_fleet/scenario_check.py --gateway http://127.0.0.1:8889

# 시작까지 · 구역 이벤트까지 (움직이는 명령을 낸다)
python3 tools/mixed_fleet/scenario_check.py --gateway http://127.0.0.1:8889 --do-start --send-zone-events --key <공유 키>

# 정지 · 재개 (재생은 정지에 멈추지 않으므로 따로 한 번 더)
python3 tools/mixed_fleet/scenario_check.py --gateway http://127.0.0.1:8889 --do-start --stop-test
```

| 항목 | 판정 |
| :-- | :-- |
| P0 구성 | 핑키1 `lane` · 핑키2 `nav2` · 프로파일 `team11_map5_mixed` |
| P1 위치 | 두 로봇 응답 · 위치 있음 |
| S1 시작 | 미션 `RUNNING` |
| S2 재생 위치 | 핑키1 의 지도 위치가 0.3 m 이상 움직임 — **영상 속 로봇이 움직인 결과** |
| S3 교차로 | 핑키1 이 `JUNCTION_STOP` 또는 `JUNCTION_PASS` 를 거침 |
| S4 양보 | 핑키2 `waiting_for` 채워짐 · `blocked_by == pinky1` |
| S5 순서 | J 점유가 핑키1 → 핑키2 |
| S6 도착 | 두 로봇 `ARRIVED` |
| S7 완료 | 구역 이벤트 뒤 미션 `DONE` (`--send-zone-events`) |
| X1 정지·재개 | 정지 → 속도 0 → 재개 → `RUNNING` (`--stop-test`, 다른 항목은 SKIP) |

전체 통과 기준: **FAIL 0.** SKIP 은 그 실행에서 판정하지 않은 항목이다. `--dump` 로 표본을 남기면 실패 뒤 원인을 볼 수 있다.
`fleet_monitor.py --gateway … --out timeline.jsonl` 는 상태를 1 초마다 JSON 줄로 남긴다.

**S3–S5 는 녹화 타이밍에 달려 있다.** 핑키2 가 J 에 닿기 전에 핑키1 이 J 를 이미 지나갔다면 양보가 일어나지 않아 S4 가 FAIL 한다.
이것은 코디네이터 결함이 아니라 겹침이 없었다는 뜻이다. 핑키2 시작을 늦추거나(`PINKY2_START`), 핑키1 이 J 에 있는 시점에 시작하도록 재생 구간을 조정한다.

## 6. 실패했을 때

| 증상 | 원인 후보 | 할 일 |
| :-- | :-- | :-- |
| P0 FAIL, 프로파일 이름이 다르다 | V2 ① 에서 안 골랐다 | `team11_map5_mixed` 선택 |
| 핑키1 위치가 안 뜬다 | 태블릿이 안 보낸다 / 브리지 다운링크 없음 | 태블릿 로그에 `Relay egress is DISABLED` 가 있는지, `curl` 로 `/api/vision/pose_fix` 응답의 `overhead_subscribers` 가 0 인지 (`tablet_pose.md` §8) |
| 핑키1 이 `LANE_LOST` → `LANE_SEARCH` | 카메라가 없어 `LanePath` 가 오지 않는다 | 이 시험의 알려진 한계. 위치 기반 주행과 교차로 판정은 계속된다 |
| 핑키1 이 3 s 뒤 선다 | 좌표가 끊겼다 (재생이 끝났거나 마커가 안 보이는 구간) | 재생 구간 조정 |
| 핑키1 이 경로에서 벗어나 멈춘다 | 녹화 궤적이 BL→J→TR 이 아니다 | §2 로 돌아간다 |
| 핑키2 위치가 튄다 | 영상에 마커 31 이 있어 태블릿도 핑키2 좌표를 보낸다 | 그 구간을 자른다 |
| V2 ② · ③ 이 500 (NameError `latched_robots`) | 함수 누락 결함 | 팀 브랜치 반영 완료 (수정됨) |
| 핑키2 컨테이너가 안 뜬다 | `nav2_msgs` 없는 이미지 | 기본 `pinky-robot-farm:jazzy` |
| 컨테이너가 중계 토픽을 못 본다 | 방화벽 / 도메인 브리지 / RMW 불일치 | `RELAY_HOST_IP`, Fast DDS 기본 RMW 일치 확인, `ros2 topic list` 를 도메인 10 · 11 · 8 에서 각각 |

## 7. 이 시험이 보지 않는 것

- **실제 주행의 정확도.** 가짜 베이스에는 슬립 · 지연 · 관성이 없다. 핑키2 는 이상적인 추종이다.
- **차선 인식 · 장애물.** 카메라 · 라이다 · 벽이 없다.
- **핑키1 의 도커 바퀴 → 위치.** 열린 고리라 `lane_agent_node` 의 조향 추종은 검증되지 않는다.
- **진짜 Nav2 의 경로 계획 · 코스트맵.** 기본은 가짜 Nav2 다.
- **Wi-Fi · 실제 태블릿 지연.** 재생은 로컬 스트림이다.

이 시험이 보는 것: 서로 다른 주행 방식의 두 로봇이 **한 코디네이터 아래에서** 경로를 받고, 같은 교차로를 예약으로 주고받고, 대기 사유가 화면에 나오고,
도착과 미션 완료가 구역 이벤트와 맞물리고, 정지 명령이 두 방식 모두에서 속도 0 으로 이어지는지.

## 8. 남은 일

- [ ] 어제 녹화로 §2 를 실제로 해 보고 궤적이 BL→J→TR 인지 확인한다.
- [ ] 브리지(도메인 8 ↔ 10 · 11)와 컨테이너 ↔ 중계 DDS 를 실제로 돌려 본다.
- [ ] 진짜 Nav2 로 핑키2 를 돌릴 때의 지도 프레임 정합(map5 ↔ 농장 월드).
- [x] 팀 브랜치의 `latched_robots` 누락 반영 완료 (게이트웨이 V2 ②·③ 버튼 500 오류 해결).
- [x] DDS Fast DDS 통일 반영 (5GHz 공유기 환경에 맞춰 CycloneDDS 사용 배제).
