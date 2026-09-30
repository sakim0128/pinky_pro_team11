# 태블릿 좌표 → 로봇 위치 (rkd1rjs2 팀원, 2026-09-29 · 09-30 갱신)

태블릿은 로봇 위 마커를 보고 **로봇 좌표**를 map5 로 계산해 중계에 HTTP 로 **계속** 보낸다. 마커 도착 판정과 갈림길 방향은
로봇(`lane_agent_node`)과 중계 코디네이터가 한다. 태블릿이 바닥 마커 번호나 "다음 마커" 를 알릴 필요는 없다.

> **소스 위치.**
> - **태블릿 비전 소스**: 이 브랜치 `tablet/vision/` (`7c59c7c` 로 들어옴, 원 저장소 main 의 것). 송신기를 새로 짤 필요가 없다.
> - **중계 게이트웨이**: `rkd1rjs2/robot_mini_project_pinky` `e59d521` (`relay_station/gateway_web/`)에서 관리한다. 태블릿이 보낸
>   `POST /api/vision/pose_fix` 를 받아 `/pinkyN/overhead_pose` 로 팀 로봇 토픽에 낸다. **이 브랜치의 중계 코드에는 이 수정이 없다** —
>   태블릿 좌표를 쓰려면 중계 PC 를 그 판으로 띄운다.
> - **도메인 브리지**: 이 브랜치의 `pinkyN_control.yaml` 에 필요한 다운링크(`/pinkyN/overhead_pose` · `/pinkyN/lane_path`)가 이미 있다.

## 1. 태블릿 비전이 하는 일 (`tablet/vision/`)

- **화면 → 지도 변환을 스스로 맞춘다.** 경기장 네 모서리 기준 마커(`40 tl` · `41 tr` · `42 br` · `43 bl`, DICT_4X4_50)를 매 프레임 찾아
  픽셀 → map 좌표 호모그래피 H 를 다시 계산하고 이전 값과 섞는다(`homography_alpha` 0.3). 알고리즘은 팀 상부 추적기
  (`overhead_tracker_node`)와 같은 계열이고, 팀 추적기는 H 를 설정에 미리 넣어야 하는 반면 태블릿은 현장에서 맞춘다.
- **로봇 좌표.** 로봇 위 마커(`30` = pinky1, `31` = pinky2)의 네 꼭짓점을 H 로 바꿔 x · y · yaw 를 구한다.
- **막는 조건.** 기준 마커가 4개 미만이면 직전 H 를 그대로 쓴다. H 가 없거나 재투영 오차가 `max_reproj`(4.0 px)를 넘으면 좌표를 보내지 않는다.
  3 s 동안 좌표가 안 가면 로봇이 선다.
- **송신 게이트.** 설정 세 파일(`markers` · `cameras` · `zones`)이 모두 `READY` 여야 중계로 보낸다. 지금은 셋 다 `FIELD_CONFIG_PENDING` 이라
  `--allow-pending-config` 를 줘야만 나간다.
- **마커 번호가 팀 추적기와 다르다.** 태블릿은 로봇 마커 30 · 31, 팀 추적기는 1 · 2. 같은 로봇에 둘을 함께 쓰지 않으므로 충돌은 없다.

## 2. 현장 녹화 프레임 실측 (태블릿 세션 보고, 2026-09-30)

Galaxy Note 10 으로 찍은 09-28 · 09-29 녹화 프레임에 `OverheadLocalizer` 를 돌린 결과다. 카메라 센서는 90° 돌려 놓은 상태다.

| 항목 | 실측 전 가정 | 보고된 결과 |
| :-- | :-- | :-- |
| 모서리 마커 검출 | 40 · 41 · 42 · 43 | 09-29 녹화 3개에서 네 개 모두 연속 검출 |
| **모서리 마커가 붙은 각도(yaw)** | 전부 0 | 41번 0.6° · 40번 91.3° · 42번 178.4° · 43번 −179.0° |
| 재투영 오차 (`rec-20260929-205632-1` 100번째 프레임) | yaw 전부 0 → **20.36 px** (4.0 px 초과, 송신 차단) | 실측 yaw 반영 → **0.998 px** |
| 모서리 마커 크기 | 0.10 m | H 를 되돌려 0.100 × 0.102 m 로 복원 |
| 로봇 위 마커 | 30 · 31 대 팀 추적기 1 · 2 | `rec-20260928-220304-1` 에서 로봇 위 30번 검출 |

`markers.yaml` 의 `reference.markers[].yaw` 에 위 각도를 반영했다(40 → 1.5708 · 41 → 0 · 42 → 3.1416 · 43 → −3.1416).

**재현한 것 / 못 한 것 (이 문서를 갱신한 쪽).**

| 확인 | 결과 |
| :-- | :-- |
| `python3 -m pytest tablet/vision/tests` | **43 통과** (ROS 2 Jazzy 컨테이너, OpenCV 4.10 · aruco) — 재현함 |
| 위 표의 프레임 분석(20.36 → 0.998 px 등) | 녹화가 이 환경에 없어 **재현 못 함** — 보고를 옮겨 적은 것 |

**읽을 때 주의할 점.**

- **재투영 오차가 작다는 것은 네 마커가 서로 맞는다는 뜻이다.** 지도 좌표가 맞다는 뜻은 아니다. 모서리 마커 x · y 는 아직 경기장 꼭짓점 가정값
  (0 / 2.34, 0 / 1.26)이고 설정 주석도 `PLACEHOLDER-COORD` 다. 각도만 실측했다.
- **보고된 샘플 로봇 좌표가 경기장 밖이다.** `x −0.667, y −1.114, yaw −156.5°` 는 0..2.34 × 0..1.26 m 안이 아니다. 로봇이 경기장 밖에 있었던 프레임인지,
  마커 배치(tl · tr · br · bl)가 화면 방향과 어긋난 것인지 아직 모른다.
- **"화면 비율 약 1.98" 은 일치의 근거가 아니다.** 원근이 섞인 화면 비율이고, 경기장 2.34 × 1.26 은 1.86 이다.
- 그래서 **로봇을 도로망 노드 위(줄자로 잰 점)에 놓고 태블릿이 낸 좌표와 대조**하는 것이 다음 확인이다.

## 3. 태블릿에서 실행

태블릿(Termux/PRoot)에서:

```bash
export RELAY_VISION_API_KEY=<중계 운영자에게 받은 키>      # 키는 저장소에 적지 않는다
python3 -m tablet.vision.runtime.entrypoint --relay-url http://<중계 PC>:8889

# 설정 상태가 FIELD_CONFIG_PENDING 인 동안에는 송신을 강제로 허용해야 나간다
python3 -m tablet.vision.runtime.entrypoint --relay-url http://<중계 PC>:8889 --allow-pending-config
```

`--relay-url` 의 코드 기본값은 예전 현장 LAN 주소다. 반드시 `--relay-url` 또는 환경 변수 `RELAY_URL` 로 현재 중계 PC 를 준다.

## 4. 흐름

1. **태블릿**이 로봇 위 마커로 map5 좌표 x · y · yaw 를 구해 중계 `POST /api/vision/pose_fix` 로 보낸다.
2. **중계 게이트웨이**가 키·형식을 검사한 뒤 같은 좌표를 `/pinkyN/overhead_pose`(`geometry_msgs/PoseStamped`, frame `map`)로 낸다(도메인 8).
   기존 `/pinkyN/pose_fix`(PoseFix)도 그대로 낸다.
3. **도메인 브리지**가 로봇 도메인(pinky1 = 10, pinky2 = 11)으로 내린다.
4. **로봇 `pose_fuser_node`**(`lane_robot.launch.xml`, `use_overhead:=True` 기본)가 odom 과 합쳐 TF map→odom 을 만든다.
5. **로봇 `lane_agent_node`**가 그 위치로 경로를 따라가고 갈림길·도착을 판정한다.

경로(`/pinkyN/route`)는 출발 전에 한 번, 통과 허가(`/pinkyN/lane_command` 의 `clear_until_idx`)는 10 Hz 로 코디네이터가 따로 보낸다.
천장 카메라 추적기(`overhead_tracker_node`, [`overhead_tracker.md`](overhead_tracker.md))와 **같은 토픽**이다 — 로봇은 출처를 가리지 않는다.

## 5. 보내는 방법 (HTTP 스펙)

```http
POST http://<중계 PC>:8889/api/vision/pose_fix
Content-Type: application/json
X-API-Key: <중계 운영자에게 받은 키>

{"robot_name": "pinky1",
 "header": {"frame_id": "map", "stamp": {"sec": 1790688000, "nanosec": 0}},
 "x": 1.18, "y": 0.64, "yaw": 1.57,
 "seq": 12, "marker_id": 30}
```

| 필드 | 필수 | 형식 | 뜻 |
| :-- | :-- | :-- | :-- |
| `robot_name` | 예 | `pinky1` · `pinky2` | 어느 로봇의 좌표인지. 로봇마다 요청 하나 |
| `header.frame_id` | 예 | `map` 고정 | 다른 값이면 400 |
| `header.stamp.sec` · `nanosec` | 예 | 0 이상의 정수 | 캡처 시각. 형식만 검사하고 로봇은 쓰지 않는다 |
| `x` · `y` | 예 | 실수, m | map5 좌표. 원점은 경기장 좌하단 |
| `yaw` | 예 | 실수, rad | 로봇 전방이 +x 에서 반시계로 이루는 각 |
| `seq` | 아니오 | 0 이상의 정수 | 보낼 때마다 1씩 — 로그 추적용 |
| `marker_id` | 아니오 | 정수 | 좌표를 만든 로봇 마커 번호 (pinky1: 30, pinky2: 31) |
| `reproj_error` · `n_markers` · `pipeline_latency` | 아니오 | 0 이상 | 진단용. 로봇 위치에는 쓰지 않는다 |

키는 `Authorization: Bearer <키>` 로 보내도 된다. 키 없이 띄운 중계는 이 입구를 전부 401 로 닫는다.

## 6. 규칙

| 규칙 | 값 | 어기면 |
| :-- | :-- | :-- |
| 좌표계 | map5 — 원점 좌하단, x 0..2.36 m · y 0..1.28 m (경기장 실측 2.34 × 1.26 m 는 그 안) | 경로에서 벗어난 것으로 보고 엉뚱하게 조향 |
| yaw | 로봇 전방 기준, rad, 반시계 + | 조향이 반대로 간다. 마커를 다르게 붙였으면 로봇 `pose_fuser` 의 `yaw_offset` · `offset_x` |
| 주기 | 멈춰 있을 때도 계속. 초당 5 회 이상 권장 | 3 s 끊기면(`fix_timeout`) 로봇이 "위치(TF map→base) 없음 — 정지" |
| 한 로봇 한 출처 | 태블릿과 천장 추적기를 같은 로봇에 동시에 쓰지 않는다 | 두 값이 섞여 위치가 튀거나 멈춘다 |

**마커 위에 섰을 때만 보내는 방식은 안 된다** — 마커 사이 구간에서도 위치가 있어야 경로를 따라간다.
0.30 m 또는 45° 넘게 튄 값은 같은 후보가 2 번 이어져야 채택된다(`gate_dist` · `gate_yaw_deg` · `confirm`).

## 7. 로봇이 받은 좌표로 하는 일

로봇은 바닥 마커 번호를 모른다. 바닥 마커는 도로망 노드(`pinky_lane_station/config/road_graph.yaml` 의 BL · BR · TR · J)의
실측 기준일 뿐이고, 로봇은 자기 좌표를 경로 좌표에 겹쳐 판단한다.

| 상황 | 로봇이 하는 일 | 기준값 |
| :-- | :-- | :-- |
| 달리는 중 | 좌표를 경로에 겹쳐 진행 번호를 구하고 앞쪽 경로 점을 향해 조향. 카메라 차선 보정을 더한다 | 앞 0.25 m |
| 갈림길 노드 근처 | 멈춘 뒤 통과 허가를 기다린다. 허가가 나면 카메라 없이 경로 좌표만 따라 서행 통과 | 반경 0.25 m · 1 s 정지 · 속도 40 % |
| 갈 방향 | 출발 전에 받은 경로의 엣지 순서를 따른다. 들어온 방향도 경로에 이미 있다 | 코디네이터가 정함 |
| 목표 노드 | 남은 거리가 기준 안이고 목표까지 허가가 나면 도착 | 0.10 m |
| 좌표가 끊김 | 위치를 버리고 정지 | 3 s |

같은 갈림길을 두 로봇이 동시에 지나지 않게 하는 것은 코디네이터의 구간 예약이다.

## 8. 확인 방법

```bash
# 같은 Wi-Fi 의 노트북에서
curl -s -X POST http://<중계 PC>:8889/api/vision/pose_fix \
  -H 'Content-Type: application/json' -H 'X-API-Key: <키>' \
  -d '{"robot_name":"pinky1","header":{"frame_id":"map","stamp":{"sec":0,"nanosec":0}},"x":0.20,"y":0.20,"yaw":0.42}'
# → {"accepted": true, ..., "overhead_topic": "/pinky1/overhead_pose", "overhead_subscribers": 2, ...}

# 로봇에서 — accepted 가 늘고 age 가 1 s 아래면 정상
ros2 topic echo /pinky1/fix_status
```

| 증상 | 원인 | 할 일 |
| :-- | :-- | :-- |
| 401 | 키가 틀렸거나 중계에 키가 없음 | 중계 운영자에게 키 확인 |
| 400 과 `reason` | 필드 형식 오류(예: `frame_id` 가 map 이 아님, stamp 가 실수) | `reason` 에 적힌 필드를 고친다 |
| `overhead_topic` 이 `null` | 중계가 이 경로를 꺼 둠(`RELAY_POSE_FIX_TO_OVERHEAD=0`) | 중계 운영자에게 확인 |
| `overhead_subscribers` 가 0 | 브리지가 안 떴거나 로봇 `pose_fuser` 가 없음 | 브리지 · 로봇 런치 확인 |
| 태블릿 로그에 송신이 안 나감 | 설정이 `FIELD_CONFIG_PENDING` | `--allow-pending-config` 또는 실측 뒤 `READY` |
| 로봇이 "위치 없음 — 정지" | 좌표가 3 s 넘게 끊김 | 보내는 주기와 Wi-Fi |
| `fix_status` 의 rejected 만 는다 | 좌표가 튀거나 다른 출처와 섞임 | 천장 추적기가 같은 로봇을 보는지 |
| 로봇이 엉뚱한 방향으로 튼다 | yaw 기준·부호가 다름 | 로봇을 +x 방향으로 놓고 yaw 가 0 근처인지 |

## 9. 남은 일

- [x] 태블릿 비전 소스(`tablet/vision/`)가 이 브랜치에 있고 시험 43개가 통과한다.
- [x] 모서리 마커 부착 각도(yaw) 실측을 `markers.yaml` 에 반영했다(태블릿 세션 보고).
- [ ] **모서리 마커 네 개의 중심 x · y 를 줄자로 재서 `markers.yaml` 에 넣고** 세 설정을 `READY` 로 바꾼다. 지금은 꼭짓점 가정값이다.
- [ ] **경기장 밖으로 나온 샘플 좌표(위 §2)의 원인을 확인한다.** 로봇을 줄자로 잰 점에 놓고 태블릿 좌표와 5 cm 안으로 맞는지, yaw 도 맞는지 본다.
- [ ] 도로망 노드(`road_graph.yaml` 의 BL · BR · TR · J, 지금은 임시값)를 바닥 마커 실측 위치로 바꾼다.
- [ ] **`relay_station/fleet/config/road_graph.yaml` 정리.** 태블릿 mock(`START_A` · `GOAL_C` 노드)이 쓰려고 `7c59c7c` 에서 다시 넣은 복사본인데,
      도로망은 `pinky_lane_station/config/road_graph.yaml` 하나라는 팀 결정과 충돌해 `test_assign_map5.py` 의
      `test_profile_points_at_the_single_graph_and_map5_without_copies` 가 실패한다(직전 커밋 `9164811` 에서는 통과).
- [ ] 실물 로봇 주행에서 `pose_fuser` 오차와 조향 추종을 끝까지 확인한다.
- [ ] 열린 질문: 태블릿 한 대가 두 로봇을 한 화면에서 볼 수 있는지, 두 대가 필요한지.
