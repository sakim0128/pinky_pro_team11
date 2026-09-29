# 태블릿 좌표 → 로봇 위치 (rkd1rjs2 팀원, 2026-09-29)

태블릿은 로봇 위 마커를 보고 **로봇 좌표**를 map5 로 계산해 중계에 HTTP 로 **계속** 보낸다. 마커 도착 판정과 갈림길 방향은
로봇(`lane_agent_node`)과 중계 코디네이터가 한다. 태블릿이 바닥 마커 번호나 "다음 마커" 를 알릴 필요는 없다.

> **소스 위치.** 중계 게이트웨이 수정은 rkd1rjs2/robot_mini_project_pinky 에서 관리한다(`e59d521`, `relay_station/gateway_web/`).
> 이 브랜치의 `relay_station/` 에는 들어 있지 않다 — 태블릿 좌표를 쓰는 중계는 rkd1rjs2 가 그 판으로 띄운다.
> 이 브랜치의 브리지 설정(`pinkyN_control.yaml`)에는 필요한 다운링크(`/pinkyN/overhead_pose` · `/pinkyN/lane_path`)가 이미 있다.

## 흐름

1. **태블릿**이 로봇 위 마커로 map5 좌표 x · y · yaw 를 구해 중계 `POST /api/vision/pose_fix` 로 보낸다.
2. **중계 게이트웨이**가 키·형식을 검사한 뒤 같은 좌표를 `/pinkyN/overhead_pose`(`geometry_msgs/PoseStamped`, frame `map`)로 낸다(도메인 8).
   기존 `/pinkyN/pose_fix`(PoseFix)도 그대로 낸다.
3. **도메인 브리지**가 로봇 도메인(pinky1 = 10, pinky2 = 11)으로 내린다.
4. **로봇 `pose_fuser_node`**(`lane_robot.launch.xml`, `use_overhead:=True` 기본)가 odom 과 합쳐 TF map→odom 을 만든다.
5. **로봇 `lane_agent_node`**가 그 위치로 경로를 따라가고 갈림길·도착을 판정한다.

경로(`/pinkyN/route`)는 출발 전에 한 번, 통과 허가(`/pinkyN/lane_command` 의 `clear_until_idx`)는 10 Hz 로 코디네이터가 따로 보낸다.
천장 카메라 추적기(`overhead_tracker_node`, [`overhead_tracker.md`](overhead_tracker.md))와 **같은 토픽**이다 — 로봇은 출처를 가리지 않는다.

## 보내는 방법

```http
POST http://<중계 PC>:8889/api/vision/pose_fix
Content-Type: application/json
X-API-Key: <중계 운영자에게 받은 키>

{"robot_name": "pinky1",
 "header": {"frame_id": "map", "stamp": {"sec": 1790688000, "nanosec": 0}},
 "x": 1.18, "y": 0.64, "yaw": 1.57,
 "seq": 12, "marker_id": 1}
```

| 필드 | 필수 | 형식 | 뜻 |
| :-- | :-- | :-- | :-- |
| `robot_name` | 예 | `pinky1` · `pinky2` | 어느 로봇의 좌표인지. 로봇마다 요청 하나 |
| `header.frame_id` | 예 | `map` 고정 | 다른 값이면 400 |
| `header.stamp.sec` · `nanosec` | 예 | 0 이상의 정수 | 캡처 시각. 형식만 검사하고 로봇은 쓰지 않는다 |
| `x` · `y` | 예 | 실수, m | map5 좌표. 원점은 경기장 좌하단 |
| `yaw` | 예 | 실수, rad | 로봇 전방이 +x 에서 반시계로 이루는 각 |
| `seq` | 아니오 | 0 이상의 정수 | 보낼 때마다 1씩 — 로그 추적용 |
| `marker_id` | 아니오 | 정수 | 좌표를 만든 로봇 마커 번호 |
| `reproj_error` · `n_markers` · `pipeline_latency` | 아니오 | 0 이상 | 진단용. 로봇 위치에는 쓰지 않는다 |

키는 `Authorization: Bearer <키>` 로 보내도 된다. 키 없이 띄운 중계는 이 입구를 전부 401 로 닫는다.

## 규칙

| 규칙 | 값 | 어기면 |
| :-- | :-- | :-- |
| 좌표계 | map5 — 원점 좌하단, x 0..2.36 m · y 0..1.28 m | 경로에서 벗어난 것으로 보고 엉뚱하게 조향 |
| yaw | 로봇 전방 기준, rad, 반시계 + | 조향이 반대로 간다. 마커를 다르게 붙였으면 로봇 `pose_fuser` 의 `yaw_offset` · `offset_x` |
| 주기 | 멈춰 있을 때도 계속. 초당 5 회 이상 권장 | 3 s 끊기면(`fix_timeout`) 로봇이 "위치(TF map→base) 없음 — 정지" |
| 한 로봇 한 출처 | 태블릿과 천장 추적기를 같은 로봇에 동시에 쓰지 않는다 | 두 값이 섞여 위치가 튀거나 멈춘다 |

**마커 위에 섰을 때만 보내는 방식은 안 된다** — 마커 사이 구간에서도 위치가 있어야 경로를 따라간다.
0.30 m 또는 45° 넘게 튄 값은 같은 후보가 2 번 이어져야 채택된다(`gate_dist` · `gate_yaw_deg` · `confirm`).
태블릿 카메라는 어두우면 느려진다(예전 실측: 밝을 때 약 8 fps, 어두울 때 약 1.7 fps) — 주기를 못 맞추면 조명부터 본다.

## 로봇이 받은 좌표로 하는 일

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

## 확인 방법

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
| 로봇이 "위치 없음 — 정지" | 좌표가 3 s 넘게 끊김 | 보내는 주기와 Wi-Fi |
| `fix_status` 의 rejected 만 는다 | 좌표가 튀거나 다른 출처와 섞임 | 천장 추적기가 같은 로봇을 보는지 |
| 로봇이 엉뚱한 방향으로 튼다 | yaw 기준·부호가 다름 | 로봇을 +x 쪽으로 놓고 yaw 가 0 근처인지 |

## 중계 PC (운영자)

```bash
export RELAY_VISION_API_KEY=<공유 키>              # 없으면 좌표 입구가 전부 401
# export RELAY_POSE_FIX_TO_OVERHEAD=0              # 같은 로봇을 천장 추적기도 볼 때만
bash relay_station/launch_master_gateway.sh

# 로봇 (pinky1 예) — pose_fuser 기본 켜짐, AMCL 기본 꺼짐
ros2 launch pinky_fleet_agent lane_robot.launch.xml robot_name:=pinky1 domain_id:=10 map:=$HOME/map/map5.yaml
```

키는 저장소에 적지 않고 따로 전한다.

## 남은 일

- [ ] 바닥 마커 위치를 map5 로 재서 `road_graph.yaml` 의 임시 좌표(BL · BR · TR · J)를 바꾼다.
- [ ] 로봇을 바닥 마커 위에 놓고 태블릿 좌표가 노드 좌표와 5 cm 안으로 맞는지, yaw 도 맞는지 본다.
- [ ] 열린 질문: 태블릿 한 대가 두 로봇을 한 화면에서 볼 수 있는지, 두 대가 필요한지.

실물 로봇으로는 아직 확인하지 않았다. 중계 쪽 검증(ROS 2 Jazzy 컨테이너): 실제 게이트웨이에 POST → 키 없음 401 · 키 있음 200 →
`/pinky1/overhead_pose` 수신(map, 1.18, 0.64, yaw 1.2) · `RELAY_POSE_FIX_TO_OVERHEAD=0` 이면 `overhead_topic: null`.
