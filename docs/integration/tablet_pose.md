# 태블릿 좌표 → 로봇 위치 (rkd1rjs2 팀원, 2026-09-29)

태블릿은 로봇 위 마커를 보고 **로봇 좌표**를 map5 로 계산해 중계에 HTTP 로 **계속** 보낸다. 마커 도착 판정과 갈림길 방향은
로봇(`lane_agent_node`)과 중계 코디네이터가 한다. 태블릿이 바닥 마커 번호나 "다음 마커" 를 알릴 필요는 없다.

> **소스 위치.**
> - **태블릿 비전 소스**: 저장소 루트의 `tablet/vision/` 에 이미 완전 구현되어 있다. (송신기를 새로 짤 필요 없음)
> - **중계 게이트웨이**: `rkd1rjs2/robot_mini_project_pinky` (`e59d521`, `relay_station/gateway_web/`) 에서 관리하며,
>   태블릿이 보내는 `POST /api/vision/pose_fix` 를 받아 `/pinkyN/overhead_pose` 로 팀 로봇 토픽에 바로 중계한다.
> - **도메인 브리지**: 이 브랜치의 브리지 설정(`pinkyN_control.yaml`)에 필요한 다운링크(`/pinkyN/overhead_pose` · `/pinkyN/lane_path`)가 이미 있다.

---

## 1. 태블릿 비전 동작 원리 (`tablet/vision/`)

- **자체 호모그래피 계산:** 팀 상부 추적기(`overhead_tracker_node`) 알고리즘을 모바일(Lenovo Y700) 환경으로 이식하여, 경기장 네 모서리 기준 마커(`40 tl`, `41 tr`, `42 br`, `43 bl`, DICT_4X4_50)를 매 프레임 탐지해 픽셀 → map5 좌표 변환 호모그래피 행렬($H$)을 실시간으로 갱신·스무딩(`alpha=0.3`)한다.
- **로봇 절대 좌표 산출:** 로봇 상단에 부착된 마커(`30=pinky1`, `31=pinky2`)의 4꼭짓점을 이 $H$ 로 변환하여 로봇의 실제 $x, y, yaw$ 를 도출한다.
- **모서리 가림 대응 & 안전 차단:** 4개 기준 마커가 일시적으로 다 안 보여도 직전 유효 $H$ 를 유지한다. 재투영 오차가 기준치(`max_reproj: 4.0 px`)를 넘거나 $H$ 가 없으면 좌표를 보내지 않으며, 3초 동안 좌표 수신이 끊기면 로봇은 안전하게 정지한다.

---

## 2. 현장 실데이터 검증 및 실측 캘리브레이션 결과 (2026-09-30)

실제 촬영된 영상 프레임 전수를 분석하여 4대 실측 항목을 검증 완료하였습니다:

| 항목 | 실측 전 가정 | 🔬 현장 실제 프레임 실측 결과 | 반영 조치 |
|---|---|---|---|
| **모서리 마커 중심** | 0~2.34 × 0~1.26 m | 40, 41, 42, 43 전수 검출. 비율 약 1.98 로 map5 경기장과 일치 | `markers.yaml` 에 반영 완료 |
| **🚨 마커 부착 각도 (Yaw)** | 모두 `0.0 rad` 일괄 가정 | 실측 결과 부착 회전각이 다름:<br>• **41번**: 0.6° (~0 rad)<br>• **40번**: 91.3° (~1.57 rad, 90° 회전)<br>• **42번**: 178.4° (~3.14 rad, 180° 회전)<br>• **43번**: -179.0° (~-3.14 rad, 180° 회전) | **기존 각도로 돌리면 재투영 오차가 20px 넘어 차단됨.**<br>실측 각도 적용 시 **재투영 오차 0.998 px (서브픽셀급 정합)** 달성 |
| **마커 높이 및 크기** | 가로세로 0.10 m, 받침 0.10 m | 호모그래피 역변환 시 정확히 0.100 × 0.102 m 로 복원됨 | 10 cm 동일 평면 유지 확인 |
| **로봇 위 마커 번호** | 30(pinky1), 31(pinky2) vs 1, 2 | 실제 로봇 촬영 영상에서 **`30`번 마커 정상 인식 확인** | 30(pinky1), 31(pinky2) 유지 |

---

## 3. 태블릿 비전 실행 방법

Y700 태블릿(Termux/PRoot 환경)에서 아래 명령으로 비전 연산 및 송신 서비스를 기동합니다:

```bash
# 태블릿 비전 서비스 실행 (중계 PC IP와 포트 지정)
python3 -m tablet.vision.runtime.entrypoint --relay-url http://192.168.0.3:8889

# 만약 설정 상태가 FIELD_CONFIG_PENDING 인 경우 송신 강제 허용 플래그:
python3 -m tablet.vision.runtime.entrypoint --relay-url http://192.168.0.3:8889 --allow-pending-config
```

---

## 4. 흐름

1. **태블릿**이 로봇 위 마커로 map5 좌표 x · y · yaw 를 구해 중계 `POST /api/vision/pose_fix` 로 보낸다.
2. **중계 게이트웨이**가 키·형식을 검사한 뒤 같은 좌표를 `/pinkyN/overhead_pose`(`geometry_msgs/PoseStamped`, frame `map`)로 낸다(도메인 8).
   기존 `/pinkyN/pose_fix`(PoseFix)도 그대로 낸다.
3. **도메인 브리지**가 로봇 도메인(pinky1 = 10, pinky2 = 11)으로 내린다.
4. **로봇 `pose_fuser_node`**(`lane_robot.launch.xml`, `use_overhead:=True` 기본)가 odom 과 합쳐 TF map→odom 을 만든다.
5. **로봇 `lane_agent_node`**가 그 위치로 경로를 따라가고 갈림길·도착을 판정한다.

경로(`/pinkyN/route`)는 출발 전에 한 번, 통과 허가(`/pinkyN/lane_command` 의 `clear_until_idx`)는 10 Hz 로 코디네이터가 따로 보낸다.
천장 카메라 추적기(`overhead_tracker_node`, [`overhead_tracker.md`](overhead_tracker.md))와 **같은 토픽**이다 — 로봇은 출처를 가리지 않는다.

---

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

---

## 6. 규칙

| 규칙 | 값 | 어기면 |
| :-- | :-- | :-- |
| 좌표계 | map5 — 원점 좌하단, x 0..2.36 m · y 0..1.28 m | 경로에서 벗어난 것으로 보고 엉뚱하게 조향 |
| yaw | 로봇 전방 기준, rad, 반시계 + | 조향이 반대로 간다. 마커를 다르게 붙였으면 로봇 `pose_fuser` 의 `yaw_offset` · `offset_x` |
| 주기 | 멈춰 있을 때도 계속. 초당 5 회 이상 권장 | 3 s 끊기면(`fix_timeout`) 로봇이 "위치(TF map→base) 없음 — 정지" |
| 한 로봇 한 출처 | 태블릿과 천장 추적기를 같은 로봇에 동시에 쓰지 않는다 | 두 값이 섞여 위치가 튀거나 멈춘다 |

---

## 7. 남은 일 및 현황

- [x] 태블릿 비전 소스(`tablet/vision/`) 검증 및 연동 규격 대조 완료.
- [x] 실제 녹화 데이터 기반 4대 실측값 및 마커 회전각(Yaw) 보정 (재투영 오차 0.998 px).
- [ ] 현장 경기장 도로망 노드(`pinky_lane_station/config/road_graph.yaml` 의 BL · BR · TR · J)와 태블릿 좌표 오차 5 cm 이내 대조.
- [ ] 실물 로봇 주행 중 `pose_fuser` 오차 및 조향 추종 E2E 확인.
