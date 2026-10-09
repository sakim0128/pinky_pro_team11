# tablet/vision — Y700 External Vision Compute

영상 → ArUco(DICT_4X4_50) → Homography → x/y/yaw → PoseFix / ZoneEvent → Relay HTTP.

- **마커 ID SoT = `config/markers.yaml`** (로봇 30=pinky1, 31=pinky2 · 기준 40 tl / 41 tr / 42 br / 43 bl).
  값이 바뀌면 `vision_data/aruco/` 인쇄물도 같은 커밋에서 바꾼다.
- 설정 3종(`markers`·`cameras`·`zones`)이 `READY` 가 아니면 송신 차단(`--allow-pending-config` 로만 해제).
  좌표는 전부 PLACEHOLDER — 줄자 실측 전.
- mock: `--mock` 은 road_graph 노드(BL/BR — `pinky_lane_station/config/road_graph.yaml`)에 두 로봇을 렌더한다. `--mock-motion <m/s>` 이면
  pinky1 이 `--mock-path`(기본 `BL,TR`)를 따라 전진한다. 송신은 `--mock-egress` 를 줘야만.
- health: 5 s 마다 `VISION_HEALTH {json}` 한 줄 — `input_fps, frames_total, markers_seen, reference_ok,
  homography_ok, posefix_rate, last_post{status,age_s}, config_status, mode, git_sha` — 지금은 **로그 한 줄뿐**이다. 중계에 이 값을 받는 라우트는 아직 없다(`/api/ops/vision` 은 계획상 이름, 업링크 경로 미정 · R-5 후속).
- 시험: `python3 -m pytest tablet/vision/tests -q`
