# 출처 — 팀11 map4 (바이트 그대로)

| 파일 | 원본 (github.com/sakim0128/pinky_pro_team11, 브랜치 mini_project_2, 커밋 1f505cb) | sha256 |
| :-- | :-- | :-- |
| `road_graph.yaml` | `pinky_lane_station/config/road_graph.yaml` | `547dae27762c635d34547d14c40665f62bea28d7d3b2bf3d87d66abc73754d07` |
| `map4.yaml` | `pinky_fleet_station/config/map4.yaml` | `2d9fada5469544355f71779ae7620ab053d2bed1942e374fdbd28eb035db4774` |
| `map4.pgm` | `pinky_fleet_station/config/map4.pgm` | `5a3794d2ef6a9936aa6b1fb112142e28e975434bd07a59ca05d143c6749bf1a7` |

- 세 파일은 고치지 않는다(시험이 sha256 을 대조한다). 바꿔야 하면 팀11 원본을 바꾸고 다시 가져온다.
- `lane_mission.yaml`·`lane_mission_s2.yaml` 은 우리 파일이다: 팀11 `pinky_lane_station/config/lane_mission.yaml` 의
  시나리오 1·2 로봇 배정 + 우리 `drive_mode: nav2`.
- 로봇의 Nav2 지도(`robot_onboard/pinky_navigation/map/map4.*`)는 관제 C-M1 이 같은 원본으로 넣는다 — 웹의
  "로봇 지도 전환" 은 로봇이 그 파일을 갖고 있어야 된다(로봇이 보고한 지도 원점·크기로 대조해 보여 준다).
