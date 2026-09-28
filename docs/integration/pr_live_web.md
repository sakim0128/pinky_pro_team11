# PR 준비: live fleet web dashboard

## 대상

- Base: `sakim0128/pinky_pro_team11:mini_project_2`
- Head: `0gpublike/pinky_pro_team10:feat/live-web-state`

## 제목

`feat: add live fleet web dashboard, map and camera telemetry`

## 설명

ROS 상태를 브라우저에서 조회하고 필요한 경우에만 명시적으로 제어하는 Fleet 대시보드를 추가합니다.
map5 점유 지도와 Figma 차선 도면을 합성하고, RobotState map 규격이 일치할 때만 AMCL·목표·공분산을
오버레이합니다. 전방·상부 JPEG 카메라와 LaneStatus를 표시하며, ArUco 상부 추적 노드는 보정된 경우에만
`/pinky*/overhead_pose`를 발행합니다.

제어는 기본으로 비활성입니다. `enable_control:=true`에서만 lane coordinator에 start/stop/resume을
전달하고, 초기화는 사용자가 지정한 map5 초기 위치로 AMCL initialpose를 설정합니다.

## 검증

- `python3 -m pytest pinky_fleet_station/test -q`: 90 passed
- `pinky_fleet_msgs`, `pinky_lane_msgs`, `pinky_fleet_station` colcon build 성공
- localhost 웹 화면에서 map5와 Figma 합성 지도 렌더링 확인

## 실기 확인 필요

- map5 규격 일치와 AMCL/공분산 오버레이 위치
- 전방·상부 JPEG bridge 수신
- ArUco 태그 ID와 image-to-map homography 보정
- control enabled 상태에서 start/stop/resume/reset/speed 동작
