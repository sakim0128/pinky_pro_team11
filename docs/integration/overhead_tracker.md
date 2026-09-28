# 상부 카메라 ArUco 추적 연결

`overhead_tracker_node`는 상부 카메라 JPEG에서 ArUco 태그를 찾고, 이미지 픽셀을 map5의
`map` 좌표로 변환해 다음 토픽을 발행한다.

| ArUco ID | 출력 |
|---|---|
| 1 | `/pinky1/overhead_pose` |
| 2 | `/pinky2/overhead_pose` |

출력 타입은 `geometry_msgs/PoseStamped`다. 태그의 상단 변을 heading으로 사용해 yaw를 산출한다.

## 보정

상부 카메라 화면에 map5의 네 기준점을 고정하고, 각 기준점에 대해 이미지 `(u, v)`와 map5
`(x, y)`를 측정한다. 이 네 쌍으로 image-pixel → map-metre homography를 구한다. 계산한 9개
row-major 값을 `config/overhead_tracker.yaml`의 `image_to_map_homography`에 넣는다.

빈 배열은 의도적으로 pose 발행을 막는다. 보정 전에는 태그가 보여도 로봇 위치를 만들지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pinky_fleet_station overhead_tracker.launch.xml
```

상부 영상 토픽, 태그 ID, map frame은 같은 YAML에서 바꿀 수 있다. OpenCV ArUco 기능이 포함된
`python3-opencv`가 관제 PC에 설치돼 있어야 한다.
