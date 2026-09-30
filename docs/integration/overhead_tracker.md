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

## 웹캠 발행 (overhead_camera_node)

같은 launch 가 기본으로 `overhead_camera_node` 를 함께 띄운다 (`use_camera:=True`). `cv2.VideoCapture` 로
천장 웹캠을 읽어 `/overhead/camera/image/compressed` 에 JPEG (`sensor_msgs/CompressedImage`, `frame_id=overhead_camera`,
stamp = 캡처 시각·관제 시계) 를 낸다. QoS 는 추적기·live 웹이 기본 프로파일로 구독하므로 RELIABLE · VOLATILE · depth 1.

| launch 인자 | 기본 | 뜻 |
|---|---|---|
| `use_camera` | `True` | False 면 영상은 다른 곳에서 와야 한다 (relay 영상 공유 등) |
| `camera_device` | `0` | `/dev/video<N>` 정수 또는 장치 경로 |
| `camera_width` · `camera_height` | `1280` · `720` | 캡처 해상도 (장치가 못 주면 경고만 내고 그대로 간다) |
| `camera_fps` | `15.0` | 발행 주기 |
| `camera_jpeg_quality` | `80` | JPEG 품질 |
| `image_topic` | `/overhead/camera/image/compressed` | 발행 토픽. YAML 의 `image_topic` 과 같아야 한다 (`test_overhead_camera.py` 가 검사) |

장치 읽기가 실패하면 경고를 내고 2 s 뒤 다시 연다 (죽지 않음). 노드 자체가 죽으면 launch 가 3 s 뒤 다시 띄운다.
