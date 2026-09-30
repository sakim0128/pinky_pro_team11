# 관제 PC -> 로봇 1대. 절대 위치 (map frame). RELIABLE depth 5. 두 소스가 같은 토픽에 발행한다.
#   항공뷰(overhead_localizer_node, 주): 천장 웹캠이 핑키 위 마커를 본다. stamp = 캡처 시각 = **관제 시계**
#   바닥 마커(lane_pipeline_node, 보조): 핑키 카메라 이미지의 stamp 를 **복사** = 로봇 시계
# 로봇의 pose_fuser 는 stamp_is_robot_clock 이면 그 시각의 odom 을 찾고, 아니면 (수신 시각 − station_latency) 의
# odom 에 붙인다. 두 기기 시계는 만나지 않는다.

std_msgs/Header header              # frame_id = map
bool    stamp_is_robot_clock        # true: header.stamp 가 로봇 시계 (바닥 마커). false: 관제 시계 (항공뷰)
string  robot_name
uint32  seq

float64 x
float64 y
float64 yaw

int32   marker_id                   # 위치를 만든 마커 (바닥: 가장 가까운 것, 항공뷰: 로봇 위 마커 id)
float32 marker_range                # 바닥: 카메라-마커 거리 (m). 항공뷰: 0
float32 reproj_error                # px. 바닥: PnP 재투영. 항공뷰: 기준 마커 호모그래피 재투영. 클수록 못 믿는다
int32   n_markers                   # 바닥: 본 알려진 마커 수. 항공뷰: 본 기준 마커 수
float32 pipeline_latency            # 관제 내부 수신->발행 (s)
