# 관제 PC -> 로봇 1대. 바닥 ArUco 마커로 얻은 절대 위치 (map frame). RELIABLE depth 5.
#
# header.stamp 는 원본 이미지 stamp 를 **복사** 한 것 (로봇 시계). 관제는 다시 찍지 않는다.
# 로봇의 pose_fuser 는 이 시각의 odom 을 찾아 map->odom 을 갱신한다 — 처리 지연이 오차가 되지 않는다.

std_msgs/Header header              # stamp = 이미지 source_stamp, frame_id = map
string  robot_name
uint32  seq

float64 x
float64 y
float64 yaw

int32   marker_id                   # 위치를 만든 마커 (가장 가까운 것)
float32 marker_range                # 카메라-마커 거리 (m)
float32 reproj_error                # px. 클수록 못 믿는다
int32   n_markers                   # 이 프레임에서 본 알려진 마커 수
float32 pipeline_latency            # 관제 내부 수신->발행 (s)
