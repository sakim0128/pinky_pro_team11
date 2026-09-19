# 로봇 -> 관제 PC. 10 Hz, RELIABLE depth 10.

std_msgs/Header header
string  robot_name

uint8 DRIVE_IDLE=0
uint8 DRIVE_CRUISE=1
uint8 DRIVE_WAIT_CLEARANCE=2        # 예약 안 된 구간 앞에서 대기
uint8 DRIVE_CROSSWALK_STOP=3        # 3 초 정지 중
uint8 DRIVE_CROSSWALK_CLEAR=4       # 통과 중 (재래치 금지 구간)
uint8 DRIVE_OBSTACLE_WAIT=5         # 장애물. 치워지면 자동 복귀
uint8 DRIVE_LANE_LOST=6             # 카메라가 차선을 못 봄. 맵 경로만으로 서행
uint8 DRIVE_ARRIVED=7
uint8 DRIVE_ESTOP=8                 # 수동 래치. RESUME 으로만 해제
uint8 DRIVE_LINK_LOST=9             # 관제 침묵 → 정지
uint8   drive_state
string  state_reason                # 사람이 읽는 이유. "초음파 0.14m", "JS 앞 대기 (pinky1)"

uint32  route_seq
int32   route_idx                   # 경로 위 진행 인덱스
string  edge_id
int32   clear_until_idx
float32 error_x_norm                # 중앙 정렬 평가용
uint8   lane_quality                # LanePath.QUALITY_*
float32 linear_velocity
float32 angular_velocity
float32 path_age                    # now - LanePath.source_stamp (s, 로봇 시계) = 실측 왕복 지연
float32 lidar_min_range             # 전방 섹터 최소 거리
float32 us_range                    # 초음파 (필터 후)
float32 odom_since_state            # 현재 상태 진입 후 주행거리 (m)
