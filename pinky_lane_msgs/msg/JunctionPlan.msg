# 관제 PC -> 로봇 1대 (TRANSIENT_LOCAL, depth 1). 비전 미션 모드(위치추정 없이 카메라 차선 주행)의 교차로 계획.
#
# 로봇은 차선을 따라가다 빨간 테이프(YOLO red_line)에서 JUNCTION_STOP 으로 서고, 관제 허가(LaneCommand CLEARANCE
# clear_until_idx >= 1, route_seq == seq) 를 받으면 아래 고정 동작을 odom 으로 수행한 뒤 다시 차선 주행으로 돌아간다.
# 교차로를 지난 뒤 흰 정지선을 stop_line_count 번째 만나면 그 앞에서 도착 정지한다.
# 같은 seq 로 다시 오면 stop_line_count·동작만 갱신한다(진행 상태 유지). seq 가 바뀌면 새 미션이다.

std_msgs/Header header
string  robot_name
uint32  seq                         # LaneCommand.route_seq 와 같은 번호
string  scenario                    # "s1" 등 (표시용)
string  maneuver                    # 동작 이름 (표시용) "right_3_to_1"

string[]  step_kind                 # "straight" | "turn"
float32[] step_value                # straight: m (+ 전진), turn: deg (+ 좌회전, CCW)
float32   linear_speed              # 동작 직진 속도 (m/s). 0 이면 로봇 기본값
float32   angular_speed             # 동작 회전 속도 (rad/s). 0 이면 로봇 기본값

uint8   stop_line_count             # 교차로 통과 뒤 이 번째 흰 정지선 앞에서 도착. 0 이면 도착 판정 안 함(아직 모름)
