# 핑키 1대의 상태 스냅샷. 로봇 도메인의 agent 노드가 10Hz 로 발행하고,
# domain_bridge 를 통해 관제 도메인으로 넘어온다.

std_msgs/Header header          # frame_id = "map"

string  name                    # "pinky1"
uint16  domain_id               # 11  (작을수록 우선순위 높음 = 먼저 출발)

# --- 현재 위치 (map frame, AMCL 기준) ---
bool    localized               # amcl_pose 를 최근에 받았는가
float64 x
float64 y
float64 yaw

# --- 현재 속도 (odom 기준) ---
float64 linear_velocity
float64 angular_velocity

# --- 주행 상태 ---
uint8 NAV_IDLE=0
uint8 NAV_ACTIVE=1
uint8 NAV_SUCCEEDED=2
uint8 NAV_ABORTED=3
uint8 NAV_CANCELED=4
uint8 NAV_HOLD=5                # coordinator 가 양보시켜 대기 중
uint8 NAV_LINK_LOST=6           # 관제 PC 신호가 끊겨 로봇이 스스로 주행을 중단함

uint8   nav_status

# --- 현재 목표 ---
bool    goal_valid
float64 goal_x
float64 goal_y
float64 goal_yaw

# --- 현재 적용된 속도 제한 ---
float64 max_linear_vel
float64 max_angular_vel

# --- 이 로봇의 Nav2 가 실제로 로드한 맵의 규격 (map 토픽 OccupancyGrid.info) ---
# 관제 PC 가 자기 맵과 비교해 서로 다른 맵을 쓰고 있는지 잡아내는 용도.
# 파일 경로가 달라도 내용이 같으면 통과하고, 경로가 같아도 내용이 다르면 잡힌다.
string  map_name                # 현재 로드한 맵 이름 (확장자 없음)
bool    map_known               # map 토픽을 받았는가 (Nav2 기동 전에는 false)
float32 map_resolution          # m/px
uint32  map_width               # px
uint32  map_height              # px
float64 map_origin_x
float64 map_origin_y

float32 battery_percent
