# 관제 PC -> 핑키 1대. 명령을 한 메시지로 통합해 브리지할 토픽 수를 최소화한다.

uint8 CMD_GOTO=0                # x, y, yaw 로 이동 시작
uint8 CMD_STOP=1                # goal 취소 + HOLD (last_goal 보존, RESUME 대기)
uint8 CMD_RESUME=2              # HOLD 해제 + last_goal 재전송
uint8 CMD_CANCEL=3              # goal 취소 + 폐기
uint8 CMD_SET_INITIAL_POSE=4    # initialpose 발행 (AMCL 재초기화)
uint8 CMD_SET_SPEED=5           # 속도 제한 변경
uint8 CMD_SET_MAP=6             # map_name 으로 로봇의 맵 교체
uint8 CMD_HEARTBEAT=7           # 관제 PC 생존 신호. 로봇은 수신 시각만 갱신한다.

uint8   command

# CMD_GOTO / CMD_SET_INITIAL_POSE 에서 사용
float64 x
float64 y
float64 yaw

# CMD_SET_SPEED 에서 사용
float64 max_linear_vel
float64 max_angular_vel

# CMD_SET_MAP 에서 사용. 경로가 아니라 확장자 없는 "이름" 이다 ("pinklab").
# 관제 PC 와 로봇은 맵 파일 경로가 서로 다르므로 이름만 주고받고,
# 로봇이 자기 맵 디렉터리에서 <map_dir>/<map_name>.yaml 을 찾는다.
string  map_name
