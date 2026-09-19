# 관제 PC -> 로봇 1대. RELIABLE depth 10.
# CMD_CLEARANCE 는 10 Hz 로 계속 온다 — 하트비트를 겸한다. 관제가 죽으면 로봇의
# link_watch 가 침묵을 감지해 멈춘다.

uint8 CMD_HEARTBEAT=0
uint8 CMD_START=1                   # 주행 시작 (Route 를 받은 뒤)
uint8 CMD_STOP=2                    # 정지 유지 (RESUME 으로 해제)
uint8 CMD_ESTOP=3                   # 비상 정지 래치 (RESUME 으로 해제)
uint8 CMD_RESUME=4
uint8 CMD_SET_SPEED=5
uint8 CMD_CLEARANCE=6               # clear_until_idx 갱신 (10 Hz, 하트비트 겸용)
uint8   command

uint32  route_seq                   # 어느 경로에 대한 clearance 인지
int32   clear_until_idx             # 이 waypoint 까지 가도 된다. 0 이면 출발점 대기
float32 max_linear_vel              # CMD_SET_SPEED
float32 max_angular_vel
