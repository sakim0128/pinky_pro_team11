# 관제 PC -> 핑키 1대. 명령을 한 메시지로 통합해 브리지할 토픽 수를 최소화한다.

uint8 CMD_GOTO=0                # x, y, yaw 로 이동 시작
uint8 CMD_STOP=1                # goal 취소 + HOLD (last_goal 보존, RESUME 대기)
uint8 CMD_RESUME=2              # HOLD 해제 + last_goal 재전송
uint8 CMD_CANCEL=3              # goal 취소 + 폐기
uint8 CMD_SET_INITIAL_POSE=4    # initialpose 발행 (AMCL 재초기화)
uint8 CMD_SET_SPEED=5           # 속도 제한 변경

uint8   command

# CMD_GOTO / CMD_SET_INITIAL_POSE 에서 사용
float64 x
float64 y
float64 yaw

# CMD_SET_SPEED 에서 사용
float64 max_linear_vel
float64 max_angular_vel
