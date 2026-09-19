# 관제 PC -> 로봇 1대. 카메라로 본 차선 중앙 (0.3 초 주기). BEST_EFFORT depth 1.
#
# header.stamp 는 관제 PC 발행 시각 — 진단 전용, 제어에 쓰지 않는다.
# source_stamp 는 원본 이미지 stamp (로봇 시계). 관제 PC 는 이 값을 절대 다시 찍지
# 않고 복사만 한다. 로봇은 now - source_stamp 로 지연을 잰다 (같은 시계끼리).

std_msgs/Header header
builtin_interfaces/Time source_stamp
string  robot_name
uint32  seq

uint8 QUALITY_BOTH=0                # 양쪽 차선 관측
uint8 QUALITY_SINGLE=1              # 한쪽만 → half_lane_px 로 추정
uint8 QUALITY_JUNCTION=2            # 분기 반경 안 — 카메라 보정 끄고 맵 경로만
uint8 QUALITY_STALE=3               # 새 추론 없이 직전 값 재발행
uint8 QUALITY_LOST=4                # 차선 없음. error_x_norm 무효
uint8   quality

float32 error_x_norm                # (target_x - W/2) / (W/2), -1..+1. 차선 중앙이 화면 오른쪽이면(로봇이 왼쪽 치우침) 양수
int32   image_width
int32   image_height
int32   target_x
int32   target_y
int32   left_x
int32   right_x
bool    left_seen
bool    right_seen
float32 half_lane_px                # 관측 이력으로 추정한 차선 반폭 (px)
float32 confidence

bool    crosswalk_detected          # 디바운스 통과한 횡단보도 트리거
int32   crosswalk_bottom_y
float32 pipeline_latency            # 관제 PC 내부 수신->발행 (s). 관제 시계끼리라 유효
