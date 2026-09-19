# 관제 PC 내부 (GUI 진단용). 로봇으로 가지 않는다 — 로봇은 LanePath 만 본다.

std_msgs/Header header
builtin_interfaces/Time source_stamp
string  robot_name

bool    crosswalk_raw               # 이번 프레임 원본 판정
bool    crosswalk_detected          # 디바운스 통과
int32   crosswalk_bottom_y
float32 crosswalk_confidence
int32   lane_count                  # 이번 프레임의 lane 인스턴스 수

string  detector_name               # "lane=yolo11n-seg@lane_v3.pt" — 재현성
float32 infer_ms
float32 fps
