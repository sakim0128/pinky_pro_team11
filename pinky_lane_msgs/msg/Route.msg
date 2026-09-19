# 관제 PC -> 로봇 1대. 시작 노드에서 목적지 노드까지의 경로 (map frame).
# road_graph.Route 와 필드가 1:1. 새 경로를 줄 때마다 route_seq 를 올린다 —
# 로봇은 자기 route_seq 와 다른 LaneCommand.clear_until_idx 를 무시한다.

std_msgs/Header header              # frame_id = map
string  robot_name
uint32  route_seq

geometry_msgs/Point[] waypoints     # 0.10 m 등간격, z = 0
string[] edge_ids                   # 통과 엣지 순서
bool[]   edge_forward               # 각 엣지를 from->to 방향으로 지나는지
int32[]  edge_end_idx               # 각 엣지가 끝나는 waypoint 인덱스
string[] node_ids                   # 시작 노드 포함 통과 노드 (len = len(edge_ids)+1)
int32[]  node_idx                   # 각 노드의 waypoint 인덱스
int32[]  crosswalk_idx              # 횡단보도 노드의 waypoint 인덱스
int32[]  junction_idx               # 분기 노드의 waypoint 인덱스
int32    goal_idx
float32  length                     # m
