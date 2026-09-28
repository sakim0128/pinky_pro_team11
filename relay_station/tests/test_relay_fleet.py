# -*- coding: utf-8 -*-
"""Relay Fleet Control Station Verification Tests (R-A1 ~ R-A21).

Tests compliance with Track R specifications and inspection defect hotfixes:
- P0-1: Robot 2 odom parameterization (R-A13)
- P0-2: Shared msgs build & deploy paths
- P0-3: Robot 3 & 4 scope isolation (R-A14)
- P0-4 & P2-2: Arena coordinates & lane_mission.yaml single source of truth (R-A15)
- P1-1: route_seq mismatch detection (R-A16)
- P1-2: Goal zone event freshness & zone_id verification (R-A17)
- P1-3: Gateway HTTP thread decoupling (R-A18)
- P1-4: START retry safety logic (R-A19)
- P1-5: DriveCommandGate E-STOP cache invalidation (R-A20)
- P2-1: MISSION_DONE state transition (R-A21)
"""

import json
import math
import os
import sys
from unittest.mock import MagicMock

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from relay_station.fleet.road_graph import RoadGraph, Route
from relay_station.fleet.reservation import Reservation
from relay_station.fleet.route_comparator import RouteComparator, RouteComparisonResult
from relay_station.fleet.fleet_coordinator import RelayFleetCoordinator, FleetRobotContext
from pinky_fleet_agent.drive_command_gate import CommandGateCore, GateSource

from pinky_fleet_msgs.msg import RobotState
from pinky_lane_msgs.msg import LaneStatus, LaneCommand
from std_msgs.msg import String

CONFIGS_DIR = os.path.join(REPO_ROOT, "relay_station", "domain_bridge", "configs")


def _load_yaml(filename):
    path = os.path.join(CONFIGS_DIR, filename)
    assert os.path.exists(path), f"File {filename} does not exist in {CONFIGS_DIR}"
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# =============================================================================
# R-A1: PoseFix R1 → D10 only
# =============================================================================
def test_R_A1_pose_fix_robot1_routes_to_domain_10_only():
    doc = _load_yaml("robot1_control.yaml")
    assert doc["from_domain"] == 10
    assert doc["to_domain"] == 8

    topics = doc.get("topics", {})
    assert "/pinky1/pose_fix" in topics, "/pinky1/pose_fix not found in robot1_control.yaml"
    spec = topics["/pinky1/pose_fix"]
    assert spec["type"] == "pinky_lane_msgs/msg/PoseFix"
    assert spec["from_domain"] == 8
    assert spec["to_domain"] == 10

    for other in ("robot2_control.yaml", "team_mirror.yaml"):
        other_doc = _load_yaml(other)
        assert "/pinky1/pose_fix" not in other_doc.get("topics", {})


# =============================================================================
# R-A2: PoseFix R2 → D11 only
# =============================================================================
def test_R_A2_pose_fix_robot2_routes_to_domain_11_only():
    doc = _load_yaml("robot2_control.yaml")
    assert doc["from_domain"] == 11
    assert doc["to_domain"] == 8

    topics = doc.get("topics", {})
    assert "/pinky2/pose_fix" in topics, "/pinky2/pose_fix not found in robot2_control.yaml"
    spec = topics["/pinky2/pose_fix"]
    assert spec["type"] == "pinky_lane_msgs/msg/PoseFix"
    assert spec["from_domain"] == 8
    assert spec["to_domain"] == 11

    for other in ("robot1_control.yaml", "team_mirror.yaml"):
        other_doc = _load_yaml(other)
        assert "/pinky2/pose_fix" not in other_doc.get("topics", {})


# =============================================================================
# R-A3: stale RobotState → new clearance 제한
# =============================================================================
def test_R_A3_stale_robot_state_restricts_clearance():
    graph_path = os.path.join(REPO_ROOT, "relay_station", "fleet", "config", "road_graph.yaml")
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.graph = RoadGraph.load(graph_path)
    coord.reservation = Reservation(coord.graph)
    coord.comparator = RouteComparator()
    coord.state_timeout_sec = 1.0
    coord.global_seq = 10
    coord.last_warning = ''
    coord.mission_state = 'RUNNING'

    coord.robots = {
        'pinky1': FleetRobotContext('pinky1', 10, start_node='START_A', goal_node='GOAL_C'),
    }
    ctx = coord.robots['pinky1']
    ctx.route = coord.graph.shortest_route('START_A', 'GOAL_C')
    ctx.route_seq = 1
    coord.reservation.register('pinky1', 10, ctx.route)

    # Fresh state
    coord._now = lambda: 100.0
    ctx.state = RobotState(localized=True, x=0.30, y=0.30)
    ctx.state_time = 100.0
    ctx.clear_until_idx = 2

    # Advance time to become STALE (> 1.0s)
    coord._now = lambda: 102.5
    coord._publish_lane_commands = MagicMock()

    coord._loop_tick()

    assert ctx.is_stale is True, "State older than timeout must be marked is_stale=True"
    assert ctx.clear_until_idx <= 2, "Clearance must not advance when RobotState is stale"


# =============================================================================
# R-A4: Route.waypoints[] progress 계산
# =============================================================================
def test_R_A4_route_waypoints_progress_calculation():
    graph_path = os.path.join(REPO_ROOT, "relay_station", "fleet", "config", "road_graph.yaml")
    graph = RoadGraph.load(graph_path)
    route = graph.shortest_route('START_A', 'GOAL_C')
    comparator = RouteComparator(off_route_threshold=0.35)

    assert len(route.waypoints) >= 5
    assert route.length > 1.0

    # Start
    wp0 = route.waypoints[0]
    res0 = comparator.compare('pinky1', wp0[0], wp0[1], 0.0, route)
    assert res0.nearest_idx == 0
    assert res0.route_progress <= 0.05
    assert res0.cross_track_error < 0.05
    assert res0.off_route is False
    assert res0.status == 'OK'

    # Mid
    mid_i = len(route.waypoints) // 2
    wp_mid = route.waypoints[mid_i]
    res_mid = comparator.compare('pinky1', wp_mid[0], wp_mid[1], 0.0, route)
    assert abs(res_mid.nearest_idx - mid_i) <= 1
    assert 0.3 <= res_mid.route_progress <= 0.7
    assert res_mid.cross_track_error < 0.05

    # End
    wp_end = route.waypoints[-1]
    res_end = comparator.compare('pinky1', wp_end[0], wp_end[1], 0.0, route)
    assert res_end.nearest_idx == route.goal_idx
    assert res_end.route_progress >= 0.95


# =============================================================================
# R-A5: route_idx discrepancy mismatch 감지
# =============================================================================
def test_R_A5_route_state_mismatch_detected():
    graph_path = os.path.join(REPO_ROOT, "relay_station", "fleet", "config", "road_graph.yaml")
    graph = RoadGraph.load(graph_path)
    route = graph.shortest_route('START_A', 'GOAL_C')
    comparator = RouteComparator(mismatch_idx_threshold=4)

    wp = route.waypoints[10]
    ls = LaneStatus()
    ls.route_seq = 1
    ls.route_idx = 2
    ls.edge_id = route.edge_ids[0]

    res = comparator.compare('pinky1', wp[0], wp[1], 0.0, route, lane_status=ls)
    assert res.route_mismatch is True, "Discrepancy between physical and logical idx must be detected"
    assert res.status == 'ROUTE_STATE_MISMATCH'


# =============================================================================
# R-A6: LaneCommand clear_until 전달
# =============================================================================
def test_R_A6_lane_command_clear_until_delivery():
    graph_path = os.path.join(REPO_ROOT, "relay_station", "fleet", "config", "road_graph.yaml")
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.graph = RoadGraph.load(graph_path)
    coord.reservation = Reservation(coord.graph)
    coord.comparator = RouteComparator()
    coord.mission_state = 'RUNNING'
    coord._now = lambda: 100.0

    coord.robots = {
        'pinky1': FleetRobotContext('pinky1', 10, start_node='START_A', goal_node='GOAL_C'),
    }
    ctx = coord.robots['pinky1']
    ctx.route = coord.graph.shortest_route('START_A', 'GOAL_C')
    ctx.route_seq = 3
    ctx.clear_until_idx = 12
    ctx.start_acknowledged = True  # already acknowledged

    mock_pub = MagicMock()
    coord.lane_cmd_pubs = {'pinky1': mock_pub}
    coord.fleet_cmd_pubs = {}  # FleetCommand 발행이 생긴 뒤 이 시험이 안 채워 AttributeError 로 죽었다

    coord._publish_lane_commands()

    assert mock_pub.publish.called
    sent_msg = mock_pub.publish.call_args[0][0]
    assert isinstance(sent_msg, LaneCommand)
    assert sent_msg.command == LaneCommand.CMD_CLEARANCE
    assert sent_msg.route_seq == 3
    assert sent_msg.clear_until_idx == 12


# =============================================================================
# R-A7: same edge → single robot reservation
# =============================================================================
def test_R_A7_same_edge_exclusive_reservation():
    graph_path = os.path.join(REPO_ROOT, "relay_station", "fleet", "config", "road_graph.yaml")
    graph = RoadGraph.load(graph_path)
    res = Reservation(graph, reserve_ahead=0.40, node_stop_margin=0.20)

    # Both robots request routes starting from JUNCTION_1 -> GOAL_C (sharing J1_TO_MID)
    r1_route = graph.shortest_route('JUNCTION_1', 'GOAL_C')
    r2_route = graph.shortest_route('JUNCTION_1', 'GOAL_C')
    assert r1_route.edge_ids[0] == r2_route.edge_ids[0] == 'J1_TO_MID'

    res.register('pinky1', domain_id=10, route=r1_route)
    res.register('pinky2', domain_id=11, route=r2_route)

    clearances = res.step(tick=1)

    assert res.edge_holder.get('J1_TO_MID') == 'pinky1'
    assert res.robots['pinky2'].waiting_for == 'J1_TO_MID'
    assert clearances['pinky2'] == 0, "Robot 2 must wait at start (clear_until=0)"
    assert clearances['pinky1'] > 0, "Robot 1 acquires edge and receives forward clearance"


# =============================================================================
# R-A8: release → waiting robot clearance 확대
# =============================================================================
def test_R_A8_release_expands_waiting_robot_clearance():
    graph_path = os.path.join(REPO_ROOT, "relay_station", "fleet", "config", "road_graph.yaml")
    graph = RoadGraph.load(graph_path)
    res = Reservation(graph, reserve_ahead=0.40, release_behind=0.25, node_stop_margin=0.20)

    r1_route = graph.shortest_route('JUNCTION_1', 'GOAL_C')
    r2_route = graph.shortest_route('JUNCTION_1', 'GOAL_C')

    res.register('pinky1', domain_id=10, route=r1_route)
    res.register('pinky2', domain_id=11, route=r2_route)

    res.step(tick=1)
    assert res.edge_holder.get('J1_TO_MID') == 'pinky1'

    # Pinky 1 advances past edge end and acquires next edge
    edge0_end_s = res.robots['pinky1'].edge_end_s(0)
    res.robots['pinky1'].progress_s = edge0_end_s + 0.30
    res.robots['pinky1'].held.append(1)

    clearances = res.step(tick=2)
    assert res.edge_holder.get('J1_TO_MID') == 'pinky2'
    assert clearances['pinky2'] > 0


# =============================================================================
# R-A9: Zone Event + ARRIVED 결합
# =============================================================================
def test_R_A9_zone_event_and_arrived_combination():
    graph_path = os.path.join(REPO_ROOT, "relay_station", "fleet", "config", "road_graph.yaml")
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.graph = RoadGraph.load(graph_path)
    coord.reservation = Reservation(coord.graph)
    coord.comparator = RouteComparator()
    coord._now = lambda: 100.0
    coord.goal_event_timeout = 3.0
    coord.get_logger = MagicMock()
    coord.mission_state = 'IDLE'
    coord.last_warning = ''

    coord.robots = {
        'pinky1': FleetRobotContext('pinky1', 10, start_node='START_A', goal_node='GOAL_C'),
    }
    ctx = coord.robots['pinky1']
    ctx.route = coord.graph.shortest_route('START_A', 'GOAL_C')
    ctx.route_seq = 1
    coord.reservation.register('pinky1', 10, ctx.route)
    # 통합 검토 FLEET-R1: 도착은 에이전트의 말만으로 받지 않는다 — 확인 창(2 s)의 포즈가 목표 근처이고 도착 보고가 그 창만큼
    # 이어져야 받는다. 예전 이 시험은 포즈 없이 보고 한 번으로 받는 것을 전제했다(그 길로 출발 노드에 선 로봇의 엣지를 놓았다).
    gx, gy = ctx.route.waypoints[ctx.route.goal_idx]
    for _ in range(20):
        coord.reservation.update_pose('pinky1', gx, gy)
    clock = [100.0]
    coord._now = lambda: clock[0]

    # Case 1: Robot reports DRIVE_ARRIVED without vision zone event
    for clock[0] in (100.0, 102.0):
        coord._cb_lane_status('pinky1', LaneStatus(
            drive_state=LaneStatus.DRIVE_ARRIVED,
            route_seq=1,
            route_idx=ctx.route.goal_idx
        ))
    status_dict = coord.get_fleet_status_dict()
    assert status_dict['robots']['pinky1']['arrival_status'] == 'ARRIVAL_PENDING'

    # Case 2: Vision zone event at goal
    coord._cb_vision_zone_event(String(data=json.dumps({
        'robot': 'pinky1',
        'zone_id': 'GOAL_C',
        'event': 'ENTER'
    })))
    status_dict2 = coord.get_fleet_status_dict()
    assert status_dict2['robots']['pinky1']['arrival_status'] == 'ARRIVAL_CONFIRMED'


# =============================================================================
# R-A10: final /cmd_vel single ownership
# =============================================================================
def test_R_A10_final_cmd_vel_single_ownership():
    gate_script = os.path.join(REPO_ROOT, "pinky_fleet_agent", "pinky_fleet_agent", "drive_command_gate.py")
    assert os.path.exists(gate_script)
    with open(gate_script, "r", encoding="utf-8") as f:
        content = f.read()

    assert "'/cmd_vel_teleop'" in content or '"/cmd_vel_teleop"' in content
    assert "'/cmd_vel_mission'" in content or '"/cmd_vel_mission"' in content
    assert "'/estop'" in content or '"/estop"' in content

    launch_file = os.path.join(REPO_ROOT, "pinky_fleet_agent", "launch", "hybrid_robot.launch.xml")
    assert os.path.exists(launch_file)
    with open(launch_file, "r", encoding="utf-8") as f:
        assert 'exec="drive_command_gate"' in f.read()


# =============================================================================
# R-A11: direct bridge bypass 없음
# =============================================================================
def test_R_A11_no_direct_bridge_bypass():
    for r in (1, 2):
        doc = _load_yaml(f"robot{r}_control.yaml")
        topics = doc.get("topics", {})
        cmd_spec = topics.get(f"robot{r}/cmd_vel")
        assert cmd_spec is not None
        assert cmd_spec.get("remap") == "cmd_vel_teleop"


# =============================================================================
# R-A12: unmeasured UI data를 정상으로 표시하지 않음
# =============================================================================
def test_R_A12_unmeasured_ui_data_not_faked():
    graph_path = os.path.join(REPO_ROOT, "relay_station", "fleet", "config", "road_graph.yaml")
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.graph = RoadGraph.load(graph_path)
    coord.reservation = Reservation(coord.graph)
    coord.comparator = RouteComparator()
    coord._now = lambda: 100.0
    coord.last_warning = ''
    coord.mission_state = 'IDLE'

    coord.robots = {
        'pinky1': FleetRobotContext('pinky1', 10, start_node='START_A', goal_node='GOAL_C'),
    }
    status_dict = coord.get_fleet_status_dict()
    r1_stat = status_dict['robots']['pinky1']

    assert r1_stat['is_stale'] is True
    assert r1_stat['state'] is None
    assert r1_stat['arrival_status'] == 'NOT_ARRIVED'


# =============================================================================
# R-A13 (P0-1): Robot 2 odom parameterization in vision_pose_adapter & launch
# =============================================================================
@pytest.mark.skip(reason='원 저장소 robot_onboard/pinky_navigation·robots/ 를 대사한다 — 팀11 레포에 없다 (원 저장소에서도 실패 중)')
def test_R_A13_robot2_odom_topic_parameterization():
    adapter_path = os.path.join(REPO_ROOT, "robot_onboard", "pinky_navigation", "scripts", "vision_pose_adapter.py")
    with open(adapter_path, "r", encoding="utf-8") as f:
        src = f.read()
    assert "odom_topic" in src, "vision_pose_adapter must declare and use odom_topic parameter"

    # Check launch files
    r1_launch = os.path.join(REPO_ROOT, "robot_onboard", "pinky_navigation", "launch", "robot1_drive.launch.py")
    with open(r1_launch, "r", encoding="utf-8") as f:
        r1_src = f.read()
    assert "'odom_topic': '/robot1/odom'" in r1_src

    r2_launch = os.path.join(REPO_ROOT, "robot_onboard", "pinky_navigation", "launch", "robot2_drive.launch.py")
    with open(r2_launch, "r", encoding="utf-8") as f:
        r2_src = f.read()
    assert "'odom_topic': '/robot2/odom'" in r2_src, "robot2_drive.launch.py must inject /robot2/odom"

    # Check start scripts
    s1 = os.path.join(REPO_ROOT, "robots", "robot1", "services", "start_robot1_drive_stack.sh")
    with open(s1, "r", encoding="utf-8") as f:
        assert "odom_topic:=/robot1/odom" in f.read()

    s2 = os.path.join(REPO_ROOT, "robots", "robot2", "services", "start_robot2_drive_stack.sh")
    with open(s2, "r", encoding="utf-8") as f:
        assert "odom_topic:=/robot2/odom" in f.read()


# =============================================================================
# R-A14 (P0-3): Robot 3 & 4 bridge isolation
# =============================================================================
def test_R_A14_robot3_robot4_configs_isolated():
    for r in (3, 4):
        doc = _load_yaml(f"robot{r}_control.yaml")
        topics = doc.get("topics", {})
        assert f"/pinky{r}/state" not in topics
        assert f"/pinky{r}/lane_status" not in topics
        assert f"/pinky{r}/route" not in topics
        assert f"/pinky{r}/lane_command" not in topics
        assert f"/pinky{r}/pose_fix" not in topics
        # cmd_vel must remain mapped to cmd_vel (untouched baseline)
        assert topics[f"robot{r}/cmd_vel"]["remap"] == "cmd_vel"


# =============================================================================
# R-A15 (P0-4 & P2-2): Arena coords & lane_mission single source of truth
# =============================================================================
def test_R_A15_lane_mission_and_road_graph_arena_spec():
    mission_path = os.path.join(REPO_ROOT, "relay_station", "fleet", "config", "lane_mission.yaml")
    with open(mission_path, "r", encoding="utf-8") as f:
        m_cfg = yaml.safe_load(f)

    # Must specify distinct starts (START_A, START_B) and common GOAL_C
    robots = {r["name"]: r for r in m_cfg["robots"]}
    assert "pinky1" in robots and "pinky2" in robots
    assert robots["pinky1"]["start"] == "START_A"
    assert robots["pinky2"]["start"] == "START_B"
    assert robots["pinky1"]["goal"] == "GOAL_C"
    assert robots["pinky2"]["goal"] == "GOAL_C"

    # RoadGraph nodes must be within 2.34m x 1.26m arena bounds
    graph_path = os.path.join(REPO_ROOT, "relay_station", "fleet", "config", "road_graph.yaml")
    graph = RoadGraph.load(graph_path)
    for nid, node in graph.nodes.items():
        assert 0.0 <= node.x <= 2.34, f"Node {nid} x={node.x} outside arena width (2.34m)"
        assert 0.0 <= node.y <= 1.26, f"Node {nid} y={node.y} outside arena height (1.26m)"

    # Coordinator must load lane_mission.yaml directly
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.config_path = mission_path
    with open(mission_path, "r", encoding="utf-8") as f:
        coord.config = yaml.safe_load(f)
    coord.robots = {
        r["name"]: FleetRobotContext(r["name"], r["domain_id"], r["start"], r["goal"])
        for r in coord.config["robots"]
    }
    assert coord.robots["pinky1"].start_node == "START_A"
    assert coord.robots["pinky2"].start_node == "START_B"
    assert coord.robots["pinky1"].goal_node == "GOAL_C"
    assert coord.robots["pinky2"].goal_node == "GOAL_C"


# =============================================================================
# R-A16 (P1-1): route_seq mismatch detection in RouteComparator
# =============================================================================
def test_R_A16_route_seq_mismatch_detection():
    graph_path = os.path.join(REPO_ROOT, "relay_station", "fleet", "config", "road_graph.yaml")
    graph = RoadGraph.load(graph_path)
    route = graph.shortest_route('START_A', 'GOAL_C')
    comparator = RouteComparator()

    wp = route.waypoints[0]
    ls = LaneStatus()
    ls.route_seq = 99  # Mismatched sequence!
    ls.route_idx = 0   # Index perfectly matches physical pose
    ls.edge_id = route.edge_ids[0]

    # Comparing with expected_route_seq = 1
    res = comparator.compare(
        'pinky1', wp[0], wp[1], 0.0, route,
        lane_status=ls, expected_route_seq=1
    )
    assert res.route_mismatch is True, "route_seq difference must trigger route_mismatch"
    assert res.status == 'ROUTE_STATE_MISMATCH'


# =============================================================================
# R-A17 (P1-2): Goal zone freshness and zone_id verification
# =============================================================================
def test_R_A17_goal_zone_freshness_and_zone_id_verification():
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.goal_event_timeout = 3.0
    coord._now = lambda: 100.0
    ctx = FleetRobotContext('pinky1', 10, start_node='START_A', goal_node='GOAL_C')

    # Case A: Stale event (> 3.0s ago)
    ctx.last_zone_event = {'zone_id': 'GOAL_C', 'event': 'ENTER', 'timestamp': 95.0}
    assert coord._is_valid_goal_zone_event(ctx) is False, "Event older than 3.0s must be rejected"

    # Case B: Wrong zone (e.g. Start Zone event lingering)
    ctx.last_zone_event = {'zone_id': 'START_A', 'event': 'ENTER', 'timestamp': 100.0}
    assert coord._is_valid_goal_zone_event(ctx) is False, "Start zone event must not confirm goal arrival"

    # Case C: Fresh goal zone event
    ctx.last_zone_event = {'zone_id': 'GOAL_C', 'event': 'ENTER', 'timestamp': 99.5}
    assert coord._is_valid_goal_zone_event(ctx) is True, "Fresh goal zone event must be accepted"


# =============================================================================
# R-A18 (P1-3): Gateway HTTP thread decoupling
# =============================================================================
def test_R_A18_gateway_web_http_thread_decoupling():
    server_script = os.path.join(REPO_ROOT, "relay_station", "gateway_web", "gateway_web_server.py")
    with open(server_script, "r", encoding="utf-8") as f:
        src = f.read()

    # Must have publishers on RobotDataSubscriberNode
    assert "self.pub_fleet_control" in src
    assert "self.pub_vision_zone_event" in src
    assert "send_fleet_control" in src
    assert "send_vision_zone_event" in src

    # HTTP POST handler must route through send_fleet_control / send_vision_zone_event
    assert "GLOBAL_ROBOT_SUB_NODE.send_fleet_control(payload)" in src
    assert "GLOBAL_ROBOT_SUB_NODE.send_vision_zone_event(req_json)" in src


# =============================================================================
# R-A19 (P1-4): START command retry logic
# =============================================================================
def test_R_A19_start_command_retry_logic():
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.start_retry_max = 5
    coord.start_retry_interval = 1.0
    coord.mission_state = 'IDLE'
    coord.get_logger = MagicMock()

    mock_pub = MagicMock()
    coord.lane_cmd_pubs = {'pinky1': mock_pub}
    coord.fleet_cmd_pubs = {}  # FleetCommand 발행이 생긴 뒤 이 시험이 안 채워 AttributeError 로 죽었다
    coord.reservation = MagicMock()  # 통합 검토 RES-F2: LaneStatus 의 진행 보고가 예약(잡기)에도 들어간다 — 같은 까닭
    ctx = FleetRobotContext('pinky1', 10, start_node='START_A', goal_node='GOAL_C')
    ctx.route = MagicMock()
    ctx.route_seq = 1
    coord.robots = {'pinky1': ctx}

    # Start fleet at t=10.0
    coord._now = lambda: 10.0
    coord.start_fleet()
    assert coord.mission_state == 'RUNNING'
    assert ctx.start_acknowledged is False
    assert ctx.start_retry_count == 5

    # At t=10.5 (within interval), should not resend START
    coord._now = lambda: 10.5
    mock_pub.reset_mock()
    coord._publish_lane_commands()
    assert not mock_pub.publish.called, "Should wait for retry interval before resending START"

    # At t=11.1 (after interval), should resend START and decrement retry count
    coord._now = lambda: 11.1
    coord._publish_lane_commands()
    assert mock_pub.publish.called
    msg = mock_pub.publish.call_args[0][0]
    assert msg.command == LaneCommand.CMD_START
    assert ctx.start_retry_count == 4

    # 옛 경로 번호의 주행 보고는 새 START 의 응답이 아니다(재배정 직후 에이전트가 새 Route 를 받기 전 — 직렬 검토)
    ls = LaneStatus()
    ls.drive_state = LaneStatus.DRIVE_CRUISE
    ls.route_seq = ctx.route_seq - 1
    coord._cb_lane_status('pinky1', ls)
    assert ctx.start_acknowledged is False

    # Robot sends LaneStatus indicating DRIVE_CRUISE on this route -> Acknowledged
    ls.route_seq = ctx.route_seq
    coord._cb_lane_status('pinky1', ls)
    assert ctx.start_acknowledged is True

    # Next cycle should publish CMD_CLEARANCE, not START
    mock_pub.reset_mock()
    coord._now = lambda: 12.5
    coord._publish_lane_commands()
    assert mock_pub.publish.called
    msg2 = mock_pub.publish.call_args[0][0]
    assert msg2.command == LaneCommand.CMD_CLEARANCE


# =============================================================================
# R-A20 (P1-5): DriveCommandGate E-STOP cache invalidation
# =============================================================================
def test_R_A20_drive_safety_estop_cache_invalidation():
    gate = CommandGateCore(teleop_timeout_sec=0.5, mission_timeout_sec=0.5)

    # Feed teleop command at t=10.0
    gate.feed_teleop(0.2, 0.1, 10.0)
    vx, wz, src = gate.step(10.1)
    assert src == GateSource.TELEOP
    assert vx == 0.2

    # Engage E-STOP
    gate.set_estop(True)
    vx, wz, src = gate.step(10.2)
    assert src == GateSource.ESTOP
    assert vx == 0.0

    # Release E-STOP at t=10.3 (within 0.5s of teleop command)
    gate.set_estop(False)

    # Stale teleop command must have been invalidated on release
    vx, wz, src = gate.step(10.3)
    assert src == GateSource.IDLE, "Cached commands must be wiped on E-STOP release"
    assert vx == 0.0
    assert wz == 0.0


# =============================================================================
# R-A21 (P2-1): MISSION_DONE state transition
# =============================================================================
def test_R_A21_mission_done_transition():
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.mission_state = 'RUNNING'
    coord.get_logger = MagicMock()

    ctx1 = FleetRobotContext('pinky1', 10, 'START_A', 'GOAL_C')
    ctx1.route = MagicMock()
    ctx2 = FleetRobotContext('pinky2', 11, 'START_B', 'GOAL_C')
    ctx2.route = MagicMock()
    coord.robots = {'pinky1': ctx1, 'pinky2': ctx2}

    # Robot 1 arrived & confirmed, but Robot 2 not yet
    ctx1.arrived = True
    ctx1.arrival_confirmed = True
    coord._check_mission_done()
    assert coord.mission_state == 'RUNNING'

    # Robot 2 arrives & confirmed
    ctx2.arrived = True
    ctx2.arrival_confirmed = True
    coord._check_mission_done()
    assert coord.mission_state == 'DONE', "When all active robots confirm arrival, mission must transition to DONE"
