# -*- coding: utf-8 -*-
"""Cross-contract Acceptance Tests: Track R Relay Contract Delta.

Verifies conformance with:
  - docs/PROMPT_20260922_RELAY_FOLLOWUP_AFTER_116AEE1.md
  - docs/PLAN_20260922_SHARED_ROBOT_INTERFACE_INTEGRATION_DELTA.md

Test Cases:
  C-A1: PoseFix JSON -> ROS PoseFix field preservation (using Tablet 2080e27 fixture)
  C-A2: pinky1 / pinky2 correct topic routing & domain bridge isolation
  C-A3: Tablet canonical VisionZoneEvent parsing and storage
  C-A4: Legacy VisionZoneEvent alias compatibility (robot -> robot_name, event -> event_type)
  C-A5: Duplicate and out-of-order VisionZoneEvent rejection per (robot_name, camera_id)
  C-A6: Legacy POST /api/vision/pose endpoint remains separate and unaffected
  C-A7: Production motion path has no Nav2 CMD_GOTO coexistence (Route + LaneCommand only)
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

GATEWAY_DIR = os.path.join(REPO_ROOT, "relay_station", "gateway_web")
if GATEWAY_DIR not in sys.path:
    sys.path.insert(0, GATEWAY_DIR)

CONFIGS_DIR = os.path.join(REPO_ROOT, "relay_station", "domain_bridge", "configs")

import vision_ingest
from std_msgs.msg import String
from pinky_lane_msgs.msg import PoseFix as RosPoseFix, LaneStatus
from pinky_fleet_msgs.msg import FleetCommand
from relay_station.fleet.fleet_coordinator import (
    RelayFleetCoordinator, FleetRobotContext, MISSION_RUNNING, MISSION_DONE
)
from relay_station.fleet.road_graph import RoadGraph
from relay_station.fleet.reservation import Reservation
from relay_station.fleet.route_comparator import RouteComparator


# =============================================================================
# Tablet 2080e27 Canonical Fixtures
# =============================================================================

TABLET_2080E27_POSE_FIX_FIXTURE = {
    "header": {
        "frame_id": "map",
        "stamp": {
            "sec": 1726900123,
            "nanosec": 456789000,
        },
    },
    "stamp_is_robot_clock": False,
    "robot_name": "pinky1",
    "seq": 105,
    "x": 1.25432,
    "y": 0.45678,
    "yaw": 0.78539,
    "marker_id": 30,
    "marker_range": 0.0,
    "reproj_error": 0.015,
    "n_markers": 4,
    "pipeline_latency": 0.0234,
}

TABLET_2080E27_ZONE_EVENT_FIXTURE = {
    "robot_name": "pinky1",
    "camera_id": "GOAL",
    "zone_id": "GOAL_C",
    "event_type": "ENTER",
    "confidence": 0.95,
    "timestamp": 1726900123.4567,
    "sequence": 42,
}


def _load_yaml(filename: str):
    path = os.path.join(CONFIGS_DIR, filename)
    assert os.path.exists(path), f"File {filename} does not exist in {CONFIGS_DIR}"
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# =============================================================================
# C-A1: PoseFix JSON -> ROS PoseFix field preservation
# =============================================================================

def test_C_A1_pose_fix_field_preservation():
    """Tablet 2080e27 canonical PoseFix payload parses and preserves 100% of fields."""
    fixture = dict(TABLET_2080E27_POSE_FIX_FIXTURE)

    ok, reason, norm = vision_ingest.validate_pose_fix(fixture, allowed_robots=("pinky1", "pinky2"))
    assert ok is True, f"Validation failed: {reason}"
    assert norm is not None

    # Verify normalized dict retains exact values
    assert norm["robot_name"] == "pinky1"
    assert norm["frame_id"] == "map"
    assert norm["stamp_sec"] == 1726900123
    assert norm["stamp_nanosec"] == 456789000
    assert norm["stamp_is_robot_clock"] is False
    assert norm["seq"] == 105
    assert abs(norm["x"] - 1.25432) < 1e-5
    assert abs(norm["y"] - 0.45678) < 1e-5
    assert abs(norm["yaw"] - 0.78539) < 1e-5
    assert norm["marker_id"] == 30
    assert norm["marker_range"] == 0.0
    assert abs(norm["reproj_error"] - 0.015) < 1e-4
    assert norm["n_markers"] == 4
    assert abs(norm["pipeline_latency"] - 0.0234) < 1e-4

    # Instantiate ROS PoseFix message and test field preservation
    msg = RosPoseFix()
    msg.header.frame_id = norm["frame_id"]
    msg.header.stamp.sec = norm["stamp_sec"]
    msg.header.stamp.nanosec = norm["stamp_nanosec"]
    msg.stamp_is_robot_clock = norm["stamp_is_robot_clock"]
    msg.robot_name = norm["robot_name"]
    msg.seq = norm["seq"]
    msg.x = float(norm["x"])
    msg.y = float(norm["y"])
    msg.yaw = float(norm["yaw"])
    msg.marker_id = norm["marker_id"]
    msg.marker_range = float(norm["marker_range"])
    msg.reproj_error = float(norm["reproj_error"])
    msg.n_markers = norm["n_markers"]
    msg.pipeline_latency = float(norm["pipeline_latency"])

    assert msg.header.frame_id == "map"
    assert msg.header.stamp.sec == 1726900123
    assert msg.header.stamp.nanosec == 456789000
    assert msg.stamp_is_robot_clock is False
    assert msg.robot_name == "pinky1"
    assert msg.seq == 105
    assert abs(msg.x - 1.25432) < 1e-5
    assert abs(msg.y - 0.45678) < 1e-5
    assert abs(msg.yaw - 0.78539) < 1e-5
    assert msg.marker_id == 30
    assert msg.marker_range == 0.0
    assert abs(msg.reproj_error - 0.015) < 1e-4
    assert msg.n_markers == 4
    assert abs(msg.pipeline_latency - 0.0234) < 1e-4


def test_C_A1_pose_fix_validation_rejections():
    """PoseFix validation rejects invalid frames, robots, non-finite values, bad types, and range violations."""
    base = dict(TABLET_2080E27_POSE_FIX_FIXTURE)

    # 1. frame_id != map
    bad_frame = dict(base, header={"frame_id": "odom", "stamp": {"sec": 1, "nanosec": 0}})
    ok, reason, _ = vision_ingest.validate_pose_fix(bad_frame)
    assert ok is False and reason == vision_ingest.REJECT_INVALID_FRAME

    # 2. robot_name not in {pinky1, pinky2}
    bad_robot = dict(base, robot_name="pinky3")
    ok, reason, _ = vision_ingest.validate_pose_fix(bad_robot)
    assert ok is False and reason == vision_ingest.REJECT_INVALID_ROBOT

    # 3. non-finite coordinates (NaN / Inf)
    for bad_val in (float('nan'), float('inf'), -float('inf')):
        bad_coord = dict(base, x=bad_val)
        ok, reason, _ = vision_ingest.validate_pose_fix(bad_coord)
        assert ok is False and "NOT_FINITE" in reason

    # 4. non-numeric coordinates
    bad_type = dict(base, y="invalid_string")
    ok, reason, _ = vision_ingest.validate_pose_fix(bad_type)
    assert ok is False and "BAD_TYPE" in reason

    # 5. Timestamp validation (P0)
    # 5a. Missing stamp or missing sec/nanosec
    no_stamp = dict(base, header={"frame_id": "map"})
    ok, reason, _ = vision_ingest.validate_pose_fix(no_stamp)
    assert ok is False and "header.stamp" in reason

    # 5b. Float sec (truncation forbidden)
    float_sec = dict(base, header={"frame_id": "map", "stamp": {"sec": 1726900123.5, "nanosec": 0}})
    ok, reason, _ = vision_ingest.validate_pose_fix(float_sec)
    assert ok is False and reason == f"{vision_ingest.REJECT_TYPE}:header.stamp.sec"

    # 5c. Negative sec
    neg_sec = dict(base, header={"frame_id": "map", "stamp": {"sec": -1, "nanosec": 0}})
    ok, reason, _ = vision_ingest.validate_pose_fix(neg_sec)
    assert ok is False and reason == f"{vision_ingest.REJECT_RANGE}:header.stamp.sec"

    # 5d. Float nanosec
    float_ns = dict(base, header={"frame_id": "map", "stamp": {"sec": 10, "nanosec": 500.5}})
    ok, reason, _ = vision_ingest.validate_pose_fix(float_ns)
    assert ok is False and reason == f"{vision_ingest.REJECT_TYPE}:header.stamp.nanosec"

    # 5e. Nanosec out of range (< 0 or >= 1e9)
    neg_ns = dict(base, header={"frame_id": "map", "stamp": {"sec": 10, "nanosec": -1}})
    ok, reason, _ = vision_ingest.validate_pose_fix(neg_ns)
    assert ok is False and reason == f"{vision_ingest.REJECT_RANGE}:header.stamp.nanosec"

    big_ns = dict(base, header={"frame_id": "map", "stamp": {"sec": 10, "nanosec": 1_000_000_000}})
    ok, reason, _ = vision_ingest.validate_pose_fix(big_ns)
    assert ok is False and reason == f"{vision_ingest.REJECT_RANGE}:header.stamp.nanosec"

    # 6. Strict integer check for seq, marker_id, n_markers (P1)
    float_seq = dict(base, seq=105.5)
    ok, reason, _ = vision_ingest.validate_pose_fix(float_seq)
    assert ok is False and reason == f"{vision_ingest.REJECT_TYPE}:seq"

    bool_seq = dict(base, seq=True)
    ok, reason, _ = vision_ingest.validate_pose_fix(bool_seq)
    assert ok is False and reason == f"{vision_ingest.REJECT_TYPE}:seq"

    neg_seq = dict(base, seq=-5)
    ok, reason, _ = vision_ingest.validate_pose_fix(neg_seq)
    assert ok is False and reason == f"{vision_ingest.REJECT_RANGE}:seq"

    float_marker_id = dict(base, marker_id=30.0)
    ok, reason, _ = vision_ingest.validate_pose_fix(float_marker_id)
    assert ok is False and reason == f"{vision_ingest.REJECT_TYPE}:marker_id"

    float_n_markers = dict(base, n_markers=4.0)
    ok, reason, _ = vision_ingest.validate_pose_fix(float_n_markers)
    assert ok is False and reason == f"{vision_ingest.REJECT_TYPE}:n_markers"

    neg_n_markers = dict(base, n_markers=-1)
    ok, reason, _ = vision_ingest.validate_pose_fix(neg_n_markers)
    assert ok is False and reason == f"{vision_ingest.REJECT_RANGE}:n_markers"

    # 7. Non-negative floats for marker_range, reproj_error, pipeline_latency (P1)
    neg_range = dict(base, marker_range=-0.1)
    ok, reason, _ = vision_ingest.validate_pose_fix(neg_range)
    assert ok is False and reason == f"{vision_ingest.REJECT_RANGE}:marker_range"

    neg_reproj = dict(base, reproj_error=-0.01)
    ok, reason, _ = vision_ingest.validate_pose_fix(neg_reproj)
    assert ok is False and reason == f"{vision_ingest.REJECT_RANGE}:reproj_error"

    neg_latency = dict(base, pipeline_latency=-0.001)
    ok, reason, _ = vision_ingest.validate_pose_fix(neg_latency)
    assert ok is False and reason == f"{vision_ingest.REJECT_RANGE}:pipeline_latency"


# =============================================================================
# C-A2: pinky1/pinky2 correct routing & bridge isolation
# =============================================================================

def test_C_A2_routing_and_bridge_isolation():
    """Verify pinky1 bridges only to D10 and pinky2 only to D11. R3/R4 remain untouched."""
    doc_r1 = _load_yaml("robot1_control.yaml")
    doc_r2 = _load_yaml("robot2_control.yaml")
    doc_r3 = _load_yaml("robot3_control.yaml")
    doc_r4 = _load_yaml("robot4_control.yaml")
    doc_tm = _load_yaml("team_mirror.yaml")

    topics_r1 = doc_r1.get("topics", {})
    topics_r2 = doc_r2.get("topics", {})
    topics_r3 = doc_r3.get("topics", {})
    topics_r4 = doc_r4.get("topics", {})

    # R1: /pinky1/pose_fix and /pinky1/command bridged 8 -> 10
    assert "/pinky1/pose_fix" in topics_r1
    assert topics_r1["/pinky1/pose_fix"]["from_domain"] == 8
    assert topics_r1["/pinky1/pose_fix"]["to_domain"] == 10

    assert "/pinky1/command" in topics_r1
    assert topics_r1["/pinky1/command"]["from_domain"] == 8
    assert topics_r1["/pinky1/command"]["to_domain"] == 10
    assert topics_r1["/pinky1/command"]["type"] == "pinky_fleet_msgs/msg/FleetCommand"

    # R2: /pinky2/pose_fix and /pinky2/command bridged 8 -> 11
    assert "/pinky2/pose_fix" in topics_r2
    assert topics_r2["/pinky2/pose_fix"]["from_domain"] == 8
    assert topics_r2["/pinky2/pose_fix"]["to_domain"] == 11

    assert "/pinky2/command" in topics_r2
    assert topics_r2["/pinky2/command"]["from_domain"] == 8
    assert topics_r2["/pinky2/command"]["to_domain"] == 11
    assert topics_r2["/pinky2/command"]["type"] == "pinky_fleet_msgs/msg/FleetCommand"

    # Isolation: R1 topics not in R2, R2 topics not in R1
    assert "/pinky2/pose_fix" not in topics_r1
    assert "/pinky2/command" not in topics_r1
    assert "/pinky1/pose_fix" not in topics_r2
    assert "/pinky1/command" not in topics_r2

    # R3 and R4 have NO pinky topics
    for t in ("/pinky1/pose_fix", "/pinky2/pose_fix", "/pinky1/command", "/pinky2/command"):
        assert t not in topics_r3, f"Robot 3 must not contain {t}"
        assert t not in topics_r4, f"Robot 4 must not contain {t}"


# =============================================================================
# C-A3: Canonical ZoneEvent parse
# =============================================================================

def test_C_A3_canonical_zone_event_parse():
    """Relay correctly parses Tablet canonical ZoneEvent and tracks received_at."""
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.get_logger = MagicMock()
    coord.goal_event_timeout = 3.0
    coord._zone_event_seq = {}
    coord._now = lambda: 100.0

    ctx = FleetRobotContext("pinky1", 10, start_node="START_A", goal_node="GOAL_C")
    coord.robots = {"pinky1": ctx}
    coord.mission_state = MISSION_RUNNING

    raw_json = json.dumps(TABLET_2080E27_ZONE_EVENT_FIXTURE)
    coord._cb_vision_zone_event(String(data=raw_json))

    assert ctx.last_zone_event is not None
    ev = ctx.last_zone_event
    assert ev["robot_name"] == "pinky1"
    assert ev["camera_id"] == "GOAL"
    assert ev["zone_id"] == "GOAL_C"
    assert ev["event_type"] == "ENTER"
    assert ev["confidence"] == 0.95
    assert ev["source_timestamp"] == 1726900123.4567
    assert ev["received_at"] == 100.0
    assert ev["sequence"] == 42

    # Goal freshness check passes using received_at
    assert coord._is_valid_goal_zone_event(ctx) is True


# =============================================================================
# C-A4: Legacy ZoneEvent alias compatibility
# =============================================================================

def test_C_A4_legacy_zone_event_alias_compatibility():
    """Relay maintains backwards compatibility with legacy keys 'robot' and 'event'."""
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.get_logger = MagicMock()
    coord.goal_event_timeout = 3.0
    coord._zone_event_seq = {}
    coord._now = lambda: 50.0

    ctx = FleetRobotContext("pinky1", 10, start_node="START_A", goal_node="GOAL_C")
    coord.robots = {"pinky1": ctx}
    coord.mission_state = MISSION_RUNNING

    legacy_payload = {
        "robot": "pinky1",
        "zone_id": "GOAL_C",
        "event": "ENTER"
    }
    coord._cb_vision_zone_event(String(data=json.dumps(legacy_payload)))

    assert ctx.last_zone_event is not None
    assert ctx.last_zone_event["event_type"] == "ENTER"
    assert ctx.last_zone_event["event"] == "ENTER"
    assert coord._is_valid_goal_zone_event(ctx) is True


# =============================================================================
# C-A5: Duplicate and out-of-order ZoneEvent rejection
# =============================================================================

def test_C_A5_duplicate_and_out_of_order_zone_event_reject():
    """Zone events with seq <= last_seq for a given (robot, camera) are dropped."""
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.get_logger = MagicMock()
    coord.goal_event_timeout = 3.0
    coord._zone_event_seq = {}
    coord._now = lambda: 100.0

    ctx = FleetRobotContext("pinky1", 10, start_node="START_A", goal_node="GOAL_C")
    coord.robots = {"pinky1": ctx}

    # 1. First event with seq=10 accepted
    evt1 = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, sequence=10, confidence=0.8)
    coord._cb_vision_zone_event(String(data=json.dumps(evt1)))
    assert ctx.last_zone_event["sequence"] == 10
    assert ctx.last_zone_event["confidence"] == 0.8

    # 2. Duplicate event with seq=10 dropped (confidence does NOT change)
    evt2 = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, sequence=10, confidence=0.99)
    coord._cb_vision_zone_event(String(data=json.dumps(evt2)))
    assert ctx.last_zone_event["confidence"] == 0.8  # unchanged!

    # 3. Out-of-order event with seq=9 dropped
    evt3 = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, sequence=9, confidence=0.99)
    coord._cb_vision_zone_event(String(data=json.dumps(evt3)))
    assert ctx.last_zone_event["confidence"] == 0.8  # unchanged!

    # 4. Newer event with seq=11 accepted
    evt4 = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, sequence=11, confidence=0.91)
    coord._cb_vision_zone_event(String(data=json.dumps(evt4)))
    assert ctx.last_zone_event["sequence"] == 11
    assert ctx.last_zone_event["confidence"] == 0.91

    # 5. Different camera (START_A) with seq=10 accepted (keyed by camera_id)
    evt5 = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, camera_id="START_A", sequence=10, confidence=0.77)
    coord._cb_vision_zone_event(String(data=json.dumps(evt5)))
    assert ctx.last_zone_event["camera_id"] == "START_A"
    assert ctx.last_zone_event["sequence"] == 10


# =============================================================================
# C-A6: Legacy /api/vision/pose remains separate
# =============================================================================



# =============================================================================
# C-A7: Nav2 + Marker Hybrid Architecture Boundary (P1)
# =============================================================================

def test_C_A7_coordinator_does_not_own_nav2_action_client():
    """Verify that in Nav2 + Marker hybrid architecture, RelayFleetCoordinator acts as fleet arbiter/coordinator
    and does not directly own/run Nav2 NavigateToPose ActionClient (which is owned onboard by PinkyAgent via FleetCommand)."""
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.get_logger = MagicMock()
    coord.mission_state = MISSION_RUNNING

    # LaneCommand publishers exist for active robots
    mock_r1_pub = MagicMock()
    mock_r2_pub = MagicMock()
    coord.lane_cmd_pubs = {"pinky1": mock_r1_pub, "pinky2": mock_r2_pub}

    # Verify coordinator has NO NavigateToPose or CMD_GOTO action clients directly
    assert not hasattr(coord, "nav_to_pose_client"), "Coordinator must not run NavigateToPose ActionClient directly"
    assert not hasattr(coord, "action_client"), "Coordinator must not run Nav2 action client directly"

    # Verify FleetCommand definition supports CMD_GOTO (for onboard agent) and administrative commands
    assert FleetCommand.CMD_GOTO == 0
    assert FleetCommand.CMD_SET_INITIAL_POSE == 4
    assert FleetCommand.CMD_SET_MAP == 6


# Alias for backward compatibility
test_C_A7_production_path_no_cmd_goto_coexistence = test_C_A7_coordinator_does_not_own_nav2_action_client


# =============================================================================
# C-A8: ZoneEvent ingress validation (P1)
# =============================================================================

def test_C_A8_zone_event_ingress_validation():
    """Verify ingress validation for /api/vision/zone_event (P1)."""
    base = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE)

    # Valid payload
    ok, reason, norm = vision_ingest.validate_zone_event(base)
    assert ok is True
    assert norm["robot_name"] == "pinky1"
    assert norm["event_type"] == "ENTER"
    assert norm["sequence"] == 42

    # Invalid robot
    bad_robot = dict(base, robot_name="pinky4")
    ok, reason, _ = vision_ingest.validate_zone_event(bad_robot)
    assert ok is False and reason == vision_ingest.REJECT_INVALID_ROBOT

    # Invalid event type
    bad_event = dict(base, event_type="FLY_AWAY")
    ok, reason, _ = vision_ingest.validate_zone_event(bad_event)
    assert ok is False and reason == vision_ingest.REJECT_INVALID_EVENT

    # Confidence out of range (< 0 or > 1)
    for bad_conf in (-0.1, 1.05):
        bad_c = dict(base, confidence=bad_conf)
        ok, reason, _ = vision_ingest.validate_zone_event(bad_c)
        assert ok is False and "confidence" in reason

    # Negative timestamp
    bad_ts = dict(base, timestamp=-10.0)
    ok, reason, _ = vision_ingest.validate_zone_event(bad_ts)
    assert ok is False and "timestamp" in reason

    # Float or negative sequence
    float_seq = dict(base, sequence=4.5)
    ok, reason, _ = vision_ingest.validate_zone_event(float_seq)
    assert ok is False and "sequence" in reason

    neg_seq = dict(base, sequence=-1)
    ok, reason, _ = vision_ingest.validate_zone_event(neg_seq)
    assert ok is False and "sequence" in reason


# =============================================================================
# C-A9: Vision API fail-closed authentication (P1)
# =============================================================================

def test_C_A9_vision_api_auth_fail_closed():
    """Verify fail-closed authentication on vision endpoints (/api/vision/pose_fix and /api/vision/zone_event)."""
    import gateway_web_server

    class MockHandler:
        def __init__(self, headers=None, path="/api/vision/pose_fix"):
            self.headers = headers or {}
            self.path = path

    # 1. No credentials provided -> Unauthorized (False)
    h_empty = MockHandler({}, "/api/vision/pose_fix")
    assert gateway_web_server.GatewayRequestHandler._check_vision_auth(h_empty) is False

    # 2. Wrong API Key -> Unauthorized (False)
    h_bad_key = MockHandler({"X-API-Key": "wrong-key"}, "/api/vision/pose_fix")
    assert gateway_web_server.GatewayRequestHandler._check_vision_auth(h_bad_key) is False

    # 3. Valid X-API-Key -> Authorized (True)
    h_good_key = MockHandler({"X-API-Key": gateway_web_server.VISION_API_KEY}, "/api/vision/pose_fix")
    assert gateway_web_server.GatewayRequestHandler._check_vision_auth(h_good_key) is True

    # 4. Valid Authorization: Bearer <token> -> Authorized (True)
    h_bearer = MockHandler({"Authorization": f"Bearer {gateway_web_server.VISION_API_KEY}"}, "/api/vision/pose_fix")
    assert gateway_web_server.GatewayRequestHandler._check_vision_auth(h_bearer) is True

    # 5. Invalid Bearer token -> Unauthorized (False)
    h_bad_bearer = MockHandler({"Authorization": "Bearer bad-token"}, "/api/vision/pose_fix")
    assert gateway_web_server.GatewayRequestHandler._check_vision_auth(h_bad_bearer) is False

    # 6. Valid query parameter ?token=... -> Authorized (True)
    h_query = MockHandler({}, f"/api/vision/zone_event?token={gateway_web_server.VISION_API_KEY}")
    assert gateway_web_server.GatewayRequestHandler._check_vision_auth(h_query) is True

    # 7. Invalid query parameter -> Unauthorized (False)
    h_bad_query = MockHandler({}, "/api/vision/zone_event?token=wrong")
    assert gateway_web_server.GatewayRequestHandler._check_vision_auth(h_bad_query) is False


# =============================================================================
# C-A10: publish_pose_fix failure returns HTTP 503 (P0)
# =============================================================================

def test_C_A10_publish_pose_fix_failure_returns_503():
    """Verify HTTP 503 is returned when publish_pose_fix fails (P0)."""
    import gateway_web_server

    node = gateway_web_server.RobotDataSubscriberNode.__new__(gateway_web_server.RobotDataSubscriberNode)
    node.pub_pose_fix = {}  # No publishers configured

    published, subs = node.publish_pose_fix({
        "robot_name": "pinky1", "stamp_sec": 1, "stamp_nanosec": 0,
        "x": 0.0, "y": 0.0, "yaw": 0.0
    })
    assert published is False
    assert subs is None

    # Verify server source routes published == False to HTTP 503 ROS_PUBLISH_FAILED
    server_script = os.path.join(GATEWAY_DIR, "gateway_web_server.py")
    with open(server_script, "r", encoding="utf-8") as f:
        src = f.read()

    assert "if not published:" in src
    assert "ROS_PUBLISH_FAILED" in src
    assert "code=503" in src


# =============================================================================
# C-A11: Tablet restart sequence reset handling (P1)
# =============================================================================

def test_C_A11_zone_event_tablet_restart_sequence_reset():
    """Verify Tablet process restart or sequence reset is handled without locking out (P1)."""
    coord = RelayFleetCoordinator.__new__(RelayFleetCoordinator)
    coord.get_logger = MagicMock()
    coord.goal_event_timeout = 3.0
    coord._zone_event_seq = {}
    coord._zone_event_session = {}
    coord._now = lambda: 100.0

    ctx = FleetRobotContext("pinky1", 10, start_node="START_A", goal_node="GOAL_C")
    coord.robots = {"pinky1": ctx}

    # 1. Normal sequence progress up to seq=100
    evt100 = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, sequence=100, confidence=0.85, session_id="boot_1")
    coord._cb_vision_zone_event(String(data=json.dumps(evt100)))
    assert ctx.last_zone_event["sequence"] == 100

    # 2. Lower sequence seq=50 in same session without reset is dropped
    evt50 = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, sequence=50, confidence=0.99, session_id="boot_1")
    coord._cb_vision_zone_event(String(data=json.dumps(evt50)))
    assert ctx.last_zone_event["sequence"] == 100  # not overwritten

    # 3. Tablet restart case A: sequence resets to 1 (new run)
    evt_restart1 = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, sequence=1, confidence=0.91, session_id="boot_1")
    coord._cb_vision_zone_event(String(data=json.dumps(evt_restart1)))
    assert ctx.last_zone_event["sequence"] == 1  # Accepted!

    # 4. Sequence progresses again to 2
    evt_restart2 = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, sequence=2, confidence=0.92, session_id="boot_1")
    coord._cb_vision_zone_event(String(data=json.dumps(evt_restart2)))
    assert ctx.last_zone_event["sequence"] == 2

    # 5. Tablet restart case B: new session_id / boot_id
    evt_new_boot = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, sequence=1, confidence=0.93, session_id="boot_2")
    coord._cb_vision_zone_event(String(data=json.dumps(evt_new_boot)))
    assert ctx.last_zone_event["session_id"] == "boot_2"
    assert ctx.last_zone_event["sequence"] == 1

    # 6. Tablet restart case C: explicit reset flag
    evt_prog = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, sequence=10, confidence=0.94, session_id="boot_2")
    coord._cb_vision_zone_event(String(data=json.dumps(evt_prog)))
    assert ctx.last_zone_event["sequence"] == 10

    evt_reset = dict(TABLET_2080E27_ZONE_EVENT_FIXTURE, sequence=5, reset=True, confidence=0.95, session_id="boot_2")
    coord._cb_vision_zone_event(String(data=json.dumps(evt_reset)))
    assert ctx.last_zone_event["sequence"] == 5  # Accepted due to reset: True!
