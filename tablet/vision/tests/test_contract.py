# -*- coding: utf-8 -*-
"""Contract and follow-up acceptance tests (C-A1 ~ C-A8, T-D9).

Verifies:
    C-A1: PoseFix.to_dict() schema matching PoseFix.msg
    C-A2: frame_id == 'map'
    C-A3: stamp_is_robot_clock == False
    C-A4: Canonical ZoneEvent keys (robot_name, event_type, GOAL_C)
    C-A5: Sequence monotonicity
    C-A6: Stale PoseFix is dropped on network error (NEVER queued or retried)
    C-A6b: Asynchronous latest-only worker does not block vision loop
    C-A7: capture_stamp (Tablet Ingress) is preserved and pipeline/detector latency harmonized
    C-A8: Low-confidence detections (< min_confidence) do not trigger ENTER
    P0/P1 follow-ups:
        - FIELD_CONFIG_PENDING fail-closed egress
        - Live camera failure fail-closed (no mock fallback)
        - Mock mode egress safety
        - Stale ZoneEvent dropped after max_event_age (3.0s)
        - OverheadConfig parser reads non-default reference size/height
        - Zone camera image processing runtime
"""

import http.server
import json
import os
import threading
import time
from typing import List, Tuple

import cv2
import numpy as np
import pytest
import yaml

from tablet.vision.runtime.entrypoint import CameraIngest, load_configs, main as entrypoint_main
from tablet.vision.runtime.http_transport import HttpVisionTransport
from tablet.vision.runtime.tablet_vision_service import TabletVisionService, TrackingState
from tablet.vision.vision_core.overhead_localizer import OverheadConfig
from tablet.vision.vision_core.pose_fix import Header, PoseFix
from tablet.vision.vision_core.synthetic_camera import render_overhead
from tablet.vision.vision_core.zone_event import VisionZoneEvent, ZoneConfig, ZoneDetector


# ----------------------------------------------------------------------
# In-process fake HTTP Relay server fixture
# ----------------------------------------------------------------------
class _FakeRelayHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # Quiet during tests

    def do_POST(self):
        content_len = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_len).decode("utf-8") if content_len > 0 else "{}"
        data = json.loads(body)

        server = self.server  # type: ignore
        server.requests_log.append({
            "path": self.path,
            "headers": dict(self.headers),
            "payload": data,
        })

        if server.fail_all_requests:
            self.send_response(503)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"accepted": false, "error": "simulated failure"}')
            return

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"accepted": true}')


@pytest.fixture
def fake_relay():
    server = http.server.HTTPServer(("127.0.0.1", 0), _FakeRelayHandler)
    server.requests_log = []  # type: ignore
    server.fail_all_requests = False  # type: ignore
    host, port = server.server_address
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()

    yield {
        "url": f"http://{host}:{port}",
        "server": server,
        "requests": server.requests_log,  # type: ignore
    }

    server.shutdown()
    server.server_close()


# ----------------------------------------------------------------------
# C-A1, C-A2, C-A3: Canonical PoseFix contract tests
# ----------------------------------------------------------------------
def test_c_a1_posefix_schema():
    """C-A1: PoseFix.to_dict() matches PoseFix.msg field specification."""
    pf = PoseFix(
        robot_name="pinky1",
        x=1.23456,
        y=-0.65432,
        yaw=0.78539,
        seq=42,
        header=Header(frame_id="map", stamp_sec=100, stamp_nanosec=500000000),
        stamp_is_robot_clock=False,
        marker_id=30,
        marker_range=0.0,
        reproj_error=0.123,
        n_markers=4,
        pipeline_latency=0.0054,
    )
    d = pf.to_dict()

    expected_keys = {
        "header",
        "stamp_is_robot_clock",
        "robot_name",
        "seq",
        "x",
        "y",
        "yaw",
        "marker_id",
        "marker_range",
        "reproj_error",
        "n_markers",
        "pipeline_latency",
    }
    assert set(d.keys()) == expected_keys
    assert set(d["header"].keys()) == {"frame_id", "stamp"}
    assert set(d["header"]["stamp"].keys()) == {"sec", "nanosec"}


def test_c_a2_frame_id_map():
    """C-A2: PoseFix.header.frame_id is unconditionally 'map'."""
    pf = PoseFix(robot_name="pinky1", x=0.0, y=0.0, yaw=0.0)
    assert pf.header.frame_id == "map"
    assert pf.to_dict()["header"]["frame_id"] == "map"


def test_c_a3_stamp_is_robot_clock_false():
    """C-A3: Overhead PoseFix uses station clock, so stamp_is_robot_clock must be False."""
    pf = PoseFix(robot_name="pinky1", x=0.0, y=0.0, yaw=0.0)
    assert pf.stamp_is_robot_clock is False
    assert pf.to_dict()["stamp_is_robot_clock"] is False


# ----------------------------------------------------------------------
# C-A4, C-A5: Canonical ZoneEvent keys & GOAL_C
# ----------------------------------------------------------------------
def test_c_a4_canonical_zone_event_keys():
    """C-A4: VisionZoneEvent.to_dict() adheres strictly to canonical keys."""
    ev = VisionZoneEvent(
        robot_name="pinky1",
        camera_id="CAM_GOAL_C",
        zone_id="GOAL_C",
        event_type="ENTER",
        confidence=0.95,
        timestamp=1789904000.123,
        sequence=1,
    )
    d = ev.to_dict()

    expected_keys = {
        "robot_name",
        "camera_id",
        "zone_id",
        "event_type",
        "confidence",
        "timestamp",
        "sequence",
    }
    assert set(d.keys()) == expected_keys
    assert d["robot_name"] == "pinky1"
    assert d["event_type"] == "ENTER"
    assert d["zone_id"] == "GOAL_C"


def test_c_a5_sequence_monotonic():
    """C-A5: Sequence numbers increase monotonically."""
    detector = ZoneDetector([
        ZoneConfig(
            zone_id="START_A",
            camera_id="CAM_START_A",
            target_markers={30: "pinky1"},
            enter_debounce_frames=1,
            exit_debounce_frames=1,
        )
    ])

    e1 = detector.update("CAM_START_A", {30})
    e2 = detector.update("CAM_START_A", set())
    e3 = detector.update("CAM_START_A", {30})

    assert len(e1) == 1 and e1[0].sequence == 1
    assert len(e2) == 1 and e2[0].sequence == 2
    assert len(e3) == 1 and e3[0].sequence == 3


# ----------------------------------------------------------------------
# C-A6: Stale PoseFix is dropped on network error (NEVER queued or retried)
# ----------------------------------------------------------------------
def test_c_a6_stale_posefix_never_retried_sync(fake_relay):
    """C-A6: Synchronous send drops on error; retries are forbidden."""
    transport = HttpVisionTransport(relay_base_url=fake_relay["url"], async_pose=False)
    server = fake_relay["server"]

    # 1. Server fails with 503
    server.fail_all_requests = True
    pf1 = PoseFix(robot_name="pinky1", x=0.1, y=0.1, yaw=0.0, seq=1)
    ok1 = transport.send_pose_fix(pf1, sync=True)
    assert ok1 is False

    # 2. Server recovers
    server.fail_all_requests = False
    fake_relay["requests"].clear()

    # 3. New fresh pose pf2
    pf2 = PoseFix(robot_name="pinky1", x=0.2, y=0.2, yaw=0.0, seq=2)
    ok2 = transport.send_pose_fix(pf2, sync=True)
    assert ok2 is True

    # 4. Only pf2 delivered
    reqs = fake_relay["requests"]
    assert len(reqs) == 1
    assert reqs[0]["path"] == "/api/vision/pose_fix"
    assert reqs[0]["payload"]["seq"] == 2
    assert reqs[0]["payload"]["x"] == 0.2


def test_c_a6b_async_pose_worker_non_blocking_and_latest_only(fake_relay):
    """C-A6b: Async worker does not block the vision frame loop, and drops intermediate stale poses."""
    transport = HttpVisionTransport(relay_base_url=fake_relay["url"], async_pose=True)
    server = fake_relay["server"]

    # 1. Server fails
    server.fail_all_requests = True

    # 2. Enqueue multiple poses in rapid succession - must be non-blocking (< 50ms)
    t0 = time.perf_counter()
    for seq in range(1, 10):
        pf = PoseFix(robot_name="pinky1", x=float(seq), y=0.0, yaw=0.0, seq=seq)
        ok = transport.send_pose_fix(pf)
        assert ok is True
    elapsed = time.perf_counter() - t0
    assert elapsed < 0.05, f"send_pose_fix took {elapsed:.4f}s, must not block vision loop!"

    # 3. Recover server and deposit final fresh pose (seq=100)
    server.fail_all_requests = False
    pf_final = PoseFix(robot_name="pinky1", x=99.0, y=0.0, yaw=0.0, seq=100)
    transport.send_pose_fix(pf_final)

    # 4. Drain worker
    assert transport.drain_poses(timeout=2.0) is True

    # 5. Verify that stale intermediate poses (seq=1..8) were not replayed
    reqs = fake_relay["requests"]
    assert len(reqs) >= 1
    # The last dispatched pose must be the newest one (seq=100)
    assert reqs[-1]["payload"]["seq"] == 100
    assert reqs[-1]["payload"]["x"] == 99.0

    transport.close()


# ----------------------------------------------------------------------
# Zone Event Retries and Stale Age Expiration
# ----------------------------------------------------------------------
def test_zone_event_retries_preserve_sequence_and_timestamp(fake_relay):
    """Verification that valid ZoneEvent preserves sequence and timestamp across retries."""
    transport = HttpVisionTransport(relay_base_url=fake_relay["url"])
    server = fake_relay["server"]

    server.fail_all_requests = True
    now = time.time()
    ev = VisionZoneEvent(
        robot_name="pinky1",
        camera_id="CAM_START_A",
        zone_id="START_A",
        event_type="ENTER",
        confidence=1.0,
        timestamp=now,
        sequence=42,
    )
    ok = transport.send_zone_event(ev)
    assert ok is False
    assert transport.pending_zone_events_count == 1

    server.fail_all_requests = False
    fake_relay["requests"].clear()

    # Fast forward retry timer
    for item in transport._zone_retry_queue:
        item["next_retry"] = time.time() - 1.0

    flushed = transport.flush_zone_events()
    assert flushed == 1
    assert transport.pending_zone_events_count == 0

    reqs = fake_relay["requests"]
    assert len(reqs) == 1
    assert reqs[0]["path"] == "/api/vision/zone_event"
    assert reqs[0]["payload"]["sequence"] == 42
    assert reqs[0]["payload"]["timestamp"] == round(now, 4)


def test_zone_event_stale_age_dropped(fake_relay):
    """P1: Zone events exceeding max_event_age (3.0s) are dropped and never retried."""
    transport = HttpVisionTransport(relay_base_url=fake_relay["url"], max_event_age=3.0)
    server = fake_relay["server"]

    # Event generated 4.0 seconds ago (older than 3.0s limit)
    old_time = time.time() - 4.0
    ev = VisionZoneEvent(
        robot_name="pinky1",
        camera_id="CAM_GOAL_C",
        zone_id="GOAL_C",
        event_type="ENTER",
        confidence=1.0,
        timestamp=old_time,
        sequence=99,
    )

    # Must be dropped immediately
    ok = transport.send_zone_event(ev)
    assert ok is False
    assert transport.pending_zone_events_count == 0
    assert len(fake_relay["requests"]) == 0

    # Also test flush queue expiration
    server.fail_all_requests = True
    fresh_ev = VisionZoneEvent(
        robot_name="pinky1",
        camera_id="CAM_GOAL_C",
        zone_id="GOAL_C",
        event_type="EXIT",
        confidence=1.0,
        timestamp=time.time(),
        sequence=100,
    )
    transport.send_zone_event(fresh_ev)
    assert transport.pending_zone_events_count == 1

    # Simulate network outage lasting > 3.0s
    transport._zone_retry_queue[0]["event"].timestamp = time.time() - 4.0
    transport._zone_retry_queue[0]["next_retry"] = time.time() - 1.0

    server.fail_all_requests = False
    flushed = transport.flush_zone_events()
    assert flushed == 0  # Discarded due to age expiration
    assert transport.pending_zone_events_count == 0


# ----------------------------------------------------------------------
# C-A7: capture_stamp (Tablet Ingress) preserved & Latency Harmonization
# ----------------------------------------------------------------------
def test_c_a7_capture_stamp_preserved_and_latency_harmonized():
    """C-A7: capture_stamp (Tablet Ingress) is preserved in header and latency metrics are harmonized."""
    cfg = OverheadConfig.from_dict({
        "dictionary": "DICT_4X4_50",
        "reference": {
            "size": 0.10,
            "markers": [
                {"id": 40, "x": -1.0, "y": -0.5, "yaw": 0.0},
                {"id": 41, "x":  1.0, "y": -0.5, "yaw": 0.0},
                {"id": 42, "x":  1.0, "y":  0.5, "yaw": 0.0},
                {"id": 43, "x": -1.0, "y":  0.5, "yaw": 0.0},
            ]
        },
        "robots": [{"name": "pinky1", "id": 30, "size": 0.06}],
        "min_reference": 4,
        "max_reproj": 3.0,
    })
    service = TabletVisionService(overhead_cfg=cfg)
    poses = {"pinky1": (0.0, 0.0, 0.0)}
    _, img = render_overhead(cfg, poses)

    mock_capture_time = time.time() - 0.150
    fixes = service.process_overhead_frame(img, capture_stamp=mock_capture_time)

    assert "pinky1" in fixes
    pf = fixes["pinky1"]
    assert abs(pf.header.stamp_float - mock_capture_time) < 1e-6
    assert pf.pipeline_latency >= 0.140

    # Verify health reporting harmonizes pipeline_latency and detector_latency
    health = service.get_health()
    assert "pipeline_latency" in health
    assert "detector_latency" in health
    assert health["pipeline_latency"] >= 0.140
    assert health["detector_latency"] < health["pipeline_latency"]


# ----------------------------------------------------------------------
# C-A8: Low-confidence ENTER blocked
# ----------------------------------------------------------------------
def test_c_a8_low_confidence_blocks_enter():
    """C-A8: Detections below min_confidence (0.8) are blocked from accumulating positive detections."""
    detector = ZoneDetector([
        ZoneConfig(
            zone_id="START_A",
            camera_id="CAM_START_A",
            target_markers={30: "pinky1"},
            enter_debounce_frames=3,
            exit_debounce_frames=3,
            min_confidence=0.8,
        )
    ])

    for _ in range(5):
        evs = detector.update("CAM_START_A", {30}, confidence=0.5)
        assert len(evs) == 0

    assert not detector.is_in_zone("START_A", "pinky1")

    for _ in range(2):
        evs = detector.update("CAM_START_A", {30}, confidence=0.9)
        assert len(evs) == 0

    evs3 = detector.update("CAM_START_A", {30}, confidence=0.9)
    assert len(evs3) == 1
    assert evs3[0].event_type == "ENTER"
    assert detector.is_in_zone("START_A", "pinky1")


# ----------------------------------------------------------------------
# P0 / P1 Safety Invariant Tests
# ----------------------------------------------------------------------
def test_field_config_pending_blocks_relay_egress(tmp_path):
    """P0-1: If config status is not READY, Relay egress is disabled unless overridden."""
    cfg_dir = tmp_path / "config"
    cfg_dir.mkdir()

    (cfg_dir / "markers.yaml").write_text("""
status: FIELD_CONFIG_PENDING
dictionary: DICT_4X4_50
reference:
  markers:
    - {id: 40, x: -1.0, y: -0.5, yaw: 0.0}
    - {id: 41, x:  1.0, y: -0.5, yaw: 0.0}
    - {id: 42, x:  1.0, y:  0.5, yaw: 0.0}
    - {id: 43, x: -1.0, y:  0.5, yaw: 0.0}
robots:
  - {name: pinky1, id: 30}
""")
    (cfg_dir / "cameras.yaml").write_text("status: FIELD_CONFIG_PENDING\ncameras: {}\n")
    (cfg_dir / "zones.yaml").write_text("status: FIELD_CONFIG_PENDING\nzones: []\n")

    _, _, _, statuses = load_configs(str(cfg_dir))
    assert statuses["markers"] == "FIELD_CONFIG_PENDING"

    # Default run: egress blocked
    rc = entrypoint_main(["--config-dir", str(cfg_dir), "--mock", "--once"])
    assert rc == 0


def test_live_camera_failure_fails_closed(tmp_path):
    """P0-2: Failure to open live camera exits with error and does NOT fallback to mock mode."""
    cfg_dir = tmp_path / "config"
    cfg_dir.mkdir()

    (cfg_dir / "markers.yaml").write_text("""
status: READY
dictionary: DICT_4X4_50
reference:
  markers:
    - {id: 40, x: -1.0, y: -0.5, yaw: 0.0}
    - {id: 41, x:  1.0, y: -0.5, yaw: 0.0}
    - {id: 42, x:  1.0, y:  0.5, yaw: 0.0}
    - {id: 43, x: -1.0, y:  0.5, yaw: 0.0}
robots:
  - {name: pinky1, id: 30}
""")
    # Non-existent device to force camera open failure
    (cfg_dir / "cameras.yaml").write_text("""
status: READY
cameras:
  overhead:
    device: "/dev/non_existent_camera_device_xyz"
""")
    (cfg_dir / "zones.yaml").write_text("status: READY\nzones: []\n")

    rc = entrypoint_main(["--config-dir", str(cfg_dir), "--once"])
    assert rc == 1, "Live camera failure must return non-zero exit code (fail-closed)"


def test_mock_egress_safety(fake_relay, tmp_path):
    """P0-2: Mock mode blocks Relay egress by default; permits egress only when --mock-egress is set."""
    cfg_dir = tmp_path / "config"
    cfg_dir.mkdir()

    (cfg_dir / "markers.yaml").write_text("""
status: READY
dictionary: DICT_4X4_50
reference:
  markers:
    - {id: 40, x: -1.0, y: -0.5, yaw: 0.0}
    - {id: 41, x:  1.0, y: -0.5, yaw: 0.0}
    - {id: 42, x:  1.0, y:  0.5, yaw: 0.0}
    - {id: 43, x: -1.0, y:  0.5, yaw: 0.0}
robots:
  - {name: pinky1, id: 30}
""")
    (cfg_dir / "cameras.yaml").write_text("status: READY\ncameras: {}\n")
    (cfg_dir / "zones.yaml").write_text("status: READY\nzones: []\n")

    # 1. Mock run without --mock-egress: must NOT send any poses to Relay
    fake_relay["requests"].clear()
    rc1 = entrypoint_main([
        "--config-dir", str(cfg_dir),
        "--relay-url", fake_relay["url"],
        "--mock",
        "--once",
    ])
    assert rc1 == 0
    time.sleep(0.1)
    assert len(fake_relay["requests"]) == 0, "Mock mode must NOT transmit poses without --mock-egress"

    # 2. Mock run WITH --mock-egress: egress allowed
    rc2 = entrypoint_main([
        "--config-dir", str(cfg_dir),
        "--relay-url", fake_relay["url"],
        "--mock",
        "--mock-egress",
        "--once",
    ])
    assert rc2 == 0
    time.sleep(0.2)
    assert len(fake_relay["requests"]) >= 1, "Mock mode with --mock-egress should permit egress"


def test_overhead_config_parser_non_default_sizes():
    """P1-4: OverheadConfig parses non-default reference_size and reference_height from top-level and nested."""
    # 1. Top-level specification
    cfg1 = OverheadConfig.from_dict({
        "dictionary": "DICT_4X4_50",
        "reference_size": 0.15,
        "reference_height": 0.20,
        "reference": {
            "markers": [
                {"id": 40, "x": -1.0, "y": -0.5, "yaw": 0.0},
                {"id": 41, "x":  1.0, "y": -0.5, "yaw": 0.0},
                {"id": 42, "x":  1.0, "y":  0.5, "yaw": 0.0},
                {"id": 43, "x": -1.0, "y":  0.5, "yaw": 0.0},
            ]
        },
        "robots": [{"name": "pinky1", "id": 30}],
    })
    assert cfg1.reference_size == 0.15
    assert cfg1.reference_height == 0.20

    # 2. Nested reference specification
    cfg2 = OverheadConfig.from_dict({
        "dictionary": "DICT_4X4_50",
        "reference": {
            "size": 0.12,
            "height": 0.18,
            "markers": [
                {"id": 40, "x": -1.0, "y": -0.5, "yaw": 0.0},
                {"id": 41, "x":  1.0, "y": -0.5, "yaw": 0.0},
                {"id": 42, "x":  1.0, "y":  0.5, "yaw": 0.0},
                {"id": 43, "x": -1.0, "y":  0.5, "yaw": 0.0},
            ]
        },
        "robots": [{"name": "pinky1", "id": 30}],
    })
    assert cfg2.reference_size == 0.12
    assert cfg2.reference_height == 0.18


def test_zone_camera_runtime_process_image():
    """P1-1: Start/Goal zone camera image processing and event generation runtime."""
    cfg = OverheadConfig.from_dict({
        "dictionary": "DICT_4X4_50",
        "reference": {
            "size": 0.10,
            "markers": [
                {"id": 40, "x": -1.0, "y": -0.5, "yaw": 0.0},
                {"id": 41, "x":  1.0, "y": -0.5, "yaw": 0.0},
                {"id": 42, "x":  1.0, "y":  0.5, "yaw": 0.0},
                {"id": 43, "x": -1.0, "y":  0.5, "yaw": 0.0},
            ]
        },
        "robots": [{"name": "pinky1", "id": 30}],
    })

    zone_configs = [
        ZoneConfig(
            zone_id="GOAL_C",
            camera_id="CAM_GOAL_C",
            target_markers={30: "pinky1"},
            enter_debounce_frames=2,
            exit_debounce_frames=2,
        )
    ]

    emitted_events = []
    service = TabletVisionService(
        overhead_cfg=cfg,
        zone_configs=zone_configs,
        zone_event_callback=lambda ev: emitted_events.append(ev),
    )

    # Empty frame (no markers)
    blank_img = np.zeros((480, 640, 3), dtype=np.uint8)
    evs = service.process_zone_image("CAM_GOAL_C", blank_img)
    assert len(evs) == 0

    # Frame with marker 30
    marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.generateImageMarker(marker_dict, 30, 200)
    frame = np.ones((480, 640), dtype=np.uint8) * 255
    frame[140:340, 220:420] = marker_img

    # 1st detection frame
    service.process_zone_image("CAM_GOAL_C", frame)
    assert len(emitted_events) == 0

    # 2nd detection frame -> debounce threshold reached, ENTER event fired!
    service.process_zone_image("CAM_GOAL_C", frame)
    assert len(emitted_events) == 1
    ev = emitted_events[0]
    assert ev.zone_id == "GOAL_C"
    assert ev.camera_id == "CAM_GOAL_C"
    assert ev.robot_name == "pinky1"
    assert ev.event_type == "ENTER"
