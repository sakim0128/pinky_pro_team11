# -*- coding: utf-8 -*-
"""Tablet Vision Runtime Service end-to-end integration tests."""

import math
import numpy as np
import pytest

from tablet.vision.runtime.tablet_vision_service import TabletVisionService, TrackingState
from tablet.vision.vision_core.overhead_localizer import OverheadConfig
from tablet.vision.vision_core.pose_fix import PoseFix
from tablet.vision.vision_core.synthetic_camera import render_overhead
from tablet.vision.vision_core.zone_event import VisionZoneEvent, ZoneConfig


@pytest.fixture
def service():
    cfg = OverheadConfig.from_dict({
        "dictionary": "DICT_4X4_50",
        "reference": {
            "size": 0.10,
            "height": 0.10,
            "markers": [
                {"id": 40, "x": -1.00, "y": -0.50, "yaw": 0.0},
                {"id": 41, "x":  1.00, "y": -0.50, "yaw": 0.0},
                {"id": 42, "x":  1.00, "y":  0.50, "yaw": 0.0},
                {"id": 43, "x": -1.00, "y":  0.50, "yaw": 0.0},
            ]
        },
        "robots": [
            {"name": "pinky1", "id": 30, "size": 0.06},
            {"name": "pinky2", "id": 31, "size": 0.06},
        ],
        "min_reference": 4,
        "max_reproj": 3.0,
    })

    zone_cfgs = [
        ZoneConfig(
            zone_id="START_A",
            camera_id="CAM_START_A",
            target_markers={30: "pinky1"},
            enter_debounce_frames=2,
            exit_debounce_frames=2,
        )
    ]
    return TabletVisionService(overhead_cfg=cfg, zone_configs=zone_cfgs)


def test_service_posefix_production_and_serialization(service):
    """Verify service produces canonical PoseFix and serializes correctly for Relay."""
    received_poses = []
    service.pose_callback = lambda p: received_poses.append(p)

    poses = {"pinky1": (0.2, -0.1, 0.5), "pinky2": (-0.4, 0.3, -1.2)}
    _, img = render_overhead(service.overhead_cfg, poses)

    out = service.process_overhead_frame(img)
    assert "pinky1" in out
    assert "pinky2" in out
    assert len(received_poses) == 2

    pf1: PoseFix = out["pinky1"]
    assert pf1.robot_name == "pinky1"
    assert pf1.header.frame_id == "map"
    assert pf1.stamp_is_robot_clock is False
    assert pf1.marker_id == 30
    assert pf1.seq == 1
    assert pf1.pipeline_latency >= 0.0

    # Serialization test
    d = pf1.to_dict()
    assert d["robot_name"] == "pinky1"
    assert "header" in d
    assert d["stamp_is_robot_clock"] is False

    relay_payload = pf1.to_relay_dict()
    assert relay_payload["robotId"] == "pinky1"
    assert "computedAtMs" in relay_payload
    assert "x" in relay_payload
    assert "y" in relay_payload
    assert "yaw" in relay_payload
    assert "markerId" in relay_payload


def test_service_health_reporting_and_occlusion_transitions(service):
    """Verify service health metrics and state transitions on occlusion."""
    import cv2

    poses = {"pinky1": (0.0, 0.0, 0.0), "pinky2": (0.5, -0.2, math.pi / 2)}
    _, img = render_overhead(service.overhead_cfg, poses)

    # Frame 1: Normal visibility
    service.process_overhead_frame(img)
    h1 = service.get_health()
    assert h1["camera_connected"] is True
    assert h1["homography_valid"] is True
    assert h1["reference_seen"] == 4
    assert h1["robot1_tracking"] == TrackingState.TRACKED
    assert h1["robot2_tracking"] == TrackingState.TRACKED

    # Frame 2: Occlude pinky1
    _, img2 = render_overhead(service.overhead_cfg, poses)
    dets = service.localizer.detect(img2)
    corners = dets[30].astype(np.int32)
    cv2.fillPoly(img2, [corners], (110, 110, 110))

    out2 = service.process_overhead_frame(img2)
    assert "pinky1" not in out2
    assert "pinky2" in out2

    h2 = service.get_health()
    assert h2["robot1_tracking"] == TrackingState.LOST
    assert h2["robot2_tracking"] == TrackingState.TRACKED


def test_service_zone_event_integration(service):
    """Verify zone frame processing dispatches zone events."""
    events = []
    service.zone_event_callback = lambda e: events.append(e)

    # Frame 1: marker 30 detected once
    service.process_zone_frame("CAM_START_A", {30})
    assert len(events) == 0

    # Frame 2: marker 30 detected 2nd time -> enter debounce satisfied
    evs = service.process_zone_frame("CAM_START_A", {30})
    assert len(evs) == 1
    assert evs[0].event_type == "ENTER"
    assert evs[0].robot_name == "pinky1"
    assert len(events) == 1
