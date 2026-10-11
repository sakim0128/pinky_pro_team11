# -*- coding: utf-8 -*-
"""Zone event generator acceptance tests (T-A9, T-A10)."""

import pytest

from tablet.vision.vision_core.zone_event import VisionZoneEvent, ZoneConfig, ZoneDetector


@pytest.fixture
def detector():
    configs = [
        ZoneConfig(
            zone_id="START_A",
            camera_id="CAM_START_A",
            target_markers={30: "pinky1"},
            enter_debounce_frames=3,
            exit_debounce_frames=3,
        ),
        ZoneConfig(
            zone_id="GOAL_C",
            camera_id="CAM_GOAL_C",
            target_markers={30: "pinky1", 31: "pinky2"},
            enter_debounce_frames=3,
            exit_debounce_frames=3,
        ),
    ]
    return ZoneDetector(configs)


def test_t_a9_start_zone_debounce(detector):
    """T-A9: Start Zone enter/exit transitions require consecutive debounce frames."""
    # Frame 1: marker 30 detected once -> no event yet
    evs1 = detector.update("CAM_START_A", {30})
    assert len(evs1) == 0
    assert not detector.is_in_zone("START_A", "pinky1")

    # Frame 2: marker 30 detected twice -> still debouncing
    evs2 = detector.update("CAM_START_A", {30})
    assert len(evs2) == 0
    assert not detector.is_in_zone("START_A", "pinky1")

    # Frame 3: marker 30 detected 3rd consecutive time -> ENTER event emitted!
    evs3 = detector.update("CAM_START_A", {30})
    assert len(evs3) == 1
    assert evs3[0].event_type == "ENTER"
    assert evs3[0].zone_id == "START_A"
    assert evs3[0].robot_name == "pinky1"
    assert detector.is_in_zone("START_A", "pinky1")

    # Frame 4: marker 30 still detected -> duplicate ENTER is suppressed
    evs4 = detector.update("CAM_START_A", {30})
    assert len(evs4) == 0

    # Frame 5: marker 30 missed for 1 frame -> debounce prevents immediate exit
    evs5 = detector.update("CAM_START_A", set())
    assert len(evs5) == 0
    assert detector.is_in_zone("START_A", "pinky1")

    # Frame 6: marker 30 missed 2nd time -> still present
    evs6 = detector.update("CAM_START_A", set())
    assert len(evs6) == 0

    # Frame 7: marker 30 missed 3rd consecutive time -> EXIT event emitted!
    evs7 = detector.update("CAM_START_A", set())
    assert len(evs7) == 1
    assert evs7[0].event_type == "EXIT"
    assert evs7[0].zone_id == "START_A"
    assert not detector.is_in_zone("START_A", "pinky1")


def test_t_a10_goal_zone_event(detector):
    """T-A10: Goal Zone event detection with multi-robot support."""
    # Feed 3 consecutive frames with both pinky1 (30) and pinky2 (31)
    for _ in range(2):
        detector.update("CAM_GOAL_C", {30, 31})

    evs = detector.update("CAM_GOAL_C", {30, 31})
    assert len(evs) == 2
    names = {e.robot_name for e in evs}
    assert names == {"pinky1", "pinky2"}
    for e in evs:
        assert e.event_type == "ENTER"
        assert e.zone_id == "GOAL_C"
        assert e.camera_id == "CAM_GOAL_C"

    assert detector.is_in_zone("GOAL_C", "pinky1")
    assert detector.is_in_zone("GOAL_C", "pinky2")
