# -*- coding: utf-8 -*-
"""Robot pose recovery & occlusion acceptance tests (T-A4, T-A5, T-A7, T-A8)."""

import math
import numpy as np
import pytest

from tablet.vision.vision_core.overhead_localizer import OverheadConfig, OverheadLocalizer
from tablet.vision.vision_core.synthetic_camera import render_overhead


@pytest.fixture
def base_config():
    d = {
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
            {"name": "pinky1", "id": 30, "size": 0.06, "yaw_offset": 0.0, "offset_x": 0.0},
            {"name": "pinky2", "id": 31, "size": 0.06, "yaw_offset": 0.0, "offset_x": 0.0},
        ],
        "min_reference": 4,
        "max_reproj": 3.0,
    }
    return OverheadConfig.from_dict(d)


def _ang_diff(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def test_t_a4_marker_30_pinky1(base_config):
    """T-A4: Marker 30 specifically maps to robot pinky1 with accurate (x, y)."""
    target_pos = (-0.35, 0.15, 0.0)
    poses = {"pinky1": target_pos}
    _, img = render_overhead(base_config, poses)
    loc = OverheadLocalizer(base_config)
    fixes = loc.update(img)

    assert "pinky1" in fixes
    f = fixes["pinky1"]
    assert f.marker_id == 30
    assert math.hypot(f.x - target_pos[0], f.y - target_pos[1]) < 0.01  # < 1cm accuracy
    assert _ang_diff(f.yaw, target_pos[2]) < math.radians(2.0)


def test_t_a5_marker_31_pinky2(base_config):
    """T-A5: Marker 31 specifically maps to robot pinky2 with accurate (x, y)."""
    target_pos = (0.42, -0.18, 0.0)
    poses = {"pinky2": target_pos}
    _, img = render_overhead(base_config, poses)
    loc = OverheadLocalizer(base_config)
    fixes = loc.update(img)

    assert "pinky2" in fixes
    f = fixes["pinky2"]
    assert f.marker_id == 31
    assert math.hypot(f.x - target_pos[0], f.y - target_pos[1]) < 0.01
    assert _ang_diff(f.yaw, target_pos[2]) < math.radians(2.0)


@pytest.mark.parametrize("target_yaw", [0.0, math.pi / 4, math.pi / 2, -math.pi / 3, math.pi])
def test_t_a7_marker_rotation_expected_yaw(base_config, target_yaw):
    """T-A7: Marker rotation correctly tracks expected robot heading yaw."""
    poses = {"pinky1": (0.1, 0.1, target_yaw)}
    _, img = render_overhead(base_config, poses)
    loc = OverheadLocalizer(base_config)
    fixes = loc.update(img)

    assert "pinky1" in fixes
    f = fixes["pinky1"]
    # 6cm marker with synthetic rendering allows 2.5 degrees precision
    assert _ang_diff(f.yaw, target_yaw) < math.radians(2.5)


def test_t_a8_marker_missing_no_stale_posefix(base_config):
    """T-A8: When a robot marker is missing/occluded, stale PoseFix is NOT republished."""
    import cv2

    poses = {"pinky1": (0.0, 0.0, 0.0), "pinky2": (0.5, -0.2, math.pi / 2)}
    _, img1 = render_overhead(base_config, poses)
    loc = OverheadLocalizer(base_config)

    # Frame 1: both robots visible
    fixes1 = loc.update(img1)
    assert "pinky1" in fixes1
    assert "pinky2" in fixes1

    # Frame 2: pinky1 is occluded by covering marker 30
    _, img2 = render_overhead(base_config, poses)
    dets2 = loc.detect(img2)
    corners30 = dets2[30].astype(np.int32)
    cv2.fillPoly(img2, [corners30], (110, 110, 110))

    fixes2 = loc.update(img2)

    # In Frame 2: pinky1 MUST NOT be present in fixes output
    assert "pinky1" not in fixes2
    # pinky2 remains tracked
    assert "pinky2" in fixes2
