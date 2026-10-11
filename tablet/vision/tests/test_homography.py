# -*- coding: utf-8 -*-
"""Homography & reference marker acceptance tests (T-A1, T-A2, T-A3, T-A6)."""

import math
import cv2
import numpy as np
import pytest

from tablet.vision.vision_core.overhead_localizer import OverheadConfig, OverheadLocalizer
from tablet.vision.vision_core.synthetic_camera import render_overhead


@pytest.fixture
def base_config():
    """Standard field configuration with 4 reference markers."""
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
            {"name": "pinky1", "id": 30, "size": 0.06},
            {"name": "pinky2", "id": 31, "size": 0.06},
        ],
        "min_reference": 4,
        "max_reproj": 3.0,
    }
    return OverheadConfig.from_dict(d)


def test_t_a1_valid_homography(base_config):
    """T-A1: 4 reference markers produce valid Homography and low reprojection error."""
    poses = {"pinky1": (0.0, 0.0, 0.0), "pinky2": (0.5, 0.2, 0.5)}
    _, img = render_overhead(base_config, poses)
    loc = OverheadLocalizer(base_config)
    fixes = loc.update(img)

    st = loc.status()
    assert st["homography"] is True
    assert st["reference_seen"] == 4
    assert loc.H is not None
    assert loc.reproj < 1.0  # Synthetic reprojection error is typically < 0.3 px
    assert "pinky1" in fixes
    assert "pinky2" in fixes


def test_t_a2_insufficient_reference_rejects_new_homography(base_config):
    """T-A2: When fewer than 4 reference markers are seen, reject new H update and retain previous H."""
    import cv2

    poses = {"pinky1": (0.0, 0.0, 0.0)}
    _, img = render_overhead(base_config, poses)
    loc = OverheadLocalizer(base_config)

    # Initial frame with all 4 reference markers -> establishes H
    assert loc.update(img)
    initial_H = loc.H.copy()
    assert loc.ref_seen == 4

    # Second frame: occlude marker 40 and marker 41 (only 2 reference markers left)
    _, img2 = render_overhead(base_config, poses)
    dets = loc.detect(img2)
    for mid in (40, 41):
        corners = dets[mid].astype(np.int32)
        cv2.fillPoly(img2, [corners], (110, 110, 110))

    loc.update(img2)
    assert loc.ref_seen == 2
    # Homography must not be updated, keeping the initial valid matrix
    assert np.allclose(loc.H, initial_H)


def test_t_a3_reprojection_threshold_exceeded_rejects_fixes(base_config):
    """T-A3: When reprojection error exceeds max_reproj threshold, PoseFix is rejected."""
    # Create config with very strict threshold
    base_config.max_reproj = 0.01  # Absurdly small threshold to force rejection
    poses = {"pinky1": (0.0, 0.0, 0.0)}
    _, img = render_overhead(base_config, poses)
    loc = OverheadLocalizer(base_config)
    fixes = loc.update(img)

    assert loc.H is not None
    assert loc.reproj > base_config.max_reproj
    # Fixes must be completely empty due to threshold violation
    assert fixes == {}


def test_t_a6_known_pixel_to_map_coordinate(base_config):
    """T-A6: Known pixel coordinate maps to expected map coordinate."""
    poses = {"pinky1": (0.0, 0.0, 0.0)}
    to_px, img = render_overhead(base_config, poses)
    loc = OverheadLocalizer(base_config)
    loc.update(img)

    # Check mapping for known map center (0, 0)
    origin_map = np.array([[0.0, 0.0]])
    origin_px = cv2.perspectiveTransform(origin_map.reshape(-1, 1, 2), to_px).reshape(-1, 2)

    mapped_pt = loc.px_to_map(origin_px)
    assert math.hypot(mapped_pt[0, 0] - 0.0, mapped_pt[0, 1] - 0.0) < 0.015  # < 1.5 cm
