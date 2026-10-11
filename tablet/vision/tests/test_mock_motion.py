# -*- coding: utf-8 -*-
"""T-7 moving mock: start poses on team road-graph nodes, pinky1 advances BL -> TR.

The mock only moves the *rendered* pose; these tests run the real render -> detect ->
PoseFix path with the SoT config (tablet/vision/config) and check what comes out.
"""

import math
import os

import pytest

from tablet.vision.runtime.entrypoint import load_configs
from tablet.vision.runtime.tablet_vision_service import TabletVisionService
from tablet.vision.vision_core.mock_motion import MockPoses, PathMotion, RoadGraph
from tablet.vision.vision_core.synthetic_camera import render_overhead

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


@pytest.fixture(scope="module")
def graph():
    return RoadGraph.load()


def test_default_poses_are_start_nodes(graph):
    poses = MockPoses(graph).poses(10.0)
    assert poses["pinky1"] == (*graph.nodes["BL"], 0.0)
    assert poses["pinky2"] == (*graph.nodes["BR"], 0.0)


def test_path_runs_bl_to_tr_through_junction(graph):
    pts = graph.path(["BL", "TR"])
    assert pts[0] == graph.nodes["BL"] and pts[-1] == graph.nodes["TR"]
    assert graph.nodes["J"] in pts
    assert len(pts) == len(set(pts))            # shared edge endpoints are not duplicated


def test_unknown_node_is_rejected(graph):
    with pytest.raises(KeyError):
        graph.path(["BL", "NOWHERE"])


def test_motion_is_constant_speed_and_holds_at_goal():
    m = PathMotion([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], 0.5)
    assert m.pose_at(1.0) == pytest.approx((0.5, 0.0, 0.0))
    assert m.pose_at(3.0) == pytest.approx((1.0, 0.5, math.pi / 2))
    assert m.pose_at(99.0)[:2] == pytest.approx((1.0, 1.0))
    assert m.done(4.0) and not m.done(3.9)


def test_zero_speed_keeps_pinky1_at_start(graph):
    assert MockPoses(graph, speed_m_s=0.0).poses(50.0)["pinky1"][:2] == graph.nodes["BL"]


def test_moving_mock_through_real_pipeline(graph):
    """N frames of the moving mock -> PoseFix between BL and TR, seq increasing."""
    cfg, _, _, _ = load_configs(CONFIG_DIR)
    fixes = []
    svc = TabletVisionService(overhead_cfg=cfg, zone_configs=[],
                              pose_callback=lambda p: fixes.append(p))
    mock = MockPoses(graph, speed_m_s=0.15)
    total_s = mock.motion.length / 0.15
    n = 25
    for i in range(n + 1):
        t = total_s * i / n
        _, img = render_overhead(cfg, mock.poses(t))
        svc.process_overhead_frame(img, capture_stamp=1000.0 + t)

    p1 = [f for f in fixes if f.robot_name == "pinky1"]
    assert len(p1) >= int(0.9 * (n + 1))        # renderer can miss an odd frame
    seqs = [f.seq for f in p1]
    assert all(b > a for a, b in zip(seqs, seqs[1:]))

    (ax, ay), (gx, gy) = graph.nodes["BL"], graph.nodes["TR"]
    for f in p1:
        assert ax - 0.05 <= f.x <= gx + 0.05 and ay - 0.05 <= f.y <= gy + 0.40
    assert p1[0].x == pytest.approx(ax, abs=0.03) and p1[0].y == pytest.approx(ay, abs=0.03)
    assert p1[-1].x == pytest.approx(gx, abs=0.03) and p1[-1].y == pytest.approx(gy, abs=0.03)
    xs = [f.x for f in p1]
    assert xs[-1] > xs[0] + 1.5                 # it actually travelled

    p2 = [f for f in fixes if f.robot_name == "pinky2"]
    assert p2 and all(f.x == pytest.approx(graph.nodes["BR"][0], abs=0.03) for f in p2)
