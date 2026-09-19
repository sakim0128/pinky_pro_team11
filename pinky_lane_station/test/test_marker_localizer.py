"""marker_localizer — 합성 마커를 CameraModel 로 그려 검출 → map 위치 복원 (ROS 불필요)."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_lane_station.marker_localizer import (  # noqa: E402
    CameraExtrinsics, CameraIntrinsics, MarkerLocalizer, MarkerMap, R_BASE_OPT0)
from pinky_lane_station.synthetic_camera import CameraModel, render_lane_frame, render_markers  # noqa: E402

CAM = CameraModel()
MM = MarkerMap(poses={0: (1.0, 0.5, 0.3), 1: (1.4, 0.7, -1.0), 2: (0.0, 0.0, 0.0)})


@pytest.fixture(scope='module')
def loc():
    return MarkerLocalizer(CameraIntrinsics.from_camera_model(CAM),
                           CameraExtrinsics.from_camera_model(CAM), MM)


def frame(robot):
    img = render_lane_frame(camera=CAM)
    return render_markers(img, robot, MM, CAM)


def test_extrinsics_match_synthetic_projection():
    """T_base_cam 규약이 CameraModel.project 와 같은지 — 바닥점을 두 방식으로 투영해 비교."""
    extr = CameraExtrinsics.from_camera_model(CAM)
    intr = CameraIntrinsics.from_camera_model(CAM)
    T_cam_base = np.linalg.inv(extr.T_base_cam)
    for X, Y in ((0.3, 0.0), (0.5, 0.1), (0.25, -0.08)):
        p = T_cam_base @ np.array([X, Y, 0.0, 1.0])
        u = intr.fx * p[0] / p[2] + intr.cx
        v = intr.fy * p[1] / p[2] + intr.cy
        su, sv = CAM.project(X, Y)
        assert abs(u - su) < 1e-6 and abs(v - sv) < 1e-6


def test_optical_axes_convention():
    # 광학 z(시선) → 로봇 +x, 광학 x(우) → 로봇 -y, 광학 y(아래) → 로봇 -z
    assert np.allclose(R_BASE_OPT0 @ np.array([0, 0, 1]), [1, 0, 0])
    assert np.allclose(R_BASE_OPT0 @ np.array([1, 0, 0]), [0, -1, 0])
    assert np.allclose(R_BASE_OPT0 @ np.array([0, 1, 0]), [0, 0, -1])


@pytest.mark.parametrize('robot', [(0.7, 0.5, 0.0), (0.6, 0.45, 0.2), (1.0, 0.15, math.pi / 2),
                                   (0.5, 0.5, 0.0), (1.55, 0.45, math.pi)])
def test_recovers_pose_within_2cm_2deg(loc, robot):
    fix = loc.localize(frame(robot))
    assert fix is not None
    assert math.hypot(fix.x - robot[0], fix.y - robot[1]) < 0.02
    assert abs(math.atan2(math.sin(fix.yaw - robot[2]), math.cos(fix.yaw - robot[2]))) < math.radians(2)
    assert fix.reproj < 1.5


def test_nearest_marker_is_chosen(loc):
    fix = loc.localize(frame((1.0, 0.15, math.pi / 2)))      # 0 은 0.35 m, 1 은 0.7 m 앞
    assert fix.n_markers == 2 and fix.marker_id == 0


def test_no_marker_gives_none(loc):
    assert loc.localize(render_lane_frame(camera=CAM)) is None


def test_unknown_marker_ignored():
    loc = MarkerLocalizer(CameraIntrinsics.from_camera_model(CAM),
                          CameraExtrinsics.from_camera_model(CAM), MarkerMap(poses={5: (9, 9, 0)}))
    assert loc.localize(frame((0.7, 0.5, 0.0))) is None


def test_range_gate(loc):
    far = MarkerLocalizer(CameraIntrinsics.from_camera_model(CAM),
                          CameraExtrinsics.from_camera_model(CAM), MM, max_range=0.2)
    assert far.localize(frame((0.5, 0.5, 0.0))) is None        # 0.46 m


def test_marker_map_from_graph(tmp_path):
    from pinky_lane_station.road_graph import RoadGraph
    graph = RoadGraph.load(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                        'config', 'road_graph.yaml'))
    mm = MarkerMap.load(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                     'config', 'markers.yaml'), graph)
    assert len(mm.poses) == len(graph.nodes)
    assert mm.poses[0][:2] == (graph.nodes['BL'].x, graph.nodes['BL'].y)
    p = tmp_path / 'm.yaml'
    p.write_text('markers:\n  - {id: 1, node: NOPE}\n', encoding='utf-8')
    with pytest.raises(ValueError):
        MarkerMap.load(str(p), graph)


def test_config_files_load():
    cfg = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
    intr = CameraIntrinsics.load(os.path.join(cfg, 'camera_intrinsics.yaml'))
    extr = CameraExtrinsics.load(os.path.join(cfg, 'camera_extrinsics.yaml'))
    assert intr.K.shape == (3, 3) and extr.T_base_cam.shape == (4, 4)
    assert not intr.calibrated and not extr.calibrated       # 실측 전
