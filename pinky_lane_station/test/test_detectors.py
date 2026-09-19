"""검출기 레지스트리 · stub · classic (합성 이미지) — ROS 불필요."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_lane_station.detectors import DetectorError, create_detector  # noqa: E402
from pinky_lane_station.lane_target import LaneTargetEstimator  # noqa: E402
from pinky_lane_station.synthetic_camera import render_lane_frame  # noqa: E402


def test_registry_does_not_import_torch_or_ultralytics():
    import pinky_lane_station.detectors  # noqa: F401
    assert 'ultralytics' not in sys.modules
    assert 'torch' not in sys.modules


def test_unknown_kind_raises():
    with pytest.raises(DetectorError):
        create_detector({'kind': 'nope'})


def test_ultralytics_missing_model_file_raises_cleanly():
    pytest.importorskip('ultralytics')
    with pytest.raises(DetectorError):
        create_detector({'kind': 'ultralytics', 'model': '/nonexistent/best.pt'})


def test_stub_returns_configured_lanes_and_crosswalk():
    det = create_detector({'kind': 'stub', 'left_x': 200, 'right_x': 440, 'crosswalk_bottom_y': 450})
    inst = det.infer(None)
    assert [i.cls for i in inst] == ['lane', 'lane', 'crosswalk']
    r = LaneTargetEstimator().update(inst, 640, 480)
    assert abs(r.error_x) < 0.02 and r.crosswalk_raw


@pytest.mark.parametrize('lateral, sign', [(0.0, 0), (0.03, +1), (-0.03, -1)])
def test_classic_on_synthetic_frame_recovers_offset_sign(lateral, sign):
    det = create_detector('classic')
    est = LaneTargetEstimator()
    img = render_lane_frame(lateral=lateral)
    r = est.update(det.infer(img), img.shape[1], img.shape[0])
    assert r.quality_name == 'BOTH'
    if sign == 0:
        assert abs(r.error_x) < 0.05
    else:
        assert r.error_x * sign > 0.1


def test_classic_detects_crosswalk_near_and_ignores_far():
    det = create_detector('classic')
    est = LaneTargetEstimator()
    near = render_lane_frame(crosswalk_ahead=0.10)
    r = est.update(det.infer(near), 640, 480)
    assert r.crosswalk_raw
    est.reset()
    far = render_lane_frame(crosswalk_ahead=0.25)
    r = est.update(det.infer(far), 640, 480)
    assert r.crosswalk_bottom_y > 0 and not r.crosswalk_raw
    assert r.quality_name == 'BOTH'


def test_classic_single_stripe_is_not_crosswalk():
    import cv2
    img = render_lane_frame()
    cv2.rectangle(img, (150, 420), (500, 440), (245, 245, 245), -1)   # 벽 하단선 같은 한 줄
    inst = create_detector('classic').infer(img)
    assert all(i.cls == 'lane' for i in inst)


def test_infer_timed_reports_ms():
    det = create_detector('classic')
    out, ms = det.infer_timed(render_lane_frame())
    assert out and ms >= 0.0
