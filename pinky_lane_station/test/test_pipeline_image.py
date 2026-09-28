"""pipeline_image: 상위 마스킹 + 원본 위 오버레이 — ROS 불필요."""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_lane_station.detectors import create_detector  # noqa: E402
from pinky_lane_station.lane_target import Instance, LaneTargetEstimator, TargetResult  # noqa: E402
from pinky_lane_station.pipeline_image import (CLASS_COLORS, TARGET_COLOR, draw_debug,  # noqa: E402
                                               mask_top, mask_top_rows)
from pinky_lane_station.synthetic_camera import render_lane_frame  # noqa: E402

cv2 = pytest.importorskip('cv2')


def _frame():
    return render_lane_frame(noise=0)


def test_mask_top_fills_exactly_top_rows_and_keeps_original():
    img = _frame()
    before = img.copy()
    out = mask_top(img, 0.30, 0)
    H = img.shape[0]
    n = int(0.30 * H)
    assert mask_top_rows(H, 0.30) == n == 144
    assert out is not img and out.shape == img.shape and out.dtype == img.dtype
    assert (out[:n] == 0).all()
    assert np.array_equal(out[n:], img[n:])
    assert np.array_equal(img, before)                       # 원본 보존


def test_mask_top_fill_value_and_edges():
    img = _frame()
    assert (mask_top(img, 0.30, 7)[:144] == 7).all()
    assert np.array_equal(mask_top(img, 0.0), img)
    assert (mask_top(img, 1.0) == 0).all()
    assert np.array_equal(mask_top(img, -1.0), img)        # 클램프
    assert (mask_top(img, 2.0) == 0).all()


def test_classic_on_masked_frame_still_both():
    det = create_detector('classic')
    img = _frame()
    r = LaneTargetEstimator().update(det.infer(mask_top(img, 0.30)), img.shape[1], img.shape[0])
    assert r.quality_name == 'BOTH' and abs(r.error_x) < 0.05


def test_masked_inference_has_no_instances_in_masked_band():
    det = create_detector('classic')
    img = _frame()
    for inst in det.infer(mask_top(img, 0.30)):
        assert inst.bbox[1] >= 144 - 2, inst.bbox


def test_draw_debug_draws_on_original_not_masked():
    img = _frame()
    r = LaneTargetEstimator().update(create_detector('classic').infer(mask_top(img, 0.30)), 640, 480)
    dbg = draw_debug(img, [], r, mask_frac=0.30)
    assert dbg.shape == img.shape and dbg is not img
    assert dbg[:100].mean() > 50                            # 마스크 영역이 검정이 아님 (원본 배경)
    assert np.array_equal(img, _frame())                     # 원본 그대로


def test_draw_debug_bbox_per_instance_and_target_point():
    img = np.full((480, 640, 3), 120, dtype=np.uint8)
    insts = [Instance('lane', 0.9, [(100, 300), (140, 300), (140, 460), (100, 460)]),
             Instance('lane', 0.8, [(500, 300), (540, 300), (540, 460), (500, 460)]),
             Instance('crosswalk', 0.7, [(200, 400), (440, 400), (440, 440), (200, 440)]),
             Instance('cone', 0.6, [(300, 200), (340, 200), (340, 260), (300, 260)])]
    r = LaneTargetEstimator().update(insts, 640, 480)
    assert r.quality_name == 'BOTH'
    dbg = draw_debug(img, insts, r, mask_frac=0.30, infer_ms=12.0)
    for inst in insts:                                       # bbox 테두리 색이 각 인스턴스에 존재
        x0, y0, x1, y1 = (int(v) for v in inst.bbox)
        color = np.array(CLASS_COLORS[inst.cls])
        edge = dbg[y0:y1 + 1, x0:x0 + 2]
        assert (np.abs(edge.astype(int) - color).sum(axis=2) < 30).any(), inst.cls
    ty = int(r.target_y)
    tx = int(r.target_x)
    assert tuple(dbg[ty, tx]) == TARGET_COLOR                # 차선 중심점
    row = dbg[144]
    assert (row == 128).all(axis=1).any()                    # 마스크 경계 점선
    assert (img == 120).all()                                # 원본 보존


def test_draw_debug_lost_has_no_target_point():
    img = np.full((480, 640, 3), 120, dtype=np.uint8)
    r = TargetResult()
    dbg = draw_debug(img, [], r, mask_frac=0.0)
    assert not (dbg[int(r.target_y)] == np.array(TARGET_COLOR)).all(axis=1).any()
    assert not (dbg == 128).all(axis=2).any()                # mask_frac 0 → 경계선 없음
