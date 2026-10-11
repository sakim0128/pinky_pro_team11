"""red_line_detector — 색(HSV)으로 교차로 빨간 테이프 찾기 (ROS 불필요)."""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

cv2 = pytest.importorskip('cv2')

from pinky_lane_station.detectors import create_detector  # noqa: E402
from pinky_lane_station.lane_target import LaneTargetEstimator, TargetParams  # noqa: E402
from pinky_lane_station.red_line_detector import RedLineColorDetector, RedLineParams, params_from_dict  # noqa: E402
from pinky_lane_station.synthetic_camera import render_lane_frame  # noqa: E402

W, H = 640, 480
RED = (40, 30, 200)          # BGR 빨간 테이프
DIM_RED = (60, 50, 150)      # 조금 어두운 빨강 (채도 ≈ 170)


def floor_with_lanes():
    img = np.full((H, W, 3), (120, 120, 120), np.uint8)             # 회색 카펫
    cv2.line(img, (170, H), (250, 250), (245, 245, 245), 14)        # 흰 차선 두 줄
    cv2.line(img, (470, H), (390, 250), (245, 245, 245), 14)
    return img


def test_red_tape_near_bottom_becomes_one_instance():
    img = floor_with_lanes()
    cv2.rectangle(img, (150, 400), (490, 420), RED, -1)              # 차로를 가로지르는 빨간 테이프
    inst = RedLineColorDetector().detect(img)
    assert len(inst) == 1 and inst[0].cls == 'red_line'
    x0, y0, x1, y1 = inst[0].bbox
    assert abs(x0 - 150) <= 3 and abs(x1 - 490) <= 3 and abs(y1 - 420) <= 3
    assert inst[0].conf >= 0.8                                        # 꽉 찬 사각형


def test_white_lines_gray_floor_and_small_red_spot_are_ignored():
    img = floor_with_lanes()
    assert RedLineColorDetector().detect(img) == []
    cv2.circle(img, (320, 430), 5, RED, -1)                           # 작은 붉은 점 (잡음)
    assert RedLineColorDetector().detect(img) == []


def test_red_above_roi_is_ignored_and_dim_red_needs_lower_saturation():
    img = floor_with_lanes()
    cv2.rectangle(img, (150, 100), (490, 120), RED, -1)              # 먼 곳 (ROI 0.55·H 위)
    assert RedLineColorDetector().detect(img) == []
    img2 = floor_with_lanes()
    cv2.rectangle(img2, (150, 400), (490, 420), (90, 80, 130), -1)   # 채도 낮은 붉은색 (S ≈ 97)
    assert RedLineColorDetector().detect(img2) == []
    assert len(RedLineColorDetector(RedLineParams(s_min=60)).detect(img2)) == 1
    img3 = floor_with_lanes()
    cv2.rectangle(img3, (150, 400), (490, 420), DIM_RED, -1)
    assert len(RedLineColorDetector().detect(img3)) == 1


def test_disabled_and_params_from_dict():
    img = floor_with_lanes()
    cv2.rectangle(img, (150, 400), (490, 420), RED, -1)
    assert RedLineColorDetector(RedLineParams(enabled=False)).detect(img) == []
    p = params_from_dict({'s_min': 80, 'unknown': 1})
    assert p.s_min == 80 and p.enabled is True


def test_pipeline_path_red_tape_triggers_lane_target_after_confirm():
    """classic 차선 검출 + 색 검출 인스턴스 → lane_target 이 확정 2 프레임 뒤 red_line_detected (파이프라인과 같은 경로)."""
    img = render_lane_frame(noise=0)
    cv2.rectangle(img, (150, 395), (490, 415), RED, -1)
    det = create_detector('classic')
    red = RedLineColorDetector()
    est = LaneTargetEstimator(TargetParams(red_line_confirm=2))
    results = []
    for _ in range(2):
        inst = list(det.infer(img)) + red.detect(img)
        results.append(est.update(inst, W, H))
    assert results[0].red_line_raw and not results[0].red_line_detected
    assert results[1].red_line_detected
    far = render_lane_frame(noise=0)
    cv2.rectangle(far, (200, 300), (440, 315), RED, -1)              # 아직 멀다 (하단 < 0.80·H)
    r = LaneTargetEstimator().update(list(det.infer(far)) + red.detect(far), W, H)
    assert r.red_line_bottom_y > 0 and not r.red_line_raw
