"""overhead_calib — 합성 ArUco 이미지에서 호모그래피 복원 (ROS 불필요, aruco 필요)."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

cv2 = pytest.importorskip('cv2')
if not hasattr(cv2, 'aruco'):
    pytest.skip('cv2.aruco 없음', allow_module_level=True)

from pinky_fleet_station.overhead_calib import (CalibError, calibrate, homography_from_pairs,  # noqa: E402
                                                parse_corner_map, project, yaml_line)
from pinky_fleet_station.overhead_math import transform_point  # noqa: E402

CORNERS = {40: (0.10, 0.10), 41: (2.25, 0.10), 42: (2.25, 1.15), 43: (0.10, 1.15)}   # map (m)
ROBOT = {1: (1.20, 0.60), 2: (0.60, 0.90)}


def synth_image(map_to_px, size=(1280, 720), marker_m=0.10, dictionary=cv2.aruco.DICT_4X4_50):
    """map 평면의 마커들을 임의 호모그래피(카메라 기울기)로 워프한 합성 항공뷰."""
    d = cv2.aruco.getPredefinedDictionary(dictionary)
    W, H = size
    img = np.full((H, W), 200, dtype=np.uint8)
    for mid, (mx, my) in {**CORNERS, **ROBOT}.items():
        tile = cv2.aruco.generateImageMarker(d, mid, 120)
        tile = cv2.copyMakeBorder(tile, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)   # 흰 테두리
        h = marker_m / 2.0 * (160 / 120)
        src = np.array([[0, 0], [160, 0], [160, 160], [0, 160]], dtype=np.float32)
        # 마커 상단(+y 방향)이 map +y 를 향하도록 map 사각형에 놓는다 (이미지 y 는 아래로 자람)
        quad_map = np.array([[mx - h, my + h], [mx + h, my + h], [mx + h, my - h], [mx - h, my - h]])
        quad_px = np.array([transform_point(map_to_px, q) for q in quad_map], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, quad_px)
        warped = cv2.warpPerspective(tile, M, (W, H), borderValue=0)
        mask = cv2.warpPerspective(np.full((160, 160), 255, np.uint8), M, (W, H))
        img[mask > 0] = warped[mask > 0]
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def tilted_map_to_px():
    """map (m) → px. 스케일 450 px/m + 살짝 기울어진 원근."""
    S = np.array([[450.0, 0.0, 60.0], [0.0, -450.0, 620.0], [0.0, 0.0, 1.0]])
    P = np.array([[1.0, 0.05, 0.0], [0.02, 1.0, 0.0], [0.00012, 0.00008, 1.0]])
    return P @ S


def test_calibrate_recovers_robot_positions_within_1cm():
    m2p = tilted_map_to_px()
    img = synth_image(m2p)
    H, rep = calibrate(img, CORNERS)
    assert rep['missing'] == [] and rep['max_reproj_m'] < 0.005
    assert set(rep['other_ids']) == {1, 2}
    from pinky_fleet_station.overhead_calib import detect_centers
    centers = detect_centers(img)
    for rid, (mx, my) in ROBOT.items():
        x, y = project(H, *centers[rid])
        assert math.hypot(x - mx, y - my) < 0.01, (rid, x, y)


def test_calibrate_needs_four_visible_corners():
    img = synth_image(tilted_map_to_px())
    with pytest.raises(CalibError):
        calibrate(img, {40: CORNERS[40], 41: CORNERS[41], 42: CORNERS[42], 99: (0.0, 0.0)})


def test_homography_pairs_and_yaml_line_roundtrip():
    m2p = tilted_map_to_px()
    pairs = []
    for mx, my in CORNERS.values():
        px, py = transform_point(m2p, (mx, my))
        pairs.append((px, py, mx, my))
    H, errs = homography_from_pairs(pairs)
    assert max(errs) < 1e-6
    line = yaml_line(H)
    assert line.startswith('image_to_map_homography: [') and line.count(',') == 8
    vals = [float(v) for v in line.split('[')[1].rstrip(']').split(',')]
    x, y = transform_point(np.array(vals).reshape(3, 3), transform_point(m2p, (1.0, 0.5)))
    assert abs(x - 1.0) < 1e-4 and abs(y - 0.5) < 1e-4
    with pytest.raises(CalibError):
        homography_from_pairs(pairs[:3])


def test_parse_corner_map():
    m = parse_corner_map('40:0.10,0.10; 41:2.25,0.10;42:2.25,1.15;43:0.10,1.15')
    assert m == CORNERS
    with pytest.raises(CalibError):
        parse_corner_map('40:0,0;41:1,0')
