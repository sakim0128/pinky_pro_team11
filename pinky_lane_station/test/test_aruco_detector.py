"""도착 지점 벽 ArUco 검출 — 합성 영상에서 id·거리, 디바운스, 설정."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pinky_lane_station.aruco_detector import (  # noqa: E402
    ArucoMarkerDetector, ArucoParams, distance_from_side, params_from_dict, side_length_px,
)

cv2 = pytest.importorskip('cv2')
if not hasattr(cv2, 'aruco'):
    pytest.skip('cv2.aruco 없음 (opencv-contrib)', allow_module_level=True)


def frame_with_markers(items, w=640, h=480):
    """items: [(id, side_px, cx, cy)] — 흰 여백을 두른 마커를 회색 배경에 붙인다."""
    img = np.full((h, w, 3), 90, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for mid, side, cx, cy in items:
        m = cv2.aruco.generateImageMarker(d, mid, side)
        pad = max(4, side // 4)
        tile = np.full((side + 2 * pad, side + 2 * pad), 255, np.uint8)
        tile[pad:pad + side, pad:pad + side] = m
        y0, x0 = cy - tile.shape[0] // 2, cx - tile.shape[1] // 2
        img[y0:y0 + tile.shape[0], x0:x0 + tile.shape[1]] = tile[:, :, None]
    return img


def test_distance_from_side_pinhole_and_width_scaling():
    p = ArucoParams(focal_px=500.0, marker_size=0.05, ref_width=640)
    assert distance_from_side(100.0, 640, p) == pytest.approx(0.25)
    assert distance_from_side(50.0, 320, p) == pytest.approx(0.25)     # 폭이 반이면 초점거리도 반
    assert distance_from_side(0.5, 640, p) is None
    assert side_length_px([[0, 0], [10, 0], [10, 10], [0, 10]]) == pytest.approx(10.0)


def test_detects_ids_and_distance():
    det = ArucoMarkerDetector(ArucoParams(focal_px=500.0, confirm=1))
    r = det.update(frame_with_markers([(1, 100, 200, 200), (3, 50, 460, 200)]))
    assert set(r.raw) == {1, 3} and set(r.markers) == {1, 3}
    assert r.raw[1] == pytest.approx(0.25, rel=0.08)
    assert r.raw[3] == pytest.approx(0.50, rel=0.08)


def test_ignores_ids_not_listed_and_far():
    det = ArucoMarkerDetector(ArucoParams(focal_px=500.0, confirm=1, ids=[1, 2, 3], max_distance=0.4))
    r = det.update(frame_with_markers([(7, 100, 200, 200), (2, 40, 460, 200)]))   # 7 은 목록 밖, 2 는 0.62 m
    assert r.raw == {} and r.markers == {}


def test_debounce_confirm_and_release_keeps_last_distance():
    det = ArucoMarkerDetector(ArucoParams(confirm=2, release=3))
    assert det.update_from_raw({1: 0.5}).markers == {}
    assert det.update_from_raw({1: 0.4}).markers == {1: 0.4}
    assert det.update_from_raw({}).markers == {1: 0.4}                # 한두 프레임 놓쳐도 유지
    assert det.update_from_raw({}).markers == {1: 0.4}
    assert det.update_from_raw({}).markers == {}                      # release 프레임 못 보면 해제


def test_params_from_yaml_and_disabled():
    import yaml
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(HERE), 'config', 'detector_yolo.yaml'), encoding='utf-8'))
    p = params_from_dict(cfg['aruco'])
    assert p.dictionary == 'DICT_4X4_50' and p.marker_size == pytest.approx(0.05) and p.ids == [1, 2, 3]
    off = ArucoMarkerDetector(ArucoParams(enabled=False))
    assert not off.available and off.update(frame_with_markers([(1, 100, 200, 200)])).markers == {}
