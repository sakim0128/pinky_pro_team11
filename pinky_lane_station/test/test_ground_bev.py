"""ground_bev — 바닥 좌표 차로 중앙 · 목표점 · 부호 규약 (ROS 불필요).

바닥에 테이프(폭 21 mm)를 놓았다고 보고 실제 캘리브레이션(config/pinky_cam.yaml)으로 영상 마스크를 만들어 넣는다.
모듈 좌표: x 오른쪽, y 전방, mm, 곡률 + = 우회전. 로봇 규약(robot_*): x 전방, y 왼쪽, m, 곡률 + = 좌회전.
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

cv2 = pytest.importorskip('cv2')

from pinky_lane_station import ground_bev  # noqa: E402
from pinky_lane_station.ground_bev import (  # noqa: E402
    BevLaneFollower, BevParams, GroundCamera, apply_bev, lane_mask_from_instances)
from pinky_lane_station.lane_target import (  # noqa: E402
    QUALITY_BOTH, QUALITY_LOST, QUALITY_SINGLE, Instance, TargetResult)

CALIB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'pinky_cam.yaml')
W, H = 640, 480
HALF = 87.0          # 차선 반폭 (테이프 중심 간 174 mm)
TAPE = 21.0
FOV_TAN = 0.5         # 화각 안만 그린다 (fx≈559 → 반화각 tan ≈ 0.57)


@pytest.fixture(scope='module')
def cam():
    return GroundCamera.from_yaml(CALIB)


def tape_mask_from_floor(cam, centre_line, width=TAPE):
    """바닥 폴리라인(mm) 을 폭 width 의 테이프로 보고 영상 마스크로 칠한다."""
    line = np.asarray(centre_line, float)
    t = np.gradient(line, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    n = np.c_[t[:, 1], -t[:, 0]]
    m = np.zeros((H, W), np.uint8)
    for a, b, na, nb in zip(line[:-1], line[1:], n[:-1], n[1:]):
        quad = np.array([a - na * width / 2, b - nb * width / 2, b + nb * width / 2, a + na * width / 2])
        if not cam.in_front(quad).all() or (np.abs(quad[:, 0]) > FOV_TAN * quad[:, 1]).any():
            continue                     # 화각 밖 — 렌즈 왜곡식이 접혀 엉뚱한 픽셀로 간다
        cv2.fillPoly(m, [np.round(cam.floor_to_pixels(quad)).astype(np.int32)], 1)
    return m.astype(bool)


def straight(x, y0=60.0, y1=700.0):
    ys = np.arange(y0, y1, 5.0)
    return np.c_[np.full_like(ys, x), ys]


def arc(radius, centre_x, y0=60.0, sweep=1.2):
    """(centre_x, 0) 을 중심으로 하는 원호 — centre_x < 0 이면 왼쪽으로 휘는 길."""
    th = np.linspace(0.0, sweep, 120)
    s = 1.0 if centre_x < 0 else -1.0
    return np.c_[centre_x + s * radius * np.cos(th), y0 + radius * np.sin(th)]


def follow(cam, *lines, **kw):
    m = np.zeros((H, W), bool)
    for ln in lines:
        m |= tape_mask_from_floor(cam, ln)
    return BevLaneFollower(cam, BevParams(enabled=True, tape_trim=False, **kw)).update(m)


def test_pixel_floor_round_trip(cam):
    pts = np.array([[0.0, 200.0], [-80.0, 150.0], [120.0, 400.0]])
    back, ok = cam.pixels_to_floor(cam.floor_to_pixels(pts))
    assert ok.all()
    assert np.allclose(back, pts, atol=0.5)


def test_horizon_pixels_are_invalid(cam):
    _, ok = cam.pixels_to_floor([[320, 5]])
    assert not ok[0]


def test_centred_on_straight_road_goes_straight(cam):
    r = follow(cam, straight(-HALF), straight(HALF))
    assert r.mode == 'both'
    assert abs(r.target[0]) < 8.0
    assert abs(r.robot_curvature()) < 0.3
    fwd, left = r.robot_target_m()
    assert fwd == pytest.approx(0.25 - 0.033, abs=0.02) and abs(left) < 0.01


def test_robot_left_of_centre_turns_right(cam):
    """로봇이 차로 왼쪽으로 30 mm 치우침 → 중앙은 오른쪽 → 우회전 (로봇 곡률 음수, 목표 y(왼쪽) 음수)."""
    r = follow(cam, straight(-HALF + 30), straight(HALF + 30))
    assert r.target[0] == pytest.approx(30.0, abs=12.0)
    assert r.curvature > 0 and r.robot_curvature() < 0
    assert r.robot_target_m()[1] < 0


def test_single_lane_gives_same_centre_as_both(cam):
    """선 하나만 보여도 바닥에선 반폭만큼 옮기면 같은 중앙 — 영상 픽셀 오프셋과 달리 쌍/단일 전환에서 옆으로 튀지 않는다.
    (치우친 쪽 선은 화각 때문에 더 멀리서부터 보여 목표점의 전방 거리는 다를 수 있다 — 옆 위치만 비교)"""
    both = follow(cam, straight(-HALF + 30), straight(HALF + 30))
    right = follow(cam, straight(HALF + 30))
    left = follow(cam, straight(-HALF + 30))
    assert right.mode == 'right_only' and left.mode == 'left_only'
    assert abs(right.target[0] - both.target[0]) < 12.0
    assert abs(left.target[0] - both.target[0]) < 12.0


@pytest.mark.parametrize('turn', ['left', 'right'])
def test_curve_direction(cam, turn):
    """반경 300 mm 커브 (중앙선 기준) — 커브 쪽으로 돈다. 바깥 선 하나만 보여도."""
    R = 300.0
    cx = -R if turn == 'left' else R
    inner, outer = arc(R - HALF, cx), arc(R + HALF, cx)
    for lines in ((inner, outer), (outer,)):
        r = follow(cam, *lines)
        assert r.valid, lines
        k = r.robot_curvature()                       # 좌회전 +
        assert (k > 1.0) if turn == 'left' else (k < -1.0)


def test_nothing_on_floor_is_lost(cam):
    r = BevLaneFollower(cam, BevParams(enabled=True)).update(np.zeros((H, W), bool))
    assert not r.valid and r.mode == 'none' and r.robot_target_m() is None


def test_fallback_thinning_matches_skimage(cam, monkeypatch):
    lines = (straight(-HALF + 20), straight(HALF + 20))
    ref = follow(cam, *lines)
    monkeypatch.setattr(ground_bev, '_sk_skeletonize', None)
    alt = follow(cam, *lines)
    assert alt.mode == ref.mode
    assert abs(alt.target[0] - ref.target[0]) < 8.0          # 골격선 끝 모양이 조금 달라 전방 위치는 1~2 cm 다를 수 있다


def test_lane_mask_from_instances_merges_left_right():
    insts = [Instance('left_lane', 0.9, [(10, 400), (40, 400), (40, 470), (10, 470)]),
             Instance('right_lane', 0.9, [(600, 400), (630, 400), (630, 470), (600, 470)]),
             Instance('crosswalk', 0.9, [(200, 400), (400, 400), (400, 470), (200, 470)])]
    m = lane_mask_from_instances(insts, (H, W), ('lane', 'left_lane', 'right_lane'))
    assert m[430, 20] and m[430, 610] and not m[430, 300]


def test_apply_bev_sets_quality_and_projected_error(cam):
    r = apply_bev(TargetResult(), follow(cam, straight(-HALF + 30), straight(HALF + 30)), cam, W, H)
    assert r.quality == QUALITY_BOTH and r.left_seen and r.right_seen
    assert r.error_x > 0.02                           # 중앙이 화면 오른쪽 = 로봇 왼쪽 치우침 (LanePath 규약)
    r = apply_bev(TargetResult(), follow(cam, straight(HALF)), cam, W, H)
    assert r.quality == QUALITY_SINGLE and r.right_seen and not r.left_seen
    r = apply_bev(TargetResult(quality=QUALITY_BOTH, error_x=0.5), ground_bev.BevResult(), cam, W, H)
    assert r.quality == QUALITY_LOST and r.error_x == 0.0


# ------------------------------------------------ 관제 웹 화면: 위에서 본 BEV (BevView)

def test_bev_view_maps_floor_points_to_their_camera_pixels(cam):
    from pinky_lane_station.ground_bev import BevView
    v = BevView(cam, BevParams(enabled=True))
    xy = np.array([[0.0, 200.0], [-60.0, 300.0], [50.0, 150.0]])
    px = v.floor_to_px(xy)
    assert np.allclose(v.px_to_floor(px[:, 0], px[:, 1]), xy)
    for (u, vv), p in zip(np.round(px).astype(int), xy):
        assert v.valid[vv, u]
        assert np.allclose([v.map_x[vv, u], v.map_y[vv, u]], cam.floor_to_pixels(p)[0], atol=3.0)
    behind = np.round(v.floor_to_px([[0.0, -50.0]])).astype(int)[0]
    assert not v.valid[behind[1], behind[0]]                      # 로봇 뒤는 카메라에 안 보인다


def test_bev_view_renders_tape_lanes_path_and_robot(cam):
    from pinky_lane_station.ground_bev import BevView
    v = BevView(cam, BevParams(enabled=True))
    m = tape_mask_from_floor(cam, straight(-HALF)) | tape_mask_from_floor(cam, straight(HALF))
    img = np.full((H, W, 3), 90, np.uint8)
    img[m] = 255
    r = BevLaneFollower(cam, BevParams(enabled=True, tape_trim=False)).update(m)
    plain = v.render(img, None)
    # 바닥의 흰 테이프는 위에서 본 그림에서도 테이프 자리(x = ±87 mm, 앞 25 cm)에 흰색, 차선 사이는 바닥색이다
    for x in (-HALF, HALF):
        u, vv = np.round(v.floor_to_px([[x, 250.0]])).astype(int)[0]
        assert plain[vv - 2:vv + 3, u - 2:u + 3].min(axis=-1).max() >= 240
    u, vv = np.round(v.floor_to_px([[0.0, 250.0]])).astype(int)[0]
    assert plain[vv, u].max() <= 100
    out = v.render(img, r, mask_top_frac=0.5)
    assert out.shape == (v.H, v.W, 3)
    # 목표점(빨간 원)과 로봇 몸통(흰 사각형)이 그려진다
    tu, tv = np.round(v.floor_to_px(r.target)).astype(int)[0]
    red = (out[..., 2] > 200) & (out[..., 1] < 60) & (out[..., 0] < 60)
    assert red[tv - 8:tv + 9, tu - 8:tu + 9].any()
    fu, fv = np.round(v.floor_to_px([[0.0, -60.0]])).astype(int)[0]
    assert (out[fv, :] >= 250).all(axis=-1).sum() >= 2              # 몸통 좌우 변
