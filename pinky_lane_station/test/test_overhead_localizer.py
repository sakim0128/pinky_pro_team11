"""overhead_localizer — 기울어진 합성 항공뷰에서 기준 마커로 H 를 잡고 로봇 위치를 복원 (ROS 불필요)."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_lane_station.overhead_localizer import OverheadConfig, OverheadLocalizer  # noqa: E402
from pinky_lane_station.synthetic_camera import render_overhead  # noqa: E402

CFG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


@pytest.fixture(scope='module')
def cfg():
    return OverheadConfig.load(os.path.join(CFG_DIR, 'overhead.yaml'))


def ang(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def test_config_loads_and_ids_disjoint(cfg):
    assert len(cfg.reference) == 4 and set(cfg.robots) == {'pinky1', 'pinky2'}
    assert not set(m.id for m in cfg.robots.values()) & set(cfg.reference)


def test_config_rejects_bad():
    with pytest.raises(ValueError):
        OverheadConfig.from_dict({'reference': {'markers': [{'id': 1, 'x': 0, 'y': 0}]}})
    with pytest.raises(ValueError):
        OverheadConfig.from_dict({'reference': {'markers': [{'id': i, 'x': i, 'y': 0} for i in range(4)]},
                                  'robots': [{'name': 'a', 'id': 2}]})


@pytest.mark.parametrize('poses', [
    {'pinky1': (0.0, 0.0, 0.0), 'pinky2': (0.5, -0.2, math.pi / 2)},
    {'pinky1': (-0.8, 0.3, 2.5), 'pinky2': (0.9, 0.4, -1.0)},
    {'pinky1': (0.7, -0.45, math.pi)},
])
def test_recovers_robot_poses(cfg, poses):
    _, img = render_overhead(cfg, poses)
    loc = OverheadLocalizer(cfg)
    fixes = loc.update(img)
    assert loc.ref_seen == 4 and loc.reproj < 1.0
    assert set(fixes) == set(poses)
    for name, (x, y, yaw) in poses.items():
        f = fixes[name]
        assert math.hypot(f.x - x, f.y - y) < 0.01, (name, f)
        assert ang(f.yaw, yaw) < math.radians(1.0)
        assert f.n_markers == 4 and f.marker_id == cfg.robots[name].id


def test_offsets_applied():
    d = {'reference': {'size': 0.10, 'markers': [{'id': 40, 'x': -1, 'y': -0.5}, {'id': 41, 'x': 1, 'y': -0.5},
                                                 {'id': 42, 'x': 1, 'y': 0.5}, {'id': 43, 'x': -1, 'y': 0.5}]},
         'robots': [{'name': 'p', 'id': 30, 'size': 0.06, 'yaw_offset': math.pi / 2, 'offset_x': 0.05}]}
    cfg = OverheadConfig.from_dict(d)
    _, img = render_overhead(cfg, {'p': (0.2, 0.1, 0.4)})
    f = OverheadLocalizer(cfg).update(img)['p']
    # 6 cm 마커 ≈ 29 px 합성(계단 모서리) 이라 yaw 는 2° 까지 허용
    assert math.hypot(f.x - 0.2, f.y - 0.1) < 0.01 and ang(f.yaw, 0.4) < math.radians(2.0)


def test_keeps_last_homography_when_reference_occluded(cfg):
    loc = OverheadLocalizer(cfg)
    _, img = render_overhead(cfg, {'pinky1': (0.0, 0.0, 0.0)})
    assert loc.update(img)
    H_before = loc.H.copy()
    # 기준 마커 하나를 가린다 (id 40 은 좌하단 → 캔버스 좌하단 부근을 덮는다)
    import cv2
    _, img2 = render_overhead(cfg, {'pinky1': (0.1, 0.0, 0.0)})
    corners = loc.detect(img2)[40].astype(np.int32)
    cv2.fillPoly(img2, [corners], (110, 110, 110))
    fixes = loc.update(img2)
    assert loc.ref_seen == 3
    assert np.allclose(loc.H, H_before)
    assert math.hypot(fixes['pinky1'].x - 0.1, fixes['pinky1'].y) < 0.01


def test_no_fix_without_homography(cfg):
    loc = OverheadLocalizer(cfg)
    _, img = render_overhead(cfg, {'pinky1': (0.0, 0.0, 0.0)})
    import cv2
    for mid in (40, 41, 42, 43):
        c = loc.detect(img)[mid].astype(np.int32)
        cv2.fillPoly(img, [c], (110, 110, 110))
    assert loc.update(img) == {} and loc.H is None


def test_pose_fuser_station_clock_rule():
    """관제 시계 fix 는 now − latency 의 odom 에 붙는다 (노드 규칙을 코어로 재현)."""
    sys.path.insert(0, os.path.join(os.path.dirname(CFG_DIR), '..', 'pinky_fleet_agent'))
    from pinky_fleet_agent.pose_fuser import PoseFuser
    f = PoseFuser()
    for i in range(21):
        f.add_odom(i * 0.05, i * 0.01, 0.0, 0.0)        # 0.2 m/s
    now, latency = 1.0, 0.15
    ok, _ = f.on_fix(now - latency, 2.0, 0.0, 0.0, now=now)
    assert ok
    assert abs(f.map_pose(0.20, 0.0, 0.0)[0] - (2.0 + 0.2 * latency)) < 1e-6
