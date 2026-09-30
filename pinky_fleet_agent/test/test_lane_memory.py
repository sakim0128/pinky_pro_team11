"""카메라 사각지대 보정 — 화면→바닥 거리 모델, 차선 중앙 기억, 기억 점 pure pursuit, 드라이버 폐루프 (lane_memory)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent.drive_fsm import CRUISE, LANE_SEARCH  # noqa: E402
from pinky_fleet_agent.lane_control import QUALITY_BOTH, QUALITY_LOST  # noqa: E402
from pinky_fleet_agent.lane_driver import DriverParams, LaneDriver  # noqa: E402
from pinky_fleet_agent.lane_memory import (  # noqa: E402
    GroundView, LaneMemory, MemoryParams, OdomBuffer, ViewParams,
)

W, H = 640, 480
HALF_PX = 100.0
DT = 0.05


def test_ground_view_matches_the_two_measured_rows():
    v = GroundView(ViewParams())
    assert v.camera_distance(1.0) == pytest.approx(0.10)
    assert v.camera_distance(0.5) == pytest.approx(0.43)
    assert v.camera_distance(0.72) == pytest.approx(0.1754, abs=0.002)      # 차선을 읽는 행 ≈ 17.5 cm
    assert v.camera_distance(0.80) == pytest.approx(0.144, abs=0.002)       # 빨간 선 정지 행
    assert v.camera_distance(0.30) is None                                  # 지평선 위
    with pytest.raises(ValueError):
        GroundView(ViewParams(bottom_m=0.4, mid_m=0.3))


def test_to_robot_forward_and_lateral():
    p = ViewParams(axle_to_camera_m=0.04)
    v = GroundView(p)
    x, y = v.to_robot(W / 2, 0.72 * H, W, H, HALF_PX)
    assert x == pytest.approx(0.04 + 0.1754, abs=0.002) and y == pytest.approx(0.0)
    # 샘플 행에서 반폭(100 px) 만큼 오른쪽 = 7.5 cm 오른쪽 (y 음수)
    _, y = v.to_robot(W / 2 + HALF_PX, 0.72 * H, W, H, HALF_PX)
    assert y == pytest.approx(-0.075)
    # 더 아래(가까운) 행은 같은 px 가 더 짧다 (원근)
    _, y_near = v.to_robot(W / 2 + HALF_PX, 0.90 * H, W, H, HALF_PX)
    assert -0.075 < y_near < 0.0
    assert v.to_robot(W / 2, 0.72 * H, W, H, 0.0) is None


def test_odom_buffer_interpolates_and_rejects_far_stamps():
    b = OdomBuffer(max_gap=0.3)
    b.add(1.0, 0.0, 0.0, 0.0)
    b.add(1.1, 0.01, 0.0, 0.1)
    assert b.at(1.05) == pytest.approx((0.005, 0.0, 0.05))
    assert b.at(1.2) == pytest.approx((0.01, 0.0, 0.1))                    # 0.1 s 넘게 뒤 — 마지막 값
    assert b.at(2.0) is None and b.at(0.5) is None


def test_memory_places_point_where_it_was_seen_even_if_robot_moved_since():
    m = LaneMemory(MemoryParams(lookahead=0.05, average=1))
    m.add((0.20, 0.03), (0.0, 0.0, 0.0))                  # 사진을 찍을 때 자세에서 본 점
    # 그 사이 로봇이 0.05 m 전진했다 — 같은 바닥점은 로봇 좌표에서 0.15 m 앞
    t = m.target((0.05, 0.0, 0.0))
    assert t == pytest.approx((0.15, 0.03))
    m.add((0.20, 0.0), (0.05, 0.0, 0.0))
    m.prune((0.30, 0.0, 0.0))                             # 지나간 점은 버린다
    assert len(m) == 0


def test_curvature_sign_and_lookahead():
    m = LaneMemory(MemoryParams(lookahead=0.12, average=1))
    for x in (0.05, 0.10, 0.14, 0.20):
        m.add((x, 0.02), (0.0, 0.0, 0.0))
    t = m.target((0.0, 0.0, 0.0))
    assert t[0] == pytest.approx(0.14)                    # 0.12 m 안쪽 점은 건너뛴다
    assert m.curvature((0.0, 0.0, 0.0)) > 0               # 왼쪽 점 → 좌회전
    assert LaneMemory().curvature((0.0, 0.0, 0.0)) is None


# ------------------------------------------------------------ 폐루프: 사각지대가 있는 카메라로 곡선 차선 주행

def course(radius=0.35):
    """중심선: 직선 0.6 m → 왼쪽 90° 원호(반경 radius) → 직선 0.8 m. 5 mm 간격 점."""
    pts = [(i * 0.005, 0.0) for i in range(121)]
    cx, cy = 0.6, radius
    for k in range(1, 158):
        a = -math.pi / 2 + (math.pi / 2) * k / 157
        pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
    ex, ey = pts[-1]
    pts += [(ex, ey + i * 0.005) for i in range(1, 161)]
    return pts


def lateral_error(pts, x, y):
    return min(math.hypot(px - x, py - y) for px, py in pts)


def camera_target(pts, pose, view):
    """카메라가 샘플 행(0.72·H)에서 보는 차선 중앙 → 화면 목표점 px. 안 보이면 None."""
    x, y, yaw = pose
    d = view.axle_distance(0.72)
    c, s = math.cos(yaw), math.sin(yaw)
    best = None
    loc = [((px - x) * c + (py - y) * s, -(px - x) * s + (py - y) * c) for px, py in pts]
    for (x0, y0), (x1, y1) in zip(loc, loc[1:]):
        if (x0 - d) * (x1 - d) <= 0 and x0 != x1:
            a = (d - x0) / (x1 - x0)
            yy = y0 + a * (y1 - y0)
            if best is None or abs(yy) < abs(best):
                best = yy
    if best is None or abs(best) > 0.2:
        return None
    m_per_px = 0.075 / HALF_PX
    return (W / 2 - best / m_per_px, 0.72 * H, W, H, HALF_PX)


def drive(follow_memory, seconds=14.0, blackout=None, y0=0.0):
    p = DriverParams()
    p.lane_only = True
    p.follow_memory = follow_memory
    p.control.v_max = 0.12
    d = LaneDriver(p)
    d.started = True
    view = GroundView(p.view)
    pts = course()
    x, y, yaw, t = 0.0, y0, 0.0, 0.0
    errs, states, log = [], [], []
    for k in range(int(seconds / DT)):
        t += DT
        d.update_odom(t, x, y, yaw, stamp=t)
        if k % 2 == 0:                                            # 카메라 10 Hz
            tgt = camera_target(pts, (x, y, yaw), view)
            dark = blackout is not None and blackout[0] <= t < blackout[1]
            if tgt is None or dark:
                d.set_lane_path(t, t, QUALITY_LOST, None)
            else:
                e = (tgt[0] - W / 2) / (W / 2)
                d.set_lane_path(t, t, QUALITY_BOTH, e, target=tgt)
        out = d.tick(t, 0.0, 0.0, 0.0)
        x += out.v * math.cos(yaw) * DT
        y += out.v * math.sin(yaw) * DT
        yaw += out.omega * DT
        errs.append(lateral_error(pts, x, y))
        states.append(out.state)
        log.append((t, out))
        if y > 1.0:
            break
    return errs, states, (x, y, yaw), log


def test_memory_follows_curve_closer_to_center_than_direct_steering():
    e_mem, _, end_mem, _ = drive(True)
    e_dir, _, _, _ = drive(False)
    rms = lambda es: math.sqrt(sum(e * e for e in es) / len(es))   # noqa: E731
    assert end_mem[1] > 0.6                                        # 커브를 돌아 나갔다
    assert max(e_mem) < 0.03                                       # 도로 반폭 7.5 cm 안에서 여유
    assert rms(e_mem) < rms(e_dir) and max(e_mem) < max(e_dir)


def test_memory_converges_from_offset_start():
    errs, states, _, _ = drive(True, seconds=6.0, y0=0.03)
    assert errs[-1] < 0.01 and states[-1] == CRUISE


def test_short_blackout_is_bridged_by_remembered_points():
    # 직선 구간에서 0.4 s 차선이 안 보여도 본 곳까지 계속 간다 (탐색 회전으로 빠지지 않는다)
    errs, states, _, log = drive(True, seconds=6.0, blackout=(2.0, 2.4))
    during = [o for t, o in log if 2.0 <= t < 2.4]
    assert all(o.v > 0.05 for o in during[2:])
    assert LANE_SEARCH not in states and max(errs) < 0.02


def test_follow_memory_off_keeps_old_behaviour():
    _, _, _, log = drive(False, seconds=1.0)
    assert all('기억' not in o.reason for _, o in log)
