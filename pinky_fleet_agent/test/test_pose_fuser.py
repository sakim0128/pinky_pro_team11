"""pose_fuser — odom 드리프트를 fix 로 바로잡는지, 게이트·타임아웃 (ROS 불필요)."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent.pose_fuser import PoseFuser, compose, invert, wrap  # noqa: E402


def test_compose_invert_roundtrip():
    a = (1.0, 2.0, 0.7)
    b = (0.3, -0.1, -1.2)
    ab = compose(a, b)
    back = compose(ab, invert(b))
    assert all(abs(x - y) < 1e-9 for x, y in zip(back, a))
    ident = compose(a, invert(a))
    assert abs(ident[0]) < 1e-9 and abs(ident[1]) < 1e-9 and abs(ident[2]) < 1e-9


def drive(f, t0, t1, dt, odom_pose, v, w, drift=(0.0, 0.0)):
    """odom 을 적분한다 (drift: 초당 (m, rad) 추가 오차). 실제 pose 도 같이 돌려준다."""
    x, y, th = odom_pose
    t = t0
    while t < t1 - 1e-9:
        th = wrap(th + (w + drift[1]) * dt)
        x += (v + drift[0]) * math.cos(th) * dt
        y += (v + drift[0]) * math.sin(th) * dt
        t += dt
        f.add_odom(t, x, y, th)
    return x, y, th


def test_first_fix_sets_map_odom_and_pose_follows_odom():
    f = PoseFuser()
    f.add_odom(0.0, 0.0, 0.0, 0.0)
    ok, _ = f.on_fix(0.0, 1.0, 0.5, math.pi / 2, now=0.1)
    assert ok
    assert f.map_pose(0.0, 0.0, 0.0) == (1.0, 0.5, math.pi / 2)
    # odom 이 x 로 0.2 m 가면 map 에서는 y 로 0.2 m
    x, y, th = f.map_pose(0.2, 0.0, 0.0)
    assert abs(x - 1.0) < 1e-9 and abs(y - 0.7) < 1e-9


def test_fix_at_past_stamp_uses_odom_at_that_time():
    f = PoseFuser()
    for i in range(21):
        f.add_odom(i * 0.05, i * 0.01, 0.0, 0.0)          # 0.2 m/s
    # 0.5 s 전 이미지의 fix (그때 odom x = 0.10) 가 map (2.0, 0, 0) 이었다
    ok, _ = f.on_fix(0.5, 2.0, 0.0, 0.0, now=1.0)
    assert ok
    assert abs(f.map_odom[0] - 1.9) < 1e-6
    assert abs(f.map_pose(0.20, 0.0, 0.0)[0] - 2.1) < 1e-6   # 지금 odom 0.20 → map 2.10


def test_odom_interpolation():
    f = PoseFuser()
    f.add_odom(0.0, 0.0, 0.0, 0.0)
    f.add_odom(1.0, 1.0, 0.0, 1.0)
    x, y, th = f.odom_at(0.25)
    assert abs(x - 0.25) < 1e-9 and abs(th - 0.25) < 1e-9
    assert f.odom_at(-1.0) is None and f.odom_at(1.1) == (1.0, 0.0, 1.0)


def test_drift_is_corrected_by_periodic_fixes():
    f = PoseFuser()
    f.add_odom(0.0, 0.0, 0.0, 0.0)
    true = (0.0, 0.0, 0.0)
    odom = (0.0, 0.0, 0.0)
    f.on_fix(0.0, *true, now=0.0)
    t = 0.0
    worst = 0.0
    for _ in range(6):                     # 3 s 마다 fix, 그 사이 odom 이 2 %/s 드리프트
        odom = drive(f, t, t + 3.0, 0.05, odom, 0.15, 0.0, drift=(0.003, 0.02))
        true = (true[0] + 0.45 * math.cos(true[2]), true[1] + 0.45 * math.sin(true[2]), true[2])
        t += 3.0
        est = f.map_pose(*odom)
        worst = max(worst, math.hypot(est[0] - true[0], est[1] - true[1]))
        ok, _ = f.on_fix(t, *true, now=t)
        assert ok
        est = f.map_pose(*odom)
        assert math.hypot(est[0] - true[0], est[1] - true[1]) < 1e-6
    assert worst < 0.08                    # 3 s 사이 오차는 수 cm 에 그친다


def test_big_jump_needs_two_consistent_fixes():
    f = PoseFuser(gate_dist=0.3, confirm=2)
    f.add_odom(0.0, 0.0, 0.0, 0.0)
    f.on_fix(0.0, 0.0, 0.0, 0.0, now=0.0)
    f.add_odom(1.0, 0.1, 0.0, 0.0)
    ok, why = f.on_fix(1.0, 2.0, 0.0, 0.0, now=1.0)        # 1.9 m 점프
    assert not ok and '보류' in why
    assert abs(f.map_pose(0.1, 0.0, 0.0)[0] - 0.1) < 1e-9   # 아직 옛 추정
    f.add_odom(2.0, 0.2, 0.0, 0.0)
    ok, why = f.on_fix(2.0, 2.1, 0.0, 0.0, now=2.0)        # 같은 곳을 다시 봄
    assert ok and '확정' in why
    assert abs(f.map_pose(0.2, 0.0, 0.0)[0] - 2.1) < 1e-9


def test_inconsistent_jumps_never_accepted():
    f = PoseFuser(gate_dist=0.3, confirm=2)
    f.add_odom(0.0, 0.0, 0.0, 0.0)
    f.on_fix(0.0, 0.0, 0.0, 0.0, now=0.0)
    for i, x in enumerate((2.0, 5.0, -3.0), start=1):
        f.add_odom(float(i), 0.0, 0.0, 0.0)
        ok, _ = f.on_fix(float(i), x, 0.0, 0.0, now=float(i))
        assert not ok
    assert f.rejected == 3 and f.accepted == 1


def test_fix_without_odom_history_rejected():
    f = PoseFuser()
    ok, why = f.on_fix(5.0, 1.0, 1.0, 0.0, now=5.0)
    assert not ok and 'odom' in why
    f.add_odom(10.0, 0, 0, 0)
    assert not f.on_fix(9.0, 1.0, 1.0, 0.0, now=10.0)[0]     # 1 s 전은 버퍼 밖


def test_alive_and_timeout():
    f = PoseFuser(fix_timeout=15.0)
    assert not f.alive(0.0)
    f.add_odom(0.0, 0, 0, 0)
    f.on_fix(0.0, 0, 0, 0, now=0.0)
    assert f.alive(14.9) and not f.alive(15.1)
    assert f.fix_age(3.0) == 3.0
