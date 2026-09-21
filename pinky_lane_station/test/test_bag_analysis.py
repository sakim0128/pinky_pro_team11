"""bag_analysis — 합성 시계열로 급변·무효·갭·속도 불일치·전이 검출과 오탐 없음 (ROS 불필요)."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_lane_station.bag_analysis import (  # noqa: E402
    dwell, front_min, gap_events, histogram, rate_stats, scalar_jumps, scan_events, transitions,
    velocity_mismatch)


def test_rate_stats_and_gap_detection():
    ts = [i * 0.1 for i in range(50)] + [5.0 + 2.0 + i * 0.1 for i in range(20)]   # 2 s 끊김
    st = rate_stats(ts)
    assert st['count'] == 70 and abs(st['mean_period'] - (ts[-1] / 69)) < 1e-9
    assert len(st['gaps']) == 1 and abs(st['gaps'][0][1] - 2.1) < 1e-9
    assert [e.kind for e in gap_events(ts, 'camera')] == ['camera_gap']
    assert rate_stats([1.0])['count'] == 1
    assert not rate_stats([i * 0.1 for i in range(100)])['gaps']            # 균일하면 갭 없음


def scan(dist, n=360, invalid_front=0):
    angle_min, inc = -math.pi, 2 * math.pi / n
    ranges = [3.0] * n
    for i in range(n):
        a = angle_min + i * inc
        if abs(math.atan2(math.sin(a), math.cos(a))) <= math.radians(35):
            ranges[i] = dist
    if invalid_front:
        k = 0
        for i in range(n):
            a = angle_min + i * inc
            if abs(math.atan2(math.sin(a), math.cos(a))) <= math.radians(35) and k < invalid_front:
                ranges[i] = float('nan')
                k += 1
    return ranges, angle_min, inc


def test_front_min_and_scan_events():
    m, valid, total = front_min(*scan(0.8))
    assert abs(m - 0.8) < 1e-9 and valid == total > 0
    fronts, t = [], []
    for i, d in enumerate([1.0, 1.0, 0.98, 0.3, 0.98, 1.0]):              # 0.3 으로 한 프레임 튐
        fronts.append(front_min(*scan(d)))
        t.append(i * 0.1)
    ev = scan_events(t, fronts)
    kinds = [e.kind for e in ev]
    assert kinds == ['scan_jump', 'scan_jump'] and abs(ev[0].t - 0.3) < 1e-9
    # 무효 급증
    fr = [front_min(*scan(1.0)), front_min(*scan(1.0, invalid_front=60)), front_min(*scan(1.0, invalid_front=70))]
    ev = scan_events([0, 0.1, 0.2], fr)
    assert 'scan_invalid' in [e.kind for e in ev] and 'scan_jump' not in [e.kind for e in ev]
    # 전부 무효
    r, a, i = scan(1.0)
    ev = scan_events([0.0], [front_min([float('nan')] * len(r), a, i)])
    assert {e.kind for e in ev} >= {'scan_empty', 'scan_invalid'}


def test_scalar_jumps_ultrasonic():
    t = [i * 0.1 for i in range(30)]
    v = [0.50] * 30
    v[15] = 0.05                     # 튐
    v[20] = float('nan')             # 무효
    ev = scalar_jumps(t, v, thresh=0.15, label='us')
    assert [(e.kind, round(e.t, 1)) for e in ev] == [('us_jump', 1.5), ('us_invalid', 2.0)]
    assert not scalar_jumps(t, [0.5 + 0.01 * math.sin(i) for i in range(30)], 0.15, label='us')


def test_velocity_mismatch_detects_stall_not_lag():
    t_cmd = [i * 0.05 for i in range(200)]
    v_cmd = [0.15] * 200
    t_odom = [i * 0.05 for i in range(200)]
    v_odom = [0.15] * 200
    assert not velocity_mismatch(t_cmd, v_cmd, t_odom, v_odom)
    v_odom[100:160] = [0.0] * 60                                    # 3 s 동안 바퀴가 안 돎
    ev = velocity_mismatch(t_cmd, v_cmd, t_odom, v_odom)
    assert len(ev) == 1 and ev[0].kind == 'vel_mismatch' and abs(ev[0].t - 5.0) < 1e-6 and ev[0].value < -0.1
    assert '3.0s' in ev[0].detail
    # 명령이 바뀌고 odom 이 0.3 s 늦게 따라가는 정상 지연은 잡지 않는다
    v_cmd2 = [0.0] * 100 + [0.15] * 100
    v_odom2 = [0.0] * 106 + [0.15] * 94
    assert not velocity_mismatch(t_cmd, v_cmd2, t_odom, v_odom2)


def test_transitions_dwell_histogram():
    t = [0, 1, 2, 3, 4, 5]
    s = [0, 1, 1, 3, 3, 1]
    names = {0: 'IDLE', 1: 'CRUISE', 3: 'CROSSWALK_STOP'}
    assert transitions(t, s, names) == [(1, 'IDLE', 'CRUISE'), (3, 'CRUISE', 'CROSSWALK_STOP'), (5, 'CROSSWALK_STOP', 'CRUISE')]
    d = dwell(t, s, names)
    assert d['CRUISE'] == 2 and d['CROSSWALK_STOP'] == 2 and d['IDLE'] == 1
    assert histogram(s, names) == {'IDLE': 1, 'CRUISE': 3, 'CROSSWALK_STOP': 2}
