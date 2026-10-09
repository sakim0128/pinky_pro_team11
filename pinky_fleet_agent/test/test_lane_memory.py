"""카메라 사각지대 보정 — 화면→바닥 거리 모델, 차선 중앙 기억, 기억 점 pure pursuit, 드라이버 폐루프 (lane_memory)."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent.drive_fsm import CROSSWALK_CLEAR, CROSSWALK_STOP, CRUISE, LANE_SEARCH  # noqa: E402
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


def bev_floor_target(pts, pose, axle, lookahead=0.25):
    """관제 BEV 가 주는 바닥 좌표 목표점: 구동축에서 lookahead 떨어진 차로 중앙 (전방, 왼쪽) m — 원점은 카메라 바로 아래 바닥."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    for px, py in pts:
        lx, ly = (px - x) * c + (py - y) * s, -(px - x) * s + (py - y) * c
        if lx > 0 and math.hypot(lx, ly) >= lookahead:
            return lx - axle, ly
    return None


def test_floor_target_is_remembered_with_axle_offset():
    p = DriverParams()
    p.lane_only = True
    d = LaneDriver(p)
    d.update_odom(1.0, 0.0, 0.0, 0.0, stamp=1.0)
    d.set_lane_path(1.0, 1.0, QUALITY_BOTH, 0.0, target=(320, 346, W, H, HALF_PX), floor=(0.20, -0.03))
    assert len(d.memory) == 1
    assert d.memory.points[0] == pytest.approx((0.20 + p.view.axle_to_camera_m, -0.03))


def test_memory_follows_curve_from_bev_floor_targets():
    """화면 픽셀 대신 BEV 바닥 좌표만 받아도 같은 커브를 돈다 (floor 가 있으면 GroundView 를 안 쓴다)."""
    p = DriverParams()
    p.lane_only = True
    p.control.v_max = 0.12
    d = LaneDriver(p)
    d.started = True
    pts = course()
    x, y, yaw, t, errs = 0.0, 0.02, 0.0, 0.0, []
    for k in range(int(14.0 / DT)):
        t += DT
        d.update_odom(t, x, y, yaw, stamp=t)
        if k % 2 == 0:
            fl = bev_floor_target(pts, (x, y, yaw), p.view.axle_to_camera_m)
            if fl is None:
                d.set_lane_path(t, t, QUALITY_LOST, None)
            else:
                d.set_lane_path(t, t, QUALITY_BOTH, 0.0, target=None, floor=fl)   # error_x 는 쓰이지 않는다
        out = d.tick(t, 0.0, 0.0, 0.0)
        x += out.v * math.cos(yaw) * DT
        y += out.v * math.sin(yaw) * DT
        yaw += out.omega * DT
        errs.append(lateral_error(pts, x, y))
        if y > 1.0:
            break
    assert y > 0.6                                  # 커브를 돌아 나갔다
    assert max(errs[60:]) < 0.03 and errs[-1] < 0.01   # 2 cm 치우친 출발에서 수렴


# ---------------------------------------------------------------- 튄 점 거르기 · 정지 중 기억 비우기 (2026-10-04 pinky2 횡단보도)

def straight_memory(n=6, y=0.0):
    m = LaneMemory(MemoryParams())
    for i in range(n):                                   # 로봇은 원점에 서 있고 앞 10~25 cm 에 중앙선
        assert m.add((0.10 + 0.03 * i, y), (0.0, 0.0, 0.0))
    return m


def test_single_frame_lateral_spike_is_not_remembered():
    m = straight_memory()
    assert not m.add((0.15, -0.15), (0.0, 0.0, 0.0))     # 같은 전방 거리의 기억보다 15 cm 오른쪽 — 보류
    assert m.add((0.26, 0.0), (0.0, 0.0, 0.0))           # 다음 프레임은 원래 자리 — 받고, 보류한 점은 버린다
    assert all(abs(q[1]) < 0.01 for q in m.points)
    assert m.target((0.0, 0.0, 0.0))[1] == pytest.approx(0.0, abs=0.01)


def test_two_frame_jump_is_still_rejected():
    """급커브 꼭짓점에서 BEV 가 2 프레임 연속 반대쪽을 낸 경우 (2026-10-04 pinky2 S자) — 받지 않는다."""
    m = straight_memory()
    assert not m.add((0.15, 0.14), (0.0, 0.0, 0.0))
    assert not m.add((0.16, 0.15), (0.0, 0.0, 0.0))
    assert m.add((0.26, 0.0), (0.0, 0.0, 0.0))
    assert all(abs(q[1]) < 0.01 for q in m.points)


def test_jump_confirmed_by_three_frames_replaces_points_ahead():
    m = straight_memory()
    assert not m.add((0.15, 0.10), (0.0, 0.0, 0.0))
    assert not m.add((0.17, 0.10), (0.0, 0.0, 0.0))
    assert m.add((0.19, 0.10), (0.0, 0.0, 0.0))           # 세 프레임 연속 같은 자리 — 장면이 바뀌었다
    ahead = [q for q in m.points if q[0] >= 0.10]
    assert ahead and all(abs(q[1] - 0.10) < 0.01 for q in ahead)


def moving_memory(params=None):
    """로봇이 x 축을 따라 가며 차로 중앙(y=0)을 본 기억 (0.10~0.40 m)."""
    m = LaneMemory(params or MemoryParams())
    for i in range(11):
        assert m.add((0.10 + 0.03 * i, 0.0), (0.0, 0.0, 0.0), stamp=0.0)
    return m


def test_jump_confirmed_while_robot_moves_6cm_per_frame():
    """목표점은 '로봇 앞 15 cm' 라 로봇이 6 cm 가면 6 cm 앞으로 간다 — 고정 4 cm 였으면 확인 실패. 이동 거리만큼 허용한다."""
    m = moving_memory()
    assert not m.add((0.15, 0.12), (0.00, 0.0, 0.0), stamp=0.0)
    assert not m.add((0.15, 0.12), (0.06, 0.0, 0.0), stamp=0.2)
    assert m.add((0.15, 0.12), (0.12, 0.0, 0.0), stamp=0.4)
    ahead = [q for q in m.points if q[0] >= 0.15]
    assert ahead and all(abs(q[1] - 0.12) < 0.01 for q in ahead)
    old = moving_memory(MemoryParams(jump_confirm=0.04))
    old._confirms = lambda a, b: math.hypot(b[0][0] - a[0][0], b[0][1] - a[0][1]) <= 0.04   # 예전 규칙 (고정 4 cm)
    for x, t in ((0.00, 0.0), (0.06, 0.2), (0.12, 0.4)):
        accepted = old.add((0.15, 0.12), (x, 0.0, 0.0), stamp=t)
    assert not accepted


def test_two_frame_spike_still_rejected_while_moving():
    m = moving_memory()
    assert not m.add((0.15, 0.14), (0.00, 0.0, 0.0), stamp=0.0)
    assert not m.add((0.15, 0.15), (0.03, 0.0, 0.0), stamp=0.2)
    assert m.add((0.25, 0.0), (0.06, 0.0, 0.0), stamp=0.4)        # 원래 자리로 돌아옴 — 보류 점은 버린다
    assert all(abs(q[1]) < 0.01 for q in m.points)


def test_stale_pending_point_is_not_counted():
    m = moving_memory()
    assert not m.add((0.15, 0.12), (0.0, 0.0, 0.0), stamp=0.0)
    assert not m.add((0.15, 0.12), (0.0, 0.0, 0.0), stamp=1.5)    # 1 s 넘게 지남 — 새로 센다
    assert not m.add((0.15, 0.12), (0.0, 0.0, 0.0), stamp=1.7)
    assert m.add((0.15, 0.12), (0.0, 0.0, 0.0), stamp=1.9)


def test_point_without_memory_at_that_distance_is_accepted():
    m = straight_memory(n=2)                               # 10·13 cm 에만 기억
    assert m.add((0.30, 0.08), (0.0, 0.0, 0.0))            # 30 cm 엔 비교할 기억이 없다 (공백 뒤 등) — 그냥 받는다


def test_driver_clears_memory_while_stopped_at_crosswalk():
    p = DriverParams()
    p.lane_only = True
    d = LaneDriver(p)
    d.started = True
    t = 0.0
    for _ in range(10):
        t += DT
        d.update_odom(t, 0.0, 0.0, 0.0, stamp=t)
        d.set_lane_path(t, t, QUALITY_BOTH, 0.0, floor=(0.20 + 0.01 * _, 0.0))
        d.tick(t, 0.0, 0.0, 0.0)
    assert len(d.memory) > 0
    t += DT
    d.update_odom(t, 0.0, 0.0, 0.0, stamp=t)
    d.set_lane_path(t, t, QUALITY_BOTH, 0.0, crosswalk=True, floor=(0.30, 0.0))
    out = d.tick(t, 0.0, 0.0, 0.0)
    assert out.state == CROSSWALK_STOP and out.v == 0.0 and len(d.memory) == 0


def replay_crosswalk_restart(params, name='crosswalk_restart_pinky2.json'):
    """실제 로그(test/data/*.json)의 odom·LanePath 를 그대로 넣는다(개루프). 기본은 2026-10-04 pinky2 오른쪽 커브 →
    횡단보도 3 s 정지 → 출발. -> [(t, state, omega)]"""
    import json
    doc = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', name)))
    d = LaneDriver(params)
    d.started = True
    lps = list(doc['lane_path_rows'])
    out, tick_t = [], None
    for stamp, x, y, yaw in doc['odom_rows']:
        d.update_odom(stamp, x, y, yaw, stamp=stamp)
        while lps and lps[0][0] <= stamp:
            recv, src, q, e, cw, fv, fx, fy = lps.pop(0)
            d.set_lane_path(recv, src, q, e if q in (0, 1) else None, bool(cw), floor=(fx, fy) if fv else None)
        if tick_t is None or stamp - tick_t >= 0.05:
            d.set_command(0, stamp)                           # 하트비트
            o = d.tick(stamp, 0.0, 0.0, 0.0)
            tick_t = stamp
            out.append((stamp, o.state, o.omega))
    return out


def restart_omegas(log, seconds=0.6):
    t_go = next(t for t, s, _ in log if s == CROSSWALK_CLEAR)
    return [w for t, s, w in log if t_go <= t < t_go + seconds]


def before_stop_omegas(log, seconds=0.6):
    t_stop = next(t for t, s, _ in log if s == CROSSWALK_STOP)
    return [w for t, s, w in log if t_stop - seconds <= t < t_stop]


def log_params(**memory):
    """현장과 같은 파라미터: params/lane_agent.yaml (예: fsm.search_on_single=false) + lane_only. memory 키워드로 덮어쓴다."""
    import yaml
    p = DriverParams()
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'params', 'lane_agent.yaml')
    ros = yaml.safe_load(open(path))['pinky_lane_agent']['ros__parameters']
    groups = {'control': p.control, 'fsm': p.fsm, 'guard': p.guard, 'maneuver': p.maneuver,
              'view': p.view, 'memory': p.memory, '': p}
    for prefix, obj in groups.items():
        src = ros.get(prefix, {}) if prefix else ros
        for k, v in vars(obj).items():
            if isinstance(v, (int, float, bool)) and k in src:
                setattr(obj, k, type(v)(src[k]))
    p.lane_only = True
    p.guard.use_lidar = False
    for k, v in memory.items():
        setattr(p.memory, k, v)
    return p


def test_log_crosswalk_restart_old_behaviour_turns_hard_right():
    """수정 전 재현: 정지 직전 튄 점 2개(차로 중앙 14~16 cm 오른쪽)가 기억에 남아 출발 직후 급우회전."""
    log = replay_crosswalk_restart(log_params(jump_reject=10.0, clear_on_stop=False, max_target_angle_deg=90.0))
    assert any(s == CROSSWALK_STOP for _, s, _ in log)
    assert min(restart_omegas(log)) < -0.3
    assert min(before_stop_omegas(log)) < -0.4                # 정지 직전에도 튄 점 쪽으로 한 번 꺾인다


def test_log_crosswalk_restart_follows_what_it_sees():
    """수정 후: 서서 본 '정면 직진' 을 따라 출발한다 (튄 점 거르기 + 정지 중 기억 비우기, 각각만으로도)."""
    for p in (log_params(), log_params(clear_on_stop=False), log_params(jump_reject=10.0)):
        assert min(restart_omegas(replay_crosswalk_restart(p))) > -0.05


def test_log_spike_before_stop_is_filtered():
    """튄 점 거르기: 정지 직전 튄 점 쪽으로 꺾던 것(−0.56 rad/s)이 없어진다 (현장 기록 −0.16~−0.18 과 비슷)."""
    assert min(before_stop_omegas(replay_crosswalk_restart(log_params()))) > -0.3


def scurve_omegas(params, t_from=0.5, t_to=1.4):
    """150416 pinky2 S자 오른쪽 급커브: BEV 가 2 프레임 연속 반대쪽(왼쪽)을 낸 뒤 t_from~t_to s 동안의 조향."""
    log = replay_crosswalk_restart(params, 'scurve_corner_pinky2.json')
    import json
    doc = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'scurve_corner_pinky2.json')))
    spike = next(r[0] for r in doc['lane_path_rows'] if r[5] and r[7] > 0.12)       # 첫 '왼쪽 12 cm 넘음' 수신
    return [w for t, s, w in log if spike + t_from <= t <= spike + t_to]


def test_log_scurve_two_frame_wrong_side_old_behaviour_turns_left():
    """수정 전 재현: 2 프레임 확인이면 반대쪽 점을 받아 오른쪽 커브 한가운데서 좌회전(+0.6 rad/s, 현장 기록과 같음)."""
    assert max(scurve_omegas(log_params(jump_confirm_count=1, max_target_angle_deg=90.0))) > 0.4


def test_log_scurve_keeps_turning_right():
    """수정 후: 3 프레임 확인 — 2 프레임 반대쪽 점을 버리고 오른쪽 커브를 이어 돈다."""
    assert max(scurve_omegas(log_params())) < 0.05


def test_target_ignores_point_beside_robot():
    """저속 급커브 뒤 로봇 바로 옆(전방 3 cm·오른쪽 14 cm)에 남은 옛 점은 목표가 아니다 (정면 ± 45° 밖)."""
    m = LaneMemory(MemoryParams())
    m.points.extend([(0.03, -0.14), (0.20, 0.02), (0.22, 0.03)])
    tx, ty = m.target((0.0, 0.0, 0.0))
    assert tx >= 0.19 and ty > 0.0
