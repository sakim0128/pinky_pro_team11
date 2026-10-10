"""비전 미션 위치 표시 — 코스 파일(vision_course.yaml) 불변식과 추정기(PoseTracker): odom 추측항법 + 확실한 지점에서 다시 맞추기."""

import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.dirname(PKG)
sys.path.insert(0, PKG)

from pinky_lane_station.vision_mission import load_vision_config  # noqa: E402
from pinky_lane_station.vision_pose import (  # noqa: E402
    DRIVE_ARRIVED, DRIVE_CROSSWALK_STOP, DRIVE_CRUISE, DRIVE_JUNCTION_PASS, DRIVE_JUNCTION_STOP, DRIVE_RED_LINE_STOP,
    PoseTracker, load_vision_course,
)

COURSE = os.path.join(PKG, 'config', 'vision_course.yaml')
ROUTES = [('2', '1'), ('1', '2'), ('3', '1'), ('1', '3'), ('2', '3'), ('3', '2')]


@pytest.fixture(scope='module')
def course():
    return load_vision_course(COURSE)


def test_png_to_map_matches_map5(course):
    # 그림의 검은 테두리 안쪽 모서리 = map5 맵 가장자리
    x, y = course.px_to_map(37, 16)
    assert (x, y) == pytest.approx((-0.01, 1.27))
    x, y = course.px_to_map(37 + 236 * 4.0125, 16 + 128 * 4.023)
    assert (x, y) == pytest.approx((2.35, -0.01))
    px = course.map_to_px(1.0, 0.5)
    assert course.px_to_map(*px) == pytest.approx((1.0, 0.5))
    assert os.path.isfile(os.path.join(REPO, course.image_file))


def test_every_scenario_route_exists_and_is_on_free_floor(course):
    cv2 = pytest.importorskip('cv2')
    pgm = cv2.imread(os.path.join(REPO, 'pinky_fleet_station', 'config', 'maps', 'map5.pgm'), 0)
    cfg = load_vision_config(os.path.join(PKG, 'config', 'vision_mission.yaml'))
    assert set(cfg.routes) == set(ROUTES)
    for a, b in ROUTES:
        r = course.route(a, b)
        assert 2.5 < r.length < 5.0
        assert 0.3 < r.s_entry < r.s_exit < r.s_goal_end < r.length
        assert 0.4 < r.s_exit - r.s_entry < 1.0                         # 교차로 안 (빨간 선 사이)
        for x, y in r.points[::5]:
            i, j = int((x + 0.01) / 0.01), int(128 - (y + 0.01) / 0.01)
            assert 0 <= i < 236 and 0 <= j < 128
            assert pgm[j, i] != 0 or r.length - r.cum[r.points.index((x, y))] < 0.06, (a, b, x, y)   # 벽 앞 마커만 예외
        # 횡단보도는 3 번 가지에만
        assert bool(r.crosswalks) == ('3' in (a, b))


def test_tracker_starts_at_start_point_and_ignores_missing_odom(course):
    r = course.route('2', '1')
    tr = PoseTracker(r)
    p = tr.pose()
    sx, sy = course.px_to_map(92, 100)
    assert p['x'] == pytest.approx(sx, abs=0.01) and p['y'] == pytest.approx(sy, abs=0.01)
    assert abs(p['yaw']) < 0.1                                           # 2 번에서 오른쪽(+x) 으로 출발
    tr.feed_odom(5.0, 5.0, 0.0)
    tr.feed_odom(5.9, 5.0, 0.0)                                          # 0.9 m 튐 — 버린다
    assert tr.s == 0.0


def _drive(course, a, b, scale=1.02, yaw_drift=0.02, v=0.10, dt=0.1):
    """참 경로를 따라 달리는 로봇 + 틀어진 odom(거리 2 % 과대, 방향 0.02 rad/s ≈ 1.1°/s 흐름). 정지 이벤트를 넣고 표시 오차를 잰다.
    가장 큰 오차는 교차로 동작(free, 2D odom) 끝 무렵 — 방향 흐름이 그대로 쌓인다."""
    r = course.route(a, b)
    tr = PoseTracker(r)
    stops = [(r.s_entry - course.red_stop_back, DRIVE_JUNCTION_STOP, 'vision:approach'),
             (r.s_exit - course.red_stop_back, DRIVE_RED_LINE_STOP, 'vision:after_junction')]
    stops += [(c[0] - course.crosswalk_stop_back, DRIVE_CROSSWALK_STOP, '') for c in r.crosswalks]
    stops.sort()
    goal_s = r.length - course.marker_stop_back
    ox, oy, oyaw = 1.0, -2.0, 0.7                                      # odom 원점은 아무 데나
    s, t, errs, after_fix = 0.0, 0.0, [], []
    prev_heading = r.point_at(0)[2]
    tr.feed_odom(ox, oy, oyaw, t)
    tr.feed_status(DRIVE_CRUISE, 'vision:approach')
    stage = 'vision:approach'
    while s < goal_s:
        ns = min(goal_s, s + v * dt)
        x0, y0, h0 = r.point_at(s)
        x1, y1, h1 = r.point_at(ns)
        step = math.hypot(x1 - x0, y1 - y0) * scale
        oyaw += (h1 - prev_heading) + yaw_drift * dt
        oyaw = math.atan2(math.sin(oyaw), math.cos(oyaw))
        prev_heading = h1
        ox += step * math.cos(oyaw)
        oy += step * math.sin(oyaw)
        s, t = ns, t + dt
        tr.feed_odom(ox, oy, oyaw, t)
        for s_stop, st, stg in stops:
            if abs(s - s_stop) < v * dt / 2 + 1e-9:
                if st == DRIVE_RED_LINE_STOP:
                    tr.feed_status(DRIVE_JUNCTION_PASS, 'vision:junction')   # (이미 교차로 동작 중)
                stage = stg or stage
                tr.feed_status(st, stage)
                p = tr.pose()
                after_fix.append(math.hypot(p['x'] - x1, p['y'] - y1))
                tr.feed_status(DRIVE_JUNCTION_PASS if st == DRIVE_JUNCTION_STOP else DRIVE_CRUISE,
                               'vision:junction' if st == DRIVE_JUNCTION_STOP else stage)
        p = tr.pose()
        errs.append(math.hypot(p['x'] - x1, p['y'] - y1))
    tr.feed_status(DRIVE_ARRIVED, 'vision:arrived', '도착 — 벽 마커 40 0.15 m')
    p = tr.pose()
    return errs, after_fix, (p['x'] - r.point_at(goal_s)[0], p['y'] - r.point_at(goal_s)[1]), tr


@pytest.mark.parametrize('a, b', ROUTES)
def test_display_error_stays_within_5cm_with_drifting_odom(course, a, b):
    errs, after_fix, end_err, tr = _drive(course, a, b)
    assert max(errs) < 0.05, max(errs)
    assert all(e < 0.01 for e in after_fix)                             # 다시 맞춘 순간은 1 cm 안
    assert math.hypot(*end_err) < 0.01 and '마커' in tr.fix


def test_junction_pass_draws_2d_odom_then_projects_back(course):
    r = course.route('2', '3')
    tr = PoseTracker(r)
    tr.feed_odom(0.0, 0.0, 0.0)
    tr.feed_status(DRIVE_JUNCTION_STOP, 'vision:approach')
    entry = tr.pose()
    tr.feed_status(DRIVE_JUNCTION_PASS, 'vision:junction')
    # 20 cm 전진 뒤 제자리 우회전 90° — 그림에서도 돈다
    for k in range(1, 21):
        tr.feed_odom(0.01 * k, 0.0, 0.0)
    for k in range(1, 31):
        tr.feed_odom(0.20, 0.0, -math.radians(3 * k))
    p = tr.pose()
    assert p['mode'] == 'free'
    assert math.hypot(p['x'] - entry['x'], p['y'] - entry['y']) == pytest.approx(0.20, abs=0.005)
    assert math.atan2(math.sin(p['yaw'] - entry['yaw'] + math.pi / 2), math.cos(p['yaw'] - entry['yaw'] + math.pi / 2)) \
        == pytest.approx(0.0, abs=0.01)
    tr.feed_status(DRIVE_RED_LINE_STOP, 'vision:after_junction')
    assert tr.pose()['mode'] == 'route' and '나가는' in tr.fix
    assert tr.s == pytest.approx(r.s_exit - course.red_stop_back)


def test_leaving_junction_without_red_stop_projects_onto_route(course):
    r = course.route('2', '1')
    tr = PoseTracker(r)
    tr.feed_odom(0.0, 0.0, 0.0)
    tr.feed_status(DRIVE_JUNCTION_STOP, 'vision:approach')
    tr.feed_status(DRIVE_JUNCTION_PASS, 'vision:junction')
    for k in range(1, 41):
        tr.feed_odom(0.01 * k, 0.0, 0.0)
    tr.feed_status(DRIVE_CRUISE, 'vision:after_junction')
    assert tr.mode == 'route'
    assert tr.s == pytest.approx(r.s_entry - course.red_stop_back + 0.40, abs=0.02)


def test_course_dict_for_web(course):
    d = course.to_dict()
    assert d['image'] == 'docs/map5.png' and len(d['lines']) == 6
    assert set(d['points']) == {'1', '2', '3'} and set(d['red_lines']) == {'A', 'B', 'C'}


def test_seek_end_without_exit_stop_snaps_to_exit_line(course):
    # 나가는 빨간 선에서 서지 않는 설정: seek 이 끝나 바로 CRUISE 로 가도 나가는 선 앞으로 맞춘다
    r = course.route('2', '3')
    tr = PoseTracker(r)
    tr.feed_odom(0.0, 0.0, 0.0)
    tr.feed_status(DRIVE_JUNCTION_STOP, 'vision:approach')
    tr.feed_status(DRIVE_JUNCTION_PASS, 'vision:junction', '교차로 동작 1/1 빨간 선 찾기(우) 회전 12°')
    for k in range(1, 31):
        tr.feed_odom(0.0, 0.0, -math.radians(3 * k))
    tr.feed_status(DRIVE_JUNCTION_PASS, 'vision:junction', '교차로 동작 1/1 빨간 선 찾기(우) — 새 선 앞 도착')
    tr.feed_status(DRIVE_CRUISE, 'vision:after_junction', '교차로 통과 — 차선 주행')
    assert tr.mode == 'route' and '나가는' in tr.fix
    assert tr.s == pytest.approx(r.s_exit - course.red_stop_back)
    tr.feed_odom(0.0, -0.10, -math.pi / 2)                              # 이어서 10 cm
    assert tr.s == pytest.approx(r.s_exit - course.red_stop_back + 0.10, abs=0.005)
