"""lane_target — 쌍 선택 · SINGLE 추정 · 횡단보도 디바운스 · 부호 규약 (ROS 불필요)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_lane_station.lane_target import (  # noqa: E402
    QUALITY_BOTH, QUALITY_LOST, QUALITY_SINGLE, CrosswalkDebounce, Instance,
    LaneTargetEstimator, TargetParams, polygon_row_span)

W, H = 640, 480


def line(x, conf=0.9, top=0.45, bottom=1.0):
    return Instance('lane', conf, [(x - 8, H * top), (x + 8, H * top),
                                   (x + 12, H * bottom), (x - 12, H * bottom)])


def crosswalk(bottom_y, conf=0.9, x0=100, x1=540):
    return Instance('crosswalk', conf, [(x0, bottom_y - 40), (x1, bottom_y - 40),
                                        (x1, bottom_y), (x0, bottom_y)])


def test_row_span_of_rectangle():
    poly = [(10, 10), (30, 10), (30, 50), (10, 50)]
    assert polygon_row_span(poly, 20) == (10, 30)
    assert polygon_row_span(poly, 60) is None


def test_both_lanes_center_gives_zero_error():
    est = LaneTargetEstimator()
    r = est.update([line(200), line(440)], W, H)
    assert r.quality == QUALITY_BOTH
    assert r.left_seen and r.right_seen
    assert abs(r.error_x) < 0.02
    assert abs(r.half_lane_px - 120) < 15


def test_lane_center_right_gives_positive_error():
    """차선 중앙이 화면 오른쪽 = 로봇이 왼쪽 치우침 → error_x 양수 (LanePath 규약)."""
    est = LaneTargetEstimator()
    r = est.update([line(260), line(500)], W, H)
    assert r.error_x > 0.1


def test_lane_center_left_gives_negative_error():
    est = LaneTargetEstimator()
    r = est.update([line(140), line(380)], W, H)
    assert r.error_x < -0.1


def test_nearest_pair_to_center_is_chosen():
    """옆 차선까지 보여도 화면 중앙에 가장 가까운 쌍을 잡는다."""
    est = LaneTargetEstimator()
    r = est.update([line(40), line(200), line(440), line(600)], W, H)
    assert r.quality == QUALITY_BOTH
    assert r.left_x == 200 and r.right_x == 440


def test_single_left_uses_half_lane_history():
    est = LaneTargetEstimator()
    for _ in range(5):
        est.update([line(200), line(440)], W, H)          # half = 120
    r = est.update([line(230)], W, H)
    assert r.quality == QUALITY_SINGLE
    assert r.left_seen and not r.right_seen
    assert abs(r.target_x - 350) <= 2
    assert r.error_x > 0


def test_single_right_uses_initial_half_before_history():
    est = LaneTargetEstimator(TargetParams(half_lane_px_init=100))
    r = est.update([line(400)], W, H)
    assert r.quality == QUALITY_SINGLE
    assert r.right_seen and not r.left_seen
    assert abs(r.target_x - 300) <= 2


def test_no_lane_is_lost_with_zero_error():
    est = LaneTargetEstimator()
    r = est.update([], W, H)
    assert r.quality == QUALITY_LOST
    assert r.error_x == 0.0
    assert r.lane_count == 0


def test_narrow_pair_is_merged_as_single():
    """한 선이 조각나 중앙 양쪽에 걸치면 두 차선으로 오인하지 않는다."""
    est = LaneTargetEstimator(TargetParams(half_lane_px_init=120))
    r = est.update([line(310), line(330)], W, H)
    assert r.quality == QUALITY_SINGLE


def test_low_confidence_and_tiny_instances_ignored():
    est = LaneTargetEstimator()
    r = est.update([line(200, conf=0.1), line(440, top=0.98)], W, H)
    assert r.quality == QUALITY_LOST


def test_half_lane_is_median_and_clamped():
    est = LaneTargetEstimator(TargetParams(half_lane_px_max=200))
    for _ in range(4):
        est.update([line(200), line(440)], W, H)
    est.update([line(50), line(600)], W, H)               # 275 > max → 이력에 안 들어감
    assert abs(est.half_lane_px - 120) < 15


def test_crosswalk_debounce_confirm_and_release():
    d = CrosswalkDebounce(confirm=3, release=5)
    assert [d.update(True) for _ in range(3)] == [False, False, True]
    assert [d.update(False) for _ in range(5)] == [True, True, True, True, False]
    d.update(True)
    d.update(False)                 # 중간에 끊기면 카운트 리셋
    assert [d.update(True) for _ in range(3)] == [False, False, True]


def test_crosswalk_triggers_only_near_bottom():
    est = LaneTargetEstimator(TargetParams(crosswalk_stop_row_frac=0.80, crosswalk_confirm=1))
    far = est.update([line(200), line(440), crosswalk(300)], W, H)
    assert far.crosswalk_bottom_y == 300 and not far.crosswalk_raw and not far.crosswalk_detected
    near = est.update([line(200), line(440), crosswalk(400)], W, H)
    assert near.crosswalk_raw and near.crosswalk_detected


def test_crosswalk_requires_width():
    est = LaneTargetEstimator(TargetParams(crosswalk_confirm=1))
    r = est.update([crosswalk(450, x0=300, x1=340)], W, H)
    assert not r.crosswalk_raw


# ------------------------------------------------ D14 작업 1: 차선 ≥ 3 → 클래스 기준 가장 바깥 쌍

def sided(x, side, conf=0.9):
    cls = {'L': 'left_lane', 'R': 'right_lane', None: 'lane'}[side]
    return Instance(cls, conf, [(x - 8, H * 0.45), (x + 8, H * 0.45), (x + 12, H), (x - 12, H)])


def test_two_lanes_use_nearest_rule_even_with_labels():
    r = LaneTargetEstimator().update([sided(200, 'L'), sided(440, 'R')], W, H)
    assert r.quality == QUALITY_BOTH and r.pair_rule == 'nearest'
    assert (r.left_x, r.right_x) == (200, 440) and (r.left_count, r.right_count) == (1, 1)


def test_left_one_right_two_picks_farthest_right():
    """사용자 예: 왼쪽 1 + 오른쪽 2 → 왼쪽 그대로, 오른쪽은 왼쪽과 가장 먼 것."""
    r = LaneTargetEstimator().update([sided(200, 'L'), sided(400, 'R'), sided(560, 'R')], W, H)
    assert r.quality == QUALITY_BOTH and r.pair_rule == 'outer'
    assert (r.left_x, r.right_x) == (200, 560) and (r.left_count, r.right_count) == (1, 2)


def test_left_two_right_one_picks_farthest_left():
    r = LaneTargetEstimator().update([sided(80, 'L'), sided(240, 'L'), sided(440, 'R')], W, H)
    assert r.pair_rule == 'outer' and (r.left_x, r.right_x) == (80, 440)


def test_two_and_two_picks_outermost_both():
    r = LaneTargetEstimator().update([sided(60, 'L'), sided(200, 'L'), sided(440, 'R'), sided(600, 'R')], W, H)
    assert r.pair_rule == 'outer' and (r.left_x, r.right_x) == (60, 600)


def test_outer_rule_falls_back_to_position_when_labels_contradict():
    """오른쪽 클래스 선이 왼쪽 클래스 선보다 왼쪽에 있으면 위치 규칙."""
    r = LaneTargetEstimator().update([sided(500, 'L'), sided(150, 'R'), sided(300, 'R')], W, H)
    assert r.pair_rule == 'nearest'
    assert (r.left_x, r.right_x) == (300, 500)


def test_outer_rule_without_labels_uses_nearest():
    r = LaneTargetEstimator().update([line(40), line(200), line(440), line(600)], W, H)
    assert r.pair_rule == 'nearest' and (r.left_x, r.right_x) == (200, 440)


def test_outer_rule_one_side_unlabeled_uses_position_for_that_side():
    r = LaneTargetEstimator().update([sided(200, 'L'), sided(440, None), sided(560, None)], W, H)
    assert r.pair_rule == 'outer' and (r.left_x, r.right_x) == (200, 440)


# ------------------------------------------------ D14 작업 5: 바리게이트

def barricade(bottom_y, conf=0.9, x0=200, x1=440):
    return Instance('barricade', conf, [(x0, bottom_y - 60), (x1, bottom_y - 60),
                                        (x1, bottom_y), (x0, bottom_y)])


def test_barricade_triggers_only_near_bottom_and_debounces():
    est = LaneTargetEstimator(TargetParams(barricade_confirm=2, barricade_release=3))
    r = est.update([line(200), line(440), barricade(300)], W, H)
    assert r.barricade_bottom_y == 300 and not r.barricade_raw and not r.barricade_detected
    r = est.update([line(200), line(440), barricade(400)], W, H)
    assert r.barricade_raw and not r.barricade_detected          # 1/2
    r = est.update([line(200), line(440), barricade(400)], W, H)
    assert r.barricade_detected                                  # 확정
    for i in range(3):
        r = est.update([line(200), line(440)], W, H)
        assert r.barricade_detected == (i < 2)                   # 3 프레임 뒤 해제
    assert r.quality == QUALITY_BOTH                             # 차선 판정에는 영향 없음


def test_barricade_requires_width_and_conf():
    est = LaneTargetEstimator(TargetParams(barricade_confirm=1))
    assert not est.update([barricade(420, x0=300, x1=340)], W, H).barricade_raw
    assert not est.update([barricade(420, conf=0.1)], W, H).barricade_raw
    assert est.update([barricade(420)], W, H).barricade_detected


# ------------------------------------------------------------ 빨간 선 (2026-09-29 교차로 규칙)

def red_line(bottom_y, conf=0.9, x0=180, x1=460):
    return Instance('red_line', conf, [(x0, bottom_y - 12), (x1, bottom_y - 12),
                                        (x1, bottom_y), (x0, bottom_y)])


def test_red_line_triggers_near_bottom_debounces_and_does_not_touch_lanes():
    est = LaneTargetEstimator(TargetParams(red_line_confirm=2, red_line_release=3))
    r = est.update([line(200), line(440), red_line(300)], W, H)
    assert r.red_line_bottom_y == 300 and not r.red_line_raw and not r.red_line_detected
    r = est.update([line(200), line(440), red_line(400)], W, H)
    assert r.red_line_raw and not r.red_line_detected            # 1/2
    r = est.update([line(200), line(440), red_line(400)], W, H)
    assert r.red_line_detected and r.red_line_confidence == 0.9   # 확정
    for i in range(3):
        r = est.update([line(200), line(440)], W, H)
        assert r.red_line_detected == (i < 2)                     # 3 프레임 뒤 해제
    assert r.quality == QUALITY_BOTH and not r.crosswalk_detected and not r.barricade_detected


def test_red_line_requires_width_and_conf_and_reset_clears_it():
    est = LaneTargetEstimator(TargetParams(red_line_confirm=1))
    r = est.update([line(200), line(440), red_line(400, x0=300, x1=340)], W, H)   # 좁은 조각
    assert not r.red_line_raw and not r.red_line_detected
    r = est.update([line(200), line(440), red_line(400, conf=0.1)], W, H)         # 낮은 conf
    assert not r.red_line_raw
    r = est.update([line(200), line(440), red_line(400)], W, H)
    assert r.red_line_detected
    est.reset()
    assert not est.update([line(200), line(440)], W, H).red_line_detected
