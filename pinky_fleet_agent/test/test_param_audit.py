"""Nav2 파라미터 감사 로직 검사 (ROS 불필요).

실기에서 Nav2 가 우리 파일이 아닌 upstream nav2_params.yaml 로 떠 있었는데도 아무
증상이 없었다. 그걸 잡으려고 만든 게 param_audit 이다. 그러니 최소한
**"upstream 기본값으로 떠 있으면 잡는다"** 는 것만은 확실해야 한다.
"""

import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent import param_audit  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FLEET = os.path.join(REPO, 'pinky_fleet_agent', 'params', 'nav2_params_fleet.yaml')

# upstream pinky_navigation/params/nav2_params.yaml 의 값 (실기 덤프와 일치했던 것들).
# 저장소에 upstream 이 없으므로 감시 대상 값만 여기 적어 둔다.
UPSTREAM = {
    ('bt_navigator', 'robot_base_frame'): 'base_link',
    ('controller_server', 'FollowPath.max_lookahead_dist'): 0.9,
    ('controller_server', 'general_goal_checker.xy_goal_tolerance'): 0.25,
    ('planner_server', 'GridBased.tolerance'): 0.5,
    ('global_costmap/global_costmap', 'footprint_padding'): 0.03,
    ('local_costmap/local_costmap', 'width'): 3,
}


@pytest.fixture(scope='module')
def fleet_params():
    with open(FLEET, encoding='utf-8') as handle:
        return yaml.safe_load(handle)


@pytest.fixture(scope='module')
def expected(fleet_params):
    return param_audit.expected_values(fleet_params)


def test_every_sentinel_exists_in_the_fleet_file(expected):
    """감시 대상이 파일에서 사라지면 감사가 조용히 무력해진다."""
    wanted = {(node, name) for node, _, name in param_audit.SENTINELS}
    assert set(expected) == wanted, f'파일에서 못 찾은 값: {sorted(wanted - set(expected))}'


def test_sentinels_differ_from_upstream(expected):
    """우리 값과 upstream 값이 같으면 그 항목으로는 아무것도 구분할 수 없다."""
    same = [key for key, value in expected.items()
            if key in UPSTREAM and param_audit.same(value, UPSTREAM[key])]
    assert not same, f'upstream 과 값이 같아 판별에 쓸모없는 항목: {same}'


def test_upstream_params_are_detected(expected):
    """실기에서 실제로 벌어진 상황. 전부 어긋난 것으로 보고돼야 한다."""
    bad = param_audit.compare(expected, UPSTREAM)
    assert len(bad) == len(expected), bad
    assert param_audit.report(bad, []) is not None


def test_matching_params_report_nothing(expected):
    assert param_audit.compare(expected, dict(expected)) == []
    assert param_audit.report([], []) is None


def test_single_mismatch_is_reported(expected):
    actual = dict(expected)
    actual[('bt_navigator', 'robot_base_frame')] = 'base_link'
    bad = param_audit.compare(expected, actual)
    assert len(bad) == 1
    assert 'robot_base_frame' in bad[0]
    assert 'base_link' in bad[0]


def test_unread_params_are_not_mismatches(expected):
    """노드가 아직 안 떠서 못 읽은 것과, 읽었는데 다른 것은 다른 문제다."""
    partial = {key: value for key, value in list(expected.items())[:1]}
    assert param_audit.compare(expected, partial) == []
    unread = param_audit.missing(expected, partial)
    assert len(unread) == len(expected) - 1
    text = param_audit.report([], unread)
    assert text is not None and '읽지 못한' in text


def test_float_comparison_tolerates_representation_noise():
    assert param_audit.same(0.1, 0.1 + 1e-12)
    assert not param_audit.same(0.1, 0.05)


def test_int_and_float_compare_by_value():
    """width 는 파일에서 int, 파라미터로는 int 로 오지만 섞여도 맞아야 한다."""
    assert param_audit.same(2, 2.0)
    assert not param_audit.same(2, 3)


def test_bool_is_not_treated_as_a_number():
    """파이썬에서 True == 1 이라, 타입이 어긋난 걸 조용히 통과시키기 쉽다."""
    assert param_audit.same(True, True)
    assert param_audit.same(False, False)
    assert not param_audit.same(True, 1)
    assert not param_audit.same(1, True)
    assert not param_audit.same(False, 0)
    assert not param_audit.same(True, False)


def test_report_names_the_likely_causes(expected):
    """로그 한 줄만 보고 무엇을 확인할지 알 수 있어야 한다."""
    text = param_audit.report(param_audit.compare(expected, UPSTREAM), [])
    for hint in ('robot.launch.xml', 'params_file', 'colcon build', 'rqt'):
        assert hint in text, hint


def test_default_params_path_points_at_the_fleet_file():
    path = param_audit.default_params_path()
    assert path and os.path.samefile(path, FLEET)


def test_dig_returns_none_for_missing_paths():
    assert param_audit.dig({'a': {'b': 1}}, ('a', 'b')) == 1
    assert param_audit.dig({'a': {'b': 1}}, ('a', 'c')) is None
    assert param_audit.dig({'a': 1}, ('a', 'b')) is None
