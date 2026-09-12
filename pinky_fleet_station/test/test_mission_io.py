"""mission.yaml 파서/검증 단위 테스트 (ROS 없이 돈다)."""

import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_station.mission_io import (  # noqa: E402
    Mission, MissionError, load_mission, save_mission,
)

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE = os.path.join(PKG_ROOT, 'config', 'mission.yaml')


def minimal(**overrides):
    data = {
        'map': {'yaml_path': '/tmp/map.yaml'},
        'robots': [
            {'name': 'pinky1', 'domain_id': 11,
             'goal': {'x': 1.0, 'y': 2.0, 'yaw': 0.0}},
            {'name': 'pinky2', 'domain_id': 12,
             'goal': {'x': -1.0, 'y': 0.5, 'yaw': 1.0}},
        ],
    }
    data.update(overrides)
    return data


def test_example_mission_loads():
    mission = load_mission(EXAMPLE)
    assert [r['name'] for r in mission.robots] == ['pinky1', 'pinky2']


def test_priority_is_ascending_domain_id():
    mission = Mission(minimal(robots=[
        {'name': 'b', 'domain_id': 12},
        {'name': 'a', 'domain_id': 11},
    ]))
    assert [r['name'] for r in mission.by_priority()] == ['a', 'b']


def test_defaults_applied_and_overridden():
    mission = Mission(minimal(
        defaults={'max_linear_vel': 0.3},
        robots=[
            {'name': 'a', 'domain_id': 1, 'max_linear_vel': 0.1},
            {'name': 'b', 'domain_id': 2},
        ]))
    assert mission.robot('a')['max_linear_vel'] == 0.1
    assert mission.robot('b')['max_linear_vel'] == 0.3
    # defaults 에 없는 항목은 내장 기본값으로 채워진다.
    assert mission.robot('b')['max_angular_vel'] == 1.5


def test_default_topics_follow_robot_name():
    mission = Mission(minimal())
    assert mission.robot('pinky1')['state_topic'] == '/pinky1/state'
    assert mission.robot('pinky1')['command_topic'] == '/pinky1/command'


def test_coordinator_defaults_filled():
    mission = Mission(minimal())
    assert mission.coordinator['stall_duration'] == 3.0
    assert mission.coordinator['clear_distance'] > mission.coordinator['conflict_distance']


def test_missing_domain_id_is_error():
    with pytest.raises(MissionError):
        Mission(minimal(robots=[{'name': 'a'}, {'name': 'b', 'domain_id': 2}]))


def test_duplicate_domain_id_is_error():
    with pytest.raises(MissionError):
        Mission(minimal(robots=[
            {'name': 'a', 'domain_id': 5},
            {'name': 'b', 'domain_id': 5},
        ]))


def test_duplicate_name_is_error():
    with pytest.raises(MissionError):
        Mission(minimal(robots=[
            {'name': 'a', 'domain_id': 1},
            {'name': 'a', 'domain_id': 2},
        ]))


def test_single_robot_is_error():
    with pytest.raises(MissionError):
        Mission(minimal(robots=[{'name': 'a', 'domain_id': 1}]))


def test_non_numeric_pose_is_error():
    with pytest.raises(MissionError):
        Mission(minimal(robots=[
            {'name': 'a', 'domain_id': 1, 'goal': {'x': 'left', 'y': 0, 'yaw': 0}},
            {'name': 'b', 'domain_id': 2},
        ]))


def test_missing_file_is_error():
    with pytest.raises(MissionError):
        load_mission('/nonexistent/mission.yaml')


def test_round_trip_preserves_values(tmp_path):
    mission = load_mission(EXAMPLE)
    mission.robot('pinky1')['goal'] = {'x': 3.25, 'y': -1.5, 'yaw': 0.75}
    target = tmp_path / 'out.yaml'
    save_mission(mission, str(target))

    raw = yaml.safe_load(target.read_text(encoding='utf-8'))
    assert raw['robots'][0]['goal']['x'] == 3.25

    reloaded = load_mission(str(target))
    assert reloaded.robot('pinky1')['goal'] == {'x': 3.25, 'y': -1.5, 'yaw': 0.75}
    # 건드리지 않은 값은 그대로 (도메인 ID 를 바꿔도 깨지지 않도록 원본과 비교)
    original = load_mission(EXAMPLE)
    assert reloaded.robot('pinky2')['domain_id'] == original.robot('pinky2')['domain_id']
    assert reloaded.robot('pinky2')['goal'] == original.robot('pinky2')['goal']
