"""타이밍 값들이 서로 어긋나지 않는지 검사한다 (ROS 불필요).

같은 값이 노드 기본값과 launch 인자 두 군데에 있는데 한쪽만 고쳐서 실제 동작이 문서와
달라진 적이 있다. launch 가 <param> 으로 명시 전달하므로 **launch 쪽이 이긴다** —
노드 기본값만 고치면 아무 효과가 없고, 아무도 눈치채지 못한다.

``test_setup_data_files.py`` 가 colcon 의 assertion 을 흉내 내 한 번 저지른 실수를 다시
잡는 것과 같은 목적이다.
"""

import os
import re

import pytest
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AGENT_PKG = os.path.join(REPO, 'pinky_fleet_agent')
STATION_PKG = os.path.join(REPO, 'pinky_fleet_station')

AGENT_NODE = os.path.join(AGENT_PKG, 'pinky_fleet_agent', 'agent_node.py')
AGENT_LAUNCH = os.path.join(AGENT_PKG, 'launch', 'agent.launch.xml')
ROBOT_LAUNCH = os.path.join(AGENT_PKG, 'launch', 'robot.launch.xml')
NAV2_PARAMS = os.path.join(AGENT_PKG, 'params', 'nav2_params_fleet.yaml')
MISSION_IO = os.path.join(STATION_PKG, 'pinky_fleet_station', 'mission_io.py')

MISSIONS = [
    os.path.join(STATION_PKG, 'config', 'mission.yaml'),
    os.path.join(STATION_PKG, 'config', 'mission_deadlock_test.yaml'),
]

# 노드 기본값과 launch 인자 기본값이 반드시 같아야 하는 파라미터
SHARED_DEFAULTS = ['command_timeout', 'restore_grace', 'hold_watchdog']


def read(path):
    with open(path, encoding='utf-8') as handle:
        return handle.read()


def node_defaults(source):
    """agent_node.py 의 declare_parameter('name', 3.0) 에서 숫자 기본값만 뽑는다."""
    pattern = re.compile(r"declare_parameter\(\s*'([a-z_]+)'\s*,\s*([-\d.]+)\s*\)")
    return {name: float(value) for name, value in pattern.findall(source)}


def launch_args(source):
    """<arg name="x" default="3.0"> 에서 숫자 기본값만 뽑는다."""
    pattern = re.compile(r'<arg\s+name="([a-z_]+)"\s+default="([-\d.]+)"')
    return {name: float(value) for name, value in pattern.findall(source)}


def launch_arg_names(source):
    return set(re.findall(r'<arg\s+name="([a-z_]+)"', source))


def launch_param_names(source):
    return set(re.findall(r'<param\s+name="([a-z_]+)"', source))


def mission_io_defaults():
    """mission_io.py 의 COORDINATOR 기본값 딕셔너리."""
    source = read(MISSION_IO)
    block = source.split('DEFAULT_COORDINATOR', 1)[1].split('}', 1)[0]
    return {name: float(value)
            for name, value in re.findall(r"'([a-z_]+)':\s*([-\d.]+)", block)}


@pytest.fixture(scope='module')
def coordinator_configs():
    """mission yaml 들 + mission_io 내장 기본값. (라벨, dict) 목록."""
    configs = [('mission_io 기본값', mission_io_defaults())]
    for path in MISSIONS:
        data = yaml.safe_load(read(path))
        merged = dict(mission_io_defaults())
        merged.update(data.get('coordinator') or {})
        configs.append((os.path.basename(path), merged))
    return configs


# --- 노드 기본값 == launch 기본값 ---------------------------------------

@pytest.mark.parametrize('name', SHARED_DEFAULTS)
def test_agent_launch_default_matches_node_default(name):
    """launch 가 param 을 명시 전달하므로 launch 값이 실제 동작이다."""
    node = node_defaults(read(AGENT_NODE))
    launch = launch_args(read(AGENT_LAUNCH))
    assert name in node, f'agent_node.py 에 {name} 기본값이 없다'
    assert name in launch, f'agent.launch.xml 에 {name} 인자가 없다'
    assert launch[name] == node[name], (
        f'{name}: agent.launch.xml={launch[name]} != agent_node.py={node[name]}. '
        'launch 가 이기므로 노드 기본값만 고치면 효과가 없다')


@pytest.mark.parametrize('name', SHARED_DEFAULTS)
def test_robot_launch_default_matches_agent_launch(name):
    robot = launch_args(read(ROBOT_LAUNCH))
    agent = launch_args(read(AGENT_LAUNCH))
    assert name in robot, f'robot.launch.xml 에 {name} 인자가 없다'
    assert robot[name] == agent[name], (
        f'{name}: robot.launch.xml={robot[name]} != agent.launch.xml={agent[name]}')


# --- 인자를 만들어 놓고 노드에 안 넘기는 실수 ------------------------------

def test_every_agent_launch_arg_is_passed_as_param():
    source = read(AGENT_LAUNCH)
    missing = launch_arg_names(source) - launch_param_names(source)
    assert not missing, f'agent.launch.xml 에서 노드로 안 넘어가는 인자: {sorted(missing)}'


def test_robot_launch_only_forwards_known_agent_args():
    robot = read(ROBOT_LAUNCH)
    agent_args = launch_arg_names(read(AGENT_LAUNCH))
    forwarded = set(re.findall(
        r'<arg\s+name="([a-z_]+)"\s+value="\$\(var [a-z_]+\)"', robot))
    unknown = {name for name in forwarded if name not in agent_args}
    # robot.launch.xml 은 pinky_bringup / pinky_navigation 에도 인자를 넘긴다.
    unknown -= {'map', 'params_file', 'use_sim_time', 'lifecycle_nodes_nav'}
    assert not unknown, f'agent.launch.xml 에 없는 인자를 넘기고 있다: {sorted(unknown)}'


# --- 순서 불변식 -------------------------------------------------------

def test_cooldown_is_shortest(coordinator_configs):
    for label, cfg in coordinator_configs:
        assert cfg['cooldown'] < cfg['stall_duration'], label


def test_resume_timeout_is_below_hold_watchdog(coordinator_configs):
    """로봇이 목표를 버리기 전에 관제가 먼저 재출발시킬 기회를 줘야 한다.

    뒤집히면 로봇은 hold_watchdog 에 목표를 버리고, 그 뒤에 도착한 CMD_RESUME 은
    되살릴 목표가 없어 아무 일도 하지 않는다.
    """
    hold_watchdog = launch_args(read(AGENT_LAUNCH))['hold_watchdog']
    for label, cfg in coordinator_configs:
        assert cfg['resume_timeout'] < hold_watchdog, (
            f'{label}: resume_timeout={cfg["resume_timeout"]} 가 '
            f'hold_watchdog={hold_watchdog} 보다 작아야 한다')


def test_stall_duration_is_below_resume_timeout(coordinator_configs):
    for label, cfg in coordinator_configs:
        assert cfg['stall_duration'] < cfg['resume_timeout'], label


def test_deadman_fires_well_before_the_hold_watchdog():
    """데드맨이 먼저 잡아야 워치독이 '마지막 그물' 로 남는다."""
    launch = launch_args(read(AGENT_LAUNCH))
    assert launch['command_timeout'] * 3 <= launch['hold_watchdog']


def test_clear_distance_has_hysteresis(coordinator_configs):
    for label, cfg in coordinator_configs:
        assert cfg['clear_distance'] > cfg['conflict_distance'], label


# --- nav2 파라미터 -----------------------------------------------------

@pytest.fixture(scope='module')
def follow_path():
    data = yaml.safe_load(read(NAV2_PARAMS))
    return data['controller_server']['ros__parameters']['FollowPath']


def test_curve_slowdown_is_enabled(follow_path):
    """급커브 감속. 꺼 두면 좁은 맵에서 코너를 전속으로 돌아 벽에 붙는다."""
    assert follow_path['use_regulated_linear_velocity_scaling'] is True


def test_curve_slowdown_values_are_sane(follow_path):
    radius = follow_path['regulated_linear_scaling_min_radius']
    min_speed = follow_path['regulated_linear_scaling_min_speed']
    desired = follow_path['desired_linear_vel']
    assert radius > 0.0
    assert 0.0 < min_speed < desired, '최소 속도가 목표 속도보다 크면 감속이 무의미하다'


def test_inflation_gains_match(follow_path):
    """nav2 문서 명시: 컨트롤러와 costmap 의 cost_scaling_factor 가 같아야 한다."""
    data = yaml.safe_load(read(NAV2_PARAMS))
    for costmap in ('local_costmap', 'global_costmap'):
        layer = data[costmap][costmap]['ros__parameters']['inflation_layer']
        assert layer['cost_scaling_factor'] == follow_path['inflation_cost_scaling_factor']
