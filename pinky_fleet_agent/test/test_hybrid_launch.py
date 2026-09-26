"""hybrid_robot.launch.xml 계열의 불변식 (ROS 불필요).

- /cmd_vel 은 drive_command_gate 만 낸다: Nav2 출력이 전부 /cmd_vel_mission 또는 /cmd_vel_nav 로 remap 됐는가.
- launch 인자 기본값 == hybrid_agent_node 기본값 (launch 가 <param> 으로 넘기므로 launch 쪽이 이긴다).
- /<robot>/diag 가 묻는 수명주기 노드 == 실제로 띄우는 노드.
- 기존 launch(robot · lane_robot · agent)는 새 노드를 띄우지 않는다.
"""

import ast
import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from pinky_fleet_agent import diag  # noqa: E402

LAUNCH = os.path.join(PKG, 'launch')
AGENT_NODE = os.path.join(PKG, 'pinky_fleet_agent', 'hybrid_agent_node.py')
HYBRID_ROBOT = os.path.join(LAUNCH, 'hybrid_robot.launch.xml')
HYBRID_AGENT = os.path.join(LAUNCH, 'hybrid_agent.launch.xml')
GATED_BRINGUP = os.path.join(LAUNCH, 'nav2_gated_bringup.launch.xml')
GATED_NAV = os.path.join(LAUNCH, 'nav2_gated_navigation.launch.xml')

NAV2_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
              'bt_navigator', 'waypoint_follower', 'velocity_smoother']

# 노드 이름 -> {from: to}. 여기 없는 cmd_vel 계열 remap 이 있으면 안 되고, 여기 있는 것은 빠지면 안 된다.
EXPECTED_CMD_REMAPS = {
    'controller_server': {'cmd_vel': 'cmd_vel_nav'},
    'smoother_server': {},
    'planner_server': {},
    'behavior_server': {'cmd_vel': 'cmd_vel_mission'},
    'bt_navigator': {'cmd_vel': 'cmd_vel_nav'},
    'waypoint_follower': {},
    'velocity_smoother': {'cmd_vel': 'cmd_vel_nav', 'cmd_vel_smoothed': 'cmd_vel_mission'},
}


def read(path):
    with open(path, encoding='utf-8') as handle:
        return handle.read()


def parse_list(text):
    """launch 인자의 "['a', 'b']" 문자열을 목록으로."""
    return ast.literal_eval(' '.join(text.split()))


def args_of(root):
    return {a.get('name'): a.get('default') for a in root.iter('arg') if a.get('default') is not None}


def node_defaults():
    """hybrid_agent_node.py 의 declare_parameter('name', 값) — 리터럴 값만."""
    out = {}
    for name, value in re.findall(r"declare_parameter\(\s*'([a-z_]+)'\s*,\s*([^)]+?)\)\s*$",
                                  read(AGENT_NODE), re.M):
        try:
            out[name] = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            pass
    return out


@pytest.mark.parametrize('path', sorted(glob.glob(os.path.join(LAUNCH, '*.launch.xml'))),
                         ids=os.path.basename)
def test_every_launch_file_parses(path):
    ET.parse(path)


@pytest.mark.parametrize('block', ['load_composable_node', 'group'])
def test_gated_navigation_never_publishes_cmd_vel_directly(block):
    root = ET.parse(GATED_NAV).getroot()
    (container,) = [e for e in root if e.tag == block]
    tag = 'composable_node' if block == 'load_composable_node' else 'node'
    seen = {}
    for node in container.iter(tag):
        name = node.get('name') or node.get('exec')
        remaps = {r.get('from'): r.get('to') for r in node.iter('remap')
                  if r.get('from') in ('cmd_vel', 'cmd_vel_smoothed')}
        seen[name] = remaps
    for name, expected in EXPECTED_CMD_REMAPS.items():
        assert seen.get(name) == expected, f'{block}/{name}: {seen.get(name)} != {expected}'
    assert all(to != 'cmd_vel' for remaps in seen.values() for to in remaps.values())


def test_gated_bringup_uses_gated_navigation():
    source = read(GATED_BRINGUP)
    assert 'pinky_fleet_agent)/launch/nav2_gated_navigation.launch.xml' in source
    assert 'pinky_navigation)/launch/navigation_launch.xml' not in source


def test_hybrid_robot_runs_the_gate_and_the_hybrid_agent():
    root = ET.parse(HYBRID_ROBOT).getroot()
    execs = {n.get('exec') for n in root.iter('node')}
    assert {'drive_command_gate', 'pose_fuser_node'} <= execs
    assert 'pinky_fleet_agent)/launch/hybrid_agent.launch.xml' in read(HYBRID_ROBOT)
    assert ET.parse(HYBRID_AGENT).getroot().find('node').get('exec') == 'hybrid_agent_node'


@pytest.mark.parametrize('name', ['robot.launch.xml', 'agent.launch.xml', 'lane_robot.launch.xml',
                                  'lane_agent.launch.xml', 'lane_only.launch.xml'])
def test_existing_launch_files_do_not_start_new_nodes(name):
    source = read(os.path.join(LAUNCH, name))
    for new in ('hybrid_agent', 'drive_command_gate', 'pose_fuser', 'nav2_gated'):
        assert new not in source, f'{name} 가 {new} 를 띄운다'


def test_every_hybrid_agent_arg_is_passed_as_param():
    root = ET.parse(HYBRID_AGENT).getroot()
    arg_names = {a.get('name') for a in root.iter('arg')}
    param_names = {p.get('name') for p in root.find('node').iter('param')}
    assert arg_names - param_names == set()


def test_hybrid_agent_launch_defaults_match_node_defaults():
    launch = args_of(ET.parse(HYBRID_AGENT).getroot())
    node = node_defaults()
    for name in ('command_timeout', 'restore_grace', 'hold_watchdog', 'max_linear_vel', 'max_angular_vel',
                 'map_dir', 'map_name', 'robot_name', 'domain_id', 'plan_in_topic', 'map_topic',
                 'global_frame', 'robot_base_frame'):
        assert name in node, name
        value = node[name]
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            assert float(launch[name]) == float(value), name
        else:
            assert launch[name] == value, name
    assert parse_list(launch['diag_lifecycle_nodes']) == list(diag.DEFAULT_NAV2_NODES)


def test_hybrid_robot_forwards_only_known_agent_args_with_same_defaults():
    robot_root = ET.parse(HYBRID_ROBOT).getroot()
    agent_args = args_of(ET.parse(HYBRID_AGENT).getroot())
    robot_args = args_of(robot_root)
    (include,) = [i for i in robot_root.iter('include') if 'hybrid_agent.launch.xml' in i.get('file')]
    forwarded = {a.get('name') for a in include.iter('arg')}
    assert forwarded <= set(agent_args)
    for name in ('command_timeout', 'restore_grace', 'hold_watchdog', 'max_linear_vel', 'max_angular_vel'):
        assert float(robot_args[name]) == float(agent_args[name]), name


def test_diag_lifecycle_nodes_match_what_is_launched():
    robot_root = ET.parse(HYBRID_ROBOT).getroot()
    lets = {('fuser' if let.get('if') else 'amcl'): parse_list(let.get('value'))
            for let in robot_root.iter('let') if let.get('name') == 'diag_lifecycle_nodes'}
    (include,) = [i for i in robot_root.iter('include') if 'nav2_gated_bringup' in i.get('file')]
    nav = [parse_list(a.get('value')) for a in include.iter('arg') if a.get('name') == 'lifecycle_nodes_nav'][0]
    assert nav == NAV2_NODES
    assert lets['amcl'] == ['map_server', 'amcl'] + nav
    assert lets['fuser'] == ['map_server'] + nav == list(diag.DEFAULT_NAV2_NODES)
