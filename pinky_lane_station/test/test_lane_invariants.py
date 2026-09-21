"""텍스트 불변식 — 메시지 상수 ↔ 파이썬 상수, 타이밍, 브릿지 토픽, 시각 규칙 (ROS 불필요).

ROS 없이는 생성된 메시지 타입을 import 할 수 없어 .msg 와 노드 소스를 텍스트로 읽는다.
"""

import os
import re
import sys

import pytest
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'pinky_lane_station'))
sys.path.insert(0, os.path.join(REPO, 'pinky_fleet_agent'))

MSG_DIR = os.path.join(REPO, 'pinky_lane_msgs', 'msg')
STATION = os.path.join(REPO, 'pinky_lane_station')
AGENT = os.path.join(REPO, 'pinky_fleet_agent')
CONSTANT = re.compile(r'^\s*uint8\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d+)\s*(?:#.*)?$')


def read(path):
    with open(path, encoding='utf-8') as fh:
        return fh.read()


def msg_constants(name):
    found = {}
    for line in read(os.path.join(MSG_DIR, name)).splitlines():
        m = CONSTANT.match(line)
        if m:
            found[m.group(1)] = int(m.group(2))
    return found


# --- 상수 --------------------------------------------------------------

@pytest.mark.parametrize('name', ['LaneCommand.msg', 'LanePath.msg', 'LaneStatus.msg'])
def test_msg_constants_unique(name):
    found = msg_constants(name)
    assert found
    assert len(set(found.values())) == len(found), found


def test_lane_command_matches_driver_constants():
    from pinky_fleet_agent import lane_driver as d
    c = msg_constants('LaneCommand.msg')
    assert c == {'CMD_HEARTBEAT': d.CMD_HEARTBEAT, 'CMD_START': d.CMD_START,
                 'CMD_STOP': d.CMD_STOP, 'CMD_ESTOP': d.CMD_ESTOP, 'CMD_RESUME': d.CMD_RESUME,
                 'CMD_SET_SPEED': d.CMD_SET_SPEED, 'CMD_CLEARANCE': d.CMD_CLEARANCE}


def test_lane_status_matches_fsm_constants():
    from pinky_fleet_agent import drive_fsm as f
    c = msg_constants('LaneStatus.msg')
    expected = {f'DRIVE_{name}': value for value, name in f.STATE_NAMES.items()}
    assert c == expected


def test_lane_path_quality_matches_python():
    from pinky_fleet_agent import lane_control as lc
    from pinky_lane_station import lane_target as lt
    c = msg_constants('LanePath.msg')
    for mod in (lc, lt):
        assert c == {'QUALITY_BOTH': mod.QUALITY_BOTH, 'QUALITY_SINGLE': mod.QUALITY_SINGLE,
                     'QUALITY_JUNCTION': mod.QUALITY_JUNCTION, 'QUALITY_STALE': mod.QUALITY_STALE,
                     'QUALITY_LOST': mod.QUALITY_LOST}


# --- 시각 규칙: source_stamp 는 복사만 ----------------------------------

def test_pipeline_copies_source_stamp_and_never_restamps():
    src = read(os.path.join(STATION, 'pinky_lane_station', 'lane_pipeline_node.py'))
    assigns = re.findall(r'\.source_stamp\s*=\s*([^\n]+)', src)
    assert assigns
    for rhs in assigns:
        assert 'now()' not in rhs and 'get_clock' not in rhs, rhs
        assert 'header.stamp' in rhs or 'source_stamp' in rhs, rhs


def test_detector_yolo_class_map_matches_model_labels():
    """yolo26n-seg: 0 왼쪽 라인, 1 횡단보도, 2 오른쪽 라인 → lane=[0,2], crosswalk=[1]."""
    cfg = yaml.safe_load(read(os.path.join(STATION, 'config', 'detector_yolo.yaml')))
    det = cfg['detector']
    assert det['kind'] == 'ultralytics' and det['device'] == 'cpu'
    assert sorted(det['class_map']['lane']) == [0, 2] and det['class_map']['crosswalk'] == [1]
    from pinky_lane_station.lane_target import TargetParams
    for key in cfg['target']:
        assert key in TargetParams.__dataclass_fields__, key
    launch = read(os.path.join(STATION, 'launch', 'lane_station.launch.xml'))
    assert 'detector_yolo.yaml' in launch and 'use_coordinator' in launch


def test_lane_only_launch_overrides_only_allowed_params(agent_params):
    launch = read(os.path.join(AGENT, 'launch', 'lane_only.launch.xml'))
    block = launch.split('exec="lane_agent_node"', 1)[1].split('</node>', 1)[0]
    keys = set(re.findall(r'<param name="([a-z_.]+)"', block))
    assert keys <= {'robot_name', 'domain_id', 'use_sim_time', 'use_ultrasonic', 'lane_only',
                    'auto_start', 'control.v_max', 'control.cam_sign'}, keys
    assert 'lane_only' in agent_params and agent_params['lane_only'] is False
    assert agent_params['lane_lost_coast'] < agent_params['path_timeout']


def test_all_launch_xml_files_parse():
    """속성값 안의 '<' 같은 문자는 launch 자체를 못 띄운다 (실기에서 한 번 겪음)."""
    import glob
    import xml.etree.ElementTree as ET
    files = glob.glob(os.path.join(REPO, '*', 'launch', '*.xml'))
    assert files
    for f in files:
        ET.parse(f)


def test_camera_node_stamps_right_after_capture():
    src = read(os.path.join(AGENT, 'pinky_fleet_agent', 'camera_node.py'))
    cap = src.index('capture_array()')
    stamp = src.index('now().to_msg()', cap)
    assert stamp - cap < 200, 'stamp 는 캡처 직후에 찍어야 한다'


# --- 타이밍 ------------------------------------------------------------

@pytest.fixture(scope='module')
def agent_params():
    data = yaml.safe_load(read(os.path.join(AGENT, 'params', 'lane_agent.yaml')))
    return data['pinky_lane_agent']['ros__parameters']


@pytest.fixture(scope='module')
def detector_cfg():
    return yaml.safe_load(read(os.path.join(STATION, 'config', 'detector_lane.yaml')))


@pytest.fixture(scope='module')
def lane_mission():
    return yaml.safe_load(read(os.path.join(STATION, 'config', 'lane_mission.yaml')))


def test_agent_yaml_keys_exist_in_driver_params(agent_params):
    from pinky_fleet_agent.lane_driver import DriverParams
    p = DriverParams()
    groups = {'control': p.control, 'fsm': p.fsm, 'guard': p.guard}
    node_keys = {'control_rate', 'status_rate', 'pose_timeout', 'use_ultrasonic', 'us_topic',
                 'scan_topic', 'cmd_vel_topic', 'restore_grace', 'auto_start'}
    for key, value in agent_params.items():
        if key in groups:
            for sub in value:
                assert hasattr(groups[key], sub), f'{key}.{sub}'
        else:
            assert key in node_keys or hasattr(p, key), key


def test_crosswalk_relatch_exceeds_zone(agent_params):
    assert agent_params['fsm']['crosswalk_relatch_distance'] > agent_params['crosswalk_zone']


def test_path_timeout_covers_three_stale_periods(agent_params, detector_cfg):
    stale = detector_cfg['pipeline']['stale_period']
    assert agent_params['path_timeout'] >= 3 * stale
    assert detector_cfg['pipeline']['stale_max_seconds'] > agent_params['path_timeout']


def test_link_timeout_covers_clearance_ticks(agent_params, lane_mission):
    tick = 1.0 / lane_mission['coordinator']['tick_rate']
    assert agent_params['link_timeout'] >= 3 * tick


def test_reservation_margins(lane_mission):
    r = lane_mission['reservation']
    assert r['reserve_ahead'] > r['node_stop_margin'] > 0
    assert r['release_behind'] > 0


def test_detector_target_keys_valid(detector_cfg):
    from pinky_lane_station.lane_target import TargetParams
    for key in detector_cfg['target']:
        assert key in TargetParams.__dataclass_fields__, key


def test_launch_defaults_match_agent_yaml(agent_params):
    """launch 의 <param from=...> 가 이기므로 yaml 이 진실. launch 에서 겹쳐 넘기는 키만 검사."""
    launch = read(os.path.join(AGENT, 'launch', 'lane_agent.launch.xml'))
    agent_block = launch.split('exec="lane_agent_node"', 1)[1].split('</node>', 1)[0]
    for key in re.findall(r'<param name="([a-z_]+)"', agent_block):
        assert key in ('robot_name', 'domain_id', 'use_sim_time', 'use_ultrasonic'), \
            f'{key}: 튜닝값은 lane_agent.yaml 한 곳에서만 정한다'


# --- 브릿지 --------------------------------------------------------------

def test_bridge_lane_topics_match_nodes_and_mission(lane_mission):
    bridge = yaml.safe_load(read(os.path.join(STATION, 'config', 'bridge_lane.yaml')))['topics']
    fleet = yaml.safe_load(read(os.path.join(REPO, 'pinky_fleet_station', 'config',
                                             'bridge_fleet.yaml')))['topics']
    assert not set(bridge) & set(fleet), '두 브릿지가 같은 토픽을 두 번 브릿지하면 안 된다'
    expect_qos = {
        'camera/image/compressed': ('best_effort', 'volatile', 1, 'up'),
        'lane_path': ('best_effort', 'volatile', 1, 'down'),
        'route': ('reliable', 'transient_local', 1, 'down'),
        'lane_command': ('reliable', 'volatile', 10, 'down'),
        'lane_status': ('reliable', 'volatile', 10, 'up'),
    }
    for robot in lane_mission['robots']:
        name, dom = robot['name'], int(robot['domain_id'])
        for suffix, (rel, dur, depth, direction) in expect_qos.items():
            key = f'/{name}/{suffix}'
            assert key in bridge, key
            t = bridge[key]
            assert t['qos']['reliability'] == rel and t['qos']['durability'] == dur \
                and t['qos']['depth'] == depth, key
            if direction == 'up':
                assert (t['from_domain'], t['to_domain']) == (dom, 0), key
            else:
                assert (t['from_domain'], t['to_domain']) == (0, dom), key
    # 노드 소스가 쓰는 토픽 접미사가 전부 브릿지에 있다
    agent = read(os.path.join(AGENT, 'pinky_fleet_agent', 'lane_agent_node.py'))
    for suffix in re.findall(r"f'/\{n\}/([a-z_/]+)'", agent):
        if suffix in ('state', 'command', 'amcl_pose'):
            assert f'/pinky1/{suffix}' in fleet, suffix      # bridge_fleet.yaml 담당
            continue
        assert f'/pinky1/{suffix}' in bridge, suffix
    for name in ('pinky1', 'pinky2'):
        t = fleet[f'/{name}/amcl_pose']
        assert t['type'] == 'geometry_msgs/msg/PoseWithCovarianceStamped' and t['to_domain'] == 0


def test_bridge_lane_message_types_exist():
    bridge = yaml.safe_load(read(os.path.join(STATION, 'config', 'bridge_lane.yaml')))['topics']
    for key, t in bridge.items():
        pkg, _, msg = t['type'].split('/')
        if pkg == 'pinky_lane_msgs':
            assert os.path.isfile(os.path.join(MSG_DIR, msg + '.msg')), key


# --- 미션 ------------------------------------------------------------------

def test_lane_mission_loads_and_routes_exist():
    from pinky_lane_station.lane_mission import load_lane_mission
    from pinky_lane_station.road_graph import RoadGraph
    m = load_lane_mission(os.path.join(STATION, 'config', 'lane_mission.yaml'))
    g = RoadGraph.load(m.graph_path)
    assert m.validate_against_graph(g) == []
    assert [r['name'] for r in m.by_priority()] == ['pinky1', 'pinky2']


def test_lane_mission_rejects_duplicates(tmp_path):
    from pinky_lane_station.lane_mission import LaneMission, LaneMissionError
    with pytest.raises(LaneMissionError):
        LaneMission({'robots': [{'name': 'a', 'domain_id': 1}, {'name': 'a', 'domain_id': 2}]})
    with pytest.raises(LaneMissionError):
        LaneMission({'robots': [{'name': 'a', 'domain_id': 1}, {'name': 'b', 'domain_id': 1}]})
    with pytest.raises(LaneMissionError):
        LaneMission({'robots': []})


def test_setup_installs_lane_configs_and_launch():
    setup = read(os.path.join(STATION, 'setup.py'))
    assert "glob('config/*.yaml')" in setup and "glob('launch/*.launch.xml')" in setup
    for f in ('road_graph.yaml', 'lane_mission.yaml', 'detector_lane.yaml', 'bridge_lane.yaml'):
        assert os.path.isfile(os.path.join(STATION, 'config', f)), f
    for f in ('lane_station.launch.xml', 'lane_bridge.launch.xml', 'fake_lane.launch.xml'):
        assert os.path.isfile(os.path.join(STATION, 'launch', f)), f
    for entry in ('lane_coordinator_node', 'lane_pipeline_node', 'fake_lane_robot', 'bench_detector'):
        assert f"'{entry} = pinky_lane_station.{entry}:main'" in setup, entry
    agent_setup = read(os.path.join(AGENT, 'setup.py'))
    for entry in ('lane_agent_node', 'camera_node'):
        assert f"'{entry} = pinky_fleet_agent.{entry}:main'" in agent_setup, entry
