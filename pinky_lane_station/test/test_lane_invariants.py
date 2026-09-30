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
    """세그 모델: 0 왼쪽 라인, 1 횡단보도, 2 오른쪽 라인, 3 라바콘, 4 신호등, 5 바리게이트."""
    cfg = yaml.safe_load(read(os.path.join(STATION, 'config', 'detector_yolo.yaml')))
    det = cfg['detector']
    assert det['kind'] == 'ultralytics' and det['device'] == 'cpu'
    cm = det['class_map']
    assert cm['left_lane'] == [0] and cm['right_lane'] == [2] and cm['crosswalk'] == [1]
    assert cm['cone'] == [3] and cm['traffic_light'] == [4] and cm['barricade'] == [5]
    assert 'lane' not in cm                                     # 좌/우 라벨을 합치지 않는다 (작업 1)
    ids = sorted(i for v in cm.values() for i in v if isinstance(i, int))
    assert ids == [0, 1, 2, 3, 4, 5], ids                     # 모든 모델 클래스가 정확히 한 번
    assert 'red_line' not in cm                               # 빨간 선은 모델 클래스가 아니라 색 검출 (red_line_color:)
    from pinky_lane_station.red_line_detector import RedLineParams
    rc = cfg['red_line_color']
    assert rc['enabled'] is True and set(rc) <= set(RedLineParams.__dataclass_fields__), set(rc)
    pipe = cfg['pipeline']
    assert abs(pipe['mask_top_frac'] - 0.50) < 1e-9 and pipe['mask_fill'] == 0   # 2026-09-30 운용값 50 % (학습은 30 %)
    allowed = {'max_rate', 'stale_period', 'stale_max_seconds', 'warmup',
               'mask_top_frac', 'mask_fill', 'debug_polygons'}
    assert set(pipe) <= allowed, set(pipe) - allowed
    from pinky_lane_station.lane_target import TargetParams
    for key in cfg['target']:
        assert key in TargetParams.__dataclass_fields__, key
    launch = read(os.path.join(STATION, 'launch', 'lane_station.launch.xml'))
    assert 'detector_yolo.yaml' in launch
    # 코디네이터는 중계(relay_station/fleet/fleet_coordinator.py) 하나다 — 관제 PC launch 는 브릿지 + 인식뿐
    for launch_name in ('lane_station.launch.xml', 'fake_lane.launch.xml'):
        text = read(os.path.join(STATION, 'launch', launch_name))
        assert 'lane_coordinator' not in text and 'use_coordinator' not in text, launch_name


def test_lane_only_launch_overrides_only_allowed_params(agent_params):
    launch = read(os.path.join(AGENT, 'launch', 'lane_only.launch.xml'))
    block = launch.split('exec="lane_agent_node"', 1)[1].split('</node>', 1)[0]
    keys = set(re.findall(r'<param name="([a-z_.]+)"', block))
    assert keys <= {'robot_name', 'domain_id', 'use_sim_time', 'use_ultrasonic', 'lane_only',
                    'auto_start', 'control.v_max', 'control.cam_sign', 'guard.use_lidar'}, keys
    assert 'use_lidar' in agent_params['guard']                   # 덮는 키는 yaml 에도 있어야 한다
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


@pytest.fixture(scope='module')
def relay_mission():
    """코디네이터(예약·CLEARANCE 틱)는 중계 하나 — 그 미션 파일이 tick_rate·reservation 의 정본이다."""
    return yaml.safe_load(read(os.path.join(REPO, 'relay_station', 'fleet', 'config', 'profiles',
                                            'team11_map5', 'lane_mission.yaml')))


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
    assert agent_params['fsm']['junction_relatch_distance'] > 2 * agent_params['junction_zone']


def test_path_timeout_covers_three_stale_periods(agent_params, detector_cfg):
    stale = detector_cfg['pipeline']['stale_period']
    assert agent_params['path_timeout'] >= 3 * stale
    assert detector_cfg['pipeline']['stale_max_seconds'] > agent_params['path_timeout']


def test_link_timeout_covers_clearance_ticks(agent_params, relay_mission):
    tick = 1.0 / relay_mission['coordinator']['tick_rate']
    assert agent_params['link_timeout'] >= 3 * tick


def test_reservation_margins(relay_mission):
    r = relay_mission['reservation']
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

def _bridge(name):
    return yaml.safe_load(read(os.path.join(STATION, 'config', name)))['topics']


def test_bridge_templates_carry_no_domain_and_match_nodes(lane_mission):
    """브리지 yaml 은 로봇별 up/down 템플릿이고 도메인 번호가 없다 — 도메인은 lane_bridge.launch.xml 인수(-from/-to)."""
    expect = {
        'up': {'state': ('pinky_fleet_msgs/msg/RobotState', 'reliable', 'volatile', 10),
               'lane_status': ('pinky_lane_msgs/msg/LaneStatus', 'reliable', 'volatile', 10),
               'camera/image/compressed': ('sensor_msgs/msg/CompressedImage', 'best_effort', 'volatile', 1)},
        'down': {'command': ('pinky_fleet_msgs/msg/FleetCommand', 'reliable', 'volatile', 10),
                 'lane_command': ('pinky_lane_msgs/msg/LaneCommand', 'reliable', 'volatile', 10),
                 'route': ('pinky_lane_msgs/msg/Route', 'reliable', 'transient_local', 1),
                 'lane_path': ('pinky_lane_msgs/msg/LanePath', 'best_effort', 'volatile', 1),
                 'overhead_pose': ('geometry_msgs/msg/PoseStamped', 'reliable', 'volatile', 10),
                 'junction_plan': ('pinky_lane_msgs/msg/JunctionPlan', 'reliable', 'transient_local', 1)},
    }
    for robot in lane_mission['robots']:
        name = robot['name']
        for direction, topics in expect.items():
            bridge = _bridge(f'bridge_{name}_{direction}.yaml')
            assert set(bridge) == {f'/{name}/{suffix}' for suffix in topics}, (name, direction)
            for suffix, (typ, rel, dur, depth) in topics.items():
                t = bridge[f'/{name}/{suffix}']
                assert t['type'] == typ and t['qos']['reliability'] == rel \
                    and t['qos']['durability'] == dur and t['qos']['depth'] == depth, (name, suffix)
                assert 'from_domain' not in t and 'to_domain' not in t, (name, suffix)
    # 노드 소스가 쓰는 토픽 접미사가 전부 브리지에 있다
    agent = read(os.path.join(AGENT, 'pinky_fleet_agent', 'lane_agent_node.py'))
    both = set(expect['up']) | set(expect['down'])
    for suffix in re.findall(r"f'/\{n\}/([a-z_/]+)'", agent):
        if suffix == 'amcl_pose':
            continue                                              # AMCL 은 항공뷰로 대체 (브리지 안 함)
        assert suffix in both, suffix
    # 항공뷰: 관제 overhead_tracker_node → 로봇 pose_fuser_node (D14)
    tracker = read(os.path.join(REPO, 'pinky_fleet_station', 'pinky_fleet_station', 'overhead_tracker_node.py'))
    assert "f'/{name}/overhead_pose'" in tracker
    fuser = read(os.path.join(AGENT, 'pinky_fleet_agent', 'pose_fuser_node.py'))
    assert "f'/{name}/overhead_pose'" in fuser and 'PoseStamped' in fuser
    setup = read(os.path.join(AGENT, 'setup.py'))
    assert 'pose_fuser_node = pinky_fleet_agent.pose_fuser_node:main' in setup
    launch = read(os.path.join(AGENT, 'launch', 'lane_robot.launch.xml'))
    assert 'exec="pose_fuser_node"' in launch and 'use_overhead' in launch and '$(eval' not in launch


def test_bridge_launch_passes_domains_as_arguments():
    launch = read(os.path.join(STATION, 'launch', 'lane_bridge.launch.xml'))
    for arg in ('station_domain', 'pinky1_domain', 'pinky2_domain'):
        assert f'<arg name="{arg}"' in launch, arg
    assert launch.count('exec="bridge_runner"') == 4          # 도메인을 채운 사본을 domain_bridge 에 넘긴다
    assert '--from $(var pinky1_domain) --to $(var station_domain) $(var config_dir)/bridge_pinky1_up.yaml' in launch
    assert '--from $(var station_domain) --to $(var pinky2_domain) $(var config_dir)/bridge_pinky2_down.yaml' in launch
    station = read(os.path.join(STATION, 'launch', 'lane_station.launch.xml'))
    for arg in ('station_domain', 'pinky1_domain', 'pinky2_domain'):
        assert f'<arg name="{arg}" value="$(var {arg})"/>' in station, arg
    assert 'bridge_lane.yaml' not in station and 'lane_bridge_config' not in station


def test_bridge_message_types_exist():
    for name in ('pinky1', 'pinky2'):
        for direction in ('up', 'down'):
            for key, t in _bridge(f'bridge_{name}_{direction}.yaml').items():
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
    assert 'reservation' not in m.to_dict() and 'coordinator' not in m.to_dict()     # 코디네이터 몫은 relay 에


def test_lane_mission_matches_relay_profile_mission():
    """로봇 목록(이름·도메인)과 기본 시작·목적지는 관제 PC 파일과 중계 프로파일이 같아야 한다 — 두 곳이 어긋나면
    파이프라인·가짜 로봇은 한 이름을, 코디네이터는 다른 이름을 본다."""
    lane = yaml.safe_load(read(os.path.join(STATION, 'config', 'lane_mission.yaml')))
    relay = yaml.safe_load(read(os.path.join(REPO, 'relay_station', 'fleet', 'config', 'profiles',
                                             'team11_map5', 'lane_mission.yaml')))
    key = lambda r: (r['name'], int(r['domain_id']), r['start'], r['goal'])       # noqa: E731
    assert [key(r) for r in lane['robots']] == [key(r) for r in relay['robots']]
    assert all(r.get('drive_mode', 'lane') == 'lane' for r in relay['robots'])
    assert lane['map']['yaml_path'].endswith('pinky_fleet_station/config/map5.yaml')


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
    for f in ('road_graph.yaml', 'lane_mission.yaml', 'detector_lane.yaml', 'bridge_pinky1_up.yaml', 'bridge_pinky2_down.yaml'):
        assert os.path.isfile(os.path.join(STATION, 'config', f)), f
    for f in ('lane_station.launch.xml', 'lane_bridge.launch.xml', 'fake_lane.launch.xml'):
        assert os.path.isfile(os.path.join(STATION, 'launch', f)), f
    for entry in ('lane_pipeline_node', 'fake_lane_robot', 'bench_detector'):
        assert f"'{entry} = pinky_lane_station.{entry}:main'" in setup, entry
    assert 'lane_coordinator_node' not in setup                     # 코디네이터는 중계(relay_station/fleet) 하나
    assert not os.path.exists(os.path.join(STATION, 'pinky_lane_station', 'lane_coordinator_node.py'))
    assert not os.path.exists(os.path.join(STATION, 'pinky_lane_station', 'reservation.py'))
    agent_setup = read(os.path.join(AGENT, 'setup.py'))
    for entry in ('lane_agent_node', 'camera_node'):
        assert f"'{entry} = pinky_fleet_agent.{entry}:main'" in agent_setup, entry


def test_bridge_runner_fills_domains_for_apt_domain_bridge(tmp_path):
    """apt domain_bridge 는 yaml 에 from_domain 이 없으면 --from 을 주어도 죽는다 — bridge_runner 가 채운 사본을 만든다."""
    import yaml
    from pinky_lane_station.bridge_runner import main as _main, render_config, write_rendered  # noqa: F401
    for name, (frm, to) in (('bridge_pinky1_up', (10, 0)), ('bridge_pinky2_down', (0, 11))):
        src = os.path.join(STATION, 'config', name + '.yaml')
        path = write_rendered(src, frm, to, out_dir=str(tmp_path))
        data = yaml.safe_load(open(path, encoding='utf-8'))
        assert data['from_domain'] == frm and data['to_domain'] == to
        assert data['topics'] and all(t['from_domain'] == frm and t['to_domain'] == to and 'type' in t
                                      for t in data['topics'].values())
        assert set(data['topics']) == set(yaml.safe_load(open(src, encoding='utf-8'))['topics'])
    setup = read(os.path.join(STATION, 'setup.py'))
    assert "'bridge_runner = pinky_lane_station.bridge_runner:main'" in setup
