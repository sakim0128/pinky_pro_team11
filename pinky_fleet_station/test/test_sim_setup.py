"""가제보 시뮬 설정이 스스로 어긋나지 않는지 검사한다 (ROS / 가제보 불필요).

시뮬 쪽 파일은 대부분 **생성된 것**이다.
  * pinky_fleet_sim/map/fleet_arena.{pgm,yaml}  <- worlds/fleet_arena.sdf
  * pinky_fleet_sim/params/nav2_sim_*.yaml      <- nav2_params_fleet.yaml

원본을 고치고 생성기를 안 돌리면 시뮬과 실기가 조용히 갈라진다. 실기 파라미터를 튜닝한
뒤 시뮬에서 "잘 되네" 하고 넘어가면 검증한 적 없는 값으로 실기를 돌리게 된다.
여기서 그걸 막는다.

mission_sim.yaml 의 출발/목표 좌표가 벽 안에 박혀 있는지도 본다. 벽 속 목표는
Nav2 가 조용히 실패하는 종류라 눈으로는 못 잡는다.
"""

import os
import subprocess
import sys

import pytest
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SIM = os.path.join(REPO, 'pinky_fleet_sim')
STATION = os.path.join(REPO, 'pinky_fleet_station')
AGENT = os.path.join(REPO, 'pinky_fleet_agent')

MISSION_SIM = os.path.join(STATION, 'config', 'mission_sim.yaml')
ARENA_PGM = os.path.join(SIM, 'map', 'fleet_arena.pgm')
ARENA_YAML = os.path.join(SIM, 'map', 'fleet_arena.yaml')

ROBOT_RADIUS = 0.06 + 0.10       # 내접 반경 + inflation_radius. 이 안에 벽이 있으면 안 된다.

pytestmark = pytest.mark.skipif(
    not os.path.isdir(SIM), reason='pinky_fleet_sim 패키지가 없다')


def read(path):
    with open(path, encoding='utf-8') as handle:
        return handle.read()


def load_yaml(path):
    return yaml.safe_load(read(path))


def run_generator(script, tmp_path):
    """생성기를 돌려 출력이 커밋된 것과 같은지 본다 (원본을 건드리지 않는다)."""
    return subprocess.run(
        [sys.executable, os.path.join(SIM, 'scripts', script)],
        capture_output=True, text=True, cwd=REPO)


# --- 생성물이 최신인가 --------------------------------------------------

def test_arena_map_is_up_to_date(tmp_path):
    before = open(ARENA_PGM, 'rb').read(), read(ARENA_YAML)
    result = run_generator('make_arena_map.py', tmp_path)
    assert result.returncode == 0, result.stderr
    after = open(ARENA_PGM, 'rb').read(), read(ARENA_YAML)
    assert before == after, (
        'fleet_arena.sdf 를 고치고 make_arena_map.py 를 안 돌렸다. '
        '맵과 월드가 다르면 AMCL 이 수렴하지 않는다')


def test_sim_nav2_params_are_up_to_date(tmp_path):
    paths = [os.path.join(SIM, 'params', f'nav2_sim_{n}.yaml')
             for n in ('pinky1', 'pinky2')]
    before = [read(p) for p in paths]
    result = run_generator('make_sim_nav2_params.py', tmp_path)
    assert result.returncode == 0, result.stderr
    assert [read(p) for p in paths] == before, (
        'nav2_params_fleet.yaml 을 고치고 make_sim_nav2_params.py 를 안 돌렸다. '
        '시뮬에서 검증한 값과 실기 값이 달라진다')


# --- 시뮬 파라미터가 실기 튜닝을 그대로 물려받았나 --------------------------

@pytest.mark.parametrize('namespace', ['pinky1', 'pinky2'])
def test_sim_params_keep_the_real_tuning(namespace):
    real = load_yaml(os.path.join(AGENT, 'params', 'nav2_params_fleet.yaml'))
    sim = load_yaml(os.path.join(SIM, 'params', f'nav2_sim_{namespace}.yaml'))

    real_fp = real['controller_server']['ros__parameters']['FollowPath']
    sim_fp = sim['controller_server']['ros__parameters']['FollowPath']
    for key in ('use_regulated_linear_velocity_scaling',
                'regulated_linear_scaling_min_radius',
                'lookahead_dist', 'desired_linear_vel',
                'inflation_cost_scaling_factor', 'approach_velocity_scaling_dist'):
        assert sim_fp[key] == real_fp[key], key

    real_gc = real['controller_server']['ros__parameters']['general_goal_checker']
    sim_gc = sim['controller_server']['ros__parameters']['general_goal_checker']
    assert sim_gc['xy_goal_tolerance'] == real_gc['xy_goal_tolerance']
    assert sim_gc['yaw_goal_tolerance'] == real_gc['yaw_goal_tolerance']

    for costmap in ('local_costmap', 'global_costmap'):
        r = real[costmap][costmap]['ros__parameters']['inflation_layer']
        s = sim[costmap][costmap]['ros__parameters']['inflation_layer']
        assert s == r, costmap


# --- 네임스페이스 치환이 제대로 됐나 ---------------------------------------

@pytest.mark.parametrize('namespace', ['pinky1', 'pinky2'])
def test_frames_are_prefixed_but_map_is_shared(namespace):
    sim = load_yaml(os.path.join(SIM, 'params', f'nav2_sim_{namespace}.yaml'))

    amcl = sim['amcl']['ros__parameters']
    assert amcl['base_frame_id'] == f'{namespace}/base_footprint'
    assert amcl['odom_frame_id'] == f'{namespace}/odom'
    # map 프레임은 공유해야 GUI 에서 찍은 좌표가 두 로봇에 그대로 통한다.
    assert amcl['global_frame_id'] == 'map'

    local = sim['local_costmap']['local_costmap']['ros__parameters']
    assert local['global_frame'] == f'{namespace}/odom'
    assert local['robot_base_frame'] == f'{namespace}/base_footprint'

    glob = sim['global_costmap']['global_costmap']['ros__parameters']
    assert glob['global_frame'] == 'map'
    assert glob['robot_base_frame'] == f'{namespace}/base_footprint'


@pytest.mark.parametrize('namespace', ['pinky1', 'pinky2'])
def test_absolute_scan_topic_is_split_per_robot(namespace):
    """costmap 의 topic: /scan 은 절대 경로라 네임스페이스가 안 붙는다."""
    sim = load_yaml(os.path.join(SIM, 'params', f'nav2_sim_{namespace}.yaml'))
    found = []
    for costmap in ('local_costmap', 'global_costmap'):
        params = sim[costmap][costmap]['ros__parameters']
        for value in params.values():
            if isinstance(value, dict) and 'scan' in value:
                found.append(value['scan']['topic'])
    assert found, 'costmap 에서 라이다 레이어를 못 찾았다 (파서 확인)'
    assert all(t == f'/{namespace}/scan' for t in found), found


@pytest.mark.parametrize('namespace', ['pinky1', 'pinky2'])
def test_every_nav2_node_uses_sim_time(namespace):
    sim = load_yaml(os.path.join(SIM, 'params', f'nav2_sim_{namespace}.yaml'))

    def nodes(node):
        found = []
        for key, value in node.items():
            if isinstance(value, dict) and 'ros__parameters' in value:
                found.append((key, value['ros__parameters']))
            elif isinstance(value, dict):
                found += nodes(value)
        return found

    listed = nodes(sim)
    assert len(listed) >= 8, f'노드가 너무 적다: {[n for n, _ in listed]}'
    missing = [n for n, p in listed if p.get('use_sim_time') is not True]
    assert not missing, f'use_sim_time 이 True 가 아닌 노드: {missing}'


# --- 브리지 설정 -------------------------------------------------------

@pytest.mark.parametrize('namespace', ['pinky1', 'pinky2'])
def test_robot_bridge_uses_absolute_names(namespace):
    entries = load_yaml(os.path.join(SIM, 'params', f'{namespace}_bridge.yaml'))
    for entry in entries:
        assert entry['ros_topic_name'].startswith('/'), entry
        assert entry['gz_topic_name'].startswith('/'), entry

    ros_names = {e['ros_topic_name'] for e in entries}
    assert f'/{namespace}/scan' in ros_names
    assert f'/{namespace}/cmd_vel' in ros_names
    assert f'/{namespace}/odom' in ros_names
    # DiffDrive 의 tf_topic 은 /tf 로 고정이라 공용 gz 토픽을 각 네임스페이스로 나른다.
    tf = [e for e in entries if e['ros_topic_name'] == f'/{namespace}/tf']
    assert tf and tf[0]['gz_topic_name'] == '/tf', tf


def test_clock_bridge_is_global_and_alone():
    """rclcpp 시간 소스는 절대 경로 /clock 을 구독한다. 네임스페이스에 넣으면 안 된다."""
    entries = load_yaml(os.path.join(SIM, 'params', 'clock_bridge.yaml'))
    assert len(entries) == 1
    assert entries[0]['ros_topic_name'] == '/clock'
    assert entries[0]['gz_topic_name'] == '/clock'

    for namespace in ('pinky1', 'pinky2'):
        robot = load_yaml(os.path.join(SIM, 'params', f'{namespace}_bridge.yaml'))
        assert not [e for e in robot if 'clock' in e['ros_topic_name']], (
            f'{namespace}_bridge.yaml 에 clock 이 또 있다. '
            '/clock 브리지는 전체에 하나만 있어야 한다')


# --- launch 가 에이전트에 없는 인자를 넘기지 않는가 -------------------------

def test_sim_launch_only_passes_known_agent_args():
    import re
    agent_args = set(re.findall(
        r'<arg\s+name="([a-z_]+)"',
        read(os.path.join(AGENT, 'launch', 'agent.launch.xml'))))
    robot_launch = read(os.path.join(SIM, 'launch', 'gz_robot.launch.xml'))
    block = robot_launch.split('agent.launch.xml', 1)[1].split('</include>', 1)[0]
    passed = set(re.findall(r'<arg\s+name="([a-z_]+)"', block))
    unknown = passed - agent_args
    assert not unknown, (
        f'gz_robot.launch.xml 이 agent.launch.xml 에 없는 인자를 넘긴다: {sorted(unknown)}')


# --- mission_sim.yaml 의 좌표가 아레나 안에 있는가 -------------------------

@pytest.fixture(scope='module')
def arena():
    """맵 pgm 을 읽어 (월드좌표 -> 벽인가) 판정 함수를 돌려준다."""
    meta = load_yaml(ARENA_YAML)
    data = open(ARENA_PGM, 'rb').read()
    parts, idx = [], 0
    while len(parts) < 4:
        nl = data.index(b'\n', idx)
        line = data[idx:nl]
        idx = nl + 1
        if not line.startswith(b'#'):
            parts += line.split()
    assert parts[0] == b'P5'
    width, height = int(parts[1]), int(parts[2])
    pixels = data[idx:]
    assert len(pixels) == width * height
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]

    def occupied(wx, wy):
        cx = int((wx - ox) / res)
        row = height - 1 - int((wy - oy) / res)
        if not (0 <= cx < width and 0 <= row < height):
            return True                      # 맵 밖은 벽으로 친다
        return pixels[row * width + cx] == 0

    return occupied


@pytest.mark.parametrize('field', ['initial_pose', 'goal'])
def test_mission_sim_poses_are_clear_of_walls(arena, field):
    mission = load_yaml(MISSION_SIM)
    for robot in mission['robots']:
        pose = robot[field]
        assert not arena(pose['x'], pose['y']), (
            f"{robot['name']}.{field} ({pose['x']}, {pose['y']}) 가 벽 안이다")
        # 로봇 반경 + inflation 만큼 주변도 비어 있어야 Nav2 가 출발한다.
        for dx, dy in ((ROBOT_RADIUS, 0), (-ROBOT_RADIUS, 0),
                       (0, ROBOT_RADIUS), (0, -ROBOT_RADIUS)):
            assert not arena(pose['x'] + dx, pose['y'] + dy), (
                f"{robot['name']}.{field} 주변 {ROBOT_RADIUS}m 안에 벽이 있다 "
                f"(+{dx}, +{dy}). Nav2 가 출발하지 못한다")


def test_mission_sim_makes_the_robots_meet():
    """두 로봇의 경로가 실제로 교차해야 교착 시험이 된다."""
    mission = load_yaml(MISSION_SIM)
    one, two = mission['robots']
    assert one['initial_pose']['y'] > 0 > one['goal']['y'], 'pinky1 이 북->남이어야 한다'
    assert two['initial_pose']['y'] < 0 < two['goal']['y'], 'pinky2 가 남->북이어야 한다'
    assert one['domain_id'] < two['domain_id'], 'pinky1 이 leader 여야 한다'


def test_mission_sim_matches_real_coordinator_tuning():
    """시뮬은 실기 값을 검증하는 자리다. 임계값이 다르면 검증이 무의미하다."""
    sim = load_yaml(MISSION_SIM)['coordinator']
    real = load_yaml(os.path.join(STATION, 'config', 'mission.yaml'))['coordinator']
    assert sim == real, (
        f'mission_sim.yaml 과 mission.yaml 의 coordinator 값이 다르다: '
        f'{ {k: (sim.get(k), real.get(k)) for k in set(sim) | set(real) if sim.get(k) != real.get(k)} }')
