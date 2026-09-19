"""시뮬 xacro 의 네임스페이스 접두사 규칙을 고정한다 (ROS / 가제보 / xacro 불필요).

왜 필요한가. upstream pinky_description 은 링크 이름에는 ``${namespace}`` 를 안 붙이고
조인트 이름에는 붙이는데, ``pinky_gz.urdf.xacro`` 는 **링크를 가리키는**
``<gazebo reference>`` 에도 붙인다. 네임스페이스를 쓰면 존재하지 않는 링크를 가리키게
되고, sdformat 은 그런 블록을 경고 없이 버린다 - 센서와 마찰 설정이 통째로 사라져
``/scan`` 이 안 나오는데 로봇은 멀쩡히 움직인다. 증상이 없어서 찾기 지독하다.

그래서 ``pinky_fleet_sim/urdf/`` 에 자체 xacro 를 두고 규칙을 뒤집었다.

    링크를 가리키면  -> 접두사 **없음**
    그 외 전부      -> 접두사 **있음**  (조인트 이름, gz_frame_id, 토픽)

이 파일은 그 규칙이 되돌아가지 않는지 본다. xacro 를 전개하지 않고 텍스트로 검사하므로
ROS 없이 돈다 (기존 test_sim_setup.py 와 같은 방식).

upstream 을 찾지 못하면 upstream 대조가 필요한 테스트만 skip 한다. 규칙 검사 자체는
우리 파일만 보므로 어디서든 돈다.
"""

import os
import re

import pytest
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SIM = os.path.join(REPO, 'pinky_fleet_sim')

GZ_FLEET = os.path.join(SIM, 'urdf', 'pinky_gz_fleet.urdf.xacro')
TOP_XACRO = os.path.join(SIM, 'urdf', 'pinky_fleet.urdf.xacro')
SPAWN_LAUNCH = os.path.join(SIM, 'launch', 'gz_spawn.launch.xml')
CMAKELISTS = os.path.join(SIM, 'CMakeLists.txt')

NS = '${namespace}'

# upstream pinky_description 후보 경로. 실기에서는 워크스페이스 안이고, 개발 컨테이너에서는
# 저장소 옆에 체크아웃돼 있다. 환경변수로도 받는다.
UPSTREAM_CANDIDATES = [
    os.environ.get('PINKY_DESCRIPTION_DIR', ''),
    os.path.join(os.path.dirname(REPO), 'pinklab-art', 'pinky_pro', 'pinky_description'),
    os.path.join(os.path.dirname(REPO), 'pinky_pro', 'pinky_description'),
    os.path.join(REPO, '..', '..', 'pinky_pro', 'pinky_description'),
]

pytestmark = pytest.mark.skipif(
    not os.path.isdir(SIM), reason='pinky_fleet_sim 패키지가 없다')


def read(path):
    """파일을 읽되 **XML 주석을 지운다.**

    처음에는 날것으로 읽었는데, 이 저장소의 xacro/launch 는 주석에 예시 코드를 많이
    적어 두어서 정규식이 주석 속 <topic> 이나 reference= 를 본문으로 착각했다.
    "왜 안 고쳤는데 통과하지" 가 아니라 "왜 고쳤는데 실패하지" 쪽이라 바로 드러났다.
    """
    with open(path, encoding='utf-8') as handle:
        return re.sub(r'<!--.*?-->', '', handle.read(), flags=re.S)


def upstream_urdf():
    """upstream pinky.urdf.xacro 의 내용. 못 찾으면 None."""
    for base in UPSTREAM_CANDIDATES:
        if not base:
            continue
        path = os.path.join(base, 'urdf', 'pinky.urdf.xacro')
        if os.path.isfile(path):
            return read(path)
    return None


def upstream_gz_urdf():
    for base in UPSTREAM_CANDIDATES:
        if not base:
            continue
        path = os.path.join(base, 'urdf', 'pinky_gz.urdf.xacro')
        if os.path.isfile(path):
            return read(path)
    return None


def names(source, tag):
    """``<link name="X">`` / ``<joint name="X">`` 의 X 들."""
    return set(re.findall(rf'<{tag}\s+name="([^"]+)"', source))


def gazebo_references(source):
    return re.findall(r'<gazebo\s+reference="([^"]+)"', source)


@pytest.fixture(scope='module')
def ours():
    if not os.path.isfile(GZ_FLEET):
        pytest.skip('pinky_gz_fleet.urdf.xacro 가 없다')
    return read(GZ_FLEET)


@pytest.fixture(scope='module')
def upstream():
    source = upstream_urdf()
    if source is None:
        pytest.skip('upstream pinky_description 을 찾을 수 없다 '
                    '(PINKY_DESCRIPTION_DIR 로 지정 가능)')
    return source


@pytest.fixture(scope='module')
def upstream_links(upstream):
    """xacro 매크로 인자(${side} 등)가 든 이름은 실제 값으로 펼쳐 둔다."""
    raw = names(upstream, 'link')
    out = set()
    for name in raw:
        if '${side}' in name:
            out.update(name.replace('${side}', side) for side in ('l', 'r'))
        else:
            out.add(name)
    return out


@pytest.fixture(scope='module')
def upstream_joints(upstream):
    raw = names(upstream, 'joint')
    out = set()
    for name in raw:
        if '${side}' in name:
            out.update(name.replace('${side}', side) for side in ('l', 'r'))
        else:
            out.add(name)
    return out


# --- 접두사 규칙 (이번 버그 그 자체) --------------------------------------

def test_gazebo_reference_targets_exist(ours, upstream_links, upstream_joints):
    """참조하는 이름이 upstream 에 실제로 있어야 한다.

    없으면 sdformat 이 그 블록을 조용히 버린다. 이 테스트가 잡는 것이 바로 그 상황이다.
    """
    known = {name.replace(NS, '') for name in upstream_links | upstream_joints}
    missing = [ref for ref in gazebo_references(ours) if ref.replace(NS, '') not in known]
    assert not missing, f'upstream 에 없는 이름을 참조한다: {missing}'


def test_link_references_are_unprefixed(ours, upstream_links, upstream_joints):
    """링크를 가리키는 reference 에는 접두사가 **없어야** 한다.

    upstream 이 링크 이름에 ${namespace} 를 안 붙이기 때문이다. 붙이면 매칭이 깨져
    센서와 마찰이 사라진다.
    """
    plain_links = {name.replace(NS, '') for name in upstream_links}
    plain_joints = {name.replace(NS, '') for name in upstream_joints}
    bad = [ref for ref in gazebo_references(ours)
           if NS in ref and ref.replace(NS, '') in plain_links - plain_joints]
    assert not bad, (
        f'링크 참조에 {NS} 가 붙어 있다: {bad}. '
        'upstream 은 링크 이름에 접두사를 안 붙이므로 매칭이 깨진다')


def test_joint_references_are_prefixed(ours, upstream_links, upstream_joints):
    """조인트를 가리키는 reference 에는 접두사가 **있어야** 한다."""
    plain_links = {name.replace(NS, '') for name in upstream_links}
    plain_joints = {name.replace(NS, '') for name in upstream_joints}
    bad = [ref for ref in gazebo_references(ours)
           if NS not in ref and ref in plain_joints - plain_links]
    assert not bad, f'조인트 참조에 {NS} 가 빠졌다: {bad}'


def test_plugin_joint_names_are_prefixed(ours):
    """플러그인이 부르는 조인트 이름. URDF 가 실제로 접두사를 붙여 짓는다."""
    joints = re.findall(r'<(?:left_joint|right_joint|joint_name)>([^<]+)<', ours)
    assert joints, '플러그인 조인트 이름을 하나도 못 찾았다'
    bad = [name for name in joints if not name.startswith(NS)]
    assert not bad, f'접두사 없는 조인트 이름: {bad}'


def test_frame_ids_are_prefixed(ours):
    """TF 프레임 이름. robot_state_publisher 의 frame_prefix 와 맞아야 한다."""
    frames = re.findall(r'<(?:gz_frame_id|frame_id|child_frame_id)>([^<]+)<', ours)
    assert frames, '프레임 이름을 하나도 못 찾았다'
    bad = [name for name in frames if not name.startswith(NS)]
    assert not bad, f'접두사 없는 프레임 이름: {bad}'


def test_topics_are_prefixed_and_absolute(ours):
    """토픽은 로봇마다 갈라야 하고, 브리지 yaml 과 맞추려고 앞 슬래시를 붙인다."""
    topics = re.findall(r'<(?:topic|odom_topic)>([^<]+)<', ours)
    assert topics, '토픽을 하나도 못 찾았다'
    bad = [name for name in topics if not name.startswith(f'/{NS}')]
    assert not bad, f'/{NS} 로 시작하지 않는 토픽: {bad}'


def test_sensor_names_have_no_slash(ours):
    """gz 는 '/' 를 스코프 구분자로 쓴다. 센서 이름에 들어가면 위험하다.

    ${namespace} 는 여기서는 글자 그대로지만 전개되면 "pinky1/" 이 되므로, 펼친 뒤에
    본다. 이름만 보고 '/' 를 찾으면 접두사가 붙은 경우를 놓친다.
    """
    names_found = re.findall(r'<sensor\s+name="([^"]+)"', ours)
    assert names_found, '센서를 하나도 못 찾았다'
    bad = [name for name in names_found if '/' in name.replace(NS, 'pinky1/')]
    assert not bad, (
        f"센서 이름이 전개되면 '/' 가 들어간다: {bad}. "
        '센서는 이미 모델 안에 스코프되므로 접두사가 필요 없다')


# --- upstream 과의 값 drift ----------------------------------------------

def upstream_values(source, tag):
    """블록 안의 같은 이름 태그를 **전부** 모은다.

    <resolution> 은 vertical 과 range 에 둘 다 있어서 첫 번째만 보면 엉뚱한 것을 비교한다.
    """
    return [value.strip() for value in re.findall(rf'<{tag}>([^<]+)</{tag}>', source)]


LIDAR_TAGS = ['samples', 'min_angle', 'max_angle', 'min', 'max', 'resolution',
              'update_rate', 'stddev']
DIFFDRIVE_TAGS = ['wheel_separation', 'wheel_radius', 'min_velocity', 'max_velocity',
                  'min_acceleration', 'max_acceleration', 'odom_publisher_frequency',
                  'tf_topic']


def lidar_block(source):
    match = re.search(r'type=.gpu_lidar.*?</sensor>', source, re.S)
    return match.group(0) if match else ''


def diffdrive_block(source):
    match = re.search(r'DiffDrive.*?</plugin>', source, re.S)
    return match.group(0) if match else ''


@pytest.fixture(scope='module')
def upstream_gz():
    source = upstream_gz_urdf()
    if source is None:
        pytest.skip('upstream pinky_gz.urdf.xacro 를 찾을 수 없다')
    return source


@pytest.mark.parametrize('tag', LIDAR_TAGS)
def test_lidar_params_match_upstream(ours, upstream_gz, tag):
    """센서 스펙이 upstream 에서 갈라지면 시뮬이 실기와 달라진다."""
    mine = upstream_values(lidar_block(ours), tag)
    theirs = upstream_values(lidar_block(upstream_gz), tag)
    assert mine and mine == theirs, f'라이다 {tag}: 우리={mine} upstream={theirs}'


@pytest.mark.parametrize('tag', DIFFDRIVE_TAGS)
def test_diffdrive_params_match_upstream(ours, upstream_gz, tag):
    """wheel_separation 0.0961 은 조인트 간격(0.0811)과 다르다. 바꾸면 회전각이 틀어진다."""
    mine = upstream_values(diffdrive_block(ours), tag)
    theirs = upstream_values(diffdrive_block(upstream_gz), tag)
    assert mine and mine == theirs, f'DiffDrive {tag}: 우리={mine} upstream={theirs}'


# --- 최상위 xacro / launch 배선 -------------------------------------------

def test_toplevel_xacro_disables_upstream_gz():
    """is_sim="false" 가 아니면 upstream 의 가제보 블록이 같이 들어와 센서가 두 번 정의된다."""
    source = read(TOP_XACRO)
    match = re.search(r'insert_robot[^/]*?is_sim="([^"]+)"', source, re.S)
    assert match, '최상위 xacro 에서 insert_robot 의 is_sim 을 못 찾았다'
    assert match.group(1) == 'false', (
        f'is_sim="{match.group(1)}" 이면 upstream gz 블록이 같이 들어온다')


def test_toplevel_xacro_uses_our_gz_macro():
    source = read(TOP_XACRO)
    assert 'pinky_gz_fleet.urdf.xacro' in source
    assert 'insert_gz_fleet' in source


def test_spawn_launch_does_not_include_upstream_upload():
    """upload_robot.launch.py 를 되살리면 버그도 되살아난다 (그 파일은 robot.urdf.xacro 고정)."""
    source = read(SPAWN_LAUNCH)
    assert 'upload_robot.launch.py' not in source, (
        'upstream upload_robot.launch.py 를 다시 include 하고 있다. '
        '그 파일은 네임스페이스 모드에서 센서가 빠지는 robot.urdf.xacro 를 쓴다')


def test_spawn_launch_uses_our_xacro():
    source = read(SPAWN_LAUNCH)
    assert 'pinky_fleet.urdf.xacro' in source


def test_spawn_launch_passes_trailing_slash():
    """xacro 인자와 frame_prefix 가 **같은 문자열**이어야 한다.

    하나만 슬래시가 빠지면 조인트 이름 / gz_frame_id 와 실제 TF 프레임이 어긋난다.
    """
    source = read(SPAWN_LAUNCH)
    xacro_ns = re.search(r'namespace:=(\$\(var [a-z_]+\)/?)', source)
    prefix = re.search(r'<param\s+name="frame_prefix"\s+value="([^"]+)"', source)
    assert xacro_ns, 'xacro 에 namespace 인자를 안 넘긴다'
    assert prefix, 'frame_prefix 파라미터가 없다'
    assert xacro_ns.group(1) == prefix.group(1), (
        f'xacro namespace={xacro_ns.group(1)} != frame_prefix={prefix.group(1)}')
    assert prefix.group(1).endswith('/'), 'frame_prefix 는 끝에 슬래시가 있어야 한다'


def test_robot_description_param_is_str():
    """type="str" 이 없으면 launch 가 여러 줄 XML 의 타입을 추론하려 들다 깨진다."""
    source = read(SPAWN_LAUNCH)
    match = re.search(r'<param\s+name="robot_description"([^>]*)>', source, re.S)
    assert match, 'robot_description 파라미터가 없다'
    assert 'type="str"' in match.group(1), 'robot_description 에 type="str" 이 없다'


# --- 브리지 / 설치 ---------------------------------------------------------

@pytest.mark.parametrize('robot', ['pinky1', 'pinky2'])
def test_bridge_gz_topics_match_xacro(ours, robot):
    """브리지의 gz 토픽 이름이 xacro 의 <topic> 과 문자 그대로 같아야 한다.

    예전에는 "실제 이름이 다르면 gz topic -l 로 확인하라" 고 적어 두었는데, 그건
    확인해 줄 사람이 있을 때만 통한다. 양쪽을 못박고 여기서 대조한다.
    """
    path = os.path.join(SIM, 'params', f'{robot}_bridge.yaml')
    entries = yaml.safe_load(read(path))
    expected = {name.replace(NS, f'{robot}/')
                for name in re.findall(r'<(?:topic|odom_topic)>([^<]+)<', ours)}
    # /tf 는 DiffDrive 가 하드코딩한 것이라 xacro 의 <topic> 에 안 나온다.
    actual = {entry['gz_topic_name'] for entry in entries} - {'/tf'}
    missing = actual - expected
    assert not missing, (
        f'{robot}_bridge.yaml 이 xacro 에 없는 gz 토픽을 건다: {sorted(missing)}')


def test_urdf_is_installed():
    """설치가 안 되면 $(find-pkg-share pinky_fleet_sim)/urdf 가 비어 launch 가 죽는다."""
    source = read(CMAKELISTS)
    block = re.search(r'install\(DIRECTORY(.*?)DESTINATION', source, re.S)
    assert block, 'install(DIRECTORY ...) 를 못 찾았다'
    assert 'urdf' in block.group(1).split(), 'CMakeLists.txt 가 urdf 를 설치하지 않는다'


def test_command_substitution_tolerates_stderr():
    """``$(command ...)`` 의 on_stderr 기본값은 fail 이다.

    xacro 가 deprecation 경고 한 줄만 뱉어도 launch 전체가 죽는다. 두 번째 인자로
    warn 을 줘야 한다. 실기에서만 드러나는 종류라 여기서 못박는다.
    """
    source = read(SPAWN_LAUNCH)
    match = re.search(r'\$\(command\s+\'[^\']*\'([^)]*)\)', source)
    assert match, 'robot_description 에 $(command ...) 가 없다'
    assert 'warn' in match.group(1), (
        "$(command '...') 뒤에 warn 이 없다. 기본값 fail 이라 xacro 경고 한 줄에 launch 가 죽는다")


def test_our_macro_name_does_not_collide():
    """upstream 의 insert_gz_sim 과 이름이 같으면 둘 다 include 됐을 때 재정의 충돌이다."""
    source = read(GZ_FLEET)
    match = re.search(r'<xacro:macro\s+name="([^"]+)"', source)
    assert match, '매크로 정의를 못 찾았다'
    assert match.group(1) != 'insert_gz_sim', 'upstream 매크로와 이름이 같다'


def test_world_provides_the_sensors_system():
    """월드에 sensors 시스템이 없으면 xacro 를 아무리 고쳐도 /scan 이 안 나온다."""
    source = read(os.path.join(SIM, 'worlds', 'fleet_arena.sdf'))
    assert 'gz-sim-sensors-system' in source, (
        'fleet_arena.sdf 에 gz-sim-sensors-system 이 없다. 라이다가 아예 안 돈다')


def test_upstream_still_needs_our_workaround(upstream_gz):
    """upstream 이 고치면 우리 복사본을 버릴 수 있다.

    포크는 유지비다. upstream 이 링크 참조에서 접두사를 떼는 날 이 테스트가 실패하면서
    "이제 우리 xacro 가 필요 없다" 고 알려 준다.
    """
    still_broken = [ref for ref in gazebo_references(upstream_gz)
                    if ref.startswith(NS) and 'joint' not in ref]
    assert still_broken, (
        'upstream pinky_gz.urdf.xacro 가 링크 참조의 접두사를 뗀 것 같다. '
        'pinky_fleet_sim/urdf/ 의 복사본을 버리고 upstream 을 직접 쓸 수 있는지 확인할 것')
