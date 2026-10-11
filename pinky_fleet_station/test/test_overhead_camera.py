"""overhead_camera — 웹캠 노드의 ROS 없는 부분 + launch 배선 (rclpy 불필요).

overhead_camera.py 는 파라미터 검증·JPEG 인코딩만 갖고 있어 rclpy 없이 돈다. 노드 본체(overhead_camera_node.py)는
rclpy 를 import 하므로 여기서는 소스만 읽어 QoS 가 추적기 구독과 맞는지 본다.
"""

import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_station.overhead_camera import (DEFAULT_FRAME_ID, DEFAULT_TOPIC,  # noqa: E402
                                                 encode_jpeg, parse_device, validate_params)

STATION = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(STATION, 'launch', 'overhead_tracker.launch.xml')
CONFIG = os.path.join(STATION, 'config', 'overhead_tracker.yaml')
NODE = os.path.join(STATION, 'pinky_fleet_station', 'overhead_camera_node.py')
TRACKER = os.path.join(STATION, 'pinky_fleet_station', 'overhead_tracker_node.py')
SETUP = os.path.join(STATION, 'setup.py')


def read(path):
    with open(path, encoding='utf-8') as handle:
        return handle.read()


# --- 파라미터 -------------------------------------------------------------

@pytest.mark.parametrize('value, expected', [
    (0, 0), (2, 2), ('0', 0), (' 3 ', 3),
    ('/dev/video2', '/dev/video2'),
    ('v4l2src device=/dev/video0 ! videoconvert ! appsink', 'v4l2src device=/dev/video0 ! videoconvert ! appsink'),
])
def test_parse_device_int_or_path(value, expected):
    assert parse_device(value) == expected


@pytest.mark.parametrize('value', [-1, True, '', '   ', 1.5, None])
def test_parse_device_rejects(value):
    with pytest.raises(ValueError):
        parse_device(value)


def test_validate_params_defaults_and_coercion():
    assert validate_params(1280, 720, 15, 80) == (1280, 720, 15.0, 80)
    assert validate_params('640', '480', '10', '70') == (640, 480, 10.0, 70)


@pytest.mark.parametrize('args', [
    (0, 720, 15, 80), (1280, -1, 15, 80), (1280, 720, 0, 80), (1280, 720, float('inf'), 80),
    (1280, 720, float('nan'), 80), (1280, 720, 15, 0), (1280, 720, 15, 101), ('x', 720, 15, 80),
])
def test_validate_params_rejects(args):
    with pytest.raises(ValueError):
        validate_params(*args)


# --- JPEG -----------------------------------------------------------------

def test_encode_jpeg_roundtrip():
    cv2 = pytest.importorskip('cv2')
    frame = np.zeros((48, 64, 3), dtype=np.uint8)
    frame[:, 32:] = (0, 0, 255)
    data = encode_jpeg(cv2, frame, 80)
    assert data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9'           # SOI / EOI
    back = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert back.shape == frame.shape
    # 품질을 낮추면 작아진다
    assert len(encode_jpeg(cv2, frame, 20)) <= len(encode_jpeg(cv2, frame, 95))


def test_encode_jpeg_empty_frame_is_none():
    cv2 = pytest.importorskip('cv2')
    assert encode_jpeg(cv2, None, 80) is None
    assert encode_jpeg(cv2, np.zeros((0, 0, 3), dtype=np.uint8), 80) is None


# --- 노드 ↔ 추적기 ↔ launch ↔ yaml 배선 ----------------------------------------

def test_default_topic_matches_tracker_and_yaml():
    """웹캠이 내는 토픽 == 추적기가 구독하는 토픽 == yaml == launch 인자."""
    config = yaml.safe_load(read(CONFIG))['overhead_tracker']['ros__parameters']
    assert config['image_topic'] == DEFAULT_TOPIC
    assert f"declare_parameter('image_topic', '{DEFAULT_TOPIC}')" in read(TRACKER)
    root = ET.parse(LAUNCH).getroot()
    args = {a.get('name'): a for a in root.findall('arg')}
    assert args['image_topic'].get('default') == DEFAULT_TOPIC
    assert DEFAULT_FRAME_ID == 'overhead_camera'


def test_publisher_qos_is_compatible_with_tracker_subscription():
    """추적기는 기본 프로파일(RELIABLE)로 구독한다. BEST_EFFORT 발행자는 그 구독자와 연결되지 않는다."""
    assert re.search(r'create_subscription\(CompressedImage,.*,\s*10\)', read(TRACKER))
    node = read(NODE)
    assert 'QoSReliabilityPolicy.RELIABLE' in node
    assert 'QoSReliabilityPolicy.BEST_EFFORT' not in node
    assert 'depth=1' in node
    assert "msg.format = 'jpeg'" in node


def test_launch_starts_camera_only_when_use_camera_and_forwards_every_arg():
    root = ET.parse(LAUNCH).getroot()
    args = {a.get('name'): a.get('default') for a in root.findall('arg')}
    assert args['use_camera'] == 'True'
    assert args['camera_device'] == '0'
    assert (args['camera_width'], args['camera_height']) == ('1280', '720')
    assert float(args['camera_fps']) == 15.0
    assert int(args['camera_jpeg_quality']) == 80

    nodes = {n.get('exec'): n for n in root.findall('node')}
    assert 'overhead_tracker_node' in nodes and nodes['overhead_tracker_node'].get('if') is None
    camera = nodes['overhead_camera_node']
    assert camera.get('if') == '$(var use_camera)'
    assert camera.get('respawn') == 'true'
    params = {p.get('name'): p.get('value') for p in camera.findall('param')}
    assert params == {'device': '$(var camera_device)', 'width': '$(var camera_width)',
                      'height': '$(var camera_height)', 'fps': '$(var camera_fps)',
                      'jpeg_quality': '$(var camera_jpeg_quality)', 'topic': '$(var image_topic)'}
    # 만들어 놓고 안 넘기는 인자가 없다 (params 는 추적기 <param from> 이 쓴다)
    used = set(re.findall(r'\$\(var ([a-z_]+)\)', read(LAUNCH)))
    assert set(args) <= used


def test_launch_and_node_defaults_agree():
    """launch 가 <param> 으로 명시 전달하므로 launch 값이 실제 동작이다 — 노드 기본값과 어긋나면 안 된다."""
    node = read(NODE)
    declared = dict(re.findall(r"declare_parameter\('([a-z_]+)',\s*([-\d.]+)", node))
    root = ET.parse(LAUNCH).getroot()
    args = {a.get('name'): a.get('default') for a in root.findall('arg')}
    for node_key, arg_key in [('width', 'camera_width'), ('height', 'camera_height'),
                              ('fps', 'camera_fps'), ('jpeg_quality', 'camera_jpeg_quality'),
                              ('device', 'camera_device')]:
        assert float(declared[node_key]) == float(args[arg_key]), (node_key, arg_key)


def test_setup_installs_camera_console_script():
    assert "'overhead_camera_node = pinky_fleet_station.overhead_camera_node:main'" in read(SETUP)
