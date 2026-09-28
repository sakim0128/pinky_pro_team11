"""Read-only boundary, independent freshness, JSON and HTTP integration."""
import ast
import json
from pathlib import Path
import sys
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pinky_fleet_station.live_state import StateStore
from pinky_fleet_station.live_http import make_server
from pinky_fleet_station.live_map import MapAsset, MapAssetError
from pinky_fleet_station.live_frames import FrameStore
from pinky_fleet_station.live_control import ControlQueue
from pinky_fleet_station.overhead_math import transform_point


MAP = Path(__file__).resolve().parents[1] / 'config/map5.yaml'


def test_freshness_independent_and_snapshot_isolation():
    now = [10.0]
    store = StateStore(['pinky1'], clock=lambda: now[0])
    assert store.snapshot()['health'] == 'waiting'
    store.update(('pinky1', 'state'), {'battery': float('nan')})
    store.update(('pinky1', 'lane'), {'drive_state': 1})
    store.update('mission', {'mission': 'RUNNING'})
    assert store.snapshot()['health'] == 'online'
    now[0] += 3
    store.update(('pinky1', 'state'), {'x': 1.0})
    result = store.snapshot()
    assert result['robots'][0]['lane']['status'] == 'stale'
    assert result['robots'][0]['state']['status'] == 'live'
    assert result['robots'][0]['overhead']['status'] == 'missing'
    assert result['health'] == 'degraded'
    result['robots'][0]['state']['data']['x'] = 99
    assert store.snapshot()['robots'][0]['state']['data']['x'] == 1


def test_http_missing_nonfinite_and_no_control():
    store = StateStore(['pinky1'])
    store.update(('pinky1', 'state'), {'battery': float('nan'), 'nested': [float('inf')]})
    server = make_server('127.0.0.1', 0, store, MapAsset(MAP))
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    root = 'http://127.0.0.1:' + str(server.server_port)
    try:
        with urlopen(root + '/api/state') as response:
            data = json.load(response)
        assert data['robots'][0]['state']['data'] == {'battery': None, 'nested': [None]}
        assert data['robots'][0]['lane']['data'] is None
        with urlopen(root + '/') as response:
            assert '조회 전용' in response.read().decode()
        with urlopen(root + '/api/map/metadata') as response:
            metadata = json.load(response)
        assert metadata['name'] == 'map5'
        assert metadata['resolution'] == .01
        with urlopen(root + '/api/map/image.png') as response:
            assert response.read(8) == b'\x89PNG\r\n\x1a\n'
        for path, method, status in [('/api/command', 'POST', 404), ('/../setup.py', 'GET', 404)]:
            with pytest.raises(HTTPError) as error:
                urlopen(Request(root + path, method=method))
            assert error.value.code == status
        with pytest.raises(HTTPError) as error:
            urlopen(Request(root + '/api/control', data=b'{"action":"start"}', method='POST'))
        assert error.value.code == 403
    finally:
        server.shutdown()
        thread.join()
        server.server_close()


def test_control_plane_is_opt_in_and_whitelisted():
    path = Path(__file__).resolve().parents[1] / 'pinky_fleet_station/live_web_node.py'
    assert "declare_parameter('enable_control', False)" in path.read_text()
    queue = ControlQueue(enabled=False)
    assert not queue.submit({'action': 'start'})
    queue = ControlQueue(enabled=True)
    assert queue.submit({'action': 'resume'})
    assert queue.submit({'action': 'speed', 'robot': 'pinky1', 'linear': .15, 'angular': 1.2})
    assert not queue.submit({'action': 'speed', 'robot': 'pinky1', 'linear': 99, 'angular': 1.2})
    assert not queue.submit({'action': 'goto'})


def test_amcl_adapter_keeps_json_primitives_only():
    path = Path(__file__).resolve().parents[1] / 'pinky_fleet_station/live_web_node.py'
    source = path.read_text()
    assert "PoseWithCovarianceStamped" in source
    assert "list(msg.pose.covariance)" in source


def test_dashboard_uses_normalized_design_lane_overlay_and_control_status():
    static = Path(__file__).resolve().parents[1] / 'pinky_fleet_station/live_static'
    html = (static / 'index.html').read_text()
    script = (static / 'app.js').read_text()
    assert 'design-lane-art' in html
    assert 'control-status' in html
    assert 'map5 + 설계 차선 정렬' in script
    assert '차선만' not in html
    assert 'qualityLabels' in script
    assert 'overhead-system-status' in html


@pytest.mark.parametrize('timeout', [0, -1, float('nan'), float('inf')])
def test_invalid_timeout(timeout):
    with pytest.raises(ValueError):
        StateStore(['pinky1'], timeout=timeout)


def test_map_asset_has_nav2_dimensions_and_rejects_bad_pgm(tmp_path):
    asset = MapAsset(MAP)
    assert (asset.width, asset.height) == (236, 128)
    assert asset.metadata['world_width'] == pytest.approx(2.36)
    assert asset.metadata['world_height'] == pytest.approx(1.28)
    assert asset.compatibility(None) == 'state_missing'
    assert asset.compatibility({'map_known': False}) == 'robot_map_unknown'
    assert asset.compatibility({'map_known': True, 'map_width': 236, 'map_height': 128,
                                'map_resolution': .01, 'map_origin_x': -.01, 'map_origin_y': -.01}) == 'compatible'
    assert asset.compatibility({'map_known': True, 'map_width': 1, 'map_height': 128,
                                'map_resolution': .01, 'map_origin_x': -.01, 'map_origin_y': -.01}) == 'mismatch:width'
    bad = tmp_path / 'bad.pgm'
    bad.write_bytes(b'P2\n1 1\n255\n0\n')
    yaml_path = tmp_path / 'bad.yaml'
    yaml_path.write_text('image: bad.pgm\nresolution: 0.01\norigin: [0, 0, 0]\n')
    with pytest.raises(MapAssetError):
        MapAsset(yaml_path)


def test_frame_store_is_bounded_jpeg_only_and_expires():
    now = [0.]
    frames = FrameStore(['pinky1'], timeout=2., maximum_bytes=10, clock=lambda: now[0])
    jpeg = b'\xff\xd8ok\xff\xd9'
    assert frames.update_jpeg('pinky1', jpeg)
    assert frames.get('pinky1') == jpeg
    assert not frames.update_jpeg('pinky2', jpeg)
    assert not frames.update_jpeg('pinky1', b'not-a-jpeg')
    now[0] = 3.
    assert frames.get('pinky1') is None
    assert frames.snapshot()['pinky1']['status'] == 'stale'


def test_overhead_homography_projects_pixels_to_map_without_mock_pose():
    identity = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    assert transform_point(identity, (1.25, .75)) == pytest.approx((1.25, .75))
    assert transform_point([1., 0., 0., 0., 1., 0., 0., 0., 0.], (1., 1.)) is None
