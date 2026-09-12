"""MapData 로딩과 좌표 변환 단위 테스트.

map_canvas 는 rclpy 에 의존하지 않으므로(PyQt5 / numpy / yaml 뿐) ROS 없이
헤드리스로 실제 코드를 돌려 검증할 수 있다.

    QT_QPA_PLATFORM=offscreen python3 -m pytest pinky_fleet_station/test -q
"""

import os
import struct
import sys

import pytest
import yaml

pytest.importorskip('numpy')
pytest.importorskip('PyQt5')

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_station.map_canvas import MapData, MapLoadError  # noqa: E402

# pinklab.yaml 과 같은 규격 (원본은 207x293 PNG)
RESOLUTION = 0.05
ORIGIN = [-0.65, -10.5, 0.0]
WIDTH, HEIGHT = 207, 293


@pytest.fixture(scope='session')
def qapp():
    """QPixmap 을 만들려면 QApplication 이 있어야 한다."""
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    yield app


def write_pgm(path, width, height, fill=254):
    """P5 (binary) PGM. nav2_map_server 가 뱉는 형식과 같다."""
    with open(path, 'wb') as handle:
        handle.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        handle.write(struct.pack('B', fill) * (width * height))


def write_png(path, width, height, fill=254):
    from PyQt5.QtGui import QImage
    image = QImage(width, height, QImage.Format_Grayscale8)
    image.fill(fill)
    assert image.save(path, 'PNG')


def make_map(tmp_path, image_name='map.pgm', width=WIDTH, height=HEIGHT, **overrides):
    image_path = str(tmp_path / image_name)
    if image_name.endswith('.png'):
        write_png(image_path, width, height)
    else:
        write_pgm(image_path, width, height)

    meta = {
        'image': image_name,
        'mode': 'trinary',
        'resolution': RESOLUTION,
        'origin': list(ORIGIN),
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    meta.update(overrides)
    yaml_path = tmp_path / 'map.yaml'
    yaml_path.write_text(yaml.safe_dump(meta), encoding='utf-8')
    return str(yaml_path)


# --- 로딩 -------------------------------------------------------------

def test_loads_pgm_metadata(tmp_path):
    m = MapData(make_map(tmp_path))
    assert m.resolution == RESOLUTION
    assert m.origin_x == ORIGIN[0]
    assert m.origin_y == ORIGIN[1]
    assert (m.width, m.height) == (WIDTH, HEIGHT)
    assert not m.image.isNull()


def test_loads_png(tmp_path):
    # 실제 pinklab.yaml 은 PGM 이 아니라 PNG 를 가리킨다.
    m = MapData(make_map(tmp_path, image_name='map.png'))
    assert (m.width, m.height) == (WIDTH, HEIGHT)
    assert not m.image.isNull()


def test_pixmap_is_lazy(tmp_path, qapp):
    """생성자는 QPixmap 을 만들지 않는다 (QApplication 없이도 로딩 가능)."""
    m = MapData(make_map(tmp_path))
    assert m._pixmap is None
    assert not m.pixmap.isNull()
    assert m.pixmap is m.pixmap          # 한 번만 만들고 캐시한다


def test_image_path_is_relative_to_yaml(tmp_path):
    sub = tmp_path / 'maps'
    sub.mkdir()
    m = MapData(make_map(sub))
    assert m.image_path == str(sub / 'map.pgm')


def test_summary_has_name_size_resolution(tmp_path):
    summary = MapData(make_map(tmp_path)).summary()
    assert 'map.yaml' in summary
    assert f'{WIDTH}x{HEIGHT}' in summary
    assert '0.05' in summary


# --- 좌표 변환 --------------------------------------------------------

def test_known_corner_values(tmp_path):
    """origin 은 이미지 좌하단의 월드 좌표다."""
    m = MapData(make_map(tmp_path))

    # 좌하단 픽셀 (0, H) -> origin 그 자체
    x, y = m.pixel_to_world(0, HEIGHT)
    assert x == pytest.approx(-0.65)
    assert y == pytest.approx(-10.50)

    # 우상단 픽셀 (W, 0)
    x, y = m.pixel_to_world(WIDTH, 0)
    assert x == pytest.approx(-0.65 + WIDTH * RESOLUTION)    # 9.70
    assert y == pytest.approx(-10.50 + HEIGHT * RESOLUTION)  # 4.15


def test_world_to_pixel_matches_known_corners(tmp_path):
    m = MapData(make_map(tmp_path))
    assert m.world_to_pixel(-0.65, -10.50) == pytest.approx((0.0, HEIGHT))
    assert m.world_to_pixel(9.70, 4.15) == pytest.approx((WIDTH, 0.0))


@pytest.mark.parametrize('wx,wy', [
    (0.0, 0.0), (-0.65, -10.5), (9.7, 4.15), (3.25, -5.0), (-0.1, 2.375),
])
def test_round_trip_world_pixel(tmp_path, wx, wy):
    m = MapData(make_map(tmp_path))
    px, py = m.world_to_pixel(wx, wy)
    rx, ry = m.pixel_to_world(px, py)
    assert rx == pytest.approx(wx, abs=1e-9)
    assert ry == pytest.approx(wy, abs=1e-9)


def test_y_axis_is_flipped(tmp_path):
    """월드 y 가 커지면 이미지 행 번호는 작아져야 한다."""
    m = MapData(make_map(tmp_path))
    _, py_low = m.world_to_pixel(0.0, -5.0)
    _, py_high = m.world_to_pixel(0.0, 0.0)
    assert py_high < py_low


def test_origin_offset_applied(tmp_path):
    m = MapData(make_map(tmp_path, origin=[0.0, 0.0, 0.0]))
    assert m.pixel_to_world(0, HEIGHT) == pytest.approx((0.0, 0.0))


# --- 에러 경로 --------------------------------------------------------

def test_missing_yaml(tmp_path):
    with pytest.raises(MapLoadError):
        MapData(str(tmp_path / 'nope.yaml'))


def test_yaml_without_image_key(tmp_path):
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump({'resolution': 0.05}), encoding='utf-8')
    with pytest.raises(MapLoadError):
        MapData(str(path))


def test_missing_image_file(tmp_path):
    path = tmp_path / 'map.yaml'
    path.write_text(
        yaml.safe_dump({'image': 'gone.pgm', 'resolution': 0.05}), encoding='utf-8')
    with pytest.raises(MapLoadError):
        MapData(str(path))


def test_expands_user_and_env(tmp_path, monkeypatch):
    yaml_path = make_map(tmp_path)
    monkeypatch.setenv('PINKY_TEST_MAP_DIR', str(tmp_path))
    m = MapData('$PINKY_TEST_MAP_DIR/map.yaml')
    assert m.yaml_path == yaml_path
