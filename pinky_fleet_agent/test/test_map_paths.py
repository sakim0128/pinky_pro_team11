"""맵 이름 -> 경로 변환 단위 테스트.

이름은 FleetCommand 로 네트워크를 타고 들어오므로 경로 탈출을 반드시 막아야 한다.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet_agent.map_paths import (  # noqa: E402
    normalize_map_name, resolve_map_path,
)


@pytest.fixture
def map_dir(tmp_path):
    d = tmp_path / 'map'
    d.mkdir()
    (d / 'pinklab.yaml').write_text('image: pinklab.png\n', encoding='utf-8')
    (d / 'my_map.yaml').write_text('image: my_map.pgm\n', encoding='utf-8')
    return str(d)


# --- 이름 정규화 --------------------------------------------------------

@pytest.mark.parametrize('given,expected', [
    ('pinklab', 'pinklab'),
    ('pinklab.yaml', 'pinklab'),
    ('pinklab.yml', 'pinklab'),
    ('  pinklab  ', 'pinklab'),
    ('pinklab.YAML', 'pinklab'),
    ('/home/sungah/maps/pinklab.yaml', 'pinklab'),   # 전체 경로가 와도 이름만
    ('../../etc/passwd', 'passwd'),                  # 경로 성분 제거
    ('a/b/c', 'c'),
    ('maps\\pinklab.yaml', 'pinklab'),               # 윈도우식 구분자
    ('pinklab.pgm', 'pinklab.pgm'),                  # 맵 확장자가 아니면 그대로
])
def test_normalize(given, expected):
    assert normalize_map_name(given) == expected


@pytest.mark.parametrize('given', ['', None, '   ', '.', '..', '/', '../', 'a/..'])
def test_normalize_rejects(given):
    assert normalize_map_name(given) is None


# --- 경로 해석 ----------------------------------------------------------

def test_resolves_existing_map(map_dir):
    path, error = resolve_map_path(map_dir, 'pinklab')
    assert error is None
    assert path == os.path.join(map_dir, 'pinklab.yaml')


def test_extension_is_optional(map_dir):
    a, _ = resolve_map_path(map_dir, 'pinklab')
    b, _ = resolve_map_path(map_dir, 'pinklab.yaml')
    c, _ = resolve_map_path(map_dir, 'pinklab.yml')
    assert a == b == c


def test_full_path_is_reduced_to_name(map_dir):
    """관제 PC 의 전체 경로가 실수로 와도 로봇 디렉터리에서 찾는다."""
    path, error = resolve_map_path(map_dir, '/home/sungah/maps/pinklab.yaml')
    assert error is None
    assert path == os.path.join(map_dir, 'pinklab.yaml')


def test_missing_map_is_an_error(map_dir):
    path, error = resolve_map_path(map_dir, 'nope')
    assert path is None
    assert '찾을 수 없습니다' in error
    assert 'nope.yaml' in error


@pytest.mark.parametrize('bad', ['', '.', '..', '/'])
def test_invalid_name_is_an_error(map_dir, bad):
    path, error = resolve_map_path(map_dir, bad)
    assert path is None
    assert '올바르지 않습니다' in error


# --- 경로 탈출 방어 -----------------------------------------------------

@pytest.mark.parametrize('attack', [
    '../../etc/passwd',
    '/etc/passwd',
    '../pinklab',
    '....//....//etc/passwd',
])
def test_path_traversal_cannot_escape(map_dir, attack):
    path, error = resolve_map_path(map_dir, attack)
    # 이름만 남으므로 map_dir 안에서 못 찾거나, 찾더라도 map_dir 안이다.
    if path is not None:
        assert os.path.commonpath([os.path.realpath(map_dir), path]) \
            == os.path.realpath(map_dir)
    else:
        assert error


def test_traversal_to_a_real_file_outside_is_blocked(tmp_path):
    """상위 디렉터리에 같은 이름의 맵이 있어도 그쪽을 읽으면 안 된다."""
    outside = tmp_path / 'secret.yaml'
    outside.write_text('image: x.pgm\n', encoding='utf-8')
    inner = tmp_path / 'map'
    inner.mkdir()

    path, error = resolve_map_path(str(inner), '../secret')
    assert path is None                      # basename 후 inner/secret.yaml -> 없음
    assert '찾을 수 없습니다' in error


def test_symlink_out_of_map_dir_is_blocked(tmp_path):
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'evil.yaml').write_text('image: x.pgm\n', encoding='utf-8')
    inner = tmp_path / 'map'
    inner.mkdir()
    os.symlink(str(outside / 'evil.yaml'), str(inner / 'evil.yaml'))

    path, error = resolve_map_path(str(inner), 'evil')
    assert path is None
    assert '밖을 가리킵니다' in error


def test_expands_user_and_env(map_dir, monkeypatch):
    monkeypatch.setenv('PINKY_TEST_MAP_DIR', map_dir)
    path, error = resolve_map_path('$PINKY_TEST_MAP_DIR', 'pinklab')
    assert error is None
    assert path == os.path.join(map_dir, 'pinklab.yaml')
