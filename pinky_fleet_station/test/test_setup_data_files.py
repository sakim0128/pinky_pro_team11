"""setup.py 의 data_files 가 colcon 검사를 통과하는지 확인한다.

colcon 은 data_files 의 source 를 **패키지 디렉터리 기준 상대경로로만** 받는다
(colcon_core/task/python/__init__.py 의 get_data_files_mapping).

    assert not os.path.isabs(source), \
        f"'data_files' must be relative, '{source}' is absolute"

절대경로를 넣으면 빌드가 AssertionError 로 죽는다. 실제로 한 번 겪었기 때문에
같은 실수가 다시 들어오지 않도록 여기서 잡는다.
"""

import os
import runpy
import sys
from unittest import mock

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PACKAGES = ('pinky_fleet_station', 'pinky_fleet_agent')


def load_setup_kwargs(package):
    """setuptools.setup 을 가로채 setup.py 가 넘기는 인자를 그대로 받아온다."""
    package_dir = os.path.join(REPO_ROOT, package)
    setup_py = os.path.join(package_dir, 'setup.py')
    captured = {}
    cwd = os.getcwd()
    argv = sys.argv
    try:
        # colcon 도 패키지 디렉터리를 cwd 로 두고 setup.py 를 읽는다.
        os.chdir(package_dir)
        sys.argv = ['setup.py']
        with mock.patch('setuptools.setup', lambda **kw: captured.update(kw)):
            runpy.run_path(setup_py, run_name='__main__')
    finally:
        os.chdir(cwd)
        sys.argv = argv
    return package_dir, captured


@pytest.mark.parametrize('package', PACKAGES)
def test_data_files_sources_are_relative(package):
    _, kwargs = load_setup_kwargs(package)
    for destination, sources in kwargs['data_files']:
        assert not os.path.isabs(destination), destination
        for source in sources:
            assert not os.path.isabs(source), (
                f"'data_files' must be relative, '{source}' is absolute")


@pytest.mark.parametrize('package', PACKAGES)
def test_data_files_sources_exist(package):
    package_dir, kwargs = load_setup_kwargs(package)
    for _, sources in kwargs['data_files']:
        for source in sources:
            path = os.path.join(package_dir, source)
            assert os.path.isfile(path), path


def test_station_installs_all_config_yaml():
    package_dir, kwargs = load_setup_kwargs('pinky_fleet_station')
    installed = {
        os.path.basename(s)
        for dest, sources in kwargs['data_files'] if dest.endswith('config')
        for s in sources
    }
    on_disk = {
        f for f in os.listdir(os.path.join(package_dir, 'config')) if f.endswith('.yaml')
    }
    assert installed == on_disk
    # launch 파일들이 $(find-pkg-share ...)/config/ 아래에서 찾는 것들
    assert {'mission.yaml', 'bridge_fleet.yaml'} <= installed


def test_agent_installs_nav2_params():
    _, kwargs = load_setup_kwargs('pinky_fleet_agent')
    installed = {
        os.path.basename(s)
        for dest, sources in kwargs['data_files'] if dest.endswith('params')
        for s in sources
    }
    assert 'nav2_params_fleet.yaml' in installed
