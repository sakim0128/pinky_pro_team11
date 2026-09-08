"""ROS 없이 돌아가는 단위 테스트 (L0 의 일부).

    cd pinky_fleet && PYTHONPATH=. python3 -m pytest test -q
"""

import math
import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet.make_bridge_yaml import build_bridge_config          # noqa: E402
from pinky_fleet.mission_config import load_mission_config            # noqa: E402
from pinky_fleet.pose_utils import (                                  # noqa: E402
    normalize_deg,
    quaternion_from_yaw_deg,
    yaw_deg_between,
    yaw_deg_from_quaternion,
)

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config', 'mission.yaml')


def _write(tmp_path, mutate):
    with open(CONFIG, encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    mutate(raw)
    path = tmp_path / 'mission.yaml'
    path.write_text(yaml.safe_dump(raw), encoding='utf-8')
    return str(path)


# ------------------------------------------------------------------ pose_utils
@pytest.mark.parametrize('deg', [0.0, 37.5, 90.0, -120.0, 179.0])
def test_quaternion_roundtrip(deg):
    q = quaternion_from_yaw_deg(deg)
    assert yaw_deg_from_quaternion(*q) == pytest.approx(deg, abs=1e-6)


def test_quaternion_matches_euler_formula():
    # tf_transformations.quaternion_from_euler(0, 0, yaw) 와 동일함을 확인 (자료 p10)
    for deg in (0.0, 45.0, 123.4):
        r = math.radians(deg)
        assert quaternion_from_yaw_deg(deg) == pytest.approx(
            (0.0, 0.0, math.sin(r / 2), math.cos(r / 2)), abs=1e-12)


def test_normalize_deg_wraps():
    # 07강 정리 2번 — 각도 오차는 감싸서 계산해야 한다
    assert normalize_deg(370.0) == pytest.approx(10.0)
    assert normalize_deg(-190.0) == pytest.approx(170.0)


def test_yaw_between_two_clicks():
    assert yaw_deg_between(0, 0, 1, 1) == pytest.approx(45.0)
    assert yaw_deg_between(1, 1, 0, 1) == pytest.approx(180.0)


# --------------------------------------------------------------- mission_config
def test_default_config_loads():
    cfg = load_mission_config(CONFIG)
    assert [r.name for r in cfg.ordered_robots()] == ['pinky1', 'pinky2']
    assert cfg.control_domain_id not in [r.domain_id for r in cfg.robots]


def test_duplicate_domain_is_rejected(tmp_path):
    # 관제 도메인이 로봇 도메인과 겹치면 브리지가 자기 출력을 되받는다
    def mutate(raw):
        raw['control_domain_id'] = raw['robots'][0]['domain_id']
    with pytest.raises(ValueError, match='도메인'):
        load_mission_config(_write(tmp_path, mutate))


def test_unknown_robot_in_order_is_rejected(tmp_path):
    def mutate(raw):
        raw['mission']['order'] = ['pinky1', 'pinky9']
    with pytest.raises(KeyError):
        load_mission_config(_write(tmp_path, mutate))


def test_bad_goal_input_mode_is_rejected(tmp_path):
    def mutate(raw):
        raw['goal_input']['mode'] = 'three_click'
    with pytest.raises(ValueError, match='two_click'):
        load_mission_config(_write(tmp_path, mutate))


# ------------------------------------------------------------ make_bridge_yaml
def test_bridge_config_remaps_and_splits_per_robot():
    cfg = load_mission_config(CONFIG)
    docs = {r.name: build_bridge_config(cfg, r) for r in cfg.robots}

    for name, doc in docs.items():
        assert doc['to_domain'] == cfg.control_domain_id
        assert doc['from_domain'] == cfg.robot(name).domain_id
        # 두 로봇이 똑같이 /amcl_pose 를 쓰므로 remap 이 없으면 관제 도메인에서 겹친다
        assert doc['topics']['amcl_pose']['remap'] == f'{name}/amcl_pose'
        # /tf 는 frame 이름이 충돌하므로 절대 중계하지 않는다
        assert 'tf' not in doc['topics']

    # /map 은 한 대에서만 가져온다
    with_map = [n for n, d in docs.items() if 'map' in d['topics']]
    assert with_map == [cfg.map_from]
    assert docs[cfg.map_from]['topics']['map']['qos']['durability'] == 'transient_local'
