"""mission.yaml 로드 / 저장 / 검증.

coordinator_node 와 gui_node 가 공유한다. ROS 에 의존하지 않아 단독 테스트가 가능하다.
"""

import copy
import os

import yaml

DEFAULT_COORDINATOR = {
    'conflict_distance': 0.70,
    'clear_distance': 1.00,
    'stall_speed': 0.03,
    'stall_duration': 3.0,
    'resume_timeout': 30.0,
    'cooldown': 2.0,
    'state_timeout': 2.0,
}

DEFAULT_DEFAULTS = {
    'max_linear_vel': 0.20,
    'max_angular_vel': 1.50,
}

_POSE_KEYS = ('x', 'y', 'yaw')


class MissionError(ValueError):
    """mission.yaml 의 내용이 잘못되었을 때."""


def _pose(raw, where):
    if raw is None:
        return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    if not isinstance(raw, dict):
        raise MissionError(f'{where} 는 {{x:, y:, yaw:}} 형식이어야 합니다: {raw!r}')
    out = {}
    for key in _POSE_KEYS:
        try:
            out[key] = float(raw.get(key, 0.0))
        except (TypeError, ValueError) as exc:
            raise MissionError(f'{where}.{key} 가 숫자가 아닙니다: {raw.get(key)!r}') from exc
    return out


class Mission:
    """mission.yaml 한 벌을 담는 값 객체."""

    def __init__(self, data, path=None):
        self.path = path
        self.map_yaml_path = os.path.expandvars(
            os.path.expanduser(str((data.get('map') or {}).get('yaml_path', ''))))

        self.defaults = dict(DEFAULT_DEFAULTS)
        self.defaults.update(data.get('defaults') or {})

        self.coordinator = dict(DEFAULT_COORDINATOR)
        self.coordinator.update(data.get('coordinator') or {})
        for key, value in self.coordinator.items():
            try:
                self.coordinator[key] = float(value)
            except (TypeError, ValueError) as exc:
                raise MissionError(f'coordinator.{key} 가 숫자가 아닙니다: {value!r}') from exc

        robots = data.get('robots') or []
        if len(robots) < 2:
            raise MissionError('robots 항목에 최소 2대를 정의해야 합니다.')

        self.robots = []
        seen_names = set()
        seen_domains = set()
        for index, raw in enumerate(robots):
            if not isinstance(raw, dict):
                raise MissionError(f'robots[{index}] 가 매핑이 아닙니다.')
            name = str(raw.get('name') or f'robot{index}')
            if name in seen_names:
                raise MissionError(f'로봇 이름이 중복됩니다: {name}')
            seen_names.add(name)

            if 'domain_id' not in raw:
                raise MissionError(f'{name}: domain_id 가 없습니다 (우선순위 결정에 필요).')
            domain_id = int(raw['domain_id'])
            if domain_id in seen_domains:
                raise MissionError(f'domain_id 가 중복됩니다: {domain_id}')
            seen_domains.add(domain_id)

            self.robots.append({
                'name': name,
                'domain_id': domain_id,
                'state_topic': str(raw.get('state_topic') or f'/{name}/state'),
                'command_topic': str(raw.get('command_topic') or f'/{name}/command'),
                'color': str(raw.get('color') or '#ff5a7a'),
                'initial_pose': _pose(raw.get('initial_pose'), f'{name}.initial_pose'),
                'goal': _pose(raw.get('goal'), f'{name}.goal'),
                'max_linear_vel': float(
                    raw.get('max_linear_vel', self.defaults['max_linear_vel'])),
                'max_angular_vel': float(
                    raw.get('max_angular_vel', self.defaults['max_angular_vel'])),
            })

    # --- 조회 ---------------------------------------------------------

    def robot(self, name):
        for robot in self.robots:
            if robot['name'] == name:
                return robot
        raise KeyError(name)

    def by_priority(self):
        """domain_id 오름차순 = 우선순위 높은 순 (미션 5번)."""
        return sorted(self.robots, key=lambda r: r['domain_id'])

    # --- 직렬화 -------------------------------------------------------

    def to_dict(self):
        return {
            'map': {'yaml_path': self.map_yaml_path},
            'defaults': copy.deepcopy(self.defaults),
            'coordinator': copy.deepcopy(self.coordinator),
            'robots': [
                {
                    'name': r['name'],
                    'domain_id': r['domain_id'],
                    'state_topic': r['state_topic'],
                    'command_topic': r['command_topic'],
                    'color': r['color'],
                    'initial_pose': copy.deepcopy(r['initial_pose']),
                    'goal': copy.deepcopy(r['goal']),
                    'max_linear_vel': r['max_linear_vel'],
                    'max_angular_vel': r['max_angular_vel'],
                }
                for r in self.robots
            ],
        }


def load_mission(path):
    path = os.path.expandvars(os.path.expanduser(str(path)))
    if not os.path.isfile(path):
        raise MissionError(f'mission 파일을 찾을 수 없습니다: {path}')
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise MissionError(f'mission 파일의 최상위가 매핑이 아닙니다: {path}')
    return Mission(data, path=path)


def save_mission(mission, path=None):
    target = os.path.expandvars(os.path.expanduser(str(path or mission.path)))
    if not target:
        raise MissionError('저장 경로가 지정되지 않았습니다.')
    with open(target, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(
            mission.to_dict(), handle,
            allow_unicode=True, sort_keys=False, default_flow_style=False)
    mission.path = target
    return target
