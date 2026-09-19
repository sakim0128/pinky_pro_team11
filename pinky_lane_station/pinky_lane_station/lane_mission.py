"""lane_mission.yaml 로드 / 검증 — 차선 주행 미션 (시작 노드 · 목적지 노드 · 출발 지연).

coordinator · fake_lane_robot · GUI 가 공유한다. ROS 에 의존하지 않는다.

    graph: ""                      # 비우면 pinky_lane_station/config/road_graph.yaml
    map: {yaml_path: ...}          # GUI 표시용
    reservation: {reserve_ahead: 0.40, release_behind: 0.25, node_stop_margin: 0.20}
    coordinator: {tick_rate: 10.0, state_timeout: 2.0, set_initial_pose: true, auto_start: false}
    defaults: {max_linear_vel: 0.15, max_angular_vel: 1.2}
    robots:
      - {name: pinky1, domain_id: 10, start: BL, goal: TC, depart_delay: 0.0, color: "#ff5a7a"}
      - {name: pinky2, domain_id: 11, start: BL, goal: RE, depart_delay: 6.0}
"""

import copy
import os

import yaml

DEFAULT_RESERVATION = {'reserve_ahead': 0.40, 'release_behind': 0.25, 'node_stop_margin': 0.20}
DEFAULT_COORDINATOR = {'tick_rate': 10.0, 'state_timeout': 2.0, 'set_initial_pose': True,
                       'auto_start': False, 'arrive_hold_seconds': 1.0}
DEFAULT_DEFAULTS = {'max_linear_vel': 0.15, 'max_angular_vel': 1.2}
COLORS = ('#ff5a7a', '#38bdf8', '#a3e635', '#fbbf24')


class LaneMissionError(ValueError):
    pass


def _num(raw, where, default=None):
    if raw is None:
        return default
    try:
        return float(raw)
    except (TypeError, ValueError) as exc:
        raise LaneMissionError(f'{where} 가 숫자가 아닙니다: {raw!r}') from exc


def default_graph_path():
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'config', 'road_graph.yaml')


class LaneMission:
    def __init__(self, data, path=None):
        if not isinstance(data, dict):
            raise LaneMissionError('lane_mission 의 최상위가 매핑이 아닙니다')
        self.path = path
        base = os.path.dirname(os.path.abspath(path)) if path else os.getcwd()
        g = str(data.get('graph') or '')
        g = os.path.expandvars(os.path.expanduser(g))
        if g and not os.path.isabs(g):
            g = os.path.join(base, g)
        self.graph_path = g or default_graph_path()
        self.map_yaml_path = os.path.expandvars(os.path.expanduser(
            str((data.get('map') or {}).get('yaml_path', ''))))

        self.reservation = dict(DEFAULT_RESERVATION)
        self.reservation.update(data.get('reservation') or {})
        for k, v in self.reservation.items():
            self.reservation[k] = _num(v, f'reservation.{k}')
        self.coordinator = dict(DEFAULT_COORDINATOR)
        self.coordinator.update(data.get('coordinator') or {})
        self.defaults = dict(DEFAULT_DEFAULTS)
        self.defaults.update(data.get('defaults') or {})

        robots = data.get('robots') or []
        if not robots:
            raise LaneMissionError('robots 항목에 최소 1대를 정의해야 합니다')
        self.robots = []
        names, domains = set(), set()
        for i, raw in enumerate(robots):
            if not isinstance(raw, dict):
                raise LaneMissionError(f'robots[{i}] 가 매핑이 아닙니다')
            name = str(raw.get('name') or f'robot{i}')
            if name in names:
                raise LaneMissionError(f'로봇 이름 중복: {name}')
            names.add(name)
            if 'domain_id' not in raw:
                raise LaneMissionError(f'{name}: domain_id 가 없습니다')
            domain_id = int(raw['domain_id'])
            if domain_id in domains:
                raise LaneMissionError(f'domain_id 중복: {domain_id}')
            domains.add(domain_id)
            self.robots.append({
                'name': name,
                'domain_id': domain_id,
                'start': str(raw.get('start') or ''),
                'goal': str(raw.get('goal') or ''),
                'depart_delay': _num(raw.get('depart_delay'), f'{name}.depart_delay', 0.0),
                'color': str(raw.get('color') or COLORS[i % len(COLORS)]),
                'state_topic': str(raw.get('state_topic') or f'/{name}/state'),
                'command_topic': str(raw.get('command_topic') or f'/{name}/command'),
                'route_topic': str(raw.get('route_topic') or f'/{name}/route'),
                'lane_command_topic': str(raw.get('lane_command_topic') or f'/{name}/lane_command'),
                'lane_status_topic': str(raw.get('lane_status_topic') or f'/{name}/lane_status'),
                'lane_path_topic': str(raw.get('lane_path_topic') or f'/{name}/lane_path'),
                'image_topic': str(raw.get('image_topic') or f'/{name}/camera/image/compressed'),
                'max_linear_vel': _num(raw.get('max_linear_vel'), f'{name}.max_linear_vel',
                                       float(self.defaults['max_linear_vel'])),
                'max_angular_vel': _num(raw.get('max_angular_vel'), f'{name}.max_angular_vel',
                                        float(self.defaults['max_angular_vel'])),
            })

    def robot(self, name):
        for r in self.robots:
            if r['name'] == name:
                return r
        raise KeyError(name)

    def by_priority(self):
        return sorted(self.robots, key=lambda r: r['domain_id'])

    def validate_against_graph(self, graph):
        """start/goal 이 그래프 노드인지, 경로가 있는지. 문제 목록을 돌려준다 (비면 OK)."""
        problems = []
        for r in self.robots:
            for key in ('start', 'goal'):
                if r[key] and r[key] not in graph.nodes:
                    problems.append(f"{r['name']}.{key} = {r[key]!r} 노드가 그래프에 없습니다")
            if r['start'] and r['goal'] and r['start'] in graph.nodes and r['goal'] in graph.nodes:
                if r['start'] == r['goal']:
                    problems.append(f"{r['name']}: start 와 goal 이 같습니다")
                else:
                    try:
                        graph.shortest_route(r['start'], r['goal'])
                    except Exception as exc:  # noqa: BLE001
                        problems.append(f"{r['name']}: {exc}")
        return problems

    def to_dict(self):
        return {
            'graph': self.graph_path,
            'map': {'yaml_path': self.map_yaml_path},
            'reservation': copy.deepcopy(self.reservation),
            'coordinator': copy.deepcopy(self.coordinator),
            'defaults': copy.deepcopy(self.defaults),
            'robots': [{k: r[k] for k in ('name', 'domain_id', 'start', 'goal', 'depart_delay',
                                          'color', 'max_linear_vel', 'max_angular_vel')}
                       for r in self.robots],
        }


def load_lane_mission(path):
    path = os.path.expandvars(os.path.expanduser(str(path)))
    if not os.path.isfile(path):
        raise LaneMissionError(f'lane_mission 파일을 찾을 수 없습니다: {path}')
    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    return LaneMission(data, path=path)


def save_lane_mission(mission, path=None):
    target = os.path.expandvars(os.path.expanduser(str(path or mission.path)))
    with open(target, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(mission.to_dict(), fh, allow_unicode=True, sort_keys=False,
                       default_flow_style=False)
    mission.path = target
    return target
