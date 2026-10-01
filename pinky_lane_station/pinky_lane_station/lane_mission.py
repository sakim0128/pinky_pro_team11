"""lane_mission.yaml 로드 / 검증 — 로봇 목록(이름 · 도메인 · 토픽 · 속도 상한 · 기본 시작/목적지 노드).

lane_pipeline_node · fake_lane_robot 가 공유한다(로봇마다 토픽을 만든다). ROS 에 의존하지 않는다.
경로 배정·구간 예약·출발 순서는 여기 없다 — 코디네이터는 중계 관제국(relay_station/fleet) 하나이고 그 미션 파일은
relay_station/fleet/config/profiles/<프로파일>/lane_mission.yaml 이다. 시작·목적지는 중계 웹(/api/fleet/assign)에서 고른다.

    graph: ""                      # 비우면 pinky_lane_station/config/road_graph.yaml
    map: {yaml_path: ...}          # GUI 표시용
    defaults: {max_linear_vel: 0.15, max_angular_vel: 1.2}
    robots:
      - {name: pinky1, domain_id: 10, start: BL, goal: TR, color: "#ff5a7a"}
      - {name: pinky2, domain_id: 11, start: BR, goal: BL}
"""

import copy
import os

import yaml

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
            'defaults': copy.deepcopy(self.defaults),
            'robots': [{k: r[k] for k in ('name', 'domain_id', 'start', 'goal',
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
