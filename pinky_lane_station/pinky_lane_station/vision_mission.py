"""비전 미션 관제 코어 — 시나리오·교차로 고정 동작 설정, 교차로 통행권(선착순), 출발·도착 진행. ROS 에 의존하지 않는다.

relay_station/fleet/vision_coordinator.py(ROS 노드) 가 이것을 감싼다.

    cfg = load_vision_config('config/vision_mission.yaml')
    run = ScenarioRun(cfg, 's1', t0=now, first_seq=1)
    run.on_status('pinky1', drive_state, route_seq, edge_id, now)   # LaneStatus 마다
    changed = run.tick(now)                                          # 10 Hz — 통행권 배정, 계획이 바뀐 로봇 이름들
    run.clearance('pinky1')                                          # LaneCommand CLEARANCE 값 (0 대기 · 1 통과)

통행권 규칙 (사용자 결정 2026-09-30)
    * 교차로는 하나, 통행권도 하나. 빨간 테이프 앞 정지(JUNCTION_STOP)를 **먼저 보고한** 로봇이 먼저 받는다.
      tie_window 안에 들어온 보고는 같은 순간으로 보고 domain_id 가 작은 로봇이 먼저다.
    * 쥔 로봇이 교차로 동작을 마치고 **차선 주행으로 돌아간 순간**(CRUISE, 교차로 뒤 단계) 반납 → 다음 로봇.
    * 흰 정지선 개수: 로봇에 stop_lines 가 있으면 그 값, 없으면 시나리오 stop_lines_by_order[통행권 받은 순서].
"""

import math
from dataclasses import dataclass, field

import yaml

# LaneStatus.drive_state (pinky_lane_msgs/msg/LaneStatus.msg)
DRIVE_IDLE, DRIVE_CRUISE, DRIVE_ARRIVED, DRIVE_ESTOP, DRIVE_LINK_LOST = 0, 1, 7, 8, 9
DRIVE_JUNCTION_STOP, DRIVE_JUNCTION_PASS = 12, 13
STAGE_AFTER = 'vision:after_junction'


class VisionConfigError(ValueError):
    pass


@dataclass
class RobotPlan:
    name: str
    domain_id: int
    start: str
    goal: str
    maneuver: str
    steps: list
    depart_delay: float = 0.0
    stop_lines: int = None          # None → 통행권 순서로 정한다


@dataclass
class Scenario:
    name: str
    label: str
    robots: dict                    # name -> RobotPlan
    stop_lines_by_order: list = field(default_factory=list)


@dataclass
class VisionConfig:
    linear_speed: float
    angular_speed: float
    tie_window: float
    robots: dict                    # name -> domain_id
    maneuvers: dict                 # name -> [(kind, value)]
    scenarios: dict                 # name -> Scenario
    path: str = ''


def _steps(name, raw):
    if not isinstance(raw, list) or not raw:
        raise VisionConfigError(f'maneuver {name}: 단계 목록이 비었다')
    out = []
    for i, st in enumerate(raw):
        if not isinstance(st, dict) or len(st) != 1:
            raise VisionConfigError(f'maneuver {name}[{i}]: {{straight: m}} 또는 {{turn: deg}} 하나여야 한다 — {st!r}')
        (kind, value), = st.items()
        kind = str(kind).strip().lower()
        if kind not in ('straight', 'turn'):
            raise VisionConfigError(f'maneuver {name}[{i}]: 알 수 없는 동작 {kind!r}')
        value = float(value)
        if not math.isfinite(value) or (kind == 'straight' and not 0 < abs(value) <= 2.0) \
                or (kind == 'turn' and not 0 < abs(value) <= 360.0):
            raise VisionConfigError(f'maneuver {name}[{i}]: 값 범위 밖 {value}')
        out.append((kind, value))
    return out


def vision_config_from_dict(data, path=''):
    data = data or {}
    speeds = data.get('speeds') or {}
    robots = {str(n): int((v or {}).get('domain_id', 0)) for n, v in (data.get('robots') or {}).items()}
    if not robots:
        raise VisionConfigError('robots 가 비었다')
    if len(set(robots.values())) != len(robots):
        raise VisionConfigError(f'domain_id 가 겹친다: {robots}')
    maneuvers = {str(n): _steps(n, v) for n, v in (data.get('maneuvers') or {}).items()}
    scenarios = {}
    for sname, sraw in (data.get('scenarios') or {}).items():
        sraw = sraw or {}
        plans = {}
        for rname, r in (sraw.get('robots') or {}).items():
            r = r or {}
            if rname not in robots:
                raise VisionConfigError(f'scenario {sname}: 모르는 로봇 {rname}')
            m = str(r.get('maneuver', ''))
            if m not in maneuvers:
                raise VisionConfigError(f'scenario {sname}/{rname}: 모르는 maneuver {m!r}')
            delay = float(r.get('depart_delay', 0.0))
            if delay < 0:
                raise VisionConfigError(f'scenario {sname}/{rname}: depart_delay < 0')
            sl = r.get('stop_lines')
            plans[rname] = RobotPlan(rname, robots[rname], str(r.get('start', '')), str(r.get('goal', '')), m,
                                     list(maneuvers[m]), delay, None if sl is None else int(sl))
        if not plans:
            raise VisionConfigError(f'scenario {sname}: 로봇이 없다')
        order = [int(x) for x in (sraw.get('stop_lines_by_order') or [])]
        for rname, plan in plans.items():
            if plan.stop_lines is None and len(order) < len(plans):
                raise VisionConfigError(f'scenario {sname}/{rname}: stop_lines 도 stop_lines_by_order(로봇 수만큼) 도 없다')
        scenarios[str(sname)] = Scenario(str(sname), str(sraw.get('label', sname)), plans, order)
    if not scenarios:
        raise VisionConfigError('scenarios 가 비었다')
    return VisionConfig(float(speeds.get('linear', 0.08)), float(speeds.get('angular', 0.5)),
                        float(data.get('tie_window', 0.5)), robots, maneuvers, scenarios, path)


def load_vision_config(path):
    with open(path, encoding='utf-8') as fh:
        return vision_config_from_dict(yaml.safe_load(fh), path)


class JunctionArbiter:
    """교차로 통행권 하나 — 먼저 선 로봇 먼저, tie_window 안이면 domain_id 작은 쪽."""

    def __init__(self, tie_window=0.5):
        self.tie_window = float(tie_window)
        self.requests = {}              # name -> (t, domain_id)
        self.holder = None
        self.grant_order = []           # 통행권을 받은 순서
        self.passed = set()

    def request(self, name, t, domain_id):
        if name in self.requests or name in self.passed or name == self.holder:
            return
        self.requests[name] = (float(t), int(domain_id))

    def release(self, name):
        if self.holder == name:
            self.holder = None
            self.passed.add(name)
            return True
        return False

    def queue(self):
        return sorted(self.requests, key=lambda n: self.requests[n])

    def tick(self, now):
        """통행권이 비었으면 배정. 새로 받은 로봇 이름(없으면 None)."""
        if self.holder is not None or not self.requests:
            return None
        first_t = min(t for t, _ in self.requests.values())
        if now < first_t + self.tie_window:
            return None                                      # 같은 순간 보고를 조금 더 기다린다
        tied = [n for n, (t, _) in self.requests.items() if t <= first_t + self.tie_window]
        name = min(tied, key=lambda n: (self.requests[n][1], self.requests[n][0]))
        del self.requests[name]
        self.holder = name
        self.grant_order.append(name)
        return name


class ScenarioRun:
    def __init__(self, cfg, scenario_name, t0, first_seq=1):
        if scenario_name not in cfg.scenarios:
            raise VisionConfigError(f'모르는 시나리오 {scenario_name!r}')
        self.cfg = cfg
        self.scenario = cfg.scenarios[scenario_name]
        self.t0 = float(t0)
        self.arbiter = JunctionArbiter(cfg.tie_window)
        self.seq = {}
        self.depart_at = {}
        self.stop_lines = {}
        self.stage = {}
        self.drive_state = {}
        self.arrived = {}
        for i, (name, plan) in enumerate(sorted(self.scenario.robots.items())):
            self.seq[name] = int(first_seq) + i
            self.depart_at[name] = self.t0 + plan.depart_delay
            self.stop_lines[name] = plan.stop_lines if plan.stop_lines is not None else 0
            self.stage[name] = 'vision:approach'
            self.drive_state[name] = DRIVE_IDLE
            self.arrived[name] = False

    @property
    def robots(self):
        return list(self.scenario.robots)

    @property
    def done(self):
        return all(self.arrived.values())

    def due(self, name, now):
        return name in self.depart_at and now >= self.depart_at[name]

    def plan_fields(self, name):
        """JunctionPlan 메시지 필드."""
        plan = self.scenario.robots[name]
        return {
            'robot_name': name, 'seq': self.seq[name], 'scenario': self.scenario.name, 'maneuver': plan.maneuver,
            'step_kind': [k for k, _ in plan.steps], 'step_value': [float(v) for _, v in plan.steps],
            'linear_speed': self.cfg.linear_speed, 'angular_speed': self.cfg.angular_speed,
            'stop_line_count': int(self.stop_lines[name]),
        }

    def on_status(self, name, drive_state, route_seq, edge_id, now):
        """LaneStatus 하나. 이 시나리오의 번호가 아니면 무시한다."""
        if name not in self.seq or int(route_seq) != self.seq[name]:
            return
        ds = int(drive_state)
        self.drive_state[name] = ds
        if edge_id:
            self.stage[name] = str(edge_id)
        if ds == DRIVE_JUNCTION_STOP:
            self.arbiter.request(name, now, self.scenario.robots[name].domain_id)
        elif self.arbiter.holder == name and (ds == DRIVE_ARRIVED or (ds == DRIVE_CRUISE and self.stage[name] == STAGE_AFTER)):
            self.arbiter.release(name)                         # 차선 주행으로 돌아갔다 → 다음 로봇
        if ds == DRIVE_ARRIVED:
            self.arrived[name] = True

    def tick(self, now):
        """통행권 배정. 계획(정지선 수)이 바뀐 로봇 이름 목록."""
        changed = []
        granted = self.arbiter.tick(now)
        if granted is not None and self.scenario.robots[granted].stop_lines is None:
            k = len(self.arbiter.grant_order) - 1
            order = self.scenario.stop_lines_by_order
            self.stop_lines[granted] = int(order[k]) if k < len(order) else int(order[-1])
            changed.append(granted)
        return changed

    def clearance(self, name):
        return 1 if (self.arbiter.holder == name or name in self.arbiter.passed) else 0

    def status(self, now):
        return {
            'scenario': self.scenario.name, 'label': self.scenario.label,
            'holder': self.arbiter.holder, 'queue': self.arbiter.queue(),
            'grant_order': list(self.arbiter.grant_order), 'done': self.done,
            'robots': {n: {
                'start': p.start, 'goal': p.goal, 'maneuver': p.maneuver,
                'steps': [[k, v] for k, v in p.steps], 'seq': self.seq[n],
                'stop_line_count': self.stop_lines[n], 'stage': self.stage[n],
                'drive_state': self.drive_state[n], 'arrived': self.arrived[n],
                'clearance': self.clearance(n),
                'depart_in': round(max(0.0, self.depart_at[n] - now), 1),
            } for n, p in self.scenario.robots.items()},
        }
