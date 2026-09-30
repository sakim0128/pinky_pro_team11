"""비전 미션 관제 코어 — 시나리오·교차로 방향·고정 동작, 교차로 통행권(선착순), 출발·도착 진행. ROS 에 의존하지 않는다.

relay_station/fleet/vision_coordinator.py(ROS 노드) 가 이것을 감싼다.

    cfg = load_vision_config('config/vision_mission.yaml')
    load_user_scenarios(cfg, user_path)                              # 웹에서 저장한 시나리오 (있으면)
    run = ScenarioRun(cfg, 's1', t0=now, first_seq=1)               # 이름 또는 Scenario (웹 직접 설정)
    run.on_status('pinky1', drive_state, route_seq, edge_id, now)   # LaneStatus 마다
    changed = run.tick(now)                                          # 10 Hz — 통행권 배정 등, 계획이 바뀐 로봇 이름들
    run.clearance('pinky1')                                          # LaneCommand CLEARANCE 값 (0 대기 · 1 통과)

코스 (2026-09-30): T자 교차로 하나, 지점 1(오른쪽 위)·2(왼쪽 아래)·3(아래 가운데). 출발·목적 지점으로 교차로 방향이
정해진다(routes). 방향마다 기본 동작(directions), 경로마다 따로 잰 동작(maneuvers)을 쓸 수 있다.
동작 단계: {straight: m} · {turn: deg} (odom 고정 동작) · {seek: left|right|straight} (새 빨간 선을 찾아 그 앞까지, 2026-09-30 기본).

통행권 규칙 (사용자 결정 2026-09-30)
    * 교차로는 하나, 통행권도 하나. 빨간 테이프 앞 정지(JUNCTION_STOP)를 **먼저 보고한** 로봇이 먼저 받는다.
      tie_window 안에 들어온 보고는 같은 순간으로 보고 domain_id 가 작은 로봇이 먼저다.
    * 쥔 로봇이 교차로 동작을 마치고 **차선 주행으로 돌아간 순간**(CRUISE, 교차로 뒤 단계) 반납 → 다음 로봇.
    * 시나리오 로봇이 1대면 통행권을 쓰지 않는다 — 로봇은 1 s 정지 뒤 허가 없이 출발한다(skip_clearance).

도착 (arrival.mode)
    * marker (기본): 도착 지점 앞 벽의 ArUco 마커(지점 → id, arrival.markers)까지 arrive_distance(15 cm) 이하에서 정지.
      같은 목적지로 먼저 도착한 로봇이 있으면 뒤 로봇은 교차로 뒤 장애물 정지를 도착으로 친다(arrive_on_obstacle).
    * stop_line: 흰 정지선 개수 — 로봇에 stop_lines 가 있으면 그 값, 없으면 시나리오 stop_lines_by_order[통행권 받은 순서].
"""

import math
import os
import re
import tempfile
from dataclasses import dataclass, field

import yaml

# LaneStatus.drive_state (pinky_lane_msgs/msg/LaneStatus.msg)
DRIVE_IDLE, DRIVE_CRUISE, DRIVE_ARRIVED, DRIVE_ESTOP, DRIVE_LINK_LOST = 0, 1, 7, 8, 9
DRIVE_OBSTACLE_WAIT = 5
DRIVE_JUNCTION_STOP, DRIVE_JUNCTION_PASS = 12, 13
STAGE_AFTER = 'vision:after_junction'
DIRECTIONS = ('straight', 'left', 'right')
SEEK_VALUES = {'left': 1.0, 'right': -1.0, 'straight': 0.0}
DIRECTION_LABELS = {'straight': '직진', 'left': '좌회전', 'right': '우회전'}
ARRIVAL_MODES = ('marker', 'stop_line')
SCENARIO_NAME_RE = re.compile(r'^[0-9A-Za-z가-힣_\-]{1,40}$')
CUSTOM_NAME = 'custom'


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
    stop_lines: int = None          # None → 통행권 순서로 정한다 (stop_line 모드)
    direction: str = ''             # straight | left | right
    goal_marker_id: int = -1        # 도착 지점 벽 ArUco id (marker 모드). -1 → 흰 정지선


@dataclass
class Scenario:
    name: str
    label: str
    robots: dict                    # name -> RobotPlan
    stop_lines_by_order: list = field(default_factory=list)
    source: str = 'preset'          # preset (vision_mission.yaml) | user (웹 저장) | custom (웹 직접 설정, 저장 안 함)
    raw: dict = field(default_factory=dict)     # 저장용 원본 (정규화)


@dataclass
class VisionConfig:
    linear_speed: float
    angular_speed: float
    tie_window: float
    robots: dict                    # name -> domain_id
    maneuvers: dict                 # name -> [(kind, value)]
    scenarios: dict                 # name -> Scenario
    path: str = ''
    directions: dict = field(default_factory=dict)      # straight|left|right -> [(kind, value)]
    routes: dict = field(default_factory=dict)          # (start, goal) -> {'direction', 'maneuver'}
    arrival_mode: str = 'stop_line'
    arrive_distance: float = 0.15
    markers: dict = field(default_factory=dict)         # 지점 -> ArUco id
    user_path: str = ''

    @property
    def points(self):
        pts = set(self.markers)
        for a, b in self.routes:
            pts.update((a, b))
        return sorted(pts)


def _steps(name, raw):
    if not isinstance(raw, list) or not raw:
        raise VisionConfigError(f'maneuver {name}: 단계 목록이 비었다')
    out = []
    for i, st in enumerate(raw):
        if not isinstance(st, dict) or len(st) != 1:
            raise VisionConfigError(f'maneuver {name}[{i}]: {{straight: m}}·{{turn: deg}}·{{seek: left|right|straight}} '
                                    f'하나여야 한다 — {st!r}')
        (kind, value), = st.items()
        kind = str(kind).strip().lower()
        if kind not in ('straight', 'turn', 'seek'):
            raise VisionConfigError(f'maneuver {name}[{i}]: 알 수 없는 동작 {kind!r}')
        if kind == 'seek':
            # 새 빨간 선 찾기 (로봇 maneuver.RedLineSeeker) — JunctionPlan 값 +1 좌 / −1 우 / 0 직진
            v = str(value).strip().lower()
            if v not in SEEK_VALUES:
                raise VisionConfigError(f'maneuver {name}[{i}]: seek 은 {"/".join(SEEK_VALUES)} — {value!r}')
            out.append((kind, SEEK_VALUES[v]))
            continue
        value = float(value)
        if not math.isfinite(value) or (kind == 'straight' and not 0 < abs(value) <= 2.0) \
                or (kind == 'turn' and not 0 < abs(value) <= 360.0):
            raise VisionConfigError(f'maneuver {name}[{i}]: 값 범위 밖 {value}')
        out.append((kind, value))
    return out


def _direction(value, where):
    d = str(value or '').strip().lower()
    if d and d not in DIRECTIONS:
        raise VisionConfigError(f'{where}: 방향은 {"/".join(DIRECTIONS)} 중 하나 — {value!r}')
    return d


def _route_key(raw_key):
    parts = re.split(r'\s*(?:>|->|→)\s*', str(raw_key).strip())
    if len(parts) != 2 or not all(parts):
        raise VisionConfigError(f'routes: 키는 "출발>목적" — {raw_key!r}')
    return parts[0], parts[1]


def _robot_plan(cfg, sname, rname, r):
    """시나리오 한 로봇 → RobotPlan (방향·동작·도착 마커 풀이)."""
    r = r or {}
    where = f'scenario {sname}/{rname}'
    if rname not in cfg.robots:
        raise VisionConfigError(f'scenario {sname}: 모르는 로봇 {rname}')
    start, goal = str(r.get('start', '')).strip(), str(r.get('goal', '')).strip()
    route = cfg.routes.get((start, goal)) if start and goal else None
    if cfg.routes and start and goal:
        pts = cfg.points
        for label, v in (('start', start), ('goal', goal)):
            if v not in pts:
                raise VisionConfigError(f'{where}: 모르는 지점 {label}={v!r} (있는 것: {", ".join(pts)})')
        if start == goal:
            raise VisionConfigError(f'{where}: 출발과 목적이 같다 ({start})')
    direction = _direction(r.get('direction'), where) or (route or {}).get('direction', '')
    m = str(r.get('maneuver', '') or '').strip()
    if m:
        if m not in cfg.maneuvers:
            raise VisionConfigError(f'{where}: 모르는 maneuver {m!r}')
        steps = list(cfg.maneuvers[m])
    elif route and route.get('maneuver') and direction == route.get('direction'):
        m = route['maneuver']
        steps = list(cfg.maneuvers[m])
    elif direction and direction in cfg.directions:
        m = f'{direction}_{start}_to_{goal}' if start and goal else direction
        steps = list(cfg.directions[direction])
    elif direction:
        raise VisionConfigError(f'{where}: 방향 {direction} 의 기본 동작(directions.{direction})이 없다')
    else:
        raise VisionConfigError(f'{where}: maneuver 도 방향도 없다 — routes 에 {start}>{goal} 을 넣거나 direction 을 준다')
    delay = float(r.get('depart_delay', 0.0) or 0.0)
    if not math.isfinite(delay) or delay < 0 or delay > 600:
        raise VisionConfigError(f'{where}: depart_delay 범위 밖 (0~600 s)')
    sl = r.get('stop_lines')
    marker_id = -1
    if cfg.arrival_mode == 'marker':
        if goal not in cfg.markers:
            raise VisionConfigError(f'{where}: 목적지 {goal!r} 의 ArUco id 가 없다 (arrival.markers)')
        marker_id = int(cfg.markers[goal])
    return RobotPlan(rname, cfg.robots[rname], start, goal, m, steps, delay,
                     None if sl in (None, '') else int(sl), direction, marker_id)


def scenario_from_dict(cfg, name, raw, source='preset'):
    """시나리오 원본(dict) → Scenario. 웹 직접 설정·저장도 같은 검증을 탄다."""
    raw = raw or {}
    if not isinstance(raw, dict):
        raise VisionConfigError(f'scenario {name}: 형식이 dict 가 아니다')
    plans, norm_robots = {}, {}
    for rname, r in (raw.get('robots') or {}).items():
        rname = str(rname)
        if isinstance(r, dict) and r.get('enabled') is False:
            continue
        plans[rname] = _robot_plan(cfg, name, rname, r)
        norm = {'start': plans[rname].start, 'goal': plans[rname].goal}
        if isinstance(r, dict):
            for k in ('direction', 'maneuver', 'depart_delay', 'stop_lines'):
                if r.get(k) not in (None, ''):
                    norm[k] = r[k]
        norm_robots[rname] = norm
    if not plans:
        raise VisionConfigError(f'scenario {name}: 로봇이 없다')
    order = [int(x) for x in (raw.get('stop_lines_by_order') or [])]
    if cfg.arrival_mode == 'stop_line':
        for rname, plan in plans.items():
            if plan.stop_lines is None and len(order) < len(plans):
                raise VisionConfigError(f'scenario {name}/{rname}: stop_lines 도 stop_lines_by_order(로봇 수만큼) 도 없다')
    norm = {'label': str(raw.get('label') or name), 'robots': norm_robots}
    if order:
        norm['stop_lines_by_order'] = order
    return Scenario(str(name), str(raw.get('label') or name), plans, order, source, norm)


def vision_config_from_dict(data, path=''):
    data = data or {}
    speeds = data.get('speeds') or {}
    robots = {str(n): int((v or {}).get('domain_id', 0)) for n, v in (data.get('robots') or {}).items()}
    if not robots:
        raise VisionConfigError('robots 가 비었다')
    if len(set(robots.values())) != len(robots):
        raise VisionConfigError(f'domain_id 가 겹친다: {robots}')
    maneuvers = {str(n): _steps(n, v) for n, v in (data.get('maneuvers') or {}).items()}
    directions = {}
    for d, v in (data.get('directions') or {}).items():
        d = _direction(d, 'directions')
        directions[d] = _steps(f'directions.{d}', v)
    arrival = data.get('arrival') or {}
    mode = str(arrival.get('mode', 'stop_line')).strip().lower()
    if mode not in ARRIVAL_MODES:
        raise VisionConfigError(f'arrival.mode 는 {"/".join(ARRIVAL_MODES)} — {mode!r}')
    arrive_distance = float(arrival.get('arrive_distance', 0.15))
    if not 0.03 <= arrive_distance <= 1.0:
        raise VisionConfigError(f'arrival.arrive_distance 범위 밖 (0.03~1.0 m): {arrive_distance}')
    markers = {str(k): int(v) for k, v in (arrival.get('markers') or {}).items()}
    if len(set(markers.values())) != len(markers):
        raise VisionConfigError(f'arrival.markers: ArUco id 가 겹친다 {markers}')
    cfg = VisionConfig(float(speeds.get('linear', 0.08)), float(speeds.get('angular', 0.5)),
                       float(data.get('tie_window', 0.5)), robots, maneuvers, {}, path,
                       directions, {}, mode, arrive_distance, markers)
    for key, v in (data.get('routes') or {}).items():
        start, goal = _route_key(key)
        v = v if isinstance(v, dict) else {'direction': v}
        d = _direction(v.get('direction'), f'routes.{key}')
        if not d:
            raise VisionConfigError(f'routes.{key}: direction 이 없다')
        m = str(v.get('maneuver', '') or '').strip()
        if m and m not in maneuvers:
            raise VisionConfigError(f'routes.{key}: 모르는 maneuver {m!r}')
        if not m and d not in directions:
            raise VisionConfigError(f'routes.{key}: maneuver 도 directions.{d} 도 없다')
        cfg.routes[(start, goal)] = {'direction': d, 'maneuver': m}
    for sname, sraw in (data.get('scenarios') or {}).items():
        cfg.scenarios[str(sname)] = scenario_from_dict(cfg, str(sname), sraw, 'preset')
    if not cfg.scenarios:
        raise VisionConfigError('scenarios 가 비었다')
    return cfg


def load_vision_config(path):
    with open(path, encoding='utf-8') as fh:
        return vision_config_from_dict(yaml.safe_load(fh), path)


# ------------------------------------------------------------------ 웹에서 저장한 시나리오

def default_user_path(cfg_path):
    """프리셋 파일 옆 vision_mission_user.yaml."""
    base = os.path.dirname(os.path.abspath(cfg_path)) if cfg_path else os.getcwd()
    return os.path.join(base, 'vision_mission_user.yaml')


def load_user_scenarios(cfg, path):
    """저장된 시나리오를 cfg.scenarios 에 더한다 (source='user'). 프리셋과 이름이 겹치거나 틀린 것은 건너뛰고 이유를 돌려준다."""
    cfg.user_path = path
    errors = []
    if not path or not os.path.exists(path):
        return errors
    try:
        with open(path, encoding='utf-8') as fh:
            data = yaml.safe_load(fh) or {}
    except (OSError, yaml.YAMLError) as exc:
        return [f'{path}: 읽기 실패 {exc}']
    for sname, sraw in (data.get('scenarios') or {}).items():
        sname = str(sname)
        if sname in cfg.scenarios and cfg.scenarios[sname].source == 'preset':
            errors.append(f'{sname}: 프리셋과 이름이 같다 — 건너뜀')
            continue
        try:
            cfg.scenarios[sname] = scenario_from_dict(cfg, sname, sraw, 'user')
        except (VisionConfigError, TypeError, ValueError) as exc:
            errors.append(f'{sname}: {exc}')
    return errors


def _write_user_file(cfg):
    path = cfg.user_path or default_user_path(cfg.path)
    cfg.user_path = path
    data = {'scenarios': {n: s.raw for n, s in cfg.scenarios.items() if s.source == 'user'}}
    text = ('# 웹 관제에서 저장한 비전 미션 시나리오 — 게이트웨이가 쓴다. 프리셋은 vision_mission.yaml.\n'
            + yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    d = os.path.dirname(path) or '.'
    os.makedirs(d, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.vision_user_', suffix='.yaml', dir=d)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as fh:
            fh.write(text)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return path


def check_scenario_name(cfg, name):
    name = str(name or '').strip()
    if not SCENARIO_NAME_RE.match(name):
        raise VisionConfigError('시나리오 이름은 글자·숫자·_·- 로 1~40자')
    if name == CUSTOM_NAME:
        raise VisionConfigError(f'{CUSTOM_NAME!r} 는 저장하지 않은 직접 설정용 이름이다')
    if name in cfg.scenarios and cfg.scenarios[name].source == 'preset':
        raise VisionConfigError(f'{name!r} 는 프리셋 이름이다 — 다른 이름으로')
    return name


def save_user_scenario(cfg, name, raw):
    """검증 뒤 저장 파일에 쓰고 cfg 에 더한다. 같은 이름의 저장 시나리오는 덮어쓴다."""
    name = check_scenario_name(cfg, name)
    scn = scenario_from_dict(cfg, name, raw, 'user')
    prev = cfg.scenarios.get(name)
    cfg.scenarios[name] = scn
    try:
        _write_user_file(cfg)
    except OSError:
        if prev is None:
            del cfg.scenarios[name]
        else:
            cfg.scenarios[name] = prev
        raise
    return scn


def delete_user_scenario(cfg, name):
    scn = cfg.scenarios.get(str(name))
    if scn is None or scn.source != 'user':
        raise VisionConfigError(f'저장한 시나리오가 아니다: {name!r}')
    del cfg.scenarios[scn.name]
    try:
        _write_user_file(cfg)
    except OSError:
        cfg.scenarios[scn.name] = scn
        raise


def course_dict(cfg):
    """웹 직접 설정 폼용 — 로봇·지점·경로별 방향·도착 방식."""
    return {
        'robots': list(cfg.robots), 'points': cfg.points,
        'routes': [{'start': a, 'goal': b, 'direction': v['direction'],
                    'direction_label': DIRECTION_LABELS.get(v['direction'], v['direction'])}
                   for (a, b), v in sorted(cfg.routes.items())],
        'directions': [d for d in DIRECTIONS if d in cfg.directions],
        'direction_labels': dict(DIRECTION_LABELS),
        'arrival': {'mode': cfg.arrival_mode, 'arrive_distance': cfg.arrive_distance, 'markers': dict(cfg.markers)},
    }


def scenario_summary(s):
    return {'name': s.name, 'label': s.label, 'source': s.source, 'raw': s.raw,
            'robots': {n: {'start': p.start, 'goal': p.goal, 'maneuver': p.maneuver, 'direction': p.direction,
                           'direction_label': DIRECTION_LABELS.get(p.direction, p.direction),
                           'depart_delay': p.depart_delay, 'goal_marker_id': p.goal_marker_id}
                       for n, p in s.robots.items()}}


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
    def __init__(self, cfg, scenario, t0, first_seq=1):
        """scenario: 시나리오 이름(cfg.scenarios) 또는 Scenario (웹 직접 설정 — scenario_from_dict)."""
        if isinstance(scenario, Scenario):
            self.scenario = scenario
        elif scenario in cfg.scenarios:
            self.scenario = cfg.scenarios[scenario]
        else:
            raise VisionConfigError(f'모르는 시나리오 {scenario!r}')
        self.cfg = cfg
        self.t0 = float(t0)
        self.arbiter = JunctionArbiter(cfg.tie_window)
        self.skip_clearance = len(self.scenario.robots) == 1      # 1대: 통행권 없이 1 s 정지 뒤 출발
        self.seq = {}
        self.depart_at = {}
        self.stop_lines = {}
        self.stage = {}
        self.drive_state = {}
        self.reason = {}
        self.arrived = {}
        self.arrive_on_obstacle = {}
        for i, (name, plan) in enumerate(sorted(self.scenario.robots.items())):
            self.seq[name] = int(first_seq) + i
            self.depart_at[name] = self.t0 + plan.depart_delay
            self.stop_lines[name] = plan.stop_lines if plan.stop_lines is not None else 0
            self.stage[name] = 'vision:approach'
            self.drive_state[name] = DRIVE_IDLE
            self.reason[name] = ''
            self.arrived[name] = False
            self.arrive_on_obstacle[name] = False

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
            'goal_marker_id': int(plan.goal_marker_id),
            'arrive_distance': float(self.cfg.arrive_distance) if plan.goal_marker_id >= 0 else 0.0,
            'skip_clearance': bool(self.skip_clearance),
            'arrive_on_obstacle': bool(self.arrive_on_obstacle[name]),
        }

    def on_status(self, name, drive_state, route_seq, edge_id, now, reason=''):
        """LaneStatus 하나. 이 시나리오의 번호가 아니면 무시한다."""
        if name not in self.seq or int(route_seq) != self.seq[name]:
            return
        ds = int(drive_state)
        self.drive_state[name] = ds
        self.reason[name] = str(reason or '')
        if edge_id:
            self.stage[name] = str(edge_id)
        if self.skip_clearance:
            pass                                               # 통행권 없음 — 로봇이 1 s 정지 뒤 스스로 간다
        elif ds == DRIVE_JUNCTION_STOP:
            self.arbiter.request(name, now, self.scenario.robots[name].domain_id)
        elif self.arbiter.holder == name and (ds == DRIVE_ARRIVED or (ds == DRIVE_CRUISE and self.stage[name] == STAGE_AFTER)):
            self.arbiter.release(name)                         # 차선 주행으로 돌아갔다 → 다음 로봇
        if ds == DRIVE_ARRIVED:
            self.arrived[name] = True

    def tick(self, now):
        """통행권 배정·같은 목적지 도착 표시. 계획(JunctionPlan)이 바뀐 로봇 이름 목록."""
        changed = []
        granted = None if self.skip_clearance else self.arbiter.tick(now)
        order = self.scenario.stop_lines_by_order
        if granted is not None and self.scenario.robots[granted].stop_lines is None and order:
            k = len(self.arbiter.grant_order) - 1
            self.stop_lines[granted] = int(order[k]) if k < len(order) else int(order[-1])
            changed.append(granted)
        # 같은 목적지에 먼저 도착한 로봇이 있으면 뒤 로봇은 그 뒤에서 장애물로 선 것이 도착이다
        for name, plan in self.scenario.robots.items():
            if self.arrived[name] or self.arrive_on_obstacle[name]:
                continue
            if any(self.arrived[o] and p.goal == plan.goal for o, p in self.scenario.robots.items() if o != name):
                self.arrive_on_obstacle[name] = True
                changed.append(name)
        return changed

    def clearance(self, name):
        if self.skip_clearance:
            return 1 if name in self.seq else 0
        return 1 if (self.arbiter.holder == name or name in self.arbiter.passed) else 0

    def status(self, now):
        return {
            'scenario': self.scenario.name, 'label': self.scenario.label, 'source': self.scenario.source,
            'holder': self.arbiter.holder, 'queue': self.arbiter.queue(),
            'grant_order': list(self.arbiter.grant_order), 'done': self.done,
            'skip_clearance': self.skip_clearance, 'arrival_mode': self.cfg.arrival_mode,
            'arrive_distance': self.cfg.arrive_distance,
            'robots': {n: {
                'start': p.start, 'goal': p.goal, 'maneuver': p.maneuver, 'direction': p.direction,
                'direction_label': DIRECTION_LABELS.get(p.direction, p.direction),
                'goal_marker_id': p.goal_marker_id,
                'steps': [[k, v] for k, v in p.steps], 'seq': self.seq[n],
                'stop_line_count': self.stop_lines[n], 'stage': self.stage[n],
                'drive_state': self.drive_state[n], 'reason': self.reason[n], 'arrived': self.arrived[n],
                'arrive_on_obstacle': self.arrive_on_obstacle[n],
                'clearance': self.clearance(n),
                'depart_in': round(max(0.0, self.depart_at[n] - now), 1),
            } for n, p in self.scenario.robots.items()},
        }
