# -*- coding: utf-8 -*-
"""좌표 프로파일 — 도로망·미션·화면 지도·로봇 지도 이름을 한 묶음으로 고른다 (R-7 웹 전환, 2026-09-26).

ROS 에 의존하지 않는다. 코디네이터가 이것으로 미션 파일을 고르고, 게이트웨이가 웹에 보여 주고 바꾼다.

## 왜 (PLAN_20260925_MAP4_FRAME_UNIFICATION)
옛 5노드 가상 경기장(좌하단 원점)과 팀11 map4(가운데 원점)는 좌표가 다르다. 한쪽만 바뀌면 도로망·지도·마커가
섞인다. 코드 커밋 날짜를 맞추는 대신, 두 묶음을 다 싣고 **현장에서 웹으로 한 번에** 고른다.

## 정직성
- 못 읽는 프로파일은 고를 수 없고, 왜 못 읽는지(`problems`)를 그대로 보인다.
- 고른 값은 상태 파일에 남긴다(재시작해도 같은 좌표). 상태 파일이 가리키는 프로파일이 깨졌으면 기본으로
  떨어지되 그 사실(코디네이터 `profile_note`, 웹의 "알림")을 보인다 — 조용히 다른 좌표로 뜨지 않는다.
- 도로망 엣지가 지도의 점유·알 수 없음·지도 밖 칸을 지나면 **경고**한다(막지 않는다, `_check_occupancy`).
"""
import math
import os
import tempfile

import yaml

from .road_graph import RoadGraph

PKG_DIR = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(PKG_DIR, 'config', 'profiles', 'profiles.yaml')
STATE_ENV = 'PINKY_RELAY_STATE_DIR'          # 시험·도커가 상태 위치를 바꾼다
BOUNDS_EPS = 1e-6
OCC_SAMPLE_STEP = 0.005                      # m — 엣지를 5 mm 간격으로 지도 칸에 대 본다(map4 칸 0.05 m 의 1/10)
OCC_KINDS = (('occupied', '점유'), ('unknown', '알 수 없음'), ('off_map', '지도 밖'))


def state_path():
    base = os.environ.get(STATE_ENV) or os.path.join(
        os.environ.get('XDG_STATE_HOME') or os.path.expanduser('~/.local/state'), 'pinky_relay')
    return os.path.join(base, 'fleet_profile')


def read_active():
    try:
        with open(state_path(), encoding='utf-8') as f:
            name = f.read().strip()
        return name or None
    except OSError:
        return None


def write_active(name):
    """원자적으로 쓴다 — 쓰는 도중 꺼져도 반쯤 쓴 이름이 남지 않게."""
    path = state_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix='.fleet_profile.')
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        f.write(name + '\n')
    os.replace(tmp, path)


def _pgm_size(path):
    """P5/P2 PGM 헤더의 (가로, 세로)."""
    with open(path, 'rb') as f:
        head = f.read(512)
    toks = []
    for line in head.split(b'\n'):
        line = line.split(b'#')[0]
        toks += line.split()
        if len(toks) >= 3:
            break
    if not toks or toks[0] not in (b'P5', b'P2') or len(toks) < 3:
        raise ValueError('PGM 헤더를 못 읽었다: %s' % path)
    return int(toks[1]), int(toks[2])


def load_map_meta(map_yaml):
    """Nav2 지도 yaml → {'yaml','image','resolution','origin':[x,y],'width','height','bounds':{x_min..y_max}}."""
    with open(map_yaml, encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    res = float(doc['resolution'])
    ox, oy = float(doc['origin'][0]), float(doc['origin'][1])
    img = doc['image']
    img_path = img if os.path.isabs(img) else os.path.join(os.path.dirname(map_yaml), img)
    w, h = _pgm_size(img_path)
    return {'yaml': map_yaml, 'image': img_path, 'resolution': res, 'origin': [ox, oy],
            'width': w, 'height': h,
            'bounds': {'x_min': ox, 'x_max': ox + w * res, 'y_min': oy, 'y_max': oy + h * res}}


def _read_pgm(path):
    """P5/P2 PGM → (가로, 세로, maxval, 화소 목록). 화소 0 번 행이 그림의 **맨 위**다."""
    with open(path, 'rb') as f:
        data = f.read()
    toks, i = [], 0
    while len(toks) < 4:                              # 마법 수 · 가로 · 세로 · maxval (사이에 # 주석이 올 수 있다)
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] not in (b'\n', b'\r'):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace() and data[j:j + 1] != b'#':
            j += 1
        if j == i:
            raise ValueError('PGM 헤더가 끊겼다')
        toks.append(data[i:j])
        i = j
    magic, w, h, maxval = toks[0], int(toks[1]), int(toks[2]), int(toks[3])
    n = w * h
    if magic == b'P5':
        width = 1 if maxval < 256 else 2
        raw = data[i + 1:i + 1 + n * width]             # 헤더 뒤 공백 한 칸 다음부터 화소
        if len(raw) < n * width:
            raise ValueError('PGM 화소가 모자란다 (%d / %d)' % (len(raw) // width, n))
        px = list(raw) if width == 1 else [raw[2 * k] << 8 | raw[2 * k + 1] for k in range(n)]
    elif magic == b'P2':
        px = [int(t) for t in data[i:].split()[:n]]
        if len(px) < n:
            raise ValueError('PGM 화소가 모자란다 (%d / %d)' % (len(px), n))
    else:
        raise ValueError('PGM 이 아니다: %r' % magic)
    return w, h, maxval, px


def edge_occupancy(map_yaml, graph):
    """도로망 엣지가 Nav2 지도의 점유·알 수 없음·지도 밖 칸을 지나는 대략의 길이 {엣지: {종류: m}} (지나는 엣지만).

    Nav2 map_server 의 trinary 규칙 그대로: p = (255 − 화소)/255 (negate 1 이면 화소/255, maxval 이 255 가 아니면 그 값으로),
    p > occupied_thresh 점유 · p < free_thresh 빈칸 · 그 사이 알 수 없음. 그림의 0 번 행이 y 최대다.
    negate·occupied_thresh·free_thresh 는 Nav2 처럼 필수 — 없으면 예외(호출하는 쪽이 경고로 바꾼다).
    """
    with open(map_yaml, encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    mode = str(doc.get('mode', 'trinary'))
    if mode != 'trinary':
        raise ValueError('trinary 지도만 본다 (mode %s)' % mode)
    res = float(doc['resolution'])
    ox, oy = float(doc['origin'][0]), float(doc['origin'][1])
    negate = int(doc['negate'])
    occ_t, free_t = float(doc['occupied_thresh']), float(doc['free_thresh'])
    img = doc['image']
    w, h, maxval, px = _read_pgm(img if os.path.isabs(img) else os.path.join(os.path.dirname(map_yaml), img))

    def kind(x, y):
        c, r = math.floor((x - ox) / res), math.floor((y - oy) / res)
        if not (0 <= c < w and 0 <= r < h):
            return 'off_map'
        shade = px[(h - 1 - r) * w + c] / float(maxval)
        p = shade if negate else 1.0 - shade
        if p > occ_t:
            return 'occupied'
        return None if p < free_t else 'unknown'

    out = {}
    for eid, e in graph.edges.items():
        acc = {}
        for (x0, y0), (x1, y1) in zip(e.waypoints, e.waypoints[1:]):
            seg = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(math.ceil(seg / OCC_SAMPLE_STEP)))
            for i in range(n + 1):
                k = kind(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n)
                if k:
                    acc[k] = acc.get(k, 0.0) + seg / n
        if acc:
            out[eid] = acc
    return out


class Profile:
    def __init__(self, name, spec, base_dir):
        self.name = name
        self.label = str(spec.get('label', name))
        self.frame = str(spec.get('frame', name))
        self.robot_map_name = spec.get('robot_map_name')
        self.mission_path = os.path.normpath(os.path.join(base_dir, spec['mission']))
        m = spec.get('map')
        self.map_yaml = None if not m else os.path.normpath(os.path.join(base_dir, m))
        self.problems = []
        self.warnings = []          # 고를 수는 있지만 알아야 하는 것(예: 배정 때 거절될 로봇, 벽 칸을 지나는 엣지)
        self.mission = {}
        self.graph = None
        self.graph_path = None
        self.map_meta = None
        self._load()

    def _load(self):
        try:
            with open(self.mission_path, encoding='utf-8') as f:
                self.mission = yaml.safe_load(f) or {}
        except Exception as exc:                           # noqa: BLE001
            self.problems.append('미션 파일을 못 읽었다: %s (%s)' % (self.mission_path, exc))
            return
        g = self.mission.get('graph', 'road_graph.yaml')
        self.graph_path = g if os.path.isabs(g) else os.path.join(os.path.dirname(self.mission_path), g)
        try:
            self.graph = RoadGraph.load(self.graph_path)
        except Exception as exc:                           # noqa: BLE001
            self.problems.append('도로망을 못 읽었다: %s (%s)' % (self.graph_path, exc))
            return
        for r in self.mission.get('robots', []) or []:
            for key in ('start', 'goal'):
                node = r.get(key)
                if node and node not in self.graph.nodes:
                    self.problems.append('%s 의 %s %r 가 도로망에 없다' % (r.get('name'), key, node))
        if not self.problems:
            self._check_assignments()
        if self.map_yaml:
            try:
                self.map_meta = load_map_meta(self.map_yaml)
            except Exception as exc:                       # noqa: BLE001
                self.problems.append('지도를 못 읽었다: %s (%s)' % (self.map_yaml, exc))
                return
            b = self.map_meta['bounds']
            out = []
            for nid, n in self.graph.nodes.items():
                if not (b['x_min'] - BOUNDS_EPS <= n.x <= b['x_max'] + BOUNDS_EPS
                        and b['y_min'] - BOUNDS_EPS <= n.y <= b['y_max'] + BOUNDS_EPS):
                    out.append(nid)
            for eid, e in self.graph.edges.items():
                for (x, y) in e.waypoints:
                    if not (b['x_min'] - BOUNDS_EPS <= x <= b['x_max'] + BOUNDS_EPS
                            and b['y_min'] - BOUNDS_EPS <= y <= b['y_max'] + BOUNDS_EPS):
                        out.append(eid)
                        break
            if out:
                self.problems.append('지도 밖 좌표: %s' % ', '.join(sorted(set(out))))
            if not self.problems:
                self._check_occupancy()

    def _check_occupancy(self):
        """도로망 엣지가 지도의 점유·알 수 없음·지도 밖 칸을 지나면 **알린다**(막지 않는다 — map4 원본을 그대로 쓴다).

        중계 항목(REVIEW_20260926 map4 리허설 후속): 지도 안 좌표 검사(노드·waypoint 가 지도 사각형 안)는 칸의 **값**을 보지
        않는다. map4 는 JW_JS·TR_MC·TR_TC 가 벽 칸을 지난다 — Nav2 비용지도에서 치명 칸이라 그 구간에서 로봇이 도로
        중심선을 벗어나 돌아가거나(예약은 중심선 기준) 목표 waypoint 가 거절될 수 있다. 어느 미션 로봇의 경로인지 같이 적는다.
        """
        try:
            crossing = edge_occupancy(self.map_yaml, self.graph)
        except Exception as exc:                           # noqa: BLE001
            self.warnings.append('지도 점유를 검사하지 못했다: %s (%s)' % (self.map_yaml, exc))
            return
        users = {}
        for r in self.mission.get('robots', []) or []:
            if not (r.get('start') and r.get('goal')):
                continue
            try:
                route = self.graph.shortest_route(r['start'], r['goal'], step=0.10)
            except Exception:                              # noqa: BLE001 — 경로 못 만듦은 _check_assignments 가 알린다
                continue
            for eid in route.edge_ids:
                # 적대 검토 RES-2: 이름이 없거나(None) 숫자(name: 1)여도 join 이 터지지 않게 글자로 — 여기서 예외가 나면
                # load_profiles 가 통째로 터져 코디네이터가 뜨지 못한다(깨진 프로파일은 problems 로만 알린다는 약속).
                users.setdefault(eid, []).append(str(r.get('name')))
        for eid in sorted(crossing):
            kinds = [(label, crossing[eid][k]) for k, label in OCC_KINDS if k in crossing[eid]]
            who = ('%s 경로' % ', '.join(users[eid])) if eid in users else '이 미션 경로에는 없다'
            self.warnings.append('지도 칸: 엣지 %s 가 %s 칸 위를 지난다(%s) — %s' % (
                eid, '·'.join(label for label, _ in kinds), ' · '.join('%s 약 %.2f m' % kl for kl in kinds), who))

    def _check_assignments(self):
        """코디네이터의 배정 거절(L3)을 미리 본다 — 도착한 로봇은 목표 노드를 계속 잡으므로, 같은 목표이거나 한쪽 목표가
        다른 쪽 경로 위면 **뒤에 배정되는 로봇은 경로를 못 받는다**. 막지는 않고(기존 미션을 깨지 않게) 알린다."""
        paths = []
        for r in self.mission.get('robots', []) or []:
            if not (r.get('start') and r.get('goal')):
                continue
            try:
                paths.append((r.get('name'), self.graph.shortest_route(r['start'], r['goal'], step=0.10).node_ids))
            except Exception as exc:                       # noqa: BLE001
                self.warnings.append('%s 의 경로를 못 만든다: %s' % (r.get('name'), exc))
        for i, (a, pa) in enumerate(paths):
            for b, pb in paths[i + 1:]:
                if pa[-1] == pb[-1]:
                    self.warnings.append('%s·%s 같은 목표 %s — %s 는 배정되지 않는다(L3)' % (a, b, pa[-1], b))
                elif pa[-1] in pb[1:-1] or pb[-1] in pa[1:-1]:
                    self.warnings.append('%s·%s 한쪽 목표가 다른 쪽 경로 위 — %s 는 배정되지 않는다(L3)' % (a, b, b))

    @property
    def valid(self):
        return not self.problems

    def robot_names(self):
        return [r.get('name') for r in self.mission.get('robots', []) or []]

    def to_dict(self):
        return {'name': self.name, 'label': self.label, 'frame': self.frame,
                'robot_map_name': self.robot_map_name,
                'mission': os.path.relpath(self.mission_path, PKG_DIR),
                'map': None if not self.map_meta else {k: self.map_meta[k] for k in
                                                       ('resolution', 'origin', 'width', 'height', 'bounds')},
                'robots': [{'name': r.get('name'), 'start': r.get('start'), 'goal': r.get('goal')}
                           for r in self.mission.get('robots', []) or []],
                'valid': self.valid, 'problems': list(self.problems), 'warnings': list(self.warnings)}


def load_profiles(manifest=MANIFEST):
    """(기본 이름, {이름: Profile}). 목록 파일이 없으면 (None, {}) — 예전 방식(lane_mission.yaml 하나)으로 돈다."""
    if not os.path.isfile(manifest):
        return None, {}
    with open(manifest, encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    base = os.path.dirname(manifest)
    profs = {name: Profile(name, spec, base) for name, spec in (doc.get('profiles') or {}).items()}
    return doc.get('default'), profs


def resolve_active(default, profiles):
    """(고를 이름, 기본으로 떨어진 이유 또는 None)."""
    want = read_active()
    if want and want in profiles and profiles[want].valid:
        return want, None
    if want and want not in profiles:
        return default, '상태 파일의 프로파일 %r 이 목록에 없다 — 기본 %r 로' % (want, default)
    if want and not profiles[want].valid:
        return default, '상태 파일의 프로파일 %r 이 깨졌다(%s) — 기본 %r 로' % (
            want, '; '.join(profiles[want].problems), default)
    return default, None


def robot_map_check(profile, state):
    """로봇이 보고한 지도(RobotState.map_*)가 이 프로파일의 지도와 같은가: 'MATCH' · 'NAME_MATCH' · 'MISMATCH' · 'UNKNOWN'.

    이름만 보지 않고 원점·크기까지 본다 — 같은 이름의 다른 파일일 수 있다.
    """
    if state is None or not getattr(state, 'map_known', False):
        return 'UNKNOWN', '로봇이 지도를 아직 보고하지 않았다'
    name_ok = (not profile.robot_map_name) or state.map_name == profile.robot_map_name
    if profile.map_meta is None:
        # 이름만 맞다 — 규격을 모르니 MATCH(초록)라 하지 않는다
        return ('NAME_MATCH', '이름만 일치(이 프로파일은 지도 규격을 싣지 않아 원점·크기를 대조하지 못한다)') if name_ok else \
               ('MISMATCH', '로봇 지도 %r ≠ %r' % (state.map_name, profile.robot_map_name))
    m = profile.map_meta
    geo_ok = (abs(state.map_resolution - m['resolution']) < 1e-4
              and int(state.map_width) == m['width'] and int(state.map_height) == m['height']
              and abs(state.map_origin_x - m['origin'][0]) < 1e-3 and abs(state.map_origin_y - m['origin'][1]) < 1e-3)
    if name_ok and geo_ok:
        return 'MATCH', '이름·원점·크기 일치'
    return 'MISMATCH', '로봇 지도 %r 원점 (%.3f, %.3f) %d×%d — 프로파일 %r 원점 (%.3f, %.3f) %d×%d' % (
        state.map_name, state.map_origin_x, state.map_origin_y, state.map_width, state.map_height,
        profile.robot_map_name, m['origin'][0], m['origin'][1], m['width'], m['height'])
