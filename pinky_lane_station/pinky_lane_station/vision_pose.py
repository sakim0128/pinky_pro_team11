"""비전 미션 위치 표시 — 천장 카메라 없이 odom + 코스 모양 + 확실한 지점에서 다시 맞추기. ROS 에 의존하지 않는다.

관제 화면에 핑키를 그리는 데만 쓴다 (주행 제어에는 안 쓴다). relay_station/fleet/vision_coordinator.py 가 로봇마다 하나 돌린다.

    course = load_vision_course('config/vision_course.yaml')
    tr = PoseTracker(course.route('2', '1'))           # 시나리오 시작 — 출발 지점에 놓는다
    tr.feed_odom(x, y, yaw)                            # RobotState (frame_id 'odom') 10 Hz
    tr.feed_status(drive_state, edge_id, reason)       # LaneStatus
    tr.pose()                                          # {'x','y','yaw','px','py','mode','fix',...}

차선을 따라가는 동안(route): odom 이 앞으로 간 거리만큼 코스 중심선 위를 나아간다 — 핑키는 차선 가운데를 달리므로 옆 오차가 작고,
odom 방향 오차가 쌓이지 않는다. 교차로 동작 중(free): 입구에서 선 자세 + 그 뒤 odom 변화로 2D 로 그린다(제자리 회전이 보인다).
다시 맞추기(fix): 교차로 입구 빨간 선 정지 · 나갈 가지 빨간 선 정지 · 횡단보도 정지 · 벽 마커 도착 — 선·마커 앞 정해진 거리로 옮긴다.
"""

import math
import os

import yaml

from .road_graph import project_to_polyline, resample

DRIVE_IDLE, DRIVE_CRUISE, DRIVE_CROSSWALK_STOP, DRIVE_ARRIVED = 0, 1, 3, 7
DRIVE_LANE_SEARCH, DRIVE_JUNCTION_STOP, DRIVE_JUNCTION_PASS, DRIVE_RED_LINE_STOP = 11, 12, 13, 14
STAGE_AFTER = 'vision:after_junction'
STEP = 0.01


class CourseError(ValueError):
    pass


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class CourseRoute:
    """출발 → 목적 코스 중심선 (map, m). 끝은 목적지 벽 마커까지 늘인다."""

    def __init__(self, start, goal, points, s_entry, s_exit, crosswalks, s_goal_end, params):
        self.start, self.goal = str(start), str(goal)
        self.points = points
        self.cum = [0.0]
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            self.cum.append(self.cum[-1] + math.hypot(x1 - x0, y1 - y0))
        self.length = self.cum[-1]
        self.s_entry, self.s_exit = s_entry, s_exit
        self.crosswalks = crosswalks          # [(s_near, s_far)] 진행 방향 기준
        self.s_goal_end = s_goal_end          # 지점 끝 (마커 벽 앞 도로 끝)
        self.params = params

    def point_at(self, s):
        s = max(0.0, min(self.length, float(s)))
        i = 1
        while i < len(self.cum) - 1 and self.cum[i] < s:
            i += 1
        s0, s1 = self.cum[i - 1], self.cum[i]
        a = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
        (x0, y0), (x1, y1) = self.points[i - 1], self.points[i]
        return x0 + a * (x1 - x0), y0 + a * (y1 - y0), math.atan2(y1 - y0, x1 - x0)

    def project(self, x, y, s_min=0.0):
        pts = self.points
        i0 = 0
        while i0 < len(self.cum) - 2 and self.cum[i0 + 1] < s_min:
            i0 += 1
        s, _, _, _, _ = project_to_polyline(x, y, pts[i0:])
        return self.cum[i0] + s


class VisionCourse:
    def __init__(self, data, path=''):
        self.path = path
        data = data or {}
        img = data.get('image') or {}
        mp = data.get('map') or {}
        try:
            self.x0, self.y0 = float(img['x0']), float(img['y0'])
            self.sx, self.sy = float(img['px_per_cell_x']), float(img['px_per_cell_y'])
            self.res = float(mp.get('resolution', 0.01))
            self.origin = tuple(float(v) for v in mp.get('origin', [0.0, 0.0])[:2])
            self.height_cells = float(mp['height_cells'])
        except (KeyError, TypeError, ValueError) as exc:
            raise CourseError(f'image/map 변환값이 없다: {exc}') from exc
        self.image_file = str(img.get('file', ''))
        self.red_stop_back = float(data.get('red_stop_back', 0.21))
        self.crosswalk_stop_back = float(data.get('crosswalk_stop_back', 0.18))
        self.marker_stop_back = float(data.get('marker_stop_back', 0.19))
        self.red = {str(k): self.px_to_map(*v) for k, v in (data.get('red_lines') or {}).items()}
        self.points = {}
        for pid, p in (data.get('points') or {}).items():
            arm = [self.px_to_map(*q) for q in p.get('arm') or []]
            red = str(p.get('red', ''))
            if len(arm) < 2 or red not in self.red:
                raise CourseError(f'points.{pid}: arm 은 점 2개 이상, red 는 red_lines 중 하나')
            self.points[str(pid)] = {'arm': arm, 'red': red, 'marker': self.px_to_map(*p['marker'])
                                     if p.get('marker') else None}
        self.connectors = {}
        for key, pts in (data.get('connectors') or {}).items():
            a, b = [s.strip() for s in str(key).split('-')]
            pts = [self.px_to_map(*q) for q in pts]
            self.connectors[(a, b)] = pts
            self.connectors[(b, a)] = list(reversed(pts))
        self.crosswalks = [tuple(self.px_to_map(*q) for q in cw) for cw in data.get('crosswalks') or []]

    # ------------------------------------------------ 좌표

    def px_to_map(self, x, y):
        cx = (float(x) - self.x0) / self.sx
        cy = (float(y) - self.y0) / self.sy
        return self.origin[0] + cx * self.res, self.origin[1] + (self.height_cells - cy) * self.res

    def map_to_px(self, x, y):
        cx = (float(x) - self.origin[0]) / self.res
        cy = self.height_cells - (float(y) - self.origin[1]) / self.res
        return self.x0 + cx * self.sx, self.y0 + cy * self.sy

    # ------------------------------------------------ 경로

    def route(self, start, goal):
        start, goal = str(start), str(goal)
        if start not in self.points or goal not in self.points or start == goal:
            raise CourseError(f'코스에 없는 경로 {start}>{goal}')
        a, b = self.points[start], self.points[goal]
        conn = self.connectors.get((a['red'], b['red']))
        if conn is None:
            raise CourseError(f'교차로 연결선 {a["red"]}-{b["red"]} 이 없다')
        raw = list(a['arm']) + conn[1:] + list(reversed(b['arm']))[1:]
        if b['marker'] is not None:
            raw.append(b['marker'])
        pts = resample(raw, STEP)
        tmp = CourseRoute(start, goal, pts, 0, 0, [], 0, self)
        s_entry = tmp.project(*self.red[a['red']])
        s_exit = tmp.project(*self.red[b['red']], s_min=s_entry + 0.05)
        s_goal_end = tmp.project(*b['arm'][0], s_min=s_exit)
        cws = []
        for p, q in self.crosswalks:
            sp, sq = tmp.project(*p), tmp.project(*q)
            if min(math.hypot(*_sub(tmp.point_at(sp)[:2], p)), math.hypot(*_sub(tmp.point_at(sq)[:2], q))) < 0.05:
                cws.append((min(sp, sq), max(sp, sq)))
        return CourseRoute(start, goal, pts, s_entry, s_exit, cws, s_goal_end, self)

    def to_dict(self):
        """웹: 배경 그림 + 코스선 (png px)."""
        lines = []
        for pid, p in self.points.items():
            lines.append({'id': f'arm{pid}', 'px': [self.map_to_px(*q) for q in p['arm']]})
        seen = set()
        for (a, b), pts in self.connectors.items():
            if (b, a) in seen:
                continue
            seen.add((a, b))
            lines.append({'id': f'{a}-{b}', 'px': [self.map_to_px(*q) for q in pts]})
        return {'image': self.image_file, 'lines': lines,
                'points': {pid: self.map_to_px(*p['arm'][0]) for pid, p in self.points.items()},
                'red_lines': {k: self.map_to_px(*v) for k, v in self.red.items()}}


def _sub(a, b):
    return a[0] - b[0], a[1] - b[1]


def load_vision_course(path):
    with open(path, encoding='utf-8') as fh:
        return VisionCourse(yaml.safe_load(fh), path)


def default_course_path(vision_config_path):
    return os.path.join(os.path.dirname(os.path.abspath(vision_config_path)), 'vision_course.yaml')


class PoseTracker:
    """로봇 하나. 경로 위 추측항법(route) / 교차로 동작 중 2D odom(free) + 확실한 지점에서 다시 맞추기(fix)."""

    CROSSWALK_WINDOW = (-0.25, 0.50)     # 횡단보도 정지로 맞출 때 지금 추정 s 에서 허용하는 범위 (뒤, 앞)

    def __init__(self, route, t=0.0):
        self.route = route
        self.s = 0.0
        self.mode = 'route'
        self.fix = f'출발 지점 {route.start}'
        self.fix_s = 0.0
        self.travel_since_fix = 0.0
        self._odom = None
        self._anchor = None               # (map 자세, odom 자세) — free
        self._free_pose = None
        self._state = None
        self._stage = ''
        self.t = t

    # ------------------------------------------------ 입력

    def feed_odom(self, x, y, yaw, t=None):
        cur = (float(x), float(y), float(yaw))
        if t is not None:
            self.t = t
        prev, self._odom = self._odom, cur
        if prev is None:
            return
        dx, dy = cur[0] - prev[0], cur[1] - prev[1]
        if math.hypot(dx, dy) > 0.5:       # odom 이 튀었다(재시작) — 이번 걸음은 버린다
            self._anchor = None if self.mode != 'free' else (self.pose_tuple(), cur)
            return
        ds = dx * math.cos(prev[2]) + dy * math.sin(prev[2])
        self.travel_since_fix += abs(ds)
        if self.mode == 'route':
            self.s = max(0.0, min(self.route.length, self.s + ds))
        elif self._anchor is not None:
            (ax, ay, ayaw), (ox, oy, oyaw) = self._anchor
            # 입구 자세 ⊕ (입구 odom⁻¹ ⊕ 지금 odom)
            ddx, ddy = cur[0] - ox, cur[1] - oy
            c, s = math.cos(-oyaw), math.sin(-oyaw)
            lx, ly = c * ddx - s * ddy, s * ddx + c * ddy
            c2, s2 = math.cos(ayaw), math.sin(ayaw)
            self._free_pose = (ax + c2 * lx - s2 * ly, ay + s2 * lx + c2 * ly, _wrap(ayaw + cur[2] - oyaw))

    def feed_status(self, drive_state, edge_id='', reason='', t=None):
        st = int(drive_state)
        if t is not None:
            self.t = t
        prev, self._state = self._state, st
        stage_prev, self._stage = self._stage, str(edge_id or '')
        if st == prev:
            return
        r = self.route
        if st == DRIVE_JUNCTION_STOP and self._stage != STAGE_AFTER:
            self._snap(r.s_entry - r.params.red_stop_back, '교차로 입구 빨간 선')
        elif st == DRIVE_JUNCTION_PASS:
            if self.mode != 'free':
                self.mode = 'free'
                cur = self.pose_tuple()
                self._free_pose = cur
                self._anchor = (cur, self._odom) if self._odom is not None else None
        elif st == DRIVE_RED_LINE_STOP and (prev == DRIVE_JUNCTION_PASS or self._stage == STAGE_AFTER
                                            or stage_prev == 'vision:junction'):
            if self.s < r.s_exit or self.mode == 'free':
                self._snap(r.s_exit - r.params.red_stop_back, '교차로 나가는 빨간 선')
        elif st == DRIVE_CROSSWALK_STOP:
            ahead = [cw for cw in r.crosswalks
                     if self.CROSSWALK_WINDOW[0] <= cw[0] - r.params.crosswalk_stop_back - self.s
                     <= self.CROSSWALK_WINDOW[1]]
            if ahead and self.mode == 'route':
                self._snap(ahead[0][0] - r.params.crosswalk_stop_back, '횡단보도')
        elif st == DRIVE_ARRIVED and '마커' in str(reason):
            self._snap(r.length - r.params.marker_stop_back, f'도착 벽 마커 ({r.goal})')
        if self.mode == 'free' and st not in (DRIVE_JUNCTION_PASS, DRIVE_JUNCTION_STOP) and prev == DRIVE_JUNCTION_PASS:
            # 나가는 선에서 서지 않고 바로 차선 주행 — 지금 2D 자세를 경로 위로 옮긴다
            fx, fy, _ = self.pose_tuple()
            self.s = r.project(fx, fy, s_min=r.s_entry)
            self.mode = 'route'

    def _snap(self, s, label):
        self.s = max(0.0, min(self.route.length, s))
        self.mode = 'route'
        self._anchor = None
        self.fix, self.fix_s, self.travel_since_fix = label, self.s, 0.0

    # ------------------------------------------------ 출력

    def pose_tuple(self):
        if self.mode == 'free' and self._free_pose is not None:
            return self._free_pose
        return self.route.point_at(self.s)

    def pose(self):
        x, y, yaw = self.pose_tuple()
        px, py = self.route.params.map_to_px(x, y)
        return {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4), 'px': round(px, 1), 'py': round(py, 1),
                'mode': self.mode, 's': round(self.s, 3), 'route_length': round(self.route.length, 3),
                'fix': self.fix, 'travel_since_fix': round(self.travel_since_fix, 3),
                'start': self.route.start, 'goal': self.route.goal, 'source': 'odom+course'}
