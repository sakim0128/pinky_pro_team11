"""비전 미션 항공뷰 관제 (2026-10-01) — 항공뷰 좌표를 코스 중심선에 겹쳐 교차로 통행권 거리와 차선 이탈 보정을 만든다.
ROS 에 의존하지 않는다. relay_station/fleet/vision_coordinator.py 가 로봇마다 하나 돌린다.

    tr = OverheadTrack(course.route('2', '1'))
    tr.feed(x, y, yaw, t)                       # /pinkyN/overhead_pose (태블릿 → 중계, frame map)
    ev = tr.evaluate(now, drive_state)          # 10 Hz
    ev['dist_to_entry']                         # 교차로 입구 빨간 선 정지 지점까지 남은 거리 (m) — 통행권
    ev['correct'], ev['lateral'], ev['heading'] # /pinkyN/lane_correction

사용자 결정 (2026-10-01)
    * 교차로 구간 = 입구 빨간 선 ~ 나가는 빨간 선. 통행권은 그 입구 정지 지점까지 **남은 거리가 짧은 로봇** 먼저.
    * 이탈 = 항공뷰 위치(머리 마커 중심)가 코스 중심선에서 옆으로 떨어진 거리. 3 cm 이상이면 보정, 8 cm 이상이면 경고.
      교차로 구간 안에서는 보정하지 않는다(seek 동작이 맡는다).
    * 항공뷰가 stale_after(0.5 s) 넘게 끊기면 거리·보정 모두 없음 — 통행권은 먼저 정지한 로봇 순서로 돌아가고, 로봇은 차선 주행만.
"""

import math
from dataclasses import dataclass

from .road_graph import project_to_polyline

DRIVE_CRUISE = 1


@dataclass
class OverheadParams:
    stale_after: float = 0.5        # 항공뷰가 이만큼(s) 끊기면 없는 것으로 본다
    correct_from: float = 0.03      # 이탈이 이 이상이면 보정 시작 (m)
    correct_until: float = 0.015    # 보정 중이면 이 아래로 돌아와야 멈춘다 (떨림 방지)
    warn_from: float = 0.08         # 이 이상이면 차선 밖 경고 (m)
    max_offtrack: float = 0.30      # 이보다 멀면 코스에 겹칠 수 없다 — 보정 안 함, 경고
    zone_before: float = 0.05       # 교차로 구간 = 입구 정지 지점 − 이 값 ~
    zone_after: float = 0.10        #             나가는 빨간 선 + 이 값
    window_back: float = 0.30       # 지난 위치 기준 투영 창 (뒤, 앞) — 코스가 가까이 지나는 곳에서 튀지 않게
    window_ahead: float = 0.60


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def locate(route, x, y, s_lo=0.0, s_hi=None):
    """코스(CourseRoute) 위 [s_lo, s_hi] 구간에 투영 → (s, 옆 이탈(왼쪽 +), 거리)."""
    cum, pts = route.cum, route.points
    s_hi = route.length if s_hi is None else s_hi
    i0, i1 = 0, len(pts) - 1
    while i0 < len(cum) - 2 and cum[i0 + 1] < s_lo:
        i0 += 1
    while i1 > i0 + 1 and cum[i1 - 1] > s_hi:
        i1 -= 1
    s, lateral, _, _, dist = project_to_polyline(x, y, pts[i0:i1 + 1])
    return cum[i0] + s, lateral, dist


class OverheadTrack:
    """로봇 하나의 항공뷰 좌표 → 코스 위 위치."""

    def __init__(self, route, params=None):
        self.route = route
        self.p = params or OverheadParams()
        self.pose = None                # (x, y, yaw) map
        self.t = None
        self.s = None
        self.lateral = None
        self.offtrack = None            # 코스선까지 거리 (m)
        self.correcting = False
        r = route
        self.s_stop = r.s_entry - r.params.red_stop_back          # 입구 빨간 선 앞 정지 지점 (구동축)
        self.zone = (self.s_stop - self.p.zone_before, r.s_exit + self.p.zone_after)

    def feed(self, x, y, yaw, t):
        self.pose = (float(x), float(y), float(yaw))
        self.t = float(t)
        if self.s is None:
            s, lat, dist = locate(self.route, x, y)
        else:
            s, lat, dist = locate(self.route, x, y, self.s - self.p.window_back, self.s + self.p.window_ahead)
            if dist > self.p.max_offtrack:                        # 창 밖으로 옮겨졌다(손으로 옮김 등) — 전체에서 다시
                s, lat, dist = locate(self.route, x, y)
        self.s, self.lateral, self.offtrack = s, lat, dist

    def fresh(self, now):
        return self.t is not None and float(now) - self.t <= self.p.stale_after

    def in_junction(self):
        return self.s is not None and self.zone[0] <= self.s <= self.zone[1]

    def evaluate(self, now, drive_state=DRIVE_CRUISE):
        """10 Hz. 통행권 거리 · 보정 · 경고."""
        p = self.p
        out = {'fresh': self.fresh(now), 'age': None if self.t is None else round(float(now) - self.t, 2),
               'dist_to_entry': None, 'correct': False, 'lateral': 0.0, 'heading': 0.0,
               'warn': False, 'in_junction': False, 's': self.s}
        if not out['fresh'] or self.s is None:
            self.correcting = False
            return out
        on_course = self.offtrack <= p.max_offtrack
        out['in_junction'] = self.in_junction()
        if on_course and self.s < self.zone[1]:
            out['dist_to_entry'] = max(0.0, self.s_stop - self.s)
        tangent = self.route.point_at(self.s)[2]
        lat, head = self.lateral, _wrap(self.pose[2] - tangent)
        out['lateral'], out['heading'] = lat, head
        out['warn'] = (not on_course) or (abs(lat) >= p.warn_from and not out['in_junction'])
        limit = p.correct_until if self.correcting else p.correct_from
        self.correcting = (on_course and not out['in_junction'] and int(drive_state) == DRIVE_CRUISE
                           and abs(lat) >= limit)
        out['correct'] = self.correcting
        return out

    def to_dict(self, now):
        x, y, yaw = self.pose if self.pose is not None else (None, None, None)
        return {'x': x, 'y': y, 'yaw': yaw, 'age': None if self.t is None else round(float(now) - self.t, 2),
                'fresh': self.fresh(now), 's': None if self.s is None else round(self.s, 3),
                'lateral': None if self.lateral is None else round(self.lateral, 4),
                'offtrack': None if self.offtrack is None else round(self.offtrack, 4),
                'in_junction': self.in_junction(), 'correcting': self.correcting}
