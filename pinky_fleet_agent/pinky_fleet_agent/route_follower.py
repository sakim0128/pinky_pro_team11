"""경로(폴리라인) 추종 — 진행도 추적, lookahead 점, pure-pursuit 조향.

ROS 에 의존하지 않는다. 좌표는 전부 map frame (m, rad).

    f = RouteFollower(waypoints, goal_idx)
    f.update(x, y)                  # 진행도 갱신 (단조 증가)
    omega = f.steer(x, y, yaw, v)   # 각속도
    f.distance_to_idx(clear_until)  # 정지 지점까지 남은 호길이
"""

import bisect
import math

DEFAULT_LOOKAHEAD = 0.25
BACKTRACK_TOL = 0.05        # 이만큼 뒤로 가는 투영은 노이즈로 보고 무시한다


def _cumulative(points):
    out = [0.0]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        out.append(out[-1] + math.hypot(x1 - x0, y1 - y0))
    return out


def _project_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 <= 1e-12:
        return 0.0, ax, ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
    return t, ax + dx * t, ay + dy * t


class RouteFollower:
    def __init__(self, waypoints, goal_idx=None, lookahead=DEFAULT_LOOKAHEAD,
                 search_window=2.0):
        if len(waypoints) < 2:
            raise ValueError('경로에 waypoint 가 2개 이상 필요합니다')
        self.points = [(float(x), float(y)) for x, y in waypoints]
        self.cum = _cumulative(self.points)
        self.length = self.cum[-1]
        self.goal_idx = len(self.points) - 1 if goal_idx is None else int(goal_idx)
        self.lookahead = float(lookahead)
        self.search_window = float(search_window)   # 진행도 앞뒤로 이만큼만 투영 후보로 본다
        self.progress_s = 0.0
        self.progress_idx = 0
        self.lateral = 0.0          # 중심선 기준 좌측 양수 (m)
        self.off_route = 0.0        # 중심선까지 거리 (m)

    # ------------------------------------------------ 진행도

    def update(self, x, y):
        """pose 를 경로에 투영해 진행도를 갱신한다. 뒤로 튀는 투영은 무시."""
        lo = bisect.bisect_left(self.cum, self.progress_s - self.search_window)
        hi = bisect.bisect_right(self.cum, self.progress_s + self.search_window)
        best = None
        for i in range(max(0, lo - 1), min(len(self.points) - 1, hi)):
            (ax, ay), (bx, by) = self.points[i], self.points[i + 1]
            t, qx, qy = _project_segment(x, y, ax, ay, bx, by)
            d = math.hypot(x - qx, y - qy)
            if best is None or d < best[0]:
                seg = self.cum[i + 1] - self.cum[i]
                cross = (bx - ax) * (y - ay) - (by - ay) * (x - ax)
                lateral = cross / seg if seg > 1e-9 else 0.0
                best = (d, self.cum[i] + seg * t, lateral)
        d, s, lateral = best
        if s >= self.progress_s - BACKTRACK_TOL:
            self.progress_s = max(self.progress_s, s)
            self.progress_idx = max(0, bisect.bisect_right(self.cum, self.progress_s + 1e-9) - 1)
        self.lateral = lateral
        self.off_route = d
        return self.progress_idx

    def distance_to_idx(self, idx):
        """진행도에서 waypoint idx 까지 남은 호길이 (뒤면 0)."""
        idx = max(0, min(len(self.points) - 1, int(idx)))
        return max(0.0, self.cum[idx] - self.progress_s)

    def remaining(self):
        return self.distance_to_idx(self.goal_idx)

    def point_at(self, s):
        s = max(0.0, min(self.length, s))
        i = max(0, bisect.bisect_right(self.cum, s) - 1)
        if i >= len(self.points) - 1:
            return self.points[-1]
        seg = self.cum[i + 1] - self.cum[i]
        t = 0.0 if seg <= 1e-9 else (s - self.cum[i]) / seg
        (x0, y0), (x1, y1) = self.points[i], self.points[i + 1]
        return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t

    def lookahead_point(self, limit_idx=None):
        """진행도 + lookahead 지점. limit_idx 를 넘지 않는다 (정지 지점 앞에서 조향 안정)."""
        s = self.progress_s + self.lookahead
        if limit_idx is not None:
            s = min(s, self.cum[max(0, min(len(self.points) - 1, int(limit_idx)))])
        return self.point_at(s)

    def heading_at(self, s):
        s = max(0.0, min(self.length, s))
        i = max(0, min(len(self.points) - 2, bisect.bisect_right(self.cum, s) - 1))
        (x0, y0), (x1, y1) = self.points[i], self.points[i + 1]
        return math.atan2(y1 - y0, x1 - x0)

    # ------------------------------------------------ 조향

    def steer(self, x, y, yaw, v, limit_idx=None, omega_max=1.5):
        """pure pursuit. lookahead 점이 뒤에 있으면 제자리 회전 방향만 준다."""
        lx, ly = self.lookahead_point(limit_idx)
        dx, dy = lx - x, ly - y
        c, s = math.cos(-yaw), math.sin(-yaw)
        rx, ry = dx * c - dy * s, dx * s + dy * c      # 로봇 좌표
        dist = math.hypot(rx, ry)
        if dist < 1e-6:
            return 0.0, False
        alpha = math.atan2(ry, rx)
        if rx < 1e-6 or abs(alpha) >= math.pi / 2 - 1e-6:
            # 점이 옆·뒤에 있다 — 전진하면 안 된다. 그 방향으로 회전만.
            return math.copysign(omega_max, alpha), True
        omega = 2.0 * max(v, 0.05) * math.sin(alpha) / max(dist, 1e-3)
        return max(-omega_max, min(omega_max, omega)), False
