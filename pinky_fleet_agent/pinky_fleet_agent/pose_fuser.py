"""외부 절대 위치(fix) + odom → map→odom 보정 (D7, AMCL 대체). ROS 에 의존하지 않는다.

    f = PoseFuser()
    f.add_odom(t, x, y, yaw)            # odom→base, 20 Hz 이상
    f.on_fix(t_fix, x, y, yaw, now)     # 이미지 stamp 시각의 map→base
    f.map_odom                          # (x, y, yaw) 또는 None
    f.map_pose(x_odom, y_odom, yaw_odom) # 현재 map→base

fix 는 "그때(t_fix) 그 위치" 라서 처리 지연이 오차가 되지 않는다 — odom 버퍼에서 t_fix 의
odom 을 보간해 map→odom = T_map_base(fix) · inv(T_odom_base(t_fix)) 로 잡는다.
게이트: 기존 추정과 gate_dist / gate_yaw 이상 다르면 바로 믿지 않고, 다음 fix 가 그 후보와
일치할 때(confirm 회) 채택한다. 첫 fix 는 게이트 없이 채택.
"""

import bisect
import math
from collections import deque


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    """SE(2) a ⊕ b."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return ax + c * bx - s * by, ay + s * bx + c * by, wrap(at + bt)


def invert(a):
    ax, ay, at = a
    c, s = math.cos(at), math.sin(at)
    return -c * ax - s * ay, s * ax - c * ay, wrap(-at)


def pose_diff(a, b):
    """(거리, |Δyaw|)."""
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(wrap(a[2] - b[2]))


class PoseFuser:
    def __init__(self, gate_dist=0.30, gate_yaw=math.radians(45), confirm=2, fix_timeout=15.0,
                 buffer_seconds=10.0, max_extrapolate=0.15, blend=1.0):
        self.gate_dist = float(gate_dist)
        self.gate_yaw = float(gate_yaw)
        self.confirm = int(confirm)
        self.fix_timeout = float(fix_timeout)
        self.buffer_seconds = float(buffer_seconds)
        self.max_extrapolate = float(max_extrapolate)
        self.blend = float(blend)
        self._t = deque()
        self._p = deque()
        self.map_odom = None
        self.last_fix_time = None           # 채택된 fix 의 수신 시각 (now)
        self.last_fix = None
        self._candidate = None
        self._candidate_count = 0
        self.rejected = 0
        self.accepted = 0

    # ------------------------------------------------ odom 버퍼

    def add_odom(self, t, x, y, yaw):
        t = float(t)
        if self._t and t <= self._t[-1]:
            return
        self._t.append(t)
        self._p.append((float(x), float(y), float(yaw)))
        while self._t and t - self._t[0] > self.buffer_seconds:
            self._t.popleft()
            self._p.popleft()

    def odom_at(self, t):
        """t 의 odom pose (선형 보간). 버퍼 밖이면 None (약간의 외삽은 최근 값으로)."""
        if not self._t:
            return None
        t = float(t)
        if t >= self._t[-1]:
            return self._p[-1] if t - self._t[-1] <= self.max_extrapolate else None
        if t <= self._t[0]:
            return self._p[0] if self._t[0] - t <= self.max_extrapolate else None
        i = bisect.bisect_right(self._t, t)
        t0, t1 = self._t[i - 1], self._t[i]
        a, b = self._p[i - 1], self._p[i]
        u = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u,
                wrap(a[2] + wrap(b[2] - a[2]) * u))

    # ------------------------------------------------ fix

    def on_fix(self, t_fix, x, y, yaw, now):
        """(accepted, reason)."""
        odom = self.odom_at(t_fix)
        if odom is None:
            self.rejected += 1
            return False, 'odom 버퍼에 fix 시각이 없음'
        fix = (float(x), float(y), float(yaw))
        candidate = compose(fix, invert(odom))          # map→odom

        if self.map_odom is not None:
            predicted = compose(self.map_odom, odom)
            d, dyaw = pose_diff(predicted, fix)
            if d > self.gate_dist or dyaw > self.gate_yaw:
                # 큰 점프: 후보와 일치하는 fix 가 confirm 회 이어져야 채택
                if self._candidate is not None:
                    cd, cyaw = pose_diff(compose(self._candidate, odom), fix)
                    if cd <= self.gate_dist and cyaw <= self.gate_yaw:
                        self._candidate_count += 1
                    else:
                        self._candidate, self._candidate_count = candidate, 1
                else:
                    self._candidate, self._candidate_count = candidate, 1
                if self._candidate_count < self.confirm:
                    self.rejected += 1
                    return False, f'점프 {d:.2f} m/{math.degrees(dyaw):.0f}° 보류 ({self._candidate_count}/{self.confirm})'
                self.map_odom = candidate
                self._candidate, self._candidate_count = None, 0
                self._accept(fix, now)
                return True, f'점프 확정 {d:.2f} m'
            if self.blend < 1.0:
                bx = self.map_odom[0] + (candidate[0] - self.map_odom[0]) * self.blend
                by = self.map_odom[1] + (candidate[1] - self.map_odom[1]) * self.blend
                bt = wrap(self.map_odom[2] + wrap(candidate[2] - self.map_odom[2]) * self.blend)
                candidate = (bx, by, bt)
        self.map_odom = candidate
        self._candidate, self._candidate_count = None, 0
        self._accept(fix, now)
        return True, 'ok'

    def _accept(self, fix, now):
        self.last_fix = fix
        self.last_fix_time = float(now)
        self.accepted += 1

    # ------------------------------------------------ 조회

    def map_pose(self, x_odom, y_odom, yaw_odom):
        if self.map_odom is None:
            return None
        return compose(self.map_odom, (float(x_odom), float(y_odom), float(yaw_odom)))

    def alive(self, now):
        return (self.last_fix_time is not None
                and (float(now) - self.last_fix_time) <= self.fix_timeout)

    def fix_age(self, now):
        return float('inf') if self.last_fix_time is None else float(now) - self.last_fix_time
