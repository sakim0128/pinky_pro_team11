"""교차로 고정 동작 실행기 — odom 으로 '직진 d m', '제자리 회전 θ°' 단계를 차례로 수행한다. ROS 에 의존하지 않는다.

    ex = ManeuverExecutor([('straight', 0.18), ('turn', -90.0), ('straight', 0.10)], ManeuverParams())
    v, w, done, reason = ex.command(now, odom_pose)      # 매 틱. odom_pose = (x, y, yaw) 또는 None

진행은 **odom 으로 잰 누적량** 이다 — 도중에 멈춰도(장애물·STOP) 다시 부르면 남은 양만 이어서 한다.
단계 시작 자세는 그 단계를 처음 부를 때의 odom 이다. odom 이 없으면 (0, 0) 을 내고 기다린다.
"""

import math
from dataclasses import dataclass


@dataclass
class ManeuverParams:
    linear_speed: float = 0.08        # m/s
    angular_speed: float = 0.5        # rad/s
    min_linear: float = 0.03          # 끝에서 감속해도 이 아래로는 안 내린다 (바퀴 데드밴드)
    min_angular: float = 0.15
    slow_distance: float = 0.05       # 남은 거리가 이보다 작으면 비례 감속
    slow_angle_deg: float = 20.0
    dist_tolerance: float = 0.005
    yaw_tolerance_deg: float = 1.5
    heading_kp: float = 1.5           # 직진 중 방향 유지 (rad/s per rad)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def parse_steps(kinds, values):
    """메시지 배열 → [(kind, value)]. 알 수 없는 kind 는 ValueError."""
    if len(kinds) != len(values):
        raise ValueError(f'step_kind {len(kinds)} 개 ≠ step_value {len(values)} 개')
    out = []
    for k, v in zip(kinds, values):
        k = str(k).strip().lower()
        if k not in ('straight', 'turn'):
            raise ValueError(f'알 수 없는 동작 {k!r} (straight | turn)')
        out.append((k, float(v)))
    return out


class ManeuverExecutor:
    def __init__(self, steps, params=None):
        self.steps = list(steps)
        self.p = params or ManeuverParams()
        self.index = 0
        self._start = None            # (x, y, yaw) — 현재 단계 시작 odom
        self._turned = 0.0            # 현재 회전 단계 누적 (rad, 부호 있음)
        self._last_yaw = None
        self.done = not self.steps

    @property
    def last_turn_sign(self):
        """마지막 회전 단계의 방향 (+1 좌, −1 우, 0 회전 없음) — 차선 탐색 회전 방향에 쓴다."""
        for kind, value in reversed(self.steps):
            if kind == 'turn' and abs(value) > 1e-6:
                return 1.0 if value > 0 else -1.0
        return 0.0

    def describe(self):
        kind, value = self.steps[min(self.index, len(self.steps) - 1)] if self.steps else ('', 0.0)
        unit = 'm' if kind == 'straight' else '°'
        return f'교차로 동작 {min(self.index + 1, len(self.steps))}/{len(self.steps)} {kind} {value:+.2f}{unit}'

    def _advance(self):
        self.index += 1
        self._start = None
        self._turned = 0.0
        self._last_yaw = None
        if self.index >= len(self.steps):
            self.done = True

    def command(self, now, pose):
        """(v, ω, done, reason)."""
        if self.done:
            return 0.0, 0.0, True, '교차로 동작 완료'
        if pose is None:
            return 0.0, 0.0, False, '교차로 동작 — odom 없음, 대기'
        x, y, yaw = pose
        p = self.p
        while not self.done:
            kind, value = self.steps[self.index]
            if self._start is None:
                self._start = (x, y, yaw)
                self._last_yaw = yaw
            if kind == 'straight':
                target = abs(value)
                moved = math.hypot(x - self._start[0], y - self._start[1])
                remain = target - moved
                if remain <= p.dist_tolerance:
                    self._advance()
                    continue
                speed = p.linear_speed
                if remain < p.slow_distance:
                    speed = max(p.min_linear, speed * remain / p.slow_distance)
                v = math.copysign(speed, value if value != 0 else 1.0)
                w = p.heading_kp * _wrap(self._start[2] - yaw)
                w = max(-p.angular_speed, min(p.angular_speed, w))
                return v, w, False, self.describe()
            # turn
            self._turned += _wrap(yaw - self._last_yaw)
            self._last_yaw = yaw
            target = math.radians(abs(value))
            remain = target - abs(self._turned) if self._turned * value >= 0 else target + abs(self._turned)
            if remain <= math.radians(p.yaw_tolerance_deg):
                self._advance()
                continue
            speed = p.angular_speed
            slow = math.radians(p.slow_angle_deg)
            if remain < slow:
                speed = max(p.min_angular, speed * remain / slow)
            return 0.0, math.copysign(speed, value), False, self.describe()
        return 0.0, 0.0, True, '교차로 동작 완료'
