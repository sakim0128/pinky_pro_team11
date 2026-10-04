"""카메라 사각지대 보정 — 본 차선 중앙을 odom 좌표에 기억했다가, 핑키가 그 자리에 왔을 때 따라간다. ROS 에 의존하지 않는다.

카메라는 핑키 앞면에서 약 10 cm 앞부터 보인다(사용자 실측: 화면 맨 아래 = 10 cm, 화면 50 % 행 = 43 cm). 차선 중앙은 0.72·H 행에서
읽으므로 핑키 몸보다 약 20 cm 앞의 상황이다. 그 오차를 바로 조향에 넣으면 몸이 아직 거기 없는데 미리 꺾는다(커브 안쪽을 자름).

    view = GroundView(ViewParams())
    pt = view.to_robot(target_x, target_y, W, H, half_lane_px)       # (x 전방, y 좌) m, 원점 = 구동 바퀴 축
    mem = LaneMemory(MemoryParams())
    mem.add(pt, odom_pose_at_capture)                                 # 그 사진을 찍은 순간의 odom 자세
    kappa = mem.curvature(odom_pose_now)                              # pure pursuit 곡률 (없으면 None)

화면 → 바닥 거리 (캘리브레이션 없이 두 실측점): 평평한 바닥의 핀홀이면 d(r) = K / (r − r_h) (r = 행/H).
옆 거리: 차선 반폭(lane_half_m, 도로 15 cm → 7.5 cm) 을 샘플 행의 half_lane_px 에 맞춰 m/px 를 구하고, 다른 행은 깊이에 비례시킨다.
"""

import math
from collections import deque
from dataclasses import dataclass


@dataclass
class ViewParams:
    bottom_m: float = 0.10          # 화면 맨 아래 행이 보는 바닥 — 카메라(핑키 앞면)에서의 거리 (실측)
    mid_m: float = 0.43             # mid_row_frac 행이 보는 바닥까지 거리 (실측)
    mid_row_frac: float = 0.50
    axle_to_camera_m: float = 0.033  # 구동 바퀴 축(제자리 회전 중심) → 카메라 거리 (pinky_pro URDF). BEV 바닥 좌표에도 이 값을 더한다
    sample_row_frac: float = 0.72   # 관제 lane_target 의 샘플 행 (half_lane_px 를 잰 행)
    lane_half_m: float = 0.075      # 도로 폭 15 cm 의 절반


@dataclass
class MemoryParams:
    lookahead: float = 0.12         # 구동축에서 이 거리 이상 떨어진 첫 기억 점을 따라간다 (m)
    average: int = 3                # 그 점부터 몇 개를 평균 (검출 잡음 완화)
    max_points: int = 80
    max_range: float = 0.80         # 이보다 먼 점은 버린다 (odom 누적 오차가 커지기 전에)
    keep_behind: float = -0.02      # 로봇 좌표 x 가 이보다 작으면(지나간 점) 버린다
    min_spacing: float = 0.005      # 직전 점과 이보다 가까우면 추가하지 않는다 (서 있을 때)


class GroundView:
    def __init__(self, params=None):
        self.p = params or ViewParams()
        p = self.p
        # d(r) = K / (r − r_h):  d(1) = bottom,  d(mid_r) = mid
        denom = p.mid_m - p.bottom_m
        if denom <= 1e-6 or not 0.0 < p.mid_row_frac < 1.0:
            raise ValueError('view: mid_m > bottom_m, 0 < mid_row_frac < 1 이어야 한다')
        self.r_h = (p.mid_m * p.mid_row_frac - p.bottom_m) / denom
        self.k = p.bottom_m * (1.0 - self.r_h)
        if not self.r_h < p.mid_row_frac:
            raise ValueError('view: 지평선이 mid 행 아래 — 실측값을 확인')

    def camera_distance(self, row_frac):
        """행(비율) → 카메라(앞면)에서 바닥까지 전방 거리 (m). 지평선 위면 None."""
        gap = float(row_frac) - self.r_h
        if gap <= 0.02:
            return None
        return self.k / gap

    def axle_distance(self, row_frac):
        d = self.camera_distance(row_frac)
        return None if d is None else d + self.p.axle_to_camera_m

    def to_robot(self, target_x, target_y, width, height, half_lane_px):
        """화면 목표점(px) → 로봇 좌표 (x 전방, y 좌) m. 원점 = 구동 바퀴 축. 계산 못 하면 None."""
        p = self.p
        if width <= 0 or height <= 0 or half_lane_px is None or half_lane_px <= 1.0:
            return None
        r = float(target_y) / float(height)
        x = self.axle_distance(r)
        rs = p.sample_row_frac - self.r_h
        if x is None or rs <= 0.02:
            return None
        m_per_px = (p.lane_half_m / float(half_lane_px)) * rs / (r - self.r_h)
        y = -(float(target_x) - width / 2.0) * m_per_px
        return x, y


def _to_world(pose, pt):
    ox, oy, oyaw = pose
    c, s = math.cos(oyaw), math.sin(oyaw)
    return ox + c * pt[0] - s * pt[1], oy + s * pt[0] + c * pt[1]


def _to_local(pose, pt):
    ox, oy, oyaw = pose
    dx, dy = pt[0] - ox, pt[1] - oy
    c, s = math.cos(oyaw), math.sin(oyaw)
    return c * dx + s * dy, -s * dx + c * dy


class LaneMemory:
    def __init__(self, params=None):
        self.p = params or MemoryParams()
        self.points = deque(maxlen=self.p.max_points)     # odom 좌표 (오래된 = 가까운 순)

    def clear(self):
        self.points.clear()

    def __len__(self):
        return len(self.points)

    def add(self, pt_robot, pose_at_capture):
        """로봇 좌표의 차선 중앙점을 그 사진을 찍은 순간의 odom 자세로 odom 좌표에 저장."""
        w = _to_world(pose_at_capture, pt_robot)
        if self.points and math.hypot(w[0] - self.points[-1][0], w[1] - self.points[-1][1]) < self.p.min_spacing:
            return
        self.points.append(w)

    def prune(self, pose_now):
        p = self.p
        keep = [q for q in self.points if _to_local(pose_now, q)[0] >= p.keep_behind
                and math.hypot(q[0] - pose_now[0], q[1] - pose_now[1]) <= p.max_range]
        if len(keep) != len(self.points):
            self.points = deque(keep, maxlen=p.max_points)

    def target(self, pose_now):
        """따라갈 점 (로봇 좌표). 구동축에서 lookahead 이상 떨어진 첫 기억 점부터 average 개 평균. 없으면 None."""
        self.prune(pose_now)
        p = self.p
        local = [_to_local(pose_now, q) for q in self.points]
        for i, (x, y) in enumerate(local):
            if x > 0.0 and math.hypot(x, y) >= p.lookahead:
                sel = local[i:i + max(1, p.average)]
                return (sum(a for a, _ in sel) / len(sel), sum(b for _, b in sel) / len(sel))
        return None

    def curvature(self, pose_now):
        """pure pursuit 곡률 κ = 2·y / (x² + y²) (좌회전 +). 따라갈 점이 없으면 None."""
        t = self.target(pose_now)
        if t is None:
            return None
        x, y = t
        return 2.0 * y / (x * x + y * y)


class OdomBuffer:
    """(stamp, x, y, yaw) 링버퍼 — 사진 stamp 의 odom 자세를 보간한다 (둘 다 로봇 시계)."""

    def __init__(self, horizon=2.0, max_gap=0.3):
        self.horizon = horizon
        self.max_gap = max_gap
        self.buf = deque()

    def add(self, stamp, x, y, yaw):
        stamp = float(stamp)
        if self.buf and stamp < self.buf[-1][0]:
            self.buf.clear()                             # 시계가 뒤로 갔다 (재시작) — 버린다
        self.buf.append((stamp, float(x), float(y), float(yaw)))
        while self.buf and self.buf[0][0] < stamp - self.horizon:
            self.buf.popleft()

    def at(self, stamp):
        """stamp 의 자세 (선형 보간). 버퍼 밖이면 max_gap 안에서 가장 가까운 것, 아니면 None."""
        if not self.buf:
            return None
        t = float(stamp)
        first, last = self.buf[0], self.buf[-1]
        if t <= first[0]:
            return first[1:] if first[0] - t <= self.max_gap else None
        if t >= last[0]:
            return last[1:] if t - last[0] <= self.max_gap else None
        prev = first
        for cur in self.buf:
            if cur[0] >= t:
                span = cur[0] - prev[0]
                a = 0.0 if span <= 0 else (t - prev[0]) / span
                dyaw = math.atan2(math.sin(cur[3] - prev[3]), math.cos(cur[3] - prev[3]))
                return (prev[1] + a * (cur[1] - prev[1]), prev[2] + a * (cur[2] - prev[2]), prev[3] + a * dyaw)
            prev = cur
        return last[1:]
