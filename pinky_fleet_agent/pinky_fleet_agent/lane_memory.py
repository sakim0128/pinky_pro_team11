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
    jump_reject: float = 0.06       # 같은 전방 거리(± jump_window)의 기억 경로와 옆으로 이만큼 넘게 다르면 바로 넣지 않고 보류 (m)
    jump_window: float = 0.05       # 비교할 기억 점의 전방 거리 범위 (m). 그 범위에 기억이 없으면(처음·공백 뒤) 그냥 받는다
    jump_confirm: float = 0.04      # 보류한 점과 다음 점이 (이만큼 + 두 사진 사이 로봇 이동 거리) 안이면 같은 자리를 본 것 (m).
                                    # 목표점은 매번 '로봇 앞 25 cm' 라 로봇이 간 만큼 앞으로 간다 — 고정 4 cm 면 빠르거나 프레임이 늦을 때
                                    # 진짜 바뀐 장면도 확인을 못 한다 (10-04 로그: 정상 연속 점 간격의 11 % 가 4 cm 초과)
    jump_confirm_seconds: float = 1.0  # 보류한 점이 이보다 오래되면 버리고 새로 센다 (s)
    jump_confirm_count: int = 2     # 보류한 점 뒤로 이만큼 더 같은 자리를 보면(합 3 프레임) 장면이 바뀐 것 — 그 앞의 예전 점을 지우고 받는다.
                                    # 2026-10-04 pinky2 S자: 급커브 꼭짓점에서 BEV 가 2 프레임 연속 반대쪽(왼쪽 14 cm)을 내 차로를 벗어났다
    clear_on_stop: bool = True      # 정지(횡단보도·빨간 선·장애물 …) 중에는 기억을 비운다 — 출발은 서서 새로 본 점부터 (lane_driver)
    max_target_angle_deg: float = 45.0  # 따라갈 점은 로봇 정면 ± 이 각도 안만. 옆에 남은 옛 점(급커브·저속)을 목표로 삼아 제자리 급회전하지 않게


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
        self._pending = []                                # 튄 점들 [(odom 좌표 점, 그 사진의 odom 자세, 촬영 시각)] — 다음 점들이 확인하면 받는다

    def clear(self):
        self.points.clear()
        self._pending = []

    def __len__(self):
        return len(self.points)

    def add(self, pt_robot, pose_at_capture, stamp=None):
        """로봇 좌표의 차선 중앙점을 그 사진을 찍은 순간의 odom 자세로 odom 좌표에 저장. 넣었으면 True.

        잠깐 옆으로 튄 점(검출이 다른 차선 조각을 잡음)은 넣지 않는다: 같은 전방 거리의 기억 경로와
        옆으로 jump_reject 넘게 다르면 보류하고, 그 뒤 jump_confirm_count 프레임이 같은 자리를 보면 그때 받는다
        (장면이 정말 바뀜 — 그 앞쪽의 예전 점은 지운다). 2026-10-04 pinky2 횡단보도: 튄 점 2개가 남아 출발 직후 급우회전.
        '같은 자리' = 직전 보류 점과 jump_confirm + (두 사진 사이 odom 이동 거리) 안, jump_confirm_seconds 안.
        stamp: 촬영 시각 (s). 없으면 시간 만료를 보지 않는다.
        """
        p = self.p
        w = _to_world(pose_at_capture, pt_robot)
        if self.points and math.hypot(w[0] - self.points[-1][0], w[1] - self.points[-1][1]) < p.min_spacing:
            return False
        dev = self._lateral_deviation(pt_robot, pose_at_capture)
        if dev is not None and dev > p.jump_reject:
            pend = self._pending
            entry = (w, tuple(pose_at_capture), stamp)
            if pend and self._confirms(pend[-1], entry):
                pend.append(entry)
            else:
                self._pending = [entry]
                pend = self._pending
            if len(pend) <= max(0, int(p.jump_confirm_count)):
                return False
            x_from = min(pt_robot[0], _to_local(pose_at_capture, pend[0][0])[0]) - p.jump_window
            self.points = deque((q for q in self.points if _to_local(pose_at_capture, q)[0] < x_from),
                                maxlen=p.max_points)
            self.points.extend(e[0] for e in pend[:-1])
        self._pending = []
        self.points.append(w)
        return True

    def _confirms(self, prev, cur):
        """cur 가 직전 보류 점 prev 와 같은 자리를 본 것인가. 허용 = jump_confirm + 두 사진 사이 로봇 이동 거리."""
        p = self.p
        (w0, pose0, t0), (w1, pose1, t1) = prev, cur
        if t0 is not None and t1 is not None and t1 - t0 > p.jump_confirm_seconds:
            return False
        moved = math.hypot(pose1[0] - pose0[0], pose1[1] - pose0[1])
        return math.hypot(w1[0] - w0[0], w1[1] - w0[1]) <= p.jump_confirm + moved

    def _lateral_deviation(self, pt_robot, pose):
        """pt_robot 과 같은 전방 거리(± jump_window)에 있는 기억 점들의 옆 위치와의 차이 (m). 비교할 점이 없으면 None."""
        x, y = pt_robot
        near = [loc for loc in (_to_local(pose, q) for q in self.points) if abs(loc[0] - x) <= self.p.jump_window]
        if not near:
            return None
        return min(abs(y - ly) for _, ly in near)

    def prune(self, pose_now):
        p = self.p
        keep = [q for q in self.points if _to_local(pose_now, q)[0] >= p.keep_behind
                and math.hypot(q[0] - pose_now[0], q[1] - pose_now[1]) <= p.max_range]
        if len(keep) != len(self.points):
            self.points = deque(keep, maxlen=p.max_points)

    def target(self, pose_now):
        """따라갈 점 (로봇 좌표). 구동축에서 lookahead 이상 떨어지고 정면 ± max_target_angle_deg 안의 첫 기억 점부터
        average 개 평균. 없으면 None.

        각도 제한: 2026-10-04 pinky2 S자 — 저속으로 급커브를 돌면 예전에 저장한 점이 로봇 바로 옆(전방 3 cm·오른쪽 14 cm)에
        남는데, x > 0 이고 거리 ≥ lookahead 라 목표가 되어 곡률 −14 /m 로 3 s 동안 제자리 우회전했다 (카메라는 왼쪽).
        """
        self.prune(pose_now)
        p = self.p
        cos_max = math.cos(math.radians(p.max_target_angle_deg))
        local = [_to_local(pose_now, q) for q in self.points]
        for i, (x, y) in enumerate(local):
            r = math.hypot(x, y)
            if r >= p.lookahead and x >= cos_max * r:
                sel = [q for q in local[i:i + max(1, p.average)] if q[0] >= cos_max * math.hypot(*q)]
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
