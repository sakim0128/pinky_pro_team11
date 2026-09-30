"""교차로 고정 동작 실행기 — odom 으로 '직진 d m', '제자리 회전 θ°' 단계를 차례로 수행한다. ROS 에 의존하지 않는다.

    ex = ManeuverExecutor([('straight', 0.18), ('turn', -90.0), ('straight', 0.10)], ManeuverParams())
    v, w, done, reason = ex.command(now, odom_pose)      # 매 틱. odom_pose = (x, y, yaw) 또는 None

('seek', +1 좌 / −1 우 / 0 직진) 단계는 각도·거리 대신 카메라로 **새 빨간 선** 을 찾아 그 앞까지 간다 (RedLineSeeker):
    좌·우: seek_forward 전진 → 제자리 회전, seek_min_turn_deg 뒤부터 화면 가운데 빨간 선을 seek_confirm 프레임 → 접근
    직진: seek_ignore_distance 동안은 빨간 선을 안 본다(입구 선) → 전진하며 가운데 빨간 선 확정 → 접근
    접근: 선 중심을 향해 seek_speed 로 전진, 선 하단 ≥ seek_arrive_row_frac·H 면 끝. 못 찾으면 멈추고 failed
    red = [(x_norm, width_frac, bottom_frac), ...] 이번 프레임 빨간 덩어리 전부, red_frame = 이미지 stamp (None 이면 새 프레임 아님)

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
    # ('seek', dir) — 새 빨간 선 찾기
    seek_forward: float = 0.20        # 좌·우: 회전 전 전진 (m). 교차로 안으로 들어가 나갈 가지가 보이게
    seek_omega: float = 0.3           # 찾는 동안 제자리 회전 (rad/s)
    seek_min_turn_deg: float = 30.0   # 이만큼 돈 뒤부터 빨간 선을 본다 (입구·직진 방향 선 무시)
    seek_max_turn_deg: float = 150.0  # 넘도록 못 찾으면 정지 (failed)
    seek_ignore_distance: float = 0.10  # 직진: 이만큼 가기 전에는 빨간 선을 안 본다 (입구 선)
    seek_max_distance: float = 0.80   # 직진: 이만큼 가도 못 찾으면 정지 (failed)
    seek_center_frac: float = 0.25    # 선 중심이 화면 중앙 ± 이 비율·W 안이면 '가운데'
    seek_min_width_frac: float = 0.10 # 선 폭 ≥ 이 비율·W
    seek_confirm: int = 2             # 가운데 선이 연속 이 프레임
    seek_speed: float = 0.05          # 직진 탐색·접근 속도 (m/s)
    seek_kp: float = 0.8              # 접근 조향 ω = −kp·x_norm (rad/s)
    seek_arrive_row_frac: float = 0.80  # 선 하단 ≥ 이 비율·H 면 도착
    seek_lost_distance: float = 0.10  # 접근 중 선이 안 보인 채 이만큼 가면 지나친 것으로 보고 끝


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


STEP_KINDS = ('straight', 'turn', 'seek')


def parse_steps(kinds, values):
    """메시지 배열 → [(kind, value)]. 알 수 없는 kind 는 ValueError. seek 값은 부호만 (+1 좌 / −1 우 / 0 직진)."""
    if len(kinds) != len(values):
        raise ValueError(f'step_kind {len(kinds)} 개 ≠ step_value {len(values)} 개')
    out = []
    for k, v in zip(kinds, values):
        k = str(k).strip().lower()
        if k not in STEP_KINDS:
            raise ValueError(f'알 수 없는 동작 {k!r} (straight | turn | seek)')
        v = float(v)
        if k == 'seek':
            v = 0.0 if abs(v) < 0.5 else math.copysign(1.0, v)
        out.append((k, v))
    return out


SEEK_NAMES = {1.0: '좌', -1.0: '우', 0.0: '직진'}


class RedLineSeeker:
    """새 빨간 선 찾기 → 그 앞까지 가기. command(pose, blobs, frame) → (v, ω, done, reason).

    blobs = [(x_norm, width_frac, bottom_frac), ...] — 이번 프레임의 빨간 덩어리 전부. frame = 프레임 식별값(이미지 stamp),
    None 이면 새 프레임이 아니다(STALE) — 확정 카운트·추적을 갱신하지 않는다.
    찾기: 가운데(|x_norm| ≤ 2·seek_center_frac)·폭 ≥ seek_min_width_frac·**아직 앞에 있는**(하단 < seek_arrive_row_frac) 덩어리.
          발밑의 입구 선은 하단이 이미 도착 행 아래라 후보가 아니다.
    접근: 확정한 덩어리를 프레임마다 가장 가까운 것으로 추적(하단이 갑자기 튀면 다른 선 — 버린다). 하단 ≥ 도착 행이면 끝.
    실패(최대 회전각·최대 거리)면 failed=True 로 멈춘다 — done 은 거짓으로 남아 교차로 상태에서 서 있는다.
    """

    TRACK_DX = 0.5          # 추적 게이트: 프레임 사이 x_norm 변화
    TRACK_DBOTTOM = 0.20    # 프레임 사이 하단(비율) 변화

    def __init__(self, direction, params=None):
        self.dir = 0.0 if abs(direction) < 0.5 else math.copysign(1.0, direction)
        self.p = params or ManeuverParams()
        self.phase = 'forward' if self.dir != 0.0 else 'ignore'
        self.done = False
        self.failed = False
        self._start = None
        self._heading = None
        self._turned = 0.0
        self._last_yaw = None
        self._hits = 0
        self._frame = None
        self._target = None          # (x_norm, bottom_frac) — 접근 중 추적
        self._seen_xy = None         # 마지막으로 목표를 본 위치
        self._w = 0.0

    @property
    def name(self):
        return SEEK_NAMES[self.dir]

    def _enter(self, phase, pose):
        self.phase = phase
        self._start = pose
        self._last_yaw = pose[2]
        self._heading = pose[2]
        self._hits = 0

    def _candidate(self, blobs):
        p = self.p
        good = [b for b in blobs if abs(b[0]) <= 2.0 * p.seek_center_frac and b[1] >= p.seek_min_width_frac
                and b[2] < p.seek_arrive_row_frac]
        return min(good, key=lambda b: abs(b[0])) if good else None

    def _new_frame(self, frame):
        if frame is None or frame == self._frame:
            return False
        self._frame = frame
        return True

    def _confirm(self, blobs, fresh):
        """가운데 새 선을 새 프레임마다 센다. 확정되면 그 덩어리, 아니면 None."""
        if not fresh:
            return None
        c = self._candidate(blobs)
        self._hits = self._hits + 1 if c is not None else 0
        return c if self._hits >= self.p.seek_confirm else None

    def _hold(self, yaw):
        p = self.p
        w = p.heading_kp * _wrap(self._heading - yaw)
        return max(-p.angular_speed, min(p.angular_speed, w))

    def _start_approach(self, pose, blob):
        self._enter('approach', pose)
        self._target = (blob[0], blob[2])
        self._seen_xy = pose[:2]
        self._w = self._steer(blob[0])

    def _steer(self, x_norm):
        p = self.p
        return max(-p.angular_speed, min(p.angular_speed, -p.seek_kp * x_norm))

    def _track(self, blobs):
        p = self.p
        tx, tb = self._target
        near = [b for b in blobs if b[1] >= p.seek_min_width_frac and abs(b[0] - tx) <= self.TRACK_DX
                and abs(b[2] - tb) <= self.TRACK_DBOTTOM]
        return min(near, key=lambda b: abs(b[0] - tx) + abs(b[2] - tb)) if near else None

    def command(self, pose, blobs=(), frame=None):
        p = self.p
        if self.done:
            return 0.0, 0.0, True, f'빨간 선 찾기({self.name}) 완료'
        if self.failed:
            return 0.0, 0.0, False, f'새 빨간 선 못 찾음({self.name}) — 정지'
        blobs = list(blobs or ())
        fresh = self._new_frame(frame)
        x, y, yaw = pose
        if self._start is None:
            self._enter(self.phase, pose)
        moved = math.hypot(x - self._start[0], y - self._start[1])
        if self.phase == 'forward':
            remain = p.seek_forward - moved
            if remain > p.dist_tolerance:
                speed = p.linear_speed
                if remain < p.slow_distance:
                    speed = max(p.min_linear, speed * remain / p.slow_distance)
                return speed, self._hold(yaw), False, f'빨간 선 찾기({self.name}) 전진 {moved:.2f}/{p.seek_forward:.2f} m'
            self._enter('rotate', pose)
            moved = 0.0
        if self.phase == 'ignore':
            if moved < p.seek_ignore_distance:
                return p.seek_speed, self._hold(yaw), False, \
                    f'빨간 선 찾기(직진) 입구 선 지나기 {moved:.2f}/{p.seek_ignore_distance:.2f} m'
            self._enter('scan', pose)
            moved = 0.0
        if self.phase == 'rotate':
            self._turned += _wrap(yaw - self._last_yaw)
            self._last_yaw = yaw
            turned = math.degrees(self._turned * self.dir)
            found = self._confirm(blobs, fresh) if turned >= p.seek_min_turn_deg else None
            if found is not None:
                self._start_approach(pose, found)
            elif turned >= p.seek_max_turn_deg:
                self.failed = True
                return 0.0, 0.0, False, f'새 빨간 선 못 찾음({self.name}) — {turned:.0f}° 회전, 정지'
            else:
                look = '찾는 중' if turned >= p.seek_min_turn_deg else '회전'
                return 0.0, self.dir * p.seek_omega, False, f'빨간 선 찾기({self.name}) {look} {turned:.0f}°'
        if self.phase == 'scan':
            found = self._confirm(blobs, fresh)
            if found is not None:
                self._start_approach(pose, found)
            elif moved >= p.seek_max_distance:
                self.failed = True
                return 0.0, 0.0, False, f'새 빨간 선 못 찾음(직진) — {moved:.2f} m, 정지'
            else:
                return p.seek_speed, self._hold(yaw), False, f'빨간 선 찾기(직진) {moved:.2f} m'
        # approach
        if fresh:
            b = self._track(blobs)
            if b is not None:
                self._target = (b[0], b[2])
                self._seen_xy = (x, y)
                if b[2] >= p.seek_arrive_row_frac:
                    self.done = True
                    return 0.0, 0.0, True, f'빨간 선 찾기({self.name}) — 새 선 앞 도착'
                self._w = self._steer(b[0])
        lost = math.hypot(x - self._seen_xy[0], y - self._seen_xy[1])
        if lost >= p.seek_lost_distance:
            self.done = True
            return 0.0, 0.0, True, f'빨간 선 찾기({self.name}) — 선이 발 밑으로 지나감, 완료'
        tx, tb = self._target
        return p.seek_speed, self._w, False, f'새 빨간 선으로 접근 x={tx:+.2f} 하단 {tb:.2f}'


class ManeuverExecutor:
    def __init__(self, steps, params=None):
        self.steps = list(steps)
        self.p = params or ManeuverParams()
        self.index = 0
        self._start = None            # (x, y, yaw) — 현재 단계 시작 odom
        self._turned = 0.0            # 현재 회전 단계 누적 (rad, 부호 있음)
        self._last_yaw = None
        self._seeker = None
        self._seek_reason = ''
        self.done = not self.steps

    @property
    def failed(self):
        """seek 단계가 새 빨간 선을 못 찾고 멈췄다."""
        return self._seeker is not None and self._seeker.failed

    @property
    def last_turn_sign(self):
        """마지막 회전 단계의 방향 (+1 좌, −1 우, 0 회전 없음) — 차선 탐색 회전 방향에 쓴다."""
        for kind, value in reversed(self.steps):
            if kind in ('turn', 'seek') and abs(value) > 1e-6:
                return 1.0 if value > 0 else -1.0
        return 0.0

    def describe(self):
        kind, value = self.steps[min(self.index, len(self.steps) - 1)] if self.steps else ('', 0.0)
        head = f'교차로 동작 {min(self.index + 1, len(self.steps))}/{len(self.steps)}'
        if kind == 'seek':
            return f'{head} {self._seek_reason or "빨간 선 찾기(" + SEEK_NAMES[value] + ")"}'
        unit = 'm' if kind == 'straight' else '°'
        return f'{head} {kind} {value:+.2f}{unit}'

    def _advance(self):
        self.index += 1
        self._start = None
        self._turned = 0.0
        self._last_yaw = None
        self._seeker = None
        self._seek_reason = ''
        if self.index >= len(self.steps):
            self.done = True

    def command(self, now, pose, red=(), red_frame=None):
        """(v, ω, done, reason). red·red_frame 은 seek 단계만 쓴다 (RedLineSeeker.command)."""
        if self.done:
            return 0.0, 0.0, True, '교차로 동작 완료'
        if pose is None:
            return 0.0, 0.0, False, '교차로 동작 — odom 없음, 대기'
        x, y, yaw = pose
        p = self.p
        while not self.done:
            kind, value = self.steps[self.index]
            if kind == 'seek':
                if self._seeker is None:
                    self._seeker = RedLineSeeker(value, p)
                v, w, fin, self._seek_reason = self._seeker.command(pose, red, red_frame)
                if fin:
                    self._advance()
                    continue
                return v, w, False, self.describe()
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
