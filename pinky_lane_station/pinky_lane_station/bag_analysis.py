"""블랙박스(bag) 시계열 분석 — 순수 함수 (D12). ROS·mcap 에 의존하지 않는다.

tools/bag_report.py 가 bag 을 읽어 여기 함수에 넣는다. 모든 시각은 float 초.
"""

import math
from dataclasses import dataclass


@dataclass
class Event:
    t: float
    kind: str
    value: float
    detail: str = ''

    def row(self):
        return [f'{self.t:.3f}', self.kind, f'{self.value:.3f}', self.detail]


# ------------------------------------------------------------------ 주기·갭

def rate_stats(stamps, gap_factor=3.0, min_gap=0.0):
    """{'count','duration','mean_period','max_gap','gaps':[(t, gap)]}. 갭 = 평균 주기의 gap_factor 배 이상."""
    ts = sorted(float(t) for t in stamps)
    n = len(ts)
    if n < 2:
        return {'count': n, 'duration': 0.0, 'mean_period': float('nan'), 'max_gap': 0.0, 'gaps': []}
    duration = ts[-1] - ts[0]
    mean = duration / (n - 1)
    thresh = max(mean * gap_factor, min_gap)
    gaps = [(t1, t1 - t0) for t0, t1 in zip(ts, ts[1:]) if t1 - t0 > thresh]
    return {'count': n, 'duration': duration, 'mean_period': mean,
            'max_gap': max(t1 - t0 for t0, t1 in zip(ts, ts[1:])), 'gaps': gaps}


def gap_events(stamps, label, gap_factor=3.0, min_gap=0.0):
    st = rate_stats(stamps, gap_factor, min_gap)
    return [Event(t, f'{label}_gap', g, f'{label} {g:.2f}s 끊김') for t, g in st['gaps']]


# ------------------------------------------------------------------ 라이다

def _valid(r, range_min, range_max):
    return r is not None and math.isfinite(r) and range_min <= r <= range_max


def front_min(ranges, angle_min, angle_increment, sector_deg=35.0, range_min=0.05, range_max=12.0):
    """전방 ±sector 의 최소 거리와 (유효 수, 전체 수)."""
    half = math.radians(sector_deg)
    best = float('inf')
    valid = total = 0
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_increment
        a = math.atan2(math.sin(a), math.cos(a))
        if abs(a) > half:
            continue
        total += 1
        if _valid(r, range_min, range_max):
            valid += 1
            best = min(best, r)
    return best, valid, total


def scan_events(t, fronts, jump=0.30, invalid_ratio=0.5, invalid_rise=0.3):
    """fronts: [(min_dist, valid, total)] 시각순.
    이벤트: 전방 최소거리가 한 프레임에 jump 이상 변함 · 무효 비율이 invalid_ratio 이상 · 이전 프레임 대비 invalid_rise 이상 급증."""
    out = []
    prev_min = prev_inv = None
    for ti, (m, valid, total) in zip(t, fronts):
        inv = 1.0 - (valid / total) if total else 1.0
        if prev_min is not None and math.isfinite(m) and math.isfinite(prev_min) and abs(m - prev_min) >= jump:
            out.append(Event(ti, 'scan_jump', m - prev_min, f'전방 최소거리 {prev_min:.2f}→{m:.2f} m'))
        if inv >= invalid_ratio and (prev_inv is None or prev_inv < invalid_ratio):
            out.append(Event(ti, 'scan_invalid', inv, f'전방 섹터 무효 {inv * 100:.0f} %'))
        elif prev_inv is not None and inv - prev_inv >= invalid_rise:
            out.append(Event(ti, 'scan_invalid_rise', inv - prev_inv, f'무효 비율 {prev_inv * 100:.0f}→{inv * 100:.0f} %'))
        if total and valid == 0:
            out.append(Event(ti, 'scan_empty', 0.0, '전방 섹터 전부 무효'))
        prev_min, prev_inv = m, inv
    return out


# ------------------------------------------------------------------ 스칼라

def scalar_jumps(t, values, thresh, window=5, label='value', invalid=lambda v: v is None or not math.isfinite(v)):
    """이동 중앙값(window) 대비 thresh 이상 벗어난 샘플. 무효값은 'invalid' 이벤트."""
    out = []
    hist = []
    for ti, v in zip(t, values):
        if invalid(v):
            out.append(Event(ti, f'{label}_invalid', float('nan'), f'{label} 무효값'))
            continue
        if len(hist) >= max(3, window // 2):
            med = sorted(hist)[len(hist) // 2]
            if abs(v - med) >= thresh:
                out.append(Event(ti, f'{label}_jump', v - med, f'{label} {med:.2f}→{v:.2f}'))
        hist = (hist + [v])[-window:]
    return out


# ------------------------------------------------------------------ 속도 불일치

def _interp(ts, vs, t):
    import bisect
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return vs[0]
    if i >= len(ts):
        return vs[-1]
    t0, t1 = ts[i - 1], ts[i]
    u = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return vs[i - 1] + (vs[i] - vs[i - 1]) * u


def velocity_mismatch(t_cmd, v_cmd, t_odom, v_odom, tol=0.05, hold=0.5, lag=0.3):
    """명령 속도와 odom 속도가 tol 이상 다른 상태가 hold 초 넘게 이어지면 그 구간을 이벤트 하나로
    (명령은 lag 만큼 늦게 비교해 정상 응답 지연은 잡지 않는다). value = 구간 내 최대 차이."""
    if len(t_cmd) < 2 or len(t_odom) < 2:
        return []
    out = []
    since = None
    worst = 0.0
    last_t = None

    def close(t_end):
        if since is not None and t_end - since >= hold:
            out.append(Event(since, 'vel_mismatch', worst,
                             f'odom−cmd {worst:+.2f} m/s 가 {t_end - since:.1f}s 지속'))

    for ti, vo in zip(t_odom, v_odom):
        vc = _interp(list(t_cmd), list(v_cmd), ti - lag)
        d = vo - vc
        if abs(d) >= tol:
            if since is None:
                since, worst = ti, d
            elif abs(d) > abs(worst):
                worst = d
        else:
            close(ti)
            since, worst = None, 0.0
        last_t = ti
    if last_t is not None:
        close(last_t)
    return out


# ------------------------------------------------------------------ 상태 전이

def transitions(t, states, names=None):
    """상태가 바뀐 시각들. [(t, from, to)]."""
    out = []
    prev = None
    for ti, s in zip(t, states):
        if prev is not None and s != prev:
            a = names.get(prev, prev) if names else prev
            b = names.get(s, s) if names else s
            out.append((ti, a, b))
        prev = s
    return out


def dwell(t, states, names=None):
    """상태별 누적 시간 (s)."""
    out = {}
    for (t0, s), t1 in zip(zip(t, states), list(t[1:]) + [t[-1] if len(t) else 0.0]):
        k = names.get(s, s) if names else s
        out[k] = out.get(k, 0.0) + max(0.0, t1 - t0)
    return out


def histogram(values, names=None):
    out = {}
    for v in values:
        k = names.get(v, v) if names else v
        out[k] = out.get(k, 0) + 1
    return out
