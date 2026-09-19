"""전방 장애물 감지 — 라이다 섹터 + 초음파. 치워지면 스스로 해제한다.

ROS 에 의존하지 않는다. 과제 필수 요구 4번 (장애물 정지, 제거 후 자동 재주행).

라이다는 바닥에서 12.5 cm 라 그보다 낮은 물체는 못 본다. 초음파가 그 아래를 맡는다.
RPLidar 는 무효 값을 NaN / inf / 0.0 세 가지로 낸다 — 전부 버린다.
"""

import math
from dataclasses import dataclass


@dataclass
class GuardParams:
    sector_deg: float = 35.0         # 전방 ± 이 각도만 본다
    box_x: float = 0.18              # 정지 박스 전방 깊이 (m, 라이다 기준)
    box_y: float = 0.10              # 정지 박스 반폭 (m). 로봇 반폭 0.055 + 여유
    us_stop: float = 0.20            # 초음파 정지 거리 (m)
    us_min_valid: float = 0.03       # 이보다 작으면 초음파 무효
    confirm_count: int = 2           # 연속 감지 횟수
    clear_seconds: float = 1.0       # 이만큼 비어 있으면 해제
    lidar_offset_x: float = 0.0      # base_footprint 기준 라이다 위치 (필요하면 조정)


class ObstacleGuard:
    def __init__(self, params=None):
        self.p = params or GuardParams()
        self._lidar_hit = False
        self._us_hit = False
        self._hits = 0
        self._blocked = False
        self._clear_since = None
        self.lidar_min = float('inf')
        self.us_range = float('nan')
        self._us_hist = []
        self.reason = ''

    # ------------------------------------------------ 입력

    def update_scan(self, ranges, angle_min, angle_increment, range_min=0.05, range_max=12.0):
        """정지 박스 안에 점이 있는지. 전방 섹터 최소 거리도 기록."""
        p = self.p
        half = math.radians(p.sector_deg)
        hit = False
        best = float('inf')
        for i, r in enumerate(ranges):
            if r is None or not math.isfinite(r) or r <= 0.0 or r < range_min or r > range_max:
                continue
            a = angle_min + i * angle_increment
            a = math.atan2(math.sin(a), math.cos(a))     # -pi..pi
            if abs(a) > half:
                continue
            x = r * math.cos(a) + p.lidar_offset_x
            y = r * math.sin(a)
            if x > 0.0:
                best = min(best, r)
            if 0.0 < x <= p.box_x and abs(y) <= p.box_y:
                hit = True
        self.lidar_min = best
        self._lidar_hit = hit
        return hit

    def update_us(self, rng):
        """초음파 3샘플 중앙값. 정지 거리 안이면 hit."""
        p = self.p
        if rng is None or not math.isfinite(rng) or rng < p.us_min_valid:
            self._us_hit = False
            return False
        self._us_hist = (self._us_hist + [float(rng)])[-3:]
        med = sorted(self._us_hist)[len(self._us_hist) // 2]
        self.us_range = med
        self._us_hit = med <= p.us_stop
        return self._us_hit

    # ------------------------------------------------ 판정

    def step(self, now):
        """(blocked, reason). 연속 confirm_count 감지로 걸리고 clear_seconds 비면 풀린다."""
        p = self.p
        any_hit = self._lidar_hit or self._us_hit
        if any_hit:
            self._hits += 1
            self._clear_since = None
            if self._hits >= p.confirm_count:
                self._blocked = True
        else:
            self._hits = 0
            if self._blocked:
                if self._clear_since is None:
                    self._clear_since = now
                elif now - self._clear_since >= p.clear_seconds:
                    self._blocked = False
                    self._clear_since = None
        if self._blocked:
            parts = []
            if self._lidar_hit and math.isfinite(self.lidar_min):
                parts.append(f'라이다 {self.lidar_min:.2f}m')
            if self._us_hit:
                parts.append(f'초음파 {self.us_range:.2f}m')
            if not parts and self._clear_since is not None:
                parts.append(f'장애물 해제 대기 {now - self._clear_since:.1f}s')
            self.reason = '장애물: ' + ', '.join(parts) if parts else '장애물'
        else:
            self.reason = ''
        return self._blocked, self.reason

    @property
    def blocked(self):
        return self._blocked
