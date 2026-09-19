"""세그 인스턴스 → 차선 중앙 목표점 (ErrorX) · 횡단보도 판정.

ROS 에 의존하지 않는다. 검출기(YOLO-seg / classic / stub) 가 낸 ``Instance`` 목록만 받는다.

    est = LaneTargetEstimator(TargetParams())
    r = est.update(instances, width, height)      # -> TargetResult (LanePath 필드와 1:1)

쌍 선택 (설계 D3)
    lane 인스턴스마다 샘플 행 y_s = sample_row_frac·H 에서 폴리곤과의 교차 구간 중점 x
    (교차가 없으면 대체 행들 → 그래도 없으면 최하단 점 x).
    좌 후보 = x < W/2 중 최대, 우 후보 = x > W/2 중 최소  — 화면 중앙에 가장 가까운 쌍.
    BOTH   : target_x = (xL + xR)/2, 반폭 이력에 (xR − xL)/2 추가
    SINGLE : target_x = x ± half_lane_px (이력 중앙값. 이력이 없으면 초기값)
    LOST   : 없음
    error_x = (target_x − W/2)/(W/2)   차선 중앙이 화면 오른쪽이면 양수 (= 로봇이 왼쪽 치우침)

횡단보도
    crosswalk 인스턴스의 최하단 y ≥ stop_row_frac·H 이면 raw. 연속 confirm 프레임이면 확정,
    연속 release 프레임 동안 raw 가 없으면 해제.
"""

from collections import deque
from dataclasses import dataclass, field

QUALITY_BOTH, QUALITY_SINGLE, QUALITY_JUNCTION, QUALITY_STALE, QUALITY_LOST = 0, 1, 2, 3, 4
QUALITY_NAMES = {QUALITY_BOTH: 'BOTH', QUALITY_SINGLE: 'SINGLE', QUALITY_JUNCTION: 'JUNCTION',
                 QUALITY_STALE: 'STALE', QUALITY_LOST: 'LOST'}

CLS_LANE = 'lane'
CLS_CROSSWALK = 'crosswalk'


@dataclass
class Instance:
    """검출기 출력 하나. polygon 은 이미지 좌표 [(x, y), ...] (px)."""
    cls: str
    conf: float
    polygon: list
    bbox: tuple = None          # (x0, y0, x1, y1). 없으면 polygon 에서 계산

    def __post_init__(self):
        self.polygon = [(float(x), float(y)) for x, y in self.polygon]
        if self.bbox is None and self.polygon:
            xs = [p[0] for p in self.polygon]
            ys = [p[1] for p in self.polygon]
            self.bbox = (min(xs), min(ys), max(xs), max(ys))

    @property
    def bottom_y(self):
        return self.bbox[3] if self.bbox else float('nan')

    @property
    def width(self):
        return self.bbox[2] - self.bbox[0] if self.bbox else 0.0

    @property
    def height(self):
        return self.bbox[3] - self.bbox[1] if self.bbox else 0.0


@dataclass
class TargetParams:
    sample_row_frac: float = 0.72         # 이 행에서 좌/우 x 를 읽는다
    fallback_row_fracs: tuple = (0.82, 0.62, 0.92)   # 샘플 행에 교차가 없을 때
    half_lane_px_init: float = 120.0      # 640 px 폭 기준 초기 반폭 (실측 후 갱신)
    half_lane_px_min: float = 40.0
    half_lane_px_max: float = 300.0
    half_lane_history: int = 15           # 이동 중앙값 창
    narrow_pair_frac: float = 0.35        # 쌍 폭 < 이 비율 × 2·half 이면 같은 선 조각으로 본다
    min_lane_conf: float = 0.25
    min_lane_height_frac: float = 0.05    # bbox 높이가 H 의 이 비율 미만이면 무시 (노이즈)
    crosswalk_stop_row_frac: float = 0.80 # 최하단 y ≥ 이 행이면 정지 트리거
    crosswalk_min_conf: float = 0.30
    crosswalk_min_width_frac: float = 0.25
    crosswalk_confirm: int = 3
    crosswalk_release: int = 5


@dataclass
class TargetResult:
    quality: int = QUALITY_LOST
    error_x: float = 0.0
    target_x: int = 0
    target_y: int = 0
    left_x: int = 0
    right_x: int = 0
    left_seen: bool = False
    right_seen: bool = False
    half_lane_px: float = 0.0
    confidence: float = 0.0
    lane_count: int = 0
    crosswalk_raw: bool = False
    crosswalk_detected: bool = False
    crosswalk_bottom_y: int = 0
    crosswalk_confidence: float = 0.0
    image_width: int = 0
    image_height: int = 0
    candidates: list = field(default_factory=list)   # [(x, conf), ...] 진단용

    @property
    def quality_name(self):
        return QUALITY_NAMES[self.quality]


def polygon_row_span(polygon, y):
    """폴리곤과 수평선 y 의 교차 x 들의 (min, max). 없으면 None."""
    xs = []
    n = len(polygon)
    if n < 3:
        return None
    for i in range(n):
        x0, y0 = polygon[i]
        x1, y1 = polygon[(i + 1) % n]
        if (y0 <= y < y1) or (y1 <= y < y0):
            xs.append(x0 + (y - y0) * (x1 - x0) / (y1 - y0))
    if not xs:
        return None
    return min(xs), max(xs)


def instance_x_at_rows(inst, rows):
    """행 후보들을 순서대로 시도해 교차 구간 중점 x 와 실제 쓴 행을 돌려준다.
    전부 실패하면 최하단 점의 x (와 그 y)."""
    for y in rows:
        span = polygon_row_span(inst.polygon, y)
        if span is not None:
            return 0.5 * (span[0] + span[1]), y
    bx, by = max(inst.polygon, key=lambda p: p[1])
    return bx, by


class CrosswalkDebounce:
    def __init__(self, confirm=3, release=5):
        self.confirm = int(confirm)
        self.release = int(release)
        self._on = 0
        self._off = 0
        self.detected = False

    def update(self, raw):
        if raw:
            self._on += 1
            self._off = 0
            if self._on >= self.confirm:
                self.detected = True
        else:
            self._off += 1
            self._on = 0
            if self._off >= self.release:
                self.detected = False
        return self.detected

    def reset(self):
        self._on = self._off = 0
        self.detected = False


class LaneTargetEstimator:
    def __init__(self, params=None):
        self.p = params or TargetParams()
        self._half_hist = deque(maxlen=self.p.half_lane_history)
        self._crosswalk = CrosswalkDebounce(self.p.crosswalk_confirm, self.p.crosswalk_release)

    def reset(self):
        self._half_hist.clear()
        self._crosswalk.reset()

    @property
    def half_lane_px(self):
        if not self._half_hist:
            return self.p.half_lane_px_init
        s = sorted(self._half_hist)
        return s[len(s) // 2]

    # ------------------------------------------------ 메인

    def update(self, instances, width, height):
        p = self.p
        W, H = int(width), int(height)
        cx = W / 2.0
        r = TargetResult(image_width=W, image_height=H)
        rows = [p.sample_row_frac * H] + [f * H for f in p.fallback_row_fracs]

        lanes = [i for i in instances if i.cls == CLS_LANE and i.conf >= p.min_lane_conf
                 and i.height >= p.min_lane_height_frac * H and len(i.polygon) >= 3]
        r.lane_count = len(lanes)

        cands = []
        for inst in lanes:
            x, y = instance_x_at_rows(inst, rows)
            cands.append((x, y, inst.conf))
        r.candidates = [(round(x, 1), round(c, 2)) for x, _, c in cands]

        left = max((c for c in cands if c[0] < cx), key=lambda c: c[0], default=None)
        right = min((c for c in cands if c[0] > cx), key=lambda c: c[0], default=None)
        half = self.half_lane_px

        if left and right and (right[0] - left[0]) < p.narrow_pair_frac * 2.0 * half:
            # 같은 선이 조각나 중앙 양쪽에 걸친 경우 → 한 선으로 합친다
            mx = 0.5 * (left[0] + right[0])
            merged = (mx, 0.5 * (left[1] + right[1]), max(left[2], right[2]))
            if mx < cx:
                left, right = merged, None
            else:
                left, right = None, merged

        if left and right:
            xl, xr = left[0], right[0]
            new_half = 0.5 * (xr - xl)
            if p.half_lane_px_min <= new_half <= p.half_lane_px_max:
                self._half_hist.append(new_half)
                half = self.half_lane_px
            r.quality = QUALITY_BOTH
            r.left_seen = r.right_seen = True
            r.left_x, r.right_x = int(round(xl)), int(round(xr))
            r.target_x = int(round(0.5 * (xl + xr)))
            r.target_y = int(round(0.5 * (left[1] + right[1])))
            r.confidence = min(left[2], right[2])
        elif left or right:
            c = left or right
            r.quality = QUALITY_SINGLE
            if left:
                r.left_seen, r.left_x = True, int(round(c[0]))
                tx = c[0] + half
            else:
                r.right_seen, r.right_x = True, int(round(c[0]))
                tx = c[0] - half
            r.target_x = int(round(tx))
            r.target_y = int(round(c[1]))
            r.confidence = c[2] * 0.8
        else:
            r.quality = QUALITY_LOST
            r.target_x = int(round(cx))
            r.target_y = int(round(rows[0]))
            r.confidence = 0.0

        r.half_lane_px = float(half)
        if r.quality != QUALITY_LOST:
            r.error_x = max(-1.0, min(1.0, (r.target_x - cx) / cx))

        # ------------------------------------------------ 횡단보도
        cws = [i for i in instances if i.cls == CLS_CROSSWALK and i.conf >= p.crosswalk_min_conf
               and i.width >= p.crosswalk_min_width_frac * W]
        if cws:
            best = max(cws, key=lambda i: i.bottom_y)
            r.crosswalk_bottom_y = int(round(best.bottom_y))
            r.crosswalk_confidence = float(best.conf)
            r.crosswalk_raw = best.bottom_y >= p.crosswalk_stop_row_frac * H
        r.crosswalk_detected = self._crosswalk.update(r.crosswalk_raw)
        return r
