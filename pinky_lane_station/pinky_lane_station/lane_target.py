"""세그 인스턴스 → 차선 중앙 목표점 (ErrorX) · 횡단보도 판정.

ROS 에 의존하지 않는다. 검출기(YOLO-seg / classic / stub) 가 낸 ``Instance`` 목록만 받는다.

    est = LaneTargetEstimator(TargetParams())
    r = est.update(instances, width, height)      # -> TargetResult (LanePath 필드와 1:1)

쌍 선택 (설계 D3 + D14 작업 1)
    lane 인스턴스마다 샘플 행 y_s = sample_row_frac·H 에서 폴리곤과의 교차 구간 중점 x
    (교차가 없으면 대체 행들 → 그래도 없으면 최하단 점 x).
    차선 ≤ 2 : 좌 후보 = x < W/2 중 최대, 우 후보 = x > W/2 중 최소  — 화면 중앙에 가장 가까운 쌍.
    차선 ≥ 3 : 모델 클래스(left_lane / right_lane) 기준 **가장 바깥 쌍** — 왼쪽 클래스 중 가장 왼쪽,
              오른쪽 클래스 중 가장 오른쪽 (예: 왼쪽 1 + 오른쪽 2 → 왼쪽 그대로, 오른쪽은 왼쪽과 가장 먼 것).
              한쪽 클래스가 없으면 그쪽은 화면 위치 규칙, 좌 ≥ 우 로 모순이면 전체를 위치 규칙으로.
    BOTH   : target_x = (xL + xR)/2, 반폭 이력에 (xR − xL)/2 추가
    SINGLE : target_x = x ± half_lane_px (이력 중앙값. 이력이 없으면 초기값)
    LOST   : 없음
    error_x = (target_x − W/2)/(W/2)   차선 중앙이 화면 오른쪽이면 양수 (= 로봇이 왼쪽 치우침)

횡단보도 / 바리게이트 / 빨간 선
    crosswalk(barricade, red_line) 인스턴스의 최하단 y ≥ stop_row_frac·H 이면 raw. 연속 confirm 프레임이면 확정,
    연속 release 프레임 동안 raw 가 없으면 해제. 바리게이트는 해제가 길다 (치워진 걸 1 s 확인).
    빨간 선은 교차로 진입 표시다 — 로봇이 이 플래그로 JUNCTION_STOP 에 들어가고 관제 허가(clear_until) 뒤 통과한다.
"""

from collections import deque
from dataclasses import dataclass, field

QUALITY_BOTH, QUALITY_SINGLE, QUALITY_JUNCTION, QUALITY_STALE, QUALITY_LOST = 0, 1, 2, 3, 4
QUALITY_NAMES = {QUALITY_BOTH: 'BOTH', QUALITY_SINGLE: 'SINGLE', QUALITY_JUNCTION: 'JUNCTION',
                 QUALITY_STALE: 'STALE', QUALITY_LOST: 'LOST'}

CLS_LANE = 'lane'               # 좌/우 정보 없는 차선 (classic·stub 검출기)
CLS_LEFT = 'left_lane'          # 모델 클래스 0
CLS_RIGHT = 'right_lane'        # 모델 클래스 2
CLS_CROSSWALK = 'crosswalk'
CLS_BARRICADE = 'barricade'
CLS_RED_LINE = 'red_line'     # 교차로 빨간 선 (모델 클래스 id 는 재학습 뒤 detector_yolo.yaml class_map 에)
LANE_CLASSES = (CLS_LANE, CLS_LEFT, CLS_RIGHT)
SIDE_OF_CLASS = {CLS_LEFT: 'L', CLS_RIGHT: 'R'}


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
    outer_pair_min_lanes: int = 3         # 차선이 이 수 이상이면 클래스 기준 가장 바깥 쌍
    single_offset_ratio: float = 1.0      # SINGLE: 보이는 선에서 반폭 × 이 비율만큼 안쪽을 목표로 (15 cm 도로, 6 cm → 0.8)
    single_use_class: bool = True         # SINGLE: 선이 하나면 좌/우를 모델 클래스(left_lane/right_lane)로 정한다
    barricade_stop_row_frac: float = 0.80 # 바리게이트 최하단 y ≥ 이 행이면 정지 트리거
    barricade_min_conf: float = 0.30
    barricade_min_width_frac: float = 0.15
    barricade_confirm: int = 3
    barricade_release: int = 10           # ≈ 1 s (10 fps) 동안 안 보여야 해제
    red_line_stop_row_frac: float = 0.80 # 빨간 선 최하단 y ≥ 이 행이면 raw (선이 얇아 하단으로 본다)
    red_line_min_conf: float = 0.30
    red_line_min_width_frac: float = 0.25 # 차로 폭을 가로지르는 선 — 좁은 조각은 무시
    red_line_confirm: int = 2
    red_line_release: int = 5
    red_line_seek_min_width_frac: float = 0.10  # 로봇 '새 빨간 선 찾기' 후보 (red_line_blobs) — 비스듬히 보이는 선도 잡게 느슨하게


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
    left_count: int = 0             # 모델이 왼쪽 클래스로 낸 차선 수
    right_count: int = 0
    pair_rule: str = 'nearest'      # 'nearest' | 'outer' — 이번 프레임 쌍 선택 규칙
    barricade_raw: bool = False
    barricade_detected: bool = False
    barricade_bottom_y: int = 0
    barricade_confidence: float = 0.0
    red_line_raw: bool = False
    red_line_detected: bool = False
    red_line_bottom_y: int = 0
    red_line_confidence: float = 0.0
    red_line_blobs: list = field(default_factory=list)  # [(중심 x, 하단 y, 폭/W), ...] 폭 ≥ seek 하한 — 로봇 seek 동작용
    image_width: int = 0
    image_height: int = 0
    candidates: list = field(default_factory=list)   # [(x, conf, side), ...] 진단용

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
        self._barricade = CrosswalkDebounce(self.p.barricade_confirm, self.p.barricade_release)
        self._red_line = CrosswalkDebounce(self.p.red_line_confirm, self.p.red_line_release)

    def reset(self):
        self._half_hist.clear()
        self._crosswalk.reset()
        self._barricade.reset()
        self._red_line.reset()

    @property
    def half_lane_px(self):
        if not self._half_hist:
            return self.p.half_lane_px_init
        s = sorted(self._half_hist)
        return s[len(s) // 2]

    # ------------------------------------------------ 쌍 선택

    @staticmethod
    def _nearest_pair(cands, cx):
        left = max((c for c in cands if c[0] < cx), key=lambda c: c[0], default=None)
        right = min((c for c in cands if c[0] > cx), key=lambda c: c[0], default=None)
        return left, right

    def _select_pair(self, cands, cx, r):
        """(left, right) 후보. 차선 ≥ outer_pair_min_lanes 이고 클래스 라벨이 있으면 가장 바깥 쌍."""
        r.pair_rule = 'nearest'
        if len(cands) < self.p.outer_pair_min_lanes:
            return self._nearest_pair(cands, cx)
        lefts = [c for c in cands if c[3] == 'L']
        rights = [c for c in cands if c[3] == 'R']
        if not lefts and not rights:
            return self._nearest_pair(cands, cx)
        near_l, near_r = self._nearest_pair(cands, cx)
        left = min(lefts, key=lambda c: c[0]) if lefts else near_l
        right = max(rights, key=lambda c: c[0]) if rights else near_r
        if left and right and left[0] >= right[0]:
            return self._nearest_pair(cands, cx)          # 클래스와 위치가 모순 → 위치 우선
        r.pair_rule = 'outer'
        return left, right

    def _single_side_is_left(self, cand, by_position_left):
        """선이 하나일 때 그 선이 왼쪽 선인가. 클래스 라벨이 있으면 클래스, 없으면 화면 위치."""
        if self.p.single_use_class and len(cand) > 3 and cand[3] in ('L', 'R'):
            return cand[3] == 'L'
        return by_position_left

    # ------------------------------------------------ 메인

    def update(self, instances, width, height):
        p = self.p
        W, H = int(width), int(height)
        cx = W / 2.0
        r = TargetResult(image_width=W, image_height=H)
        rows = [p.sample_row_frac * H] + [f * H for f in p.fallback_row_fracs]

        lanes = [i for i in instances if i.cls in LANE_CLASSES and i.conf >= p.min_lane_conf
                 and i.height >= p.min_lane_height_frac * H and len(i.polygon) >= 3]
        r.lane_count = len(lanes)
        r.left_count = sum(1 for i in lanes if i.cls == CLS_LEFT)
        r.right_count = sum(1 for i in lanes if i.cls == CLS_RIGHT)

        cands = []
        for inst in lanes:
            x, y = instance_x_at_rows(inst, rows)
            cands.append((x, y, inst.conf, SIDE_OF_CLASS.get(inst.cls)))
        r.candidates = [(round(x, 1), round(c, 2), side) for x, _, c, side in cands]

        left, right = self._select_pair(cands, cx, r)
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
            is_left = self._single_side_is_left(c, bool(left))
            offset = p.single_offset_ratio * half           # 보이는 선에서 안쪽으로 (반폭 = 도로 폭/2 에 해당하는 px)
            if is_left:
                r.left_seen, r.left_x = True, int(round(c[0]))
                tx = c[0] + offset
            else:
                r.right_seen, r.right_x = True, int(round(c[0]))
                tx = c[0] - offset
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

        # ------------------------------------------------ 바리게이트
        bars = [i for i in instances if i.cls == CLS_BARRICADE and i.conf >= p.barricade_min_conf
                and i.width >= p.barricade_min_width_frac * W]
        if bars:
            best = max(bars, key=lambda i: i.bottom_y)
            r.barricade_bottom_y = int(round(best.bottom_y))
            r.barricade_confidence = float(best.conf)
            r.barricade_raw = best.bottom_y >= p.barricade_stop_row_frac * H
        r.barricade_detected = self._barricade.update(r.barricade_raw)

        # ------------------------------------------------ 빨간 선 (교차로 진입)
        sls = [i for i in instances if i.cls == CLS_RED_LINE and i.conf >= p.red_line_min_conf
               and i.width >= p.red_line_min_width_frac * W]
        if sls:
            best = max(sls, key=lambda i: i.bottom_y)
            r.red_line_bottom_y = int(round(best.bottom_y))
            r.red_line_confidence = float(best.conf)
            r.red_line_raw = best.bottom_y >= p.red_line_stop_row_frac * H
        r.red_line_detected = self._red_line.update(r.red_line_raw)
        r.red_line_blobs = [(int(round((i.bbox[0] + i.bbox[2]) / 2.0)), int(round(i.bottom_y)),
                             float(i.width / W) if W > 0 else 0.0)
                            for i in instances if i.cls == CLS_RED_LINE and i.conf >= p.red_line_min_conf
                            and i.width >= p.red_line_seek_min_width_frac * W]
        return r
