"""목적지 흰 정지선 검출 — 영상 처리 (YOLO 클래스 아님). ROS 에 의존하지 않는다.

흰 정지선은 차로를 가로지르는 흰 테이프(차로 폭과 비슷한 길이)다. 차선도 흰색이지만 로봇 앞에서는 세로·대각으로 뻗고,
정지선은 **가로로 넓게** 이어진다 — 이 차이로 가른다.

    det = WhiteStopLineDetector(StopLineParams())
    r = det.update(bgr_image)          # StopLineResult(raw, detected, bottom_y, width_frac)

판정
    1. 아래쪽 ROI(roi_top_frac·H ~ H) 에서 흰 픽셀: HSV 명도 ≥ v_min, 채도 ≤ s_max
    2. 가로 틈 메우기(close_px) 뒤 행마다 가장 긴 연속 흰 구간 — 폭 ≥ min_width_frac·W 인 행을 '가로선 행' 으로 친다
    3. 이어진 가로선 행 묶음(두께 min_rows ~ max_rows)의 가장 아래 것 = 정지선 후보, 그 하단 y = bottom_y
    4. bottom_y ≥ stop_row_frac·H 이면 raw (로봇 바로 앞). confirm 프레임 연속이면 확정, release 프레임 없으면 해제

로봇은 교차로를 통과한 **뒤에만** 이 플래그를 센다 (출발 지점 정지선을 도착으로 읽지 않게) — lane_driver 몫.
"""

from dataclasses import dataclass

import numpy as np

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None


@dataclass
class StopLineParams:
    enabled: bool = True
    roi_top_frac: float = 0.55        # 이 행 아래만 본다 (먼 곳의 가로 차선·벽을 뺀다)
    v_min: int = 180                  # 흰색 명도 하한 (0-255)
    s_max: int = 70                   # 흰색 채도 상한 — 빨간 테이프·카펫 색을 뺀다
    close_px: int = 15                # 가로 틈 메우기 커널 폭 (px, 640 기준 비례)
    min_width_frac: float = 0.45      # 한 행의 연속 흰 구간이 이 비율 이상이면 가로선 행
    min_rows: int = 4                 # 가로선 묶음 두께 하한 (px) — 한두 줄 잡음 제거
    max_rows_frac: float = 0.20       # 두께 상한 (H 비율) — 화면을 덮는 흰 물체 제외
    stop_row_frac: float = 0.80       # 하단 y ≥ 이 행이면 raw (로봇이 선 바로 앞)
    confirm: int = 2
    release: int = 3


@dataclass
class StopLineResult:
    raw: bool = False
    detected: bool = False
    bottom_y: int = 0
    top_y: int = 0
    width_frac: float = 0.0


def white_mask(bgr, v_min, s_max):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return ((hsv[:, :, 2] >= v_min) & (hsv[:, :, 1] <= s_max)).astype(np.uint8)


def longest_runs(mask):
    """행마다 가장 긴 연속 1 구간의 길이 (px)."""
    h, w = mask.shape
    out = np.zeros(h, dtype=np.int32)
    for y in range(h):
        row = mask[y]
        if not row.any():
            continue
        padded = np.concatenate(([0], row.astype(np.int8), [0]))
        d = np.diff(padded)
        starts, ends = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
        out[y] = int((ends - starts).max())
    return out


def find_band(bgr, p):
    """(top_y, bottom_y, width_frac) 또는 None — ROI 안의 가장 아래 가로 흰 띠."""
    if cv2 is None:
        raise RuntimeError('python3-opencv 가 필요합니다')
    H, W = bgr.shape[:2]
    y0 = int(max(0.0, min(1.0, p.roi_top_frac)) * H)
    mask = white_mask(bgr[y0:], p.v_min, p.s_max)
    k = max(1, int(round(p.close_px * W / 640.0)))
    if k > 1:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((1, k), np.uint8))
    runs = longest_runs(mask)
    rows = runs >= p.min_width_frac * W
    max_rows = max(p.min_rows, int(p.max_rows_frac * H))
    best = None
    y = 0
    n = len(rows)
    while y < n:
        if not rows[y]:
            y += 1
            continue
        start = y
        while y < n and rows[y]:
            y += 1
        thick = y - start
        if p.min_rows <= thick <= max_rows:
            best = (start, y - 1, float(runs[start:y].max()) / W)     # 아래로 갈수록 덮어쓴다 = 가장 아래 띠
    if best is None:
        return None
    return best[0] + y0, best[1] + y0, best[2]


class WhiteStopLineDetector:
    def __init__(self, params=None):
        self.p = params or StopLineParams()
        self._on = 0
        self._off = 0
        self.detected = False

    def reset(self):
        self._on = self._off = 0
        self.detected = False

    def update(self, bgr):
        r = StopLineResult()
        if self.p.enabled and bgr is not None:
            band = find_band(bgr, self.p)
            if band is not None:
                r.top_y, r.bottom_y, r.width_frac = int(band[0]), int(band[1]), band[2]
                r.raw = r.bottom_y >= self.p.stop_row_frac * bgr.shape[0]
        if r.raw:
            self._on += 1
            self._off = 0
            if self._on >= self.p.confirm:
                self.detected = True
        else:
            self._off += 1
            self._on = 0
            if self._off >= self.p.release:
                self.detected = False
        r.detected = self.detected
        return r


def params_from_dict(d):
    d = d or {}
    return StopLineParams(**{k: v for k, v in d.items() if k in StopLineParams.__dataclass_fields__})
