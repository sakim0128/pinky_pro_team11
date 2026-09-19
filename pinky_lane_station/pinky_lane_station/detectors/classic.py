"""OpenCV 흰색 테이프 검출기 — 모델이 없을 때의 대체 · 가짜 카메라 검증용.

회색 카펫 위 흰 테이프를 밝기 임계로 뽑아 연결 요소마다 폴리곤을 만든다.
벽도 흰색이라 화면 위쪽은 버린다 (roi_top_frac). 가로로 넓고 납작한 요소가 여러 개
같은 높이에 모이면 crosswalk 로 낸다. 실전 검출기는 YOLO-seg 이고 이건 안전망이다.
"""

import numpy as np

from .base import Detector, Instance

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None


class ClassicDetector(Detector):
    name = 'classic'

    def __init__(self, white_thresh=170, sat_max=70, roi_top_frac=0.40, min_area=150,
                 stripe_run_frac=0.15, crosswalk_min_width_frac=0.30, crosswalk_min_stripes=2,
                 blur=5, **_ignored):
        if cv2 is None:
            raise RuntimeError('python3-opencv 가 필요합니다')
        self.white_thresh = int(white_thresh)
        self.sat_max = int(sat_max)
        self.roi_top_frac = float(roi_top_frac)
        self.min_area = int(min_area)
        self.stripe_run_frac = float(stripe_run_frac)       # 이 비율×W 이상 가로 런만 줄무늬
        self.crosswalk_min_width_frac = float(crosswalk_min_width_frac)
        self.crosswalk_min_stripes = int(crosswalk_min_stripes)
        self.blur = int(blur)

    def mask(self, image_bgr):
        H, W = image_bgr.shape[:2]
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, (0, 0, self.white_thresh), (180, self.sat_max, 255))
        m[: int(self.roi_top_frac * H), :] = 0
        if self.blur > 1:
            m = cv2.medianBlur(m, self.blur | 1)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        return m

    def split(self, m):
        """흰색 마스크를 (차선, 줄무늬) 로 나눈다. 가로로 긴 런만 남기는 열림 연산이 줄무늬다."""
        W = m.shape[1]
        k = max(15, int(self.stripe_run_frac * W))
        stripes = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((1, k), np.uint8))
        lanes = cv2.bitwise_and(m, cv2.bitwise_not(
            cv2.dilate(stripes, np.ones((7, 7), np.uint8))))
        return lanes, stripes

    def _contours(self, mask, cls):
        H, W = mask.shape[:2]
        out = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            poly = [(float(px), float(py)) for px, py in c.reshape(-1, 2)]
            if len(poly) < 3:
                continue
            conf = min(1.0, 0.5 + area / (0.02 * W * H))
            out.append(Instance(cls, conf, poly, (x, y, x + w, y + h)))
        return out

    def infer(self, image_bgr):
        H, W = image_bgr.shape[:2]
        lane_mask, stripe_mask = self.split(self.mask(image_bgr))
        out = self._contours(lane_mask, 'lane')
        stripes = [s for s in self._contours(stripe_mask, 'crosswalk')
                   if s.width >= self.crosswalk_min_width_frac * W]
        if len(stripes) >= self.crosswalk_min_stripes:
            x0 = min(s.bbox[0] for s in stripes)
            y0 = min(s.bbox[1] for s in stripes)
            x1 = max(s.bbox[2] for s in stripes)
            y1 = max(s.bbox[3] for s in stripes)
            conf = max(s.conf for s in stripes)
            out.append(Instance('crosswalk', conf, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)],
                                (x0, y0, x1, y1)))
        # 줄무늬가 하나뿐이면 횡단보도로 내지 않는다 — 벽 하단선일 수 있다
        return out
