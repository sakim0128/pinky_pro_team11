"""교차로 빨간 테이프 검출 — 영상 처리(HSV 색). 모델에 빨간 선 클래스가 없어서 색으로 찾는다. ROS 에 의존하지 않는다.

    det = RedLineColorDetector(RedLineParams())
    insts = det.detect(bgr_image)      # [Instance('red_line', conf, polygon, bbox)]

판정
    1. 아래쪽 ROI(roi_top_frac·H ~ H) 에서 빨간 픽셀: 색상 H ≤ h_low_max 또는 H ≥ h_high_min (OpenCV 0~179), 채도 ≥ s_min, 명도 ≥ v_min
       (회색 카펫·흰 테이프·흰 벽은 채도가 낮아 빠진다)
    2. 가로 틈 메우기(close_px) + 잡음 제거(open_px) 뒤 윤곽선마다 면적 ≥ min_area_frac·(ROI 면적) 인 덩어리
    3. 덩어리마다 Instance('red_line') — conf = 외접 사각형 대비 채움 비율(0.3~1.0 로 자름), 좌표는 원본 영상 px

인스턴스는 YOLO 결과에 합쳐 lane_target 으로 넘긴다 → 폭(red_line_min_width_frac)·하단 행(red_line_stop_row_frac)·
확정 프레임(red_line_confirm) 판정은 lane_target 이 그대로 한다. 원본(마스킹 전) 영상에서 찾는다.
"""

from dataclasses import dataclass

import numpy as np

from .lane_target import Instance

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None

CLS_RED_LINE = 'red_line'


@dataclass
class RedLineParams:
    enabled: bool = True
    roi_top_frac: float = 0.55        # 이 행 아래만 본다 (먼 곳·벽의 붉은 물체 제외)
    h_low_max: int = 10               # 빨강은 색상 원의 양 끝 — 0~h_low_max
    h_high_min: int = 170             #                     h_high_min~179
    s_min: int = 100                  # 채도 하한 — 회색 카펫·흰 테이프 제외. 조명이 약해 테이프가 안 잡히면 낮춘다
    v_min: int = 60                   # 명도 하한 — 그림자 속 어두운 픽셀 제외
    close_px: int = 9                 # 가로 틈 메우기 커널 폭 (px, 640 폭 기준 비례)
    open_px: int = 3                  # 잡음 제거 커널 (px)
    min_area_frac: float = 0.004      # ROI 면적 대비 덩어리 면적 하한 (640×216 ROI 에서 ≈ 550 px)


class RedLineColorDetector:
    def __init__(self, params=None):
        self.p = params or RedLineParams()
        self.last_mask = None          # 디버그용 (ROI 크기)

    def mask(self, bgr):
        """ROI 의 빨간 픽셀 마스크 (uint8 0/1) 와 ROI 시작 행."""
        if cv2 is None:
            raise RuntimeError('python3-opencv 가 필요합니다')
        p = self.p
        H, W = bgr.shape[:2]
        y0 = int(max(0.0, min(1.0, p.roi_top_frac)) * H)
        hsv = cv2.cvtColor(bgr[y0:], cv2.COLOR_BGR2HSV)
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        red = ((h <= p.h_low_max) | (h >= p.h_high_min)) & (s >= p.s_min) & (v >= p.v_min)
        m = red.astype(np.uint8)
        scale = W / 640.0
        ko = max(1, int(round(p.open_px * scale)))
        if ko > 1:
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((ko, ko), np.uint8))
        kc = max(1, int(round(p.close_px * scale)))
        if kc > 1:
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((max(1, kc // 3), kc), np.uint8))
        return m, y0

    def detect(self, bgr):
        if not self.p.enabled or bgr is None:
            return []
        m, y0 = self.mask(bgr)
        self.last_mask = m
        roi_area = float(m.shape[0] * m.shape[1]) or 1.0
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            area = float(cv2.contourArea(c))
            if area < self.p.min_area_frac * roi_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            fill = area / float(max(1, w * h))
            poly = [(float(px), float(py + y0)) for px, py in c.reshape(-1, 2)]
            if len(poly) < 3:
                poly = [(x, y + y0), (x + w, y + y0), (x + w, y + h + y0), (x, y + h + y0)]
            out.append(Instance(CLS_RED_LINE, max(0.3, min(1.0, fill)), poly,
                                (float(x), float(y + y0), float(x + w), float(y + h + y0))))
        return out


def params_from_dict(d):
    d = d or {}
    return RedLineParams(**{k: v for k, v in d.items() if k in RedLineParams.__dataclass_fields__})
