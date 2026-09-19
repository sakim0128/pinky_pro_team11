"""고정 인스턴스를 돌려주는 검출기 — 파이프라인 배선·테스트용. 이미지를 보지 않는다."""

from .base import Detector, Instance


class StubDetector(Detector):
    name = 'stub'

    def __init__(self, left_x=200.0, right_x=440.0, lane_conf=0.9, crosswalk_bottom_y=None,
                 width=640, height=480, **_ignored):
        self.left_x = None if left_x is None else float(left_x)
        self.right_x = None if right_x is None else float(right_x)
        self.lane_conf = float(lane_conf)
        self.crosswalk_bottom_y = crosswalk_bottom_y
        self.width, self.height = int(width), int(height)

    def _line(self, x):
        H = self.height
        return Instance('lane', self.lane_conf,
                        [(x - 8, H * 0.45), (x + 8, H * 0.45), (x + 12, H), (x - 12, H)])

    def infer(self, image_bgr):
        out = []
        if self.left_x is not None:
            out.append(self._line(self.left_x))
        if self.right_x is not None:
            out.append(self._line(self.right_x))
        if self.crosswalk_bottom_y is not None:
            y1 = float(self.crosswalk_bottom_y)
            out.append(Instance('crosswalk', 0.9,
                                [(80, y1 - 40), (self.width - 80, y1 - 40),
                                 (self.width - 80, y1), (80, y1)]))
        return out
