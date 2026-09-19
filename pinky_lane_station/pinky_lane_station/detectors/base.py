"""검출기 공통 인터페이스. 출력은 lane_target.Instance 목록."""

import time

from ..lane_target import Instance

__all__ = ['Detector', 'DetectorError', 'Instance']


class DetectorError(RuntimeError):
    pass


class Detector:
    name = 'base'

    def warmup(self, width=640, height=480):
        """첫 추론 지연을 미리 치른다. 기본은 아무것도 안 함."""

    def infer(self, image_bgr):
        """image_bgr (H×W×3 uint8) → [Instance]. 좌표는 입력 이미지 px."""
        raise NotImplementedError

    def infer_timed(self, image_bgr):
        t0 = time.perf_counter()
        out = self.infer(image_bgr)
        return out, (time.perf_counter() - t0) * 1000.0

    def close(self):
        pass
