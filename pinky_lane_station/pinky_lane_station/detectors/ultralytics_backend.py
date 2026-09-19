"""ultralytics YOLO-seg 백엔드. import 는 생성 시점에만 한다 (torch 로딩 수 초).

    det = UltralyticsDetector(model='~/models/lane_v1.pt', device='cpu', imgsz=416, conf=0.35,
                              class_map={'lane': ['lane', 'line'], 'crosswalk': ['crosswalk']})

class_map 은 우리 클래스명 → 모델 클래스명(들). 비우면 모델의 names 를 그대로 쓴다.
polygon 은 results[0].masks.xy (원본 이미지 px). masks 가 없으면(det 모델) bbox 사각형.
"""

import os

from .base import Detector, DetectorError, Instance


class UltralyticsDetector(Detector):
    name = 'ultralytics'

    def __init__(self, model, device='cpu', imgsz=416, conf=0.35, iou=0.5, half=False,
                 class_map=None, max_det=20, **_ignored):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise DetectorError('ultralytics 가 설치되어 있지 않습니다 (pip install ultralytics)') from exc
        path = os.path.expanduser(os.path.expandvars(str(model)))
        if not os.path.isfile(path):
            raise DetectorError(f'모델 파일이 없습니다: {path}')
        self.model_path = path
        self.model = YOLO(path)
        self.device = device
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        self.half = bool(half)
        self.max_det = int(max_det)
        names = self.model.names if isinstance(self.model.names, dict) else dict(enumerate(self.model.names))
        self.names = {int(k): str(v) for k, v in names.items()}
        self.class_map = self._build_class_map(class_map)
        self.name = f'ultralytics@{os.path.basename(path)}'

    def _build_class_map(self, class_map):
        """모델 클래스 id → 우리 클래스명."""
        out = {}
        if class_map:
            for ours, theirs in class_map.items():
                theirs = [theirs] if isinstance(theirs, (str, int)) else list(theirs)
                for t in theirs:
                    for cid, nm in self.names.items():
                        if (isinstance(t, int) and cid == t) or (str(t).lower() == nm.lower()):
                            out[cid] = str(ours)
        else:
            for cid, nm in self.names.items():
                out[cid] = nm.lower()
        if not out:
            raise DetectorError(f'class_map 이 모델 클래스와 하나도 맞지 않습니다: {self.names}')
        return out

    def warmup(self, width=640, height=480):
        import numpy as np
        self.infer(np.zeros((height, width, 3), dtype=np.uint8))

    def infer(self, image_bgr):
        results = self.model.predict(image_bgr, imgsz=self.imgsz, conf=self.conf, iou=self.iou,
                                     device=self.device, half=self.half, max_det=self.max_det,
                                     verbose=False)
        if not results:
            return []
        r = results[0]
        out = []
        if r.boxes is None or len(r.boxes) == 0:
            return out
        cls_ids = r.boxes.cls.tolist()
        confs = r.boxes.conf.tolist()
        boxes = r.boxes.xyxy.tolist()
        polys = list(r.masks.xy) if r.masks is not None else [None] * len(cls_ids)
        for cid, conf, box, poly in zip(cls_ids, confs, boxes, polys):
            ours = self.class_map.get(int(cid))
            if ours is None:
                continue
            x0, y0, x1, y1 = box
            if poly is None or len(poly) < 3:
                pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            else:
                pts = [(float(p[0]), float(p[1])) for p in poly]
            out.append(Instance(ours, float(conf), pts, (x0, y0, x1, y1)))
        return out
