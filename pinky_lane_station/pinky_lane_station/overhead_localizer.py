"""항공뷰(천장 카메라) → 핑키 map 위치 (D8). ROS 에 의존하지 않는다.

    cfg = OverheadConfig.load('overhead.yaml')
    loc = OverheadLocalizer(cfg)
    fixes = loc.update(img_bgr)     # {robot_name: Fix}

원리: 바닥 고정 기준 마커(map 좌표 기지) 의 네 모서리(px) ↔ map 좌표로 호모그래피 H 를 구한다.
"이미지에서 맵 크기 ↔ nav 맵 크기 비율" 이 아니라 H 라서 원근·배경·카메라 흔들림에 무관하다.
로봇 마커 네 모서리를 H 로 map 에 옮겨 중심(위치) 과 +x 축 방향(yaw) 을 얻는다.
기준 마커와 로봇 마커는 같은 높이에 있어야 한다 (다르면 연직점에서 멀수록 스케일 오차).
"""

import math
import os
from dataclasses import dataclass, field

import numpy as np
import yaml

from .marker_localizer import Fix, marker_object_points, rot_z

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None


@dataclass
class RobotMarker:
    name: str
    id: int
    size: float = 0.06
    yaw_offset: float = 0.0
    offset_x: float = 0.0


@dataclass
class OverheadConfig:
    dictionary: str = 'DICT_4X4_50'
    reference: dict = field(default_factory=dict)      # id -> (x, y, yaw)
    reference_size: float = 0.10
    reference_height: float = 0.10
    robots: dict = field(default_factory=dict)         # name -> RobotMarker
    station_latency: float = 0.15
    homography_alpha: float = 0.3
    min_reference: int = 4
    max_reproj: float = 4.0
    camera: dict = field(default_factory=lambda: {'device': 0, 'width': 1280, 'height': 720, 'fps': 15})

    @classmethod
    def from_dict(cls, d):
        ref = d.get('reference') or {}
        cfg = cls(
            dictionary=str(d.get('dictionary', 'DICT_4X4_50')),
            reference={int(m['id']): (float(m['x']), float(m['y']), float(m.get('yaw', 0.0)))
                       for m in ref.get('markers') or []},
            reference_size=float(ref.get('size', 0.10)),
            reference_height=float(ref.get('height', 0.0)),
            robots={str(r['name']): RobotMarker(str(r['name']), int(r['id']), float(r.get('size', 0.06)),
                                                float(r.get('yaw_offset', 0.0)), float(r.get('offset_x', 0.0)))
                    for r in d.get('robots') or []},
            station_latency=float(d.get('station_latency', 0.15)),
            homography_alpha=float(d.get('homography_alpha', 0.3)),
            min_reference=int(d.get('min_reference', 4)),
            max_reproj=float(d.get('max_reproj', 4.0)),
            camera=dict(d.get('camera') or {}),
        )
        if len(cfg.reference) < 4:
            raise ValueError('reference.markers 는 4개 이상이어야 합니다')
        ids = [m.id for m in cfg.robots.values()]
        if set(ids) & set(cfg.reference):
            raise ValueError('로봇 마커 id 와 기준 마커 id 가 겹칩니다')
        return cfg

    @classmethod
    def load(cls, path):
        with open(os.path.expanduser(path), encoding='utf-8') as fh:
            return cls.from_dict(yaml.safe_load(fh) or {})

    def reference_corners_map(self, mid):
        """기준 마커 네 모서리(TL TR BR BL) 의 map (x, y). 마커 +x = yaw 방향."""
        x, y, yaw = self.reference[mid]
        R = rot_z(yaw)[:2, :2]
        obj = marker_object_points(self.reference_size)[:, :2].astype(float)
        return (R @ obj.T).T + np.array([x, y])


class OverheadLocalizer:
    def __init__(self, cfg):
        if cv2 is None:
            raise RuntimeError('python3-opencv 가 필요합니다')
        self.cfg = cfg
        self.H = None                 # px -> map
        self.reproj = float('inf')    # 기준 마커 재투영 오차 (px)
        self.ref_seen = 0
        self.frames = 0
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg.dictionary))
        if hasattr(cv2.aruco, 'ArucoDetector'):
            p = cv2.aruco.DetectorParameters()
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._det = cv2.aruco.ArucoDetector(d, p)
            self._legacy = None
        else:
            self._det = None
            self._legacy = (d, cv2.aruco.DetectorParameters_create())
        self._by_id = {m.id: m for m in cfg.robots.values()}

    # ------------------------------------------------ 검출

    def detect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        if self._det is not None:
            corners, ids, _ = self._det.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._legacy[0], parameters=self._legacy[1])
        if ids is None:
            return {}
        return {int(i): np.asarray(c).reshape(4, 2).astype(np.float64)
                for i, c in zip(np.asarray(ids).reshape(-1), corners)}

    # ------------------------------------------------ 호모그래피

    def update_homography(self, dets):
        """기준 마커로 H(px→map) 를 갱신한다. 갱신됐으면 True."""
        src, dst = [], []
        seen = 0
        for mid, corners in dets.items():
            if mid in self.cfg.reference:
                seen += 1
                src.extend(corners.tolist())
                dst.extend(self.cfg.reference_corners_map(mid).tolist())
        self.ref_seen = seen
        if seen < self.cfg.min_reference:
            return False
        src_a = np.array(src, np.float64)
        dst_a = np.array(dst, np.float64)
        H, _ = cv2.findHomography(src_a, dst_a, cv2.RANSAC, 0.01)
        if H is None:
            return False
        H = H / H[2, 2]
        if self.H is None or self.cfg.homography_alpha >= 1.0:
            self.H = H
        else:
            a = self.cfg.homography_alpha
            self.H = (1 - a) * self.H + a * H
            self.H /= self.H[2, 2]
        # 재투영 오차: map → px
        back = cv2.perspectiveTransform(dst_a.reshape(-1, 1, 2), np.linalg.inv(self.H)).reshape(-1, 2)
        self.reproj = float(np.sqrt(np.mean(np.sum((back - src_a) ** 2, axis=1))))
        return True

    def px_to_map(self, pts):
        pts = np.asarray(pts, np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(pts, self.H).reshape(-1, 2)

    # ------------------------------------------------ 로봇 위치

    def robot_pose(self, corners, marker):
        """마커 네 모서리(px, TL TR BR BL) → base (x, y, yaw)."""
        m = self.px_to_map(corners)
        center = m.mean(axis=0)
        right = 0.5 * (m[1] + m[2]) - 0.5 * (m[0] + m[3])       # 마커 +x
        up = 0.5 * (m[0] + m[1]) - 0.5 * (m[3] + m[2])          # 마커 +y (= +x 를 +90° 돌린 것)
        a1 = math.atan2(right[1], right[0])
        a2 = math.atan2(up[1], up[0]) - math.pi / 2
        # 두 축의 원형 평균 — 한 변의 픽셀 양자화 오차를 반으로
        yaw = math.atan2(math.sin(a1) + math.sin(a2), math.cos(a1) + math.cos(a2)) - marker.yaw_offset
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        bx = center[0] - marker.offset_x * math.cos(yaw)
        by = center[1] - marker.offset_x * math.sin(yaw)
        return float(bx), float(by), float(yaw)

    def update(self, img):
        """{robot_name: Fix}. H 가 없거나 재투영 오차가 크면 빈 dict."""
        self.frames += 1
        dets = self.detect(img)
        self.update_homography(dets)
        if self.H is None or self.reproj > self.cfg.max_reproj:
            return {}
        out = {}
        for mid, corners in dets.items():
            marker = self._by_id.get(mid)
            if marker is None:
                continue
            x, y, yaw = self.robot_pose(corners, marker)
            out[marker.name] = Fix(x, y, yaw, mid, 0.0, self.reproj, self.ref_seen)
        return out

    def status(self):
        return {'homography': self.H is not None, 'reference_seen': self.ref_seen,
                'reproj_px': None if math.isinf(self.reproj) else round(self.reproj, 2),
                'frames': self.frames}
