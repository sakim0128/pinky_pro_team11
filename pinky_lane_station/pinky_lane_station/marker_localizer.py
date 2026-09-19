"""바닥 ArUco 마커 → 로봇 map 위치 (D7). ROS 에 의존하지 않는다.

    loc = MarkerLocalizer(intr, extr, marker_map)
    fix = loc.localize(img_bgr)        # Fix(x, y, yaw, marker_id, range, reproj, n) 또는 None

프레임 규약
  robot(base_footprint): x 전방, y 좌측, z 상방.  카메라 광학: z 전방(시선), x 우측, y 아래.
  마커: 인쇄면 중심 원점, x 는 인쇄물의 오른쪽, y 는 위쪽, z 는 인쇄면에서 솟는 방향 (바닥 마커 = 상방).
  markers.yaml 의 yaw = 마커 +x 가 가리키는 map 방향.
  T_map_base = T_map_marker · inv(T_cam_marker) · inv(T_base_cam)
"""

import math
import os
from dataclasses import dataclass, field

import numpy as np
import yaml

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None


# ------------------------------------------------------------------ SE(3) 유틸

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def inv_T(T):
    R = T[:3, :3]
    t = T[:3, 3]
    return make_T(R.T, -R.T @ t)


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


# 광학 프레임 축을 로봇 프레임으로: x_opt→-y, y_opt→-z, z_opt→+x  (열 = 광학 축의 로봇 표현)
R_BASE_OPT0 = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)


# ------------------------------------------------------------------ 설정

@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    dist: list = field(default_factory=lambda: [0.0] * 5)
    width: int = 640
    height: int = 480
    calibrated: bool = False

    @property
    def K(self):
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=float)

    @property
    def D(self):
        return np.array(self.dist, dtype=float).reshape(-1, 1)

    @classmethod
    def load(cls, path):
        with open(os.path.expanduser(path), encoding='utf-8') as fh:
            d = yaml.safe_load(fh) or {}
        return cls(float(d['fx']), float(d['fy']), float(d['cx']), float(d['cy']),
                   [float(v) for v in d.get('dist', [0] * 5)], int(d.get('width', 640)),
                   int(d.get('height', 480)), bool(d.get('calibrated', False)))

    @classmethod
    def from_camera_model(cls, cam):
        """synthetic_camera.CameraModel 과 같은 핀홀."""
        return cls(cam.fx, cam.fy, cam.cx, cam.cy, [0.0] * 5, cam.W, cam.H, True)

    def to_dict(self):
        return {'calibrated': self.calibrated, 'width': self.width, 'height': self.height,
                'fx': self.fx, 'fy': self.fy, 'cx': self.cx, 'cy': self.cy,
                'dist': [float(v) for v in self.dist]}


@dataclass
class CameraExtrinsics:
    """base_footprint → 카메라 광학 프레임."""
    x: float = 0.06
    y: float = 0.0
    z: float = 0.10
    roll: float = 0.0
    pitch: float = math.radians(25.0)       # 양수 = 아래를 본다
    yaw: float = 0.0
    calibrated: bool = False

    @property
    def T_base_cam(self):
        R = rot_z(self.yaw) @ rot_y(self.pitch) @ rot_x(self.roll) @ R_BASE_OPT0
        return make_T(R, (self.x, self.y, self.z))

    @classmethod
    def load(cls, path):
        with open(os.path.expanduser(path), encoding='utf-8') as fh:
            d = yaml.safe_load(fh) or {}
        return cls(float(d.get('x', 0.06)), float(d.get('y', 0.0)), float(d.get('z', 0.10)),
                   math.radians(float(d.get('roll_deg', 0.0))),
                   math.radians(float(d.get('pitch_deg', 25.0))),
                   math.radians(float(d.get('yaw_deg', 0.0))), bool(d.get('calibrated', False)))

    @classmethod
    def from_camera_model(cls, cam):
        return cls(cam.forward, 0.0, cam.h, 0.0, cam.pitch, 0.0, True)

    def to_dict(self):
        return {'calibrated': self.calibrated, 'x': self.x, 'y': self.y, 'z': self.z,
                'roll_deg': math.degrees(self.roll), 'pitch_deg': math.degrees(self.pitch),
                'yaw_deg': math.degrees(self.yaw)}


@dataclass
class MarkerMap:
    dictionary: str = 'DICT_4X4_50'
    size: float = 0.08
    white_border: float = 0.015
    poses: dict = field(default_factory=dict)      # id -> (x, y, yaw)

    @classmethod
    def load(cls, path, graph=None):
        with open(os.path.expanduser(path), encoding='utf-8') as fh:
            d = yaml.safe_load(fh) or {}
        m = cls(str(d.get('dictionary', 'DICT_4X4_50')), float(d.get('size', 0.08)),
                float(d.get('white_border', 0.015)))
        for raw in d.get('markers') or []:
            mid = int(raw['id'])
            yaw = float(raw.get('yaw', 0.0))
            if 'node' in raw:
                if graph is None or raw['node'] not in graph.nodes:
                    raise ValueError(f"마커 {mid}: 노드 {raw['node']!r} 를 그래프에서 찾을 수 없습니다")
                n = graph.nodes[raw['node']]
                m.poses[mid] = (n.x, n.y, yaw)
            else:
                m.poses[mid] = (float(raw['x']), float(raw['y']), yaw)
        return m

    def T_map_marker(self, mid):
        x, y, yaw = self.poses[mid]
        return make_T(rot_z(yaw), (x, y, 0.0))


# ------------------------------------------------------------------ 검출·위치

@dataclass
class MarkerObs:
    id: int
    corners: np.ndarray          # (4, 2) px  TL TR BR BL
    T_cam_marker: np.ndarray
    range: float
    reproj: float


@dataclass
class Fix:
    x: float
    y: float
    yaw: float
    marker_id: int
    range: float
    reproj: float
    n_markers: int


def marker_object_points(size):
    s = size / 2.0
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)


class MarkerLocalizer:
    def __init__(self, intrinsics, extrinsics, marker_map, min_range=0.12, max_range=0.9,
                 max_reproj=3.0):
        if cv2 is None:
            raise RuntimeError('python3-opencv 가 필요합니다')
        self.intr = intrinsics
        self.extr = extrinsics
        self.map = marker_map
        self.min_range, self.max_range, self.max_reproj = min_range, max_range, max_reproj
        self._obj = marker_object_points(marker_map.size)
        self._T_cam_base = inv_T(extrinsics.T_base_cam)
        dict_id = getattr(cv2.aruco, marker_map.dictionary)
        self._dict = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detector = cv2.aruco.ArucoDetector(self._dict, params)
        else:                                       # OpenCV < 4.7
            self._detector = None
            self._params = cv2.aruco.DetectorParameters_create()

    def _detect_raw(self, gray):
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        if ids is None:
            return []
        ids = np.asarray(ids).reshape(-1)
        return [(int(i), np.asarray(c).reshape(4, 2).astype(np.float32)) for i, c in zip(ids, corners)]

    def detect(self, img_bgr):
        """알려진 마커만 [MarkerObs]. 거리·재투영 게이트 통과한 것만."""
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY) if img_bgr.ndim == 3 else img_bgr
        out = []
        for mid, corners in self._detect_raw(gray):
            if mid not in self.map.poses:
                continue
            ok, rvec, tvec = cv2.solvePnP(self._obj, corners, self.intr.K, self.intr.D,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            proj, _ = cv2.projectPoints(self._obj, rvec, tvec, self.intr.K, self.intr.D)
            reproj = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - corners) ** 2, axis=1))))
            rng = float(np.linalg.norm(tvec))
            if not (self.min_range <= rng <= self.max_range) or reproj > self.max_reproj:
                continue
            R, _ = cv2.Rodrigues(rvec)
            out.append(MarkerObs(mid, corners, make_T(R, tvec), rng, reproj))
        return out

    def pose_from(self, obs):
        T_map_base = self.map.T_map_marker(obs.id) @ inv_T(obs.T_cam_marker) @ self._T_cam_base
        return float(T_map_base[0, 3]), float(T_map_base[1, 3]), yaw_of(T_map_base)

    def localize(self, img_bgr):
        obs = self.detect(img_bgr)
        if not obs:
            return None
        best = min(obs, key=lambda o: o.range)          # 가까운 마커가 가장 정확하다
        x, y, yaw = self.pose_from(best)
        return Fix(x, y, yaw, best.id, best.range, best.reproj, len(obs))


def marker_corners_map(marker_map, mid):
    """마커 네 모서리(TL TR BR BL, 마커 프레임 순서)의 map 좌표 (4, 3). 합성 렌더용."""
    T = marker_map.T_map_marker(mid)
    pts = marker_object_points(marker_map.size).astype(float)
    return (T[:3, :3] @ pts.T).T + T[:3, 3]
