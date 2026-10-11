# -*- coding: utf-8 -*-
"""Overhead ArUco localizer with homography-based map projection.

Directly adapted from Team11 (sakim0128/pinky_pro_team11) to eliminate algorithm drift.
Calculates planar homography H from 4 reference ground/pedestal markers to map coordinates,
then maps robot top markers to base (x, y, yaw).
"""

from dataclasses import dataclass, field
import math
import os
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

from .marker_localizer import Fix, RobotMarker, marker_object_points, rot_z


@dataclass
class OverheadConfig:
    """Overhead camera and marker layout configuration."""
    dictionary: str = "DICT_4X4_50"
    reference: Dict[int, Tuple[float, float, float]] = field(default_factory=dict)  # id -> (x, y, yaw)
    reference_size: float = 0.10
    reference_height: float = 0.10
    robots: Dict[str, RobotMarker] = field(default_factory=dict)  # name -> RobotMarker
    station_latency: float = 0.15
    homography_alpha: float = 0.3
    min_reference: int = 4
    max_reproj: float = 4.0
    camera: dict = field(default_factory=lambda: {"device": 0, "width": 1280, "height": 720, "fps": 15})

    @classmethod
    def from_dict(cls, d: dict) -> "OverheadConfig":
        ref = d.get("reference") or {}
        robots_dict = {}
        for r in d.get("robots") or []:
            name = str(r["name"])
            robots_dict[name] = RobotMarker(
                name=name,
                id=int(r["id"]),
                size=float(r.get("size", 0.06)),
                yaw_offset=float(r.get("yaw_offset", 0.0)),
                offset_x=float(r.get("offset_x", 0.0)),
            )

        ref_markers = {
            int(m["id"]): (float(m["x"]), float(m["y"]), float(m.get("yaw", 0.0)))
            for m in ref.get("markers") or []
        }

        ref_size = d.get("reference_size")
        if ref_size is None:
            ref_size = ref.get("size", 0.10)

        ref_height = d.get("reference_height")
        if ref_height is None:
            ref_height = ref.get("height", 0.10)

        cfg = cls(
            dictionary=str(d.get("dictionary", "DICT_4X4_50")),
            reference=ref_markers,
            reference_size=float(ref_size),
            reference_height=float(ref_height),
            robots=robots_dict,
            station_latency=float(d.get("station_latency", 0.15)),
            homography_alpha=float(d.get("homography_alpha", 0.3)),
            min_reference=int(d.get("min_reference", 4)),
            max_reproj=float(d.get("max_reproj", 4.0)),
            camera=dict(d.get("camera") or {}),
        )

        if len(cfg.reference) < cfg.min_reference:
            raise ValueError(f"reference.markers 는 {cfg.min_reference}개 이상이어야 합니다 (현재 {len(cfg.reference)}개)")

        robot_ids = [m.id for m in cfg.robots.values()]
        overlap = set(robot_ids) & set(cfg.reference.keys())
        if overlap:
            raise ValueError(f"로봇 마커 id와 기준 마커 id가 겹칩니다: {overlap}")

        return cfg

    @classmethod
    def load(cls, path: str) -> "OverheadConfig":
        with open(os.path.expanduser(path), "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
            return cls.from_dict(data)

    def reference_corners_map(self, mid: int) -> np.ndarray:
        """Map frame (x, y) coordinates of reference marker corners (TL, TR, BR, BL)."""
        x, y, yaw = self.reference[mid]
        R = rot_z(yaw)[:2, :2]
        obj = marker_object_points(self.reference_size)[:, :2].astype(float)
        return (R @ obj.T).T + np.array([x, y], dtype=float)


class OverheadLocalizer:
    """Computes robot poses on map coordinate plane via homography."""

    def __init__(self, cfg: OverheadConfig):
        if cv2 is None:
            raise RuntimeError("python3-opencv가 필요합니다")
        self.cfg = cfg
        self.H: Optional[np.ndarray] = None  # px -> map homography matrix
        self.reproj: float = float("inf")    # reprojection error (px)
        self.ref_seen: int = 0
        self.frames: int = 0
        self.last_ids: List[int] = []        # marker ids detected in the latest frame

        # Setup ArUco detector (handles both OpenCV 4.7+ and legacy)
        dict_attr = getattr(cv2.aruco, cfg.dictionary, None)
        if dict_attr is None:
            raise ValueError(f"알 수 없는 ArUco 딕셔너리: {cfg.dictionary}")
        d = cv2.aruco.getPredefinedDictionary(dict_attr)
        if hasattr(cv2.aruco, "ArucoDetector"):
            p = cv2.aruco.DetectorParameters()
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._det = cv2.aruco.ArucoDetector(d, p)
            self._legacy = None
        else:
            self._det = None
            self._legacy = (d, cv2.aruco.DetectorParameters_create())

        self._by_id = {m.id: m for m in cfg.robots.values()}

    def detect(self, img: np.ndarray) -> Dict[int, np.ndarray]:
        """Detect all ArUco markers in image. Returns {marker_id: corners_array(4, 2)}."""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        if self._det is not None:
            corners, ids, _ = self._det.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._legacy[0], parameters=self._legacy[1])
        if ids is None:
            return {}
        return {
            int(i): np.asarray(c).reshape(4, 2).astype(np.float64)
            for i, c in zip(np.asarray(ids).reshape(-1), corners)
        }

    def update_homography(self, dets: Dict[int, np.ndarray]) -> bool:
        """Update homography H (px -> map) using visible reference markers.
        Returns True if H was updated, False otherwise.
        """
        src, dst = [], []
        seen = 0
        for mid, corners in dets.items():
            if mid in self.cfg.reference:
                seen += 1
                src.extend(corners.tolist())
                dst.extend(self.cfg.reference_corners_map(mid).tolist())
        self.ref_seen = seen

        # If reference markers are fewer than minimum, retain last valid H
        if seen < self.cfg.min_reference:
            return False

        src_a = np.array(src, dtype=np.float64)
        dst_a = np.array(dst, dtype=np.float64)
        H, _ = cv2.findHomography(src_a, dst_a, cv2.RANSAC, 0.01)
        if H is None:
            return False

        H = H / H[2, 2]
        if self.H is None or self.cfg.homography_alpha >= 1.0:
            self.H = H
        else:
            a = self.cfg.homography_alpha
            self.H = (1.0 - a) * self.H + a * H
            self.H /= self.H[2, 2]

        # Reprojection error: map -> px
        try:
            inv_H = np.linalg.inv(self.H)
            back = cv2.perspectiveTransform(dst_a.reshape(-1, 1, 2), inv_H).reshape(-1, 2)
            self.reproj = float(np.sqrt(np.mean(np.sum((back - src_a) ** 2, axis=1))))
        except Exception:
            self.reproj = float("inf")

        return True

    def px_to_map(self, pts: np.ndarray) -> np.ndarray:
        """Transform pixel points to map coordinates using current homography."""
        if self.H is None:
            raise RuntimeError("Homography가 아직 초기화되지 않았습니다")
        pts_arr = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(pts_arr, self.H).reshape(-1, 2)

    def robot_pose(self, corners: np.ndarray, marker: RobotMarker) -> Tuple[float, float, float]:
        """Compute base pose (x, y, yaw) from 4 detected marker corners (TL, TR, BR, BL)."""
        m = self.px_to_map(corners)
        center = m.mean(axis=0)

        # Compute marker coordinate axes in map frame
        right = 0.5 * (m[1] + m[2]) - 0.5 * (m[0] + m[3])  # marker +X axis
        up = 0.5 * (m[0] + m[1]) - 0.5 * (m[3] + m[2])     # marker +Y axis

        a1 = math.atan2(right[1], right[0])
        a2 = math.atan2(up[1], up[0]) - (math.pi / 2.0)

        # Circular mean of both orthogonal axes to halve quantization error
        yaw = math.atan2(math.sin(a1) + math.sin(a2), math.cos(a1) + math.cos(a2)) - marker.yaw_offset
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        # Apply robot center offset if marker is not positioned directly at wheel axle
        bx = center[0] - marker.offset_x * math.cos(yaw)
        by = center[1] - marker.offset_x * math.sin(yaw)
        return float(bx), float(by), float(yaw)

    def update(self, img: np.ndarray) -> Dict[str, Fix]:
        """Process an image frame.
        Returns:
            Dict of {robot_name: Fix}. Empty if H is not available or reprojection exceeds max_reproj.
        """
        self.frames += 1
        dets = self.detect(img)
        self.last_ids = sorted(int(i) for i in dets)
        self.update_homography(dets)

        if self.H is None or self.reproj > self.cfg.max_reproj:
            return {}

        out = {}
        for mid, corners in dets.items():
            marker = self._by_id.get(mid)
            if marker is None:
                continue
            x, y, yaw = self.robot_pose(corners, marker)
            out[marker.name] = Fix(
                x=x,
                y=y,
                yaw=yaw,
                marker_id=mid,
                range=0.0,
                reproj=self.reproj,
                n_markers=self.ref_seen,
            )
        return out

    def status(self) -> Dict[str, Any]:
        """Current internal status of the localizer."""
        return {
            "homography": self.H is not None,
            "reference_seen": self.ref_seen,
            "reproj_px": None if math.isinf(self.reproj) else round(self.reproj, 2),
            "frames": self.frames,
        }
