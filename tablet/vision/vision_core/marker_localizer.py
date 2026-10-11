# -*- coding: utf-8 -*-
"""Marker localization primitives and data structures.

Reused from Team11 (sakim0128/pinky_pro_team11) to guarantee algorithmic identity.
No ROS dependency - runs in standard Python with NumPy and OpenCV.
"""

from dataclasses import dataclass
import math
import numpy as np


def rot_z(a: float) -> np.ndarray:
    """Rotation matrix around Z-axis."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def marker_object_points(size: float) -> np.ndarray:
    """Four corners of a square marker centered at origin on XY plane (Z=0).
    Order: Top-Left (-s, s), Top-Right (s, s), Bottom-Right (s, -s), Bottom-Left (-s, -s).
    """
    s = size / 2.0
    return np.array([[-s,  s, 0.0],
                     [ s,  s, 0.0],
                     [ s, -s, 0.0],
                     [-s, -s, 0.0]], dtype=np.float32)


@dataclass
class Fix:
    """Raw 2D pose fix from vision."""
    x: float
    y: float
    yaw: float
    marker_id: int
    range: float
    reproj: float
    n_markers: int


@dataclass
class RobotMarker:
    """Robot tracking marker configuration."""
    name: str
    id: int
    size: float = 0.06
    yaw_offset: float = 0.0
    offset_x: float = 0.0
