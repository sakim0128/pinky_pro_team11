# -*- coding: utf-8 -*-
"""Synthetic camera frame generator for overhead localizer testing.

Reused from Team11 (sakim0128/pinky_pro_team11).
Renders map plane with reference markers and robots at known poses,
then applies perspective warp to simulate an overhead camera.
Used by synthetic test fixtures without requiring physical hardware.
"""

import math
from typing import Dict, Optional, Tuple

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

from .marker_localizer import marker_object_points, rot_z


def _marker_image(dictionary_name: str, mid: int, px: int = 160) -> np.ndarray:
    """Generate ArUco marker bitmap."""
    dict_attr = getattr(cv2.aruco, dictionary_name)
    d = cv2.aruco.getPredefinedDictionary(dict_attr)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(d, mid, px)
    return cv2.aruco.drawMarker(d, mid, px)


def render_overhead(
    cfg,
    robot_poses: Dict[str, Tuple[float, float, float]],
    ppm: int = 400,
    margin: float = 0.3,
    view_H: Optional[np.ndarray] = None,
    out_size: Tuple[int, int] = (1280, 720),
    floor_val: int = 110,
) -> Tuple[np.ndarray, np.ndarray]:
    """Generate a synthetic overhead camera frame with reference and robot markers.

    Args:
        cfg: OverheadConfig instance
        robot_poses: {robot_name: (x, y, yaw)}
        ppm: pixels per meter on intermediate canvas
        margin: canvas border margin in meters
        view_H: 3x3 perspective warp matrix from canvas to output camera view
        out_size: (width, height) of output image
        floor_val: grayscale background color

    Returns:
        (view_H @ to_px, image_bgr)
    """
    if cv2 is None:
        raise RuntimeError("python3-opencv가 필요합니다")

    xs = [p[0] for p in cfg.reference.values()] + [p[0] for p in robot_poses.values()]
    ys = [p[1] for p in cfg.reference.values()] + [p[1] for p in robot_poses.values()]
    x0, y0 = min(xs) - margin, min(ys) - margin
    x1, y1 = max(xs) + margin, max(ys) + margin
    W, H = int((x1 - x0) * ppm), int((y1 - y0) * ppm)
    canvas = np.full((H, W, 3), floor_val, dtype=np.uint8)

    # map(m) -> canvas(px): Y axis inverted (image coordinate convention)
    to_px = np.array([
        [ ppm,  0.0, -x0 * ppm],
        [ 0.0, -ppm,  y1 * ppm],
        [ 0.0,  0.0,  1.0     ]
    ], dtype=np.float64)

    def draw(mid: int, x: float, y: float, yaw: float, size: float, white: float = 0.02):
        # Draw white quiet border then the marker inside
        for s_, col in ((size + 2.0 * white, 255), (size, None)):
            obj = marker_object_points(s_)[:, :2].astype(float)
            pts = (rot_z(yaw)[:2, :2] @ obj.T).T + np.array([x, y])
            px = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), to_px).reshape(-1, 2).astype(np.float32)
            if col is not None:
                cv2.fillPoly(canvas, [px.astype(np.int32)], (col, col, col))
                continue
            mk = _marker_image(cfg.dictionary, mid, 160)
            src = np.array([[0, 0], [160, 0], [160, 160], [0, 160]], dtype=np.float32)
            Hm, _ = cv2.findHomography(src, px)
            if Hm is None:
                continue
            warped = cv2.warpPerspective(mk, Hm, (W, H), flags=cv2.INTER_NEAREST, borderValue=255)
            mask = np.zeros((H, W), dtype=np.uint8)
            cv2.fillPoly(mask, [px.astype(np.int32)], 255)
            canvas[mask > 0] = np.repeat(warped[mask > 0][:, None], 3, axis=1)

    for mid, (x, y, yaw) in cfg.reference.items():
        draw(mid, x, y, yaw, cfg.reference_size)

    for name, (x, y, yaw) in robot_poses.items():
        if name not in cfg.robots:
            continue
        m = cfg.robots[name]
        cx = x + m.offset_x * math.cos(yaw)
        cy = y + m.offset_x * math.sin(yaw)
        draw(m.id, cx, cy, yaw + m.yaw_offset, m.size)

    if view_H is None:
        # Slight perspective tilt simulating a mounted overhead webcam
        ow, oh = out_size
        src = np.array([[0, 0], [W, 0], [W, H], [0, H]], dtype=np.float32)
        dst = np.array([[ow * 0.08, oh * 0.06],
                        [ow * 0.94, oh * 0.03],
                        [ow * 0.98, oh * 0.97],
                        [ow * 0.04, oh * 0.92]], dtype=np.float32)
        view_H, _ = cv2.findHomography(src, dst)

    img = cv2.warpPerspective(canvas, view_H, out_size, borderValue=(floor_val, floor_val, floor_val))
    return view_H @ to_px, img
