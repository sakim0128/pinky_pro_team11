"""ROS-free image-to-map projection helpers for the overhead tracker."""
import numpy as np


def transform_point(homography, point):
    """Project an image point into map coordinates using a 3×3 homography."""
    vector = np.asarray([point[0], point[1], 1.0], dtype=float)
    projected = np.asarray(homography, dtype=float).reshape(3, 3) @ vector
    if not np.isfinite(projected).all() or abs(projected[2]) < 1e-9:
        return None
    return projected[0] / projected[2], projected[1] / projected[2]
