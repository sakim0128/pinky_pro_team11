"""천장 카메라 호모그래피 캘리브레이션 (ROS-free). 팀원 `overhead_tracker_node` 의
`image_to_map_homography` 9개 값을 만든다.

    corners = {40: (0.10, 0.10), 41: (2.25, 0.10), 42: (2.25, 1.15), 43: (0.10, 1.15)}   # id → map (m)
    H, report = calibrate(image_bgr, corners)          # 4장 이상 보여야 한다
    H.flatten().tolist()                                → overhead_tracker.yaml 에 붙여넣기

map 좌표는 줄자로 잰 마커 **중심** 이다 (map 원점 = 라이다 맵 yaml 의 origin 기준).
"""

import math

import numpy as np

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None


class CalibError(RuntimeError):
    pass


def detect_centers(image_bgr, dictionary='DICT_4X4_50'):
    """{id: (px, py)} — 각 마커 네 모서리의 평균."""
    if cv2 is None or not hasattr(cv2, 'aruco'):
        raise CalibError('python3-opencv (aruco 포함) 가 필요합니다')
    dict_id = getattr(cv2.aruco, dictionary, None)
    if dict_id is None:
        raise CalibError(f'알 수 없는 ArUco 사전: {dictionary}')
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(dict_id),
                                  cv2.aruco.DetectorParameters())
    gray = image_bgr if image_bgr.ndim == 2 else cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = det.detectMarkers(gray)
    out = {}
    if ids is None:
        return out
    for c, i in zip(corners, ids.flatten()):
        pts = c.reshape(4, 2)
        out[int(i)] = (float(pts[:, 0].mean()), float(pts[:, 1].mean()))
    return out


def homography_from_pairs(pairs):
    """[(px, py, mx, my), ...] (≥ 4) → 3×3 H (px → map). 재투영 오차(m) 도 함께."""
    if len(pairs) < 4:
        raise CalibError(f'대응점이 {len(pairs)}개 — 4개 이상 필요')
    src = np.array([[p[0], p[1]] for p in pairs], dtype=np.float64)
    dst = np.array([[p[2], p[3]] for p in pairs], dtype=np.float64)
    H, _ = cv2.findHomography(src, dst, 0)
    if H is None or not np.isfinite(H).all():
        raise CalibError('호모그래피 계산 실패 (마커가 한 직선 위?)')
    errs = []
    for (px, py), (mx, my) in zip(src, dst):
        v = H @ np.array([px, py, 1.0])
        errs.append(math.hypot(v[0] / v[2] - mx, v[1] / v[2] - my))
    return H, errs


def calibrate(image_bgr, corner_map, dictionary='DICT_4X4_50'):
    """corner_map: {id: (map_x, map_y)}. 반환 (H, report dict)."""
    centers = detect_centers(image_bgr, dictionary)
    seen = {i: centers[i] for i in corner_map if i in centers}
    missing = sorted(set(corner_map) - set(seen))
    if len(seen) < 4:
        raise CalibError(f'기준 마커 {len(seen)}/{len(corner_map)} 검출 — 안 보임: {missing}')
    pairs = [(seen[i][0], seen[i][1], corner_map[i][0], corner_map[i][1]) for i in sorted(seen)]
    H, errs = homography_from_pairs(pairs)
    return H, {'detected': {i: seen[i] for i in sorted(seen)}, 'missing': missing,
               'reproj_m': dict(zip(sorted(seen), errs)), 'max_reproj_m': max(errs),
               'other_ids': sorted(set(centers) - set(corner_map))}


def project(H, px, py):
    v = np.asarray(H, dtype=float).reshape(3, 3) @ np.array([px, py, 1.0])
    return float(v[0] / v[2]), float(v[1] / v[2])


def parse_corner_map(text):
    """"40:0.10,0.10;41:2.25,0.10;..." → {40: (0.10, 0.10), ...}"""
    out = {}
    for item in text.replace(' ', '').split(';'):
        if not item:
            continue
        i, xy = item.split(':')
        x, y = xy.split(',')
        out[int(i)] = (float(x), float(y))
    if len(out) < 4:
        raise CalibError('기준 마커는 4개 이상')
    return out


def yaml_line(H):
    vals = ', '.join(f'{v:.8g}' for v in np.asarray(H, dtype=float).reshape(-1))
    return f'image_to_map_homography: [{vals}]'
