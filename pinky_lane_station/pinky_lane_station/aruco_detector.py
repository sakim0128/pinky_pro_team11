"""도착 지점 벽 ArUco 마커 검출 — 영상 처리 (YOLO 아님). ROS 에 의존하지 않는다.

비전 미션의 도착 지점(1·2·3번) 앞 벽에 같은 번호의 ArUco 마커를 카메라 높이로 붙인다. 로봇 카메라 영상에서 마커를 찾고
한 변 픽셀 길이로 카메라~마커 거리를 잰다 (핀홀: 거리 = 초점거리(px) × 마커 한 변(m) / 한 변(px)).
벽 마커는 카메라를 정면으로 보므로 한 변 길이만으로 충분하다 — 카메라 보정 행렬이 없어도 된다.

    det = ArucoMarkerDetector(ArucoParams())
    r = det.update(bgr_image)          # ArucoResult(markers={id: 거리 m}, raw={id: 거리 m}, corners={id: 4×2})

로봇은 교차로를 통과한 **뒤에만** 목적지 id 의 거리를 본다 — lane_driver 몫.

초점거리 맞추기 (현장, 한 번): 마커를 카메라 정면 d m 에 두고 lane_debug 오버레이의 한 변 px 를 읽어
    focal_px = side_px × d / marker_size       (ref_width 기준 폭으로 환산된 값을 적는다)
"""

from dataclasses import dataclass, field

import numpy as np

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None


@dataclass
class ArucoParams:
    enabled: bool = True
    dictionary: str = 'DICT_4X4_50'
    marker_size: float = 0.05         # 마커 검은 테두리 포함 한 변 (m)
    focal_px: float = 500.0           # ref_width 폭 영상 기준 초점거리 (px). 현장에서 한 번 잰다 (모듈 설명)
    ref_width: int = 640              # focal_px 를 잰 영상 폭. 다른 폭이면 비례로 환산
    ids: list = field(default_factory=lambda: [1, 2, 3])   # 이 id 만 쓴다 (빈 목록이면 전부)
    max_distance: float = 1.5         # 이보다 먼 추정은 버린다 (m)
    confirm: int = 2                  # 연속 프레임 수 — 확정
    release: int = 3                  # 안 보인 프레임 수 — 해제


@dataclass
class ArucoResult:
    markers: dict = field(default_factory=dict)    # 확정 {id: 거리 m}
    raw: dict = field(default_factory=dict)        # 이번 프레임 {id: 거리 m}
    corners: dict = field(default_factory=dict)    # 이번 프레임 {id: np.ndarray 4×2} (오버레이용)
    side_px: dict = field(default_factory=dict)    # 이번 프레임 {id: 한 변 평균 px}


def params_from_dict(d):
    d = dict(d or {})
    fields = ArucoParams.__dataclass_fields__
    kw = {k: v for k, v in d.items() if k in fields}
    if 'ids' in kw:
        kw['ids'] = [int(x) for x in (kw['ids'] or [])]
    return ArucoParams(**kw)


def side_length_px(corners):
    """마커 네 변 길이의 평균 (px)."""
    c = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    return float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))


def distance_from_side(side_px, image_width, p):
    """한 변 px → 카메라~마커 거리 (m). 계산 불가면 None."""
    if side_px <= 1.0 or image_width <= 0 or p.ref_width <= 0:
        return None
    focal = p.focal_px * float(image_width) / float(p.ref_width)
    return focal * p.marker_size / side_px


def _make_detector(p):
    if cv2 is None or not hasattr(cv2, 'aruco'):
        return None
    aruco = cv2.aruco
    dict_id = getattr(aruco, p.dictionary, None)
    if dict_id is None:
        raise ValueError(f'알 수 없는 ArUco 사전 {p.dictionary!r}')
    dictionary = aruco.getPredefinedDictionary(dict_id)
    if hasattr(aruco, 'ArucoDetector'):                               # OpenCV ≥ 4.7
        det = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        return lambda gray: det.detectMarkers(gray)[:2]
    params = aruco.DetectorParameters_create()                        # pragma: no cover — 옛 OpenCV
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


class ArucoMarkerDetector:
    def __init__(self, params=None):
        self.p = params or ArucoParams()
        self._detect = _make_detector(self.p) if self.p.enabled else None
        self._hits = {}          # id -> 연속 본 프레임
        self._miss = {}          # id -> 연속 못 본 프레임 (확정된 것만)
        self._last = {}          # id -> 마지막 거리
        self._confirmed = set()

    @property
    def available(self):
        return self._detect is not None

    def detect_raw(self, bgr):
        """이번 프레임의 ({id: 거리}, {id: corners}, {id: side_px})."""
        if self._detect is None or bgr is None:
            return {}, {}, {}
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        corners, ids = self._detect(gray)
        out, cs, sides = {}, {}, {}
        if ids is None:
            return out, cs, sides
        W = gray.shape[1]
        for c, i in zip(corners, np.asarray(ids).reshape(-1)):
            mid = int(i)
            if self.p.ids and mid not in self.p.ids:
                continue
            side = side_length_px(c)
            dist = distance_from_side(side, W, self.p)
            if dist is None or dist > self.p.max_distance:
                continue
            if mid not in out or dist < out[mid]:
                out[mid], cs[mid], sides[mid] = dist, np.asarray(c).reshape(4, 2), side
        return out, cs, sides

    def update(self, bgr):
        raw, corners, sides = self.detect_raw(bgr)
        return self.update_from_raw(raw, corners, sides)

    def update_from_raw(self, raw, corners=None, sides=None):
        """디바운스: confirm 프레임 연속이면 확정, 확정 뒤 release 프레임 못 보면 해제 (그동안 마지막 거리 유지)."""
        p = self.p
        for mid in set(self._hits) | set(raw) | self._confirmed:
            if mid in raw:
                self._hits[mid] = self._hits.get(mid, 0) + 1
                self._miss[mid] = 0
                self._last[mid] = raw[mid]
                if self._hits[mid] >= p.confirm:
                    self._confirmed.add(mid)
            else:
                self._hits[mid] = 0
                if mid in self._confirmed:
                    self._miss[mid] = self._miss.get(mid, 0) + 1
                    if self._miss[mid] >= p.release:
                        self._confirmed.discard(mid)
        markers = {mid: self._last[mid] for mid in sorted(self._confirmed)}
        return ArucoResult(markers=markers, raw=dict(raw), corners=dict(corners or {}), side_px=dict(sides or {}))
