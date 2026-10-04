"""바닥 좌표(BEV) 차선 중앙 경로 + pure pursuit 목표점. ROS 에 의존하지 않는다.

    cam = GroundCamera.from_yaml('config/pinky_cam.yaml')     # 렌즈 + 바닥 캘리브레이션 (scripts/calib_lens.py · calib_floor.py)
    bev = BevLaneFollower(cam, BevParams(enabled=True))      # 로봇마다 하나 (직전 회전 방향을 기억한다)
    r = bev.update(lane_mask, image_bgr)                     # -> BevResult
    r.robot_target_m()                                       # (전방, 왼쪽) m — 카메라 바로 아래 바닥 기준

왜 화면이 아니라 바닥인가: 화면에선 차선 폭이 거리마다 달라지고, 카메라(바닥 6 cm 위)는 앞 10~20 cm 에서 좌우 ±10 cm 만 보여
차선(약 17 cm)이 한 줄만 보이는 게 보통이다. 바닥에선 차선 폭이 일정하므로 선 하나로도 중앙을 안다.

  1. 차선 마스크 픽셀 → 바닥 점 (mm). 흰 테이프 마스크(tape_mask)와 겹치면 테이프 픽셀로 깎는다 (세그 마스크가 약간 넓다)
  2. 5 mm 격자로 래스터 → 조각마다 골격선 → 한쪽 끝에서 다른 끝까지 순서 있는 폴리라인
  3. 좌/우 라벨을 쓰지 않는다: 로봇에서 가까운 점부터 앞으로 더 멀어지는 쪽이 진행 방향, 로봇이 있는 쪽이 도로 쪽
     (선이 로봇 앞을 가로질러 애매하면 직전 회전 방향 hint 로 정한다)
  4. 진행 방향에 수직으로 차선 반폭만큼 도로 쪽으로 옮긴 선 = 차로 중앙 경로. 직선·커브·선 하나·둘 모두 같다
  5. 구동축에서 lookahead 떨어진 경로 위 점 = 목표점. 곡률 = 2·x / Ld² (pure pursuit, 구동축 기준)

이 모듈의 좌표: 원점 = 카메라 바로 아래 바닥, x 오른쪽, y 전방, mm (scripts/bev.py 와 같다). 곡률 + = 우회전.
로봇(lane_memory) 좌표는 x 전방, y 왼쪽, m — robot_target_m() 이 바꾼다. 구동축 오프셋은 로봇이 더한다.
"""

from dataclasses import dataclass, field

import numpy as np
import yaml

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None

try:
    from skimage.morphology import skeletonize as _sk_skeletonize
except ImportError:              # pragma: no cover - 관제 PC 에 scikit-image 가 없으면 아래 Zhang-Suen
    _sk_skeletonize = None

from .lane_target import QUALITY_BOTH, QUALITY_LOST, QUALITY_SINGLE

LEFT, RIGHT = 'L', 'R'


@dataclass
class BevParams:
    enabled: bool = False
    calib: str = 'pinky_cam.yaml'       # 렌즈·바닥 캘리브레이션. 상대경로면 detector yaml 이 있는 디렉터리 기준
    lookahead_mm: float = 250.0         # 구동축에서 목표점까지 (pure pursuit L_d)
    lane_width_mm: float = 174.0        # 테이프 중심 ~ 테이프 중심 (테이프 사이 152 mm + 테이프 21 mm, 직선 7 프레임 실측)
    axle_mm: float = 33.0               # 구동축 → 카메라 (pinky_pro URDF: 바퀴 joint x=0, front_camera_link x≈33 mm)
    max_range_mm: float = 700.0         # 이보다 먼 바닥은 안 쓴다 (1 m 에서 한 행 ≈ 3 cm — 너무 거칠다)
    cell_mm: float = 5.0                # 골격선 격자
    min_piece_mm: float = 60.0          # 이보다 짧은 조각은 버린다
    max_pieces: int = 3
    tape_trim: bool = True              # 세그 마스크를 흰 테이프 픽셀로 깎는다
    tape_top_frac: float = 0.40         # tape_mask: 이 행 위는 안 본다 (벽)
    tape_threshold: int = 60            # tape_mask: L 채널 top-hat 문턱
    hint_curvature: float = 1.0 / 2000.0   # |곡률| (1/mm) 이 이 이상이면 회전 방향을 기억한다 (애매한 가로 선 판정용)
    min_mask_px: int = 50


@dataclass
class BevResult:
    mode: str = 'none'                  # both | both(disagree) | left_only | right_only | none
    target: np.ndarray = None           # (x 오른쪽, y 전방) mm, 카메라 바로 아래 바닥 기준. 없으면 None
    curvature: float = None             # 1/mm, + = 우회전 (구동축 기준 pure pursuit)
    path: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))   # 차로 중앙 경로 (N, 2) mm, 로봇에서 멀어지는 순
    lanes: dict = field(default_factory=dict)     # {'L'|'R': 차선 폴리라인 (N, 2) mm}  — 도로가 오른쪽이면 'L'
    ambiguous: bool = False

    @property
    def valid(self):
        return self.target is not None

    def robot_target_m(self):
        """목표점을 로봇 규약으로: (전방 m, 왼쪽 m). 원점은 카메라 바로 아래 바닥 (구동축 오프셋 미포함)."""
        if self.target is None:
            return None
        return float(self.target[1]) / 1000.0, -float(self.target[0]) / 1000.0

    def robot_curvature(self):
        """곡률을 로봇 규약으로: 1/m, + = 좌회전."""
        return None if self.curvature is None else -float(self.curvature) * 1000.0


class GroundCamera:
    """카메라 픽셀 ↔ 바닥 mm. 평평한 바닥, 카메라는 바닥에서 camera_height_mm 위 (원점 바로 위)."""

    def __init__(self, K, dist, R_floor_to_cam, camera_height_mm, image_size=(640, 480)):
        self.K = np.asarray(K, np.float64)
        self.D = np.asarray(dist, np.float64)
        self.R = np.asarray(R_floor_to_cam, np.float64)
        self.h = float(camera_height_mm)
        self.t = -self.R @ np.array([0.0, 0.0, self.h])
        self.image_size = (int(image_size[0]), int(image_size[1]))
        self._rvec = cv2.Rodrigues(self.R)[0]

    @classmethod
    def from_yaml(cls, path):
        with open(path, encoding='utf-8') as fh:
            c = yaml.safe_load(fh)
        missing = [k for k in ('K', 'dist', 'R_floor_to_cam', 'camera_height_mm') if k not in c]
        if missing:
            raise ValueError(f'{path}: 바닥 캘리브레이션 항목이 없다 {missing} (scripts/calib_floor.py)')
        return cls(c['K'], c['dist'], c['R_floor_to_cam'], c['camera_height_mm'], c.get('image_size', (640, 480)))

    def pixels_to_floor(self, uv):
        """-> (xy mm (N, 2), valid). 지평선 위 픽셀은 바닥과 안 만나서 무효."""
        uv = np.asarray(uv, np.float64).reshape(-1, 1, 2)
        n = cv2.undistortPoints(uv, self.K, self.D).reshape(-1, 2)
        d = (self.R.T @ np.c_[n, np.ones(len(n))].T).T
        with np.errstate(divide='ignore', invalid='ignore'):
            s = -self.h / d[:, 2]
        return (d * s[:, None])[:, :2], s > 0

    def floor_to_pixels(self, xy):
        xy = np.asarray(xy, np.float64).reshape(-1, 2)
        uv, _ = cv2.projectPoints(np.c_[xy, np.zeros(len(xy))], self._rvec, self.t, self.K, self.D)
        return uv.reshape(-1, 2)

    def in_front(self, xy, min_depth_mm=1.0):
        """카메라 앞(깊이 > 0)에 있는 바닥 점인가 — 뒤쪽 점은 투영하면 거울상 쓰레기가 된다."""
        xy = np.asarray(xy, np.float64).reshape(-1, 2)
        z = (np.c_[xy, np.zeros(len(xy))] @ self.R.T + self.t)[:, 2]
        return z > min_depth_mm


# ---------------------------------------------------------------- 마스크

def tape_mask(img, top_frac=0.40, threshold=60):
    """밝고 가는 흰 테이프 픽셀 (L 채널 top-hat). 넓게 밝은 벽·바닥 반사는 top-hat 이 지운다."""
    h = img.shape[0]
    y0 = int(top_frac * h)
    L = cv2.cvtColor(img[y0:], cv2.COLOR_BGR2LAB)[..., 0]
    th = cv2.morphologyEx(cv2.medianBlur(L, 9), cv2.MORPH_TOPHAT,
                          cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (91, 91)))
    m = (th > threshold).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    out = np.zeros(img.shape[:2], bool)
    out[y0:] = m.astype(bool)
    return out


def lane_mask_from_instances(instances, shape, classes):
    """검출 인스턴스 중 차선 클래스 폴리곤을 칠한 마스크 (H, W) bool. 좌/우 라벨은 합친다."""
    m = np.zeros(shape[:2], np.uint8)
    for inst in instances:
        if inst.cls in classes and len(inst.polygon) >= 3:
            cv2.fillPoly(m, [np.round(np.asarray(inst.polygon)).astype(np.int32)], 1)
    return m.astype(bool)


# ---------------------------------------------------------------- 골격선

def _zhang_suen(img):
    """scikit-image 가 없을 때의 골격선 (Zhang-Suen thinning). img: bool (H, W)."""
    a = np.pad(img.astype(np.uint8), 1)
    while True:
        changed = False
        for step in (0, 1):
            p2, p3, p4 = a[:-2, 1:-1], a[:-2, 2:], a[1:-1, 2:]
            p5, p6, p7 = a[2:, 2:], a[2:, 1:-1], a[2:, :-2]
            p8, p9 = a[1:-1, :-2], a[:-2, :-2]
            nb = [p2, p3, p4, p5, p6, p7, p8, p9]
            b = sum(x.astype(np.int32) for x in nb)
            seq = nb + [p2]
            t = sum(((seq[k] == 0) & (seq[k + 1] == 1)).astype(np.int32) for k in range(8))
            c = a[1:-1, 1:-1] == 1
            if step == 0:
                cond = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                cond = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            rm = c & (b >= 2) & (b <= 6) & (t == 1) & cond
            if rm.any():
                a[1:-1, 1:-1][rm] = 0
                changed = True
        if not changed:
            return a[1:-1, 1:-1].astype(bool)


def skeletonize(img):
    if _sk_skeletonize is not None:
        return _sk_skeletonize(img.astype(bool))
    return _zhang_suen(img)


def _walk(pts, start, max_step):
    """골격선 점들을 start 에서 가장 가까운 이웃으로 계속 따라간다 (U턴도 따라간다)."""
    left = np.ones(len(pts), bool)
    chain, cur = [start], start
    left[start] = False
    while left.any():
        idx = np.nonzero(left)[0]
        d = np.linalg.norm(pts[idx] - pts[cur], axis=1)
        j = int(np.argmin(d))
        if d[j] > max_step:
            break
        cur = int(idx[j])
        left[cur] = False
        chain.append(cur)
    return chain


def _smooth_resample(line, step=10.0, k=7):
    if len(line) > k:
        pad = np.pad(line, ((k // 2, k // 2), (0, 0)), mode='edge')
        line = np.stack([np.convolve(pad[:, i], np.ones(k) / k, mode='valid') for i in (0, 1)], 1)
    seg = np.linalg.norm(np.diff(line, axis=0), axis=1)
    s = np.r_[0, np.cumsum(seg)]
    ss = np.arange(0, s[-1], step)
    return np.c_[np.interp(ss, s, line[:, 0]), np.interp(ss, s, line[:, 1])], s[-1]


def skeleton_pieces(points, cell=5.0, max_range=700.0, min_len=60.0, max_pieces=3):
    """바닥의 테이프 점 (좌/우 없음) → 조각마다 끝에서 끝까지 순서 있는 폴리라인 목록, 큰 조각부터."""
    points = points[(points[:, 1] > 0) & (np.linalg.norm(points, axis=1) < max_range)]
    if len(points) < 20:
        return []
    x0, y0 = points.min(0) - 3 * cell
    ij = np.floor((points - [x0, y0]) / cell).astype(int)
    grid = np.zeros(ij.max(0)[::-1] + 4, np.uint8)
    grid[ij[:, 1], ij[:, 0]] = 1
    grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(grid, 8)
    out = []
    for k in 1 + np.argsort(-st[1:, cv2.CC_STAT_AREA])[:max_pieces]:
        yy, xx = np.nonzero(skeletonize(lab == k))
        if len(xx) < 5:
            continue
        pts = np.c_[x0 + (xx + 0.5) * cell, y0 + (yy + 0.5) * cell]
        end = _walk(pts, int(np.argmin(np.linalg.norm(pts, axis=1))), 3 * cell)[-1]   # 한쪽 끝으로 간 다음
        line, length = _smooth_resample(pts[_walk(pts, end, 3 * cell)])               # 그 끝에서 반대쪽 끝까지
        if length >= min_len:
            out.append(line)
    return out


def orient_piece(line, hint=0, ambiguous_dy=30.0):
    """라벨 없는 차선 조각의 '앞' 방향과 도로가 있는 쪽.

    로봇에서 가장 가까운 점에서 조각은 두 방향으로 뻗는다. 다음 15 cm 안에서 앞으로(y) 더 멀어지는 쪽이 진행 방향.
    조각이 로봇 앞을 가로질러 둘이 비슷하면 hint (+1 = 최근 우회전, −1 = 좌회전) 쪽.
    -> (가까운 점부터의 폴리라인, side +1/−1 = 진행 방향의 오른쪽/왼쪽이 도로, 애매했는가)
    도로는 로봇이 있는 쪽이다 (로봇은 두 차선 사이를 달린다).
    """
    i0 = int(np.argmin(np.linalg.norm(line, axis=1)))
    cands = [c for c in (line[i0:], line[: i0 + 1][::-1]) if len(c) >= 2]

    def gain(c):
        e = c[min(len(c) - 1, 15)]
        return e[1] - c[0][1], e[0] - c[0][0]
    g = [gain(c) for c in cands]
    best = int(np.argmax([dy for dy, _ in g]))
    amb = len(cands) == 2 and abs(g[0][0] - g[1][0]) < ambiguous_dy
    if amb and hint:
        best = int(np.argmax([np.sign(dx) == np.sign(hint) for _, dx in g]))
    c = cands[best]
    t = c[min(len(c) - 1, 3)] - c[0]
    t = t / (np.linalg.norm(t) + 1e-9)
    right = np.array([t[1], -t[0]])
    side = +1 if (-c[0]) @ right > 0 else -1          # 로봇(원점)이 진행 방향 오른쪽 → 도로도 오른쪽
    return c, side, amb


def offset(line, side, dist):
    """폴리라인을 옆으로 옮긴다. side=+1: 진행 방향 오른쪽, −1: 왼쪽."""
    t = np.gradient(line, axis=0)
    t = t / (np.linalg.norm(t, axis=1, keepdims=True) + 1e-9)
    return line + side * dist * np.c_[t[:, 1], -t[:, 0]]


def point_at_distance(path, ld):
    """경로(로봇에서 멀어지는 순) 위에서 원점과의 거리가 처음 ld 에 닿는 점. 경로가 짧으면 끝점."""
    r = np.linalg.norm(path, axis=1)
    idx = np.nonzero(r >= ld)[0]
    if len(idx) == 0:
        return path[-1]
    i = idx[0]
    if i == 0:
        return path[0]
    f = (ld - r[i - 1]) / max(r[i] - r[i - 1], 1e-6)
    return path[i - 1] + f * (path[i] - path[i - 1])


# ---------------------------------------------------------------- 추종

class BevLaneFollower:
    """로봇 1대의 BEV 차선 중앙 목표. 직전 회전 방향(hint)을 기억한다."""

    def __init__(self, camera, params=None):
        self.cam = camera
        self.p = params or BevParams()
        self.hint = 0

    def reset(self):
        self.hint = 0

    def update(self, lane_mask, image=None):
        p = self.p
        m = np.asarray(lane_mask, bool)
        if p.tape_trim and image is not None and m.any():
            tape = tape_mask(image, p.tape_top_frac, p.tape_threshold)
            if (m & tape).sum() > 0.3 * m.sum():
                m = m & tape
        if m.sum() < p.min_mask_px:
            return BevResult()
        ys, xs = np.nonzero(m)
        P, ok = self.cam.pixels_to_floor(np.c_[xs, ys])
        lanes, paths, targets, amb = {}, {}, {}, False
        axle = np.array([0.0, p.axle_mm])
        for piece in skeleton_pieces(P[ok], p.cell_mm, p.max_range_mm, p.min_piece_mm, p.max_pieces):
            line, side, a = orient_piece(piece, self.hint)
            cls = LEFT if side > 0 else RIGHT                  # 도로가 오른쪽이면 그 선은 왼쪽 차선
            if cls in lanes and len(lanes[cls]) >= len(line):   # 쪽마다 가장 긴 조각
                continue
            lanes[cls] = line
            amb = amb or a
            paths[cls] = offset(line, side, p.lane_width_mm / 2.0)
            targets[cls] = point_at_distance(paths[cls] + axle, p.lookahead_mm) - axle   # 거리는 구동축에서 잰다
        if not targets:
            return BevResult()
        mode = 'both' if len(targets) == 2 else ('left_only' if LEFT in targets else 'right_only')
        if len(targets) == 2 and np.linalg.norm(targets[LEFT] - targets[RIGHT]) < 0.5 * p.lane_width_mm:
            tgt = (targets[LEFT] + targets[RIGHT]) / 2.0
            path = paths[LEFT] if len(paths[LEFT]) >= len(paths[RIGHT]) else paths[RIGHT]
        else:
            cls = min(targets, key=lambda c: np.linalg.norm(lanes[c][0]))   # 둘이 다르면 로봇에서 가까운 선
            tgt, path = targets[cls], paths[cls]
            if len(targets) == 2:
                mode = 'both(disagree)'
        rel = tgt + axle
        ld2 = float(rel @ rel)
        curv = 2.0 * rel[0] / ld2 if ld2 > 1.0 else 0.0
        if abs(curv) > p.hint_curvature:
            self.hint = 1 if curv > 0 else -1
        return BevResult(mode, tgt, curv, path, lanes, amb)


def apply_bev(r, b, camera, width, height):
    """BEV 결과로 TargetResult 의 차선 중앙(품질·목표 픽셀·error_x·좌/우)을 덮어쓴다. 횡단보도·빨간 선 등은 그대로.

    로봇은 바닥 좌표(LanePath.floor_*)로 따라가고, error_x·target_x 는 같은 목표점을 영상에 투영한 값이라
    follow_memory 를 끈 예전 제어와 디버그 오버레이에서도 같은 점을 본다.
    """
    r.left_seen, r.right_seen = 'L' in b.lanes, 'R' in b.lanes
    if not b.valid:
        r.quality, r.error_x, r.confidence = QUALITY_LOST, 0.0, 0.0
        return r
    r.quality = QUALITY_BOTH if b.mode.startswith('both') else QUALITY_SINGLE
    u, v = camera.floor_to_pixels(b.target)[0] if camera.in_front(b.target)[0] else (width / 2.0, height - 1.0)
    cx = width / 2.0
    r.target_x = int(round(min(max(u, -width), 2 * width)))
    r.target_y = int(round(min(max(v, 0.0), height - 1.0)))
    r.error_x = max(-1.0, min(1.0, (u - cx) / cx))
    return r


def draw_bev(img, camera, result, color_path=(0, 0, 255), color_target=(0, 0, 255), color_lane=(255, 0, 255)):
    """카메라 영상 위에 BEV 결과(차선 · 중앙 경로 · 목표점)를 투영해 그린다. 카메라 앞 바닥 점만."""
    h, w = img.shape[:2]

    def poly(pts, color, thick):
        pts = np.asarray(pts).reshape(-1, 2)
        pts = pts[camera.in_front(pts)] if len(pts) else pts
        if len(pts) < 2:
            return
        uv = camera.floor_to_pixels(pts)
        ok = (uv[:, 1] >= 0) & (uv[:, 1] < h) & (uv[:, 0] > -w) & (uv[:, 0] < 2 * w)
        if ok.sum() > 1:
            cv2.polylines(img, [np.round(uv[ok]).astype(np.int32)], False, color, thick)

    for line in result.lanes.values():
        poly(line, color_lane, 1)
    poly(result.path, color_path, 2)
    if result.target is not None and camera.in_front(result.target)[0]:
        u, v = camera.floor_to_pixels(result.target)[0]
        if 0 <= v < h:
            cv2.circle(img, (int(round(u)), int(round(v))), 7, color_target, 2)
    return img
