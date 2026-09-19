"""가짜 로봇용 합성 카메라 — 회색 바닥 위 흰 테이프 두 줄(+횡단보도) 을 핀홀 투영으로 그린다.

ROS 에 의존하지 않는다. fake_camera_pub 와 classic 검출기 테스트가 쓴다.

    img = render_lane_frame(lateral=+0.03, heading=0.0, crosswalk_ahead=0.5)

lateral : 로봇이 차선 중심에서 **왼쪽** 으로 치우친 거리 (m, 좌측 양수 = route_follower 와 동일)
          → 차선이 화면에서 오른쪽으로 밀린다 → error_x 양수 (LanePath 규약)
heading : 로봇 진행방향 − 차선 방향 (rad, 좌회전 양수)
"""

import math

import numpy as np

try:
    import cv2
except ImportError:              # pragma: no cover
    cv2 = None


class CameraModel:
    def __init__(self, width=640, height=480, hfov_deg=100.0, cam_height=0.10, pitch_deg=25.0,
                 cam_forward=0.06):
        # 실제 핑키 카메라 기하는 미확인 — 가짜 파이프라인용 근사값. 0.72·H 행에서 반폭 ≈ 180 px.
        self.W, self.H = int(width), int(height)
        self.fx = (self.W / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        self.fy = self.fx
        self.cx, self.cy = self.W / 2.0, self.H / 2.0
        self.h = float(cam_height)
        self.pitch = math.radians(pitch_deg)
        self.forward = float(cam_forward)

    def project(self, X, Y):
        """바닥점 (X 전방, Y 좌측) [m, 로봇 base 기준] → (u, v) px. 뒤에 있으면 None."""
        X = X - self.forward
        # 카메라 좌표: z 전방, x 우측, y 아래. 피치(아래로) 만큼 회전
        cz = X * math.cos(self.pitch) + self.h * math.sin(self.pitch)
        cy = -X * math.sin(self.pitch) + self.h * math.cos(self.pitch)
        cx = -Y
        if cz <= 0.02:
            return None
        u = self.cx + self.fx * cx / cz
        v = self.cy + self.fy * cy / cz
        return u, v


def _lane_points(lateral, heading, y_off, x_from=0.05, x_to=1.6, step=0.05):
    """차선 좌표계(차선 방향 = x) 에서 y = y_off 인 선을 로봇 좌표계로 옮긴다."""
    c, s = math.cos(-heading), math.sin(-heading)
    pts = []
    x = x_from
    while x <= x_to + 1e-9:
        # 차선 좌표 (x, y_off - lateral) 를 로봇 프레임으로 회전
        lx, ly = x, y_off - lateral
        pts.append((lx * c - ly * s, lx * s + ly * c))
        x += step
    return pts


def render_lane_frame(lateral=0.0, heading=0.0, lane_width=0.20, tape_width=0.018,
                      crosswalk_ahead=None, crosswalk_stripes=4, camera=None,
                      floor_bgr=(120, 120, 120), tape_bgr=(245, 245, 245), noise=0):
    if cv2 is None:
        raise RuntimeError('python3-opencv 가 필요합니다')
    cam = camera or CameraModel()
    img = np.full((cam.H, cam.W, 3), floor_bgr, dtype=np.uint8)
    half = lane_width / 2.0
    for y_off in (+half, -half):
        # 테이프를 폭이 있는 띠로: 좌우 가장자리 두 선을 투영해 폴리곤으로 채운다
        inner = _lane_points(lateral, heading, y_off - tape_width / 2.0)
        outer = _lane_points(lateral, heading, y_off + tape_width / 2.0)
        poly = [cam.project(X, Y) for X, Y in inner] + [cam.project(X, Y) for X, Y in reversed(outer)]
        poly = [p for p in poly if p is not None]
        if len(poly) >= 3:
            cv2.fillPoly(img, [np.array(poly, dtype=np.int32)], tape_bgr)
    if crosswalk_ahead is not None:
        # 차선을 가로지르는 줄무늬: 차선 좌표에서 x ∈ [d, d+0.04·k], y ∈ [-half, +half]
        c, s = math.cos(-heading), math.sin(-heading)
        for k in range(crosswalk_stripes):
            x0 = crosswalk_ahead + k * 0.08
            x1 = x0 + 0.04
            corners = []
            for (lx, ly) in ((x0, -half), (x0, half), (x1, half), (x1, -half)):
                ly = ly - lateral
                corners.append(cam.project(lx * c - ly * s, lx * s + ly * c))
            if all(p is not None for p in corners):
                cv2.fillPoly(img, [np.array(corners, dtype=np.int32)], tape_bgr)
    if noise > 0:
        n = np.random.randint(-noise, noise + 1, img.shape, dtype=np.int16)
        img = np.clip(img.astype(np.int16) + n, 0, 255).astype(np.uint8)
    return img


# ------------------------------------------------------------------ 마커 렌더 (D7)

def _marker_image(dictionary_name, mid, px=200):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(d, mid, px)
    return cv2.aruco.drawMarker(d, mid, px)


def render_markers(img, robot_pose, marker_map, camera=None, px=200):
    """로봇 (x, y, yaw) 에서 보이는 바닥 마커들을 img 위에 원근 워프로 그린다 (제자리 수정).

    marker_map: marker_localizer.MarkerMap. 흰 여백(white_border) 포함.
    """
    from .marker_localizer import marker_object_points, rot_z, make_T
    cam = camera or CameraModel()
    rx, ry, ryaw = robot_pose
    c, s = math.cos(-ryaw), math.sin(-ryaw)

    def to_robot(mx, my):
        dx, dy = mx - rx, my - ry
        return dx * c - dy * s, dx * s + dy * c

    for mid, (x, y, yaw) in marker_map.poses.items():
        T = make_T(rot_z(yaw), (x, y, 0.0))
        for size, color in ((marker_map.size + 2 * marker_map.white_border, 255), (marker_map.size, None)):
            obj = marker_object_points(size).astype(float)
            world = (T[:3, :3] @ obj.T).T + T[:3, 3]
            uv = [cam.project(*to_robot(wx, wy)) for wx, wy, _ in world]
            if any(p is None for p in uv):
                break
            dst = np.array(uv, dtype=np.float32)
            if color is not None:
                cv2.fillPoly(img, [dst.astype(np.int32)], (color, color, color))
                continue
            src_img = _marker_image(marker_map.dictionary, mid, px)
            src = np.array([[0, 0], [px, 0], [px, px], [0, px]], dtype=np.float32)   # TL TR BR BL
            H, _ = cv2.findHomography(src, dst)
            if H is None:
                continue
            warped = cv2.warpPerspective(src_img, H, (img.shape[1], img.shape[0]),
                                         flags=cv2.INTER_NEAREST, borderValue=255)
            mask = np.zeros(img.shape[:2], dtype=np.uint8)
            cv2.fillPoly(mask, [dst.astype(np.int32)], 255)
            img[mask > 0] = np.repeat(warped[mask > 0][:, None], 3, axis=1)
    return img


# ------------------------------------------------------------------ 항공뷰 합성 (D8)

def render_overhead(cfg, robot_poses, ppm=400, margin=0.3, view_H=None, out_size=(1280, 720),
                    floor=110):
    """천장 카메라 합성 이미지. cfg: overhead_localizer.OverheadConfig, robot_poses: {name: (x, y, yaw)}.

    map 평면을 ppm px/m 로 그린 뒤 view_H(3×3, 평면→이미지) 로 원근 워프한다. view_H 가 None 이면
    기본 기울기 하나를 쓴다. (canvas_H, image) 를 돌려준다 — canvas_H 는 map(m)→이미지(px) 진값.
    """
    from .marker_localizer import marker_object_points, rot_z
    xs = [p[0] for p in cfg.reference.values()] + [p[0] for p in robot_poses.values()]
    ys = [p[1] for p in cfg.reference.values()] + [p[1] for p in robot_poses.values()]
    x0, y0 = min(xs) - margin, min(ys) - margin
    x1, y1 = max(xs) + margin, max(ys) + margin
    W, H = int((x1 - x0) * ppm), int((y1 - y0) * ppm)
    canvas = np.full((H, W, 3), floor, dtype=np.uint8)
    # map(m) → canvas(px): 위가 +y (항공 사진처럼)
    to_px = np.array([[ppm, 0, -x0 * ppm], [0, -ppm, y1 * ppm], [0, 0, 1]], np.float64)

    def draw(mid, x, y, yaw, size, white=0.02):
        for s_, col in ((size + 2 * white, 255), (size, None)):
            obj = marker_object_points(s_)[:, :2].astype(float)
            pts = (rot_z(yaw)[:2, :2] @ obj.T).T + np.array([x, y])
            px = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), to_px).reshape(-1, 2).astype(np.float32)
            if col is not None:
                cv2.fillPoly(canvas, [px.astype(np.int32)], (col, col, col))
                continue
            mk = _marker_image(cfg.dictionary, mid, 160)
            src = np.array([[0, 0], [160, 0], [160, 160], [0, 160]], np.float32)
            Hm, _ = cv2.findHomography(src, px)
            warped = cv2.warpPerspective(mk, Hm, (W, H), flags=cv2.INTER_NEAREST, borderValue=255)
            mask = np.zeros((H, W), np.uint8)
            cv2.fillPoly(mask, [px.astype(np.int32)], 255)
            canvas[mask > 0] = np.repeat(warped[mask > 0][:, None], 3, axis=1)

    for mid, (x, y, yaw) in cfg.reference.items():
        draw(mid, x, y, yaw, cfg.reference_size)
    for name, (x, y, yaw) in robot_poses.items():
        m = cfg.robots[name]
        cx, cy = x + m.offset_x * math.cos(yaw), y + m.offset_x * math.sin(yaw)
        draw(m.id, cx, cy, yaw + m.yaw_offset, m.size)

    if view_H is None:
        # 살짝 기울어진 카메라: 캔버스 네 귀퉁이를 사다리꼴로
        ow, oh = out_size
        src = np.array([[0, 0], [W, 0], [W, H], [0, H]], np.float32)
        dst = np.array([[ow * 0.08, oh * 0.06], [ow * 0.94, oh * 0.03],
                        [ow * 0.98, oh * 0.97], [ow * 0.04, oh * 0.92]], np.float32)
        view_H, _ = cv2.findHomography(src, dst)
    img = cv2.warpPerspective(canvas, view_H, out_size, borderValue=(floor, floor, floor))
    return view_H @ to_px, img
