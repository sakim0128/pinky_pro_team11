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
