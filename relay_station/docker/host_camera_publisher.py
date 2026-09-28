# -*- coding: utf-8 -*-
"""USB 웹캠 → MJPEG. **호스트에서** 돌린다 (컨테이너 안이 아니다).

    python relay_station/docker/host_camera_publisher.py --camera 0

윈도우 Docker Desktop 은 USB 장치를 컨테이너에 그냥 못 넘긴다(usbipd-win 으로 WSL2 에
붙이고 uvcvideo 가 올라와야 /dev/video0 이 생긴다). 그래서 **호스트가 발행하고
컨테이너가 당겨온다.** 게이트웨이의 `pull` transport 가 원래 그러라고 있는 것이다.

⭐ 이게 단순히 우회가 아니라 **현장에 더 가깝다.** 현장의 폰·태블릿도 정확히 pull 이다.
   `local_camera.py`(V4L2 lazy·LED) 경로만 안 탄다 — 그건 정직하게 적어 둔다.

이 발행기는 현장 사슬을 흉내낸다. 흉내내는 것과 그 이유:

    X-Capture-Clock     촬영 시각. 곁표·시계 오차 계산이 이걸로 돈다
    X-Publisher-Session 기동마다 새로 발급 — 앱에 [MUST] 로 요구한 바로 그 규약이다
    X-Frame-Seq         **내보낸** 프레임만 센다 — 또 하나의 [MUST]
    X-Process-Ms/Rules  검열 상자를 적용한 시간과 규칙 버전 (가공 단 흉내)
    GET /process        검열 상자 좌표를 낸다 ← 홈랩 쪽에 요청해 둔 것을 여기선 먼저 쓴다
    GET /status         **디코드한 실제 프레임 크기**를 낸다 (설정값이 아니다, MCVA-70)

⭐ 검열 상자를 흉내내는 이유: 현장 관측 경로는 검열 단 **아래**에 있어서, 흐린 영역이
   아레나 코너를 덮으면 정착이 안 된다. 그 상황을 실물 없이 재현해야 MCV-2M/2R 을 짤 수 있다.
"""
import argparse
import io
import json
import math
import os
import random
import socket
import socketserver
import string
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy
import cv2

BOUNDARY = "raasframe"

# 합성 아레나의 외벽 두께(cm). 줄자 명세 §1 의 실물과 같게 둔다 —
# 픽스처가 실물과 다르면 소비자가 실물에서만 실패한다.
OUTER_WALL_CM = 4.0          # 현장과 같은 경계선. 다르게 두면 흉내가 아니다
IDLE_RELEASE_S = 10.0           # 보는 사람이 없으면 카메라를 놓는다 (LED 가 꺼진다)


def _session_id():
    return "host-" + "".join(random.choices(string.ascii_lowercase + string.digits, k=8))


class Censor(object):
    """가공 단의 검열 흐림을 흉내낸다. 좌표는 프레임 비율(0~1)이다 — 현장 규칙과 같다."""

    def __init__(self, box=None, version="host-v1", kernel=31, declare_only=False):
        self.box = box              # {"x","y","w","h"} 또는 None
        self.version = version
        self.kernel = int(kernel) | 1
        # ⭐ 시험 전용: 상자를 **선언만 하고 흐리지 않는다.**
        #    소비자의 픽셀 검산이 거짓 선언을 잡아내는지 확인하려는 것이다.
        #    라벨을 믿지 않는 검사가 정말로 라벨을 안 믿는지는 이렇게만 알 수 있다.
        self.declare_only = bool(declare_only)
        # 규칙을 적재한 시각. 가공 단이 mtime 을 붙이는 것과 같은 자리다.
        self.loaded_at = time.strftime("%H:%M:%S")

    def apply(self, frame):
        if not self.box or self.declare_only:
            return frame, 0.0
        t0 = time.time()
        h, w = frame.shape[:2]
        x0 = max(0, int(self.box["x"] * w))
        y0 = max(0, int(self.box["y"] * h))
        x1 = min(w, x0 + int(self.box["w"] * w))
        y1 = min(h, y0 + int(self.box["h"] * h))
        if x1 > x0 and y1 > y0:
            frame = frame.copy()
            frame[y0:y1, x0:x1] = cv2.GaussianBlur(
                frame[y0:y1, x0:x1], (self.kernel, self.kernel), 0)
        return frame, (time.time() - t0) * 1000.0

    def rules_value(self):
        """`<구현>@<규칙 적재 시각>` — **기기가 실제로 내는 형태**다.

        ⭐ 가공 단은 `v1@04:17:32` 처럼 낸다(실측). 처음엔 발행기가 접미사 없이
           `host-v2` 만 내서, 소비자의 allowlist 대조가 **기기가 보내지 않는 형태**로만
           시험됐다. 그 사이 정확 일치 대조는 실기기에서 영원히 거짓이었는데
           내 시험은 전부 초록이었다 — 픽스처가 같은 거짓말을 하면 결함을 못 본다.
        """
        return "%s@%s" % (self.version, self.loaded_at)

    def header_value(self):
        """`x,y,w,h` - 가공 단의 format_blur 와 같은 규약. 상자가 없으면 None."""
        if not self.box:
            return None
        return "%g,%g,%g,%g" % (self.box["x"], self.box["y"],
                                self.box["w"], self.box["h"])

    def public(self):
        return {"rulesVersion": self.version, "blur": self.box, "kernel": self.kernel}


class ArenaPattern(object):
    """아레나를 **알려진** 호모그래피로 사영해 그린다. 웹캠 없이 경로를 증명하려는 것이다.

    ⭐ 이 그림은 정답을 안다. 아레나 cm → 화면 px 변환이 코드 안에 있으므로,
       사람이 찍은 네 모서리로 푼 호모그래피가 맞는지 **대볼 수 있다.**
       네 점의 재투영 오차(구조상 0)로는 못 하는 일이다.
    """

    def __init__(self, width, height, arena_path=None):
        self.w, self.h = int(width), int(height)
        self.arena_path = self._arena_path(arena_path)
        self.arena = self._load_arena(arena_path)
        self.matrix = (self._build_homography() if self.arena else None)
        self.drift_after = None      # 시험용 (drift_offset 참조)
        self.drift_px = 0
        self.started_at = time.time()
        self.corners_px = self._corner_pixels()
        self._t = 0.0

    @staticmethod
    def _arena_path(path):
        """어느 파일을 보려 했나. ⭐ 못 찾았을 때 **그 경로를 말해야** 사람이 고친다."""
        return path or os.environ.get(
            "MCV_ARENA_CONFIG",
            os.path.expanduser("~/pinky_pro/src/pinky_pro_team11/configs/arena.json"))

    @staticmethod
    def _load_arena(path):
        p = ArenaPattern._arena_path(path)
        try:
            with io.open(p, encoding="utf-8") as fh:
                raw = json.load(fh)
            return {"widthCm": float(raw["widthCm"]),
                    "heightCm": float(raw["heightCm"]),
                    "landmarks": raw.get("landmarks") or {},
                    "version": raw.get("version")}
        except Exception:
            # 아레나를 모르면 지어내지 않는다. 격자만 그린다.
            return None

    def _build_homography(self):
        """아레나 cm → 화면 px 의 **진짜 사영행렬**.

        ⭐ 처음엔 폭을 v 에 비례해 줄이는 식으로 그렸는데, 그건 px 에 u*v 항이 있는
           **쌍선형**이라 사영변환이 아니다. 네 모서리로 푼 호모그래피와 내부 점이
           최대 103 px 어긋났다(2026-09-10 검산). 그 그림으로 검증점을 재면
           **캘리브레이션 탓이 아닌 오차**가 나오고, 대조군이 아니라 함정이 된다.

        그래서 네 모서리 픽셀만 사람이 고르고, 나머지는 전부 이 행렬을 통과시킨다.
        """
        a = self.arena
        w, h = a["widthCm"], a["heightCm"]
        # 아래(가까운 쪽)는 넓고 위(먼 쪽)는 좁은 사다리꼴. 화면 y 는 아래로 자란다.
        near, far = 0.94, 0.46
        cx = self.w * 0.5
        top, bottom = self.h * 0.16, self.h * 0.92
        dst = numpy.array([
            [cx - 0.5 * self.w * near, bottom],   # bl (아레나 0,0)
            [cx + 0.5 * self.w * near, bottom],   # br
            [cx + 0.5 * self.w * far, top],       # tr
            [cx - 0.5 * self.w * far, top],       # tl
        ], dtype=numpy.float32)
        src = numpy.array([[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]],
                          dtype=numpy.float32)
        return cv2.getPerspectiveTransform(src, dst)

    def _project(self, x_cm, y_cm):
        """아레나 cm → 화면 px. 사영변환 하나만 통과시킨다."""
        m = self.matrix
        d = m[2][0] * x_cm + m[2][1] * y_cm + m[2][2]
        return ((m[0][0] * x_cm + m[0][1] * y_cm + m[0][2]) / d,
                (m[1][0] * x_cm + m[1][1] * y_cm + m[1][2]) / d)

    def _corner_pixels(self):
        if not self.arena:
            return None
        a = self.arena
        w, h = a["widthCm"], a["heightCm"]
        return {"bl": self._project(0.0, 0.0), "br": self._project(w, 0.0),
                "tr": self._project(w, h), "tl": self._project(0.0, h)}

    def drift_offset(self):
        """시험용: 기동 후 일정 시간이 지나면 그림을 통째로 민다.

        ⭐ **세션·해상도·렌즈는 그대로**다. 오직 세상만 옮겨간다 —
           무효화 3종이 어느 것도 못 잡는 바로 그 상황이고,
           드리프트 감시가 진짜로 그걸 잡는지는 이렇게만 확인할 수 있다.
           (발행기를 재기동하면 세션이 바뀌어 다른 사유로 먼저 걸린다.)
        """
        if self.drift_after is None:
            return (0, 0)
        if (time.time() - self.started_at) < self.drift_after:
            return (0, 0)
        return (self.drift_px, int(self.drift_px * 0.6))

    def frame(self):
        import numpy as np
        img = np.full((self.h, self.w, 3), 28, dtype=np.uint8)
        if not self.arena:
            cv2.putText(img, "no arena.json", (20, self.h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (60, 60, 200), 2)
            return img
        a = self.arena
        w, h = a["widthCm"], a["heightCm"]

        ox, oy = self.drift_offset()
        def pt(x, y):
            p = self._project(x, y)
            return (int(round(p[0])) + ox, int(round(p[1])) + oy)

        # 바닥
        floor = np.array([pt(0, 0), pt(w, 0), pt(w, h), pt(0, h)], dtype=np.int32)
        cv2.fillPoly(img, [floor], (52, 58, 54))
        # 50 cm 격자 — 사영이 제대로 됐는지 **사람이 눈으로** 보라고 긋는다.
        # ⭐ 바닥과 거의 같은 밝기로 긋는다. 처음엔 또렷하게 그었는데, 소비자가
        #    그 선들을 **벽으로 읽어** 온 바닥을 가로지르는 격자벽이 생겼고
        #    모든 광선이 막혀 "비었다 0" 이 나왔다(2026-09-11 실측).
        #    **실제 아레나 바닥에는 격자선이 없다.** 픽스처는 실물을 닮아야 한다 —
        #    이 세션에서 픽스처가 실물과 달라 소비자를 속인 네 번째 경우다.
        x = 50.0
        while x < w:
            cv2.line(img, pt(x, 0), pt(x, h), (58, 64, 60), 1)
            x += 50.0
        y = 50.0
        while y < h:
            cv2.line(img, pt(0, y), pt(w, y), (58, 64, 60), 1)
            y += 50.0
        # 외벽 — ⭐ **선이 아니라 두께 있는 벽**으로 그린다.
        #    처음엔 polylines 로 3px 선을 그었는데, 선에는 발자국이 없다.
        #    실제 아레나 외벽은 4cm 이고(줄자 명세 §1), 소비자가 테두리에서 벽/바닥
        #    극성을 찾으므로 두께가 없으면 **판정할 수가 없다**(2026-09-11 실측).
        #    픽스처가 실물과 다르면 파이프라인이 실물에서만 실패한다.
        for x0, y0, x1, y1 in ((0.0, 0.0, w, OUTER_WALL_CM),
                               (0.0, h - OUTER_WALL_CM, w, h),
                               (0.0, 0.0, OUTER_WALL_CM, h),
                               (w - OUTER_WALL_CM, 0.0, w, h)):
            ow = numpy.array([pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)],
                             dtype=numpy.int32)
            cv2.fillPoly(img, [ow], (96, 104, 100))
        # 아레나 명세의 구조물 (줄자 §3)
        for x0, y0, x1, y1 in ((97.0, 0.0, 143.0, 70.5),      # 중앙 폐쇄 블록
                               (205.0, 0.0, 209.0, 62.5),      # 주방 격벽
                               (95.0, 96.5, 99.0, h)):         # 침실 상부 격벽
            box = np.array([pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)],
                           dtype=np.int32)
            cv2.fillPoly(img, [box], (96, 104, 100))
            cv2.polylines(img, [box], True, (200, 206, 203), 2)
        # 명세가 아는 지점들 — 검증점으로 찍을 자리다
        for name, p in (a["landmarks"] or {}).items():
            q = pt(float(p["x"]), float(p["y"]))
            cv2.drawMarker(img, q, (60, 180, 220), cv2.MARKER_CROSS, 14, 2)
        # 움직이는 점 — 프레임이 매번 달라야 수신 쪽이 살아있다고 본다
        self._t += 0.06
        rx = (w * 0.5) + (w * 0.32) * math.cos(self._t)
        ry = (h * 0.5) + (h * 0.26) * math.sin(self._t * 1.7)
        cv2.circle(img, pt(rx, ry), 9, (80, 220, 140), -1)
        cv2.putText(img, "SYNTHETIC ARENA %s  %.0fx%.0f cm"
                    % (a.get("version"), w, h), (12, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 160, 155), 1)
        return img


class Camera(object):
    """수요가 있을 때만 연다. 없으면 놓는다 — 현장 relay-cam(MCV-1C)과 같은 규약."""

    def __init__(self, index, width, height, fps, censor, quality=85,
                 pattern=None):
        self.index = index
        self.pattern = pattern
        self.lens = None
        self.want = (width, height)
        self.fps = fps
        self.censor = censor
        self.quality = quality

        self._lock = threading.Lock()
        self._cap = None
        self._jpeg = None
        self._size = None           # 디코드된 **실제** 크기
        self._seq = 0               # 내보낸 프레임만 센다
        self._capture_ms = 0
        self._process_ms = 0.0
        self._last_demand = 0.0
        self._session = _session_id()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @property
    def session(self):
        return self._session

    def demand(self):
        with self._lock:
            self._last_demand = time.time()

    def _open(self):
        if self.pattern is not None:
            return "pattern"      # 장치를 안 연다
        backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
        cap = cv2.VideoCapture(self.index, backend)
        if self.want[0]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.want[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.want[1])
        if not cap.isOpened():
            cap.release()
            return None
        print("  카메라 %s 열림" % self.index, flush=True)
        return cap

    def _release(self):
        if self._cap is not None and self.pattern is not None:
            self._cap = None
            return
        if self._cap is not None:
            self._cap.release()
            self._cap = None
            print("  카메라 %s 놓음 (수요 없음)" % self.index, flush=True)

    def _run(self):
        interval = 1.0 / max(1.0, float(self.fps))
        while not self._stop.is_set():
            with self._lock:
                wanted = (time.time() - self._last_demand) <= IDLE_RELEASE_S
            if not wanted:
                self._release()
                time.sleep(0.3)
                continue
            if self._cap is None:
                self._cap = self._open()
                if self._cap is None:
                    time.sleep(1.0)
                    continue
            if self.pattern is not None:
                ok, frame = True, self.pattern.frame()
            else:
                ok, frame = self._cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            capture_ms = int(time.time() * 1000.0)
            frame, proc_ms = self.censor.apply(frame)
            ok, buf = cv2.imencode(".jpg", frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            if not ok:
                continue
            with self._lock:
                self._jpeg = buf.tobytes()
                self._size = (int(frame.shape[1]), int(frame.shape[0]))
                self._capture_ms = capture_ms
                self._process_ms = proc_ms
                # ⭐ 내보낼 프레임에만 번호를 준다. 버린 프레임에 번호를 매기면
                #    받는 쪽이 그만큼을 '유실'로 읽는다 — 앱에 [MUST] 로 요구한 것.
                self._seq += 1
            time.sleep(interval)

    def latest(self):
        with self._lock:
            return (self._jpeg, self._seq, self._capture_ms,
                    self._process_ms, self._size)

    def status(self):
        with self._lock:
            size = self._size
            return {
                "lane": ("HOST-PATTERN" if self.pattern is not None
                         else "HOST-CAMERA"),
                "synthetic": self.pattern is not None,
                "cameraIndex": self.index,
                "requested": {"width": self.want[0], "height": self.want[1]},
                # ⭐ 요청한 값이 아니라 **디코드된 프레임**의 크기다.
                #    /status 가 설정값을 실물처럼 내서 오진했던 일을 여기서 반복하지 않는다.
                "actualWidth": (size[0] if size else None),
                "actualHeight": (size[1] if size else None),
                "resolution": ("%dx%d" % size if size else None),
                "framesEmitted": self._seq,
                "publisherSession": self._session,
                "open": self._cap is not None,
                "targetFps": self.fps,
            }


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    camera = None
    censor = None

    def log_message(self, fmt, *args):
        pass

    def _json(self, payload, code=200):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/video", "/stream.mjpg"):
            return self.video()
        if path == "/status":
            return self._json(self.camera.status())
        if path == "/process":
            # ⭐ 검열 상자를 **좌표로** 낸다. 홈랩 가공 단에 요청해 둔 바로 그것이라,
            #    게이트웨이의 ROI 제외 로직을 실물 없이 여기에 대고 만들 수 있다.
            return self._json(self.censor.public())
        if path == "/healthz":
            return self._json({"ok": True})
        self.send_response(404)
        self.send_header("Content-Length", "0")
        self.end_headers()

    def video(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=%s" % BOUNDARY)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        last_seq = -1
        try:
            while True:
                self.camera.demand()
                jpeg, seq, capture_ms, proc_ms, _size = self.camera.latest()
                if jpeg is None or seq == last_seq:
                    time.sleep(0.005)
                    continue
                last_seq = seq
                head = io.BytesIO()
                head.write(b"--" + BOUNDARY.encode() + b"\r\n")
                head.write(b"Content-Type: image/jpeg\r\n")
                head.write(b"Content-Length: %d\r\n" % len(jpeg))
                head.write(b"X-Capture-Clock: %d\r\n" % capture_ms)
                head.write(b"X-Publisher-Session: %s\r\n"
                           % self.camera.session.encode())
                head.write(b"X-Frame-Seq: %d\r\n" % seq)
                head.write(b"X-Process-Ms: %.1f\r\n" % proc_ms)
                head.write(b"X-Process-Rules: %s\r\n"
                           % self.censor.rules_value().encode())
                # 가공 단과 **같은 형식**으로 낸다: x,y,w,h (프레임 비율). 꺼져 있으면 헤더 없음.
                blur = self.censor.header_value()
                if blur:
                    head.write(b"X-Process-Blur: %s\r\n" % blur.encode())
                # 앱 v0.2.1 이 내는 것과 같은 이름·형식. 인코딩은 가공이 아니다.
                if self.camera.lens:
                    head.write(b"X-Camera-Lens: %s\r\n"
                               % self.camera.lens.encode())
                head.write(b"\r\n")
                self.wfile.write(head.getvalue())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


class ThreadedHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    """⭐ 윈도우에서 `allow_reuse_address=True` 는 **같은 포트에 둘이 붙는 것을 허용한다.**

    리눅스의 SO_REUSEADDR 은 TIME_WAIT 재사용만 허용하지만, 윈도우에서는 살아 있는
    소켓을 가로챌 수 있다. 그러면 발행기 두 개가 :18082 에 붙고 **어느 쪽이 응답하는지
    알 수 없다** — 2026-09-10 실측: 재기동했는데 받는 쪽은 옛 세션을 계속 봤다.

    조용히 성공하는 것보다 **시끄럽게 실패하는 편이 낫다.** 앱 쪽에도 같은 요구를 했다
    (바인드 실패를 /healthz 에 드러낼 것).
    """
    daemon_threads = True
    allow_reuse_address = (sys.platform != "win32")

    def server_bind(self):
        if sys.platform == "win32":
            try:
                self.socket.setsockopt(socket.SOL_SOCKET,
                                       getattr(socket, "SO_EXCLUSIVEADDRUSE", 0), 1)
            except (OSError, AttributeError):
                pass
        return HTTPServer.server_bind(self)


def _console_utf8():
    """윈도우 콘솔은 기본이 cp949 라 이모지 한 글자에 **프로그램이 죽는다.**

    이 도구는 호스트(윈도우)에서 도는 것이 본업이다. 배너 때문에 안 뜨면 도구가 아니다.
    2026-09-10 실측: UnicodeEncodeError: 'cp949' codec can't encode '📷'
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError, OSError):
            pass


def main():
    _console_utf8()
    ap = argparse.ArgumentParser(description="USB 웹캠 → MJPEG (호스트에서 실행)")
    ap.add_argument("--camera", type=int, default=0, help="OpenCV 카메라 번호")
    ap.add_argument("--port", type=int, default=18082,
                    help="현장 시청 중계와 같은 포트 (기본 18082)")
    ap.add_argument("--bind", default="0.0.0.0",
                    help="컨테이너가 host.docker.internal 로 닿아야 하므로 기본 0.0.0.0")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=float, default=15.0,
                    help="현장 앱에 요청한 값과 같게 두면 부하도 비슷해진다")
    ap.add_argument("--quality", type=int, default=85)
    ap.add_argument("--censor", default="",
                    help='검열 상자 "x,y,w,h" (프레임 비율 0~1). 비우면 끔. '
                         '현장 기본값을 흉내내려면 "0,0,0.5,0.5"')
    ap.add_argument("--censor-kernel", type=int, default=31)
    ap.add_argument("--censor-declare-only", action="store_true",
                    help="시험 전용: 상자를 선언만 하고 **흐리지 않는다**. "
                         "소비자의 픽셀 검산이 거짓 선언을 잡는지 보려는 것")
    ap.add_argument("--rules-version", default="host-v1")
    ap.add_argument("--drift-after", type=float, default=None,
                    help="시험 전용: 이 초가 지나면 그림을 통째로 민다. "
                         "세션·해상도·렌즈는 그대로 — 드리프트 감시만 잡을 수 있는 상황")
    ap.add_argument("--drift-px", type=int, default=30)
    ap.add_argument("--lens", default="",
                    help="X-Camera-Lens 로 내보낼 렌즈 이름 (예: WIDE / ULTRA_WIDE). "
                         "비우면 헤더를 안 낸다")
    ap.add_argument("--pattern", action="store_true",
                    help="웹캠 대신 **정답을 아는** 합성 아레나를 낸다. "
                         "경로 증명과 캘리브레이션 검산에 쓴다")
    args = ap.parse_args()

    box = None
    if args.censor.strip():
        try:
            x, y, w, h = [float(v) for v in args.censor.split(",")]
        except ValueError:
            sys.exit('--censor 는 "x,y,w,h" 형식이다 (예: 0,0,0.5,0.5)')
        box = {"x": x, "y": y, "w": w, "h": h}

    censor = Censor(box, version=args.rules_version, kernel=args.censor_kernel,
                    declare_only=args.censor_declare_only)
    pattern = (ArenaPattern(args.width, args.height) if args.pattern else None)
    if pattern is not None and not pattern.arena:
        # ⭐⭐ `--pattern` 의 존재 이유는 **정답을 아는 화면**이다. 아레나를 못 읽으면
        #    그건 합성 아레나가 아니라 글자 한 줄이 적힌 어두운 화면이고, 그걸
        #    "합성 아레나" 라고 배너에 찍으면 **픽스처가 자기에 대해 거짓말한다.**
        #    2026-09-11 에 그 화면으로 흔들림을 재서 "정합 1.0 으로 62cm" 가 나왔다.
        sys.exit("--pattern 인데 아레나 규격을 못 읽었다: %s%s"
                 "정답을 모르는 화면은 합성 아레나가 아니다. "
                 "MCV_ARENA_CONFIG 로 configs/arena.json 을 가리켜라."
                 % (pattern.arena_path, os.linesep))
    camera = Camera(args.camera, args.width, args.height, args.fps, censor,
                    quality=args.quality, pattern=pattern)
    camera.lens = (args.lens.strip() or None)
    if pattern is not None and args.drift_after is not None:
        pattern.drift_after = args.drift_after
        pattern.drift_px = args.drift_px
    Handler.camera = camera
    Handler.censor = censor

    try:
        srv = ThreadedHTTPServer((args.bind, args.port), Handler)
    except OSError as exc:
        camera._stop.set()
        sys.exit("포트 %d 를 못 열었다: %s" % (args.port, exc) + 
                 "  이미 다른 발행기가 붙어 있을 수 있다. 조용히 넘어가면 "
                 "받는 쪽이 **어느 쪽을 보고 있는지 모르게 된다.**")
    print("=" * 60)
    print(" 📷 호스트 웹캠 발행기")
    print("=" * 60)
    if pattern is not None:
        print("  화면        : **합성 아레나** (웹캠 아님)")
        if pattern.corners_px:
            print("  진짜 모서리 픽셀 — 정착 결과를 이것과 대본다:")
            for k in ("bl", "br", "tr", "tl"):
                p = pattern.corners_px[k]
                print("      %-3s  %8.2f , %8.2f" % (k, p[0], p[1]))
    else:
        print("  카메라 번호 : %d" % args.camera)
    print("  요청 해상도 : %dx%d @ %.0f fps  (실제 크기는 /status 가 낸다)"
          % (args.width, args.height, args.fps))
    print("  검열 상자   : %s%s" % (box or "없음",
          "  ⚠️ 선언만 하고 안 흐림(시험 모드)" if args.censor_declare_only else ""))
    print("  발행자 세션 : %s   (기동마다 새로 발급)" % camera.session)
    print("  주소        : http://%s:%d/video" % (args.bind, args.port))
    print("  컨테이너에서: http://host.docker.internal:%d/video" % args.port)
    print("  멈추려면 Ctrl+C. 보는 사람이 없으면 %.0f초 뒤 카메라를 놓는다."
          % IDLE_RELEASE_S)
    print("=" * 60, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n  종료", flush=True)
    finally:
        camera._stop.set()
        srv.server_close()


if __name__ == "__main__":
    main()
