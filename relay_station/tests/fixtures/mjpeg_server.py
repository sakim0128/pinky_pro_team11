# -*- coding: utf-8 -*-
"""픽스처 MJPEG 서버 + 파서.

결정 3A 는 관측기가 게이트웨이의 MJPEG 를 loopback 으로 구독하게 했다.
그러면 관측기(T6)를 개발할 때 게이트웨이가 떠 있어야 하는데, 로봇도 장비도 없는
자리에서는 그게 걸림돌이다. 이 픽스처가 게이트웨이 **자리를 대신** 선다.

    FixtureMjpegServer(frames=[...])          관측기(T6)
        │  GET /video_feed?src=cam1              │
        └──── multipart/x-mixed-replace ────────>│  iter_mjpeg_frames()
                                                 └─> JPEG bytes 하나씩

게이트웨이와 같은 계약(`multipart/x-mixed-replace; boundary=frame`)을 낸다.
따라서 여기서 도는 관측기는 실제 게이트웨이에서도 돈다.
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

BOUNDARY = b"frame"


def iter_mjpeg_frames(response, max_frames=None, timeout_s=10.0):
    """multipart/x-mixed-replace 응답에서 JPEG 를 하나씩 뽑는다.

    게이트웨이 구현과 같은 방식(파트 헤더 뒤 Content-Length 만큼 읽기)이 아니라
    **JPEG 매직 기반**으로 자른다 — 헤더 형식이 조금 달라도 견디게 하려는 것이다.
    """
    buf = b""
    started = time.time()
    count = 0
    while True:
        if time.time() - started > timeout_s:
            return
        chunk = response.read(4096)
        if not chunk:
            return
        buf += chunk
        while True:
            start = buf.find(b"\xff\xd8\xff")
            if start < 0:
                break
            end = buf.find(b"\xff\xd9", start + 3)
            if end < 0:
                break
            yield buf[start:end + 2]
            buf = buf[end + 2:]
            count += 1
            if max_frames is not None and count >= max_frames:
                return


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass  # 테스트 출력이 지저분해지지 않게

    def do_GET(self):
        parsed = urlparse(self.path)
        src = parse_qs(parsed.query).get("src", ["default"])[0]

        if parsed.path == "/api/sources":
            body = self.server.sources_json()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parsed.path not in ("/video_feed", "/video"):
            self.send_response(404)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=" + BOUNDARY.decode())
        self.send_header("Cache-Control", "no-cache, private")
        self.end_headers()

        try:
            for jpeg, extra in self.server.frame_provider(src):
                self.wfile.write(b"--" + BOUNDARY + b"\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(("Content-Length: %d\r\n" % len(jpeg)).encode())
                for key, value in (extra or ()):
                    self.wfile.write(("%s: %s\r\n" % (key, value)).encode())
                self.wfile.write(b"\r\n")
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                time.sleep(self.server.frame_interval_s)
        except (BrokenPipeError, ConnectionResetError):
            return


class FixtureMjpegServer:
    """with 블록 안에서만 사는 MJPEG 서버. 포트는 OS 가 고른다(충돌 없음)."""

    def __init__(self, frames, frame_interval_s=0.01, loop=True, sources=("default",),
                 sidecar=False, clock_skew_ms=0, seq_start=1, seq_jump_after=None,
                 seq_jump_by=3, session="fixture-session-1", clock=None,
                 process_ms=None, process_rules="v1@fixture"):
        self._frames = list(frames)
        self._interval = frame_interval_s
        self._loop = loop
        self._sources = list(sources)
        # MCV-1B1 곁표. 앱(CAMERA_SOURCE_LOOPBACK_v1)과 같은 파트 헤더를 낸다.
        #   sidecar=False  헤더 없음 - 곁표 없는 상대를 흉내낸다(관용 검증)
        #   clock_skew_ms  기기 시계가 중계보다 이만큼 **늦다** -> 관측 오차가 이 값이 된다
        #   seq_jump_after N 프레임 뒤 순번을 건너뛴다 -> 손실 카운트 검증
        self._sidecar = bool(sidecar)
        self._clock_skew_ms = int(clock_skew_ms)
        self._seq_start = int(seq_start)
        self._seq_jump_after = seq_jump_after
        self._seq_jump_by = int(seq_jump_by)
        self._session = session
        self._clock = clock or time.time
        # MCV-1B4 - 계약에 없지만 앱이 보내는 헤더. None 이면 안 보낸다(구식 상대 흉내).
        self._process_ms = process_ms
        self._process_rules = process_rules
        self._httpd = None
        self._thread = None

    def _provider(self, _src):
        seq = self._seq_start
        sent = 0
        while True:
            for f in self._frames:
                extra = ()
                if self._sidecar:
                    capture_ms = int(self._clock() * 1000.0) - self._clock_skew_ms
                    extra = (("X-Capture-Clock", capture_ms),
                             ("X-Publisher-Session", self._session),
                             ("X-Frame-Seq", seq))
                    if self._process_ms is not None:
                        extra += (("X-Process-Ms", self._process_ms),
                                  ("X-Process-Rules", self._process_rules))
                yield f, extra
                sent += 1
                seq += 1
                if self._seq_jump_after is not None and sent == self._seq_jump_after:
                    seq += self._seq_jump_by      # 이만큼 잃은 것으로 보여야 한다
            if not self._loop:
                return

    def _sources_json(self):
        import json
        payload = [{"id": s, "transport": "fixture", "fps": 1.0 / max(self._interval, 1e-6)}
                   for s in self._sources]
        return json.dumps({"sources": payload}).encode("utf-8")

    def __enter__(self):
        self._httpd = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
        self._httpd.daemon_threads = True
        self._httpd.frame_provider = self._provider
        self._httpd.frame_interval_s = self._interval
        self._httpd.sources_json = self._sources_json
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *_exc):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=3)

    @property
    def port(self):
        return self._httpd.server_address[1]

    @property
    def base_url(self):
        return "http://127.0.0.1:%d" % self.port

    def feed_url(self, src="default"):
        return "%s/video_feed?src=%s" % (self.base_url, src)
