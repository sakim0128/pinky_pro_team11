# -*- coding: utf-8 -*-
"""접속은 되는데 **바이트를 안 주는** 주소에 묶이지 않는다.

2026-09-12 실측(교육장 중계): 태블릿 앱 `:18086/processed` 가
`HTTP 200 · Content-Type: multipart/x-mixed-replace` 까지 정상으로 주고 **본문을
한 바이트도 안 보냈다**. 같은 순간 Termux 중계 `:18082/video` 는 9.6 fps 로 멀쩡했고,
그 주소는 `urlCandidates` 안에 **이미 들어 있었다.** 그런데 게이트웨이는 8분 넘게
죽은 첫 후보에 붙어 있었다 — `tablet-relay` 가 conn=False · recvFps=0 · served=0.

원인: 후보를 넘기는 `idx` 가 **접속 실패에서만** 오른다. 읽기 실패(스톨·중간 끊김)는
같은 후보로 재접속하므로 **후보 목록이 있어도 영영 안 쓰인다.**

⭐ 이건 '연결됨' 과 '데이터가 옴' 을 같은 것으로 본 결과다. 이 레포가 이미
   `jpeg_only_provider` 에서 배운 구분인데(`connected = jpeg is not None` 은 항상 참),
   재접속 경로에는 아직 안 와 있었다.
"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gateway_web"))

from mjpeg_puller import MjpegPuller

JPEG = b"\xff\xd8\xff" + b"\x11" * 64 + b"\xff\xd9"
BODY = (b"--raasframe\r\nContent-Type: image/jpeg\r\n\r\n" + JPEG + b"\r\n")


class 말없는응답:
    """접속·헤더는 주고 본문은 **영영 안 주는** 상대. 태블릿 앱이 이 상태였다."""

    def __init__(self):
        self.closed = False

    def read1(self, _n):
        # 소켓 타임아웃이 이렇게 올라온다(urlopen(timeout=) 은 읽기에도 걸린다).
        time.sleep(0.05)
        raise TimeoutError("timed out")

    read = read1

    def close(self):
        self.closed = True


class 멀쩡한응답:
    def __init__(self):
        self.closed = False

    def read1(self, _n):
        time.sleep(0.01)
        return BODY

    read = read1

    def close(self):
        self.closed = True


def _기다린다(조건, timeout=10.0):
    끝 = time.time() + timeout
    while time.time() < 끝:
        if 조건():
            return True
        time.sleep(0.05)
    return False


def test_바이트를_안_주는_첫_후보를_버리고_다음_후보로_넘어간다():
    """🔴 읽기 실패에 후보를 안 넘기면 빨개진다.

    첫 후보는 접속만 되고 본문이 없다. 둘째 후보는 멀쩡하다.
    후보를 넘기지 않으면 프레임이 **영원히 0** 이고 이 시험이 시간초과로 죽는다 —
    2026-09-12 현장에서 태블릿이 정확히 그 상태였다.
    """
    열린주소 = []
    lock = threading.Lock()

    def opener(url, _timeout):
        with lock:
            열린주소.append(url)
        return 말없는응답() if url.endswith("/stalled") else 멀쩡한응답()

    p = MjpegPuller(["http://t/stalled", "http://t/good"],
                    name="fallback-test", stale_after_s=3.0, opener=opener).start()
    try:
        assert _기다린다(lambda: p.stats()["frames"] > 0, timeout=15.0), (
            "말없는 첫 후보에 묶였다 — 둘째 후보를 영영 안 썼다. 연 주소: %r"
            % (열린주소,))
        jpeg, _, connected = p.get_latest_jpeg()
        assert jpeg == JPEG
        assert connected is True
        assert "http://t/good" in 열린주소
    finally:
        p.stop()


def test_후보가_하나뿐이면_그_하나로_계속_재시도한다():
    """후보가 없는데 넘기려 들어 IndexError 가 나면 빨개진다."""
    열린주소 = []

    def opener(url, _timeout):
        열린주소.append(url)
        return 말없는응답()

    p = MjpegPuller(["http://t/only"], name="single", opener=opener).start()
    try:
        assert _기다린다(lambda: len(열린주소) >= 2, timeout=10.0), "재시도를 안 한다"
        assert set(열린주소) == {"http://t/only"}
    finally:
        p.stop()
