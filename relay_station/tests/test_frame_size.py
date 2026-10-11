# -*- coding: utf-8 -*-
"""MCV-1B6 — 프레임 크기는 **프레임 자신에게** 묻는다.

2026-09-10 실측:

    폰      /status: 640x480     실제: 1088x1088     0.16 B/픽셀
    태블릿  /status: 1280x720    실제: 1280x720      0.13 B/픽셀

⭐ B/픽셀이 정상 범위였는데도 "저조도 노이즈" 라고 오진했다. 원인은 픽셀 3.85배였다.
   `/status.resolution` 이 **설정값**이었기 때문이다.

⭐⭐ 이 프로젝트에서 세 번째로 같은 종류의 오진이다 — 지표가 세계를 안 보고 설정을 봤다
   (하드코딩 도메인 · 거짓 `connected` · 이번 `resolution`).
   그래서 이 유닛은 **바이트에서** 읽는다. 설정을 참조할 길이 아예 없어야 한다.
"""
import struct
import time

import pytest

from mjpeg_puller import MjpegPuller, jpeg_dimensions
from fixtures.frames import make_jpeg
from fixtures.mjpeg_server import FixtureMjpegServer


def _wait_for(predicate, timeout=8.0, interval=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


# ---- 파서 -------------------------------------------------------------------

@pytest.mark.parametrize("w,h", [(640, 480), (1280, 720), (1088, 1088), (16, 16)])
def test_실제_인코딩한_크기를_읽는다(w, h):
    assert jpeg_dimensions(make_jpeg(width=w, height=h)) == (w, h)


def test_JPEG_이_아니면_None():
    assert jpeg_dimensions(b"") is None
    assert jpeg_dimensions(b"\xff\xd8\xff") is None
    assert jpeg_dimensions(b"not a jpeg at all, really not") is None


def test_잘린_프레임에서_지어내지_않는다():
    """반쪽 프레임에 숫자를 붙이면 그게 거짓말이 된다."""
    data = make_jpeg(width=1280, height=720)
    assert jpeg_dimensions(data[:6]) is None


def test_DHT_를_SOF_로_오독하지_않는다():
    """⭐ 0xC4(허프만 테이블)는 0xC0~0xCF 안에 있지만 프레임 헤더가 **아니다.**

    범위로 뭉뚱그리면 허프만 테이블의 바이트를 폭·높이로 읽는다. 그러면 크기가
    프레임마다 요동치고, 캘리브레이션이 매번 무효로 떨어진다.
    """
    # SOI + DHT(0xFFC4, 길이 20, 내용 아무거나) + SOF0(0xFFC0) 8x8x1
    dht_body = bytes(range(18))
    dht = b"\xff\xc4" + struct.pack(">H", 2 + len(dht_body)) + dht_body
    sof = b"\xff\xc0" + struct.pack(">H", 11) + b"\x08" + \
          struct.pack(">HH", 777, 999) + b"\x01\x01\x11\x00"
    assert jpeg_dimensions(b"\xff\xd8" + dht + sof) == (999, 777)


def test_세그먼트_길이가_망가져도_무한루프에_안_빠진다():
    bad = b"\xff\xd8" + b"\xff\xdb" + struct.pack(">H", 0) + b"\x00" * 40
    assert jpeg_dimensions(bad) is None


# ---- 곁표 -------------------------------------------------------------------

def test_곁표가_스트림에서_받은_크기를_낸다():
    with FixtureMjpegServer([make_jpeg(width=1088, height=1088)],
                            frame_interval_s=0.01, sidecar=True) as fx:
        p = MjpegPuller(fx.feed_url(), name="fx").start()
        try:
            assert _wait_for(lambda: p.sidecar()["frameWidth"] is not None)
            sc = p.sidecar()
            assert (sc["frameWidth"], sc["frameHeight"]) == (1088, 1088)
            assert p.stats()["frameSize"] == [1088, 1088]
        finally:
            p.stop()


def test_해상도가_바뀌면_곁표도_따라간다():
    """앱이 도중에 해상도를 바꾸면(APP-1) 그 순간부터 새 값이어야 한다.

    캘리브레이션은 이 값이 바뀌는 걸 보고 정착을 내린다. 여기서 안 따라가면
    옛 호모그래피가 새 화각 위에서 **맞는 척** 계속 산다.
    """
    frames = [make_jpeg(width=640, height=480), make_jpeg(width=1280, height=720)]
    with FixtureMjpegServer(frames, frame_interval_s=0.02, sidecar=True) as fx:
        p = MjpegPuller(fx.feed_url(), name="fx").start()
        try:
            assert _wait_for(
                lambda: p.sidecar()["frameWidth"] == 1280, timeout=8.0)
        finally:
            p.stop()


def test_프레임을_받기_전에는_None_이다():
    """모르는 걸 0 이나 기본값으로 채우지 않는다."""
    p = MjpegPuller("http://127.0.0.1:9/video", name="unit")
    sc = p.sidecar()
    assert sc["frameWidth"] is None
    assert sc["frameHeight"] is None
