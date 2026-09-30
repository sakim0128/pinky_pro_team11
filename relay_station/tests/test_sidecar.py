# -*- coding: utf-8 -*-
"""MCV-1B1/1B2/1B3 — 곁표는 만드는 것이 아니라 줍는 것이었다.

앱(외부 레포의 MJPEG 서버)은 프레임마다
`X-Capture-Clock` · `X-Publisher-Session` · `X-Frame-Seq` 를 실어 보내고,
가공(:18083)과 시청 중계(:18082)가 그걸 **파트 전체 바이트 그대로** 재발행한다.
게이트웨이의 `MjpegPuller` 만 JPEG 매직으로 잘라 그 헤더를 버리고 있었다.

여기서 고정하는 성질:

    MCVA-14  헤더가 오면 그 값을 그대로 낸다. 헤더가 없으면 receive 로 떨어지되 **수신은 계속**
    MCVA-15  순번이 건너뛰면 손실을 세어서 보고한다 (조용히 넘어가지 않는다)
    MCVA-16  relay-cam 은 receive 다 — V4L2 는 촬영 시각을 안 준다
    MCVA-17  중앙값과 지터를 **각각** 낸다 (시계 오차와 전송 지연을 한 이름으로 부르지 않는다)

폰·태블릿이 꺼져 있어도 픽스처로 전부 판정된다 — 그게 이 설계의 조건이었다.
"""
import io
import time

import pytest

from mjpeg_puller import (MjpegPuller, parse_part_headers, CLOCK_WINDOW_S,
                          H_CAPTURE_CLOCK, H_FRAME_SEQ, H_PUBLISHER_SESSION)
from source_registry import (Source, SourceRegistry, PULL, LOCAL, TRUSTED,
                             clock_alignment)
from fixtures.frames import make_jpeg
from fixtures.mjpeg_server import FixtureMjpegServer


def _frames(n):
    """프레임마다 색을 바꾼다 — 정지 화면과 구분되게(레지스트리 판정이 바이트 변화를 본다)."""
    return [make_jpeg(color=(40, 44, 40 + i * 30)) for i in range(n)]


class _Clock:
    """합성 시각용 시계.

    ⚠️ 표본을 합성 시각으로 넣고 통계를 실시계로 읽으면 **전부 창 밖으로 밀린다.**
    창이 있는 통계를 시험할 때는 넣는 쪽과 읽는 쪽의 시계가 같아야 한다.
    """

    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def _wait_for(predicate, timeout=8.0, interval=0.02):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


# ---- 파서 자체 --------------------------------------------------------------

def test_파트_헤더에서_곁표를_뽑는다():
    preamble = (b"\r\n--frame\r\nContent-Type: image/jpeg\r\n"
                b"Content-Length: 100\r\n"
                b"X-Capture-Clock: 1788900000123\r\n"
                b"X-Publisher-Session: sess-9\r\n"
                b"X-Frame-Seq: 42\r\n\r\n")
    got = parse_part_headers(preamble)
    assert got[H_CAPTURE_CLOCK] == "1788900000123"
    assert got[H_PUBLISHER_SESSION] == "sess-9"
    assert got[H_FRAME_SEQ] == "42"


def test_헤더_이름은_대소문자를_안_가린다():
    """상대 구현이 x-capture-clock 으로 써도 받는다 — 관용은 유지한다."""
    got = parse_part_headers(b"--frame\r\nx-CAPTURE-clock: 7\r\n\r\n")
    assert got[H_CAPTURE_CLOCK] == "7"


def test_곁표가_없으면_빈_dict_다():
    assert parse_part_headers(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n") == {}
    assert parse_part_headers(b"") == {}


# ---- MCVA-14: 헤더가 오면 쓴다 / 없으면 receive 로 떨어지되 수신은 계속 -----------

def test_MCVA14_곁표가_오면_captureClock_을_그대로_낸다():
    frames = _frames(3)
    with FixtureMjpegServer(frames, frame_interval_s=0.01, sidecar=True,
                            clock_skew_ms=0) as fx:
        p = MjpegPuller(fx.feed_url(), name="fx").start()
        try:
            assert _wait_for(lambda: p.stats()["frames"] >= 3)
            sc = p.sidecar()
            assert sc["source"] == "capture"
            assert isinstance(sc["captureClockMs"], int)
            assert sc["publisherSession"] == "fixture-session-1"
            assert sc["frameSeq"] >= 1
            assert sc["framesWithSidecar"] >= 3
        finally:
            p.stop()


def test_MCVA14_곁표가_없으면_receive_로_떨어지고_수신은_계속된다():
    """관용을 없애지 않는다. 곁표가 없다고 영상을 끊으면 그건 개선이 아니라 회귀다."""
    frames = _frames(3)
    with FixtureMjpegServer(frames, frame_interval_s=0.01, sidecar=False) as fx:
        p = MjpegPuller(fx.feed_url(), name="fx").start()
        try:
            assert _wait_for(lambda: p.stats()["frames"] >= 3), "곁표가 없다고 수신이 멈췄다"
            sc = p.sidecar()
            assert sc["source"] == "receive"
            assert sc["captureClockMs"] is None
            assert sc["offsetMedianMs"] is None
            assert sc["framesWithSidecar"] == 0
            jpeg, stamp, connected = p.get_latest_jpeg()
            assert connected is True and jpeg is not None and stamp > 0
        finally:
            p.stop()


# ---- MCVA-15: 순번이 건너뛰면 세어서 보고 -------------------------------------

def test_MCVA15_순번이_건너뛰면_손실을_세어_보고한다():
    frames = _frames(2)
    with FixtureMjpegServer(frames, frame_interval_s=0.01, sidecar=True,
                            seq_jump_after=2, seq_jump_by=3, loop=True) as fx:
        p = MjpegPuller(fx.feed_url(), name="fx").start()
        try:
            assert _wait_for(lambda: p.sidecar()["framesLost"] >= 3, timeout=8.0), \
                "순번이 건너뛰었는데 조용히 넘어갔다"
            sc = p.sidecar()
            assert sc["seqGaps"] >= 1
            assert sc["framesLost"] >= 3
        finally:
            p.stop()


def test_연속_순번이면_손실이_0_이다():
    """전제 확인 — 정상일 때 0 이 아니면 위 테스트가 무의미하다."""
    frames = _frames(3)
    with FixtureMjpegServer(frames, frame_interval_s=0.01, sidecar=True) as fx:
        p = MjpegPuller(fx.feed_url(), name="fx").start()
        try:
            assert _wait_for(lambda: p.stats()["frames"] >= 6)
            assert p.sidecar()["framesLost"] == 0
            assert p.sidecar()["seqGaps"] == 0
        finally:
            p.stop()


def test_세션이_바뀌면_순번_리셋을_손실로_세지_않는다():
    """앱이 재시작하면 순번이 1 로 돌아간다. 그걸 손실로 세면 거짓 경보가 된다."""
    p = MjpegPuller("http://127.0.0.1:9/video", name="unit")
    now = 1000.0
    p._note_sidecar({H_FRAME_SEQ: "50", H_PUBLISHER_SESSION: "sess-A"}, now)
    p._note_sidecar({H_FRAME_SEQ: "1", H_PUBLISHER_SESSION: "sess-B"}, now + 0.1)
    assert p.sidecar()["framesLost"] == 0
    assert p.sidecar()["publisherSession"] == "sess-B"


def test_같은_세션에서_건너뛰면_그_수만큼_센다():
    p = MjpegPuller("http://127.0.0.1:9/video", name="unit")
    now = 1000.0
    p._note_sidecar({H_FRAME_SEQ: "10", H_PUBLISHER_SESSION: "s"}, now)
    p._note_sidecar({H_FRAME_SEQ: "13", H_PUBLISHER_SESSION: "s"}, now + 0.1)
    sc = p.sidecar()
    assert sc["framesLost"] == 2, "10 -> 13 이면 11·12 두 장을 잃었다"
    assert sc["seqGaps"] == 1


def test_숫자가_아닌_헤더는_통계를_오염시키지_않는다():
    p = MjpegPuller("http://127.0.0.1:9/video", name="unit")
    p._note_sidecar({H_CAPTURE_CLOCK: "not-a-number", H_FRAME_SEQ: "??"}, 1000.0)
    sc = p.sidecar()
    assert sc["source"] == "receive"
    assert sc["samples"] == 0
    assert sc["frameSeq"] is None


# ---- MCVA-17: 중앙값과 지터를 각각 ---------------------------------------------

def test_MCVA17_중앙값과_지터를_각각_낸다():
    """중앙값 = 시계오차 + 전송지연의 합. 지터 = 그 표본의 흔들림.

    한 숫자로 뭉치면 '시계가 3초 틀렸다'와 '네트워크가 3초 밀렸다'를 구분 못 한다.
    """
    clk = _Clock(1000.0)
    p = MjpegPuller("http://127.0.0.1:9/video", name="unit", clock=clk)
    # 기기 시계가 중계보다 500 ms 늦다 + 전송 지연이 ±20 ms 로 흔들린다
    base = 1000.0
    for i, jitter in enumerate((0.0, 0.02, -0.02, 0.0, 0.02)):
        now = base + i * 0.1
        clk.t = now
        capture_ms = int((now - 0.5 + jitter) * 1000.0)
        p._note_sidecar({H_CAPTURE_CLOCK: str(capture_ms)}, now)
    sc = p.sidecar()
    assert sc["samples"] == 5
    assert sc["offsetMedianMs"] == pytest.approx(500.0, abs=25.0)
    assert sc["offsetJitterMs"] is not None
    assert sc["offsetJitterMs"] < 40.0, "지터가 중앙값만큼 크면 두 값을 분리한 의미가 없다"
    assert sc["offsetMedianMs"] != sc["offsetJitterMs"]


def test_창_밖의_표본은_버린다():
    """창이 없으면 아침의 오차가 저녁 판정에 남는다."""
    clk = _Clock(1000.0)
    p = MjpegPuller("http://127.0.0.1:9/video", name="unit", clock=clk)
    p._note_sidecar({H_CAPTURE_CLOCK: str(int((1000.0 - 5.0) * 1000))}, 1000.0)
    assert p.sidecar()["samples"] == 1
    clk.t = 1000.0 + CLOCK_WINDOW_S + 5.0
    assert p.sidecar()["samples"] == 0, "창 밖 표본이 남아 있다"


# ---- MCVA-16: relay-cam 은 receive ----------------------------------------------

def test_MCVA16_sidecar_가_없는_소스는_receive_다():
    src = Source("gazebo", lambda: (b"x", 1.0, True), PULL, TRUSTED)
    assert src.status()["clock"] == {"source": "receive"}


def test_MCVA16_local_camera_는_receive_로_표기한다():
    """V4L2 는 촬영 시각을 안 준다. capture 라고 부르면 없는 정확도를 주장하는 것이다."""
    from local_camera import LocalCameraSource
    cam = LocalCameraSource(device=0)
    info = cam.sidecar()
    assert info["source"] == "receive"
    assert info["captureClockMs"] is None
    src = Source("relay-cam", cam.get_latest_jpeg, LOCAL, TRUSTED,
                 probe=cam.probe, sidecar=cam.sidecar)
    assert src.status()["clock"]["source"] == "receive"


def test_곁표가_예외를_던져도_소스는_산다():
    def boom():
        raise RuntimeError("곁표 계산이 깨졌다")
    src = Source("phone", lambda: (b"x", 1.0, True), PULL, TRUSTED, sidecar=boom)
    assert src.status()["clock"] == {"source": "receive"}
    assert src.status()["connected"] is True


# ---- MCVA-13: 쌍의 시각 오차 -----------------------------------------------------

def _live(sid, sidecar):
    return Source(sid, lambda: (b"\xff\xd8\xff\xff\xd9", 1000.0, True), PULL, TRUSTED,
                  sidecar=sidecar)


def test_MCVA13_두_소스의_시각_오차를_쌍으로_낸다():
    reg = SourceRegistry()
    reg.register(_live("phone", lambda: {"source": "capture", "offsetMedianMs": 520.0,
                                         "offsetJitterMs": 12.0, "samples": 30}))
    reg.register(_live("tablet-relay", lambda: {"source": "capture", "offsetMedianMs": 180.0,
                                                "offsetJitterMs": 8.0, "samples": 30}))
    got = clock_alignment(reg)
    assert len(got["pairs"]) == 1
    pair = got["pairs"][0]
    assert {pair["a"], pair["b"]} == {"phone", "tablet-relay"}
    assert pair["offsetDiffMs"] == pytest.approx(340.0)
    assert pair["jitterMs"]["phone"] == 12.0
    assert got["unavailable"] == []


def test_MCVA13_촬영시각이_없는_쌍은_판정불가로_낸다():
    """숫자를 지어내지 않는다 — 없는 정확도를 주장하는 것이 이 프로젝트의 반복 실패였다."""
    reg = SourceRegistry()
    reg.register(_live("phone", lambda: {"source": "capture", "offsetMedianMs": 500.0,
                                         "offsetJitterMs": 10.0, "samples": 30}))
    reg.register(_live("relay-cam", lambda: {"source": "receive", "offsetMedianMs": None,
                                             "offsetJitterMs": None, "samples": 0}))
    got = clock_alignment(reg)
    assert got["pairs"] == []
    assert len(got["unavailable"]) == 1
    assert got["unavailable"][0]["reason"] == "NO_CAPTURE_CLOCK"


def test_MCVA13_판정에_사람용_문구가_없다():
    reg = SourceRegistry()
    reg.register(_live("phone", lambda: {"source": "receive"}))
    reg.register(_live("tablet-relay", lambda: {"source": "receive"}))
    got = clock_alignment(reg)
    for item in got["unavailable"]:
        assert set(item.keys()) == {"a", "b", "reason"}
        assert " " not in item["reason"], "reason 은 코드다. 문장이 아니다"


def test_소스가_하나면_쌍이_없다():
    reg = SourceRegistry()
    reg.register(_live("phone", lambda: {"source": "capture", "offsetMedianMs": 1.0,
                                         "offsetJitterMs": 1.0, "samples": 1}))
    got = clock_alignment(reg)
    assert got["pairs"] == [] and got["unavailable"] == []
    assert got["liveIds"] == ["phone"]


# ---- 통합: 픽스처 두 대의 시계가 어긋나면 그 차이가 나온다 -------------------------

def test_두_픽스처의_시계_차이가_쌍_오차로_나온다():
    """앱 없이, 기기 없이 MCVA-13 을 판정한다."""
    frames = _frames(2)
    with FixtureMjpegServer(frames, frame_interval_s=0.01, sidecar=True,
                            clock_skew_ms=600, session="a") as fx_a, \
         FixtureMjpegServer(frames, frame_interval_s=0.01, sidecar=True,
                            clock_skew_ms=100, session="b") as fx_b:
        pa = MjpegPuller(fx_a.feed_url(), name="a").start()
        pb = MjpegPuller(fx_b.feed_url(), name="b").start()
        try:
            assert _wait_for(lambda: pa.sidecar()["samples"] >= 5 and
                             pb.sidecar()["samples"] >= 5, timeout=10.0)
            reg = SourceRegistry()
            reg.register(Source("phone", pa.get_latest_jpeg, PULL, TRUSTED,
                                sidecar=pa.sidecar))
            reg.register(Source("tablet-relay", pb.get_latest_jpeg, PULL, TRUSTED,
                                sidecar=pb.sidecar))
            got = clock_alignment(reg)
            assert len(got["pairs"]) == 1, got
            diff = abs(got["pairs"][0]["offsetDiffMs"])
            assert diff == pytest.approx(500.0, abs=120.0), \
                "600ms - 100ms = 500ms 여야 한다: %r" % (got,)
        finally:
            pa.stop(); pb.stop()
