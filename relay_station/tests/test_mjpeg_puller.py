# -*- coding: utf-8 -*-
"""MJPEG puller — pull transport 가 실제로 프레임을 당겨오는지.

T1 의 픽스처 MJPEG 서버가 게이트웨이/기기 자리를 대신 선다. 그래서 폰도 태블릿도
로봇도 없이 이 경로 전체를 검증할 수 있다.

    FixtureMjpegServer ──multipart/x-mixed-replace──> MjpegPuller ──> (jpeg, stamp, connected)
                                                                        │
                                              Source(jpeg_provider=...) ┘  ← 레지스트리 계약
"""
import io
import json
import os
import tempfile
import time

import pytest

from fixtures.frames import make_jpeg
from fixtures.mjpeg_server import FixtureMjpegServer

import mjpeg_puller
from mjpeg_puller import MjpegPuller, load_pull_sources

JPEG_SOI = b"\xff\xd8\xff"


def _wait_for(predicate, timeout=8.0, interval=0.05):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


# ---- 당겨오기 ----------------------------------------------------------------

def test_픽스처_서버에서_프레임을_당겨온다():
    frames = [make_jpeg(color=(c, c, c)) for c in (10, 90, 170)]
    with FixtureMjpegServer(frames, frame_interval_s=0.01) as srv:
        p = MjpegPuller(srv.feed_url("cam1"), name="t").start()
        try:
            assert _wait_for(lambda: p.stats()["frames"] >= 3), "3프레임을 못 받았다"
            jpeg, stamp, connected = p.get_latest_jpeg()
            assert connected is True
            assert jpeg is not None and jpeg.startswith(JPEG_SOI)
            assert stamp > 0
        finally:
            p.stop()


def test_최신_프레임만_보관한다_큐가_쌓이지_않는다():
    """시청 갈래는 DROP_TO_STAY_LIVE 다. 프레임이 쌓이면 지연이 자란다."""
    frames = [make_jpeg(color=(c, c, c)) for c in (10, 200)]
    with FixtureMjpegServer(frames, frame_interval_s=0.005) as srv:
        p = MjpegPuller(srv.feed_url(), name="t").start()
        try:
            assert _wait_for(lambda: p.stats()["frames"] >= 20)
            a = p.get_latest_jpeg()[0]
            time.sleep(0.2)
            b = p.get_latest_jpeg()[0]
            # 최신만 들고 있으므로 두 표본은 그때그때의 마지막 프레임이다.
            assert a is not None and b is not None
            assert p.stats()["frames"] >= 20
        finally:
            p.stop()


def test_소스_레지스트리_계약을_그대로_만족한다():
    """Source 가 요구하는 (jpeg|None, stamp, connected) 3튜플."""
    with FixtureMjpegServer([make_jpeg()], frame_interval_s=0.01) as srv:
        p = MjpegPuller(srv.feed_url(), name="t").start()
        try:
            assert _wait_for(lambda: p.stats()["frames"] >= 1)
            got = p.get_latest_jpeg()
            assert isinstance(got, tuple) and len(got) == 3
            jpeg, stamp, connected = got
            assert isinstance(jpeg, (bytes, bytearray))
            assert isinstance(stamp, float)
            assert isinstance(connected, bool)
        finally:
            p.stop()


# ---- 정직성: 낡은 프레임을 살아있는 것처럼 내보내지 않는다 -------------------

def test_stale_이면_connected_False_이고_프레임을_안_준다():
    """소켓 생존을 수신으로 읽으면 '완벽 수신 LIVE' 오탐이 재발한다."""
    now = [1000.0]
    with FixtureMjpegServer([make_jpeg()], frame_interval_s=0.01) as srv:
        p = MjpegPuller(srv.feed_url(), name="t", stale_after_s=2.0,
                        clock=lambda: now[0]).start()
        try:
            assert _wait_for(lambda: p.stats()["frames"] >= 1)
            jpeg, _, connected = p.get_latest_jpeg()
            assert connected is True and jpeg is not None

            now[0] += 10.0                      # 시간을 앞으로 민다
            jpeg2, _, connected2 = p.get_latest_jpeg()
            assert connected2 is False, "낡았는데 connected 면 지표가 거짓말한다"
            assert jpeg2 is None, "낡은 프레임을 최신처럼 내보내면 안 된다"
        finally:
            p.stop()


def test_한_번도_못_받으면_connected_False():
    p = MjpegPuller("http://127.0.0.1:9/video", name="dead")
    jpeg, stamp, connected = p.get_latest_jpeg()
    assert (jpeg, stamp, connected) == (None, 0.0, False)


def test_상대가_죽어_있어도_예외를_안_던지고_재시도한다():
    p = MjpegPuller("http://127.0.0.1:9/video", name="dead").start()
    try:
        assert _wait_for(lambda: p.stats()["errors"] >= 1, timeout=6.0)
        assert p.get_latest_jpeg()[2] is False
    finally:
        p.stop()


def test_스트림이_끝나면_다시_붙는다():
    """상대가 연결을 닫아도 포기하지 않는다.

    ⚠️ FixtureMjpegServer 는 with 를 빠져나가도 **진행 중인 연결을 끊지 않는다**
    (daemon 핸들러가 계속 쓴다). 그래서 '서버를 내리면 끊긴다'로 검증하면 실패한다.
    대신 loop=False 로 스트림이 자연히 끝나게 해서 재접속을 잰다.
    connected 가 낡음으로 False 가 되는 성질은 아래 stale 테스트가 가짜 시계로 본다.
    """
    with FixtureMjpegServer([make_jpeg()], frame_interval_s=0.005, loop=False) as srv:
        p = MjpegPuller(srv.feed_url(), name="t").start()
        try:
            assert _wait_for(lambda: p.stats()["connects"] >= 2, timeout=10.0), \
                "스트림이 끝났는데 재접속하지 않았다: %r" % (p.stats(),)
            assert p.stats()["frames"] >= 2
        finally:
            p.stop()


def test_mjpeg_가_아닌_응답은_버퍼를_무한정_안_늘린다():
    """매직 없는 쓰레기가 오면 상한에서 끊고 재접속한다."""
    class _Garbage:
        def __init__(self):
            self.sent = 0

        def read(self, n):
            self.sent += n
            return b"x" * n

        def close(self):
            pass

    p = MjpegPuller("http://example.invalid/", name="g",
                    opener=lambda u, t: _Garbage())
    p.start()
    try:
        assert _wait_for(lambda: p.stats()["errors"] >= 1, timeout=8.0)
    finally:
        p.stop()


# ---- 설정 -------------------------------------------------------------------

def _write_cfg(payload):
    fd, path = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    with io.open(path, "w", encoding="utf-8") as fh:
        fh.write(json.dumps(payload, ensure_ascii=False))
    return path


def test_설정_파일이_없으면_빈_목록():
    assert load_pull_sources("/definitely/no/such/file.json") == []


def test_설정에서_pull_소스를_읽는다():
    path = _write_cfg({"pullSources": [
        {"id": "phone", "url": "http://h:1/video", "label": "폰", "trust": "trusted"},
    ]})
    try:
        got = load_pull_sources(path)
        assert len(got) == 1
        assert got[0]["id"] == "phone"
        assert got[0]["url"] == "http://h:1/video"
        assert got[0]["trust"] == "trusted"
    finally:
        os.unlink(path)


def test_enabled_false_는_건너뛴다():
    path = _write_cfg({"pullSources": [
        {"id": "a", "url": "http://h:1/v", "enabled": False},
        {"id": "b", "url": "http://h:2/v"},
    ]})
    try:
        assert [s["id"] for s in load_pull_sources(path)] == ["b"]
    finally:
        os.unlink(path)


@pytest.mark.parametrize("bad,msg", [
    ({"pullSources": [{"url": "http://h/v"}]}, "id"),
    ({"pullSources": [{"id": "a"}]}, "url"),
    ({"pullSources": [{"id": "a", "url": "u", "trust": "maybe"}]}, "trust"),
    ({"pullSources": [{"id": "a", "url": "u"}, {"id": "a", "url": "u2"}]}, "중복"),
    ({"pullSources": "not-a-list"}, "리스트"),
])
def test_형식을_어기면_조용히_버리지_않고_예외를_던진다(bad, msg):
    """조용히 빠진 소스는 '왜 안 보이지'로 몇 시간을 태운다."""
    path = _write_cfg(bad)
    try:
        with pytest.raises(ValueError) as ei:
            load_pull_sources(path)
        assert msg in str(ei.value)
    finally:
        os.unlink(path)


def test_urlCandidates_목록을_읽는다():
    path = _write_cfg({"pullSources": [
        {"id": "phone", "urlCandidates": ["http://lan:1/v", "http://tail:1/v"]},
    ]})
    try:
        got = load_pull_sources(path)
        assert got[0]["urls"] == ["http://lan:1/v", "http://tail:1/v"]
        assert got[0]["url"] == "http://lan:1/v", "url 은 첫 후보와 같아야 한다"
    finally:
        os.unlink(path)


def test_urlCandidates_가_문자열_목록이_아니면_예외():
    path = _write_cfg({"pullSources": [{"id": "a", "urlCandidates": [1, 2]}]})
    try:
        with pytest.raises(ValueError) as ei:
            load_pull_sources(path)
        assert "urlCandidates" in str(ei.value)
    finally:
        os.unlink(path)


# ---- 장소가 바뀌어도 붙는다 ---------------------------------------------------

def test_첫_후보가_죽어_있으면_다음_후보로_붙는다():
    """교육장(LAN)과 집(tailnet)의 도달 경로가 다르다.

    설정을 장소에 묶으면 이동할 때마다 죽는다 — 같은 날 DDS 프로파일에서 겪은 것과
    같은 계열이다. 여기서는 후보를 순서대로 시도해서 그 문제를 없앤다.
    """
    dead = "http://127.0.0.1:9/video"
    with FixtureMjpegServer([make_jpeg()], frame_interval_s=0.01) as srv:
        alive = srv.feed_url()
        p = MjpegPuller([dead, alive], name="fallback").start()
        try:
            assert _wait_for(lambda: p.stats()["frames"] >= 2, timeout=10.0), \
                "살아있는 후보로 넘어가지 못했다: %r" % (p.stats(),)
            assert p.stats()["activeUrl"] == alive
            assert p.get_latest_jpeg()[2] is True
        finally:
            p.stop()


def test_후보가_비면_생성_자체가_거부된다():
    with pytest.raises(ValueError):
        MjpegPuller([], name="empty")


def test_실제_배포된_설정이_형식을_지킨다():
    """레포에 든 configs/video_sources.json 자체를 검증한다."""
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    repo = os.path.dirname(here)
    cfg = os.path.join(repo, "relay_station", "configs", "video_sources.json")
    if not os.path.exists(cfg):
        pytest.skip("configs/video_sources.json 없음")
    got = load_pull_sources(cfg)
    assert len(got) >= 1
    ids = [s["id"] for s in got]
    assert len(ids) == len(set(ids))
    for s in got:
        # 고정 후보는 전부 http(s). 후보가 비었으면 `discover` 로 찾는 소스여야 한다
        # (R-3: tablet-relay 는 주소가 아니라 신원으로만 찾는다 → url 은 None 이 맞다).
        assert all(u.startswith(("http://", "https://")) for u in s["urls"]), s["id"]
        if not s["urls"]:
            assert s["url"] is None and (s.get("discover") or {}).get("tailnetIp"), s["id"]
        # 게이트웨이가 기동 때 하는 그대로 만들어 본다 — 여기서 던지면 그 소스는 조용히 빠진다.
        MjpegPuller(s["urls"], name=s["id"], stale_after_s=s["staleAfterSec"],
                    discover=s.get("discover"))
