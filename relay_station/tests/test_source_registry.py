# -*- coding: utf-8 -*-
"""source_registry (MCV-1A) 테스트.

수락:
  MCVA-11  여러 소스가 동시에 fps>0 이고 최근 프레임을 갖는다
  MCVA-12  한 소스를 죽여도 나머지 소스가 계속 증가한다(독립성)
  결정 1A  trust 등급 — untrusted 소스는 trusted_ids 에서 빠진다
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # gateway_web
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))                    # 이 폴더

import pytest

from source_registry import (PULL, PUSH, ROS, TRUSTED, UNTRUSTED, Source,
                             SourceRegistry)

JPEG_SOI = b"\xff\xd8\xff"


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_provider(frames):
    """호출할 때마다 frames 에서 하나씩 내주는 provider. 소진되면 (None,0,False)."""
    state = {"i": 0, "stamp": 1000.0}

    def provider():
        i = state["i"]
        if i >= len(frames):
            return None, 0.0, False
        state["i"] += 1
        state["stamp"] += 0.05
        return frames[i], state["stamp"], True

    return provider


def const_provider(jpeg, connected=True):
    def provider():
        return jpeg, 1234.5, connected
    return provider


def test_source_는_provider_에서_프레임을_뽑는다():
    clock = FakeClock()
    s = Source("cam", const_provider(JPEG_SOI + b"x"), PUSH, UNTRUSTED, clock=clock)
    jpeg, stamp, connected = s.latest_jpeg()
    assert jpeg.startswith(JPEG_SOI)
    assert connected is True


def test_trust_기본값은_없다_명시해야_한다():
    with pytest.raises(ValueError):
        Source("c", const_provider(b""), PUSH, "maybe")


def test_transport_도_검증된다():
    with pytest.raises(ValueError):
        Source("c", const_provider(b""), "carrier-pigeon", TRUSTED)


def test_provider_가_콜러블이_아니면_거부():
    with pytest.raises(TypeError):
        Source("c", b"not callable", PUSH, TRUSTED)


def test_untrusted_소스는_trusted_목록에서_빠진다():
    """결정 1A 의 핵심 — 관측은 trusted 만 입력으로 쓴다."""
    reg = SourceRegistry()
    reg.register(Source("phone", const_provider(JPEG_SOI), PUSH, UNTRUSTED))
    reg.register(Source("overhead", const_provider(JPEG_SOI), ROS, TRUSTED))
    reg.register(Source("relay-cam", const_provider(JPEG_SOI), PULL, TRUSTED))
    assert reg.trusted_ids() == ["overhead", "relay-cam"]
    assert "phone" in reg.ids()
    assert "phone" not in reg.trusted_ids()


def test_push_업로드_소스는_관측에_안_쓰인다_회귀():
    """가짜 프레임 주입이 pose 보정 경로에 닿지 않는다는 성질을 고정한다."""
    reg = SourceRegistry()
    reg.register(Source("uploaded", const_provider(JPEG_SOI), PUSH, UNTRUSTED))
    for sid in reg.trusted_ids():
        assert reg.get(sid).transport != PUSH


def test_id_중복_등록은_거부():
    reg = SourceRegistry()
    reg.register(Source("a", const_provider(b""), PUSH, UNTRUSTED))
    with pytest.raises(ValueError):
        reg.register(Source("a", const_provider(b""), PULL, TRUSTED))


def test_resolve_는_src_없으면_기본소스():
    reg = SourceRegistry()
    reg.register(Source("first", const_provider(JPEG_SOI + b"1"), PUSH, UNTRUSTED))
    reg.register(Source("second", const_provider(JPEG_SOI + b"2"), PULL, TRUSTED))
    assert reg.resolve(None).id == "first"
    assert reg.resolve("second").id == "second"
    assert reg.resolve("nonexistent") is None


def test_기본소스는_명시로_바꿀_수_있다():
    reg = SourceRegistry()
    reg.register(Source("a", const_provider(b""), PUSH, UNTRUSTED))
    reg.register(Source("b", const_provider(b""), PULL, TRUSTED), default=True)
    assert reg.default().id == "b"


def test_MCVA_11_여러소스_동시에_fps_와_최근프레임():
    clock = FakeClock()
    reg = SourceRegistry()
    for sid in ("s1", "s2", "s3"):
        reg.register(Source(sid, const_provider(JPEG_SOI), PUSH, UNTRUSTED, clock=clock))
    # 각 소스를 1초 창에서 여러 번 뽑는다.
    for _ in range(15):
        for sid in ("s1", "s2", "s3"):
            reg.get(sid).latest_jpeg()
        clock.advance(0.1)
    clock.advance(0.1)
    for sid in ("s1", "s2", "s3"):
        reg.get(sid).latest_jpeg()  # fps 창을 닫는 한 번 더
    pub = reg.sources_public()
    assert pub["default"] == "s1"
    for st in pub["sources"]:
        assert st["fps"] > 0, "%s fps=%s" % (st["id"], st["fps"])
        assert st["ageOfLastFrameSec"] is not None
        assert st["ageOfLastFrameSec"] <= 1.0


def test_MCVA_12_한소스_죽어도_나머지_증가():
    """s2 의 provider 가 마르면 s1·s3 는 계속 증가해야 한다(팬아웃 독립성)."""
    clock = FakeClock()
    reg = SourceRegistry()
    reg.register(Source("s1", const_provider(JPEG_SOI), PUSH, UNTRUSTED, clock=clock))
    reg.register(Source("s2", make_provider([JPEG_SOI, JPEG_SOI]), PUSH, UNTRUSTED, clock=clock))
    reg.register(Source("s3", const_provider(JPEG_SOI), PUSH, UNTRUSTED, clock=clock))

    served = {"s1": 0, "s3": 0}
    for _ in range(10):
        for sid in ("s1", "s2", "s3"):
            j, _, _ = reg.get(sid).latest_jpeg()
        clock.advance(0.1)

    st = {s["id"]: s for s in reg.sources_public()["sources"]}
    # s2 는 2프레임 뒤 말라 stale/disconnected
    assert st["s2"]["framesServed"] == 2
    assert st["s2"]["connected"] is False
    # s1·s3 는 10번씩 계속
    assert st["s1"]["framesServed"] == 10
    assert st["s3"]["framesServed"] == 10


def test_age_는_아무도_안뽑으면_None():
    clock = FakeClock()
    s = Source("idle", const_provider(JPEG_SOI), PUSH, UNTRUSTED, clock=clock)
    assert s.age_of_last_frame() is None
    s.latest_jpeg()
    clock.advance(2.5)
    assert abs(s.age_of_last_frame() - 2.5) < 1e-6


def test_sources_public_에_자격증명_흔적이_없다():
    """provider 안에 URL·토큰이 있어도 공개 목록엔 안 나온다."""
    reg = SourceRegistry()

    def secret_provider():
        # 클로저 안에 비밀이 있어도
        _url = "rtsp://user:pass@10.0.0.1/stream"  # noqa: F841
        return JPEG_SOI, 1.0, True

    reg.register(Source("sec", secret_provider, PULL, TRUSTED, label="원격캠"))
    import json
    blob = json.dumps(reg.sources_public())
    assert "pass" not in blob and "rtsp" not in blob and "10.0.0.1" not in blob
