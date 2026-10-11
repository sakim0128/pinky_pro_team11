# -*- coding: utf-8 -*-
"""이동 기기의 주소를 **박는 대신 찾는다**.

🔴 왜 생겼나 (2026-09-19):

    소스 10개가 **전부 미접속**이었다. `phone`·`phone2` 의 후보가 핫스팟
    `203.0.113.2x` **하나뿐**인데 장비가 현장 LAN(`198.51.100.x`)으로 옮겨 갔기 때문이다.

⭐ 후보를 하나 더 넣는 것이 답이 아니다. `configs/video_sources.json` 의 `_why` 가
   이미 그렇게 적어 뒀다 — *"사이트가 늘 때마다 이 파일이 자란다. **근본 해결은
   발견이다** — tailscale 이 피어의 LAN endpoint 를 알고 있다."* `tablet-relay` 는
   그때 이미 후보가 셋이었다.

## 중계에서 실측한 것 (이 시험이 흉내 내는 것)

    tailnet 100.64.0.35  ->  via 198.51.100.17:41801  ->  :18086 open 3 ms
    tailnet 100.64.0.6    ->  via 198.51.100.2:47522   ->  :18086 refused (앱 꺼짐)
    tailnet 100.64.0.7    ->  timed out (기기 오프라인)

⚠️ **tailnet 주소로 직접 TCP 를 걸면 안 된다.** 중계는 shared-in 노드라 `tailscale ping`
   은 되는데 `:18086` TCP 는 timeout 이다. 반드시 LAN 주소로 **바꿔서** 붙는다.

⚠️ 발견은 **fail-open** 이다 — tailscale 이 없는 소비자(도커 복제본)도 그대로 돌아야
   하므로, 못 찾으면 고정 후보로 떨어진다. 이 시험은 그 폴백도 고정한다.
"""
import pytest

import mjpeg_puller as MP


# 2026-09-19 에 중계에서 실제로 나온 출력. **손으로 지어내지 않았다.**
PONG_DIRECT = ("pong from device-of-shared-to-user (100.64.0.35) "
               "via 198.51.100.17:41801 in 267ms")
PONG_Y700 = ("pong from device-of-shared-to-user (100.64.0.6) "
             "via 198.51.100.2:47522 in 5ms")
TIMED_OUT = 'ping "100.64.0.7" timed out'
VIA_DERP = "pong from x (100.1.2.3) via DERP(tok) in 40ms"


def runner_of(text):
    """`tailscale` 자리를 대신한다 — CI 에는 tailscale 이 없다."""
    def _run(argv, timeout_s):
        assert argv[0] == "tailscale" and argv[1] == "ping", argv
        return text
    return _run


# ---- 대조군 먼저 ---------------------------------------------------------------

def test_실제로_나온_출력을_파싱한다():
    """🔴🔴 박아 둔 기대값 대조. 정규식을 고쳐도 **현장 출력**으로 판정한다."""
    assert MP.tailscale_lan_endpoint("100.64.0.35",
                                     runner=runner_of(PONG_DIRECT)) == "198.51.100.17"
    assert MP.tailscale_lan_endpoint("100.64.0.6",
                                     runner=runner_of(PONG_Y700)) == "198.51.100.2"


def test_해석기가_실제로_불린다():
    """🔴 뮤테이션 표적 — 배선을 지우면 러너가 안 불려 여기서 빨개진다."""
    calls = []

    def _run(argv, timeout_s):
        calls.append(argv)
        return PONG_DIRECT

    p = MP.MjpegPuller([], name="t", discover={"tailnetIp": "100.64.0.35"},
                       resolver=_run)
    p._candidates()
    assert calls, "해석기가 한 번도 안 불렸다"
    assert "100.64.0.35" in calls[0]


# ---- 못 찾는 경우를 구분한다 -------------------------------------------------------

def test_DERP_경유면_LAN_주소가_없다():
    """⭐ DERP 는 중계 서버 경유다 — LAN 주소가 아니다. 지어내면 안 된다."""
    assert MP.tailscale_lan_endpoint("100.1.2.3", runner=runner_of(VIA_DERP)) is None


def test_기기가_없으면_None():
    assert MP.tailscale_lan_endpoint("100.64.0.7", runner=runner_of(TIMED_OUT)) is None


def test_러너가_터져도_죽지_않는다():
    """발견 실패는 치명적이지 않다 — 고정 후보가 있다."""
    def _boom(argv, timeout_s):
        raise OSError("tailscale 없음")
    assert MP.tailscale_lan_endpoint("100.1.1.1", runner=_boom) is None


def test_주소가_없으면_묻지도_않는다():
    def _never(argv, timeout_s):
        raise AssertionError("부르면 안 된다")
    assert MP.tailscale_lan_endpoint(None, runner=_never) is None
    assert MP.discovered_url(None, runner=_never) is None
    assert MP.discovered_url({}, runner=_never) is None


# ---- URL 조립 -------------------------------------------------------------------

def test_기본_포트와_경로():
    u = MP.discovered_url({"tailnetIp": "100.64.0.35"}, runner=runner_of(PONG_DIRECT))
    assert u == "http://198.51.100.17:18086/processed"


def test_포트와_경로를_바꿀_수_있다():
    u = MP.discovered_url({"tailnetIp": "100.64.0.35", "port": 18099, "path": "video"},
                          runner=runner_of(PONG_DIRECT))
    assert u == "http://198.51.100.17:18099/video"          # 앞 / 없어도 붙는다


# ---- 후보 순서와 폴백 ------------------------------------------------------------

def test_찾은_주소를_먼저_시도한다():
    """⭐ 고정 후보는 **지난 사이트** 의 주소다. 먼저 걸면 매번 timeout 을 기다린다."""
    p = MP.MjpegPuller(["http://203.0.113.22:18086/processed"], name="t",
                       discover={"tailnetIp": "100.64.0.35"},
                       resolver=runner_of(PONG_DIRECT))
    c = p._candidates()
    assert c[0] == "http://198.51.100.17:18086/processed"
    assert "http://203.0.113.22:18086/processed" in c


def test_발견이_실패하면_고정_후보로_떨어진다():
    """🔴 fail-open. tailscale 이 없는 자리에서도 그대로 돌아야 한다."""
    p = MP.MjpegPuller(["http://203.0.113.22:18086/processed"], name="t",
                       discover={"tailnetIp": "100.64.0.7"},
                       resolver=runner_of(TIMED_OUT))
    assert p._candidates() == ["http://203.0.113.22:18086/processed"]


def test_발견도_후보도_없으면_빈_목록이다():
    """⭐ 예외를 던지지 않는다 — 기기가 잠깐 꺼진 것일 수 있다. 기다렸다 다시 찾는다."""
    p = MP.MjpegPuller([], name="t", discover={"tailnetIp": "100.64.0.7"},
                       resolver=runner_of(TIMED_OUT))
    assert p._candidates() == []


def test_고정_후보도_discover_도_없으면_만들_수_없다():
    with pytest.raises(ValueError):
        MP.MjpegPuller([], name="t")


# ---- 다시 찾는 시점 --------------------------------------------------------------

def test_TTL_안에서는_다시_안_묻는다():
    """짧으면 `tailscale ping` 을 쉴 새 없이 부른다."""
    calls = []

    def _run(argv, timeout_s):
        calls.append(1)
        return PONG_DIRECT

    p = MP.MjpegPuller([], name="t", discover={"tailnetIp": "100.64.0.35"},
                       resolver=_run, clock=lambda: 1000.0)
    p._candidates()
    p._candidates()
    p._candidates()
    assert len(calls) == 1, "TTL 안인데 %d 번 물었다" % len(calls)


def test_TTL_이_지나면_다시_묻는다():
    calls = []
    now = [1000.0]

    def _run(argv, timeout_s):
        calls.append(1)
        return PONG_DIRECT

    p = MP.MjpegPuller([], name="t", discover={"tailnetIp": "100.64.0.35"},
                       resolver=_run, clock=lambda: now[0])
    p._candidates()
    now[0] += MP.DISCOVER_TTL_S + 1
    p._candidates()
    assert len(calls) == 2


def test_강제하면_TTL_을_무시한다():
    """한 바퀴 다 실패했으면 주소가 낡은 것일 수 있다 — 그때 강제로 다시 찾는다."""
    calls = []

    def _run(argv, timeout_s):
        calls.append(1)
        return PONG_DIRECT

    p = MP.MjpegPuller([], name="t", discover={"tailnetIp": "100.64.0.35"},
                       resolver=_run, clock=lambda: 1000.0)
    p._candidates()
    p._candidates(force_discover=True)
    assert len(calls) == 2


# ---- 상태를 숨기지 않는다 ----------------------------------------------------------

def test_곁표가_발견_상태를_드러낸다():
    """⭐ `discoveredUrl` 이 None 인데 안 붙으면 **주소를 못 찾은 것**이고,
       주소는 찾았는데 안 붙으면 **앱이 꺼진 것**이다. 처방이 다르다."""
    p = MP.MjpegPuller([], name="t", discover={"tailnetIp": "100.64.0.35"},
                       resolver=runner_of(PONG_DIRECT))
    p._candidates()
    st = p.stats()
    assert st["discoveredUrl"] == "http://198.51.100.17:18086/processed"
    assert st["discover"]["tailnetIp"] == "100.64.0.35"
    assert st["discoverFails"] == 0

    q = MP.MjpegPuller(["http://x/y"], name="u", discover={"tailnetIp": "100.64.0.7"},
                       resolver=runner_of(TIMED_OUT))
    q._candidates()
    assert q.stats()["discoveredUrl"] is None
    assert q.stats()["discoverFails"] == 1


def test_discover_가_없는_소스는_예전_그대로다():
    """음성 대조군 — 범위를 잘못 넓히지 않는다."""
    p = MP.MjpegPuller(["http://a/1", "http://b/2"], name="t")
    assert p._candidates() == ["http://a/1", "http://b/2"]
    assert p.stats()["discover"] is None


# ---- 설정 파서 ------------------------------------------------------------------

def test_설정의_discover_가_스펙까지_간다(tmp_path):
    import io as _io
    import json as _json
    cfg = tmp_path / "v.json"
    cfg.write_text(_json.dumps({"pullSources": [
        {"id": "phone2", "discover": {"tailnetIp": "100.64.0.35",
                                      "port": 18086, "path": "/processed"}},
        {"id": "old", "urlCandidates": ["http://1.2.3.4:18086/processed"]},
    ]}, ensure_ascii=False), encoding="utf-8")
    specs = {s["id"]: s for s in MP.load_pull_sources(str(cfg))}
    assert specs["phone2"]["discover"]["tailnetIp"] == "100.64.0.35"
    assert specs["phone2"]["urls"] == []
    assert specs["old"]["discover"] is None


def test_tailnetIp_없는_discover_는_거절한다(tmp_path):
    import json as _json
    cfg = tmp_path / "v.json"
    cfg.write_text(_json.dumps({"pullSources": [
        {"id": "bad", "discover": {"port": 18086}},
    ]}, ensure_ascii=False), encoding="utf-8")
    with pytest.raises(ValueError) as e:
        MP.load_pull_sources(str(cfg))
    assert "tailnetIp" in str(e.value)


def test_실제_설정_파일이_읽히고_이동_기기가_discover_를_가진다():
    """🔴 실물 `configs/video_sources.json` 을 읽는다 — 시험용 픽스처가 아니다."""
    import os as _os
    here = _os.path.dirname(_os.path.abspath(__file__))
    repo = _os.path.dirname(_os.path.dirname(here))
    specs = {s["id"]: s for s in MP.load_pull_sources(
        _os.path.join(repo, "relay_station", "configs", "video_sources.json"))}
    for sid in ("phone", "phone2", "tablet-relay"):
        assert sid in specs, "%s 가 설정에서 사라졌다" % sid
        d = specs[sid].get("discover")
        assert d and d.get("tailnetIp"), (
            "%s 에 discover.tailnetIp 가 없다 — 사이트가 바뀌면 또 못 찾는다" % sid)
