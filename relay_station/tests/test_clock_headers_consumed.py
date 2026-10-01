# -*- coding: utf-8 -*-
"""앱이 보내기 시작한 시계 셋을 **꺼내 쓴다** — 화이트리스트에 넣는 것만으로는 부족하다.

🔴 왜 생겼나 (2026-09-19):

    앱(vc18/vc19)이 `X-Capture-Basis` · `X-Publish-Clock` · `X-Clock-Suspect` 를
    내보내기 시작했다. 나는 그 전에 **화이트리스트에 미리 등재**해 뒀고(①정적설정),
    앱이 ②생산자를 끝냈다. 그런데 확인해 보니 `_note_sidecar` 가 그 셋을
    **한 번도 안 꺼내고 있었다**(`meta.get(H_*)` 호출 **0곳**).

    즉 헤더는 도착해서 파싱되고 **그 다음 단계에서 조용히 버려지고 있었다.**
    ①만 하고 ③을 안 한 것이다 — "정의를 찾았다 ≠ 그게 돈다" 의 내 판.

## ⭐ `publish − capture` 가 왜 중요한가

`offsetJitterMs` 는 **시계차 + 전송 + 큐** 의 합이다(모듈이 스스로 그렇게 적는다).
그래서 *"35.8 ms 가 작다"* 에서 끌어낼 수 있는 건 단측 결론뿐이었다.

`X-Publish-Clock − X-Capture-Clock` 은 **기기 한 시계 안의 뺄셈**이라 시계차가 상쇄된다.
그 흔들림은 **기기 안에서 일어난 몫**이다. 둘을 나란히 두면 사람이 성분을 가를 수 있다.

⚠️ **빼서 '전송 지터' 를 만들지 않는다.** 중앙값절대편차는 그렇게 쪼개지지 않는다.
   나란히 낼 뿐이고, 해석은 읽는 사람 몫이다.
"""
import mjpeg_puller as MP


def _part(**headers):
    lines = [b"--frame", b"Content-Type: image/jpeg"]
    for k, v in headers.items():
        lines.append(("%s: %s" % (k.replace("_", "-"), v)).encode("ascii"))
    return b"\r\n".join(lines) + b"\r\n\r\n"


def _puller(now=None):
    """⭐ **시계를 주입한다.** `_note_sidecar` 에 준 시각과 `sidecar()` 가 읽는 시각이
       다르면 창 필터가 표본을 통째로 버린다 — 2026-09-19 에 이 시험이 그렇게 틀렸다.
       코드가 아니라 **시험의 계측기**가 틀린 것이었다.
    """
    clk = now if callable(now) else (lambda: 1000.0)
    return MP.MjpegPuller("http://127.0.0.1:9/video", name="unit", clock=clk)


# ---- 꺼내 쓰는가 (그 결함 자체) ---------------------------------------------------

def test_기준을_꺼내_곁표에_싣는다():
    """🔴 2026-09-19 이전에는 이 시험이 빨갰다 — 화이트리스트에만 있었다."""
    p = _puller()
    p._note_sidecar(MP.parse_part_headers(
        _part(**{"X-Capture-Basis": "sensor-realtime"})), 1000.0)
    assert p.sidecar()["captureBasis"] == "sensor-realtime"


def test_앱의_시계_의심_선언을_꺼낸다():
    p = _puller()
    p._note_sidecar(MP.parse_part_headers(
        _part(**{"X-Clock-Suspect": "true"})), 1000.0)
    assert p.sidecar()["clockSuspectApp"] is True

    q = _puller()
    q._note_sidecar(MP.parse_part_headers(
        _part(**{"X-Clock-Suspect": "false"})), 1000.0)
    assert q.sidecar()["clockSuspectApp"] is False


def test_안_오면_None_이지_False_가_아니다():
    """⭐ '앱이 아니라고 했다' 와 '앱이 말 안 했다' 는 다른 상태다.

    옛 빌드(vc14)는 이 헤더를 안 보낸다. 그걸 `false` 로 읽으면
    **옛 빌드를 '시계가 멀쩡하다고 선언한 빌드'** 로 세게 된다.
    """
    p = _puller()
    p._note_sidecar(MP.parse_part_headers(_part(**{"X-Capture-Clock": "1"})), 1000.0)
    sc = p.sidecar()
    assert sc["captureBasis"] is None
    assert sc["clockSuspectApp"] is None


def test_알_수_없는_기준값도_그대로_싣는다():
    """앱의 열거가 늘어도 중계가 값을 걸러내지 않는다 — 판정은 소비자 몫이다."""
    p = _puller()
    p._note_sidecar(MP.parse_part_headers(
        _part(**{"X-Capture-Basis": "converted-unknown-origin"})), 1000.0)
    assert p.sidecar()["captureBasis"] == "converted-unknown-origin"


# ---- 기기 내부 지연 ---------------------------------------------------------------

def test_publish_빼기_capture_를_잰다():
    """⭐ 기기 한 시계 안의 뺄셈이라 시계가 얼마나 틀렸든 결과가 같아야 한다."""
    p = _puller()
    for i, lat in enumerate((100, 110, 105, 108, 102)):
        cap = 1_000_000 + i * 120
        p._note_sidecar(MP.parse_part_headers(_part(**{
            "X-Capture-Clock": str(cap),
            "X-Publish-Clock": str(cap + lat),
        })), 1000.0 + i)
    sc = p.sidecar()
    assert sc["deviceLatencySamples"] == 5
    assert sc["deviceLatencyMedianMs"] == 105.0
    assert sc["deviceLatencyJitterMs"] == 3.0          # |100-105|,|110-105|,0,3,3 의 중앙값


def test_시계가_통째로_틀려도_기기_내부_지연은_같다():
    """🔴🔴 이게 이 값의 존재 이유다.

    같은 지연을 **3.46일 어긋난 시계**로 찍어도 `publish − capture` 는 안 변한다.
    `offsetMedianMs` 는 그 3.46일을 통째로 뒤집어쓴다.
    """
    SKEW = 299_275_817                     # 2026-09-19 노트3 Neo 실측 오프셋(ms)
    a, b = _puller(), _puller()
    for i, lat in enumerate((100, 110, 105)):
        for pul, base in ((a, 1_000_000), (b, 1_000_000 + SKEW)):
            cap = base + i * 120
            pul._note_sidecar(MP.parse_part_headers(_part(**{
                "X-Capture-Clock": str(cap),
                "X-Publish-Clock": str(cap + lat),
            })), 1000.0 + i)
    assert a.sidecar()["deviceLatencyMedianMs"] == b.sidecar()["deviceLatencyMedianMs"]
    assert a.sidecar()["deviceLatencyJitterMs"] == b.sidecar()["deviceLatencyJitterMs"]


def test_한쪽만_오면_지어내지_않는다():
    """`X-Publish-Clock` 만 오거나 `X-Capture-Clock` 만 오면 뺄 수 없다."""
    p = _puller()
    p._note_sidecar(MP.parse_part_headers(
        _part(**{"X-Publish-Clock": "1000100"})), 1000.0)
    q = _puller()
    q._note_sidecar(MP.parse_part_headers(
        _part(**{"X-Capture-Clock": "1000000"})), 1000.0)
    for pul in (p, q):
        sc = pul.sidecar()
        assert sc["deviceLatencySamples"] == 0
        assert sc["deviceLatencyMedianMs"] is None
        assert sc["deviceLatencyJitterMs"] is None


def test_창_밖_표본은_버린다():
    """60초 창. 안 버리면 기기가 느려진 걸 영영 못 본다."""
    현재 = [1000.0]
    p = _puller(lambda: 현재[0])
    p._note_sidecar(MP.parse_part_headers(_part(**{
        "X-Capture-Clock": "1000000", "X-Publish-Clock": "1000100"})), 1000.0)
    현재[0] = 1000.0 + MP.CLOCK_WINDOW_S + 5
    p._note_sidecar(MP.parse_part_headers(_part(**{
        "X-Capture-Clock": "1000120", "X-Publish-Clock": "1000320"})), 현재[0])
    sc = p.sidecar()
    assert sc["deviceLatencySamples"] == 1
    assert sc["deviceLatencyMedianMs"] == 200.0


# ---- 대조군 --------------------------------------------------------------------

def test_옛_빌드는_기기_내부_지연을_못_낸다():
    """음성 대조군 — vc14 는 `X-Publish-Clock` 을 안 보낸다. 그러면 None 이어야 한다.

    ⭐ 여기서 0 이나 추정값이 나오면, 옛 빌드가 붙어 있는데도 화면이
       **측정했다고 말하게 된다.**
    """
    p = _puller()
    for i in range(5):
        p._note_sidecar(MP.parse_part_headers(_part(**{
            "X-Capture-Clock": str(1_000_000 + i * 120),
            "X-Encode-Ms": "79.1",
            "X-Encode-Rules": "app-0.2.6@14",
        })), 1000.0 + i)
    sc = p.sidecar()
    assert sc["deviceLatencySamples"] == 0
    assert sc["deviceLatencyMedianMs"] is None
    assert sc["captureBasis"] is None
    assert sc["clockSuspectApp"] is None
    # 그래도 옛 빌드의 값들은 그대로 읽힌다 — 범위를 잘못 넓히지 않았다
    assert sc["encodeRules"] == "app-0.2.6@14"


def test_앱이_보내는_열세_헤더를_전부_꺼내_쓴다():
    """🔴🔴 **박아 둔 기대값 대조.**

    2026-09-19 에 앱 vc18/vc19 소스에서 전수한 파트 헤더 13종이다. 화이트리스트에
    있는 것과 **실제로 꺼내 쓰는 것**은 다른 집합이라, 여기서 후자를 고정한다.
    """
    p = _puller()
    p._note_sidecar(MP.parse_part_headers(_part(**{
        "X-Capture-Clock": "1000000",
        "X-Publisher-Session": "session-abc",
        "X-Frame-Seq": "42",
        "X-Process-Ms": "0.0",
        "X-Process-Rules": "app-v1@18",
        "X-Encode-Ms": "80.5",
        "X-Encode-Rules": "app-0.2.9@18",
        "X-Camera-Lens": "WIDE",
        "X-Camera-Rotation": "90",
        "X-Camera-Zoom": "1.00",
        "X-Capture-Basis": "sensor-realtime",
        "X-Publish-Clock": "1000100",
        "X-Clock-Suspect": "false",
    })), 1000.0)
    sc = p.sidecar()
    기대 = {
        "captureClockMs": 1000000, "publisherSession": "session-abc", "frameSeq": 42,
        "processMs": 0.0, "processRules": "app-v1@18",
        "encodeMs": 80.5, "encodeRules": "app-0.2.9@18",
        "cameraLens": "WIDE", "cameraRotation": 90.0, "cameraZoom": 1.0,
        "captureBasis": "sensor-realtime", "clockSuspectApp": False,
        "deviceLatencyMedianMs": 100.0,
    }
    빠진 = {k: (sc.get(k), v) for k, v in 기대.items() if sc.get(k) != v}
    assert not 빠진, "앱이 보내는데 곁표에 안 나오거나 값이 다르다: %s" % 빠진
    assert sc["unknownHeaders"] == [], "모르는 헤더가 생겼다: %s" % sc["unknownHeaders"]
