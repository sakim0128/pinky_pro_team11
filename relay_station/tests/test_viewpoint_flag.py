# -*- coding: utf-8 -*-
"""R-3 — **찍히는 것과 아레나를 보는 것은 다른 사실이다.**

중계 노트북에는 카메라가 둘이다. USB 웹캠은 사용자가 아레나 쪽으로 두고,
내장 LG 캠은 노트북 화면 방향이라 **사람 쪽을 찍는다**. 그런데
`real_viewpoint_ids()` 는 transport(local/pull) · trust · connected 만 봤다.
그래서 내장 캠을 열면 `/api/safety` 의 `realViewpoints.connected` 가 **올랐다** —
아레나를 보는 시점이 하나도 안 늘었는데 "관측 가능" 이 참이 됐다.

⭐ 이건 화면이 거짓말하는 종류의 결함이다. `MCVA-21`(2시점 5cm 일치)이 기대는 숫자라
   부풀면 **없는 대조를 있다고 주장**하게 된다(관제 세션 인계서 §6-G).

처방: 설정 항목에 `"viewpoint": false` 를 두고 판정이 그 값을 본다. 기본은 `true` —
대부분의 소스는 시점이고, 아닌 것만 끈다.

⚠️ **지우는 것이 아니라 세지 않는 것**이다. 내장 캠은 전환용 보조로 그대로 남는다.
"""
import io
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gateway_web"))

import pytest

from source_registry import (SourceRegistry, Source, LOCAL, PULL, ROS, PUSH,
                             TRUSTED, UNTRUSTED, real_viewpoint_ids, readiness)

LIVE = b"\xff\xd8\xff-live-\xff\xd9"
_CFG = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "relay_station", "configs", "video_sources.json")


def _provider(connected=True, stamp=1000.0):
    def p():
        return (LIVE if connected else None), (stamp if connected else 0.0), connected
    return p


# ---- 판정 --------------------------------------------------------------------

def test_전제_viewpoint_기본값은_True():
    """기본값이 False 로 뒤집히면 빨개진다 — 모든 기존 시점이 조용히 사라진다."""
    s = Source("cam", _provider(True), LOCAL, TRUSTED)
    assert s.viewpoint is True


def test_viewpoint_False_면_살아_있어도_안_센다():
    """🔴 판정이 플래그를 무시하면 빨개진다. 이것이 이 파일의 핵심이다.

    내장 캠은 connected=True 이고 transport 도 local 이고 trusted 다 —
    옛 판정 기준 셋을 **전부 통과**한다. 플래그만이 이걸 가른다.
    """
    reg = SourceRegistry()
    reg.register(Source("relay-cam", _provider(True), LOCAL, TRUSTED))
    reg.register(Source("relay-cam-internal", _provider(True), LOCAL, TRUSTED,
                        viewpoint=False))
    assert real_viewpoint_ids(reg) == ["relay-cam"]


def test_세지_않아도_레지스트리에는_남는다():
    """보조 캠을 목록에서 지워 버리면 빨개진다 — 전환용으로 골라야 한다."""
    reg = SourceRegistry()
    reg.register(Source("relay-cam-internal", _provider(True), LOCAL, TRUSTED,
                        viewpoint=False))
    assert "relay-cam-internal" in reg.ids()
    assert reg.get("relay-cam-internal") is not None


def test_readiness_의_실물_시점_수도_같이_줄어든다():
    """판정만 고치고 화면에 내보내는 수를 안 고치면 빨개진다."""
    reg = SourceRegistry()
    reg.register(Source("relay-cam", _provider(True), LOCAL, TRUSTED))
    reg.register(Source("relay-cam-internal", _provider(True), LOCAL, TRUSTED,
                        viewpoint=False))
    r = readiness(reg, observing=True)
    assert r["realViewpoints"]["connected"] == 1
    assert r["realViewpoints"]["ids"] == ["relay-cam"]


def test_viewpoint_True_는_예전과_똑같이_센다():
    """플래그를 넣느라 기존 동작이 바뀌면 빨개진다(회귀 방지)."""
    reg = SourceRegistry()
    reg.register(Source("relay-cam", _provider(True), LOCAL, TRUSTED, viewpoint=True))
    reg.register(Source("phone", _provider(True), PULL, TRUSTED))
    reg.register(Source("gazebo", _provider(True), ROS, TRUSTED))
    reg.register(Source("tablet", _provider(True), PUSH, UNTRUSTED))
    assert real_viewpoint_ids(reg) == ["relay-cam", "phone"]


# ---- 설정 로더 ----------------------------------------------------------------

def test_local_로더가_플래그를_읽는다():
    """로더가 키를 버리면 빨개진다 — 설정에 적어도 코드에 안 닿는다."""
    from local_camera import load_local_sources
    specs = load_local_sources({"localSources": [
        {"id": "a", "device": "/dev/null"},
        {"id": "b", "device": "/dev/null", "viewpoint": False},
    ]})
    got = {s["id"]: s["viewpoint"] for s in specs}
    assert got == {"a": True, "b": False}


def test_pull_로더가_플래그를_읽는다(tmp_path):
    from mjpeg_puller import load_pull_sources
    p = tmp_path / "v.json"
    p.write_text(json.dumps({"pullSources": [
        {"id": "a", "url": "http://h:1/v"},
        {"id": "b", "url": "http://h:2/v", "viewpoint": False},
    ]}), encoding="utf-8")
    specs = load_pull_sources(str(p))
    got = {s["id"]: s["viewpoint"] for s in specs}
    assert got == {"a": True, "b": False}


# ---- 배포된 설정 --------------------------------------------------------------

def test_배포_설정에서_내장캠만_시점이_아니다():
    """설정이 되돌아가면 빨개진다. 코드가 맞아도 설정이 틀리면 현장은 부푼 채다."""
    doc = json.load(io.open(_CFG, encoding="utf-8"))
    flags = {e["id"]: e.get("viewpoint", True) for e in doc["localSources"]}
    assert flags.get("relay-cam") is True, "USB 웹캠은 아레나 시점이다"
    assert flags.get("relay-cam-internal") is False, "내장 캠은 시점으로 세면 안 된다"


def test_소스_id_가_기기_하나씩을_가리킨다():
    """🔴 id 하나가 **다른 기기**를 가리키게 되면 빨개진다.

    보정 영수증 키는 sourceId + arenaVersion 뿐이라 **기기 정체가 키에 없다.**
    그래서 id 를 다른 기기가 물려받으면 예전 정착 영수증이 다른 렌즈·다른 해상도의
    기기에 그대로 적용된다 — 금지 조항 6 위반이다.

    ## 🔴 이 시험의 첫 판은 **틀린 전제** 위에 있었다 (2026-09-14 -> 09-17 정정)

    처음엔 `"tablet-relay" not in ids` 로 못 박았다. 근거는 "태블릿 y700 이 다른 폰으로
    **교체**됐다" 였고, 그래서 그 id 를 되살리는 것을 금지했다. 그런데 09-17 현장 결정은
    **Y700 재개**다 — 교체가 아니었다. `tablet-relay` 는 지금도 **같은 기기(Y700)** 를
    가리키므로 id 재사용이 아니다. 금지해야 할 것은 이름이 아니라 **기기가 바뀌는 것**이었다.

    ⭐ 그래서 판정을 이름 금지에서 **id↔기기 대응**으로 옮긴다. 이쪽이 원래 지키려던 것이고,
      기기가 또 바뀌면(그때는 진짜 교체) 라벨이 안 맞아 빨개진다.

    ⚠️ 라벨은 사람이 읽는 값이라 문구가 흔들릴 수 있다 — 그래서 **기종을 가리키는 토막**만 본다.
    """
    doc = json.load(io.open(_CFG, encoding="utf-8"))
    entries = {e["id"]: e for e in doc["pullSources"]}
    expected = {
        "phone": ("노트10", "note10"),          # Galaxy Note 10
        "tablet-relay": ("y700",),              # Lenovo Y700 (09-17 재개 — 교체 아님)
        "phone2": ("note 3", "note3"),          # Galaxy Note 3 Neo
    }
    assert set(entries) == set(expected), (
        "pull 소스 구성이 바뀌었다. 기대 %r 실제 %r" % (sorted(expected), sorted(entries)))
    for sid, needles in expected.items():
        label = (entries[sid].get("label") or "").lower()
        assert any(n.lower() in label for n in needles), (
            "id %r 의 라벨이 기대한 기기를 안 가리킨다 — 기기가 바뀌었으면 **새 id + 재보정**이다. "
            "라벨=%r" % (sid, entries[sid].get("label")))
