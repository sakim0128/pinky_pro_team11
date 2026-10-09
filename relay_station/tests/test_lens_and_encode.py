# -*- coding: utf-8 -*-
"""MCV-2L — 앱 v0.2.1 이 새로 내는 곁표와, **렌즈 교체로 정착을 내리는 것**.

앱 실측(2026-09-11, 태블릿 versionCode 9):

    X-Camera-Lens: WIDE
    X-Encode-Ms: 8.2
    X-Encode-Rules: app-0.2.1@9      ← **인코더 신원이지 검열 선언이 아니다**
    X-Process-* 없음                  ← 앱은 검열하지 않는다

⭐⭐ `X-Camera-Lens` 가 내가 막아 달라고 한 구멍을 메운다:
   광각 1280x720 과 일반 1280x720 은 **프레임 크기가 같아서** 크기 검사가 못 잡는다.
"""
import io
import json
import os

import pytest

import calibration as C
import censorship as CZ
from mjpeg_puller import (MjpegPuller, parse_part_headers,
                          H_CAMERA_LENS, H_ENCODE_MS, H_ENCODE_RULES,
                          H_PROCESS_MS, H_PROCESS_RULES)

_HERE = os.path.dirname(os.path.abspath(__file__))
_RELAY = os.path.dirname(_HERE)

FRAME = (1280, 720)
QUAD = [(180.0, 640.0), (1120.0, 660.0), (900.0, 210.0), (330.0, 200.0)]
ARENA = {"version": "t1", "widthCm": 270.0, "heightCm": 125.0, "landmarks": {}}
EOL = bytes([13, 10])


@pytest.fixture
def store(tmp_path):
    p = tmp_path / "arena.json"
    p.write_text(json.dumps(ARENA), encoding="utf-8")
    return C.CalibrationStore(C.load_arena(str(p)),
                              state_dir=str(tmp_path / "cal"), clock=lambda: 1000.0)


def _puller():
    return MjpegPuller("http://127.0.0.1:9/video", name="unit")


def _part(**headers):
    out = b"--raasframe" + EOL + b"Content-Type: image/jpeg" + EOL
    for k, v in headers.items():
        out += k.encode("ascii") + b": " + str(v).encode("utf-8") + EOL
    return out + EOL


# ---- 실경로로 들어오는가 ---------------------------------------------------------

def test_앱이_내는_세_헤더가_실제_경로로_들어온다():
    """⭐ `_note_sidecar` 를 직접 부르지 않는다 — 허용목록을 건너뛰면 죽은 기능을 통과시킨다.

    X-Process-Blur 가 정확히 그렇게 물렸다(2026-09-10).
    """
    meta = parse_part_headers(_part(**{
        "X-Camera-Lens": "WIDE",
        "X-Encode-Ms": "8.2",
        "X-Encode-Rules": "app-0.2.1@9",
        "X-Capture-Clock": "1789107192000",
    }))
    for h in (H_CAMERA_LENS, H_ENCODE_MS, H_ENCODE_RULES):
        assert h in meta, "파트 헤더 파서가 %s 를 버렸다" % h
    p = _puller()
    p._note_sidecar(meta, 1000.0)
    sc = p.sidecar()
    assert sc["cameraLens"] == "WIDE"
    assert sc["encodeMs"] == pytest.approx(8.2)
    assert sc["encodeRules"] == "app-0.2.1@9"


def test_인코딩은_가공과_다른_칸에_들어간다():
    """⭐ 같은 칸에 넣으면 `offsetMedian - processMedian` 계산이 조용히 틀린다.

    앱의 인코딩 8.2ms 와 가공 단의 검열 47ms 는 **다른 양**이다.
    """
    p = _puller()
    p._note_sidecar(parse_part_headers(_part(**{
        "X-Encode-Ms": "8.2", "X-Encode-Rules": "app-0.2.1@9"})), 1000.0)
    sc = p.sidecar()
    assert sc["encodeMs"] == pytest.approx(8.2)
    assert sc["processMs"] is None, "인코딩 시간이 가공 시간 칸에 들어갔다"
    assert sc["processRules"] is None
    assert sc["processSamples"] == 0, "인코딩이 가공 통계를 오염시켰다"


def test_헤더가_사라지면_값도_사라진다():
    """앱이 렌즈 헤더를 안 내기 시작하면 마지막 값을 붙들면 안 된다."""
    p = _puller()
    p._note_sidecar(parse_part_headers(_part(**{"X-Camera-Lens": "ULTRA_WIDE"})), 1000.0)
    assert p.sidecar()["cameraLens"] == "ULTRA_WIDE"
    p._note_sidecar(parse_part_headers(_part(**{"X-Capture-Clock": "1"})), 1000.1)
    assert p.sidecar()["cameraLens"] is None


# ---- 🔴 인코더 신원을 검열 신원으로 읽지 않는다 ----------------------------------------

def test_검열_판정은_인코더_규칙을_절대_안_본다():
    """🔴 `app-0.2.1@9` 는 **인코더 신원**이다. 검열 선언이 아니다.

    이걸 trustedRules 에 올리면 **거짓 DECLARED_CONSISTENT** 가 난다 —
    앱은 검열을 하지 않는데 소비자가 검열됐다고 말하게 된다.
    앱 세션도 같은 판단을 했다(2026-09-11).
    """
    pol = CZ.Policy(("proot-v2", "app-0.2.1@9"))      # 실수로 올렸다고 치자
    # 검열 판정에 넘기는 것은 **process** rules 다. 인코더 값은 애초에 안 들어온다.
    st, sev, _ = CZ.classify(None, None, None, pol)
    assert st == CZ.STATE_UNKNOWN, "process rules 가 없으면 모르는 구현이어야 한다"
    assert sev == "degraded"


def test_서버가_검열_판정에_process_rules_만_넘긴다():
    src = io.open(os.path.join(_RELAY, "gateway_web", "gateway_web_server.py"),
                  encoding="utf-8").read()
    i = src.index("def _censorship_state")
    body = src[i:i + 1400]
    assert "processRules" in body
    assert "encodeRules" not in body, "검열 판정이 인코더 규칙을 보고 있다"


def test_레포_정책에_앱_인코더_신원이_없다():
    """운영 정책 파일에 인코더 값이 섞이면 그 순간 거짓말이 된다."""
    p = os.path.join(os.path.dirname(_RELAY), "relay_station", "configs", "censorship.json")
    pol = CZ.load_policy(p)
    for r in pol.trusted_rules:
        assert not r.startswith("app-"), (
            "앱 인코더 신원 %r 이 검열 allowlist 에 있다 — 앱은 검열하지 않는다" % r)


def test_등재를_어떻게_적느냐가_대조_단위를_정한다():
    """⭐ 정정: 처음엔 "정확한 목록이라 구분자는 무관" 이라고 적었는데 **틀렸다.**

    `@` 가 의미를 갖는다 — 앞은 구현, 뒤는 그 구현의 재적재/빌드다.
    가공 단이 `v1@04:17:32` 처럼 시각을 붙여 내기 때문에, 정확 일치만 하면
    **실기기에서 영원히 안 걸린다**(2026-09-11 다른 세션이 잡았다).

    그래서 등재를 **어떻게 적는지**가 단위를 고른다 — 규칙이 아니라 표현으로.
    """
    # 구현 단위로 적으면 그 구현의 모든 재적재본을 인정한다
    impl = CZ.Policy(("app-0.2.1",))
    assert impl.trusts("app-0.2.1@9")
    assert impl.trusts("app-0.2.1@10")
    assert not impl.trusts("app-0.2.2@9")

    # 빌드 단위로 적으면 그 값만 인정한다
    build = CZ.Policy(("app-0.2.1@9",))
    assert build.trusts("app-0.2.1@9")
    assert not build.trusts("app-0.2.1@10"), "빌드까지 좁혔으면 다른 빌드는 아니다"
    assert not build.trusts("app-0.2.1"), "접미사 없는 값은 그 빌드가 아니다"

    # 구분자가 다르면 구현 식별자 자체가 달라진다
    assert not impl.trusts("app-0.2.1+9"), "`+` 는 구분자가 아니라 이름의 일부다"


# ---- ⭐⭐ 렌즈가 바뀌면 정착이 내려온다 -------------------------------------------------

def test_크기가_같아도_렌즈가_다르면_무효다(store):
    """⭐⭐ 이 시험이 이 유닛의 이유다.

    광각 1280x720 과 일반 1280x720 은 **프레임 크기가 같다.** 세션·크기 검사만으로는
    못 잡는데, 화각이 다르면 같은 픽셀이 다른 곳을 가리킨다.
    """
    store.set_points("phone", QUAD, FRAME, publisher_session="s1", lens="WIDE")
    store.settle("phone")
    same = store.state("phone", frame_size=FRAME, publisher_session="s1", lens="WIDE")
    assert same["state"] == C.STATE_SETTLED

    changed = store.state("phone", frame_size=FRAME, publisher_session="s1",
                          lens="ULTRA_WIDE")
    assert changed["state"] == C.STATE_DRIFT
    assert changed["reason"] == C.REASON_LENS_CHANGED
    assert store.homography_for("phone", FRAME, "s1", "ULTRA_WIDE") is None


def test_렌즈가_크기보다_먼저_보고된다(store):
    """둘 다 바뀌었으면 "렌즈가 바뀌었다" 가 사람에게 더 정확한 말이다."""
    store.set_points("phone", QUAD, FRAME, publisher_session="s1", lens="WIDE")
    store.settle("phone")
    st = store.state("phone", frame_size=(1920, 1080), publisher_session="s1",
                     lens="ULTRA_WIDE")
    assert st["reason"] == C.REASON_LENS_CHANGED


def test_렌즈를_모르면_그것으로_무효화하지_않는다(store):
    """앱이 아직 그 헤더를 안 내는 기기도 있다. 모르는 것을 근거로 내리지 않는다."""
    store.set_points("phone", QUAD, FRAME, publisher_session="s1", lens=None)
    store.settle("phone")
    st = store.state("phone", frame_size=FRAME, publisher_session="s1", lens="WIDE")
    assert st["state"] == C.STATE_SETTLED, "잴 때 몰랐으면 비교할 기준이 없다"


def test_영수증이_렌즈를_적는다(store):
    store.set_points("phone", QUAD, FRAME, publisher_session="s1", lens="ULTRA_WIDE")
    r = store.settle("phone")
    assert r["lens"] == "ULTRA_WIDE", "어느 렌즈로 잰 값인지 남아야 한다"


def test_서버가_렌즈를_곁표에서_읽어_넘긴다():
    src = io.open(os.path.join(_RELAY, "gateway_web", "gateway_web_server.py"),
                  encoding="utf-8").read()
    assert "def _calib_lens" in src
    assert "cameraLens" in src
    assert "lens=lens" in src, "set_points 에 렌즈를 안 넘기고 있다"
    assert "liveLens" in src


def test_발행기도_렌즈_헤더를_낸다():
    """실기기 없이 이 경로를 시험하려면 발행기가 같은 헤더를 내야 한다."""
    src = io.open(os.path.join(_RELAY, "docker", "host_camera_publisher.py"),
                  encoding="utf-8").read()
    assert "X-Camera-Lens" in src
    assert "--lens" in src
