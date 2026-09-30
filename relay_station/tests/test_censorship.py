# -*- coding: utf-8 -*-
"""MCV-2C — "이 프레임이 검열을 거쳤다" 고 **말해도 되는가**.

가공(검열)이 proot 리눅스에서 **앱 안으로** 옮겨간다(사용자 의도). 그러면
`X-Process-Rules` 를 찍는 구현이 둘이 되고, "헤더 있음 = 검열됨" 이라는 추론이
**조용히** 죽는다 — 프레임도 오고 헤더도 붙어 있는데 뜻만 사라진다.

여기서 고정하는 성질:

    allowlist   모르는 구현은 **검열 안 된 것**으로 친다 (fail-closed)
    반증        픽셀 검사는 **반증만** 한다 — 통과가 증명이 아니다
    말만 한다   시청을 끊지 않는다 (사용자 결정). 이 모듈은 /video_feed 에서 안 불린다
"""
import io
import json
import os

import numpy
import pytest

import censorship as CZ

_HERE = os.path.dirname(os.path.abspath(__file__))
_RELAY = os.path.dirname(_HERE)

BOX = {"x": 0.0, "y": 0.0, "w": 0.5, "h": 0.5}


# ---- 정책 --------------------------------------------------------------------

def test_정책_파일이_없으면_아무도_인정하지_않는다(tmp_path):
    """⭐ 없다고 '다 믿는다' 로 떨어지면 이 장치의 의미가 없다."""
    p = CZ.load_policy(str(tmp_path / "없다.json"))
    assert p.trusted_rules == ()
    assert not p.trusts("v2")


def test_망가진_정책_파일도_빈_목록(tmp_path):
    f = tmp_path / "c.json"
    f.write_text("{이건 JSON 이 아니다", encoding="utf-8")
    assert CZ.load_policy(str(f)).trusted_rules == ()


def test_목록을_읽는다(tmp_path):
    f = tmp_path / "c.json"
    f.write_text(json.dumps({"trustedRules": ["proot-v2", "v1"]}), encoding="utf-8")
    p = CZ.load_policy(str(f))
    assert p.trusts("proot-v2") and p.trusts("v1")
    assert not p.trusts("app-0.2.0")


def test_레포의_정책_파일이_읽힌다():
    """실제로 배포되는 파일이 형식을 지키는지 본다."""
    p = os.path.join(os.path.dirname(_RELAY), "relay_station", "configs", "censorship.json")
    assert os.path.exists(p), "configs/censorship.json 이 없다"
    pol = CZ.load_policy(p)
    assert pol.trusted_rules, "목록이 비어 있으면 아무 소스도 인정되지 않는다"


# ---- 판정 --------------------------------------------------------------------

def _policy(*rules):
    return CZ.Policy(rules)


def test_모르는_구현은_검열_안_된_것으로_친다():
    """⭐ 이게 전환 순서를 지키는 자리다. 앱이 :18082 를 먼저 잡아도 조용히 인정 안 한다."""
    st, sev, _ = CZ.classify("app-0.2.0", BOX, None, _policy("proot-v2"))
    assert st == CZ.STATE_UNKNOWN
    assert sev == "degraded"


def test_rules_가_아예_없어도_모르는_구현이다():
    st, _sev, _ = CZ.classify(None, BOX, None, _policy("proot-v2"))
    assert st == CZ.STATE_UNKNOWN


def test_인정된_구현인데_상자가_없으면_검열_꺼짐():
    st, sev, _ = CZ.classify("proot-v2", None, None, _policy("proot-v2"))
    assert st == CZ.STATE_OFF
    assert sev == "info"


def test_상자는_선언됐는데_아직_안_봤으면_미검산():
    st, sev, _ = CZ.classify("proot-v2", BOX, None, _policy("proot-v2"))
    assert st == CZ.STATE_UNVERIFIED
    assert sev == "info"


def test_안쪽이_또렷하면_거짓_선언이다():
    """⭐ 31 커널 가우시안이 고주파를 남길 수 없다. 이건 **증명**이다."""
    ev = {"insideSharpness": 900.0, "outsideSharpness": 1000.0, "ratio": 0.9}
    st, sev, _ = CZ.classify("proot-v2", BOX, ev, _policy("proot-v2"))
    assert st == CZ.STATE_FALSE
    assert sev == "safety", "만든 쪽이 사실과 다른 말을 하는 것은 가장 센 신호다"


def test_안쪽이_밋밋하면_반증되지_않았을_뿐이다():
    """⭐⭐ 이름이 VERIFIED 가 아니라 CONSISTENT 인 이유.

    민무늬 벽도 고주파가 없다. 밋밋함은 흐림의 **증명이 아니다.**
    """
    ev = {"insideSharpness": 5.0, "outsideSharpness": 1000.0, "ratio": 0.005}
    st, _sev, _ = CZ.classify("proot-v2", BOX, ev, _policy("proot-v2"))
    assert st == CZ.STATE_CONSISTENT
    assert st != "VERIFIED", "증명하지 못하는 것을 증명했다고 부르지 않는다"


def test_증명을_뜻하는_상태가_아예_없다():
    """소스에 VERIFIED 라는 상태가 생기면 그 순간부터 거짓말이 된다."""
    names = [v for k, v in vars(CZ).items()
             if k.startswith("STATE_") and isinstance(v, str)]
    assert "VERIFIED" not in names
    assert CZ.STATE_CONSISTENT in names


def test_ok_는_저하_항목을_안_만든다():
    assert CZ.degradation_for("phone", CZ.STATE_CONSISTENT, "ok", {}) is None


def test_저하_항목이_판단_근거를_함께_낸다():
    ev = {"ratio": 0.9}
    st, sev, detail = CZ.classify("app-x", BOX, ev, _policy("proot-v2"))
    item = CZ.degradation_for("phone", st, sev, detail)
    assert item["code"] == "CENSORSHIP_" + CZ.STATE_UNKNOWN
    assert item["sourceId"] == "phone"
    assert item["rules"] == "app-x"
    assert item["declaredBox"] == BOX
    assert item["trustedProducer"] is False


# ---- 픽셀 증거 ----------------------------------------------------------------

def _frame(blur_box=None):
    """고주파가 가득한 프레임. 필요하면 상자 영역만 흐린다."""
    import cv2
    rng = numpy.random.default_rng(7)
    img = rng.integers(0, 255, (240, 320, 3), dtype=numpy.uint8)
    if blur_box:
        h, w = img.shape[:2]
        x0, y0 = int(blur_box["x"] * w), int(blur_box["y"] * h)
        x1 = int((blur_box["x"] + blur_box["w"]) * w)
        y1 = int((blur_box["y"] + blur_box["h"]) * h)
        img[y0:y1, x0:x1] = cv2.GaussianBlur(img[y0:y1, x0:x1], (31, 31), 0)
    return img


def test_흐린_영역은_또렷함이_확_낮다():
    ev = CZ.blur_evidence(_frame(blur_box=BOX), BOX)
    assert ev is not None
    assert ev["ratio"] < CZ.SHARPNESS_REFUTES_AT, ev


def test_안_흐린데_선언만_했으면_반증된다():
    """⭐ 이게 라벨을 안 믿는 유일한 검사다."""
    ev = CZ.blur_evidence(_frame(blur_box=None), BOX)
    assert ev is not None
    assert ev["ratio"] >= CZ.SHARPNESS_REFUTES_AT, ev
    st, _sev, _ = CZ.classify("proot-v2", BOX, ev, _policy("proot-v2"))
    assert st == CZ.STATE_FALSE


def test_상자가_너무_작으면_재지_않는다():
    """표본이 모자라면 숫자를 만들어내지 않는다."""
    tiny = {"x": 0.0, "y": 0.0, "w": 0.01, "h": 0.01}
    assert CZ.blur_evidence(_frame(), tiny) is None


def test_상자가_화면_전체면_바깥이_없어서_못_잰다():
    whole = {"x": 0.0, "y": 0.0, "w": 1.0, "h": 1.0}
    assert CZ.blur_evidence(_frame(), whole) is None


def test_프레임이나_상자가_없으면_None():
    assert CZ.blur_evidence(None, BOX) is None
    assert CZ.blur_evidence(_frame(), None) is None


# ---- 🔴 시청을 끊지 않는다 (사용자 결정 2026-09-11) ------------------------------------

def _server_src():
    with io.open(os.path.join(_RELAY, "gateway_web", "gateway_web_server.py"),
                 encoding="utf-8") as fh:
        return fh.read()


def test_시청_경로에서_검열_판정을_부르지_않는다():
    """🔴 사용자 결정: 검열을 못 믿어도 **시청은 계속한다.**

    교육장 화면이 비는 쪽이 더 나쁘고, 게이트웨이는 집행 지점이 아니다.
    이 모듈이 /video_feed 쪽으로 새어 들어가면 그 결정이 조용히 뒤집힌다.
    """
    src = _server_src()
    i = src.find("def video_feed")
    if i < 0:
        i = src.find("'/video_feed'")
    assert i >= 0, "시청 경로를 못 찾았다"
    # 시청 핸들러 앞뒤 넉넉히 잘라서 본다
    window = src[max(0, i - 2000): i + 6000]
    for bad in ("censorship.", "_censorship_state", "_merge_censorship"):
        assert bad not in window, "시청 경로에 검열 판정이 들어왔다: %s" % bad


def test_안전_판정에서는_부른다():
    """⭐ 판정을 **문자 거리**가 아니라 **핸들러 블록**으로 한다 (2026-09-12).

    예전 판은 `/api/safety` 뒤 600자 안에 있는지를 봤다. 그래서 그 핸들러에 줄이
    몇 개 늘자 **성질은 그대로인데 시험이 깨졌다** — 그리고 반대 방향이 더 나쁘다:
    바로 다음 핸들러의 호출이 창 안에 들어오면 **없는데도 통과**한다.
    블록으로 자르면 둘 다 안 생긴다.
    """
    src = _server_src()
    i = src.find("if parsed.path == '/api/safety':")
    assert i >= 0
    nxt = src.find("if parsed.path ==", i + 10)
    block = src[i:nxt if nxt > 0 else len(src)]
    assert "_merge_censorship" in block, "안전 핸들러가 검열 판정을 안 부른다"


def test_정책이_안전_응답에_함께_나온다():
    """무엇을 인정하고 있는지 화면이 볼 수 있어야 한다."""
    assert "censorshipPolicy" in _server_src()


# ---- 생산자 식별 — `@` 접미사 (2026-09-11 실측 결함) -------------------------
#
# 가공 단은 `<구현>@<규칙파일 시각>` 을 낸다(`proot-v2@08:18:51`). 시각은 규칙을
# 다시 읽을 때마다 바뀐다. 그런데 대조가 정확 일치여서 **목록이 영원히 안 걸렸다** —
# 모든 프레임이 UNKNOWN_PRODUCER 였고 그 아래 네 상태가 도달 불가능했다.
#
# ⭐ 위 시험 21개가 이걸 못 잡은 이유: 전부 `"proot-v2"` 를 접미사 없이 썼다.
#    아래는 **기기가 실제로 내보내는 형태**로만 쓴다.

LIVE = "proot-v2@08:18:51"          # 태블릿 :18083 실측 형태


def test_실제_형태가_구현_등재로_인정된다():
    """⭐⭐ 이 절의 존재 이유. 정확 일치 시절에는 영원히 거짓이었다."""
    assert CZ.Policy(("proot-v2",)).trusts(LIVE)


def test_접미사_없는_값도_그대로_인정된다():
    """하위 호환 — 접미사를 안 붙이는 구현이 있어도 깨지지 않는다."""
    assert CZ.Policy(("proot-v2",)).trusts("proot-v2")


def test_목록에_없는_구현은_접미사가_있어도_거부된다():
    assert not CZ.Policy(("proot-v2",)).trusts("evil-v9@08:18:51")


def test_앞이_비면_거부된다():
    """`@proot-v2` 의 구현 식별자는 빈 문자열이다."""
    assert not CZ.Policy(("proot-v2",)).trusts("@proot-v2")


def test_빈_등재값이_있어도_빈_식별자를_인정하지_않는다():
    """⭐ `Policy` 는 공개 생성자다. 목록에 ""가 섞이면 `@x` 가 전부 통과할 수 있다."""
    assert not CZ.Policy(("", "proot-v2")).trusts("@무엇이든")


def test_뒤에_등재값이_있어도_거부된다():
    """⭐⭐ `@` 를 **첫 번째**에서만 자른다. 아니면 이것이 곧 위조다."""
    assert not CZ.Policy(("proot-v2",)).trusts("evil@proot-v2")


def test_빌드까지_등재하면_그_값만_인정한다():
    """등재를 어떻게 적었는지가 대조 단위를 정한다."""
    p = CZ.Policy(("app-0.2.1@9",))
    assert p.trusts("app-0.2.1@9")
    assert not p.trusts("app-0.2.1@10")
    assert not p.trusts("app-0.2.1")


def test_빈_목록이면_실제_형태도_거부된다():
    """fail-closed 는 그대로다 — 느슨해진 것이 아니다."""
    assert not CZ.Policy(()).trusts(LIVE)


def test_실제_형태에서_판정이_아래_단계까지_간다():
    """⭐⭐ 죽어 있던 경로가 살아나는지. 이전에는 전부 UNKNOWN_PRODUCER 였다."""
    pol = CZ.Policy(("proot-v2",))
    st, _sev, detail = CZ.classify(LIVE, None, None, pol)
    assert st == CZ.STATE_OFF, st                    # 상자 없음까지 도달
    assert detail["trustedProducer"] is True
    st2, _s2, _d2 = CZ.classify(LIVE, BOX, None, pol)
    assert st2 == CZ.STATE_UNVERIFIED, st2           # 상자 선언, 아직 안 봄
    st3, sev3, _d3 = CZ.classify(LIVE, BOX, {"ratio": 0.99}, pol)
    assert st3 == CZ.STATE_FALSE and sev3 == "safety"  # 선언했는데 또렷하다


# ---- R-4 — allowlist 에서 app-* 를 어떻게 다루나 -----------------------------

def test_app_는_아직_등재되지_않는다():
    """⭐⭐ 앱 검열은 **미구현**이다(흡수 1단계 미착수). 그래서 UNKNOWN_PRODUCER 가 맞다.

    이 가드를 바꾸려면 `configs/censorship.json` 의 `_app_규칙` 을 같이 읽어야 한다 —
    등재 조건은 "사람이 실제로 흐리는 것을 확인한 뒤" 다.
    """
    import io as _io, json as _json, os as _os
    p = _os.path.join(_RELAY, "..", "relay_station", "configs", "censorship.json")
    raw = _json.load(_io.open(_os.path.normpath(p), encoding="utf-8"))
    assert "_app_규칙" in raw, "R-4 규칙이 설정에서 사라졌다"
    offenders = [r for r in raw["trustedRules"] if r.startswith("app-")]
    assert not offenders, (
        "앱 검열이 확인되기 전에 등재됐다: %s — 거짓 DECLARED_CONSISTENT 가 난다" % offenders)


def test_인코더_신원은_검열_선언이_아니다():
    """⭐ `app-0.2.1@9` 는 X-Encode-Rules 의 값이다. 같은 모양이라 함정이다.

    목록에 없으면 검열로 인정되지 않는다 — 그 성질을 여기서 고정한다.
    """
    pol = CZ.load_policy(_os_path_censorship())
    assert not pol.trusts("app-0.2.1@9")


def _os_path_censorship():
    import os as _os
    return _os.path.normpath(_os.path.join(_RELAY, "..", "relay_station", "configs", "censorship.json"))
