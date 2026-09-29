# -*- coding: utf-8 -*-
"""R-A — `vision_pose` 가 **어디까지 갔는지**를 정직하게 말한다.

🔴 고친 결함: `POST /api/vision/pose` 응답의 `hasReceiver` 가 `true` 인데 그 수신자가
   **브리지**였다. 게이트웨이는 도메인 8 참여자라 `get_subscription_count()` 가 세는 것이
   도메인 8 의 구독자이고, 로봇이 있는 도메인 10 은 **여기서 볼 수 없다.**

   2026-09-19 실측:
       D8  /pinky1/vision_pose   Pub 1 (게이트웨이) · Sub 1 (pinky_bridge_pinky1_8)
       D10 /vision_pose          Pub 1 (브리지)     · Sub 0        ← 여기서 끊긴다

   연산 노드 세션이 그 값을 종단 근거로 쓸 뻔했고, 관제 세션과 함께
   "이 필드를 어느 수락의 근거로도 쓰지 않는다" 로 합의했다(2026-09-20).

⭐ 그리고 이 계약은 **쓰는 쪽을 위한 것**이다 — 2026-09-19 기준 `robots/` 전체에
   `vision_pose` 소비자가 **0 건**이라, 그 코드를 쓸 사람이 브리지 설정을 뒤지지 않아도
   되게 응답이 이름·타입·QoS 를 직접 말한다.
"""
import ast
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gateway_web"))

import pytest
import yaml

import vision_path as vp

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_CFG_DIR = os.path.join(_REPO, "relay_station", "domain_bridge", "configs")
_GATEWAY = os.path.join(_REPO, "relay_station", "gateway_web", "gateway_web_server.py")


# ---- 구독자 분류 --------------------------------------------------------------

def test_브리지만_있으면_consumer_가_0_이다():
    """🔴 이게 이 파일의 핵심이다. 수만 세면 1 이고, 그 1 을 '로봇이 받는다' 로 읽는다."""
    r = vp.classify_receivers(["/pinky_bridge_pinky1_8"], topic="/pinky1/vision_pose")
    assert r["total"] == 1
    assert r["bridge"] == 1
    assert r["consumer"] == 0, "브리지를 소비자로 세면 종단 판정이 거짓이 된다"
    assert r["measured"] is True


def test_브리지가_아닌_구독자가_생기면_consumer_가_오른다():
    r = vp.classify_receivers(["/pinky_bridge_pinky1_8", "/pinky1_vision_consumer"])
    assert (r["total"], r["bridge"], r["consumer"]) == (2, 1, 1)


def test_아무도_없으면_전부_0():
    r = vp.classify_receivers([])
    assert (r["total"], r["bridge"], r["consumer"]) == (0, 0, 0)
    assert r["measured"] is True, "빈 목록은 **쟀는데 없는 것**이다"


def test_못_쟀으면_0_이_아니라_None_이다():
    """⚠️ 0 은 '없다' 는 **다른 주장**이다. 이 레포가 반복해 밟은 함정."""
    r = vp.classify_receivers(None, topic="/pinky1/vision_pose")
    assert r["measured"] is False
    assert r["total"] is None and r["bridge"] is None and r["consumer"] is None
    assert r["nodes"] is None


def test_네임스페이스가_붙어도_브리지로_센다():
    r = vp.classify_receivers(["/ns/pinky_bridge_pinky1_8"])
    assert r["bridge"] == 1


def test_이름이_비슷하기만_한_것은_브리지가_아니다():
    """접두어 판정이 느슨해지면 빨개진다 — 소비자를 브리지로 세면 종단이 영영 0 이 된다."""
    r = vp.classify_receivers(["/my_pinky_bridge_fake", "/bridge_pinky_pinky1"])
    assert r["bridge"] == 0 and r["consumer"] == 2


# ---- 하류 계약 ----------------------------------------------------------------

def test_계약이_잴_수_없는_것을_0_으로_적지_않는다():
    """🔴 `subscribers: 0` 으로 바꾸면 빨개진다 — 게이트웨이는 도메인 10 을 못 본다."""
    c = vp.downstream_contract("pinky1")
    assert c["subscribers"] is None and c["flowHz"] is None
    assert c["why"] == vp.NOT_OBSERVABLE


def test_계약이_쓰는_쪽이_베낄_것을_다_갖는다():
    """이름·타입·프레임·QoS 중 하나라도 빠지면 빨개진다 — 소비자가 설정을 뒤지게 된다."""
    c = vp.downstream_contract("pinky1")
    for k in ("domain", "topic", "type", "frameId", "qos", "howToMeasure"):
        assert c.get(k), "계약에 %s 가 없다" % k
    assert str(c["domain"]) in c["howToMeasure"] and c["topic"] in c["howToMeasure"]


def test_모르는_로봇은_계약을_지어내지_않는다():
    assert vp.downstream_contract("robot9") is None


# ---- 🔴 리터럴이 **브리지 설정과 같은가** (이 파일에서 제일 중요한 검사) ----------

def test_하류_계약이_브리지_설정과_일치한다():
    """🔴 상수와 yaml 이 어긋나면 빨개진다.

    ⭐ 리터럴로 적는 대가가 이것이다 — 설정이 바뀌면 조용히 거짓말이 된다.
      그래서 **대사를 시험이 진다.** 소비자는 이 응답을 믿고 구독하므로,
      틀리면 "구독했는데 아무것도 안 온다" 가 되고 원인이 API 응답이라고는 아무도 안 본다.
    """
    for rid, want in vp.DOWNSTREAM.items():
        path = os.path.join(_CFG_DIR, "%s_control.yaml" % rid)
        if not os.path.exists(path):
            pytest.fail("브리지 설정이 없다: %s" % path)
        doc = yaml.safe_load(io.open(path, encoding="utf-8"))
        key = "%s/vision_pose" % rid
        assert key in (doc.get("topics") or {}), "%s 에 %s 가 없다" % (path, key)
        blk = doc["topics"][key]

        assert blk.get("to_domain") == want["domain"], (
            "%s: 설정 to_domain=%r 인데 상수는 %r" % (rid, blk.get("to_domain"), want["domain"]))
        assert "/" + str(blk.get("remap")) == want["topic"], (
            "%s: 설정 remap=%r 인데 상수는 %r" % (rid, blk.get("remap"), want["topic"]))
        assert blk.get("type") == vp.DOWNSTREAM_TYPE, (
            "%s: 설정 type=%r 인데 상수는 %r" % (rid, blk.get("type"), vp.DOWNSTREAM_TYPE))
        q = blk.get("qos") or {}
        for k, v in vp.DOWNSTREAM_QOS.items():
            assert q.get(k) == v, "%s: qos.%s 설정=%r 상수=%r" % (rid, k, q.get(k), v)


# ---- 응답에 실제로 실리는가 (AST — 본체는 rclpy 라 import 못 한다) ---------------

def _gateway_tree():
    return ast.parse(io.open(_GATEWAY, encoding="utf-8").read())


def _keys_of_response_containing(tree, marker):
    """`marker` 키를 가진 dict 리터럴의 키 집합을 돌려준다."""
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        keys = {k.value for k in node.keys
                if isinstance(k, ast.Constant) and isinstance(k.value, str)}
        if marker in keys:
            return keys
    return set()


def test_POST_응답이_receivers_와_downstream_을_싣는다():
    """🔴 둘 중 하나라도 빠지면 빨개진다 — 그러면 종단을 판정할 값이 없다."""
    keys = _keys_of_response_containing(_gateway_tree(), "hasReceiver")
    assert keys, "hasReceiver 를 싣는 응답을 못 찾았다"
    assert "receivers" in keys, "응답에 receivers 가 없다"
    assert "downstream" in keys, "응답에 downstream 이 없다"
    assert "hasReceiverMeans" in keys, (
        "hasReceiver 가 무엇을 뜻하는지 응답이 말하지 않는다 — 그 오해가 이 작업의 계기였다")


def test_본체가_계약을_두_벌_갖지_않는다():
    """본체에 상수를 되살리면 빨개진다 — 두 벌이면 어느 쪽이 참인지 파일이 답 못 한다."""
    src = io.open(_GATEWAY, encoding="utf-8").read()
    assert "from vision_path import" in src
    assert "DOWNSTREAM_QOS = {" not in src, "본체가 QoS 상수를 다시 갖고 있다"


# ---- 재는 절차가 **실제로 되는 문장**인가 --------------------------------------

def test_재는_절차가_환경_세우기와_no_daemon_을_둘_다_갖는다():
    """🔴 2026-09-20 실측으로 붙인 시험 — 절차를 직접 돌리다 함정 셋을 밟았다.

    * RMW 를 안 맞추면 `Unknown topic` 이 난다(중계는 cyclonedds, 기본값은 fastrtps).
      쓰는 쪽이 그걸 "소비자 없음" 으로 읽으면 **정반대 결론**이다.
    * DDS 설정(`CYCLONEDDS_URI`)까지 맞춰야 한다. **URI 없이 6회 중 0회**, `bridge_env.sh`
      를 부르면 **6회 중 6회** 보였다. 값을 손으로 베끼면 현장/원격 프로파일이 어긋나므로
      절차는 **스크립트를 부르는 형태**여야 한다.
    * `--no-daemon` 을 빼면 이미 떠 있는 `ros2 daemon` 이 호출자의 RMW 와 무관하게 캐시로
      답한다. 그래서 RMW 를 **틀리게 주고도 초록이 났다**.
    """
    for rid in ("pinky1", "pinky2"):
        how = vp.downstream_contract(rid)["howToMeasure"]
        assert vp.MEASURE_ENV in how, "환경을 안 세우면 프로파일이 어긋난다: %s" % how
        assert "--no-daemon" in how, "데몬이 캐시로 답해 거짓 초록이 난다: %s" % how
        assert str(vp.DOWNSTREAM[rid]["domain"]) in how


def test_절차가_가리키는_스크립트가_실제로_있다():
    """경로가 죽으면 절차도 죽는다 — 이름이 바뀌면 여기서 걸린다."""
    assert os.path.isfile(os.path.join(_REPO, vp.MEASURE_ENV)), (
        "계약이 가리키는 %s 가 없다" % vp.MEASURE_ENV)


def test_계약의_RMW_가_브리지가_실제로_쓰는_것과_같다():
    """리터럴을 손으로 믿지 않는다 — `bridge_env.sh` 의 export 한 줄과 대사한다."""
    text = io.open(os.path.join(_REPO, vp.MEASURE_ENV), encoding="utf-8").read()
    exported = [ln.split("=", 1)[1].strip().strip('"\'')
                for ln in text.splitlines()
                if ln.strip().startswith("export RMW_IMPLEMENTATION=")]
    assert exported, "bridge_env.sh 가 RMW 를 정하지 않는다 — 계약이 기댈 바닥이 없다"
    assert vp.DOWNSTREAM_RMW == exported[0], (
        "계약이 말하는 RMW(%s) 와 브리지가 실제로 쓰는 것(%s) 이 다르다"
        % (vp.DOWNSTREAM_RMW, exported[0]))


def test_함정_설명이_응답에_실린다():
    """쓰는 쪽이 같은 자리에서 또 넘어지지 않게, 함정을 **응답이 들고 간다**."""
    c = vp.downstream_contract("pinky1")
    assert c["measureTrap"], "함정 설명이 비었다"
    for 조각 in ("daemon", "CYCLONEDDS_URI", "fastrtps"):
        assert 조각 in c["measureTrap"], "함정 %s 가 설명에서 빠졌다" % 조각
    assert c["measureFrom"] == "RELAY", "어디서 재는 문장인지 말하지 않으면 로봇 위에서 헛돈다"
