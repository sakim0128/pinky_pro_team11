# -*- coding: utf-8 -*-
"""서비스·액션을 토픽으로 감쌀 때 **전화가 편지가 되면서 생기는 문제**를 고정한다.

## 감싸면 무엇이 달라지나

전화(서비스)는 한 번 걸면 한 번이다. 편지(토픽)는 **두 번 올 수 있다.**
LED 는 두 번 켜도 그만이지만 `문을 연다` · `팔을 움직인다` 는 아니다.

그래서 어댑터는 세 가지를 지켜야 한다:

    ① 요청 id 가 **필수**다 — 없으면 응답을 어디에 붙일지 모르고 중복도 못 막는다
    ② 같은 id 가 다시 오면 **서비스를 다시 부르지 않는다**
    ③ 실패해도 **응답은 반드시 낸다** — 조용히 사라지면 요청자가 영원히 기다린다

여기서는 ROS 없이 그 규약(봉투 + 중복 방지)만 본다. ROS 왕복은 실기에서 따로 쟀다
(2026-09-12 리플리카 실측: 왕복 5.3ms · 재전송 2회에 서비스 호출 1회).

⚠️ 액션은 **실기 왕복 검증을 못 했다** — 시험 환경에 액션 타입이 하나도 없다.
   봉투 규약은 서비스와 같은 코드를 쓰므로 여기서 같이 고정하되, 액션 자체는
   로봇에서 한 번 돌려 보고 판단해야 한다.
"""
import io
import json
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(_HERE))
BRIDGE = os.path.join(REPO, "relay_station", "domain_bridge")
GEN = os.path.join(BRIDGE, "generate_configs.sh")
CFG = os.path.join(BRIDGE, "configs")

import sys
sys.path.insert(0, BRIDGE)
import topic_wrap as tw          # noqa: E402  — ROS 를 늦게 부르므로 여기서 안전하다


# ---- ① 봉투 ---------------------------------------------------------------------

def test_정상_요청을_읽는다():
    rid, args, deadline = tw.parse_request(
        json.dumps({"id": "abc", "args": {"data": True}, "deadline_ms": 500}))
    assert (rid, args, deadline) == ("abc", {"data": True}, 500.0)


def test_id_가_없으면_거절한다():
    """⭐ 편의로 id 를 만들어 주면 **재전송이 새 요청이 되어 부작용이 두 번 난다.**"""
    for raw in (json.dumps({"args": {}}), json.dumps({"id": "", "args": {}}),
                json.dumps({"id": "   "}), json.dumps({"id": 3})):
        with pytest.raises(tw.EnvelopeError) as e:
            tw.parse_request(raw)
        assert e.value.why == tw.WHY_NO_ID, raw


def test_망가진_봉투를_사유와_함께_거절한다():
    cases = [("{not json", tw.WHY_BAD_JSON), ("[1,2]", tw.WHY_BAD_JSON),
             (json.dumps({"id": "a", "args": 5}), tw.WHY_BAD_ARGS),
             (json.dumps({"id": "a", "deadline_ms": "빠르게"}), tw.WHY_BAD_ARGS),
             (json.dumps({"id": "a", "deadline_ms": 0}), tw.WHY_BAD_ARGS),
             (json.dumps({"id": "a", "deadline_ms": 999999}), tw.WHY_BAD_ARGS)]
    for raw, why in cases:
        with pytest.raises(tw.EnvelopeError) as e:
            tw.parse_request(raw)
        assert e.value.why == why, raw


def test_실패_응답에도_사유가_들어간다():
    """⭐ 실패도 응답이다. 사유 없는 실패는 화면에서 '생각 중' 과 구분이 안 된다."""
    env = json.loads(tw.fail_result("r1", tw.WHY_TIMEOUT, "set_led", 1200.0))
    assert env["id"] == "r1" and env["ok"] is False
    assert env["why"] == tw.WHY_TIMEOUT and "set_led" in env["detail"]
    assert env["tookMs"] == 1200.0


def test_id_를_못_읽어도_응답을_만든다():
    """id 조차 못 읽은 요청에도 응답을 낸다 — 어디로 갈지는 몰라도 화면은 이유를 본다."""
    env = json.loads(tw.fail_result(None, tw.WHY_NO_ID))
    assert env["id"] == "" and env["ok"] is False


# ---- ② 중복 방지 ------------------------------------------------------------------

def test_같은_id_는_다시_부르지_않는다():
    """🔴 이게 빨개지면 **부작용이 두 번 일어난다.**"""
    c = tw.ResultCache()
    assert c.seen("r1") is None
    c.begin("r1")
    assert c.is_inflight("r1")
    c.finish("r1", tw.ok_result("r1", {"success": True}))
    again = c.seen("r1")
    assert again is not None and json.loads(again)["ok"] is True
    assert not c.is_inflight("r1")


def test_처리_중인_id_는_가려낸다():
    c = tw.ResultCache()
    c.begin("r1")
    assert c.is_inflight("r1") and c.seen("r1") is None


def test_동시_요청_상한이_있다():
    """상한이 없으면 큐가 쌓여 터진다. 넘치면 BUSY 로 **즉시** 답한다."""
    c = tw.ResultCache()
    for i in range(tw.MAX_INFLIGHT):
        c.begin("r%d" % i)
    with pytest.raises(tw.EnvelopeError) as e:
        c.begin("one-too-many")
    assert e.value.why == tw.WHY_BUSY


def test_오래된_기억은_버린다():
    """영원히 기억하면 메모리가 샌다. 시계를 주입해서 시간을 안 쓰고 본다."""
    now = [0.0]
    c = tw.ResultCache(ttl_s=10.0, clock=lambda: now[0])
    c.begin("r1")
    c.finish("r1", "payload")
    assert c.seen("r1") == "payload"
    now[0] = 11.0
    assert c.seen("r1") is None


# ---- ③ 이름 규칙 — 두 곳이 어긋나면 조용히 안 나른다 ------------------------------------

def test_이름_규칙이_한_곳에서만_나온다():
    assert tw.topic_names("robot1", "svc", "led") == {
        "request": "robot1/svc/led/request", "result": "robot1/svc/led/result"}
    assert set(tw.topic_names("robot1", "act", "nav")) == {
        "goal", "cancel", "feedback", "result"}


def test_팀원용은_teleop_밑으로_들어간다():
    """⭐ 벌 2 는 teleop 접두어만 나른다. 규칙을 깨는 대신 **그 밑으로 들어간다.**"""
    t = tw.topic_names("robot1", "svc", "led", via_team=True)
    assert t["request"] == "robot1/teleop/svc/led/request"
    assert all("teleop/" in v for v in t.values())


def test_생성기와_어댑터가_같은_이름을_쓴다():
    """🔴 두 곳이 어긋나면 **브리지가 조용히 안 나른다.**

    증상은 "감쌌는데 아무 일도 없다" 이고, 사람은 어댑터를 의심한다.
    실제로는 브리지가 다른 이름을 기다리고 있을 뿐이다.
    """
    src = io.open(GEN, encoding="utf-8").read()
    declared = re.search(r"^WRAP_SERVICES=\(([^)]*)\)", src, re.M)
    assert declared, "생성기에 WRAP_SERVICES 선언이 없다"
    names = declared.group(1).split()
    assert names, "감싼 서비스가 하나도 선언되지 않았다 — 그러면 이 시험이 공허하다"

    for n in (1, 2, 3, 4):
        import yaml
        doc = yaml.safe_load(io.open(
            os.path.join(CFG, "robot%d_control.yaml" % n), encoding="utf-8"))
        topics = set(doc.get("topics") or {})
        for s in names:
            want = tw.topic_names("robot%d" % n, "svc", s)
            for role, t in want.items():
                assert t in topics, (
                    "브리지 설정에 %s 가 없다 (%s) — 어댑터는 이 이름으로 내놓는다"
                    % (t, role))


def test_감싼_요청은_내려가고_응답은_올라온다():
    """방향이 뒤집히면 명령이 로봇 쪽에서 관제로 가려 한다 — 아무 일도 안 일어난다."""
    import yaml
    for n, d in ((1, 10), (2, 11), (3, 12), (4, 13)):
        doc = yaml.safe_load(io.open(
            os.path.join(CFG, "robot%d_control.yaml" % n), encoding="utf-8"))
        topics = doc["topics"]
        for t, spec in topics.items():
            if "/svc/" not in t:
                continue
            frm = spec.get("from_domain", doc["from_domain"])
            to = spec.get("to_domain", doc["to_domain"])
            if t.endswith("/request"):
                assert (frm, to) == (8, d), "%s: %s -> %s" % (t, frm, to)
            elif t.endswith("/result"):
                assert (frm, to) == (d, 8), "%s: %s -> %s" % (t, frm, to)


def test_팀원에게_연_것이_없으면_벌2에도_없다():
    """기본은 닫힘 — 여는 것이 결정이다."""
    src = io.open(GEN, encoding="utf-8").read()
    team = re.search(r"^WRAP_SERVICES_TEAM=\(([^)]*)\)", src, re.M)
    assert team, "생성기에 WRAP_SERVICES_TEAM 선언이 없다"
    opened = team.group(1).split()
    import yaml
    doc = yaml.safe_load(io.open(
        os.path.join(CFG, "robot1_teleop_in.yaml"), encoding="utf-8"))
    svc_topics = [t for t in (doc.get("topics") or {}) if "/svc/" in t]
    if not opened:
        assert not svc_topics, "아무것도 안 열었는데 벌 2 에 감싼 토픽이 있다: %s" % svc_topics
    else:
        for s in opened:
            assert tw.topic_names("robot1", "svc", s, via_team=True)["request"] in svc_topics


# ---- 한계를 적어 뒀는가 --------------------------------------------------------------

def test_감싸면_잃는_것을_적어_뒀다():
    """⭐ 감싸는 것은 공짜가 아니다. 안 적으면 다음 사람이 서비스와 같다고 믿는다."""
    src = io.open(os.path.join(BRIDGE, "topic_wrap.py"), encoding="utf-8").read()
    for phrase in ("잃는 것", "취소가 늦", "실행 시점"):
        assert phrase in src, "한계 설명 '%s' 가 없다" % phrase


def test_액션이_미검증이라고_적혀_있다():
    """실기 확인을 못 한 것을 확인한 것처럼 두면 다음 사람이 믿고 쓴다."""
    src = io.open(os.path.join(BRIDGE, "topic_wrap.py"), encoding="utf-8").read()
    assert "실기 왕복 검증을 못 했다" in src
