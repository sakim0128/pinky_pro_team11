# -*- coding: utf-8 -*-
"""팀원 주행 벌(벌 2)이 **원래 막으려던 사고를 그대로 막는가**.

## 무엇을 바꿨나

사용자 지시(2026-09-12): *"팀원 노트북도 영상 처리는 안 되어도 **직접 주행에 대한
관제는 기능 구현이 되어야 한다.**"*

그런데 `generate_configs.sh` 머리말의 비대칭 원칙은 이랬다:

> `9 -> 어디든 : 없음. 팀원 도메인에서 나가는 경로는 단 하나도 만들지 않는다.`
> `(팀원 시뮬레이션 cmd_vel 이 실물 로봇으로 나가는 사고 차단)`

⭐ **막으려던 것은 "팀원" 이 아니라 "시뮬레이션 트래픽" 이다.** 그래서 길을 막는 대신
   **이름을 가른다**: 벌 2 는 `robotN/teleop/cmd_vel` 만 보고, 팀원이 시뮬에 쓰는
   `robotN/cmd_vel` 은 **쳐다보지 않는다.** 조종기가 일부러 그 이름으로 쏘지 않는 한
   아무것도 안 나간다.

이 성질은 **이름이 갈려 있다는 것 하나**로만 성립한다. 그래서 여기서 고정한다.
누가 편의를 위해 벌 2 에 `robotN/cmd_vel` 을 한 줄 넣는 순간 사고가 되살아나는데,
설정 파일만 보면 그게 한 줄 추가로 보인다.

## 두 벌로 쪼갠 이유

    robotN_teleop_in.yaml    9 -> 8   팀원이 보낸 것을 관제국이 **본다**
    robotN_teleop_out.yaml   8 -> N   그것을 로봇에게 **내보낸다**

`_out` 만 끄면 화면에는 그대로 보이면서 로봇은 안 움직인다 — **관측을 잃지 않는 정지**다.
한 벌로 뭉쳐 두면 끄는 순간 무엇을 시키려 했는지도 같이 안 보이게 된다.
"""
import io
import os
import re

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(_HERE))
CFG = os.path.join(REPO, "relay_station", "domain_bridge", "configs")
GEN = os.path.join(REPO, "relay_station", "domain_bridge", "generate_configs.sh")

RELAY_DOMAIN = 8
TEAM_DOMAIN = 9
ROBOT_DOMAINS = {1: 10, 2: 11, 3: 12, 4: 13}

TELEOP_PREFIX = "teleop/"


def _load(name):
    p = os.path.join(CFG, name)
    if not os.path.exists(p):
        pytest.fail("설정이 없다: %s — `generate_configs.sh` 를 돌렸는가" % name)
    return yaml.safe_load(io.open(p, encoding="utf-8"))


def _topics(doc):
    return doc.get("topics") or {}


# ---- ⭐ 볼 것이 있는가 --------------------------------------------------------------

def test_네_대_모두_두_벌이_생겼다():
    """⭐⭐ 파일이 없으면 아래 시험은 구조적으로 통과한다."""
    for n in ROBOT_DOMAINS:
        for half in ("in", "out"):
            doc = _load("robot%d_teleop_%s.yaml" % (n, half))
            assert _topics(doc), "robot%d_teleop_%s 에 토픽이 없다" % (n, half)


# ---- 🔴 원래 막으려던 사고를 여전히 막는가 ------------------------------------------------

def test_팀원_시뮬_이름을_벌2가_안_본다():
    """🔴 이게 빨개지면 **시뮬 cmd_vel 이 실물 로봇으로 나간다.**

    벌 2 가 보는 이름은 `teleop/` 접두어가 붙은 것뿐이어야 한다.
    """
    bad = []
    for n in ROBOT_DOMAINS:
        for half in ("in", "out"):
            fn = "robot%d_teleop_%s.yaml" % (n, half)
            for t in _topics(_load(fn)):
                if TELEOP_PREFIX not in t:
                    bad.append("%s: %s" % (fn, t))
    assert not bad, (
        "팀원 주행 벌이 teleop 전용이 아닌 이름을 나른다 — 팀원 도메인에서 돌던"
        + chr(10) + "시뮬레이션 트래픽이 그대로 실물 로봇으로 나간다." + chr(10)
        + chr(10) + chr(10).join("  " + b for b in bad))


def test_시뮬이_쓰는_이름은_명시적으로_제외다():
    """음성 대조군 — `robotN/cmd_vel` 이 벌 2 어디에도 없어야 한다."""
    for n in ROBOT_DOMAINS:
        for half in ("in", "out"):
            names = set(_topics(_load("robot%d_teleop_%s.yaml" % (n, half))))
            assert "robot%d/cmd_vel" % n not in names, \
                "robot%d_teleop_%s 가 시뮬 이름을 물었다" % (n, half)


# ---- 방향과 도메인이 맞는가 -----------------------------------------------------------

def test_들어오는_다리는_팀원에서_관제로만_간다():
    for n in ROBOT_DOMAINS:
        doc = _load("robot%d_teleop_in.yaml" % n)
        assert doc["from_domain"] == TEAM_DOMAIN, doc["from_domain"]
        assert doc["to_domain"] == RELAY_DOMAIN, doc["to_domain"]
        for t, spec in _topics(doc).items():
            assert spec.get("to_domain", RELAY_DOMAIN) == RELAY_DOMAIN, t
            assert "remap" not in spec, "들어오는 다리에서 이름을 바꾸면 추적이 끊긴다: %s" % t


def test_나가는_다리만_로봇_도메인에_닿는다():
    """⭐ 로봇 도메인에 닿는 유일한 자리다. 여기만 끄면 바퀴가 선다."""
    for n, d in ROBOT_DOMAINS.items():
        doc = _load("robot%d_teleop_out.yaml" % n)
        assert doc["from_domain"] == RELAY_DOMAIN
        assert doc["to_domain"] == d, "robot%d 가 도메인 %s 로 간다" % (n, doc["to_domain"])
        for t, spec in _topics(doc).items():
            assert spec.get("remap") == "cmd_vel", \
                "로봇 안에서는 표준 이름이어야 온보드를 안 고친다: %s" % t
    # 들어오는 다리는 로봇 도메인에 **못 닿는다**
    for n in ROBOT_DOMAINS:
        doc = _load("robot%d_teleop_in.yaml" % n)
        for t, spec in _topics(doc).items():
            assert spec.get("to_domain", doc["to_domain"]) not in ROBOT_DOMAINS.values(), \
                "들어오는 다리가 로봇에 직접 닿는다 — 그러면 끌 손잡이가 하나로 줄어든다: %s" % t


def test_한_로봇의_벌이_다른_로봇에_안_닿는다():
    """B-2 와 같은 성질 — 이름에 자기 번호가 박혀 있어야 한다."""
    for n in ROBOT_DOMAINS:
        for half in ("in", "out"):
            for t in _topics(_load("robot%d_teleop_%s.yaml" % (n, half))):
                assert t.startswith("robot%d/" % n), \
                    "robot%d 벌이 %s 를 나른다" % (n, t)


# ---- 🔴 루프 -------------------------------------------------------------------

def test_미러와_teleop_이름이_안_겹친다():
    """🔴 겹치면 9 → 8 → 9 로 메시지가 **돈다.**

    미러는 8 → 9 이고 들어오는 다리는 9 → 8 이다. 같은 이름이 양쪽에 있으면
    브리지 둘이 서로의 출력을 먹는다. 조용히 도는 루프라 화면에는 "잘 되는 것" 처럼
    보이고, AP 만 죽는다.
    """
    mirror = set(_topics(_load("team_mirror.yaml")))
    for n in ROBOT_DOMAINS:
        incoming = set(_topics(_load("robot%d_teleop_in.yaml" % n)))
        overlap = mirror & incoming
        assert not overlap, "미러와 teleop 들어오는 다리가 겹친다(루프): %s" % sorted(overlap)


def test_미러에는_여전히_명령이_없다():
    """미러는 읽기 전용이다 — 여기에 명령이 들어가면 팀원 도메인이 명령 출처가 된다."""
    mirror = _topics(_load("team_mirror.yaml"))
    for t in mirror:
        last = t.rstrip("/").split("/")[-1]
        assert last not in ("cmd_vel", "goal_pose", "mission_cmd"), \
            "읽기 전용 미러에 명령이 들어 있다: %s" % t


# ---- 규약이 글로도 남아 있는가 ---------------------------------------------------------

def test_생성기가_바뀐_원칙을_적어_뒀다():
    """⭐⭐ 옛 머리말은 `9 -> 어디든 : 없음` 이라고 단언한다. 그대로 두면 **문서가 거짓말**이
       되고, 다음 사람이 그 문장을 믿고 설계를 판단한다."""
    src = io.open(GEN, encoding="utf-8").read()
    assert "teleop" in src, "생성기가 벌 2 를 안 만든다"
    assert "이름을 가른다" in src, "왜 안전한지(이름 분리)를 안 적었다"
    assert "기본으로 안 켠다" in src or "자동으로 켜지 않는다" in src, \
        "기본 꺼짐이라는 것을 안 적었다"


def test_미러_파일의_단언이_사실과_맞는다():
    """⭐⭐ `team_mirror.yaml` 은 "팀원 쪽에서 명령이 나가는 일이 **구조적으로 불가능**하다"
       고 적고 있었다. 벌 2 가 생긴 뒤에도 그 문장이 남아 있으면 그게 제일 위험하다 —
       읽는 사람이 없는 보호를 있다고 믿는다.
    """
    txt = io.open(os.path.join(CFG, "team_mirror.yaml"), encoding="utf-8").read()

    # ⭐⭐ 처음엔 `"구조적으로 불가능" not in txt` 로 썼다가 **내 정정문 안의 인용구**에
    #    걸렸다. 정정은 무엇이 틀렸는지 그대로 인용해야 읽는 사람이 안다 — 인용을
    #    피하려고 말을 바꾸면 그건 검사를 속이는 것이지 고치는 게 아니다.
    #    그래서 문자열이 아니라 **그 주장이 철회되었는지**를 본다.
    if "구조적으로 불가능" in txt:
        assert "더 이상 사실이 아니다" in txt, (
            "미러 파일이 아직 '팀원은 명령을 못 낸다' 고 **단언**한다 — 벌 2 가 그 문장을"
            + chr(10) + "깼다. 없는 보호를 있다고 적어 두는 것이 보호가 없는 것보다 나쁘다."
            + chr(10) + "지우든지, 철회되었다고 적든지 해야 한다.")
    assert "이름이 갈려" in txt, "지금의 보호가 무엇인지(이름 분리)를 안 적었다"
    assert "teleop" in txt, "어디로 명령이 나가는지(벌 2)를 미러 파일이 안 가리킨다"
