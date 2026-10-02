# -*- coding: utf-8 -*-
"""관제 평면(도메인 8)에서 **명령은 로봇별 이름으로만** 나간다.

## 무엇을 막는가

브리지 설정이 스스로 이유를 적어 두고 있다:

> "관제국은 로봇별로 구분된 이름으로 발행하고, 브릿지가 해당 로봇 도메인 안에서만
>  표준 이름으로 되돌린다. 이렇게 해야 **하나의 목표 지점이 4대 전부에게 동시에
>  전달되는 사고**가 발생하지 않는다."

그런데 2026-09-12 실측에서 관제 쪽 발행자 **두 곳**이 접두어 없는 `/goal_pose` 를
쓰고 있었다. `cmd_vel`·`mission_cmd` 는 접두어가 붙어 있었으니 **오타가 아니라
한 토픽만 안전장치를 우회한 상태**였다.

    게이트웨이  `/goal_pose`      ← 클릭-내비게이션
    손 조작 스크립트 `/goal_pose`  ← 같은 파일 머리말은 "로봇별 이름으로 발행한다" 고 적혀 있었다

⭐ **부분 수정은 수정이 아니다.** 설계서는 게이트웨이 한 곳만 지목했는데, 한 곳만
   고치면 브리지 수락 시험은 통과하면서 사고 경로는 손 스크립트로 그대로 열려 있다.
   그래서 이 시험은 파일을 지목하지 않고 **관제 쪽 발행자 전부**를 훑는다.

## 왜 시험으로 고정하나

이건 이름이 **두 곳에서 일치해야** 성립하는 성질이다 — 발행하는 쪽과 브리지 설정.
한쪽만 고치면 조용히 어긋나고, 증상은 "브리지가 안 나른다" 로 나타나서
**설정 탓으로 오진**하게 된다. 어긋남 자체를 여기서 잡는다.

⚠️ 로봇 **안**(도메인 10~13)에서는 `/goal_pose` 가 **맞는 이름**이다(SLAM·Nav2 호환).
   그래서 `robot_onboard/` 는 대상이 아니다 — 이 시험은 `relay_station/` 만 본다.
"""
import io
import os
import re

import pytest
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(_HERE))
RELAY = os.path.join(REPO, "relay_station")
BRIDGE_CFG = os.path.join(RELAY, "domain_bridge", "configs")

CONTROL_DOMAIN = 8

# `create_publisher(Type, '/topic', qos)` — 파이썬 소스와 셸 안의 파이썬 heredoc 둘 다.
PUBLISHER = re.compile(r"create_publisher\(\s*\w+\s*,\s*['\"]([^'\"]+)['\"]")

PREFIXED = re.compile(r"^/pinky[1-4]/")


def _bridge_downlink_names():
    """브리지가 도메인 8에서 **받아 내려보내는** 토픽들 → (전체키 집합, 명령어 집합)."""
    keys, commands = set(), set()
    # ⭐ **명령은 `pinkyN_control.yaml` 의 다운링크뿐이다.** 처음엔 configs/ 전체를
    #    읽었는데, `team_mirror.yaml`(8→9 읽기 전용 미러)의 `bridge/health` 까지
    #    "명령" 으로 잡혀 감시자가 거짓 양성으로 걸렸다. 미러는 명령 경로가 아니다.
    for fn in sorted(os.listdir(BRIDGE_CFG)):
        if not re.match(r"pinky[1-4]_control\.yaml$", fn):
            continue
        with io.open(os.path.join(BRIDGE_CFG, fn), encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
        default_from = doc.get("from_domain")
        for topic, spec in (doc.get("topics") or {}).items():
            if not isinstance(spec, dict):
                continue
            if spec.get("from_domain", default_from) != CONTROL_DOMAIN:
                continue
            keys.add("/" + topic.lstrip("/"))
            commands.add(topic.rstrip("/").split("/")[-1])
    return keys, commands


def _relay_publishers():
    """[(상대경로, 줄번호, 토픽)] — 중계 쪽 소스가 발행하는 이름 전부."""
    got = []
    for root, dirs, files in os.walk(RELAY):
        dirs[:] = [d for d in dirs if d not in ("__pycache__", "tests", "var")]
        for f in files:
            if not f.endswith((".py", ".sh")):
                continue
            p = os.path.join(root, f)
            rel = os.path.relpath(p, REPO).replace(os.sep, "/")
            try:
                text = io.open(p, encoding="utf-8", errors="replace").read()
            except OSError:
                continue
            for i, line in enumerate(text.split(chr(10)), 1):
                for topic in PUBLISHER.findall(line):
                    got.append((rel, i, topic))
    return got


# ---- ⭐ 검사가 실패할 수 있는가 ------------------------------------------------------

def test_브리지_화이트리스트를_읽어낸다():
    """⭐⭐ 명령어 집합이 비면 아래 본 검사는 구조적으로 통과한다."""
    keys, commands = _bridge_downlink_names()
    assert commands, "브리지 설정에서 다운링크 명령을 하나도 못 읽었다"
    # 2026-09-29: goal_pose·mission_cmd·cmd_vel 다운링크는 없다(로봇은 lane_agent_node 뿐) — 남은 명령이 화이트리스트다
    assert {"route", "lane_command", "command", "pose_fix"} <= commands, commands
    assert "vision_pose" not in commands, commands          # 2026-10: 소비자 0 인 /api/vision/pose 와 함께 지웠다
    assert not {"goal_pose", "cmd_vel", "mission_cmd"} & commands, commands
    assert "/pinky1/pose_fix" in keys and "/pinky1/vision_pose" not in keys, keys


def test_볼_발행자가_있다():
    pubs = _relay_publishers()
    assert len(pubs) >= 5, "중계 쪽 발행자를 %d 개밖에 못 찾았다" % len(pubs)


def test_탐지기가_접두어_없는_이름을_잡는다():
    """⭐ 양성 대조군 — 정규식이 죽으면 본 검사가 조용히 0건이 된다."""
    assert PUBLISHER.findall("pub = node.create_publisher(PoseStamped, '/goal_pose', 10)") \
        == ["/goal_pose"]
    assert not PREFIXED.match("/goal_pose")
    assert PREFIXED.match("/pinky1/vision_pose")
    assert PREFIXED.match("/pinky1/pose_fix")
    assert not PREFIXED.match("/robot1/vision_pose"), "이름은 pinkyN 하나다 (2026-09-29)"
    assert not PREFIXED.match("/pinky5/pose_fix"), "로봇은 1~4 다"


# ---- 본 검사 -----------------------------------------------------------------

def test_명령은_로봇별_이름으로만_발행한다():
    """🔴 이게 빨개지면 **하나의 명령이 4대 전부에게 가는** 경로가 열린 것이다."""
    _keys, commands = _bridge_downlink_names()
    bad = [(rel, ln, t) for rel, ln, t in _relay_publishers()
           if t.rstrip("/").split("/")[-1] in commands and not PREFIXED.match(t)]
    assert not bad, (
        "관제 평면에서 접두어 없는 명령 토픽을 발행한다 — 로봇이 같은 도메인에"
        + chr(10) + "들어오면 그 하나가 4대 전부에게 간다." + chr(10)
        + "브리지도 이 이름은 안 나른다(설정은 /pinkyN/<이름> 을 기다린다)." + chr(10)
        + chr(10) + chr(10).join("  %s:%d  ->  %s" % b for b in bad))


def test_발행하는_명령_이름을_브리지가_실제로_나른다():
    """반대 방향 — 이름을 붙였는데 브리지 설정에 그 키가 없으면 역시 안 간다."""
    keys, commands = _bridge_downlink_names()
    missing = [(rel, ln, t) for rel, ln, t in _relay_publishers()
               if t.rstrip("/").split("/")[-1] in commands and t not in keys]
    assert not missing, (
        "브리지 설정에 없는 명령 이름으로 발행한다 — 조용히 아무 데도 안 간다."
        + chr(10) + chr(10)
        + chr(10).join("  %s:%d  ->  %s" % m for m in missing))


@pytest.mark.skip(reason='음성 대조군의 대상(원 저장소 robot_onboard 의 /goal_pose 발행 스크립트)이 이 레포에 없다 — 여기 로봇 코드(pinky_fleet_agent)는 NavigateToPose 액션으로 목표를 보내 /goal_pose 문자열이 0 이다. 양성 시험(중계 이름)은 그대로 돈다')
def test_로봇_온보드는_표준_이름을_지킨다():
    """⚠️ 로봇 **안**에서는 `/goal_pose` 가 맞다 — 여기까지 고치면 Nav2 가 깨진다.

    범위를 잘못 넓히는 것을 막는 **음성 대조군**이다.
    """
    onboard = os.path.join(REPO, "robot_onboard")
    if not os.path.isdir(onboard):
        pytest.fail("robot_onboard 가 없다 — 이 시험의 전제가 깨졌다")
    hits = 0
    for root, _d, files in os.walk(onboard):
        for f in files:
            if not f.endswith(".py"):
                continue
            text = io.open(os.path.join(root, f), encoding="utf-8",
                           errors="replace").read()
            hits += text.count("'/goal_pose'")
    assert hits >= 1, "온보드에서 표준 이름이 사라졌다 — 범위를 잘못 넓힌 것이 아닌지 본다"
