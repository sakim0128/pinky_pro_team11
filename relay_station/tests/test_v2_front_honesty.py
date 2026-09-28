# -*- coding: utf-8 -*-
"""R-5 ⑤ · V2 관제 화면(`fleet_control_v2.js`)의 정직성 — 초록 기본값 금지 · DEMO 배지 · 마커 ID.

이 장비에는 JS 런타임이 없어서 **주석을 걷어 낸 소스**로 구조를 잰다. 2026-09-24 에는 내장 브라우저로
데모·무데이터·가짜 API 세 경우를 실제로 띄워 색을 확인했다(커밋 메시지). 그때 찾은 거짓 초록 넷:
진단 카드 머리(`d.overall ? "ok"`) · 카메라 상태(`state.ops.vision ? "ok"`) · 데이터 없는 로봇 카드
(`toneForRobot` 기본값 "ok") · E-STOP 로봇 카드(사유 문구만 보고 "ok").
"""
import pytest  # 내보내기: 팀11 레포에서 건너뛰는 시험 표식용
import os
import re

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
JS = os.path.join(REPO, "relay_station", "gateway_web", "static", "fleet_control_v2.js")
MARKERS = os.path.join(REPO, "tablet", "vision", "config", "markers.yaml")


def _code():
    """주석을 걷어 낸 JS — 주석 속 글이 구조 판정을 속이지 않게."""
    src = open(JS, encoding="utf-8").read()
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return "\n".join(re.sub(r"(^|[^:\"'])//.*$", r"\1", line) for line in src.splitlines())


def _function(code, name):
    i = code.index("function " + name)
    j = code.index("\n  }", i)
    return code[i:j]


def test_DEMO_배지가_화면_맨_위에_붙는다():
    code = _code()
    block = code[code.index("if (DEMO) {"):]
    block = block[:block.index("initTabs();")]
    assert "DEMO DATA" in block and "prepend" in block
    assert '"[DEMO] "' in block                     # 탭 제목에도 — 캡처가 잘려도 남는다


def test_값이_있기만_하면_초록이_되는_패턴이_없다():
    code = _code()
    assert not re.search(r"d\.overall\s*\?\s*\"ok\"", code)
    assert not re.search(r"state\.ops\.vision\s*\?\s*\"ok\"", code)
    assert "pillTone(d.overall)" in code and "pillTone(vis)" in code


def test_pillTone_은_statusClass_를_따른다():
    body = _function(_code(), "pillTone")
    assert "statusClass(v)" in body and '"neutral"' in body


def test_로봇_카드는_데이터가_없으면_대기__ESTOP_링크유실은_빨강():
    body = _function(_code(), "toneForRobot")
    lines = [l.strip() for l in body.splitlines() if l.strip()]
    assert lines[1] == 'if (!r.known) return "pending";', lines[:3]       # 첫 판정이다
    assert "r.driveCode === 8" in body and "r.driveCode === 9" in body
    derive = _function(_code(), "deriveRobot")
    assert "known: Boolean(fleetRobot(index))" in derive
    assert "driveCode: ls.drive_state" in derive


def test_위치_출처_행이_있고_없으면_미수신():
    code = _code()
    assert "위치 출처" in code and 'r.poseSource || "미수신"' in code


@pytest.mark.skip(reason='원 저장소 tablet/ 설정을 대사한다 — 태블릿 코드는 이 레포에 싣지 않았다')
def test_데모_마커_ID_가_태블릿_설정_정본과_같다():
    """`tablet/vision/config/markers.yaml` 이 마커 ID 의 정본이다(T-9)."""
    cfg = yaml.safe_load(open(MARKERS, encoding="utf-8"))
    robots = {r["name"]: int(r["id"]) for r in cfg["robots"]}
    refs = sorted(int(m["id"]) for m in cfg["reference"]["markers"])
    demo = _function(_code(), "demoData")
    assert "robot_marker_ids:{pinky1:%d,pinky2:%d}" % (robots["pinky1"], robots["pinky2"]) in demo
    assert "reference_ids:[%s]" % ",".join(map(str, refs)) in demo
    assert "marker_id:%d," % robots["pinky1"] in demo and "marker_id:%d," % robots["pinky2"] in demo
