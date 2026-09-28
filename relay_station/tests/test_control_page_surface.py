# -*- coding: utf-8 -*-
"""U-7 — 현장 관제 화면에 **배포 트리거가 없다.**

사용자 지시 2026-09-12: "CI/CD 탭 거기는 일단 뺀다."
⭐ **일단**이라 했으니 되돌릴 수 있어야 한다 — 지운 게 아니라
   `static/ops-deploy.html` 로 **옮겼다.**

## ⚠️ 이건 보안 수정이 아니다

`/api/jenkins/build` 엔드포인트는 **그대로 열려 있다**(FD-13 미해소). 버튼을 화면에서
뺀다고 엔드포인트가 닫히지 않는다 — LAN 에서 curl 하면 된다. 이 변경의 효과는
**현장 관제 중 오조작 표면을 없애는 것**이지 접근 통제가 아니다.
그렇게 적어 두지 않으면 다음 사람이 "배포 트리거는 막혔다" 고 읽는다.

## ⚠️ 이 Jenkins 는 **홈랩 플랫폼과 다른 계통**이다

`trigger_jenkins_build` 는 `localhost:8085` 의 `deploy-pinky-fleet` 잡을 쳐서
**로봇 플릿**에 배포·백업한다. 홈랩 쪽 배포는 **별도 체계**(푸시→CI→사람이 누르는
승인 화면)이고 여기와는 별개다. 한쪽 정책으로 다른 쪽을 판단하면 안 된다(사용자 확인 2026-09-12).
"""
import io
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(os.path.dirname(_HERE), "gateway_web", "static")
INDEX = os.path.join(STATIC, "index.html")
OPS = os.path.join(STATIC, "ops-deploy.html")

# 관제 화면에 있으면 안 되는 것들
TRIGGERS = ("/api/jenkins", "triggerJenkins", "pane-ci-cd", "jenkins-badge-link")


def _read(path):
    if not os.path.exists(path):
        pytest.skip("%s 없음" % os.path.basename(path))
    with io.open(path, encoding="utf-8") as fh:
        return fh.read()


def _live(text):
    """HTML 주석을 걷어낸다.

    ⭐⭐ 옮길 때 남긴 안내 주석에 `/api/jenkins/build` 가 들어 있다. 주석을 안 걷으면
       **이 검사가 그 주석을 잡는다** — 실제로 옮기는 스크립트에서 그렇게 한 번 걸렸다.
       오늘 같은 모양(검사가 자기 설명문을 잡는 것)을 네 번 밟았다.
    """
    return re.sub(r"<!--.*?-->", "", text, flags=re.S)


# ---- ⭐ 검사가 실패할 수 있는가 ------------------------------------------------------

def test_관제_화면이_실재하고_충분히_크다():
    """⭐ 파일이 비어 있으면 아래 시험은 구조적으로 통과한다."""
    t = _read(INDEX)
    assert len(t) > 50000, "index.html 이 %d 바이트뿐이다" % len(t)


def test_검사어가_옮긴_페이지에서는_실제로_잡힌다():
    """⭐⭐ 같은 검사어로 **옮긴 쪽**을 보면 걸려야 한다. 안 걸리면 검사어가 죽은 것이고,
       그러면 관제 화면이 0건인 것도 아무 의미가 없다."""
    ops = _live(_read(OPS))
    for pat in ("/api/jenkins", "triggerJenkins", "pane-ci-cd"):
        assert pat in ops, "옮긴 페이지에 %s 가 없다 — 검사어가 죽었거나 옮기다 잃었다" % pat


# ---- 본 검사 -----------------------------------------------------------------

def test_관제_화면에_배포_트리거가_없다():
    live = _live(_read(INDEX))
    found = [p for p in TRIGGERS if p in live]
    assert not found, (
        "현장 관제 화면에 배포 트리거가 남아 있다: %s" % found
        + chr(10) + "static/ops-deploy.html 로 옮긴다 — 지우지 않는다.")


def test_옮긴_것이지_지운_것이_아니다():
    """⭐ 사용자가 '일단' 이라고 했다. 되돌릴 수 있어야 한다."""
    assert os.path.exists(OPS), "옮길 곳이 없다 — 지워 버렸다"
    ops = _read(OPS)
    assert ops.count("triggerJenkins(") >= 10, "버튼을 옮기다 잃었다"
    assert "index.html" in ops, "어디로 되돌리는지 안 적었다"


def test_보안_수정이_아니라고_적었다():
    """⭐⭐ 안 적으면 다음 사람이 '배포 트리거는 막혔다' 고 읽는다.
       엔드포인트는 그대로 열려 있다(FD-13)."""
    ops = _read(OPS)
    assert "보안 수정이 아니다" in ops
    assert "그대로 열려 있다" in ops


def test_홈랩_계통과_구분해_적었다():
    """⚠️ 사용자 확인 2026-09-12: 중계장비 배포는 홈랩과 별도다.

    ⭐ 처음 쓴 안내문에 홈랩 webhook 정책을 옮겨 적었다가 사용자가 잡아 줬다 —
       한쪽 계통의 규칙을 다른 쪽 화면에 붙이면, 읽는 사람이 그 규칙으로 판단한다.
    """
    ops = _read(OPS)
    assert "계통이 다르다" in ops
    assert "deploy-pinky-fleet" in ops, "어느 잡을 치는지 안 적었다"
    assert "webhook 정책" in ops and "적용되지 않는다" in ops


def test_관제_화면에서_옮긴_자리를_표시했다():
    """빈자리만 남기면 다음 사람이 '원래 없었나' 한다."""
    t = _read(INDEX)
    assert "ops-deploy.html" in t, "어디로 옮겼는지 관제 화면에 표시가 없다"
