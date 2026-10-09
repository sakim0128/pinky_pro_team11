# -*- coding: utf-8 -*-
"""T10 — 재기동 정책이 **그 자리에서 유효한지**를 고정한다.

2026-09-10 실측: 유닛은 `Restart=no` 였고, 그것을 `on-failure` 로 바꿔도
**영원히 안 걸리는** 상태였다. 런처의 `cleanup()` 이 `exit 0` 으로 끝나서
게이트웨이가 어떤 이유로 죽어도 systemd 가 성공 종료로 봤기 때문이다.

⭐ 설정했다 ≠ 그 자리에서 유효하다. 두 조각이 맞물려야 재기동이 성립한다:
     ① 런처가 종료 코드를 보존한다
     ② 유닛이 정상 정지(0·130·143)만 성공으로 친다
여기서는 ①을 **실제 bash 를 돌려** 증명하고, ②를 유닛 파일에서 읽어 고정한다.
"""
import os
import re
import subprocess
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_RELAY = os.path.dirname(_HERE)
UNIT = os.path.join(_RELAY, "systemd", "field-master-gateway.service")
LAUNCHER = os.path.join(_RELAY, "launch_master_gateway.sh")


def _unit_text():
    if not os.path.exists(UNIT):
        pytest.skip("유닛 파일 없음: %s" % UNIT)
    with open(UNIT, encoding="utf-8") as fh:
        return fh.read()


def _unit_value(key):
    for line in _unit_text().splitlines():
        line = line.strip()
        if line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        if k.strip() == key:
            return v.strip()
    return None


# ---- ① 런처가 종료 코드를 보존하는가 (실제 bash 로 증명) -------------------------

def _run_trap(exit_code, preserve):
    """런처와 같은 trap 구조를 돌려 실제 종료 코드를 본다."""
    keep = 'local rc=$?' if preserve else ''
    tail = 'exit "$rc"' if preserve else 'exit 0'
    script = (
        "cleanup() {\n"
        "    %s\n"
        "    :\n"
        "    %s\n"
        "}\n"
        "trap cleanup SIGINT SIGTERM EXIT\n"
        "exit %d\n" % (keep, tail, exit_code)
    )
    return subprocess.run(["bash", "-c", script]).returncode


def test_트랩이_종료코드를_삼키면_실패가_성공으로_보인다():
    """이게 09-10 이전 상태다. 이 성질을 고정해 둬야 왜 고쳤는지가 남는다."""
    assert _run_trap(7, preserve=False) == 0, "전제: 예전 구조는 7 을 0 으로 만든다"


def test_트랩이_종료코드를_보존하면_실패가_실패로_보인다():
    assert _run_trap(7, preserve=True) == 7
    assert _run_trap(0, preserve=True) == 0


def test_런처의_cleanup_이_종료코드를_보존한다():
    if not os.path.exists(LAUNCHER):
        pytest.skip("런처 없음")
    with open(LAUNCHER, encoding="utf-8") as fh:
        text = fh.read()
    body = re.search(r"cleanup\(\)\s*\{(.*?)\n\}", text, re.S)
    assert body, "cleanup() 을 못 찾았다"
    inner = body.group(1)
    assert "local rc=$?" in inner, "종료 코드를 안 잡는다 - Restart=on-failure 가 안 걸린다"
    assert 'exit "$rc"' in inner, "보존한 코드로 안 나간다"
    assert not re.search(r"^\s*exit 0\s*$", inner, re.M), \
        "cleanup 안에 exit 0 이 남아 있다 - 실패가 성공으로 보고된다"


def test_런처가_그_트랩을_실제로_건다():
    if not os.path.exists(LAUNCHER):
        pytest.skip("런처 없음")
    with open(LAUNCHER, encoding="utf-8") as fh:
        text = fh.read()
    assert re.search(r"trap\s+cleanup\b.*\bEXIT\b", text), "trap 이 EXIT 에 안 걸려 있다"


# ---- ② 유닛이 정상 정지만 성공으로 치는가 ----------------------------------------

def test_유닛이_레포에_있다():
    """장비에만 있으면 반납·재설치 때 같이 안 따라온다 (09-09 죽은 사본과 같은 계열)."""
    assert os.path.exists(UNIT), "유닛 파일이 레포에 없다: %s" % UNIT


def test_실패하면_다시_띄운다():
    assert _unit_value("Restart") == "on-failure"


def test_정상_정지는_재기동하지_않는다():
    """0 만 적으면 systemctl stop(143)·Ctrl-C(130) 이 실패로 잡혀 되살아난다."""
    codes = (_unit_value("SuccessExitStatus") or "").split()
    assert "0" in codes
    assert "130" in codes, "SIGINT 가 빠졌다 - Ctrl-C 로 끈 것이 되살아난다"
    assert "143" in codes, "SIGTERM 이 빠졌다 - systemctl stop 이 되살아난다"


def test_재기동_폭주를_막는다():
    """깨진 소스로 무한 재기동하면 로그만 쌓이고 원인이 묻힌다."""
    assert _unit_value("StartLimitIntervalSec") is not None
    burst = _unit_value("StartLimitBurst")
    assert burst is not None and 1 <= int(burst) <= 10


def test_정리할_시간을_준다():
    """런처 cleanup 이 gz sim·socat·GUI 를 정리해야 한다."""
    v = _unit_value("TimeoutStopSec")
    assert v is not None and int(v) >= 20


def test_ExecStart_가_런처를_가리킨다():
    v = _unit_value("ExecStart") or ""
    assert v.endswith("launch_master_gateway.sh"), v


def test_부팅_대상에_들어간다():
    assert _unit_value("WantedBy") == "default.target"


def test_Linger_한계를_유닛이_문서로_남긴다():
    """Linger 는 유닛이 못 정한다. 그 사실이 파일에 없으면 다음 사람이 또 헤맨다."""
    text = _unit_text()
    assert "linger" in text.lower()
    assert "enable-linger" in text


# ---- 설치 스크립트 -------------------------------------------------------------

def test_설치는_복사가_아니라_심링크다():
    """사본이 둘이면 어느 쪽이 권위인지 파일이 답하지 못한다 (09-09 교훈)."""
    path = os.path.join(_RELAY, "scripts", "install_gateway_unit.sh")
    if not os.path.exists(path):
        pytest.skip("설치 스크립트 없음")
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    assert "ln -sfn" in text, "심링크로 설치하지 않는다"
    assert not re.search(r"^\s*cp\s+.*field-master-gateway", text, re.M), \
        "cp 로 설치하면 사본이 다시 둘이 된다"


def test_설치_스크립트가_실행중_게이트웨이를_재기동하지_않는다():
    """수업 중에 화면이 끊기면 안 된다. 재기동은 사람이 정한다."""
    path = os.path.join(_RELAY, "scripts", "install_gateway_unit.sh")
    if not os.path.exists(path):
        pytest.skip("설치 스크립트 없음")
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    assert not re.search(r"^\s*systemctl --user restart", text, re.M), \
        "설치가 마음대로 재기동한다"
    assert "enable --now" not in text, "--now 는 재기동을 유발한다"
