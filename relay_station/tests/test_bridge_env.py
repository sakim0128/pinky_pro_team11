# -*- coding: utf-8 -*-
"""브리지가 **어디서 기동하든** DDS 프로파일을 스스로 고르고, 못 고르면 말한다 (D-3/B-3).

## 무엇이 문제였나

유닛이 DDS 설정을 박아 뒀다:

    Environment=CYCLONEDDS_URI=file://…/configs/cyclonedds.xml

그 파일은 교육장 유선 NIC 에 바인딩을 박아 둔 **현장 전용**이다. 현장 밖에서는
CycloneDDS 가 도메인 생성에 실패하고(2026-09-09 실측), `Restart=always` 라
**조용히 재시작 루프**를 돈다. 같은 기계의 `launch_master_gateway.sh` 는 NIC 유무를
보고 고르는데 브리지만 박혀 있었다.

⭐ **같은 기계의 두 서비스가 같은 질문에 다르게 답하면 안 된다.**

## ⚠️ 설계서 B-3 의 문구를 그대로는 못 만든다 — 둘이 서로 어긋난다

    B-3 수락   "현장 NIC 를 내린 상태로 기동하면 **사유를 남기고 멈춘다**"
    §4 하지말것 "`Restart=always` 를 지우지 말 것. 문제는 재시작이 아니라
                **실패를 안 말하는 것**이다. B-3 은 사유를 남기게 하는 것이지
                재시작을 끄는 것이 아니다."

그리고 D-3 의 처방(런처와 같은 자동 판정)을 적용하면 NIC 가 없을 때는 **offsite 로
기동하는 것이 맞다** — 멈추면 오히려 런처와 답이 갈린다. 그래서 이렇게 갈랐다:

    일시적 실패(상대가 안 떴다·망이 깜빡)        → 재시작이 맞다. 안 건드린다.
    설정 실패(고른 프로파일 파일이 없다)          → 재시작해도 같다. **78 로 죽고 멈춘다.**

`RestartPreventExitStatus=78` 이 그 78 만 받는다. `Restart=always` 는 그대로다.

⭐ 그래서 이 시험이 고정하는 수락은 **B-3′** 이다:
   *NIC 가 없으면 offsite 를 골랐다고 로그에 남기고 기동한다. 고른 프로파일 파일이
   없으면 사유를 남기고 78 로 멈춘다.* — 원문 B-3 은 만족시킬 수 없다고 보고했다.
"""
import io
import os
import shutil
import stat
import subprocess

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(_HERE))
BRIDGE = os.path.join(REPO, "relay_station", "domain_bridge")
ENV_SH = os.path.join(BRIDGE, "bridge_env.sh")
UNIT = os.path.join(BRIDGE, "systemd", "pinky-domain-bridge@.service")
WATCHDOG = os.path.join(BRIDGE, "systemd", "pinky-bridge-watchdog.service")
LAUNCHER = os.path.join(REPO, "relay_station", "launch_master_gateway.sh")

EX_CONFIG = 78


def _bash():
    for name in ("bash", "sh"):
        p = shutil.which(name)
        if p:
            return p
    pytest.fail("bash 가 없다 — 이 시험은 셸 동작을 본다")


def _run(tmp_path, nic_present, repo_root=None):
    """가짜 `ip` 를 PATH 앞에 놓고 실제 스크립트를 돌린다.

    ⭐ 프로브를 환경변수로 갈아끼우지 않고 **진짜 경로**(`ip link show`)를 태운다.
       주입식 seam 은 그 seam 만 시험하게 되고, 정작 실기에서 도는 줄은 안 밟는다.
    """
    shim = tmp_path / "shim"
    shim.mkdir(exist_ok=True)
    ip = shim / "ip"
    ip.write_text("#!/bin/sh\nexit %d\n" % (0 if nic_present else 1), encoding="utf-8")
    os.chmod(str(ip), os.stat(str(ip)).st_mode | stat.S_IEXEC)
    env = dict(os.environ, PATH=str(shim) + os.pathsep + os.environ.get("PATH", ""))
    if repo_root is not None:
        env["REPO_ROOT"] = str(repo_root)
    r = subprocess.run([_bash(), ENV_SH], cwd=REPO, env=env,
                       capture_output=True, timeout=120)
    return (r.returncode,
            r.stdout.decode("utf-8", "replace"),
            r.stderr.decode("utf-8", "replace"))


def _text(p):
    return io.open(p, encoding="utf-8", errors="replace").read()


def _directives(unit_path):
    """유닛의 **실효** 지시자만 — 주석은 뺀다.

    ⭐⭐ 처음엔 `"RestartPreventExitStatus=78" in text` 로 봤다. 그런데 그 줄을
       `#RestartPreventExitStatus=78` 로 **주석 처리해도 부분문자열은 그대로 있어서**
       뮤테이션 5종 중 이것 하나만 안 잡혔다. 부분문자열로 구조를 판정하면 틀린다.
    """
    got = set()
    for line in _text(unit_path).split(chr(10)):
        s = line.strip()
        if not s or s.startswith("#") or s.startswith("[") or "=" not in s:
            continue
        got.add(s)
    return got


# ---- B-3′ 동작 ------------------------------------------------------------------

def test_현장_NIC_가_있으면_field_를_고른다(tmp_path):
    code, out, err = _run(tmp_path, nic_present=True)
    assert code == 0, err
    assert "field" in err, err
    assert out.strip().endswith("configs/cyclonedds.xml"), out
    assert "offsite" not in out


def test_현장_NIC_가_없으면_offsite_를_고르고_사유를_남긴다(tmp_path):
    """⭐ 여기서 멈추면 런처와 답이 갈린다 — 기동하되 **왜 그 프로파일인지** 남긴다."""
    code, out, err = _run(tmp_path, nic_present=False)
    assert code == 0, err
    assert "offsite" in err, "어느 프로파일을 왜 골랐는지 안 남겼다: %r" % err
    assert out.strip().endswith("configs/cyclonedds-offsite.xml"), out


def test_고른_파일이_없으면_사유를_남기고_78_로_멈춘다(tmp_path):
    """🔴 조용한 재시작 루프를 막는 자리. 78 이 아니면 유닛이 계속 되살린다."""
    empty = tmp_path / "norepo"
    (empty / "configs").mkdir(parents=True)
    code, _out, err = _run(tmp_path, nic_present=False, repo_root=empty)
    assert code == EX_CONFIG, "설정 오류인데 %d 로 죽었다 — 유닛이 루프를 돈다" % code
    assert "설정 오류" in err and "cyclonedds-offsite.xml" in err, err


def test_source_해도_78_이_부모까지_간다(tmp_path):
    """유닛은 `source … && exec …` 형태다. 여기서 안 죽으면 브리지가 그냥 뜬다."""
    empty = tmp_path / "norepo2"
    (empty / "configs").mkdir(parents=True)
    shim = tmp_path / "shim"
    shim.mkdir(exist_ok=True)
    (shim / "ip").write_text("#!/bin/sh\nexit 1\n", encoding="utf-8")
    os.chmod(str(shim / "ip"), os.stat(str(shim / "ip")).st_mode | stat.S_IEXEC)
    env = dict(os.environ, PATH=str(shim) + os.pathsep + os.environ.get("PATH", ""),
               REPO_ROOT=str(empty))
    r = subprocess.run(
        [_bash(), "-c", 'source "$1" >/dev/null && echo REACHED', "_", ENV_SH],
        cwd=REPO, env=env, capture_output=True, timeout=120)
    assert r.returncode == EX_CONFIG, r.stdout.decode("utf-8", "replace")
    assert b"REACHED" not in r.stdout


# ---- 두 서비스가 같은 답을 하는가 ------------------------------------------------------

def test_런처와_같은_NIC_을_본다():
    """⭐⭐ NIC 이름이 런처와 여기 두 곳에 있다. **어긋나면 여기서 먼저 빨개진다.**

    한 곳으로 모으려면 런처를 고쳐야 하는데 런처는 현장의 살아 있는 기동 경로다.
    실기 검증 없이 건드리지 않기로 하고, 대신 일치를 시험으로 고정했다.
    """
    import re
    pat = re.compile(r'FIELD_NIC="?\$?\{?FIELD_NIC:-([A-Za-z0-9_]+)\}?"?|FIELD_NIC="([A-Za-z0-9_]+)"')
    def nic_of(path):
        for m in pat.finditer(_text(path)):
            return m.group(1) or m.group(2)
        return None
    a, b = nic_of(ENV_SH), nic_of(LAUNCHER)
    assert a and b, "NIC 이름을 못 읽었다 (bridge_env=%r launcher=%r)" % (a, b)
    assert a == b, "브리지 %r 와 런처 %r 가 다른 NIC 을 본다" % (a, b)


# ---- 유닛이 실제로 그 길을 타는가 ------------------------------------------------------

@pytest.mark.parametrize("unit", [UNIT, WATCHDOG], ids=["bridge", "watchdog"])
def test_유닛이_DDS_설정을_안_박는다(unit):
    live = _directives(unit)
    assert not [d for d in live if d.startswith("Environment=CYCLONEDDS_URI=")], (
        "유닛이 DDS 프로파일을 박아 뒀다 — 현장 밖에서 조용히 죽는다")
    assert [d for d in live if d.startswith("ExecStart=") and "bridge_env.sh" in d], (
        "ExecStart 가 프로파일 판정을 안 거친다")


@pytest.mark.parametrize("unit", [UNIT, WATCHDOG], ids=["bridge", "watchdog"])
def test_설정오류만_재시작에서_뺀다(unit):
    """⚠️ §4: `Restart=always` 를 지우지 말 것 — 그대로 있는지도 같이 고정한다."""
    live = _directives(unit)
    assert "RestartPreventExitStatus=78" in live, (
        "설정 오류가 루프를 돈다 — 주석 처리된 줄은 안 센다")
    assert "Restart=always" in live, "일시적 실패까지 재시작을 껐다 — §4 위반"
