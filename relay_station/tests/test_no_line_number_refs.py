# -*- coding: utf-8 -*-
"""코드가 **이 레포 파일을 줄 번호로** 가리키지 않는다.

2026-09-12, 병합 관제 세션이 내 주석에서 잡았다. `case` 로 못 옮기는 자리 셋을
`01-relay.sh: 80 / 109 / 120` 으로 적었는데 실제로는 `106 / 135 / 147` 이었다.

⭐⭐ **주석을 더하는 커밋이 그 주석 안의 줄 번호를 밀어 버린다.** 같은 파일에 주석
   12줄을 더했으니 그 표는 **자기 커밋 안에서 이미** 틀린 채로 태어났다.
   가리킨 자리에는 `dmesg` 플랩 검사, `if [ "$CNT" -gt 1 ]`, ARP 캐시 단계가 있었다.

⭐ 주장이 맞고 **좌표만 틀린** 것이 더 나쁘다. 읽는 사람은 정밀해 보이는 참조를 믿고
   그 자리를 열었다가 엉뚱한 코드를 보고 자기가 잘못 읽었다고 생각한다.
   `amcl_pose_relay.py` 가 그랬다 — "게이트웨이가 `/robotN/pose` 를 `PoseStamped` 로
   구독한다(`gateway_web_server.py:1352`)" 는 **주장은 참**인데 `:1352` 는
   `elif mode == 'watermark':` 였다(진짜 자리는 `:1695`).

대신 **찾을 수 있는 것**을 적는다 — 검색어나 식별자. 줄이 밀려도 안 틀린다:

    ❌ `gateway_web_server.py:1352`
    ✅ `gateway_web_server.py` 의 `PoseStamped, '/robot1/pose'` 구독

## ⭐⭐ 이 파일의 첫 판도 못 실패하는 검사였다

처음엔 "주석 줄만 본다" 고 `#` · `\"\"\"` · `*` 로 시작하는 줄만 훑었다. 그런데
`amcl_pose_relay.py` 의 그 참조는 **docstring 본문**이라 백틱으로 시작한다 — 못 봤고,
검사는 **깨끗해서가 아니라 못 봐서** 초록이었다. 답을 미리 알고 있어서 잡았다.

그래서 줄 필터를 **버렸다.** 모든 줄을 훑고, **이 레포에 실재하는 파일**을 가리키는
것만 센다. 무엇이 주석인지 판정하지 않으니 그 판정이 틀릴 일도 없다.

⚠️ **문서는 대상이 아니다.** 보고서·요청서는 *기록*이라 as-of 좌표가 그 자체로 의미가
   있다(그때 그 줄이었다). 여기서 막는 것은 **살아 있는 코드**다.
⚠️ 다른 레포 파일도 대상이 아니다 — 여기서 확인할 수 없으니
   금지해도 검사가 못 한다. 대신 어느 레포인지·언제 봤는지를 같이 적는다.
"""
import io
import os
import re
import subprocess

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(_HERE))

# `이름.확장자:숫자`
REF = re.compile(r"([A-Za-z0-9_.-]+\.(?:py|sh|html|json|yml|yaml)):(\d{1,5})")

# 이 파일 자신은 예로 옛 형태를 적고 있다. 자기를 세면 영원히 빨갛다.
SELF = os.path.basename(__file__)


def _walk():
    """git 이 없을 때 파일시스템으로 훑는다.

    ⭐⭐ 예전엔 여기서 `pytest.skip` 했다. 그런데 **skip 은 통과가 아닌데 exit=0 이라
       통과처럼 보인다.** 2026-09-12 에 양쪽 세션이 하루에 한 번씩 그 함정을 밟았다 —
       내보낸 트리에서 `1 passed, 4 skipped`, 마운트한 워크트리에서 `7 skipped`.
       둘 다 "검사가 돌았다" 로 읽힐 뻔했다. 검사를 못 하느니 다른 길로 한다.
    ⚠️ **두 경로가 같은 것을 보지는 않는다.** `git ls-files` 는 *추적되는* 파일,
       `os.walk` 는 *디스크에 있는* 파일이다. 이 체크아웃에서는 지금 둘 다 163개로
       같지만(2026-09-12 확인), 추적 안 되는 파일이 생기면 갈린다 — git 이 없는 쪽에서만
       걸리는 거짓 양성이 가능하다. 그래도 이 길을 쓰는 이유: **안 도는 검사보다
       범위가 조금 넓은 검사가 낫다.** git 이 되면 언제나 추적 목록이 먼저다.
    """
    out = []
    skip = {".git", "__pycache__", "node_modules", ".pytest_cache", "var"}
    for root, dirs, files in os.walk(REPO):
        dirs[:] = [d for d in dirs if d not in skip]
        if os.path.join(".claude", "worktrees") in root:
            continue
        for f in files:
            p = os.path.relpath(os.path.join(root, f), REPO)
            out.append(p.replace(os.sep, "/"))
    return out


def _tracked():
    """추적 파일 목록. git 이 안 되면 **건너뛰지 않고** 파일시스템으로 떨어진다."""
    try:
        out = subprocess.run(["git", "ls-files"], cwd=REPO,
                             capture_output=True, text=True, timeout=30)
        if out.returncode == 0:
            got = [f for f in out.stdout.split() if f.strip()]
            if got:
                return got
    except (OSError, subprocess.SubprocessError):
        pass
    return _walk()


def _by_basename(files):
    d = {}
    for f in files:
        d.setdefault(os.path.basename(f), []).append(f)
    return d


def refs_in_text(text, known):
    """(줄번호, 참조) — **이 레포에 딱 하나 있는** 파일을 줄 번호로 가리키는 것들.

    ⭐ 순수 함수로 뺐다. 첫 판은 '주석 줄 판정' 안에 숨어 있어서, 그 판정이 틀렸을 때
       (docstring 본문을 못 봤다) 검사가 **조용히 0건**이 됐다. 이제 이 함수 자체를
       표본으로 시험한다.
    """
    out = []
    for i, line in enumerate(text.split(chr(10)), 1):
        for name, num in REF.findall(line):
            # 없거나 이름이 겹치면 여기서 확인할 수 없다 — 안 센다.
            if len(known.get(name, [])) == 1:
                out.append((i, "%s:%s" % (name, num)))
    return out


def _offenders():
    files = _tracked()
    known = _by_basename(files)
    bad = []
    for f in files:
        if not f.endswith((".py", ".sh")) or os.path.basename(f) == SELF:
            continue
        try:
            text = io.open(os.path.join(REPO, f), encoding="utf-8",
                           errors="replace").read()
        except OSError:
            continue
        for ln, ref in refs_in_text(text, known):
            bad.append((f, ln, ref))
    return bad


# ---- ⭐ 검사가 실패할 수 있는가 ------------------------------------------------------

def test_볼_파일이_있다():
    """⭐⭐ 대상이 0 이면 아래 시험은 구조적으로 실패할 수 없다."""
    n = len([f for f in _tracked() if f.endswith((".py", ".sh"))])
    assert n >= 20, "훑을 py/sh 가 %d 개뿐이다" % n


def test_docstring_본문도_본다():
    """⭐⭐ 이 파일의 첫 판이 **정확히 여기서** 눈이 멀었다.

    `#` 로 시작하는 줄만 봤더니 docstring 본문의 참조를 못 봤고, 검사는 깨끗해서가
    아니라 못 봐서 초록이었다. 그 실패 모양을 표본으로 박아 둔다.
    """
    sample = ('"""왜 필요한가' + chr(10)
              + "    게이트웨이는 `/robotN/pose` 를 구독한다" + chr(10)
              + "    (`gateway_web_server.py:1352`). AMCL 은 다르다." + chr(10)
              + '"""' + chr(10))
    got = refs_in_text(sample, {"gateway_web_server.py": ["x/gateway_web_server.py"]})
    assert got == [(3, "gateway_web_server.py:1352")], got


def test_주석_줄도_본다():
    sample = "# 참조: 01-relay.sh:120 을 보라"
    assert refs_in_text(sample, {"01-relay.sh": ["s/01-relay.sh"]}) == \
        [(1, "01-relay.sh:120")]


def test_레포에_없으면_안_센다():
    """다른 레포 파일은 여기서 확인할 수 없으니 금지해도 검사가 못 한다."""
    # ⭐ 확장자가 붙은 **파일 이름 모양**이어야 한다. 이름을 역할어로 바꾸면
    #    정규식이 아예 안 물어서 단언이 공허해진다(스크럽이 여기를 한 번 깼다).
    foreign = "other_repo_module.py"
    assert refs_in_text("# %s:131" % foreign, {}) == []
    known = _by_basename(_tracked())
    assert foreign not in known, "이 이름이 레포에 생기면 시험 전제가 깨진다"
    # 대조군 — 같은 모양인데 레포에 **있는** 이름이면 반드시 걸려야 한다.
    here = {foreign: ["x/" + foreign]}
    assert refs_in_text("# %s:131" % foreign, here) == [(1, "%s:131" % foreign)]


def test_검색어_형태는_안_걸린다():
    """대체 형태가 걸리면 사람이 되돌린다."""
    assert refs_in_text("# `PoseStamped, '/robot1/pose'` 구독", {}) == []


# ---- 본 검사 -----------------------------------------------------------------

def test_코드가_레포_파일을_줄번호로_안_가리킨다():
    bad = _offenders()
    assert not bad, (
        "코드가 이 레포 파일을 줄 번호로 가리킨다 — 주석 한 줄만 늘어도 밀린다."
        + chr(10) + "대신 찾을 수 있는 것(검색어·식별자)을 적는다."
        + chr(10) + chr(10)
        + chr(10).join("  %s:%d  ->  %s" % r for r in bad))
