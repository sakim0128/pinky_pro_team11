# -*- coding: utf-8 -*-
"""pipefail 인 셸 스크립트에서 `| grep -q` 를 금지한다.

`grep -q` 는 매치하는 순간 끝나며 상류에 SIGPIPE 를 던지고, `pipefail` 이 그 신호를
파이프라인 실패로 올린다. **조건이 참일 때 오히려 실패한다.**
다른 세션이 `simulation/docker/entrypoint-localization.sh` 에서 실측해 알려줬고
(`326df34`), 이 레포의 나머지를 2026-09-12 에 재서 고쳤다.

실측 (같은 조건 25회씩, 매치가 맨 앞, `set -euo pipefail`):

    상류 288 KB  파이프                 실패 25/25   <- 파이프 용량(64K) 초과로 막힌다
    상류  48 KB  파이프                 실패  0/25   <- 한 번에 써서 안 막힌다
    느리거나 여러 프로세스인 상류         실패 21~25/25 (크기와 무관)
    변수 + `printf | grep -q`            1.9MB 에서 실패 25/25   <- 변수도 안 낫다
    변수 + `grep -q ... <<< "$var"`      1.9MB 에서 실패  0/25   <- 이 형태를 쓴다

⭐ 방아쇠가 둘이라 "출력이 작으니 괜찮다" 는 판단이 반쪽이다:
   ① 출력이 파이프 용량을 넘는다  ② 상류가 일찍 flush 하고 **계속 산다**(느린 명령·다단 파이프)
   ②는 크기와 무관하다.

⭐ pipefail 이 **없는** 스크립트는 대상이 아니다 — 거기선 상류 SIGPIPE 가 파이프라인
   상태에 안 올라온다. 범위를 넓히면 근거 없는 금지가 된다.
"""
import io
import os
import re
import subprocess

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(_HERE))

PIPE_GREP_Q = re.compile(r"\|\s*grep\s+-[a-zA-Z]*q")


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


def _shell_scripts():
    """추적 파일 목록. git 이 안 되면 **건너뛰지 않고** 파일시스템으로 떨어진다."""
    try:
        out = subprocess.run(["git", "ls-files", "*.sh"], cwd=REPO,
                             capture_output=True, text=True, timeout=30)
        if out.returncode == 0:
            got = [f for f in out.stdout.split() if f.strip()]
            if got:
                return got
    except (OSError, subprocess.SubprocessError):
        pass
    return [f for f in _walk() if f.endswith('.sh')]


def _read(rel):
    p = os.path.join(REPO, rel)
    if not os.path.exists(p):
        return None
    with io.open(p, encoding="utf-8", errors="replace") as fh:
        return fh.read()


def _pipefail_scripts():
    """`set ... pipefail` 을 켠 스크립트들."""
    out = []
    for rel in _shell_scripts():
        t = _read(rel)
        if t is None:
            continue
        if any(l.startswith("set -") and "pipefail" in l
               for l in t.split(chr(10))):
            out.append((rel, t))
    return out


def _offending(text):
    """주석이 아닌 줄의 파이프-grep-q. ⭐ 규칙을 적어 둔 주석 자체를 세면 안 된다."""
    return [(i + 1, l.strip())
            for i, l in enumerate(text.split(chr(10)))
            if PIPE_GREP_Q.search(l) and not l.lstrip().startswith("#")]


# ---- ⭐ 검사가 실패할 수 있는가부터 ---------------------------------------------------

def test_대상이_실제로_있다():
    """⭐⭐ 대상이 0 개면 아래 시험은 **구조적으로 실패할 수 없다** — 검사가 아니다.

    이 프로젝트에서 하루에 네 번 같은 함정을 밟았다(방금 쓴 파일을 자기와 비교 등).
    그래서 '무엇을 검사하고 있는가'를 먼저 단언한다.
    """
    got = _pipefail_scripts()
    assert got, "pipefail 인 스크립트를 하나도 못 찾았다 — 검사 대상이 없다"
    assert len(got) >= 4, "pipefail 스크립트가 %d 개뿐이다: %s" % (
        len(got), [r for r, _ in got])


def test_정규식이_옛_형태를_실제로_잡는다():
    """⭐ 정규식이 아무것도 안 잡으면 위 시험도 항상 통과한다."""
    for sample in ('if ip route | grep -qE "^default"; then',
                   'echo "$X" | grep -q foo && a || b',
                   'cmd 2>/dev/null |grep -qi bar'):
        assert PIPE_GREP_Q.search(sample), sample
    for ok_sample in ('grep -q foo <<< "$X"',
                      'cmd | grep -c foo',
                      'cmd | grep -oE "x"'):
        assert not PIPE_GREP_Q.search(ok_sample), ok_sample


# ---- 본 검사 -----------------------------------------------------------------

def test_pipefail_스크립트에_파이프_grep_q_가_없다():
    bad = []
    for rel, text in _pipefail_scripts():
        for ln, line in _offending(text):
            bad.append("%s:%d  %s" % (rel, ln, line[:90]))
    assert not bad, (
        "pipefail 인데 grep -q 로 파이프한다 — **조건이 참일 때 실패한다**."
        + chr(10) + "대신: out=\"$(cmd || true)\"; grep -q ... <<< \"$out\""
        + chr(10) + chr(10).join(bad))


def test_대체_형태를_실제로_쓰고_있다():
    """고쳤다면 herestring 이 보여야 한다. 안 보이면 그냥 지운 것이다."""
    hits = 0
    for _rel, text in _pipefail_scripts():
        hits += len(re.findall(r"grep\s+-[a-zA-Z]*q\s+.*<<<", text))
    assert hits >= 2, "herestring 형태가 %d 곳뿐이다 — 정말 바꿨나" % hits   # 팀11 레포: 중계 스크립트만(원 저장소 8)


def test_왜_그런지_파일에_적혀_있다():
    """⭐ 다음 사람이 되돌리지 않게 근거를 파일 안에 둔다 — 실측까지."""
    documented = [rel for rel, text in _pipefail_scripts()
                  if "SIGPIPE" in text and "pipefail" in text]
    assert len(documented) >= 1, (   # 팀11 레포: 중계 스크립트만(원 저장소 4)
        "규칙을 적어 둔 pipefail 스크립트가 %d 개뿐이다: %s"
        % (len(documented), documented))
