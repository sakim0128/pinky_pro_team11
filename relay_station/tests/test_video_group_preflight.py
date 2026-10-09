# -*- coding: utf-8 -*-
"""R-4 — 설치 스크립트가 **카메라 권한을 미리 말한다.**

2026-09-14 실측: 중계 노트북의 USB 웹캠·내장 캠이 둘 다 안 열렸다. 장치를 잡은
프로세스는 없었고 cv2.VideoCapture 가 네 방식 모두 isOpened=False 였다.
원인은 권한 — `/dev/video*` 는 `root:video 0660 + ACL` 이고 logind 는 **활성 seat
세션**에만 uaccess ACL 을 준다. Chrome Remote Desktop 세션은 seat 가 없어 못 받는다.

⭐⭐ 이 시험의 요점은 **중간 상태**다. `usermod -aG video` 만 하고 재부팅을 안 하면
   계정 정보에는 video 가 보이는데 **이미 떠 있는 프로세스는 옛 그룹**이라
   카메라가 여전히 안 열린다. 한 가지만 보는 검사는 그 상태를 "됐다"로 읽고,
   사람은 "그룹 넣었는데 왜 안 되지"로 시간을 태운다.
   (2026-09-14 20:5x 중계 노트북이 정확히 이 상태였다 — 이 분기가 실물에서 걸렸다.)

⚠️ 스크립트는 sudo 를 치지 않는다 — 장비에서 직접 고치지 않는 것이 이 레포의 규칙이다.
   말만 한다. 그 성질도 여기서 고정한다.
"""
import os
import re
import subprocess
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_RELAY = os.path.dirname(_HERE)
SCRIPT = os.path.join(_RELAY, "scripts", "install_gateway_unit.sh")


def _function_text():
    """스크립트에서 preflight 함수 본문만 떼어낸다.

    ⚠️ 스크립트 전체를 돌리면 유닛을 진짜로 설치한다. 함수만 떼어 돌린다.
    """
    if not os.path.exists(SCRIPT):
        pytest.skip("install_gateway_unit.sh 없음: %s" % SCRIPT)
    with open(SCRIPT, encoding="utf-8") as fh:
        lines = fh.read().splitlines()
    try:
        start = next(i for i, l in enumerate(lines)
                     if l.startswith("preflight_video_group() {"))
    except StopIteration:
        pytest.fail("preflight_video_group 함수가 스크립트에 없다")
    end = next(i for i in range(start + 1, len(lines)) if lines[i] == "}")
    return "\n".join(lines[start:end + 1])


def _run(db_groups, proc_groups):
    """가짜 `id` 를 PATH 앞에 두고 함수를 돌린다.

    `id -nG <user>` = 계정 정보, `id -nG` = 지금 프로세스 그룹. 둘을 따로 준다.
    """
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        stub = os.path.join(d, "id")
        with open(stub, "w", encoding="utf-8") as fh:
            fh.write("#!/usr/bin/env bash\n"
                     # `id -nG "$USER"` 는 인자 2개(-nG, 사용자), `id -nG` 는 1개다.
                     'if [ "$#" -ge 2 ]; then echo "%s"; else echo "%s"; fi\n'
                     % (db_groups, proc_groups))
        os.chmod(stub, 0o755)
        body = _function_text() + "\npreflight_video_group\n"
        env = dict(os.environ, PATH=d + os.pathsep + os.environ["PATH"], USER="testuser")
        r = subprocess.run(["bash", "-c", body], capture_output=True, text=True, env=env)
        return r


def test_그룹이_없으면_빨간_안내를_낸다():
    """🔴 안내가 사라지면 빨개진다 — 설치한 사람이 카메라가 왜 안 열리는지 모른다."""
    r = _run(db_groups="testuser sudo", proc_groups="testuser sudo")
    assert r.returncode == 0, "안내는 설치를 막지 않는다"
    assert "🔴" in r.stdout
    assert "usermod -aG video testuser" in r.stdout


def test_등록됐지만_세션에_안_붙었으면_다르게_말한다():
    """⭐ 이 분기가 없으면 빨개진다. 한 가지만 보면 이 상태를 '됐다'로 읽는다."""
    r = _run(db_groups="testuser sudo video", proc_groups="testuser sudo")
    assert r.returncode == 0
    assert "🟡" in r.stdout
    assert "재부팅" in r.stdout
    assert "🔴" not in r.stdout, "이미 등록했는데 usermod 를 다시 시키면 안 된다"


def test_둘_다_붙었으면_조용하다():
    """멀쩡한 기계에서 잔소리가 나오면 빨개진다 — 잔소리는 진짜 경고를 묻는다."""
    r = _run(db_groups="testuser sudo video", proc_groups="testuser sudo video")
    assert r.returncode == 0
    assert r.stdout.strip() == "", "출력: %r" % r.stdout


def test_스크립트가_sudo_를_치지_않는다():
    """스크립트가 직접 sudo 를 부르면 빨개진다 — 장비에서 직접 고치지 않는다(규칙 4)."""
    body = _function_text()
    # 안내 문구 안의 'sudo usermod' 는 **말**이지 실행이 아니다. 실행 형태만 본다.
    stripped = re.sub(r'"[^"]*"', '""', body)          # 따옴표 안(= echo 문구)을 지운다
    assert not re.search(r'(^|[;&|(]\s*)sudo\s', stripped, re.M), \
        "스크립트가 sudo 를 실행한다"
