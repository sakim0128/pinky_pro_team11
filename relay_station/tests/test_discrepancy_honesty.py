# -*- coding: utf-8 -*-
"""정합 오차는 **안 왔으면 숫자를 만들지 않는다**.

QA Q-1 (2026-09-12): 로봇 발행자 0 · pose 수신 0 인데 화면이
`0.0 cm` + 초록 `🟢 실측 일치 극우수` 를 띄웠다. 거짓말이 세 겹이었다:

    1. 백엔드가 기본값 (0,0,0) 둘을 빼서 0.0 을 만들고
    2. 화면이 undefined 를 0.0 으로 떨어뜨리고
    3. 등급이 0.0 을 최고로 쳤다

⭐ 방향이 최악이다. `CPU 0%` 는 "부하 없음" 으로 읽혀 무해했지만
   `0.0cm 극우수` 는 **"완벽히 일치한다"** 로 읽힌다 — 못 쟀다는 사실이
   가장 좋은 소식으로 둔갑한다.

이 시험은 **함수를 떼어내 스텁에 붙여 실제로 돌린다.** 모듈을 import 하지
않는 이유는 gateway_web_server 가 최상단에서 cv2·rclpy 를 물기 때문이다 —
그걸 피하려고 소스 검사만 하면 "문자열이 있다" 만 보게 되고, 그건 동작이 아니다.
"""
import ast
import math
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(_HERE))
SERVER = os.path.join(REPO, "relay_station", "gateway_web", "gateway_web_server.py")
PAGE = os.path.join(REPO, "relay_station", "gateway_web", "static", "index.html")

POSE = {"x": 1.0, "y": 2.0, "yaw": 0.5}


def _load_get_discrepancy():
    """소스에서 함수 하나만 떼어내 실행 가능한 형태로 만든다."""
    src = open(SERVER, encoding="utf-8").read()
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "get_discrepancy":
            seg = ast.get_source_segment(src, node)
            assert seg, "get_discrepancy 소스를 못 떼어냈다"
            import textwrap
            ns = {"math": math}
            exec(textwrap.dedent(seg), ns)
            return ns["get_discrepancy"]
    pytest.fail("get_discrepancy 를 못 찾았다 — 이름이 바뀌었나")


class _Stub(object):
    def __init__(self, r1, gz):
        self.latest_r1_pose = r1
        self.latest_gz_pose = gz


def _page_without_comments():
    """주석을 걷고 본다.

    ⭐ 첫 판은 주석을 안 걷어서 **내가 쓴 설명 주석**이 걸렸다 —
       수정 이유를 적으며 배지 문구를 인용했더니 그게 '초록 배지' 로 잡혔다.
       규칙을 설명하는 문장이 그 규칙을 어기는 자리가 된다.
       위치를 비교하는 검사는 **판정 대상만 남기고** 봐야 한다.
    """
    page = open(PAGE, encoding="utf-8").read()
    page = re.sub(r"<!--.*?-->", "", page, flags=re.S)   # HTML 주석
    page = re.sub(r"^\s*//.*$", "", page, flags=re.M)    # JS 줄 주석
    return page


def test_기본값이_원점이_아니라_None_이다():
    """(0,0,0) 으로 두면 '안 왔다' 와 '원점에 있다' 가 구분되지 않는다."""
    src = open(SERVER, encoding="utf-8").read()
    assert "self.latest_gz_pose = None" in src, "gz 기본값이 None 이 아니다"
    assert "self.latest_r1_pose = None" in src, "r1 기본값이 None 이 아니다"


def test_둘_다_안_왔으면_숫자를_안_만든다():
    fn = _load_get_discrepancy()
    out = fn(_Stub(None, None))
    assert out.get("state") == "UNMEASURED", out
    assert "dist_err_cm" not in out, "안 왔는데 오차 숫자를 만들었다: %r" % out
    assert "ROBOT_POSE" in out.get("why", "") and "SIM_POSE" in out.get("why", "")


@pytest.mark.parametrize("r1,gz,missing", [
    (None, POSE, "ROBOT_POSE"),
    (POSE, None, "SIM_POSE"),
])
def test_한쪽만_없어도_숫자를_안_만든다(r1, gz, missing):
    fn = _load_get_discrepancy()
    out = fn(_Stub(r1, gz))
    assert out.get("state") == "UNMEASURED", out
    assert "dist_err_cm" not in out
    assert missing in out.get("why", "")


def test_둘_다_오면_잰다():
    """가드가 너무 넓어 **정상 경로까지 막으면** 그것도 결함이다."""
    fn = _load_get_discrepancy()
    out = fn(_Stub({"x": 1.0, "y": 0.0, "yaw": 0.0}, {"x": 0.0, "y": 0.0, "yaw": 0.0}))
    assert out.get("state") == "MEASURED", out
    assert out["dist_err_cm"] == pytest.approx(100.0), out


def test_같은_자리면_0_이_나온다_그건_진짜_0_이다():
    """0.0 자체를 막는 게 아니다 — **안 재고 만든 0** 을 막는 것이다."""
    fn = _load_get_discrepancy()
    out = fn(_Stub(dict(POSE), dict(POSE)))
    assert out["state"] == "MEASURED"
    assert out["dist_err_cm"] == 0.0


def test_화면이_state_를_먼저_본다():
    """숫자를 쓰기 **전에** 게이트가 있어야 한다. 뒤에 있으면 이미 그린 뒤다."""
    page = _page_without_comments()
    gate = page.find("disc.state !== 'MEASURED'")
    assert gate != -1, "화면에 state 게이트가 없다"
    use = page.find("disc.dist_err_cm")
    assert use != -1, "오차를 쓰는 자리가 없다 — 검사 대상이 사라졌나"
    assert gate < use, "게이트가 숫자 사용보다 뒤에 있다"


def test_초록_배지는_잰_경우에만_도달한다():
    """`극우수` 배지가 게이트보다 앞에 있으면 안 잰 값에도 붙는다."""
    page = _page_without_comments()
    gate = page.find("disc.state !== 'MEASURED'")
    green = page.find("실측 일치 극우수")
    assert green != -1, "초록 배지 문구가 사라졌다 — 검사 대상 확인 필요"
    assert gate < green, "초록 배지가 게이트보다 앞에 있다"


def test_정착시각이_1970_으로_안_뜬다():
    """`settledAt || 0` 은 Date(0) 이라 '정착한 적 없음' 이 '1970년에 정착함' 이 된다."""
    page = open(PAGE, encoding="utf-8").read()
    assert "settledAt || 0" not in page, "settledAt 이 아직 0 으로 떨어진다"
    assert "r.settledAt ?" in page, "settledAt 존재 검사가 없다"
