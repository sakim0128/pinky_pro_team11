# -*- coding: utf-8 -*-
"""MCV-2A-UI — 캘리브레이션 HTTP 표면이 지켜야 할 **안전 성질**.

`gateway_web_server.py` 는 module-level 에서 rclpy·cv_bridge 를 import 하므로
ROS 없이는 못 불러온다(conftest 참조). 그래서 여기서는 **소스를 AST 로 읽는다.**

⭐ 동작 테스트로는 이 성질을 못 잡는다. 게이트를 지워도 기능은 멀쩡히 돈다 —
   원격에서도 된다는 것만 달라진다. 그래서 구조를 직접 본다.

⭐⭐ 2026-09-10 에 이 방식이 실제로 버그를 잡았다(멀티뷰 배지 선택자/속성 불일치).
"""
import ast
import os

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
SERVER = os.path.join(os.path.dirname(_HERE), "gateway_web", "gateway_web_server.py")

# 로컬 게이트가 **없어도 되는** POST 경로. 늘리려면 근거를 여기 적는다.
UNGATED_KNOWN = {
    # R-4 미결: 정지 계열은 게이트가 없다. 원격 무인증 정지가 가능하다는 뜻이다.
    # "위험할 때 누구나 세울 수 있어야 한다" 와 "아무나 세우면 안 된다" 가 부딪힌다.
    # 사용자 판단 대기 중이라 여기 등재해 둔다 - **모르고 지나치지 않기 위해서다.**
    '/api/robot1/stop', '/api/stop', '/api/nav/stop',
    # 프레임 수신·주소 변경. 앱이 무인증으로 밀어넣는 경로라 원래 열려 있다(UNTRUSTED).
    '/api/camera/url', '/api/camera/upload', '/upload', '/camera', '/image',
    '/video', '/frame', '/shot.jpg',
    # 2026-09-19 신설. 연산 노드(태블릿)가 **원격에서** 좌표를 밀어넣는 경로라
    # `LOCAL_CONTROL_IPS` 로 막으면 그 자리에서 죽는다. 대가는 무인증 주입이다.
    # 🔴 **회수 조건**: `/robotN/vision_pose` 의 소비자가 생기는 순간(지금 0) 인증을
    #    붙이고 여기서 지운다 — 그때부터는 주입이 관제 판단을 바꾼다.
    '/api/vision/pose',
    # 2026-09-22 신설 (Track R): 태블릿에서 전송하는 비전 구역 진입/도착 이벤트 수신구
    '/api/vision/zone_event',
    # 2026-09-22 신설 (Track R: R-D1): 태블릿 canonical PoseFix 수신구 (원격 비전 연산 노드)
    '/api/vision/pose_fix',
}


def _source():
    with open(SERVER, encoding="utf-8") as fh:
        return fh.read()


def _tree():
    return ast.parse(_source())


def _func(name):
    for node in ast.walk(_tree()):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    pytest.fail("%s 를 못 찾았다" % name)


def _path_literals(test_node):
    """`parsed.path == 'x'` / `parsed.path in ('x','y')` 에서 문자열들을 뽑는다."""
    out = []
    for cmp_node in ast.walk(test_node):
        if not isinstance(cmp_node, ast.Compare):
            continue
        left = cmp_node.left
        if not (isinstance(left, ast.Attribute) and left.attr == "path"):
            continue
        for comparator in cmp_node.comparators:
            for lit in ast.walk(comparator):
                if isinstance(lit, ast.Constant) and isinstance(lit.value, str):
                    out.append(lit.value)
    return out


def _post_branches():
    """do_POST 의 if/elif 사슬 → [(경로들, 그 가지의 노드)]."""
    node = _func("do_POST")
    out = []
    stack = [s for s in node.body if isinstance(s, ast.If)]
    while stack:
        branch = stack.pop(0)
        paths = _path_literals(branch.test)
        if paths:
            out.append((paths, branch))
        for s in branch.orelse:
            if isinstance(s, ast.If):
                stack.append(s)
    return out


def _names_in(node):
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


def _attrs_in(node):
    return {n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute)}


# ---- 게이트 -------------------------------------------------------------------

def test_로컬_게이트가_한_곳에_정의돼_있다():
    """세 군데에 같은 튜플을 복붙해 두면 한 곳만 고치고 끝난다."""
    src = _source()
    assert "LOCAL_CONTROL_IPS = (" in src
    # 리터럴 튜플이 다시 흩어지지 않았는지
    assert src.count("'127.0.0.1', '::1', 'localhost'") == 1




def test_게이트_없는_POST_경로가_새로_생기지_않았다():
    """⭐ 이 시험의 값은 **새 경로**에 있다. 알려진 예외는 위에 근거와 함께 등재한다."""
    ungated = set()
    for paths, branch in _post_branches():
        if "LOCAL_CONTROL_IPS" in _names_in(branch):
            continue
        # 2026-09-28 제어권 정책: 움직이는 경로(플릿 start/resume/assign · 목표 · 미션 · 재개 · 좌표 전환)는
        # `self._deny_if_cannot_move(...)` 가, /api/control/acquire·release 는 `CONTROL_POLICY` 가 문이다
        # (허용 목록 주소·로컬만 — 기본 목록은 비어 있어 예전 LOCAL_CONTROL_IPS 와 같다). test_control_policy.py ·
        # test_control_0928_control_policy_wiring.py 가 그 문을 잰다. 정지 계열이 같은 가지 안에서 문 뒤에 있지 않은 것은
        # 예전(LOCAL_CONTROL_IPS 시절)과 같다 — R-4 는 아래 시험이 따로 기록한다.
        if "CONTROL_POLICY" in _names_in(branch) or "_deny_if_cannot_move" in _attrs_in(branch):
            continue
        ungated.update(paths)
    new = ungated - UNGATED_KNOWN
    assert not new, "게이트 없는 새 POST 경로: %s" % sorted(new)


def test_정지_계열이_아직_무인증이라는_사실이_기록돼_있다():
    """R-4. 해소되면 이 시험이 실패하고, 그때 UNGATED_KNOWN 에서 지운다."""
    assert '/api/stop' in UNGATED_KNOWN


# ---- 프레임 크기는 클라이언트가 말하는 값이 아니다 -----------------------------------







# ---- 미리보기 -----------------------------------------------------------------







# ---- 읽기 표면 ----------------------------------------------------------------



