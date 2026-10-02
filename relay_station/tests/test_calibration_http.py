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
    '/api/pinky1/stop', '/api/pinky2/stop',
    # 프레임 수신·주소 변경. 앱이 무인증으로 밀어넣는 경로라 원래 열려 있다(UNTRUSTED).
    '/api/camera/url', '/api/camera/upload', '/upload', '/camera', '/image',
    '/video', '/frame', '/shot.jpg',
    # 2026-09-19 신설. 연산 노드(태블릿)가 **원격에서** 좌표를 밀어넣는 경로라
    # `LOCAL_CONTROL_IPS` 로 막으면 그 자리에서 죽는다. 대가는 무인증 주입이다.
    # 🔴 **회수 조건**: `/pinkyN/vision_pose` 의 소비자가 생기는 순간(지금 0) 인증을
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


def _branch_nodes(branch):
    """If 가지 하나의 **몸통**만 걷는다. `branch.orelse`(= 뒤따르는 elif 사슬 전부)를 걷으면 뒤 가지의 게이트가 앞 가지의 것으로 센다."""
    for stmt in branch.body:
        yield from ast.walk(stmt)


def _branch_names(branch):
    return {n.id for n in _branch_nodes(branch) if isinstance(n, ast.Name)}


def _branch_attrs(branch):
    return {n.attr for n in _branch_nodes(branch) if isinstance(n, ast.Attribute)}


# ---- 게이트 -------------------------------------------------------------------

def test_로컬_게이트가_한_곳에_정의돼_있다():
    """세 군데에 같은 튜플을 복붙해 두면 한 곳만 고치고 끝난다."""
    src = _source()
    assert "LOCAL_CONTROL_IPS = (" in src
    # 리터럴 튜플이 다시 흩어지지 않았는지
    assert src.count("'127.0.0.1', '::1', 'localhost'") == 1


def test_캘리브레이션_POST_는_주행_명령과_같은_게이트를_쓴다():
    for paths, branch in _post_branches():
        if not any(p.startswith('/api/calibration') for p in paths):
            continue
        assert "LOCAL_CONTROL_IPS" in _branch_names(branch), \
            "%s 에 로컬 게이트가 없다 - 원격에서 좌표계를 바꿀 수 있다" % paths
        return
    pytest.fail("캘리브레이션 POST 가지를 못 찾았다")


def test_게이트_없는_POST_경로가_새로_생기지_않았다():
    """⭐ 이 시험의 값은 **새 경로**에 있다. 알려진 예외는 위에 근거와 함께 등재한다."""
    ungated = set()
    for paths, branch in _post_branches():
        if "LOCAL_CONTROL_IPS" in _branch_names(branch):
            continue
        # 2026-09-28 제어권 정책: 움직이는 경로(플릿 start/resume/assign · 목표 · 미션 · 재개 · 좌표 전환)는
        # `self._deny_if_cannot_move(...)` 가, /api/control/acquire·release 는 `CONTROL_POLICY` 가 문이다
        # (허용 목록 주소·로컬만 — 기본 목록은 비어 있어 예전 LOCAL_CONTROL_IPS 와 같다). test_control_policy.py ·
        # test_control_0928_control_policy_wiring.py 가 그 문을 잰다. 정지 계열이 같은 가지 안에서 문 뒤에 있지 않은 것은
        # 예전(LOCAL_CONTROL_IPS 시절)과 같다 — R-4 는 아래 시험이 따로 기록한다.
        if "CONTROL_POLICY" in _branch_names(branch) or "_deny_if_cannot_move" in _branch_attrs(branch):
            continue
        ungated.update(paths)
    new = ungated - UNGATED_KNOWN
    assert not new, "게이트 없는 새 POST 경로: %s" % sorted(new)


def test_정지_계열이_아직_무인증이라는_사실이_기록돼_있다():
    """R-4. 해소되면 이 시험이 실패하고, 그때 UNGATED_KNOWN 에서 지운다."""
    assert '/api/pinky1/stop' in UNGATED_KNOWN


# ---- 프레임 크기는 클라이언트가 말하는 값이 아니다 -----------------------------------

def test_영수증의_프레임_크기를_요청에서_받지_않는다():
    """⭐ 클라이언트가 보낸 크기를 믿으면 영수증이 거짓이 된다.

    /status.resolution 이 설정값이라 거짓이었던 것과 같은 실수다 - 이번엔 우리가 안 한다.
    """
    src = _source()
    assert "req_json.get('frameSize')" not in src
    assert "req_json.get('frame_size')" not in src
    assert "_calib_frame_facts" in src


def test_프레임_크기를_곁표에서_읽는다():
    fn = _func("_calib_frame_facts")
    body = ast.dump(fn)
    assert "clock_info" in body, "크기를 곁표(clock_info)가 아닌 데서 가져오고 있다"
    assert "frameWidth" in body and "frameHeight" in body


def test_set_points_에_넘기는_크기가_실측에서_온다():
    """⭐ 문자열 검사만으로는 `req_json.get('size')` 같은 변형을 놓친다.

    그래서 **자료 흐름**을 본다: set_points 의 3번째 인자가 _calib_frame_facts 가
    묶어 준 이름인가. 다른 데서 온 이름이면 실패한다.
    """
    post = _func("do_POST")
    measured = set()
    for node in ast.walk(post):
        if not isinstance(node, ast.Assign):
            continue
        call = node.value
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == "_calib_frame_facts"):
            for tgt in node.targets:
                for n in ast.walk(tgt):
                    if isinstance(n, ast.Name):
                        measured.add(n.id)
    assert measured, "do_POST 가 _calib_frame_facts 로 실측을 안 가져온다"
    found = False
    for node in ast.walk(post):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "set_points"):
            found = True
            assert len(node.args) >= 3, "set_points 인자가 모자란다"
            third = node.args[2]
            assert isinstance(third, ast.Name) and third.id in measured,                 "프레임 크기가 실측이 아닌 데서 온다: %s" % ast.dump(third)
    assert found, "set_points 호출을 못 찾았다"


# ---- 미리보기 -----------------------------------------------------------------

def test_미리보기가_y축을_뒤집는다():
    """화면은 위가 0, 아레나는 아래가 0. 안 뒤집으면 거꾸로인 그림을 정합이라 부른다."""
    src = _source()
    assert "out_h - targets[c][1] * scale" in src


def test_미리보기가_정착_전에도_되지만_행렬을_내주지는_않는다():
    """찍으면서 확인하는 화면이다. 그렇다고 그 값이 관측 좌표가 되지는 않는다."""
    fn = _func("_calib_preview")
    names = _names_in(fn)
    assert "STATE_SETTLED" not in names, "미리보기가 정착을 요구하면 찍는 중에 못 본다"
    src = _source()
    assert "def _calib_preview" in src
    # 관측용 행렬은 여전히 store 의 fail-closed 경로로만 나간다
    assert "homography_for" not in src.split("def _calib_preview", 1)[1][:2000]


def test_아레나가_없으면_미리보기가_거절한다():
    fn = _func("_calib_preview")
    assert "'NO_ARENA'" in ast.dump(fn)


# ---- 읽기 표면 ----------------------------------------------------------------

@pytest.mark.parametrize("path", [
    '/api/calibration', '/api/calibration/receipts',
    '/api/calibration/still', '/api/calibration/preview',
])
def test_읽기_엔드포인트가_있다(path):
    assert "parsed.path == '%s'" % path in _source()


def test_상태_API_가_문구를_내지_않는다():
    """R-6 규약: 서버는 코드와 숫자만 낸다. 말은 UI 가 소유한다."""
    src = _source()
    block = src.split("if parsed.path == '/api/calibration':", 1)[1][:900]
    for word in ("정합", "완료", "실패했습니다", "하세요"):
        assert word not in block, "상태 API 가 문구(%s)를 내고 있다" % word


def test_do_POST_가지를_충분히_읽어낸다():
    """⭐ 양성 대조군 — 가지를 못 읽으면 위 시험은 구조적으로 통과한다(기대 경로가 읽혔는지 센다)."""
    seen = {p for paths, _ in _post_branches() for p in paths}
    assert {'/api/observe', '/api/fault/toggle_pose_fix', '/api/calibration/masks',
            '/api/vision/world', '/api/vision/pose_fix'} <= seen, sorted(seen)
    assert len(_post_branches()) >= 12, len(_post_branches())      # 삭제 전 13, /api/vision/pose 삭제 후 12


def test_고장주입_토글은_로컬_게이트_뒤에_있다():
    for paths, branch in _post_branches():
        if '/api/fault/toggle_pose_fix' in paths:
            assert "LOCAL_CONTROL_IPS" in _branch_names(branch), "원격에서 고장 주입이 가능하다"
            return
    pytest.fail("toggle_pose_fix 가지를 못 찾았다")
