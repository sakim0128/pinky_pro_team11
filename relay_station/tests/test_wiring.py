# -*- coding: utf-8 -*-
"""배선 — 만든 모듈이 실제로 **불리는가**.

⭐ 안 불리는 모듈은 기능이 아니다. `drift.py` 와 `vision_world.py` 는 시험이 전부
   초록이어도, 게이트웨이가 안 부르면 운영에서 아무 일도 안 일어난다.
   그 사실은 모듈 시험으로는 절대 드러나지 않는다.

게이트웨이 본체는 rclpy 때문에 유닛에서 import 못 한다(conftest 참조). 그래서 AST 로 본다.
⭐ 다만 도커 복제본에는 rclpy 가 있어 **진짜 import** 가 되고, 게이트가 pyflakes 로
   정의 안 된 이름까지 본다 — 그쪽이 더 센 검사다.
"""
import ast
import io
import os

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_RELAY = os.path.dirname(_HERE)
SERVER = os.path.join(_RELAY, "gateway_web", "gateway_web_server.py")


def _src():
    with io.open(SERVER, encoding="utf-8") as fh:
        return fh.read()


def _func(name):
    for node in ast.walk(ast.parse(_src())):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    pytest.fail("%s 를 못 찾았다" % name)


def _names(node):
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


def _attrs(node):
    return {n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute)}


# ---- 드리프트가 불리는가 -----------------------------------------------------------

def test_모듈을_들여온다():
    src = _src()
    assert "import drift" in src
    assert "import vision_world" in src


def test_정착할_때_기준을_잡는다():
    """⭐ 기준선은 정착 **직후**여야 한다. 나중에 잡으면 움직인 상태가 기준이 된다."""
    post = _func("do_POST")
    assert "_drift_anchor" in _names(post), "정착 경로가 기준을 안 잡는다"
    anchor = _func("_drift_anchor")
    assert "save_reference" in _attrs(anchor)
    assert "take_patches" in _attrs(anchor)


def test_상태에_드리프트가_실린다():
    st = _func("_calib_state")
    assert "_drift_state" in _names(st)


def test_움직였으면_정착을_내린다():
    """⭐ 카메라가 움직였으면 그 행렬은 틀린 것이다. 무효화 3종이 못 잡는 자리다."""
    src = _src()
    i = src.index("def _calib_state")
    body = src[i:i + 2200]
    assert "CAMERA_MOVED" in body
    assert "'DRIFT'" in body or '"DRIFT"' in body


def test_정착_해제하면_기준도_지운다():
    """남겨 두면 다음 정착 때 옛 기준을 쓴다."""
    post = _func("do_POST")
    assert "clear_reference" in _attrs(post)


def test_드리프트_판정을_throttle_한다():
    """⭐ 조각 정합 4번이 매 폴링마다 돌면 서빙이 느려지고, 느린 화면은 사람이 안 본다."""
    src = _src()
    assert "DRIFT_MIN_INTERVAL_S" in src
    ds = _func("_drift_state")
    assert "GLOBAL_DRIFT_CACHE" in _names(ds)


def test_정착_상태일_때만_본다():
    """아직 안 찍었으면 움직였는지 물을 것도 없다."""
    src = _src()
    i = src.index("def _calib_state")
    body = src[i:i + 2200]
    assert "STATE_SETTLED" in body


# ---- 영상→월드가 불리는가 ----------------------------------------------------------

def test_영상월드_엔드포인트가_있다():
    assert "'/api/vision/world'" in _src()


def test_영상월드도_같은_로컬_게이트다():
    """관측 좌표계를 쓰는 산출물이다. 원격에서 만들 수 있으면 안 된다."""
    src = _src()
    i = src.index("parsed.path == '/api/vision/world'")
    assert "LOCAL_CONTROL_IPS" in src[i:i + 500]


def test_영상월드가_검열_상자를_넘긴다():
    """가려진 자리를 바닥으로 읽으면 안 된다."""
    vb = _func("_vision_build")
    assert "_calib_censor" in _names(vb)
    assert "censor_box" in ast.dump(vb)


def test_산출물을_파일로_남긴다():
    """sdf·yaml·pgm·출처가 다 남아야 나중에 대조한다."""
    vb = ast.dump(_func("_vision_build"))
    for ext in (".sdf", ".yaml", ".pgm", ".json"):
        assert ext in vb, "%s 를 안 남긴다" % ext


def test_라이다_산출물을_덮어쓰지_않는다():
    """⭐ `map2@`(라이다)와 나란히 남아야 서로를 대조할 수 있다(MCVA-73)."""
    vb = ast.dump(_func("_vision_build"))
    assert "map2" not in vb
    src = _src()
    i = src.index("def _vision_build")
    body = src[i:i + 1800]
    assert "vision" in body


# ---- 시청 경로는 여전히 안 건드린다 --------------------------------------------------

def _branch_for(func_name, path):
    """그 경로를 다루는 if/elif **가지 하나**를 준다.

    ⭐ 문자열 창(`src[i-2000:i+6000]`)으로 보면 **옆 핸들러를 삼킨다** —
       실제로 /api/vision/world 가 창에 들어와 거짓 실패가 났다(2026-09-11).
       좁은 문자열 검사로 구조를 판정하면 틀린다는 것을 또 한 번 겪었다.
    """
    fn = _func(func_name)
    stack = [s for s in fn.body if isinstance(s, ast.If)]
    while stack:
        br = stack.pop(0)
        lits = []
        for cmp_node in ast.walk(br.test):
            if not isinstance(cmp_node, ast.Compare):
                continue
            left = cmp_node.left
            if not (isinstance(left, ast.Attribute) and left.attr == "path"):
                continue
            for comp in cmp_node.comparators:
                for lit in ast.walk(comp):
                    if isinstance(lit, ast.Constant) and isinstance(lit.value, str):
                        lits.append(lit.value)
        if path in lits:
            return br
        for s in br.orelse:
            if isinstance(s, ast.If):
                stack.append(s)
    pytest.fail('%s 의 %s 가지를 못 찾았다' % (func_name, path))


def test_시청_경로에_드리프트도_영상월드도_안_들어간다():
    """🔴 시청은 계속한다(사용자 결정). 무거운 판정이 그 경로로 새면 화면이 느려진다."""
    br = _branch_for('do_GET', '/video_feed')
    names = _names(br) | _attrs(br)
    for bad in ('_drift_state', '_vision_build', 'vision_world', 'drift',
                '_censorship_state', '_merge_censorship'):
        assert bad not in names, '시청 경로에 %s 가 들어왔다' % bad


# ---- 마스크가 불리는가 (MCV-2M) -----------------------------------------------------

def test_마스크_모듈을_들여온다():
    assert "import masks" in _src()


def test_마스크_엔드포인트가_있고_같은_로컬_게이트다():
    """관측 좌표계를 바꾸는 선언이다. 원격에서 그릴 수 있으면 안 된다."""
    src = _src()
    assert "'/api/calibration/masks'" in src
    i = src.index("parsed.path == '/api/calibration/masks'")
    assert "LOCAL_CONTROL_IPS" in src[i:i + 500]


def test_상태에_마스크가_실린다():
    st = _func("_calib_state")
    assert "_calib_masks" in _names(st)
    body = ast.dump(st)
    assert "masksProblem" in body, "못 읽은 것을 조용히 넘긴다"


def test_마스크_안에는_핀을_못_찍는다():
    """⭐ 안 보기로 한 자리를 모서리로 쓰면 그 행렬은 못 보는 점으로 푼 것이 된다."""
    post = _func("do_POST")
    assert "_mask_blocking" in _names(post)


def test_기준_조각이_마스크를_피한다():
    """⭐ 흔들리는 케이블 하나가 MATCH_LOST 로 **진짜 흔들림을 가린다**."""
    anchor = _func("_drift_anchor")
    src = _src()
    i = src.index("def _drift_anchor")
    body = src[i:i + 1200]
    assert "take_patches(frame, corners, masks=" in body,         "조각을 뜰 때 마스크를 안 넘긴다"
    assert "_calib_masks" in _names(anchor)


def test_평면도가_마스크를_넘긴다():
    """가린 자리를 바닥으로 읽으면 로봇이 삼각대로 들어간다."""
    vb = _func("_vision_build")
    assert "_calib_masks" in _names(vb)
    assert "masks=ms" in _src()[_src().index("def _vision_build"):][:1400]


def test_마스크가_바뀌면_흔들림_캐시를_버린다():
    """조각 구성이 달라지는데 옛 판정을 5초 더 보여 주면 거짓이다."""
    src = _src()
    i = src.index("parsed.path == '/api/calibration/masks'")
    assert "GLOBAL_DRIFT_CACHE.pop" in src[i:i + 1800]


def test_시청_경로에_마스크도_안_들어간다():
    """🔴 시청은 계속한다(사용자 결정). 무거운 판정이 새면 화면이 느려진다."""
    br = _branch_for('do_GET', '/video_feed')
    names = _names(br) | _attrs(br)
    for bad in ('masks', '_calib_masks', '_mask_blocking'):
        assert bad not in names, '시청 경로에 %s 가 들어왔다' % bad


# ---- 화각 점검이 불리는가 (MCV-2G) ----------------------------------------------------

def test_화각_모듈을_들여온다():
    assert "import framing" in _src()


def test_화각_엔드포인트가_있다():
    src = _src()
    assert "'/api/calibration/framing'" in src


def test_화각은_읽기라_로컬_게이트가_없다():
    """⭐ 이 값은 관측을 **바꾸지 않는다.** 바꾸는 것만 로컬로 묶는다 —
       게이트를 남발하면 정작 중요한 게이트가 관습으로 보인다."""
    src = _src()
    i = src.index("parsed.path == '/api/calibration/framing'")
    body = src[i:i + 700]
    assert "LOCAL_CONTROL_IPS" not in body


def test_화각이_곁표에서_수평을_읽는다():
    fr = _func("_calib_framing")
    assert "clock_info" in _attrs(fr)
    assert "sidecar=sc" in _src()[_src().index("def _calib_framing"):][:1400]


def test_화각이_프레임_크기를_요청에서_안_받는다():
    """⭐ 클라이언트가 보낸 크기를 믿으면 가이드가 다른 화면을 그린다."""
    fr = _func("_calib_framing")
    assert "_calib_frame_facts" in _names(fr)


def test_시청_경로에_화각도_안_들어간다():
    br = _branch_for('do_GET', '/video_feed')
    names = _names(br) | _attrs(br)
    for bad in ('framing', '_calib_framing'):
        assert bad not in names, '시청 경로에 %s 가 들어왔다' % bad
