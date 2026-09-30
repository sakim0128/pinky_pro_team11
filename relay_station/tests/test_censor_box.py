# -*- coding: utf-8 -*-
"""MCV-2R — 검열 상자를 받아서 쓰는 쪽의 성질.

가공 단(외부 레포)이 파트 헤더로 낸다:

    X-Process-Blur: 0,0,0.5,0.5      프레임 비율 x,y,w,h · 꺼지면 **헤더 자체가 없음**

⭐ 그 파서를 저쪽 모듈에서 import 하지 않는다. 두 레포는 따로 배포되고 현장 장비에는
   외부 레포가 없다. **계약은 헤더 형식이지 코드가 아니다.** 그래서 형식을 여기서 고정한다.
"""
import io
import json
import os

import pytest

import calibration as C
from mjpeg_puller import MjpegPuller, parse_blur, H_PROCESS_BLUR, H_CAPTURE_CLOCK

_HERE = os.path.dirname(os.path.abspath(__file__))
_RELAY = os.path.dirname(_HERE)

FRAME = (1280, 720)
QUAD = [(180.0, 640.0), (1120.0, 660.0), (900.0, 210.0), (330.0, 200.0)]
ARENA = {"version": "t1", "widthCm": 270.0, "heightCm": 125.0,
         "landmarks": {"block-nw": {"x": 97.0, "y": 70.5}}}


@pytest.fixture
def store(tmp_path):
    p = tmp_path / "arena.json"
    p.write_text(json.dumps(ARENA), encoding="utf-8")
    return C.CalibrationStore(C.load_arena(str(p)),
                              state_dir=str(tmp_path / "cal"), clock=lambda: 1000.0)


# ---- 형식 --------------------------------------------------------------------

def test_가공_단_형식_그대로_읽는다():
    assert parse_blur("0,0,0.5,0.5") == {"x": 0.0, "y": 0.0, "w": 0.5, "h": 0.5}
    assert parse_blur("0.25,0.1,0.3,0.2") == {"x": 0.25, "y": 0.1, "w": 0.3, "h": 0.2}


@pytest.mark.parametrize("bad", ["", None, "0,0,0.5", "0,0,0.5,0.5,0.1",
                                 "a,b,c,d", "0,0,0,0.5", "0,0,0.5,0",
                                 "0,0,nan,0.5", "0,0,inf,0.5"])
def test_못_읽는_값은_None(bad):
    assert parse_blur(bad) is None


def test_넓이가_0_이면_상자가_아니다():
    """w 나 h 가 0 이면 가려진 영역이 없다. 그걸 상자라고 부르면 헛되이 핀을 막는다."""
    assert parse_blur("0.1,0.1,0,0.5") is None
    assert parse_blur("0.1,0.1,0.5,0") is None


# ---- 곁표 -------------------------------------------------------------------

def _puller():
    return MjpegPuller("http://127.0.0.1:9/video", name="unit")


def test_곁표에_상자와_원문이_함께_실린다():
    p = _puller()
    p._note_sidecar({H_PROCESS_BLUR: "0,0,0.5,0.5"}, 1000.0)
    sc = p.sidecar()
    assert sc["censorBox"] == {"x": 0.0, "y": 0.0, "w": 0.5, "h": 0.5}
    assert sc["censorBoxRaw"] == "0,0,0.5,0.5"


def test_헤더가_사라지면_상자도_사라진다():
    """⭐ 상자를 끄면 헤더 자체가 없어진다. 마지막 값을 붙들고 있으면 **없는 검열을 믿는다.**"""
    p = _puller()
    p._note_sidecar({H_PROCESS_BLUR: "0,0,0.5,0.5"}, 1000.0)
    assert p.sidecar()["censorBox"] is not None
    p._note_sidecar({H_CAPTURE_CLOCK: "1000000"}, 1000.1)      # 상자 헤더 없음
    assert p.sidecar()["censorBox"] is None
    assert p.sidecar()["censorBoxRaw"] is None


def test_모른다와_없다를_가른다():
    """⭐ 부재는 '상자가 없다', 못 읽음은 '모른다'. 둘을 뭉치면 규약이 바뀐 날
    관측기가 조용히 '안 가려졌다'고 믿는다."""
    p = _puller()
    p._note_sidecar({H_PROCESS_BLUR: "이건 뭐지"}, 1000.0)
    sc = p.sidecar()
    assert sc["censorBox"] is None, "못 읽었으니 상자는 없다"
    assert sc["censorBoxRaw"] == "이건 뭐지", "그런데 값이 오긴 왔다는 사실은 남아야 한다"


def test_상자가_없으면_곁표도_None():
    sc = _puller().sidecar()
    assert sc["censorBox"] is None and sc["censorBoxRaw"] is None


# ---- 기하 --------------------------------------------------------------------

def test_상자_안팎을_가른다():
    box = {"x": 0.0, "y": 0.0, "w": 0.5, "h": 0.5}       # 좌상단 1/4
    assert C.point_in_box((10.0, 10.0), FRAME, box)
    assert C.point_in_box((639.0, 359.0), FRAME, box)
    assert not C.point_in_box((641.0, 361.0), FRAME, box)
    assert not C.point_in_box((1200.0, 700.0), FRAME, box)


def test_상자가_없으면_아무데나_된다():
    assert not C.point_in_box((10.0, 10.0), FRAME, None)


# ---- 캘리브레이션 ----------------------------------------------------------------

def test_가려진_자리의_핀을_거부한다(store):
    """⭐ 안 보이는 자리에 찍은 핀은 찍은 게 아니라 **짐작한 것**이다.

    현장 기본 상자가 좌상단 1/4 이었고 아레나 코너는 화면 가장자리에 온다 —
    tl 핀이 정확히 거기 앉는다.
    """
    box = {"x": 0.0, "y": 0.0, "w": 0.5, "h": 0.5}
    with pytest.raises(C.CalibrationError, match="검열 상자 안"):
        store.set_points("phone", QUAD, FRAME, censor_box=box)


def test_상자_밖이면_통과한다(store):
    box = {"x": 0.0, "y": 0.0, "w": 0.1, "h": 0.1}       # 아주 작은 상자
    store.set_points("phone", QUAD, FRAME, censor_box=box)
    assert store.state("phone")["state"] == C.STATE_CALIBRATING


def test_가려진_검증점도_거부한다(store):
    """못 본 것을 잰 척하면, 그 오차가 캘리브레이션 탓이 아닌데 그렇게 읽힌다."""
    box = {"x": 0.0, "y": 0.0, "w": 0.1, "h": 0.1}
    with pytest.raises(C.CalibrationError, match="검증점"):
        store.set_points("phone", QUAD, FRAME, censor_box=box,
                         verify=[{"name": "block-nw", "px": [40.0, 40.0]}])


def test_영수증이_그때의_상자를_적는다(store):
    box = {"x": 0.6, "y": 0.6, "w": 0.2, "h": 0.2}
    store.set_points("phone", QUAD, FRAME, censor_box=box)
    r = store.settle("phone")
    assert r["censorBox"] == box, "무엇을 못 본 채 잰 값인지 남아야 한다"


def test_상자가_없으면_영수증에도_None(store):
    store.set_points("phone", QUAD, FRAME)
    assert store.settle("phone")["censorBox"] is None


def test_상자가_바뀌어도_정착은_무효가_아니다(store):
    """⭐ 이건 **일부러** 그렇게 뒀다. 흐림은 기하가 아니라 검출을 건드린다.

    호모그래피는 픽셀→cm 사상이라 어디가 흐리든 그대로 맞다. 상자 변화로 정착을
    내리면 **맞는 값을 버리게 된다.** 그래서 무효화 조건이 아니라 출처로만 적는다.
    (무효화 조건은 아레나 버전·프레임 크기·발행자 세션 셋이다.)
    """
    box = {"x": 0.6, "y": 0.6, "w": 0.2, "h": 0.2}
    store.set_points("phone", QUAD, FRAME, publisher_session="s1", censor_box=box)
    store.settle("phone")
    st = store.state("phone", frame_size=FRAME, publisher_session="s1")
    assert st["state"] == C.STATE_SETTLED
    assert st["reason"] is None
    assert st["censorBox"] == box


# ---- 서버 표면 ----------------------------------------------------------------

def _server():
    with io.open(os.path.join(_RELAY, "gateway_web", "gateway_web_server.py"),
                 encoding="utf-8") as fh:
        return fh.read()


def test_서버가_상자를_곁표에서_읽는다():
    src = _server()
    assert "def _calib_censor" in src
    assert "'censorBox'" in src and "'censorBoxRaw'" in src
    assert "censor_box=box" in src, "set_points 에 상자를 안 넘기고 있다"


def test_상태에_지금_걸린_상자를_낸다():
    src = _server()
    assert "liveCensorBox" in src and "liveCensorBoxRaw" in src


# ---- 화면 --------------------------------------------------------------------

def _js():
    with io.open(os.path.join(_RELAY, "gateway_web", "static", "index.html"),
                 encoding="utf-8") as fh:
        t = fh.read()
    a = t.find("// ===== MCV-2A-UI 좌표계 정착 =====")
    b = t.find("// --- 관제 카메라 다중 소스 및 AR", a + 10)
    assert a >= 0 and b > a
    return t[a:b]


def test_화면이_상자를_그린다():
    """서버가 거부하기는 한다. 그래도 **왜** 못 찍는지 눈에 보여야 사람이 고친다."""
    js = _js()
    assert "calCensor" in js
    assert "liveCensorBox" in js
    assert "검열됨" in js


def test_화면도_모른다와_없다를_가른다():
    js = _js()
    assert "liveCensorBoxRaw" in js
    assert "못 읽었다" in js


# ---- 발행기 (실기기 없이 시험하려고) ---------------------------------------------------

def test_발행기가_같은_형식으로_낸다():
    with io.open(os.path.join(_RELAY, "docker", "host_camera_publisher.py"),
                 encoding="utf-8") as fh:
        src = fh.read()
    assert "X-Process-Blur" in src
    assert "def header_value" in src
    assert '"%g,%g,%g,%g"' in src, "가공 단의 format_blur 와 같은 규약이어야 한다"


def test_발행기는_상자가_없으면_헤더를_안_낸다():
    with io.open(os.path.join(_RELAY, "docker", "host_camera_publisher.py"),
                 encoding="utf-8") as fh:
        src = fh.read()
    body = src.split("def header_value", 1)[1][:300]
    assert "if not self.box" in body and "return None" in body


# ---- 짝 (2026-09-10 실제 사고) -------------------------------------------------

def test_모든_곁표_상수가_허용목록에_있다():
    """⭐ `parse_part_headers` 는 `SIDECAR_HEADERS` 에 없는 헤더를 **버린다.**

    상수만 만들고 목록에 안 넣으면 **기능이 죽은 채로 유닛테스트가 통과한다** —
    `_note_sidecar` 를 직접 부르는 시험은 그 구간을 건너뛰기 때문이다.
    2026-09-10 에 X-Process-Blur 로 정확히 그 일이 났다. 멀티뷰 배지 때와 같은 모양이다.

    그래서 **짝**을 강제한다: 모듈의 H_* 상수는 전부 목록에 있어야 한다.
    """
    import mjpeg_puller as MP
    consts = {n: v for n, v in vars(MP).items()
              if n.startswith("H_") and isinstance(v, str)}
    assert consts, "H_* 상수를 못 찾았다"
    missing = sorted(n for n, v in consts.items() if v not in MP.SIDECAR_HEADERS)
    assert not missing, ("상수는 있는데 SIDECAR_HEADERS 에 없다 — 그 헤더는 버려진다: %s"
                         % missing)


def test_실제_파트_헤더_경로로_상자가_들어온다():
    """⭐ `_note_sidecar` 를 직접 부르지 않는다. **바이트에서 시작한다.**

    직접 부르면 허용목록을 건너뛰어서 죽은 기능을 통과시킨다.
    """
    from mjpeg_puller import parse_part_headers
    eol = bytes([13, 10])
    pre = (b"--raasframe" + eol + b"Content-Type: image/jpeg" + eol +
           b"X-Capture-Clock: 1789000000000" + eol +
           b"X-Process-Rules: v2" + eol +
           b"X-Process-Blur: 0,0,0.5,0.5" + eol + eol)
    meta = parse_part_headers(pre)
    assert H_PROCESS_BLUR in meta, "파트 헤더 파서가 상자를 버렸다"
    p = _puller()
    p._note_sidecar(meta, 1000.0)
    sc = p.sidecar()
    assert sc["censorBox"] == {"x": 0.0, "y": 0.0, "w": 0.5, "h": 0.5}
    assert sc["processRules"] == "v2"


def test_상자_헤더가_빠진_파트도_정상이다():
    """상자가 꺼져 있으면 헤더 자체가 없다. 그게 정상 경로다."""
    from mjpeg_puller import parse_part_headers
    eol = bytes([13, 10])
    pre = (b"--raasframe" + eol + b"Content-Type: image/jpeg" + eol +
           b"X-Capture-Clock: 1789000000000" + eol + eol)
    meta = parse_part_headers(pre)
    assert H_PROCESS_BLUR not in meta
    p = _puller()
    p._note_sidecar(meta, 1000.0)
    assert p.sidecar()["censorBox"] is None
