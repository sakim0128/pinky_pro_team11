# -*- coding: utf-8 -*-
"""같은 그림을 두 번 보내지 않는다 — 그리고 **새 그림을 삼키지 않는다.**

배경: 2026-09-12 에 `jpeg is not prev`(객체 동일성)로 한 번 짰다가 되돌렸다
(커밋 5a252e4 -> 7ef5c29). 동일성은 "제공자가 프레임마다 새 객체를 준다"는,
**우리가 소유하지 않는 전제** 위에 서 있었다. 이 파일은 값 비교로 바뀐 판정을 지킨다.

각 시험은 *무엇이 틀리면 빨개지는가* 를 docstring 첫 줄에 적는다.

⚠️ 이 파일의 전제부터 단언한다. 예전 시험이 `b"XY"*10` 을 두 번 써서
   "값 같고 객체 다름" 을 만들었다고 믿었는데, 파이썬이 상수를 인터닝해
   **같은 객체**였다 — 즉 그 시험은 자기가 재려던 것을 한 번도 안 쟀다.
"""
import ast
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # relay_station
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gateway_web"))

import pytest

from mjpeg_serving import (STREAM_KEEPALIVE_SEC, should_send_frame,
                           write_mjpeg_frame)

_GATEWAY = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "gateway_web", "gateway_web_server.py")


def 같은값_다른객체(data):
    """값은 같고 객체는 다른 bytes 를 만든다. **만들어졌는지 단언한다.**"""
    other = (data + b"\x00")[:-1]
    assert other is not data, "전제 실패: 인터닝돼 같은 객체다 — 이 시험은 아무것도 안 재고 있다"
    assert other == data, "전제 실패: 값이 다르다"
    return other


FRAME_A = bytes(bytearray(range(256)) * 40)      # 10KB
FRAME_B = bytes(bytearray(range(255, -1, -1)) * 40)


def test_전제_같은값_다른객체를_실제로_만들_수_있다():
    """헬퍼가 인터닝에 걸리면 빨개진다. 아래 시험 전부가 이것에 기댄다."""
    other = 같은값_다른객체(FRAME_A)
    assert other is not FRAME_A
    assert other == FRAME_A


# ---- should_send_frame ----------------------------------------------------

def test_프레임이_없으면_안_보낸다():
    """None 을 보내려 들면 빨개진다(len(None) 로 터지는 것을 막는다)."""
    assert should_send_frame(None, None, 0.0, 100.0) is False
    assert should_send_frame(None, FRAME_A, 0.0, 100.0) is False


def test_첫_프레임은_보낸다():
    """prev 가 없을 때 안 보내면 빨개진다 — 스트림이 영영 안 시작된다."""
    assert should_send_frame(FRAME_A, None, 0.0, 100.0) is True


def test_값이_다르면_보낸다():
    """새 그림을 삼키면 빨개진다. 이게 되돌린 회귀의 증상이었다."""
    assert should_send_frame(FRAME_B, FRAME_A, 100.0, 100.01) is True


def test_값이_같으면_객체가_달라도_안_보낸다():
    """🔴 동일성(is) 판정으로 되돌아가면 빨개진다.

    제공자가 프레임마다 새 객체를 주면서 내용은 같을 수 있다(재인코딩·복사).
    `is` 비교는 이걸 '새 프레임'으로 보고 중복을 그대로 흘린다 — 중복 제거가 무력해진다.
    """
    other = 같은값_다른객체(FRAME_A)
    assert should_send_frame(other, FRAME_A, 100.0, 100.01) is False


def test_같은_객체면_안_보낸다():
    """제공자가 같은 객체를 돌려주는 흔한 경우. 보내면 빨개진다."""
    assert should_send_frame(FRAME_A, FRAME_A, 100.0, 100.01) is False


def test_keepalive_주기가_지나면_같은_그림도_다시_보낸다():
    """멈춘 소스에 아무것도 안 보내면 빨개진다 — 클라이언트가 연결을 죽은 걸로 본다."""
    now = 100.0 + STREAM_KEEPALIVE_SEC
    assert should_send_frame(FRAME_A, FRAME_A, 100.0, now) is True


def test_keepalive_직전에는_안_보낸다():
    """경계가 밀리면 빨개진다(>= 를 > 로 바꾸는 변이를 잡는다)."""
    now = 100.0 + STREAM_KEEPALIVE_SEC - 0.001
    assert should_send_frame(FRAME_A, FRAME_A, 100.0, now) is False


# ---- write_mjpeg_frame ----------------------------------------------------

class 기록기:
    def __init__(self):
        self.buf = b""

    def write(self, b):
        self.buf += b


def test_구분자는_CRLF_다():
    """CRLF 를 LF 로 바꾸면 빨개진다 — 일부 클라이언트가 파트를 못 자른다."""
    w = 기록기()
    write_mjpeg_frame(w, FRAME_A)
    assert w.buf.startswith(b"--frame\r\n")
    assert w.buf.endswith(b"\r\n")
    # ⚠️ 페이로드(JPEG)에는 0x0A 가 얼마든지 들어 있다. **헤더 구간만** 본다 —
    #    첫 판은 본문까지 훑어서 자기 픽스처에 걸려 빨개졌다.
    헤더, _, 본문 = w.buf.partition(b"\r\n\r\n")
    assert 본문, "헤더와 본문을 가르는 빈 줄이 없다"
    assert 헤더.replace(b"\r\n", b"").count(b"\n") == 0, "헤더에 LF 단독이 섞였다"
    assert 헤더.count(b"\r\n") == 2, "헤더 줄은 둘(--frame, Content-Type)이어야 한다"


def test_Content_Length_가_실제_길이다():
    """길이를 잘못 쓰면 빨개진다 — 클라이언트가 파트 경계를 잃는다."""
    w = 기록기()
    write_mjpeg_frame(w, FRAME_A)
    assert b"Content-Length: %d\r\n\r\n" % len(FRAME_A) in w.buf
    assert w.buf.count(FRAME_A) == 1


# ---- 모양: 네 루프 전부 ----------------------------------------------------
#
# ⭐ 한 곳만 고치면 나머지 셋에 같은 결함이 남는다. 그리고 판정은 **AST** 로 한다 —
#    부분문자열은 주석에 걸린다("설명 주석이 자기 검사에 걸린다").

@pytest.fixture(scope="module")
def 본체():
    src = io.open(_GATEWAY, encoding="utf-8").read()
    return ast.parse(src)


def _호출이름(node):
    f = node.func
    return f.id if isinstance(f, ast.Name) else getattr(f, "attr", None)


def test_본체에_손으로_쓴_프레임_기록이_없다(본체):
    """루프 어딘가가 다시 `--frame` 을 손으로 쓰면 빨개진다 — 그 루프엔 중복 제거가 없다."""
    손쓰기 = [n for n in ast.walk(본체)
             if isinstance(n, ast.Constant) and isinstance(n.value, bytes)
             and n.value.startswith(b"--frame")]
    assert 손쓰기 == [], "본체가 프레임을 직접 쓴다 — write_mjpeg_frame 을 지나야 한다"


def test_모든_프레임_쓰기가_판정_안에_있다(본체):
    """`write_mjpeg_frame` 이 `should_send_frame` 밖에서 불리면 빨개진다.

    ⭐ 개수만 세면 '4개씩 있으니 됐다'로 통과할 수 있다. **포함 관계**를 본다.
    """
    전체 = {id(n) for n in ast.walk(본체)
           if isinstance(n, ast.Call) and _호출이름(n) == "write_mjpeg_frame"}
    판정안 = set()
    판정수 = 0
    for node in ast.walk(본체):
        if not isinstance(node, ast.If):
            continue
        if not (isinstance(node.test, ast.Call)
                and _호출이름(node.test) == "should_send_frame"):
            continue
        판정수 += 1
        for child in node.body:
            for n in ast.walk(child):
                if isinstance(n, ast.Call) and _호출이름(n) == "write_mjpeg_frame":
                    판정안.add(id(n))
    # 개편 2단계(2026-09-29): /control_feed · /gazebo_feed · /robot_camera_feed 를 지웠다 — 그 소스들은 /video_feed?src= 한 루프로 나간다
    assert 판정수 == 1, "스트림 루프는 /video_feed 하나다. 실제 판정 %d 곳" % 판정수
    assert len(전체) == 1, "프레임 쓰기는 하나여야 한다. 실제 %d" % len(전체)
    assert 전체 == 판정안, "판정 밖에서 프레임을 쓰는 자리가 있다"


def test_보정하는_루프는_보정_전_원본을_prev_로_삼는다(본체):
    """보정본(`jpeg`)을 prev 에 넣으면 빨개진다.

    boost/watermark 가 켜지면 `jpeg` 이 재인코딩본으로 바뀐다. 그걸 prev 에 저장하면
    다음 회차에 **원본 vs 보정본**을 비교하게 되어 항상 '다르다'가 나오고
    중복 제거가 통째로 무력해진다. 2026-09-12 에 실제로 한 번 심었던 버그다.
    """
    보정루프 = []
    for node in ast.walk(본체):
        if not (isinstance(node, ast.If) and isinstance(node.test, ast.Call)
                and _호출이름(node.test) == "should_send_frame"):
            continue
        # 이 몸통 안에서 jpeg 이 재대입되는가 = 보정하는 루프다
        재대입 = any(isinstance(t, ast.Name) and t.id == "jpeg"
                   for child in node.body for n in ast.walk(child)
                   if isinstance(n, ast.Assign) for t in n.targets)
        if 재대입:
            보정루프.append(node)
    assert len(보정루프) == 1, "보정하는 루프는 하나(video_feed)여야 한다. 실제 %d" % len(보정루프)

    prev출처 = []
    for child in 보정루프[0].body:
        for n in ast.walk(child):
            if not isinstance(n, ast.Assign):
                continue
            for t in n.targets:
                if isinstance(t, ast.Tuple) and any(
                        isinstance(e, ast.Name) and e.id == "_prev" for e in t.elts):
                    i = [e.id for e in t.elts if isinstance(e, ast.Name)].index("_prev")
                    v = n.value.elts[i]
                    prev출처.append(v.id if isinstance(v, ast.Name) else "?")
    assert prev출처, "보정 루프가 _prev 를 갱신하지 않는다"
    assert "jpeg" not in prev출처, (
        "보정본을 _prev 에 넣었다 — 재인코딩 전 원본을 따로 붙들어야 한다. 실제: %r" % prev출처)
