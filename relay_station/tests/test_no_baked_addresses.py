# -*- coding: utf-8 -*-
"""이동하는 기기의 주소를 **코드에 박지 않는다.**

2026-09-12, 다른 세션이 도커 리그에서 실측해 알려줬다:

    [Mobile Ingest] Waiting for Mobile/Tablet at http://198.51.100.2:8080/video ...
    누적 5,293줄 · 15초에 2줄씩 증가

주소도 포트도 죽어 있었다 — 태블릿은 앱 `:18086` / 중계 `:18082` 를 쓴다. 셋 다 8080 이
아니었다. ⭐⭐ 그런데 진짜 피해는 로그 소음이 아니라 **가림**이다: 닿지 않는 재접속이
로그를 채우면 **진짜 접속 실패가 그 사이에 묻힌다.** 그리고 `connected:false` 가
"기기가 없다" 인지 "주소가 틀렸다" 인지 화면에서 구분이 안 된다.

## ⭐ 무엇이 허용되고 무엇이 아닌가

    고정 설비    공유기·로봇은 아레나 명세가 주소를 정한다 — 박아도 된다
    이동 기기    태블릿·폰은 들고 다닌다. 사이트마다 바뀐다 — **박으면 안 된다**
                 후보 목록은 `configs/video_sources.json` 이 가진다
    보안 허용목록 `_BASE_LOCAL_CONTROL_IPS` 는 **누가 캘리브레이션할 수 있나**를 정하는
                 의도적 결정이다. 여기서 막을 대상이 아니다.

⚠️ 기록(접속 안내·기동 배너)은 다른 범주다 — 거기는 지우지 않고 as-of 를 붙였다.
"""
import io
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
GW = os.path.join(os.path.dirname(_HERE), "gateway_web")

IPV4 = re.compile(r"\b(?:\d{1,3}\.){3}\d{1,3}\b")

# 고정 설비와 관용 주소. ⭐ 왜 허용되는지를 값 옆에 적는다 - 다음 사람이 늘릴 때 근거를 본다.
ALLOWED = {
    "127.0.0.1": "루프백",
    "0.0.0.0": "모든 인터페이스에 바인드",
    "255.255.255.255": "브로드캐스트",
    "8.8.8.8": "외부망 도달 확인용 공개 DNS",
    "198.51.100.1": "공유기 — 아레나 명세가 정한 고정 설비",
    "198.51.100.3": "중계 노트북 자신 — 보안 허용목록(누가 캘리브레이션하나)",
    "198.51.100.5": "Robot #1 — 고정 설비",
    "198.51.100.6": "Robot #2 — 고정 설비",
    "198.51.100.8": "Robot #3 — 고정 설비",
    # 기동 배너의 접속 **기록**. 코드에 있지만 상태 보고가 아니고 as-of 를 달고 찍는다.
    "203.0.113.150": "기동 배너의 접속 기록 (as-of 표기 있음)",
    "100.64.0.81": "기동 배너의 접속 기록 (as-of 표기 있음)",
}

# 이동 기기의 옛 주소. 되돌아오면 잡아야 한다.
FORBIDDEN = ("198.51.100.2", "100.64.0.6", "198.51.100.7", "100.64.0.7",
             "203.0.113.59")


def _py_files():
    return [os.path.join(GW, f) for f in sorted(os.listdir(GW))
            if f.endswith(".py")]


def _code_lines(path):
    """주석을 뺀 줄들. ⭐ '예전에 이랬다' 는 주석을 잡으면 거짓 실패다."""
    out = []
    with io.open(path, encoding="utf-8", errors="replace") as fh:
        for i, line in enumerate(fh, 1):
            code = line.split("#", 1)[0]
            if code.strip():
                out.append((i, code, line.rstrip()))
    return out


# ---- ⭐ 검사가 실패할 수 있는가 ------------------------------------------------------

def test_볼_파일이_있다():
    """⭐⭐ 대상이 0 개면 아래 시험은 구조적으로 실패할 수 없다."""
    files = _py_files()
    assert len(files) >= 8, "게이트웨이 파이썬이 %d 개뿐이다" % len(files)


def test_정규식이_주소를_실제로_잡는다():
    assert IPV4.findall('url = "http://198.51.100.2:8080/video"') == ["198.51.100.2"]
    assert not IPV4.findall("port = 18082")


# ---- 본 검사 -----------------------------------------------------------------

def test_이동_기기_주소가_코드에_없다():
    """⭐ 태블릿·폰 주소는 `configs/video_sources.json` 이 후보 목록으로 가진다."""
    bad = []
    for p in _py_files():
        for ln, code, raw in _code_lines(p):
            for ip in IPV4.findall(code):
                if ip in FORBIDDEN:
                    bad.append("%s:%d  %s  | %s"
                               % (os.path.basename(p), ln, ip, raw.strip()[:80]))
    assert not bad, (
        "이동 기기 주소가 코드에 박혀 있다 — 사이트마다 바뀌고, 죽은 주소로 "
        "재접속하면 **진짜 접속 실패를 가린다**." + chr(10)
        + "후보 목록은 configs/video_sources.json 이 가진다." + chr(10)
        + chr(10).join(bad))


def test_모르는_주소가_늘면_근거를_요구한다():
    """⭐ 허용 목록에 없는 주소가 새로 들어오면 여기서 멈춘다 — 근거를 적고 올린다."""
    unknown = {}
    for p in _py_files():
        for ln, code, raw in _code_lines(p):
            for ip in IPV4.findall(code):
                if ip not in ALLOWED:
                    unknown.setdefault(ip, []).append(
                        "%s:%d %s" % (os.path.basename(p), ln, raw.strip()[:70]))
    assert not unknown, (
        "근거 없는 주소가 코드에 있다. 고정 설비면 ALLOWED 에 **왜 허용되는지와 함께** "
        "올리고, 이동 기기면 설정으로 뺀다:" + chr(10)
        + chr(10).join("  %s  %s" % (k, v[0]) for k, v in sorted(unknown.items())))


# ---- ⭐ 주소가 없으면 재접속하지 않는다 -------------------------------------------------

def _ingest_src():
    p = os.path.join(GW, "stream_ingest.py")
    if not os.path.exists(p):
        pytest.skip("stream_ingest.py 없음")
    with io.open(p, encoding="utf-8") as fh:
        return fh.read()


def test_기본_주소가_비어_있다():
    """⭐⭐ 주소를 **지어내지 않는다.** 지어낸 주소는 닿지 않고, 닿지 않는 재접속이
       로그를 채우고, 그 로그가 진짜 실패를 가린다."""
    src = _ingest_src()
    m = re.search(r"def __init__\(self,\s*stream_url=([^,]+),\s*fallback_url=([^)]+)\)",
                  src)
    assert m, "생성자 서명을 못 찾았다"
    for g in (m.group(1).strip(), m.group(2).strip()):
        assert g in ('""', "''"), "기본 주소가 비어 있지 않다: %s" % g


def test_주소가_없으면_붙으러_안_간다():
    """⭐ '못 붙었다' 와 '붙을 데를 안 줬다' 는 다른 사실이다. 뭉개면 앞의 것을 못 본다."""
    src = _ingest_src()
    i = src.find("def _capture_loop")
    assert i > 0, "_capture_loop 을 못 찾았다"
    body = src[i:i + 2500]
    assert "if not self.primary_url" in body, "주소가 없어도 붙으러 간다"
    guard = body.index("if not self.primary_url")
    attempt = body.index("_try_open")
    assert guard < attempt, "붙어 본 뒤에 주소를 확인한다 — 순서가 거꾸로다"


def test_주소_없음을_한_번만_말한다():
    """그것도 5초마다 반복하면 같은 소음이 된다."""
    assert "idle_noted" in _ingest_src()
