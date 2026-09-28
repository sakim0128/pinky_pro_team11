# -*- coding: utf-8 -*-
"""팀11 레포로 내보낼 때 지킨 규칙을 잠근다 — 다시 내보내도 깨지지 않게.

1. 비전 API 키: 코드에 기본값이 없고, 키가 없으면 전부 거절(fail-closed)이다.
2. 주소: 사설·tailnet 주소는 문서용 자리표시자(RFC 5737 대역, 100.64.0.x)만 남는다.
3. 개인 기기 경로가 없다.
4. 이 폴더는 colcon 이 건너뛴다(COLCON_IGNORE) — 팀11 워크스페이스 빌드에 끼지 않는다.
"""
import io
import os
import re

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
RELAY = os.path.dirname(_HERE)
GATEWAY = os.path.join(RELAY, "gateway_web", "gateway_web_server.py")
TEXT_EXT = (".py", ".sh", ".md", ".json", ".yaml", ".yml", ".xml", ".html", ".js", ".css", ".txt",
            ".service", ".env", ".svg")

# RFC 1918 사설 대역 + CGNAT(100.64/10, tailnet 이 쓰는 대역)
PRIVATE = re.compile(r"\b(10\.\d+\.\d+\.\d+|172\.(1[6-9]|2\d|3[01])\.\d+\.\d+|192\.168\.\d+\.\d+"
                     r"|100\.(6[4-9]|[7-9]\d|1[01]\d|12[0-7])\.\d+\.\d+)\b")
TAILNET_PLACEHOLDER = re.compile(r"^100\.64\.0\.\d+$")
ALLOWED = {
    ("tests/test_source_registry.py", "10.0.0.1"),   # 비밀이 새지 않는지 보는 시험의 가짜 RTSP 주소
}


def _texts():
    for root, dirs, files in os.walk(RELAY):
        dirs[:] = [d for d in dirs if d != "__pycache__"]
        for f in files:
            if f.endswith(TEXT_EXT):
                p = os.path.join(root, f)
                if os.path.abspath(p) == os.path.abspath(__file__):
                    continue                          # 이 파일은 찾을 모양을 적고 있다
                with io.open(p, encoding="utf-8", errors="replace") as fh:
                    yield os.path.relpath(p, RELAY), fh.read()


def _gateway_src():
    with io.open(GATEWAY, encoding="utf-8") as fh:
        return fh.read()


def test_비전_키는_코드에_기본값이_없다():
    src = _gateway_src()
    assert re.search(r"^VISION_API_KEY = os\.environ\.get\('RELAY_VISION_API_KEY'\) "
                     r"or os\.environ\.get\('RELAY_VISION_TOKEN'\) or ''$", src, re.M), \
        "VISION_API_KEY 의 마지막 기본값이 빈 문자열이 아니다"
    leaked = [rel for rel, text in _texts() if "pinky-field-vision-key" in text]
    assert not leaked, leaked


def test_키가_없으면_거절한다__소스():
    src = _gateway_src()
    i = src.index("def _check_vision_auth")
    assert re.search(r"if not VISION_API_KEY:\s*\n\s*return False", src[i:i + 900]), "키가 비면 통과시킨다(fail-open)"


def test_키가_없으면_거절한다__진짜_메서드(monkeypatch):
    try:
        import gateway_web_server as gws
    except Exception as exc:                                  # noqa: BLE001
        pytest.skip("게이트웨이 import 불가(ROS 없음): %s" % exc)
    monkeypatch.setattr(gws, "VISION_API_KEY", "")

    class H:
        def __init__(self, headers):
            self.headers = headers
            self.path = "/api/vision/pose_fix?token="

    for hdr in ({}, {"X-API-Key": ""}, {"X-API-Key": "anything"}, {"Authorization": "Bearer "}):
        assert gws.GatewayRequestHandler._check_vision_auth(H(hdr)) is False, hdr


def test_사설_주소는_문서용_자리표시자뿐이다():
    bad = []
    for rel, text in _texts():
        for m in PRIVATE.finditer(text):
            ip = m.group(0)
            if TAILNET_PLACEHOLDER.match(ip) or (rel, ip) in ALLOWED:
                continue
            bad.append("%s: %s" % (rel, ip))
    assert not bad, "현장 주소처럼 보이는 것이 남았다:\n  " + "\n  ".join(bad)


def test_개인_기기_경로가_없다():
    pat = re.compile(r"/home/[a-z][a-z0-9_-]*/|[A-Za-z]:\\Users\\")
    bad = ["%s: %s" % (rel, m.group(0)) for rel, text in _texts() for m in pat.finditer(text)]
    assert not bad, bad


def test_colcon_이_건너뛴다():
    assert os.path.exists(os.path.join(RELAY, "COLCON_IGNORE"))


def test_외부_사이트를_가리키는_낱말이_없다():
    """공개 팀 레포 — 원 저장소 운영 환경의 이름(홈랩/homelab)은 식별자가 아니어도 밖의 사이트를 가리킨다 (REQ E-2)."""
    pat = re.compile(r"홈랩|homelab", re.I)
    bad = ["%s: %s" % (rel, m.group(0)) for rel, text in _texts() for m in pat.finditer(text)]
    assert not bad, bad
