# -*- coding: utf-8 -*-
"""U-1 서버 플래그 (HANDOFF_20260928 §3-3): `/api/status` 최상위의 `view_only`(V2 가 응답을 state.gateway 로 두고 읽는 자리) — 이 요청의 출처가 움직이는 조작을 낼 수
없는 곳(LOCAL_CONTROL_IPS 밖)이면 true. 18081 원격 보기 경로는 socat 이 127.0.0.2 로 붙어 들어온다(launch_master_gateway.sh),
그래서 포트·쿼리 없이도 화면이 보기 전용을 안다. 화면이 숨기는 조작 = 서버가 403 으로 거절할 조작(같은 판정 집합).
"""
import json
import os
import sys
import types
from unittest.mock import MagicMock

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)


@pytest.fixture
def g(monkeypatch):
    import gateway_web_server as gws
    ingest = MagicMock()
    ingest.get_latest_jpeg.return_value = (None, 0.0, False)
    ingest.primary_url = ingest.current_url = ""
    control = MagicMock()
    control.map_source = "synthetic"
    control.map_w, control.map_h, control.resolution, control.origin = 0, 0, 0.0, [0.0, 0.0, 0.0]
    control.get_robot_data.return_value = {}
    monkeypatch.setattr(gws, "GLOBAL_INGEST", ingest)
    monkeypatch.setattr(gws, "GLOBAL_CONTROL", control)
    cpu = MagicMock()
    cpu.value.return_value = 15.0
    monkeypatch.setattr(gws, "GLOBAL_CPU", cpu)
    monkeypatch.setattr(gws, "GLOBAL_NETWORK_MONITOR", None)
    monkeypatch.setattr(gws, "GLOBAL_ROBOT_SUB_NODE", None)
    monkeypatch.setattr(gws, "GLOBAL_FLEET_COORDINATOR", None)
    return gws


def _status(g, ip):
    h = g.GatewayRequestHandler.__new__(g.GatewayRequestHandler)
    h.path = "/api/status"
    h.headers = {}
    h.client_address = (ip, 5000)
    h.server = types.SimpleNamespace(server_port=8889)
    out = {}

    def _send_json(body_bytes, code=200):
        out["code"], out["body"] = code, json.loads(body_bytes.decode("utf-8"))
    h._send_json = _send_json
    h.do_GET()
    return out["code"], out["body"]


@pytest.mark.parametrize("ip,expect", [
    ("127.0.0.1", False),          # 현장 노트북 자신(:8889)
    ("127.0.0.2", True),           # :18081 socat 이 붙는 출처 — 보기 전용
    ("192.0.2.50", True),          # 원격
], ids=["local", "socat-18081", "remote"])
def test_U1_api_status_의_view_only_는_요청_출처가_제어_허용_밖일_때만_true(g, ip, expect):
    code, body = _status(g, ip)
    assert code == 200 and body["view_only"] is expect


def test_U1_view_only_판정은_서버가_움직이는_조작을_거절하는_집합과_같다(g):
    """화면이 숨기는 것과 서버가 403 으로 막는 것이 같은 집합이어야 '보기 전용' 이 거짓말이 아니다."""
    for ip in g.LOCAL_CONTROL_IPS:
        if ip == "localhost":
            continue
        assert _status(g, ip)[1]["view_only"] is False, ip
    assert "127.0.0.2" not in g.LOCAL_CONTROL_IPS                 # socat 출처는 제어 허용에 없다(OPS-1)


