# -*- coding: utf-8 -*-
"""T-8 health contract fields (planned for a Relay ops view; no route consumes them yet) and POST outcome capture."""

import http.server
import threading

import cv2
import numpy as np
import pytest

from tablet.vision.runtime.entrypoint import build_vision_health, load_configs
from tablet.vision.runtime.http_transport import HttpVisionTransport
from tablet.vision.runtime.tablet_vision_service import TabletVisionService
from tablet.vision.vision_core.pose_fix import PoseFix
from tablet.vision.vision_core.synthetic_camera import render_overhead
import os

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")

CONTRACT = {"input_fps", "frames_total", "markers_seen", "reference_ok", "homography_ok",
            "posefix_rate", "last_post", "config_status", "mode", "git_sha"}


def _service():
    cfg, _, _, statuses = load_configs(CONFIG_DIR)
    return cfg, TabletVisionService(overhead_cfg=cfg, zone_configs=[]), statuses


def test_contract_fields_after_frames():
    cfg, svc, statuses = _service()
    for _ in range(3):
        _, img = render_overhead(cfg, {"pinky1": (0.30, 0.30, 0.0), "pinky2": (0.30, 0.95, 0.0)})
        svc.process_overhead_frame(img)
    vh = build_vision_health(svc.get_health(), None, statuses, "mock", "abc1234")
    assert set(vh) == CONTRACT
    assert vh["frames_total"] == 3
    assert vh["markers_seen"] == [30, 31, 40, 41, 42, 43]
    assert vh["reference_ok"] is True and vh["homography_ok"] is True
    assert vh["posefix_rate"] > 0
    assert vh["last_post"] is None                 # egress off is not a failed POST
    assert vh["config_status"] == "PENDING"        # SoT config is still FIELD_CONFIG_PENDING
    assert vh["mode"] == "mock"


def test_reference_not_ok_when_references_hidden():
    """Positive control: only robot markers visible -> reference_ok/homography_ok False, no PoseFix."""
    cfg, svc, statuses = _service()
    img = np.full((720, 1280, 3), 110, np.uint8)
    img[300:420, 600:720] = 255
    mk = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 30, 80)
    img[320:400, 620:700] = mk[:, :, None]
    svc.process_overhead_frame(img)
    vh = build_vision_health(svc.get_health(), None, statuses, "mock", "x")
    assert vh["markers_seen"] == [30]
    assert vh["reference_ok"] is False and vh["homography_ok"] is False
    assert vh["posefix_rate"] == 0


class _Handler(http.server.BaseHTTPRequestHandler):
    code = 200

    def do_POST(self):
        self.rfile.read(int(self.headers.get("Content-Length", 0)))
        self.send_response(type(self).code)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b"{}")

    def log_message(self, *a):
        pass


@pytest.fixture
def relay():
    srv = http.server.HTTPServer(("127.0.0.1", 0), _Handler)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    yield srv
    srv.shutdown()


def _fix():
    return PoseFix(robot_name="pinky1", x=0.3, y=0.3, yaw=0.0, seq=1)


def test_last_post_records_401_then_200(relay):
    url = f"http://127.0.0.1:{relay.server_address[1]}"
    tr = HttpVisionTransport(url, async_pose=False, api_key="test-only")
    assert tr.post_stats()["status"] is None
    _Handler.code = 401
    assert tr.send_pose_fix(_fix(), sync=True) is False
    st = tr.post_stats()
    assert st["status"] == 401 and st["fail"] == 1 and st["age_s"] is not None
    _Handler.code = 200
    assert tr.send_pose_fix(_fix(), sync=True) is True
    st = tr.post_stats()
    assert st["status"] == 200 and st["ok"] == 1
    tr.close()


def test_last_post_records_unreachable():
    tr = HttpVisionTransport("http://127.0.0.1:9", async_pose=False, timeout=0.5)
    tr.send_pose_fix(_fix(), sync=True)
    st = tr.post_stats()
    assert isinstance(st["status"], str) and st["fail"] == 1
    tr.close()
