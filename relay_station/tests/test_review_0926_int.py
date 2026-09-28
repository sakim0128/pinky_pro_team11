# -*- coding: utf-8 -*-
"""검수 R-script-1(게이트웨이 쪽): 목표 API 가 깨진 본문을 {} 로 받아 (0,0,0) 목표를 냈고 NaN·Infinity 도 받았다."""
import os
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from relay_station.fleet.fleet_coordinator import FleetRobotContext  # noqa: E402
from test_d7_robot_stop import _post, gw  # noqa: E402,F401  (gw 는 픽스처)



# ---- 목표 본문 ---------------------------------------------------------------------------------

@pytest.mark.parametrize("body", [
    b"not json", b"[1, 2]", b'"x"', b'"xy"', b"{}", b'{"y": 1.0}', b'{"x": 1.0}',
    b'{"x": NaN, "y": 0}', b'{"x": 0, "y": Infinity}', b'{"x": 1e999, "y": 0}',
    b'{"x": "abc", "y": 0}', b'{"x": true, "y": 0}', b'{"x": null, "y": 0}',
    b'{"x": 1, "y": 0, "yaw": NaN}', b'{"x": 1, "y": 0, "yaw": "north"}',
], ids=["not-json", "list", "string", "string-xy", "empty", "no-x", "no-y", "nan", "inf", "overflow",
        "text", "bool", "null", "yaw-nan", "yaw-text"])
@pytest.mark.parametrize("path", ["/api/robot1/goal", "/api/goal"])
def test_목표는_온전한_본문일_때만_보낸다__모자라거나_유한하지_않으면_400(gw, path, body):
    g, node, coord = gw
    coord.robots = {"pinky1": FleetRobotContext("pinky1", 10)}
    code, res = _post(g, path, ip="127.0.0.1", body=body)
    assert code == 400 and res["reason"] == "BAD_GOAL" and res["dispatched"] is False
    node.send_goal.assert_not_called()


def test_온전한_목표는_그대로_보낸다__yaw_는_없으면_0(gw):
    g, node, coord = gw
    coord.robots = {"pinky1": FleetRobotContext("pinky1", 10)}
    code, _ = _post(g, "/api/robot1/goal", ip="127.0.0.1", body=b'{"x": "1.5", "y": -0.25}')
    assert code == 200
    node.send_goal.assert_called_once_with(1.5, -0.25, 0.0)
    node.send_goal.reset_mock()
    code, _ = _post(g, "/api/goal", ip="127.0.0.1", body=b'{"x": 0, "y": 0, "yaw": 1.2}')
    assert code == 200
    node.send_goal.assert_called_once_with(0.0, 0.0, 1.2)


def test_목표_본문_검사는_S1_래치_검사보다_늦다__비상정지_중이면_본문과_무관하게_409(gw):
    g, node, coord = gw
    coord.robots = {"pinky1": FleetRobotContext("pinky1", 10)}
    coord.estop_latched = True
    code, res = _post(g, "/api/robot1/goal", ip="127.0.0.1", body=b"not json")
    assert code == 409
    node.send_goal.assert_not_called()
