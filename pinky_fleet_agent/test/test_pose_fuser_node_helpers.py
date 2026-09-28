"""pose_fuser_node 의 ROS-free 보조 함수."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import importlib.util  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load_marker_to_base():
    """rclpy 없이 함수만 꺼낸다 (모듈 import 는 rclpy 를 요구한다)."""
    src = open(os.path.join(HERE, 'pinky_fleet_agent', 'pose_fuser_node.py'), encoding='utf-8').read()
    start = src.index('def marker_to_base(')
    end = src.index('\nclass PoseFuserNode')
    ns = {'math': math}
    exec(src[start:end], ns)
    return ns['marker_to_base']


def test_marker_to_base_offsets():
    f = _load_marker_to_base()
    assert f(1.0, 2.0, 0.5) == (1.0, 2.0, 0.5)
    x, y, yaw = f(1.0, 2.0, math.pi / 2, yaw_offset=math.pi / 2)        # 마커가 90° 돌아 붙음
    assert abs(yaw) < 1e-9 and (x, y) == (1.0, 2.0)
    x, y, yaw = f(1.0, 2.0, 0.0, offset_x=0.05)                         # 마커가 5 cm 앞에
    assert abs(x - 0.95) < 1e-9 and abs(y - 2.0) < 1e-9
    x, y, yaw = f(1.0, 2.0, math.pi / 2, offset_x=0.05)
    assert abs(x - 1.0) < 1e-9 and abs(y - 1.95) < 1e-9
