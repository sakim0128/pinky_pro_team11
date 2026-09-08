#!/usr/bin/env python3
"""빌드(colcon build) 전에 소스 트리에서 바로 브리지 YAML을 만들기 위한 얇은 래퍼.

빌드 후에는 `ros2 run pinky_fleet make_bridge_yaml` 을 쓰면 된다.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinky_fleet.make_bridge_yaml import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
