# -*- coding: utf-8 -*-
"""관제 화면 렌더러가 **살아서 프레임을 낸다**.

🔴 왜 이 시험이 생겼나: 2026-09-19 에 로봇 좌표 기본값을 `0.0` → `None` 으로 바꾸면서
   마커 루프에만 가드를 넣고 우측 HUD 의 `f"{r1['x']:+.2f}"` 를 빠뜨렸다. `None` 은
   `+.2f` 로 포맷되지 않아 **렌더 스레드가 기동 즉시 죽었고**, `/control_feed` 는
   프레임을 한 장도 못 냈다. 그런데 화면은 그 사실을 말하지 않았다 — 관제사 눈에는
   그냥 검은 칸이다.

⭐ 이 레포에 `ControlScreenRenderer` 를 보는 시험이 **0 건**이었다. 계약을 바꿀 때
   소비자를 전수로 세지 않으면 이런 자리가 남는다.
"""
import time

import pytest

cv2 = pytest.importorskip("cv2")
import control_renderer


def _renderer(tmp_path):
    """맵 없는 경로로 띄운다 — 합성 지도 + 좌표 전부 미관측이 **최악**의 조합이다."""
    r = control_renderer.ControlScreenRenderer(
        map_yaml_path=str(tmp_path / "없는파일.yaml"))
    return r


def test_맵이_없어도_스레드가_살아_있고_프레임을_낸다(tmp_path):
    r = _renderer(tmp_path)
    try:
        jpeg = r.get_latest_jpeg()
        assert r.thread is None, "상시 렌더 스레드가 없어야 한다 (CPU 회수)"
        assert r.is_running is False
        assert jpeg, "프레임이 준비되어 있어야 한다"
        assert jpeg[:2] == b"\xff\xd8", "JPEG 이 아니다"
    finally:
        r.is_running = False


def test_좌표를_한_번도_못_받아도_죽지_않는다(tmp_path):
    """🔴 뮤테이션 표적 — HUD 가드를 지우면 여기서 빨개진다."""
    r = _renderer(tmp_path)
    try:
        for rid in ("pinky1", "pinky2", "gazebo_sim"):
            assert r.robots[rid]["x"] is None      # 지어낸 기본값이 없다
        assert r.thread is None
        assert r.is_running is False
        assert r.get_latest_jpeg()
    finally:
        r.is_running = False


def test_좌표를_받으면_보고에_실리고_못_받으면_키가_없다(tmp_path):
    r = _renderer(tmp_path)
    try:
        before = r.get_robot_data()["pinky1"]
        assert before["observed"] is False
        assert "x" not in before and "y" not in before and "yaw" not in before

        r.update_robot_pose("pinky1", 0.4, -0.2, 1.0)
        after = r.get_robot_data()["pinky1"]
        assert after["observed"] is True
        assert after["x"] == 0.4 and after["y"] == -0.2 and after["yaw"] == 1.0

        # 안 받은 로봇은 여전히 키가 없다 — 하나 받았다고 전부 관측이 되지 않는다
        assert r.get_robot_data()["pinky2"]["observed"] is False
        assert "x" not in r.get_robot_data()["pinky2"]
    finally:
        r.is_running = False


def test_맵을_못_읽으면_합성이라고_말한다(tmp_path):
    r = _renderer(tmp_path)
    try:
        assert r.map_source == "synthetic"
    finally:
        r.is_running = False


def test_좌표를_받은_뒤에도_스레드가_살아_있다(tmp_path):
    """숫자가 들어온 경로도 밟아 본다 — 좌표 갱신 후에도 안전하게 유지되는지."""
    r = _renderer(tmp_path)
    try:
        r.update_robot_pose("pinky1", 1.0, 2.0, 0.5)
        r.update_robot_pose("pinky2", -1.0, 0.0, -0.5)
        assert r.thread is None
        assert r.get_latest_jpeg()
    finally:
        r.is_running = False
