# -*- coding: utf-8 -*-
"""테스트 공통 설정.

게이트웨이 본체(gateway_web_server.py)는 module-level 에서 rclpy·cv_bridge 를 import 하는데
그 둘은 `source /opt/ros/jazzy/setup.bash` 없이는 없다. 그래서 여기서는 **import 하지 않는다.**
테스트 표적은 cv2·numpy 만 쓰는 stream_ingest 와, HTTP 경계다.

    gateway_web_server.py  ── rclpy, cv_bridge 필요 ──> 유닛테스트 대상 아님 (HTTP 로만 시험)
    stream_ingest.py       ── cv2, numpy 만        ──> 유닛테스트 대상 ✔
    mcv_observer/ (T6~)    ── cv2, numpy 만        ──> 유닛테스트 대상 ✔
"""
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_RELAY = os.path.dirname(_HERE)                       # relay_station/
_GATEWAY_WEB = os.path.join(_RELAY, "gateway_web")

for _p in (_RELAY, _GATEWAY_WEB, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


# ---- 팀11 레포 구조 --------------------------------------------------------------------------
# 에이전트는 pinky_fleet_agent 패키지, 도로망(road_graph)은 pinky_lane_station 패키지다 — 둘 다 소스에서 바로
# import 한다(실물 중계 PC 에서는 install/setup.bash 가 같은 이름을 준다). 비전 API 키는 코드에 기본값이 없으므로
# 시험에서만 넣는다(fail-closed 규칙은 test_team11_export.py 가 따로 잰다).
_T11_REPO = os.path.dirname(_RELAY)
for _p in (os.path.join(_T11_REPO, "pinky_fleet_agent", "test"), os.path.join(_T11_REPO, "pinky_fleet_agent"),
           os.path.join(_T11_REPO, "pinky_lane_station")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
os.environ.setdefault("RELAY_VISION_API_KEY", "test-only-vision-key")


@pytest.fixture
def ingest():
    """스레드를 띄우지 않은 TabletStreamIngest.

    생성자는 캡처 루프를 시작하지 않는다(is_running=False). start() 를 부르지 않는 한
    네트워크도 타지 않으므로 유닛테스트에서 안전하다. 도달 불가 URL 을 주어
    혹시라도 pull 이 돌면 즉시 실패하도록 한다.
    """
    import stream_ingest
    obj = stream_ingest.TabletStreamIngest("http://127.0.0.1:9/video", None)
    assert obj.is_running is False, "생성자가 캡처 루프를 시작하면 이 테스트 전제가 깨진다"
    yield obj
    if getattr(obj, "is_running", False):
        obj.stop()


@pytest.fixture
def jpeg():
    from fixtures.frames import make_jpeg
    return make_jpeg()


@pytest.fixture
def aruco():
    """(jpeg_bytes, truth) — 마커를 아는 위치에 그린 프레임."""
    from fixtures.frames import make_aruco_frame
    return make_aruco_frame()


@pytest.fixture(autouse=True)
def _호스트_카메라_환경변수_격리(monkeypatch):
    """`MCV_HOST_CAMERA_URL` 이 설정된 곳에서 테스트가 돌면 pull 소스가 하나 더 생긴다.

    ⭐ 도커 복제본 안에서 게이트를 돌렸더니 기존 시험 둘이 깨졌다 —
       설정 파일만 보는 시험인데 **환경변수가 숨은 입력**으로 끼어들었기 때문이다.
       그 시험들이 옳다. 여기서 걷어내고, 그 변수를 시험하는 쪽만 직접 켠다.
    """
    monkeypatch.delenv("MCV_HOST_CAMERA_URL", raising=False)
