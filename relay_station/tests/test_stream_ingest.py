# -*- coding: utf-8 -*-
"""stream_ingest 특성화 테스트 — T2/T3/T4 리팩터의 안전망.

여기 적힌 것은 "이래야 한다"가 아니라 **"지금 이렇다"** 이다.
소스 레지스트리 통합(T2)·MJPEG 루프 통합(T3)·lazy decode(T4)는 이 파일을 초록으로
유지한 채 진행해야 한다. 의도적으로 동작을 바꾸는 경우에만 이 파일을 같이 고친다.

수신 경로 (ingest_direct_frame):

    bytes ──┬─ b"{" 로 시작?  ─yes─> JSON 파싱 → image|frame|data → base64 디코드
            │                        (data URL 이면 "," 뒤만)
            └─no──────────────────┐
                                  ▼
            b"\\xff\\xd8\\xff" 포함? ─yes─> 매직~EOI 구간만 잘라냄 (multipart 대응)
                                  ▼
                          cv2.imdecode 성공?
                            ├─yes─> _latest_frame/_latest_jpeg/is_connected/current_url 갱신, True
                            └─no──> 아무것도 안 바꾸고 False
"""
import time

import pytest

from fixtures.frames import (make_corrupt_bytes, make_jpeg, wrap_json_base64,
                             wrap_multipart)

JPEG_SOI = b"\xff\xd8\xff"

# 캡처 루프가 직접 업로드에 양보하는 창(초). stream_ingest 의 206·240 행이 이 값을 쓴다.
DIRECT_UPLOAD_YIELD_S = 3.0


def test_생성자는_플레이스홀더를_이미_들고_있다(ingest):
    """프레임이 하나도 안 왔어도 /video_feed 가 낼 것이 있어야 한다."""
    jpeg_bytes, _stamp, connected = ingest.get_latest_jpeg()
    assert jpeg_bytes is not None and jpeg_bytes.startswith(JPEG_SOI)
    assert connected is False, "아무것도 안 받았는데 connected 면 지표가 거짓말한다"


def test_KNOWN_GAP_플레이스홀더가_last_stamp_를_민다(ingest):
    """플레이스홀더를 다시 그릴 때마다 _latest_stamp 가 현재 시각으로 올라간다.

    그래서 /api/status.last_stamp 는 **실프레임이 하나도 없어도** 계속 최신으로 보인다.
    2026-09-08 '완벽 수신 LIVE' 오탐의 두 번째 원인이 이것이다(첫째는 current_url).
    last_stamp 를 '마지막 실프레임 수신 시각'으로 바꿀 때 이 테스트를 뒤집는다.
    """
    first = ingest.get_latest_jpeg()[1]
    assert first > 0.0, "실프레임이 없는데도 시각이 찍혀 있다"
    time.sleep(0.01)
    ingest._update_placeholder("다른 안내 문구")
    second = ingest.get_latest_jpeg()[1]
    assert second > first, "지금은 플레이스홀더가 시각을 민다(알려진 갭)"


def test_raw_jpeg_를_받으면_그_바이트_그대로_보관한다(ingest, jpeg):
    assert ingest.ingest_direct_frame(jpeg) is True
    stored, stamp, connected = ingest.get_latest_jpeg()
    assert stored == jpeg, "재인코딩 없이 원본 바이트를 보관해야 한다(대역·화질 손실 방지)"
    assert connected is True
    assert stamp > 0.0


def test_json_base64_봉투를_푼다(ingest, jpeg):
    assert ingest.ingest_direct_frame(wrap_json_base64(jpeg)) is True
    stored, _, connected = ingest.get_latest_jpeg()
    assert stored == jpeg
    assert connected is True


def test_data_url_형식도_푼다(ingest, jpeg):
    assert ingest.ingest_direct_frame(wrap_json_base64(jpeg, data_url=True)) is True
    assert ingest.get_latest_jpeg()[0] == jpeg


@pytest.mark.parametrize("key", ["image", "frame", "data"])
def test_json_키_세_가지를_모두_받는다(ingest, jpeg, key):
    assert ingest.ingest_direct_frame(wrap_json_base64(jpeg, key=key)) is True
    assert ingest.get_latest_jpeg()[0] == jpeg


def test_multipart_에서_jpeg_구간만_잘라낸다(ingest, jpeg):
    assert ingest.ingest_direct_frame(wrap_multipart(jpeg)) is True
    stored = ingest.get_latest_jpeg()[0]
    assert stored.startswith(JPEG_SOI)
    assert stored == jpeg, "봉투를 벗기고 원본과 같은 바이트가 남아야 한다"


def test_깨진_바이트는_False_이고_상태를_안_건드린다(ingest, jpeg):
    ingest.ingest_direct_frame(jpeg)
    before_jpeg, before_stamp, _ = ingest.get_latest_jpeg()

    assert ingest.ingest_direct_frame(make_corrupt_bytes()) is False

    after_jpeg, after_stamp, connected = ingest.get_latest_jpeg()
    assert after_jpeg == before_jpeg, "실패한 업로드가 마지막 좋은 프레임을 덮으면 안 된다"
    assert after_stamp == before_stamp
    assert connected is True


def test_빈_바이트도_False(ingest):
    assert ingest.ingest_direct_frame(b"") is False


def test_업로드가_캡처루프_양보_시계를_민다(ingest, jpeg):
    """이 값이 안 움직이면 pull 루프가 push 프레임을 덮어쓴다(경합 회귀 감지)."""
    before = ingest._last_direct_upload_stamp
    ingest.ingest_direct_frame(jpeg)
    after = ingest._last_direct_upload_stamp
    assert after > before
    assert time.time() - after < DIRECT_UPLOAD_YIELD_S, \
        "방금 넣은 프레임이 이미 양보 창 밖이면 캡처 루프가 즉시 끼어든다"


def test_업로드_성공이_current_url_을_송출_표시로_바꾼다(ingest, jpeg):
    before = ingest.current_url
    ingest.ingest_direct_frame(jpeg)
    assert ingest.current_url != before
    assert "앱" in ingest.current_url or "송출" in ingest.current_url


def test_KNOWN_GAP_push가_끊겨도_current_url_이_안_되돌아온다(ingest, jpeg):
    """현재 동작을 고정해 둔다 — 바람직해서가 아니라 **아직 안 고쳤기 때문**이다.

    2026-09-08 에 이것 때문에 "완벽 수신 LIVE" 오탐이 났다. push 가 멈춰도
    current_url 이 송출 표시로 남아 있어서다. 고칠 때 이 테스트를 뒤집으면 된다.
    """
    ingest.ingest_direct_frame(jpeg)
    live_url = ingest.current_url
    ingest._last_direct_upload_stamp = 0.0          # 오래 전에 끊긴 상황을 흉내
    assert ingest.current_url == live_url, "지금은 되돌아오지 않는다(알려진 갭)"


def test_update_target_url_이_두_필드를_같이_바꾼다(ingest):
    ingest.update_target_url("http://127.0.0.1:9/other")
    assert ingest.primary_url == "http://127.0.0.1:9/other"
    assert ingest.current_url == "http://127.0.0.1:9/other"


def test_큰_프레임도_받는다(ingest):
    big = make_jpeg(width=1280, height=720, quality=95)
    assert ingest.ingest_direct_frame(big) is True
    assert ingest.get_latest_jpeg()[0] == big
