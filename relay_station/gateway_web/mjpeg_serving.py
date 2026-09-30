# -*- coding: utf-8 -*-
"""MJPEG 을 클라이언트로 내보내는 두 가지: 프레임 한 장 쓰기, 그리고 보낼지 말지.

게이트웨이 본체에서 갈라 나왔다. 본체는 module-level 에서 rclpy·cv_bridge 를 import 하므로
ROS 없는 곳에서 import 되지 않는다 — 즉 **본체 안에 있으면 유닛테스트가 닿지 못한다.**
이 판정은 실제로 한 번 틀렸으니(아래) 닿는 자리에 두어야 한다.

## 왜 값 비교인가 — 동일성으로 한 번 짰다가 되돌렸다

2026-09-12, 중복 53~62% 를 잡겠다고 `jpeg is not prev` 로 걸렀다(커밋 5a252e4).
되돌렸다(7ef5c29). 그때의 반성이 이 모듈의 존재 이유다.

객체 동일성은 **"제공자가 프레임마다 새 객체를 준다"** 는 전제 위에 선다.
그 전제는 제공자마다 다르고 **우리가 소유하지 않는다** — 제공자는 버퍼를 재사용해도
되고, 그렇게 바뀌어도 우리 쪽 시험은 하나도 안 빨개진다. 전제가 깨지면 새 프레임을
'같다'고 오판해 스트림이 keepalive 속도까지 조용히 죽는다.

⭐ **값 비교는 그 오판이 원리적으로 불가능하다.** 값이 같으면 화면에 보이는 그림도
   같으므로, 값 비교로 버린 프레임은 **사람이 볼 수 있었던 것이 아니다.**
   동일성은 '빠르지만 틀릴 수 있고', 값은 '틀릴 수 없고 충분히 빠르다'.

⭐ "178KB 를 매 회차 훑으니 비싸다" 는 걱정은 **재 보니 사실이 아니었다.**
   실측 2026-09-12 (이 기계, Python 3.12.3):

       값 같고 객체 다름   91KB  3.2us/회 ·  178KB  4.3us/회   -> 30Hz 에서 CPU 0.013%
       같은 객체           178KB  0.1us/회   (CPython 이 동일성으로 단축한다)

   즉 제공자가 같은 객체를 주면 **동일성의 빠름을 그대로 얻고**, 다른 객체를 주면
   memcmp 가 ~30GB/s 로 훑는다. 동일성의 취약함만 안 받는다.

⚠️ 완전히 안 보내면 안 된다. 소스가 멈췄을 때 클라이언트가 연결을 죽은 것으로 볼 수
   있으므로 새 프레임이 없어도 KEEPALIVE 주기로 한 번은 다시 보낸다(0.5 fps).
   ⭐ 그 0.5 fps 는 고장이 아니라 **정직한 신호**다. 예전 루프는 얼어붙은 소스를 30Hz 로
      재전송해 살아 있는 것처럼 보이게 했다 — 2026-09-12 실측으로 `src` 없는
      `/video_feed` 가 전송 29.94 fps 인데 **고유 0.08 fps** 였다. 30Hz 재전송이
      그 사실을 덮고 있었다.
"""

# 새 프레임이 없어도 이 주기로는 한 번 다시 보낸다(초).
STREAM_KEEPALIVE_SEC = 2.0


def write_mjpeg_frame(wfile, jpeg):
    """multipart/x-mixed-replace 프레임 한 장.

    ⚠️ 구분자는 **CRLF** 다. LF 로 쓰면 일부 클라이언트가 파트를 못 자른다.
    """
    wfile.write(b'--frame\r\n')
    wfile.write(b'Content-Type: image/jpeg\r\n')
    wfile.write(('Content-Length: %d\r\n\r\n' % len(jpeg)).encode('utf-8'))
    wfile.write(jpeg)
    wfile.write(b'\r\n')


def should_send_frame(jpeg, prev, last_sent, now):
    """보낼 것이 있는가. **값이 같은 프레임은 안 보낸다** (keepalive 주기는 예외).

    jpeg      이번에 제공자가 준 바이트 (없으면 None)
    prev      마지막으로 보낸 **원본** 바이트 (보정 전. 보정본을 넣으면 비교가 깨진다)
    last_sent 마지막으로 보낸 시각 (time.monotonic)
    now       지금 (time.monotonic)
    """
    if jpeg is None:
        return False
    if prev is None or jpeg != prev:
        return True
    return (now - last_sent) >= STREAM_KEEPALIVE_SEC
