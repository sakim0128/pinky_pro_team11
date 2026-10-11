# -*- coding: utf-8 -*-
"""앱이 보내는 곁표 헤더를 **우리가 버리지 않는다**.

🔴 왜 생겼나 (2026-09-19, 카메라앱 세션이 양쪽 소스를 대조해 찾았다):

    앱이 보내는데 우리가 버림 : X-Camera-Rotation · X-Camera-Zoom
    우리가 기다리는데 앱이 안 냄 : X-Camera-Pitch · X-Camera-Roll · X-Process-Blur

   뒤의 셋은 알고 있던 것(O-10)이고, **앞의 둘이 새 결함**이다. 회전은 사람이 버튼으로
   정하는 값이라 센서를 안 따라가고, 현장 설정이 `rotationDegrees 90` 이다. 그걸 모르는
   소비자는 **x·y 축이 뒤바뀐** 좌표를 만든다. 호모그래피에 직접 걸리는 양이다.

## ⭐⭐ 그리고 이 병은 **두 번째**다

`mjpeg_puller` 의 `SIDECAR_HEADERS` 위에는 2026-09-10 부터 이런 주석이 있었다:

    ⭐ 여기 없는 헤더는 parse_part_headers 가 **버린다.** 상수만 만들고 이 목록에
       안 넣으면 기능이 죽은 채로 유닛테스트는 통과한다 — 실제로 그러했다.

**주석은 통제가 아니었다.** 그리고 이번 것은 그 주석이 막으려던 형태도 아니다 —
우리가 상수를 빠뜨린 게 아니라 **상대가 새 헤더를 보내기 시작했다.** 그건 이 레포
안에서 어떤 시험도 못 본다(앱은 다른 레포다).

그래서 목록을 늘리는 것으로 끝내지 않는다. **모르는 `x-` 헤더를 세어 곁표에 싣는다.**
다음 어긋남은 사람이 두 레포를 손으로 대조할 때가 아니라 **그 자리에서** 보인다.
"""
import mjpeg_puller as MP


def _part(**headers):
    """앱과 같은 모양의 파트 서두를 만든다."""
    lines = [b"--frame", b"Content-Type: image/jpeg"]
    for k, v in headers.items():
        lines.append(("%s: %s" % (k.replace("_", "-"), v)).encode("ascii"))
    return b"\r\n".join(lines) + b"\r\n\r\n"


# ---- 그 결함 자체 ---------------------------------------------------------------

def test_회전을_버리지_않는다():
    """🔴 이 시험이 2026-09-19 이전에는 빨갰다."""
    meta = MP.parse_part_headers(_part(**{"X-Camera-Rotation": "90"}))
    assert meta.get(MP.H_CAMERA_ROTATION) == "90"


def test_줌을_버리지_않는다():
    meta = MP.parse_part_headers(_part(**{"X-Camera-Zoom": "1.00"}))
    assert meta.get(MP.H_CAMERA_ZOOM) == "1.00"


def test_회전과_줌이_곁표에_실린다():
    """파싱만 되고 곁표에 안 실리면 소비자는 여전히 못 본다 — 정의 != 배선."""
    p = MP.MjpegPuller("http://127.0.0.1:9/video", name="unit")
    p._note_sidecar(MP.parse_part_headers(
        _part(**{"X-Camera-Rotation": "90", "X-Camera-Zoom": "2.50"})), 1000.0)
    sc = p.sidecar()
    assert sc["cameraRotation"] == 90.0
    assert sc["cameraZoom"] == 2.5


def test_안_오면_None_이지_0_이_아니다():
    """⭐ 회전 0도와 '회전을 모른다' 는 다른 상태다."""
    p = MP.MjpegPuller("http://127.0.0.1:9/video", name="unit")
    sc = p.sidecar()
    assert sc["cameraRotation"] is None
    assert sc["cameraZoom"] is None


# ---- 다음 어긋남을 드러내는 장치 ---------------------------------------------------

def test_모르는_x_헤더를_이름으로_기록한다():
    """🔴 이게 이 파일의 본체다. 목록을 늘리는 것만으로는 다음 번을 못 막는다."""
    seen = set()
    MP.parse_part_headers(_part(**{"X-Brand-New-Thing": "42"}), seen)
    assert "x-brand-new-thing" in seen


def test_값은_담지_않는다():
    """이름만 싣는다 — 곁표는 화면·API 로 나가므로 내용이 새면 안 된다."""
    seen = set()
    MP.parse_part_headers(_part(**{"X-Secret-Thing": "sensitive-value"}), seen)
    assert seen == {"x-secret-thing"}
    assert not any("sensitive" in x for x in seen)


def test_파트의_정상_구성요소는_모르는_헤더가_아니다():
    """`Content-Type`·`Content-Length` 는 파트의 일부다. 그걸 세면 늘 시끄럽다."""
    seen = set()
    MP.parse_part_headers(
        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 100\r\n\r\n", seen)
    assert seen == set()


def test_상한을_넘기면_더_안_담는다():
    """무한정 쌓이면 상대가 헤더를 흘릴 때 우리 메모리가 자란다."""
    seen = set()
    for i in range(MP.UNKNOWN_HEADER_CAP + 8):
        MP.parse_part_headers(_part(**{"X-Junk-%d" % i: "1"}), seen)
    assert len(seen) == MP.UNKNOWN_HEADER_CAP


def test_모르는_헤더가_곁표에_실린다():
    p = MP.MjpegPuller("http://127.0.0.1:9/video", name="unit")
    p._unknown_headers.add("x-something-new")
    assert p.sidecar()["unknownHeaders"] == ["x-something-new"]


def test_아무것도_모르는_게_없으면_빈_목록이다():
    """⭐ 정상은 **빈 목록**이지 None 이 아니다 — '안 재고 있다' 와 구분된다."""
    p = MP.MjpegPuller("http://127.0.0.1:9/video", name="unit")
    assert p.sidecar()["unknownHeaders"] == []


# ---- 대조군: 목록이 실제로 무엇을 덮는가 ---------------------------------------------

def test_앱이_보내는_것으로_확인된_헤더를_전부_덮는다():
    """🔴🔴 **박아 둔 기대값 대조.**

    2026-09-19 에 **앱 레포**(이 레포가 아니다 — 앱은 별도 저장소다)의 운영 브랜치에서
    `git grep -oE '"X-[A-Za-z-]+'` 로 전수한 목록이다.
    ⚠️ 그 레포 이름은 여기 안 적는다 — 공개 범위 밖이고 `scripts/check_publishable.py`
    가 막는다. 실제로 2026-09-19 에 내가 적었다가 그 검사에 걸렸다. 앱이 헤더를 더 보내기
    시작하면 이 시험이 아니라 `unknownHeaders` 가 현장에서 먼저 말해 준다 —
    이 목록은 **그때 무엇이 이미 알려져 있었는지**를 고정한다.
    """
    앱이_보내는_것 = {
        "x-camera-lens", "x-camera-rotation", "x-camera-zoom",
        "x-capture-clock", "x-encode-ms", "x-encode-rules",
        "x-frame-seq", "x-process-ms", "x-process-rules",
        "x-publisher-session",
    }
    빠진_것 = 앱이_보내는_것 - set(MP.SIDECAR_HEADERS)
    assert not 빠진_것, (
        "앱이 보내는데 화이트리스트에 없다 — parse_part_headers 가 버린다: %s"
        % sorted(빠진_것)
    )


def test_아직_안_오는_것도_미리_등재해_둔다():
    """①정적설정 → ②생산자 순서. 뒤집으면 앱을 재설치하고도 값이 여기서 버려진다."""
    for h in (MP.H_CAPTURE_BASIS, MP.H_PUBLISH_CLOCK, MP.H_CLOCK_SUSPECT):
        assert h in MP.SIDECAR_HEADERS


def test_목록에_없는_상수를_만들어_두지_않는다():
    """2026-09-10 X-Process-Blur 사고의 반대 방향 — 상수만 있고 목록에 없는 것."""
    상수들 = {v for k, v in vars(MP).items()
              if k.startswith("H_") and isinstance(v, str)}
    빠진_것 = 상수들 - set(MP.SIDECAR_HEADERS)
    assert not 빠진_것, "상수는 있는데 SIDECAR_HEADERS 에 없다: %s" % sorted(빠진_것)
