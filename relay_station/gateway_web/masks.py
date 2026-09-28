# -*- coding: utf-8 -*-
"""MCV-2M — 운영자가 선언하는 **관측 제외 영역**(삼각대·베젤·책상 케이블).

카메라는 아레나만 보지 않는다. 삼각대 다리, 모니터 베젤, 책상 위 케이블이 화면에
같이 들어온다. 그것들은 **아레나 바닥이 아니다.** 그런데 영상→월드는 그걸 모르고
어두운 화소를 벽으로 읽는다 — 있지도 않은 장애물이 평면도에 생기고, 로봇은 없는 것을
피해 돌아간다. 실제로 태블릿 시점이 그랬다(모니터 베젤·HDMI 케이블·책상 배선).

## ⭐ 검열 상자와 **같은 기계, 다른 주인**

    검열 상자   가공 단이 **선언**한다. 사생활. 우리는 못 바꾼다. 헤더로 온다.
    마스크      운영자가 선언한다. "여긴 바닥이 아니다". 여기 저장한다.

둘을 한 개념으로 묶으면 안 된다. 수명도, 주인도, 바뀌는 이유도 다르다.
같은 것은 **결과**뿐이다 — 두 영역 다 "못 본 자리"가 된다.

## ⭐⭐ 가린 자리는 '비었다' 가 아니라 '모른다' 다

이게 이 모듈의 안전 성질이다. 삼각대 다리를 가리고 그 칸을 **비었다**로 적으면
로봇이 삼각대로 들어간다 — 충돌 방지를 하려다 충돌을 만든다. 가린 곳은 모른다.

## ⭐ 정착한 모서리를 덮는 마스크는 거부한다

핀은 아레나 모서리다. 그 자리를 "안 본다"고 선언하면 그 정합은 **못 보는 점으로 푼
행렬**이 된다. 거부 사유를 사람에게 말한다 — 카메라를 옮기든 마스크를 줄이든
사람이 정해야 할 일이지, 우리가 조용히 한쪽을 고를 일이 아니다.

## ⭐ 흔들림 조각도 마스크를 피한다

삼각대는 카메라와 **한 몸**이다. 거기 뜬 조각은 카메라가 움직여도 안 움직인다.
그것만이면 별 문제가 없어 보이지만 — 누가 책상 케이블을 건드리면 그 조각의 정합도가
떨어지고, `classify` 는 MATCH_LOST 를 SHIFTED 보다 **먼저** 본다. 즉 흔들리는
케이블 하나가 **진짜 흔들림을 가린다.** 그래서 마스크 안에는 조각을 안 뜬다.
"""
import io
import json
import os

# 프레임 비율. 이보다 작은 변은 실수로 찍은 점이다 — 1280px 에서 12px.
MIN_SIDE = 0.01

# 이 이상 덮으면 볼 게 남지 않는다. ⭐ 튜닝한 문턱이 아니라 **다른 뜻이 되는 값**이다 —
# 화면을 다 가리는 것은 마스킹이 아니라 소스를 끄는 것이고, 그건 다른 스위치가 있다.
DEGENERATE_FRACTION = 0.98

# 라벨 최대 길이. 라벨은 사람이 읽는 것이라 길면 화면에서 잘린다.
MAX_WHY = 40


class MaskError(ValueError):
    """사람이 고칠 수 있는 입력 문제."""


def normalize(raw):
    """하나의 마스크를 검사해서 정규형으로. 틀리면 왜 틀렸는지 말한다."""
    if not isinstance(raw, dict):
        raise MaskError("마스크는 객체여야 한다")
    out = {}
    for k in ("x", "y", "w", "h"):
        try:
            out[k] = float(raw[k])
        except (KeyError, TypeError, ValueError):
            raise MaskError("마스크에 %s 가 없거나 숫자가 아니다" % k)
    if out["w"] < MIN_SIDE or out["h"] < MIN_SIDE:
        raise MaskError("너무 작다 (변이 화면의 %.0f%% 보다 커야 한다) — "
                        "잘못 눌린 점일 수 있다" % (MIN_SIDE * 100))
    if out["x"] < 0 or out["y"] < 0 or out["x"] + out["w"] > 1.0001 \
            or out["y"] + out["h"] > 1.0001:
        raise MaskError("마스크가 화면 밖으로 나간다 (좌표는 0~1 의 화면 비율이다)")
    # ⭐ 라벨을 **필수**로 둔다. 라벨 없는 마스크는 평면도에 설명 없는 구멍을 남기고,
    #    나중에 본 사람이 그 구멍을 관측 실패로 읽는다.
    why = str(raw.get("why") or "").strip()
    if not why:
        raise MaskError("무엇을 가리는지 적어야 한다 (예: 삼각대 다리, 모니터 베젤)")
    out["why"] = why[:MAX_WHY]
    return out


def union_fraction(masks):
    """겹침을 빼고 화면의 몇 할을 덮는가. 겹쳐 놓고 100% 를 넘게 세지 않는다."""
    if not masks:
        return 0.0
    # 화면을 성긴 격자로 찍어 센다. 정확한 다각형 합집합이 필요할 만큼 정밀한
    # 값이 아니다 - '얼마나 가렸나'를 사람에게 말해 주는 수다.
    n = 200
    cells = 0
    for i in range(n):
        cy = (i + 0.5) / n
        for j in range(n):
            cx = (j + 0.5) / n
            for m in masks:
                if m["x"] <= cx < m["x"] + m["w"] and m["y"] <= cy < m["y"] + m["h"]:
                    cells += 1
                    break
    return round(cells / float(n * n), 4)


def contains(mask, px, frame_size, margin=0.0):
    """픽셀 점이 이 마스크 안인가. 마스크는 **화면 비율**이라 크기가 필요하다."""
    if not mask or not frame_size:
        return False
    fw, fh = float(frame_size[0]), float(frame_size[1])
    if fw <= 0 or fh <= 0:
        return False
    x0 = (mask["x"] - margin) * fw
    y0 = (mask["y"] - margin) * fh
    x1 = (mask["x"] + mask["w"] + margin) * fw
    y1 = (mask["y"] + mask["h"] + margin) * fh
    return (x0 <= px[0] <= x1) and (y0 <= px[1] <= y1)


def covering(masks, px, frame_size, margin=0.0):
    """그 점을 덮는 첫 마스크. 없으면 None. 라벨을 돌려주려고 마스크째 준다."""
    for m in (masks or []):
        if contains(m, px, frame_size, margin):
            return m
    return None


def validate_against_calibration(masks, state, frame_size):
    """⭐ 정착한 **모서리·검증점**을 덮으면 거부한다.

    그 자리를 안 본다고 선언하면 그 정합은 못 보는 점으로 푼 행렬이 된다.
    카메라를 옮길지 마스크를 줄일지는 **사람이 정한다** — 우리가 조용히 고르지 않는다.
    """
    if not state or not frame_size:
        return
    named = []
    for i, p in enumerate(state.get("corners") or []):
        named.append(("%d번 모서리" % (i + 1), (p[0], p[1])))
    # ⭐⭐ 키가 **두 가지**다. 상태 딕트는 `verify`, 영수증은 `verifyPoints` 다.
    #    처음엔 `verifyPoints` 만 읽었고 - 시험은 초록인데 **운영에서만** 검증점
    #    보호가 안 걸렸다. 이 세션에서 픽스처가 실물과 달라 소비자를 속인
    #    다섯 번째 경우다. 둘 다 실재하는 모양이므로 둘 다 읽는다.
    for key in ("verify", "verifyPoints"):
        for v in (state.get(key) or []):
            px = (v or {}).get("px")
            if px:
                named.append(("검증점 %s" % v.get("name"), (px[0], px[1])))
    for label, px in named:
        m = covering(masks, px, frame_size)
        if m:
            raise MaskError(
                "마스크 '%s' 가 %s 를 덮는다. 그 점을 안 보면 그 정합은 "
                "못 보는 점으로 푼 것이 된다 — 카메라를 옮기거나 마스크를 줄여야 한다."
                % (m["why"], label))


def parse(raw_list, state=None, frame_size=None):
    """목록 전체를 검사한다. 하나라도 틀리면 **아무것도 안 바꾼다**."""
    if raw_list is None:
        raw_list = []
    if not isinstance(raw_list, list):
        raise MaskError("마스크 목록은 배열이어야 한다")
    out = [normalize(r) for r in raw_list]
    frac = union_fraction(out)
    if frac >= DEGENERATE_FRACTION:
        raise MaskError(
            "화면의 %.0f%% 를 덮는다 — 이건 마스킹이 아니라 소스를 끄는 것이다. "
            "끄려면 소스를 끄면 된다." % (frac * 100))
    validate_against_calibration(out, state, frame_size)
    return out


# ---- 보관 --------------------------------------------------------------------
#
# ⭐ 마스크는 **정착과 따로** 산다. 카메라 자리가 그대로면 다시 정착해도 삼각대는
#    같은 자리에 있다. 정착을 해제할 때 마스크까지 지우면 사람이 매번 다시 그린다.

def path_for(state_dir, source_id):
    return os.path.join(state_dir, "masks_%s.json" % source_id)


def save(state_dir, source_id, masks):
    os.makedirs(state_dir, exist_ok=True)
    p = path_for(state_dir, source_id)
    payload = {"sourceId": source_id, "masks": masks}
    tmp = p + ".tmp"
    # 운영 중 파일을 0바이트로 자르지 않는다 - 원자적으로만 쓴다.
    with io.open(tmp, "w", encoding="utf-8") as fh:
        fh.write(json.dumps(payload, ensure_ascii=False, indent=2))
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, p)
    return payload


def load(state_dir, source_id):
    """(masks, problem). problem 이 None 이 아니면 **읽다가 틀어진 것**이다.

    ⭐ 못 읽었을 때 빈 목록을 쓰는 쪽이 **안전한 방향**이다 — 마스크가 없으면
       삼각대가 벽으로 읽히고, 벽은 로봇을 돌아가게 한다(보수적). 반대로 마스크가
       있는 척하면 삼각대 자리가 바닥이 되고 로봇이 거기로 들어간다.

    ⭐⭐ 그렇다고 **조용히** 빈 목록을 쓰지는 않는다. 운영자는 마스크를 그려 놨는데
       평면도에 유령 벽이 생기면 관측 실패로 읽는다. 왜 그런지 같이 돌려준다.
    """
    p = path_for(state_dir, source_id)
    if not os.path.exists(p):
        return [], None
    try:
        with io.open(p, encoding="utf-8") as fh:
            got = json.load(fh)
        return [normalize(m) for m in (got.get("masks") or [])], None
    except (OSError, ValueError, MaskError) as exc:
        return [], ("마스크 파일을 못 읽었다 (%s). 마스크 없이 본다 — "
                    "가린 자리가 벽으로 읽힐 수 있다: %s"
                    % (os.path.basename(p), exc))


def overlaps_rect(mask, rect_px, frame_size):
    """마스크가 이 **픽셀 사각형**과 겹치나. rect = (x0, y0, x1, y1).

    점이 아니라 사각형으로 보는 이유: 흔들림 조각은 96px 상자다. 한쪽 귀퉁이에
    케이블이 걸려 있어도 그 조각의 정합도는 케이블을 따라 흔들린다.
    """
    if not mask or not frame_size:
        return False
    fw, fh = float(frame_size[0]), float(frame_size[1])
    if fw <= 0 or fh <= 0:
        return False
    mx0, my0 = mask["x"] * fw, mask["y"] * fh
    mx1, my1 = mx0 + mask["w"] * fw, my0 + mask["h"] * fh
    x0, y0, x1, y1 = rect_px
    return not (x1 <= mx0 or x0 >= mx1 or y1 <= my0 or y0 >= my1)


def any_overlaps_rect(masks, rect_px, frame_size):
    """겹치는 첫 마스크. 없으면 None — 라벨을 쓰려고 마스크째 준다."""
    for m in (masks or []):
        if overlaps_rect(m, rect_px, frame_size):
            return m
    return None
