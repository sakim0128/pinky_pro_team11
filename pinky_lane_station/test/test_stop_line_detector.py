"""목적지 흰 정지선 검출 (영상 처리). 실제 코스 사진(교차로 앞, 빨간 테이프·흰 차선, 정지선 없음)과 합성 정지선."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

cv2 = pytest.importorskip('cv2')

from pinky_lane_station.stop_line_detector import (StopLineParams, WhiteStopLineDetector,  # noqa: E402
                                                   find_band, params_from_dict)

HERE = os.path.dirname(os.path.abspath(__file__))
PHOTO = os.path.join(HERE, 'fixtures', 'junction_red_tape.jpg')


def photo():
    img = cv2.imread(PHOTO)
    assert img is not None, PHOTO
    return img


def with_line(img, y0_frac, y1_frac, x0_frac=0.12, x1_frac=0.88, value=235):
    out = img.copy()
    H, W = out.shape[:2]
    out[int(y0_frac * H):int(y1_frac * H), int(x0_frac * W):int(x1_frac * W)] = value
    return out


def carpet(W=640, H=480):
    """회색 카펫 + 흰 차선 두 개(대각) — 정지선 없음."""
    img = np.full((H, W, 3), 120, np.uint8)
    cv2.line(img, (140, H), (260, int(0.5 * H)), (240, 240, 240), 14)
    cv2.line(img, (500, H), (380, int(0.5 * H)), (240, 240, 240), 14)
    return img


def test_real_course_photo_has_no_stop_line():
    """빨간 테이프(채도 높음)·대각 흰 차선·먼 가로 차선(ROI 위)은 정지선이 아니다."""
    assert find_band(photo(), StopLineParams()) is None


def test_stop_line_near_bottom_is_raw_and_far_one_is_not():
    p = StopLineParams()
    H = photo().shape[0]
    near = find_band(with_line(photo(), 0.86, 0.89), p)
    assert near is not None and near[1] >= p.stop_row_frac * H and near[2] > 0.7
    far = find_band(with_line(photo(), 0.65, 0.68), p)
    assert far is not None and far[1] < p.stop_row_frac * H          # 보이지만 아직 멀다 → raw 아님
    det = WhiteStopLineDetector(p)
    assert not det.update(with_line(photo(), 0.65, 0.68)).raw


def test_diagonal_lane_lines_are_not_a_stop_line():
    assert find_band(carpet(), StopLineParams()) is None
    img = carpet()
    cv2.rectangle(img, (100, 400), (540, 418), (240, 240, 240), -1)
    band = find_band(img, StopLineParams())
    assert band is not None and 395 <= band[0] <= 402 and 414 <= band[1] <= 420


def test_narrow_or_thin_white_is_ignored():
    p = StopLineParams()
    assert find_band(with_line(photo(), 0.86, 0.89, 0.40, 0.60), p) is None     # 폭 20 % — 조각
    assert find_band(with_line(photo(), 0.860, 0.864), p) is None               # 두께 2 px — 잡음


def test_debounce_confirm_and_release():
    det = WhiteStopLineDetector(StopLineParams(confirm=2, release=3))
    line, empty = with_line(photo(), 0.86, 0.89), photo()
    assert not det.update(line).detected                  # 1/2
    r = det.update(line)
    assert r.raw and r.detected                           # 확정
    for i in range(3):
        assert det.update(empty).detected == (i < 2)      # 3 프레임 뒤 해제
    det.update(line)
    det.reset()
    assert not det.detected


def test_disabled_and_params_from_dict():
    p = params_from_dict({'enabled': False, 'v_min': 200, 'unknown_key': 1})
    assert p.enabled is False and p.v_min == 200
    det = WhiteStopLineDetector(p)
    assert not det.update(with_line(photo(), 0.86, 0.89)).raw


def test_detector_yolo_yaml_stop_line_block_matches_params():
    import yaml
    cfg = yaml.safe_load(open(os.path.join(HERE, '..', 'config', 'detector_yolo.yaml'), encoding='utf-8'))
    block = cfg['stop_line']
    assert set(block) <= set(StopLineParams.__dataclass_fields__), set(block) - set(StopLineParams.__dataclass_fields__)
    assert params_from_dict(block).enabled is True
