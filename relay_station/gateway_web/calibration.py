# -*- coding: utf-8 -*-
"""MCV-2A-UI / MCV-2S3 — 화면 픽셀을 아레나 평면(cm)으로 옮기는 호모그래피와 그 영수증.

`docs/VIEWPOINT_AND_OVERLAY_DESIGN.md` §4(V-2) 의 사슬에서 **4단계**다.

    1 회전 정규화 · 2 종횡비 · 3 렌즈 왜곡 보정 · 4 **호모그래피** · 5 합성

⭐ **3이 4보다 먼저다.** 사영변환은 선형이라 비선형 왜곡을 흡수 못 하는데,
   아레나 코너는 정확히 화면 **가장자리**에 온다 — 왜곡이 가장 큰 자리다.
   이 모듈은 왜곡 보정을 하지 않는다. 그래서 영수증에 `undistorted` 를 적어 둔다 —
   무엇을 **안 하고** 잰 값인지 남기려는 것이다.

## ⭐⭐ 네 점만으로는 정확도를 말할 수 없다

4 대응점 = 미지수 8개 = 방정식 8개. **재투영 오차가 항상 0이다.** 구조상 그렇다.
그러니 "재투영 오차 0.0 px, 정합 완료" 는 아무것도 증명하지 않는다.

    풀이에 쓴 점    -> 오차 0 (구조상)     : 품질 신호가 아니다
    풀이에 안 쓴 점 -> 실제 오차            : **이것만 측정이다**

그래서 이 모듈은 두 종류의 점을 가른다:
  · `corners`  네 모서리 — 푸는 데 쓴다
  · `verify`   아레나 명세가 cm 를 아는 **다른** 지점 — 재는 데만 쓴다
검증점이 없으면 정착은 되지만 영수증에 **`accuracyMeasured: false`** 가 박힌다.
"정확도를 안 쟀다"와 "정확도가 좋다"를 같은 화면에 같은 색으로 두지 않으려는 것이다.

## ⭐ 오늘 현장이 바뀐다 — 그래서 아레나가 입력이다

줄자 명세는 270 × 125 cm 인데 라이다 맵에서 나온 월드는 235 × 125 cm 였다(가로 35 cm 차이).
캘리브레이션은 **어느 아레나를 상대로 쟀는지**를 모르면 조용히 틀린다.
`configs/arena.json` 의 `version` 이 바뀌면 기존 정착은 **무효**로 떨어진다.
"""
import json
import os
import threading
import time

import numpy


class CalibrationError(ValueError):
    """사람이 고칠 수 있는 입력 문제. 서버는 이걸 400 으로 바꾼다."""


# ---- 상태 (V-3 §5 표와 같은 이름을 쓴다) ------------------------------------------

STATE_UNCALIBRATED = "UNCALIBRATED"
STATE_CALIBRATING = "CALIBRATING"
STATE_SETTLED = "SETTLED"
STATE_DRIFT = "DRIFT"

# 정착이 무효로 떨어진 이유. 문구가 아니라 **코드**다 — 표현은 UI 소유(R-6).
REASON_ARENA_CHANGED = "ARENA_CHANGED"
REASON_FRAME_SIZE_CHANGED = "FRAME_SIZE_CHANGED"
REASON_PUBLISHER_RESTARTED = "PUBLISHER_RESTARTED"
# ⭐ 광각 1280x720 과 일반 1280x720 은 **프레임 크기가 같다.** 크기 검사로는 못 잡는다.
REASON_LENS_CHANGED = "LENS_CHANGED"

# 아레나 좌표계는 원점이 **좌하단**이다(줄자 명세 §2). 화면은 좌상단이 원점이다.
# 둘을 헷갈리면 상이 위아래로 뒤집힌 채 "정합됐다"고 말하게 된다.
CORNERS = ("bl", "br", "tr", "tl")
IMAGE_ORIGIN = "top-left"
ARENA_ORIGIN = "bottom-left"

# 기하 게이트. 이 값들은 **실패할 수 있어야** 의미가 있다.
MIN_CORNER_SEP_PX = 8.0        # 두 핀이 이보다 가까우면 사실상 같은 점이다
MIN_AREA_FRACTION = 0.02       # 화면의 2% 미만이면 찍다 만 것이다
MAX_CONDITION_NUMBER = 1.0e10  # 수치적으로 못 믿을 행렬

# 검열 상자에 핀이 앉는 것을 막는 여유(프레임 비율). 상자 경계에 딱 붙는 것도 위험하다.
CENSOR_MARGIN = 0.0

DEFAULT_ARENA_PATH = os.environ.get(
    "MCV_ARENA_CONFIG",
    os.path.expanduser("~/pinky_pro/src/pinky_pro_team11/configs/arena.json"),
)
DEFAULT_STATE_DIR = os.environ.get(
    "MCV_CALIBRATION_DIR",
    os.path.expanduser("~/pinky_pro/src/pinky_pro_team11/var/calibration"),
)


# ---- 아레나 ------------------------------------------------------------------

class Arena(object):
    """캘리브레이션의 목적지. 실물 줄자 명세에서 온다."""

    def __init__(self, version, width_cm, height_cm, landmarks=None, path=None):
        if not version:
            raise CalibrationError("아레나에 version 이 없다 — 영수증이 무엇을 잰 건지 못 적는다")
        if not (width_cm and height_cm) or width_cm <= 0 or height_cm <= 0:
            raise CalibrationError("아레나 치수가 양수가 아니다")
        self.version = str(version)
        self.width_cm = float(width_cm)
        self.height_cm = float(height_cm)
        self.landmarks = dict(landmarks or {})
        self.path = path

    def corner_targets(self):
        """네 모서리의 cm 좌표. CORNERS 순서와 짝이다."""
        w, h = self.width_cm, self.height_cm
        return {"bl": (0.0, 0.0), "br": (w, 0.0), "tr": (w, h), "tl": (0.0, h)}

    def landmark(self, name):
        got = self.landmarks.get(name)
        if got is None:
            raise CalibrationError("아레나 명세에 없는 지점: %s" % name)
        return (float(got["x"]), float(got["y"]))

    def to_dict(self):
        return {"version": self.version, "widthCm": self.width_cm,
                "heightCm": self.height_cm,
                "landmarks": sorted(self.landmarks.keys())}


def load_arena(path=None):
    """`configs/arena.json` 을 읽는다. 없으면 None — **추정치를 지어내지 않는다.**

    아레나를 모르면 캘리브레이션도 없다. 기본값을 코드에 박으면 현장이 바뀐 날
    조용히 옛 치수로 정착한다 — 그게 오늘 실제로 일어나는 일이다.
    """
    p = path or DEFAULT_ARENA_PATH
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as fh:
        raw = json.load(fh)
    return Arena(raw.get("version"), raw.get("widthCm"), raw.get("heightCm"),
                 raw.get("landmarks"), path=p)


# ---- 기하 --------------------------------------------------------------------

def _pt(value):
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError, IndexError, KeyError):
        raise CalibrationError("점 형식이 (x, y) 가 아니다: %r" % (value,))
    if not (numpy.isfinite(x) and numpy.isfinite(y)):
        raise CalibrationError("점에 유한하지 않은 값이 있다: %r" % (value,))
    return (x, y)


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def polygon_area(pts):
    """부호 없는 넓이. 자기교차 사각형(나비넥타이)에서는 작아진다."""
    s = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


def is_convex_quad(pts):
    """주어진 **순서대로** 이었을 때 볼록한가.

    ⭐ 이게 나비넥타이를 잡는다. 핀 두 개를 엇갈리게 끌어다 놓아도 호모그래피는
       멀쩡히 풀린다 — 그리고 상을 뒤집어 놓는다. 숫자로는 아무 이상이 없다.
    """
    if len(pts) != 4:
        return False
    signs = []
    for i in range(4):
        c = _cross(pts[i], pts[(i + 1) % 4], pts[(i + 2) % 4])
        if abs(c) < 1e-9:
            return False                       # 세 점이 한 줄
        signs.append(c > 0)
    return len(set(signs)) == 1


def point_in_box(px, frame_size, box, margin=CENSOR_MARGIN):
    """픽셀 점이 검열 상자 안인가. 상자는 **프레임 비율**이므로 크기가 필요하다."""
    if not box or not frame_size:
        return False
    fw, fh = float(frame_size[0]), float(frame_size[1])
    if fw <= 0 or fh <= 0:
        return False
    x0 = (float(box["x"]) - margin) * fw
    y0 = (float(box["y"]) - margin) * fh
    x1 = (float(box["x"]) + float(box["w"]) + margin) * fw
    y1 = (float(box["y"]) + float(box["h"]) + margin) * fh
    return (x0 <= px[0] <= x1) and (y0 <= px[1] <= y1)


def check_corner_quad(pts, frame_size=None, censor_box=None):
    """정착시켜도 되는 사각형인가. 실패하면 이유를 문장으로 던진다."""
    if len(pts) != 4:
        raise CalibrationError("모서리는 정확히 4개다 (받은 것 %d개)" % len(pts))
    for i in range(4):
        for j in range(i + 1, 4):
            dx = pts[i][0] - pts[j][0]
            dy = pts[i][1] - pts[j][1]
            if (dx * dx + dy * dy) ** 0.5 < MIN_CORNER_SEP_PX:
                raise CalibrationError(
                    "%s 와 %s 가 %.1f px 안에 겹친다"
                    % (CORNERS[i], CORNERS[j], MIN_CORNER_SEP_PX))
    if not is_convex_quad(pts):
        raise CalibrationError("네 점이 볼록한 사각형이 아니다 — 순서가 엇갈렸거나 한 줄에 있다")
    if frame_size:
        fw, fh = float(frame_size[0]), float(frame_size[1])
        if fw > 0 and fh > 0:
            for i, (x, y) in enumerate(pts):
                if not (0.0 <= x <= fw and 0.0 <= y <= fh):
                    raise CalibrationError(
                        "%s 가 화면 밖이다 (%.1f, %.1f) / %dx%d"
                        % (CORNERS[i], x, y, int(fw), int(fh)))
            frac = polygon_area(pts) / (fw * fh)
            if frac < MIN_AREA_FRACTION:
                raise CalibrationError(
                    "사각형이 화면의 %.2f%% 뿐이다 (최소 %.0f%%)"
                    % (frac * 100.0, MIN_AREA_FRACTION * 100.0))
    # ⭐ 가려진 자리에 찍은 핀은 **찍은 게 아니라 짐작한 것**이다.
    #    관측 경로는 검열 단 아래에 있어서 그 영역을 볼 수가 없다.
    if censor_box:
        for i, p in enumerate(pts):
            if point_in_box(p, frame_size, censor_box):
                raise CalibrationError(
                    "%s 가 검열 상자 안이다 — 안 보이는 자리는 찍을 수 없다. "
                    "가공 단의 상자를 옮기거나 카메라 화각을 바꾼다" % CORNERS[i])
    return True


# ---- 호모그래피 ---------------------------------------------------------------

def solve_homography(src, dst):
    """src(화면 px) → dst(아레나 cm) 사영행렬 3x3. 네 대응점이면 유일해다."""
    src = [_pt(p) for p in src]
    dst = [_pt(p) for p in dst]
    if len(src) != 4 or len(dst) != 4:
        raise CalibrationError("네 쌍이 필요하다 (src=%d dst=%d)" % (len(src), len(dst)))
    rows, rhs = [], []
    for (x, y), (u, v) in zip(src, dst):
        rows.append([x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y])
        rhs.append(u)
        rows.append([0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y])
        rhs.append(v)
    a = numpy.asarray(rows, dtype=float)
    b = numpy.asarray(rhs, dtype=float)
    # ⭐ 특이 행렬은 solve 가 **예외를 안 던지고 쓰레기를 돌려주기도 한다.**
    #    조건수를 먼저 본다 — 통과 못 할 수 있어야 게이트다.
    cond = numpy.linalg.cond(a)
    if not numpy.isfinite(cond) or cond > MAX_CONDITION_NUMBER:
        raise CalibrationError("특이에 가까운 배치다 (조건수 %.3g) — 점들이 한 줄에 있다" % cond)
    try:
        h = numpy.linalg.solve(a, b)
    except numpy.linalg.LinAlgError as exc:
        raise CalibrationError("풀리지 않는다: %s" % exc)
    return [[float(h[0]), float(h[1]), float(h[2])],
            [float(h[3]), float(h[4]), float(h[5])],
            [float(h[6]), float(h[7]), 1.0]]


def project(matrix, x, y):
    """화면 px → 아레나 cm."""
    m = matrix
    w = m[2][0] * x + m[2][1] * y + m[2][2]
    if abs(w) < 1e-12:
        raise CalibrationError("무한원점으로 사영된다 (%.1f, %.1f)" % (x, y))
    return ((m[0][0] * x + m[0][1] * y + m[0][2]) / w,
            (m[1][0] * x + m[1][1] * y + m[1][2]) / w)


def reprojection_errors_cm(matrix, pairs):
    """[(픽셀점, 아레나cm점)] → [오차 cm]. **풀이에 안 쓴 점에만** 의미가 있다."""
    out = []
    for px, cm in pairs:
        u, v = project(matrix, px[0], px[1])
        out.append(((u - cm[0]) ** 2 + (v - cm[1]) ** 2) ** 0.5)
    return out


# ---- 상태 저장소 --------------------------------------------------------------

def _atomic_write_json(path, payload):
    """운영 중 파일을 0바이트로 만들지 않는다(2026-09-08 사고)."""
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    tmp = path + ".tmp"
    data = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
    with open(tmp, "wb") as fh:
        fh.write(data)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


class CalibrationStore(object):
    """소스별 캘리브레이션 상태 + **덧붙이기만 하는** 영수증(MCV-2S3)."""

    def __init__(self, arena, state_dir=None, clock=time.time):
        self.arena = arena
        self.state_dir = state_dir or DEFAULT_STATE_DIR
        self._clock = clock
        self._lock = threading.RLock()
        self._working = {}     # source_id -> 작업 중인 점들
        self._settled = {}     # source_id -> 마지막 영수증
        self._load()

    # -- 경로 --
    @property
    def working_path(self):
        return os.path.join(self.state_dir, "working.json")

    @property
    def receipts_path(self):
        return os.path.join(self.state_dir, "receipts.jsonl")

    # -- 적재 --
    def _load(self):
        try:
            with open(self.working_path, encoding="utf-8") as fh:
                self._working = json.load(fh) or {}
        except (OSError, ValueError):
            self._working = {}
        for rec in self.read_receipts():
            sid = rec.get("sourceId")
            if sid:
                self._settled[sid] = rec

    def read_receipts(self, source_id=None, limit=None):
        """영수증은 **덮어쓰지 않는다.** 과거 정착이 남아 있어야 오늘 값을 의심할 수 있다."""
        out = []
        try:
            with open(self.receipts_path, encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        rec = json.loads(line)
                    except ValueError:
                        continue          # 한 줄이 깨져도 나머지는 읽는다
                    if source_id and rec.get("sourceId") != source_id:
                        continue
                    out.append(rec)
        except OSError:
            return []
        if limit:
            return out[-limit:]
        return out

    # -- 쓰기 --
    def set_points(self, source_id, corners, frame_size, publisher_session=None,
                   verify=None, undistorted=False, censor_box=None, lens=None):
        """네 모서리(+검증점)를 놓는다. 아직 정착은 아니다 — CALIBRATING."""
        if self.arena is None:
            raise CalibrationError("아레나 명세가 없다 (%s) — 목적지 좌표를 모른다"
                                   % DEFAULT_ARENA_PATH)
        pts = [_pt(p) for p in corners]
        check_corner_quad(pts, frame_size, censor_box)
        vpairs = self._verify_pairs(verify)
        # 검증점도 가려진 자리면 안 된다 - 못 본 것을 잰 척하게 된다.
        for name, px, _cm in vpairs:
            if point_in_box(px, frame_size, censor_box):
                raise CalibrationError(
                    "검증점 %s 가 검열 상자 안이다 - 안 보이는 자리는 못 잰다" % name)
        entry = {
            "corners": [list(p) for p in pts],
            "frameSize": ([int(frame_size[0]), int(frame_size[1])]
                          if frame_size else None),
            "publisherSession": publisher_session,
            "arenaVersion": self.arena.version,
            "verify": [{"name": n, "px": list(px)} for n, px, _cm in vpairs],
            "undistorted": bool(undistorted),
            "censorBox": (dict(censor_box) if censor_box else None),
            "lens": lens,
            "updatedAt": self._clock(),
        }
        with self._lock:
            self._working[source_id] = entry
            _atomic_write_json(self.working_path, self._working)
        return self.state(source_id, frame_size=frame_size,
                          publisher_session=publisher_session)

    def clear(self, source_id):
        with self._lock:
            self._working.pop(source_id, None)
            _atomic_write_json(self.working_path, self._working)

    def _verify_pairs(self, verify):
        """[{name, px}] → [(name, px, cm)]. 이름은 아레나 명세의 지점이어야 한다."""
        out = []
        for item in (verify or []):
            name = (item or {}).get("name")
            px = _pt((item or {}).get("px") or [])
            out.append((name, px, self.arena.landmark(name)))
        return out

    def settle(self, source_id, note=None):
        """호모그래피를 확정하고 **영수증을 덧붙인다.** 되돌리려면 다시 정착한다."""
        with self._lock:
            entry = self._working.get(source_id)
        if not entry:
            raise CalibrationError("놓인 점이 없다 — 먼저 네 모서리를 찍는다")
        if self.arena is None:
            raise CalibrationError("아레나 명세가 없다")
        if entry.get("arenaVersion") != self.arena.version:
            raise CalibrationError(
                "점을 찍은 아레나(%s)와 지금 아레나(%s)가 다르다 — 다시 찍는다"
                % (entry.get("arenaVersion"), self.arena.version))
        pts = [_pt(p) for p in entry["corners"]]
        frame_size = entry.get("frameSize")
        check_corner_quad(pts, frame_size, entry.get("censorBox"))
        targets = self.arena.corner_targets()
        dst = [targets[c] for c in CORNERS]
        matrix = solve_homography(pts, dst)

        vpairs = self._verify_pairs(entry.get("verify"))
        errs = reprojection_errors_cm(matrix, [(px, cm) for _n, px, cm in vpairs])
        receipt = {
            "sourceId": source_id,
            "settledAt": self._clock(),
            "arena": self.arena.to_dict(),
            "arenaVersion": self.arena.version,
            "frameSize": frame_size,
            "publisherSession": entry.get("publisherSession"),
            "imageOrigin": IMAGE_ORIGIN,
            "arenaOrigin": ARENA_ORIGIN,
            "corners": [{"name": CORNERS[i], "px": list(pts[i]),
                         "cm": list(dst[i])} for i in range(4)],
            "homography": [[float(v) for v in row] for row in matrix],
            # ⭐ 풀이에 쓴 네 점의 재투영 오차는 **구조상 0** 이라 적지 않는다.
            #    적으면 그 0 을 정확도로 읽는 사람이 반드시 나온다.
            "accuracyMeasured": bool(errs),
            "verifyPoints": [
                {"name": n, "px": list(px), "expectedCm": list(cm),
                 "errorCm": round(e, 2)}
                for (n, px, cm), e in zip(vpairs, errs)],
            "maxVerifyErrorCm": (round(max(errs), 2) if errs else None),
            "undistorted": bool(entry.get("undistorted")),
            # ⭐ 무엇을 **못 본 채** 잰 값인지 남긴다. 상자가 나중에 바뀌어도 이 정착은
            #    무효가 아니다 - 흐림은 기하가 아니라 검출을 건드린다. 그래서 무효화
            #    조건에 넣지 않고 출처로만 적는다.
            "censorBox": entry.get("censorBox"),
            # ⭐ 어느 렌즈로 잰 값인지. 같은 픽셀 크기라도 화각이 다르면 다른 사상이다.
            "lens": entry.get("lens"),
            "note": note,
            "receiptVersion": 1,
        }
        with self._lock:
            os.makedirs(self.state_dir, exist_ok=True)
            with open(self.receipts_path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(receipt, ensure_ascii=False) + "\n")
                fh.flush()
                os.fsync(fh.fileno())
            self._settled[source_id] = receipt
            self._working.pop(source_id, None)
            _atomic_write_json(self.working_path, self._working)
        return receipt

    # -- 읽기 --
    def state(self, source_id, frame_size=None, publisher_session=None, lens=None):
        """지금 이 소스의 정합 상태. **모르는 소스도 예외를 던지지 않는다.**

        ⭐ 정착은 잰 순간의 조건에 묶여 있다. 조건이 바뀌면 **자동으로 내려온다.**
           크기를 다시 맞춰 살려내지 않는다 — 화각이 같다는 보장이 없기 때문이다
           (APP-1 은 렌즈를 광각으로 바꾼다. 픽셀 수만 바뀌는 게 아니다).
        """
        with self._lock:
            work = self._working.get(source_id)
            rec = self._settled.get(source_id)
        out = {
            "sourceId": source_id,
            "state": STATE_UNCALIBRATED,
            "reason": None,
            "arenaVersion": (self.arena.version if self.arena else None),
            "arena": (self.arena.to_dict() if self.arena else None),
            "corners": None,
            "verify": None,
            "homography": None,
            "settledAt": None,
            "accuracyMeasured": False,
            "maxVerifyErrorCm": None,
            "frameSize": None,
            "censorBox": None,
            "lens": None,
        }
        if work:
            out["state"] = STATE_CALIBRATING
            out["corners"] = work.get("corners")
            out["verify"] = work.get("verify")
            out["frameSize"] = work.get("frameSize")
            out["censorBox"] = work.get("censorBox")
            return out
        if not rec:
            return out
        out.update({
            "state": STATE_SETTLED,
            "corners": [c["px"] for c in rec.get("corners", [])],
            "verify": rec.get("verifyPoints"),
            "homography": rec.get("homography"),
            "settledAt": rec.get("settledAt"),
            "accuracyMeasured": rec.get("accuracyMeasured", False),
            "maxVerifyErrorCm": rec.get("maxVerifyErrorCm"),
            "frameSize": rec.get("frameSize"),
            "censorBox": rec.get("censorBox"),
            "lens": rec.get("lens"),
        })
        reason = self._invalidation(rec, frame_size, publisher_session, lens)
        if reason:
            out["state"] = STATE_DRIFT
            out["reason"] = reason
        return out

    def _invalidation(self, rec, frame_size, publisher_session, lens=None):
        if self.arena and rec.get("arenaVersion") != self.arena.version:
            return REASON_ARENA_CHANGED
        # ⭐ 렌즈를 크기보다 **먼저** 본다. 둘 다 바뀌었을 때 "크기가 바뀌었다" 보다
        #    "렌즈가 바뀌었다" 가 사람에게 더 정확한 말이기 때문이다.
        #    (처음엔 주석만 이렇게 쓰고 코드는 아래에 뒀다 — 시험이 그 불일치를 잡았다)
        was_lens = rec.get("lens")
        if was_lens and lens and lens != was_lens:
            return REASON_LENS_CHANGED
        if frame_size and rec.get("frameSize"):
            if [int(frame_size[0]), int(frame_size[1])] != list(rec["frameSize"]):
                return REASON_FRAME_SIZE_CHANGED
        was = rec.get("publisherSession")
        if was and publisher_session and publisher_session != was:
            return REASON_PUBLISHER_RESTARTED
        return None

    def homography_for(self, source_id, frame_size=None, publisher_session=None,
                       lens=None):
        """정착된 행렬. **정착이 아니면 None** — 추정값을 관측으로 내보내지 않는다."""
        st = self.state(source_id, frame_size, publisher_session, lens)
        if st["state"] != STATE_SETTLED:
            return None
        return st["homography"]
