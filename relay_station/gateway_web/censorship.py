# -*- coding: utf-8 -*-
"""MCV-2C — 이 프레임이 검열을 거쳤다고 **말해도 되는가**.

## 왜 이 모듈이 생겼나

지금까지 소비자는 "`X-Process-Rules` 헤더가 있으면 검열된 프레임" 으로 알았다.
그게 성립한 건 **그 헤더를 찍는 구현이 하나뿐**(proot 가공 단)이었기 때문이다.

사용자 의도에 따라 가공·검열이 **앱 안으로** 옮겨간다. 그러면 그 헤더를 찍는 구현이
둘이 되고, 위의 추론은 **조용히** 죽는다 — 프레임도 오고 헤더도 붙어 있는데 뜻만 사라진다.
이 프로젝트가 같은 모양으로 네 번 물렸다(거짓 `connected` · `curl -I 200` ·
`/status.resolution` · 구독만으로 뜨는 `topic list`).

## ⭐⭐ 원칙 — 라벨의 **존재**로 검열을 추론하지 않는다

두 축으로 본다:

    ① 누가 만들었나   rules 값이 구현을 식별해야 한다 (`proot-v2` / `app-0.2.0+빌드`)
                      그리고 **내가 인정한 목록에 있어야** 한다. 모르면 검열 안 된 것으로 친다.
    ② 정말 그런가     선언한 상자가 실제로 흐린지 **픽셀로** 본다.

## ⭐⭐ 픽셀 검사는 **반증만 한다** — 증명하지 못한다

흐린 영역은 고주파가 없다. 그런데 **민무늬 벽도 고주파가 없다.** 그래서:

    안쪽이 또렷하다        -> 흐리지 않았다는 **증명** (31 커널 가우시안이 이걸 남길 수 없다)
    안쪽이 밋밋하다        -> 흐렸을 수도, 원래 밋밋했을 수도 있다. **증명이 아니다**

그래서 통과 상태의 이름이 `VERIFIED` 가 아니라 `DECLARED_CONSISTENT` 다.
"검산을 통과했다"와 "검열됐음이 증명됐다"는 다른 말이고, 후자는 우리가 할 수 없다.

## 🔴 시청은 끊지 않는다 (사용자 결정, 2026-09-11)

검열을 못 믿는 상태여도 **시청 화면은 계속 나간다.** 교육장 화면이 비는 쪽이 더 나쁘다는
판단이다. 그리고 게이트웨이는 **집행 지점이 아니다** — 미검열이 나가는 것을 막는 일은
가공 단(또는 그 역할을 넘겨받은 앱)의 fail-closed 에 있다.

이 모듈이 하는 일은 **말을 정확히 하는 것**뿐이다:
검열됐다고 말하지 않고, 저하로 드러내고, 관측 입력에서 뺄지 판단할 근거를 준다.
그래서 이 모듈은 `/video_feed` 경로에서 **불리지 않는다.** 시험이 그걸 고정한다.
"""
import io
import json
import os

# ---- 상태 --------------------------------------------------------------------

VERIFIED_IMPOSSIBLE = None      # 존재하지 않는 상태 — 위 독스트링 참조

STATE_CONSISTENT = "DECLARED_CONSISTENT"   # 인정된 구현 + 상자 선언 + 픽셀이 반증 안 함
STATE_UNVERIFIED = "DECLARED_UNVERIFIED"   # 인정된 구현 + 상자 선언 + 아직 안 봄
STATE_FALSE = "DECLARED_FALSE"             # 상자를 선언했는데 그 영역이 또렷하다
STATE_OFF = "CENSORSHIP_OFF"               # 인정된 구현인데 상자가 없다 (규칙상 꺼짐)
STATE_UNKNOWN = "UNKNOWN_PRODUCER"         # rules 가 없거나 목록에 없다

# 시청은 계속하므로 severity 는 **말의 세기**지 차단 등급이 아니다.
SEVERITY = {
    STATE_FALSE: "safety",        # 만든 쪽이 사실과 다른 말을 하고 있다
    STATE_UNKNOWN: "degraded",    # 검열됐는지 우리가 알 수 없다
    STATE_UNVERIFIED: "info",
    STATE_OFF: "info",
    STATE_CONSISTENT: "ok",
}

DEFAULT_POLICY_PATH = os.environ.get(
    "MCV_CENSORSHIP_POLICY",
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))), "relay_station", "configs", "censorship.json"),
)

# 안쪽이 바깥쪽보다 이만큼 또렷하면 "안 흐렸다" 고 말한다.
# 흐린 영역이 배경보다 또렷할 방법은 없으므로, 1.0 근처는 이미 반증이다.
# 여유를 둬서 0.6 — 노이즈·압축으로 흔들리는 폭을 넘기려는 것이다.
SHARPNESS_REFUTES_AT = 0.6
MIN_SAMPLES_PX = 400            # 상자가 너무 작으면 통계가 안 된다



def producer_of(rules):
    """rules 값에서 **구현 식별자**만 뽑는다 — `@` 앞이다.

    ⭐⭐ 왜 필요한가 (2026-09-11 실측으로 드러난 결함)
        가공 단은 `X-Process-Rules` 를 `<구현>@<규칙파일 시각>` 으로 낸다
        (다른 레포 외부 가공 단 구현, 2026-09-11 확인.
        예: `proot-v2@08:18:51`). 그 시각은 규칙을
        다시 읽을 때마다 바뀐다 — **같은 구현의 변형이지 다른 구현이 아니다.**

        그런데 대조가 정확 일치였다. 그래서 `proot-v2` 를 등재해도 실제 프레임은
        **영원히 안 걸렸고**, 모든 프레임이 `UNKNOWN_PRODUCER` 로 떨어져
        그 아래 네 상태(OFF·UNVERIFIED·FALSE·CONSISTENT)가 **도달 불가능**했다.
        fail-closed 라 안전했지만 **무조건** 그래서, allowlist 도 픽셀 검산도
        아무 일을 하지 않았다. 늘 안전한 자리에 있는 가드는 작동하는 것처럼 보인다.

        시험 21개가 이걸 못 잡은 이유: 전부 `"proot-v2"` 를 **접미사 없이** 썼다.
        대역이 가정과 같은 거짓말을 하면 결함이 안 보인다.

    ⭐ `@` 는 **첫 번째**에서만 자른다. `evil@proot-v2` 가 `proot-v2` 로 읽히면
       그게 곧 위조다.
    """
    if not rules:
        return ""
    return rules.split("@", 1)[0].strip()

class Policy(object):
    """검열됐다고 인정할 구현 목록. **없으면 아무도 인정하지 않는다**(fail-closed)."""

    def __init__(self, trusted_rules=None, path=None):
        self.trusted_rules = tuple(trusted_rules or ())
        self.path = path

    def trusts(self, rules):
        """등재를 **어떻게 적었는지**가 대조 단위를 정한다.

            `proot-v2`     -> 그 구현의 모든 재적재본을 인정한다 (구현 단위)
            `app-0.2.1@9`  -> **그 값만** 인정한다 (빌드 단위)

        `_등재_조건` 이 "사람이 실제로 검열함을 확인한 뒤" 이므로, 빌드까지 좁히고
        싶은 소비자는 `@` 까지 적으면 된다. 규칙을 바꾸지 않고 **표현으로** 고른다.
        """
        if not rules:
            return False
        if rules in self.trusted_rules:          # 빌드 단위 — 정확 일치
            return True
        producer = producer_of(rules)            # 구현 단위 — `@` 앞
        return bool(producer) and producer in self.trusted_rules

    def to_dict(self):
        return {"trustedRules": list(self.trusted_rules), "path": self.path}


def load_policy(path=None):
    """`configs/censorship.json` 을 읽는다. 없으면 **빈 목록** — 지어내지 않는다.

    ⭐ 파일이 없다고 "다 믿는다" 로 떨어지면 이 장치의 의미가 없다.
    """
    p = path or DEFAULT_POLICY_PATH
    if not os.path.exists(p):
        return Policy((), path=p)
    try:
        with io.open(p, encoding="utf-8") as fh:
            raw = json.load(fh)
    except (OSError, ValueError):
        return Policy((), path=p)
    rules = raw.get("trustedRules")
    if not isinstance(rules, list):
        return Policy((), path=p)
    return Policy([r for r in rules if isinstance(r, str) and r], path=p)


# ---- 픽셀 증거 ----------------------------------------------------------------

def _sharpness(gray):
    """고주파의 양. 라플라시안 분산 — 흐릴수록 작다."""
    import cv2
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def blur_evidence(frame_bgr, box):
    """선언된 상자 안/밖의 또렷함을 잰다. 잴 수 없으면 None.

    돌려주는 것은 **증거**지 판정이 아니다. 판정은 classify 가 한다.
    """
    if frame_bgr is None or not box:
        return None
    import cv2
    import numpy
    h, w = frame_bgr.shape[:2]
    if w <= 0 or h <= 0:
        return None
    x0 = max(0, min(w - 1, int(round(float(box["x"]) * w))))
    y0 = max(0, min(h - 1, int(round(float(box["y"]) * h))))
    x1 = max(0, min(w, int(round((float(box["x"]) + float(box["w"])) * w))))
    y1 = max(0, min(h, int(round((float(box["y"]) + float(box["h"])) * h))))
    if (x1 - x0) * (y1 - y0) < MIN_SAMPLES_PX:
        return None
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    inside = gray[y0:y1, x0:x1]
    mask = numpy.ones(gray.shape, dtype=bool)
    mask[y0:y1, x0:x1] = False
    if int(mask.sum()) < MIN_SAMPLES_PX:
        return None
    # 바깥은 직사각형이 아니라 구멍 뚫린 모양이다. 라플라시안은 전체에 걸고 마스크로 고른다.
    import cv2 as _cv2
    lap = _cv2.Laplacian(gray, _cv2.CV_64F)
    outside_var = float(lap[mask].var())
    inside_var = float(_sharpness(inside))
    ratio = (inside_var / outside_var) if outside_var > 1e-9 else None
    return {
        "insideSharpness": round(inside_var, 3),
        "outsideSharpness": round(outside_var, 3),
        "ratio": (round(ratio, 4) if ratio is not None else None),
        "boxPx": [x0, y0, x1, y1],
    }


# ---- 판정 --------------------------------------------------------------------

def classify(rules, box, evidence=None, policy=None):
    """(state, severity, detail). **시청을 끊는 데 쓰지 않는다** — 말을 정하는 데 쓴다."""
    policy = policy or Policy(())
    detail = {
        "rules": rules,
        "declaredBox": (dict(box) if box else None),
        "trustedProducer": policy.trusts(rules),
        "evidence": evidence,
    }
    if not policy.trusts(rules):
        # ⭐ 모르는 구현은 **검열 안 된 것으로 친다.** 앱이 :18082 를 먼저 잡아도
        #    조용히 인정하지 않는다 — 전환 순서를 구성 파일이 지키게 하는 자리다.
        return STATE_UNKNOWN, SEVERITY[STATE_UNKNOWN], detail
    if not box:
        return STATE_OFF, SEVERITY[STATE_OFF], detail
    if not evidence or evidence.get("ratio") is None:
        return STATE_UNVERIFIED, SEVERITY[STATE_UNVERIFIED], detail
    if evidence["ratio"] >= SHARPNESS_REFUTES_AT:
        # 안쪽이 바깥만큼 또렷하다 = 흐리지 않았다. 이건 **증명**이다.
        return STATE_FALSE, SEVERITY[STATE_FALSE], detail
    # 반증되지 않았을 뿐이다. 증명이 아니라서 이름이 CONSISTENT 다.
    return STATE_CONSISTENT, SEVERITY[STATE_CONSISTENT], detail


def degradation_for(source_id, state, severity, detail):
    """readiness 의 degradations 에 넣을 항목. 문구는 내지 않는다 — UI 소유(R-6)."""
    if severity == "ok":
        return None
    return {
        "code": "CENSORSHIP_" + state,
        "severity": severity,
        "sourceId": source_id,
        "rules": detail.get("rules"),
        "trustedProducer": detail.get("trustedProducer"),
        "declaredBox": detail.get("declaredBox"),
        "evidence": detail.get("evidence"),
    }
