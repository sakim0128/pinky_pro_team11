# -*- coding: utf-8 -*-
"""R-6 — 저하 안내가 읽을 판정.

승인된 설계(2026-09-09):
    safety   빨강 · 모달   안전 기능이 죽었고 사람이 조치해야 한다
    degraded 노랑 · 배너   미뤄질 뿐 안전은 그대로 돈다
    info     배지          지금 아무것도 그것에 기대지 않는다 (MCV-1C, 2026-09-10)

⭐ 노랑을 모달로 안 띄우는 이유: 중단이 잦은 환경에서 모달이 반복되면 사람이 무시하게 되고,
   그러면 **진짜 빨강도 같이 무시된다.** 그 성질을 테스트로 고정한다.
⭐ 같은 이유로, 관측 세션이 꺼져 있을 때 실물 시점 부족은 safety 가 아니다 — 아무도 그 시점에
   기대고 있지 않다. 09-10 실측: `--no-camera` 기본 기동에서 빨강 모달이 상시로 떠 있었다.
"""
import pytest

from source_registry import (SourceRegistry, Source, PUSH, PULL, ROS, LOCAL,
                             TRUSTED, UNTRUSTED, readiness, real_viewpoint_ids,
                             MIN_REAL_VIEWPOINTS, SEVERITY_ORDER)

LIVE = b"\xff\xd8\xff-live-\xff\xd9"


def _provider(connected=True, stamp=1000.0):
    def p():
        return (LIVE if connected else None), (stamp if connected else 0.0), connected
    return p


def _registry(specs):
    """specs = [(id, transport, trust, connected)]"""
    reg = SourceRegistry()
    for sid, transport, trust, conn in specs:
        reg.register(Source(sid, _provider(conn), transport, trust))
    return reg


# ---- 실물 시점 세기 -----------------------------------------------------------

def test_local_과_pull_만_실물_시점으로_센다():
    """ros 는 Gazebo(가상)가 섞여 있고, push 는 출처 미상이라 관측 입력이 아니다."""
    reg = _registry([
        ("relay-cam", LOCAL, TRUSTED, True),
        ("phone", PULL, TRUSTED, True),
        ("gazebo", ROS, TRUSTED, True),
        ("robot1", ROS, TRUSTED, True),
        ("tablet", PUSH, UNTRUSTED, True),
    ])
    assert real_viewpoint_ids(reg) == ["relay-cam", "phone"]


def test_untrusted_는_pull_이어도_안_센다():
    reg = _registry([("sketchy", PULL, UNTRUSTED, True)])
    assert real_viewpoint_ids(reg) == []


def test_끊긴_소스는_안_센다():
    reg = _registry([("relay-cam", LOCAL, TRUSTED, True),
                     ("phone", PULL, TRUSTED, False)])
    assert real_viewpoint_ids(reg) == ["relay-cam"]


# ---- probe 는 수요가 아니다 (MCV-1C) ----------------------------------------------

def test_status_는_probe_를_쓰고_provider_를_부르지_않는다():
    """lazy 카메라의 provider 는 수요다. /api/safety 폴링이 카메라를 켜면 안 된다."""
    calls = {"provider": 0, "probe": 0}

    def provider():
        calls["provider"] += 1
        return LIVE, 1000.0, True

    def probe():
        calls["probe"] += 1
        return None, 1000.0, True

    src = Source("relay-cam", provider, LOCAL, TRUSTED, probe=probe)
    reg = SourceRegistry(); reg.register(src)
    assert real_viewpoint_ids(reg) == ["relay-cam"]
    for _ in range(5):
        src.status()
    assert calls["provider"] == 0, "상태 조회가 수요를 만들었다"
    assert calls["probe"] >= 6
    src.latest_jpeg()
    assert calls["provider"] == 1, "프레임을 뽑아가는 것은 수요다"


def test_probe_가_없으면_provider_로_상태를_본다():
    src = Source("phone", _provider(True), PULL, TRUSTED)
    assert src.status()["connected"] is True


# ---- 심각도 -----------------------------------------------------------------

def test_관측_중_시점_하나면_safety_다():
    """관측 세션이 켜져 있는데 시점이 모자라면 안전 기능이 실제로 죽은 것이다."""
    reg = _registry([("relay-cam", LOCAL, TRUSTED, True)])
    r = readiness(reg, robot_publishers=0, observing=True)
    codes = {d["code"]: d for d in r["degradations"]}
    assert codes["VIEWPOINTS_INSUFFICIENT"]["severity"] == "safety"
    assert codes["VIEWPOINTS_INSUFFICIENT"]["have"] == 1
    assert codes["VIEWPOINTS_INSUFFICIENT"]["need"] == MIN_REAL_VIEWPOINTS
    assert r["collisionPredictionPossible"] is False
    assert r["worstSeverity"] == "safety"
    assert r["observation"]["active"] is True


def test_관측이_꺼져_있으면_시점_부족은_info_다_모달이_아니다():
    """09-10 실측: --no-camera 기본 기동에서 시점 0 인데 빨강 모달이 상시로 떠 있었다.

    아무도 그 시점에 기대지 않는데 빨강이 상시면 사람이 모달을 무시하게 되고,
    그러면 진짜 빨강도 같이 무시된다. A안의 취지 그대로다.
    """
    reg = _registry([])
    r = readiness(reg, robot_publishers=0)          # observing 기본값 = False
    codes = {d["code"]: d for d in r["degradations"]}
    assert codes["VIEWPOINTS_INSUFFICIENT"]["severity"] == "info"
    assert r["worstSeverity"] != "safety"
    assert r["worstSeverity"] == "degraded", "로봇 데이터 없음(배너)만 남는다"
    assert r["observation"]["active"] is False


def test_관측이_꺼져_있고_로봇도_있으면_info_가_worst_다():
    reg = _registry([])
    r = readiness(reg, robot_publishers=2, pose_publishers=2,
                  observing=False, robot_pose_msgs=5)
    assert r["worstSeverity"] == "info"


def test_로봇_데이터_없음은_degraded_다_safety_가_아니다():
    """충돌 예측은 관측만으로도 된다. 로봇 데이터는 위치 '보정'에 쓴다."""
    reg = _registry([("relay-cam", LOCAL, TRUSTED, True),
                     ("phone", PULL, TRUSTED, True)])
    r = readiness(reg, robot_publishers=0, observing=True)
    codes = {d["code"]: d for d in r["degradations"]}
    assert "VIEWPOINTS_INSUFFICIENT" not in codes
    assert codes["NO_ROBOT_DATA"]["severity"] == "degraded"
    assert r["collisionPredictionPossible"] is True, "시점 2개면 충돌 예측은 된다"
    assert r["poseCorrectionPossible"] is False, "보정은 로봇 데이터가 있어야 한다"
    assert r["worstSeverity"] == "degraded", "모달이 아니라 배너다"


def test_둘_다_갖추면_ok():
    reg = _registry([("relay-cam", LOCAL, TRUSTED, True),
                     ("phone", PULL, TRUSTED, True)])
    # R-1: "로봇이 있다" 는 발행자 수가 아니라 **값이 왔다**로 말해야 한다.
    r = readiness(reg, robot_publishers=3, pose_publishers=3,
                  observing=True, robot_pose_msgs=12)
    assert r["degradations"] == []
    assert r["worstSeverity"] == "ok"
    assert r["collisionPredictionPossible"] is True
    assert r["poseCorrectionPossible"] is True


def test_safety_가_있으면_worst_는_safety_다():
    """빨강과 노랑이 같이 있으면 빨강이 이긴다."""
    reg = _registry([("relay-cam", LOCAL, TRUSTED, True)])
    r = readiness(reg, robot_publishers=0, observing=True)
    sev = {d["severity"] for d in r["degradations"]}
    assert sev == {"safety", "degraded"}
    assert r["worstSeverity"] == "safety"


def test_심각도_순서가_고정돼_있다():
    assert SEVERITY_ORDER == ("ok", "info", "degraded", "safety")


# ---- 문구는 여기서 만들지 않는다 ------------------------------------------------

def test_판정에_사람용_문구가_들어있지_않다():
    """말은 UI 가 소유한다. 백엔드가 문구를 박으면 번역·수정이 코드 배포가 된다."""
    reg = _registry([("relay-cam", LOCAL, TRUSTED, True)])
    for observing in (True, False):
        r = readiness(reg, robot_publishers=0, observing=observing)
        for d in r["degradations"]:
            assert set(d.keys()) <= {"code", "severity", "have", "need", "liveIds",
                                     "publishers"}
            assert "title" not in d and "message" not in d


def test_최소_시점_수는_2다():
    """하나로는 가림도 대조도 안 된다. 이 상수가 흔들리면 안전 판정이 흔들린다."""
    assert MIN_REAL_VIEWPOINTS == 2


# ---- R-1 — 발행자 수가 아니라 **유량**으로 판정한다 (2026-09-12) --------------
#
# 🔴 실측: `/robot1/pose` 가 Publisher 1 인데 18초간 메시지 0 인 동안 이 값이 true 였다.
#    AMCL `update_min_d: 0.05` — 정지한 로봇은 pose 를 안 낸다.

def _two_views():
    return _registry([("relay-cam", LOCAL, TRUSTED, True),
                      ("phone", PULL, TRUSTED, True)])


def test_발행자는_있는데_값이_안_오면_보정_불가다():
    """⭐⭐ 이 절의 존재 이유. 실측 그대로 — 발행자 1, 메시지 0."""
    r = readiness(_two_views(), robot_publishers=1, pose_publishers=1,
                  observing=True, robot_pose_msgs=0)
    assert r["poseCorrectionPossible"] is False
    codes = {d["code"] for d in r["degradations"]}
    assert "POSE_NEVER_RECEIVED" in codes, codes


def test_값이_안_온_것과_발행자가_없는_것은_다른_코드다():
    """⭐ 같은 코드로 묶으면 '시뮬이 안 떴다' 와 '로봇이 서 있다' 를 못 가른다."""
    none_pub = {d["code"] for d in readiness(
        _two_views(), robot_publishers=0, pose_publishers=0,
        observing=True, robot_pose_msgs=0)["degradations"]}
    no_msg = {d["code"] for d in readiness(
        _two_views(), robot_publishers=1, pose_publishers=1,
        observing=True, robot_pose_msgs=0)["degradations"]}
    assert "NO_ROBOT_DATA" in none_pub and "POSE_NEVER_RECEIVED" not in none_pub
    assert "POSE_NEVER_RECEIVED" in no_msg and "NO_ROBOT_DATA" not in no_msg


def test_유량을_안_넘기면_통과가_아니다():
    """⭐⭐ fail-closed. 모르는 것을 가능하다고 하면 이 결함이 그대로 되돌아온다."""
    r = readiness(_two_views(), robot_publishers=3, pose_publishers=3, observing=True)
    assert r["poseCorrectionPossible"] is False
    assert "POSE_FLOW_UNMEASURED" in {d["code"] for d in r["degradations"]}


def test_값이_오면_보정_가능이다():
    r = readiness(_two_views(), robot_publishers=1, pose_publishers=1,
                  observing=True, robot_pose_msgs=1)
    assert r["poseCorrectionPossible"] is True
    assert r["degradations"] == []


def test_충돌_예측은_pose_유량과_무관하다():
    """⭐ 회귀 — 충돌 예측은 관측만으로 된다. pose 를 묶으면 안 된다."""
    r = readiness(_two_views(), robot_publishers=1, observing=True, robot_pose_msgs=0)
    assert r["collisionPredictionPossible"] is True


def test_pose_가용성을_산출물에_싣는다():
    """불린 하나로는 '왜' 를 못 말한다 — 그래서 여기까지 왔다."""
    r = readiness(_two_views(), robot_publishers=1, pose_publishers=1,
                  observing=True, robot_pose_msgs=0)
    pa = r["poseAvailability"]
    assert pa["state"] == "POSE_NEVER_RECEIVED"
    assert pa["publishers"] == 1 and pa["msgs"] == 0


# ---- pose 발행자와 로봇 발행자는 **다른 수량이다** (2026-09-14) ------------------
#
# 🔴 실측: `/robot1/pose` 발행자 0 인데 `/api/safety` 가 `publishers: 1` 을 냈다.
#    호출자가 로봇당 탐침 5토픽(odom·pose·image_raw·compressed·scan)의 **합계**를
#    넘겼고, 그때 살아 있던 것은 `/robot1/scan` 하나였다. 그래서
#    `POSE_NO_PUBLISHER`(pose 를 내는 곳이 없다) 가 `POSE_NEVER_RECEIVED`
#    (발행자는 있는데 로봇이 서 있다) 로 **가려졌다.** 이 파일이 위에서 일부러
#    갈라 놓은 그 두 상태가 상류에서 뭉개져 있었던 것이다.

def test_scan_만_흐르고_pose_발행자가_없으면_POSE_NO_PUBLISHER_다():
    """🔴 두 수량을 다시 뭉개면 빨개진다. 이것이 2026-09-14 현장 상태 그대로다.

    scan 이 9.76 Hz 로 흐르니 로봇 데이터는 있다(NO_ROBOT_DATA 아님).
    그런데 pose 를 내는 곳은 없다 — 그 사실을 그대로 말해야 한다.
    """
    r = readiness(_two_views(), robot_publishers=1, pose_publishers=0,
                  observing=True, robot_pose_msgs=0)
    codes = {d["code"] for d in r["degradations"]}
    assert "NO_ROBOT_DATA" not in codes, "scan 이 흐르는데 '로봇 데이터 없음' 은 거짓이다"
    assert r["poseAvailability"]["state"] == "POSE_NO_PUBLISHER"
    assert r["poseAvailability"]["publishers"] == 0
    assert r["poseCorrectionPossible"] is False


def test_pose_발행자를_안_주면_모르는_것으로_친다():
    """⭐⭐ fail-closed. 안 주면 옛 뜻(합계)으로 답하지 않는다 — 틀린 초록보다 모른다가 낫다."""
    r = readiness(_two_views(), robot_publishers=5, observing=True, robot_pose_msgs=9)
    assert r["poseAvailability"]["state"] == "POSE_PUBLISHERS_UNMEASURED"
    assert r["poseCorrectionPossible"] is False


def test_pose_발행자를_못_쟀으면_0_으로_적지_않는다():
    """못 잰 것을 0(=없다)으로 떨어뜨리면 빨개진다 — 이 레포가 반복해 밟은 함정이다."""
    r = readiness(_two_views(), robot_publishers=1, pose_publishers=None,
                  observing=True, robot_pose_msgs=0)
    assert r["poseAvailability"]["state"] == "POSE_PUBLISHERS_UNMEASURED"
    assert r["poseAvailability"]["publishers"] is None, "None 이어야 한다 — 0 은 다른 주장이다"


# ---- 게이트웨이가 **두 수량을 따로** 넘기는가 (AST) ------------------------------
#
# ⭐ 위 시험들은 readiness() 를 직접 부른다. 그런데 2026-09-14 의 결함은 **호출자**에 있었다 —
#    게이트웨이가 탐침 5토픽 합계 하나를 두 자리에 다 썼다. 그 자리를 따로 본다.
# ⚠️ gateway_web_server.py 는 rclpy 를 module-level 로 import 하므로 여기서 import 할 수 없다.
#    **AST 로 읽는다.** 부분문자열로 보면 주석에 걸린다.

def test_게이트웨이가_pose_발행자를_따로_넘긴다():
    """🔴 두 자리에 같은 식을 넣으면 빨개진다 — 그게 2026-09-14 의 결함 그대로다."""
    import ast, io, os
    gw = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "gateway_web", "gateway_web_server.py")
    tree = ast.parse(io.open(gw, encoding="utf-8").read())
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call)
             and getattr(n.func, "id", None) == "readiness"]
    assert len(calls) == 1, "readiness 호출은 한 곳이어야 한다. 실제 %d" % len(calls)
    kw = {k.arg: k.value for k in calls[0].keywords}
    assert "robot_publishers" in kw, "로봇 발행자를 안 넘긴다"
    assert "pose_publishers" in kw, "pose 발행자를 따로 안 넘긴다 — 옛 결함이 되살아났다"
    assert ast.dump(kw["robot_publishers"]) != ast.dump(kw["pose_publishers"]), \
        "두 자리에 **같은 식**을 넣었다 — 수량을 다시 뭉갰다"


def test_get_link_status_가_못_잰_pose_발행자를_0_으로_안_적는다():
    """🔴 `else 0` 으로 바꾸면 빨개진다.

    `count_publishers` 가 예외를 내면 그 토픽의 수는 **모른다**. 0 으로 적으면
    "pose 를 내는 곳이 없다" 는 **다른 주장**이 되고, 아래 fail-closed 경로가
    통째로 무력해진다(모르는 것이 '없다' 로 둔갑해 판정이 확신 있게 틀린다).

    ⚠️ 뮤테이션 7개 중 이 변이 하나만 안 잡혀서 뒤늦게 더한 시험이다 —
       `get_link_status` 는 rclpy 때문에 import 가 안 되므로 AST 로 본다.
    """
    import ast, io, os
    gw = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "gateway_web", "gateway_web_server.py")
    tree = ast.parse(io.open(gw, encoding="utf-8").read())
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for k, v in zip(node.keys, node.values):
            if isinstance(k, ast.Constant) and k.value == "pose_publishers":
                found.append(v)
    assert len(found) == 1, "'pose_publishers' 를 싣는 자리는 하나여야 한다. 실제 %d" % len(found)
    v = found[0]
    assert isinstance(v, ast.IfExp), "못 잰 경우를 가르지 않는다 (조건식이 아니다)"
    assert isinstance(v.orelse, ast.Constant) and v.orelse.value is None, \
        "못 쟀을 때 None 이 아니다 — 0 으로 떨어뜨리면 '없다' 라는 다른 주장이 된다"
