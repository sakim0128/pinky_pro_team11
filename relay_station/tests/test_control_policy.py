# -*- coding: utf-8 -*-
"""제어권 정책(`gateway_web/control_policy.py`) — 순수 파이썬이라 ROS 없이 어디서나 돈다.

관제 2026-09-28: 팀원 노트북이 중계를 거쳐 움직이는 명령을 낼 수 있게 하되, 기본은 닫힘 · 허용 목록은 파일 ·
한 번에 한 사람 · 로컬이 언제나 이김 · 멈추는 명령은 정책 밖.
"""
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
GW = os.path.join(os.path.dirname(HERE), "gateway_web")
if GW not in sys.path:
    sys.path.insert(0, GW)

import control_policy as cp  # noqa: E402

LOCAL = ("127.0.0.1", "::1", "localhost", "198.51.100.3")


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def _allow(tmp_path, rows, ttl=None, name="control_allow.json"):
    p = tmp_path / name
    doc = {"controllers": rows}
    if ttl is not None:
        doc["ttl_s"] = ttl
    p.write_text(json.dumps(doc, ensure_ascii=False), encoding="utf-8")
    return str(p)


def _policy(tmp_path, rows=None, ttl=None, path=None):
    clock = Clock()
    if path is None:
        path = _allow(tmp_path, rows or [], ttl) if rows is not None else str(tmp_path / "missing.json")
    return cp.ControlPolicy(LOCAL, cp.AllowList(path), clock=clock), clock


# ---- 기본은 닫힘 --------------------------------------------------------------------------------------------------

def test_파일이_없으면_로컬만_움직이고_남은_전부_보기_전용(tmp_path):
    pol, _ = _policy(tmp_path)                      # missing.json
    assert pol.may_move("127.0.0.1")[0] is True
    ok, code, _ = pol.may_move("198.51.100.21")
    assert (ok, code) == (False, "FORBIDDEN") and pol.http_code(code) == 403
    st = pol.status("198.51.100.21")
    assert st["view_only"] is True and st["allow_count"] == 0 and st["allow_error"] is None


def test_enabled_가_아닌_항목은_없는_것이다(tmp_path):
    pol, _ = _policy(tmp_path, [{"ip": "198.51.100.21", "name": "A", "enabled": False},
                                {"ip": "198.51.100.22", "name": "B"}])
    assert pol.may_move("198.51.100.21")[1] == "FORBIDDEN" and pol.may_move("198.51.100.22")[1] == "FORBIDDEN"
    assert pol.status("198.51.100.21")["allow_count"] == 0


def test_깨진_파일은_닫힘이고_이유를_말한다(tmp_path):
    p = tmp_path / "control_allow.json"
    p.write_text("{not json", encoding="utf-8")
    pol = cp.ControlPolicy(LOCAL, cp.AllowList(str(p)), clock=Clock())
    assert pol.may_move("198.51.100.21")[1] == "FORBIDDEN"
    st = pol.status("198.51.100.21")
    assert st["view_only"] is True and st["allow_error"] and "JSONDecodeError" in st["allow_error"]
    assert pol.may_move("127.0.0.1")[0] is True    # 로컬은 그래도 된다


# ---- 허용 목록 · 한 번에 한 사람 ------------------------------------------------------------------------------------

def test_허용_주소의_첫_움직이는_명령이_제어권을_잡고_다른_허용_주소는_409(tmp_path):
    pol, clock = _policy(tmp_path, [{"ip": "198.51.100.21", "name": "팀원 A", "enabled": True},
                                    {"ip": "198.51.100.22", "name": "팀원 B", "enabled": True}])
    ok, code, msg = pol.may_move("198.51.100.21")
    assert (ok, code) == (True, "ALLOWED") and "팀원 A" in msg
    ok, code, msg = pol.may_move("198.51.100.22")
    assert (ok, code) == (False, "CONTROL_HELD") and pol.http_code(code) == 409 and "팀원 A" in msg and "198.51.100.21" in msg
    st_a, st_b = pol.status("198.51.100.21"), pol.status("198.51.100.22")
    assert st_a["mine"] is True and st_b["mine"] is False and st_b["holder"] == "팀원 A" and st_b["view_only"] is False
    # 쥔 쪽의 다음 명령은 계속 된다
    assert pol.may_move("198.51.100.21")[0] is True


def test_쥔_쪽이_조용하면_ttl_뒤_만료되고_다음_사람이_잡는다(tmp_path):
    pol, clock = _policy(tmp_path, [{"ip": "198.51.100.21", "name": "A", "enabled": True},
                                    {"ip": "198.51.100.22", "name": "B", "enabled": True}], ttl=30)
    assert pol.may_move("198.51.100.21")[0]
    clock.t += 29
    assert pol.may_move("198.51.100.22")[1] == "CONTROL_HELD"
    pol.touch("198.51.100.21")                      # 화면 폴링 = 살아 있음
    clock.t += 29
    assert pol.may_move("198.51.100.22")[1] == "CONTROL_HELD"   # touch 로 연장됐다
    clock.t += 31
    ok, code, _ = pol.may_move("198.51.100.22")
    assert (ok, code) == (True, "ALLOWED") and pol.status("198.51.100.22")["mine"] is True


def test_touch_는_쥔_쪽만_연장한다(tmp_path):
    pol, clock = _policy(tmp_path, [{"ip": "198.51.100.21", "name": "A", "enabled": True},
                                    {"ip": "198.51.100.22", "name": "B", "enabled": True}], ttl=30)
    pol.may_move("198.51.100.21")
    clock.t += 20
    pol.touch("198.51.100.22")                      # 남이 폴링해도 연장 안 됨
    clock.t += 15
    assert pol.may_move("198.51.100.22")[0] is True


def test_놓기는_쥔_쪽이나_로컬만(tmp_path):
    pol, _ = _policy(tmp_path, [{"ip": "198.51.100.21", "name": "A", "enabled": True},
                                {"ip": "198.51.100.22", "name": "B", "enabled": True}])
    pol.may_move("198.51.100.21")
    assert pol.release("198.51.100.22")[1] == "CONTROL_HELD"
    assert pol.release("198.51.100.21")[1] == "RELEASED" and pol.status("198.51.100.21")["holder"] is None
    pol.may_move("198.51.100.22")
    assert pol.release("127.0.0.1")[1] == "RELEASED"          # 로컬은 남의 것도 놓는다


def test_acquire_는_이름을_붙이고_남이_쥐면_409(tmp_path):
    pol, _ = _policy(tmp_path, [{"ip": "198.51.100.21", "name": "A", "enabled": True},
                                {"ip": "198.51.100.22", "name": "B", "enabled": True}])
    assert pol.acquire("198.51.100.21", name="A 노트북(민수)")[1] == "ACQUIRED"
    assert pol.status("198.51.100.22")["holder"] == "A 노트북(민수)"
    assert pol.acquire("198.51.100.22")[1] == "CONTROL_HELD"
    assert pol.acquire("203.0.113.9")[1] == "FORBIDDEN"


# ---- 로컬은 언제나 이긴다 --------------------------------------------------------------------------------------------

def test_로컬_콘솔의_움직이는_명령은_제어권을_가져온다(tmp_path):
    pol, _ = _policy(tmp_path, [{"ip": "198.51.100.21", "name": "A", "enabled": True}])
    assert pol.may_move("198.51.100.21")[0]
    ok, code, _ = pol.may_move("127.0.0.1")
    assert (ok, code) == (True, "LOCAL")
    st = pol.status("198.51.100.21")
    assert st["holder"] == cp.LOCAL_NAME and st["mine"] is False
    assert pol.may_move("198.51.100.21")[1] == "CONTROL_HELD"     # 팀원은 이제 막힌다(로컬이 쥠)
    assert pol.release("127.0.0.1")[0] and pol.may_move("198.51.100.21")[0]


# ---- 파일을 다시 읽는다(재기동 없이) ------------------------------------------------------------------------------------

def test_파일을_고치면_재기동_없이_먹는다(tmp_path):
    p = _allow(tmp_path, [])
    al = cp.AllowList(p)
    pol = cp.ControlPolicy(LOCAL, al, clock=Clock())
    assert pol.may_move("198.51.100.21")[1] == "FORBIDDEN"
    import time
    time.sleep(0.02)
    with open(p, "w", encoding="utf-8") as fh:
        json.dump({"ttl_s": 45, "controllers": [{"ip": "198.51.100.21", "name": "A", "enabled": True}]}, fh)
    os.utime(p, None)
    assert pol.may_move("198.51.100.21")[0] is True
    assert pol.ttl_s() == 45.0


def test_상태에는_출처와_개수와_남은_시간이_있다(tmp_path):
    pol, clock = _policy(tmp_path, [{"ip": "198.51.100.21", "name": "A", "enabled": True}], ttl=30)
    pol.may_move("198.51.100.21")
    clock.t += 10
    st = pol.status("127.0.0.1")
    assert st["allow_source"].startswith("file:") and st["allow_count"] == 1 and st["ttl_s"] == 30.0
    assert st["holder_ttl_s"] == 20.0 and st["controller_name"] == cp.LOCAL_NAME and st["view_only"] is False


def test_기본_설정_파일은_enabled_가_하나도_없다():
    """레포에 실린 configs/control_allow.json 은 닫힌 채 나간다(fail-closed) — 자리표시자 주소만."""
    relay = os.path.dirname(HERE)                       # relay_station/
    repo = os.path.dirname(relay)                       # 원 저장소 루트 (팀11 export 에서는 relay_station/configs 가 맞는다)
    for cand in (os.path.join(repo, "configs", "control_allow.json"),
                 os.path.join(relay, "configs", "control_allow.json")):
        if os.path.exists(cand):
            break
    else:
        pytest.fail("control_allow.json 이 configs/ 에 없다")
    doc = json.load(open(cand, encoding="utf-8"))
    assert all(not r.get("enabled") for r in doc["controllers"])
    assert all(r["ip"].startswith("198.51.100.") for r in doc["controllers"])
    assert doc.get("ttl_s") == 30
