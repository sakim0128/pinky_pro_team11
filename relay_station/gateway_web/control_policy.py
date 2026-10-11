# -*- coding: utf-8 -*-
"""제어권 정책 — 누가 '움직이는' 명령(플릿 시작·재개·배정, 로봇 재개, 좌표 전환, 목표·미션)을 낼 수 있나.

관제 2026-09-28 (사용자 결정: "중계를 거치되 팀원 노트북에서도 명령을 낼 수 있게").

원칙
- **기본은 닫힘.** 허용 목록 파일(`configs/control_allow.json`)이 없거나 깨졌거나 비어 있으면 예전과 같다 —
  중계 노트북(로컬) 만 움직이는 명령을 낼 수 있다. 환경변수로는 열리지 않는다(현장 장비에서 누가 켜면 그대로 구멍).
- **파일은 운영자가 일부러 고친다.** 재기동 없이 먹는다(mtime 을 보고 다시 읽는다) — 게이트웨이 재기동은 로봇을 래치시키므로.
- **한 번에 한 사람.** 허용된 주소 중 처음 움직이는 명령을 낸 쪽이 제어권을 쥔다. 다른 허용 주소의 움직이는 명령은 409
  `CONTROL_HELD` 다(누가 쥐었는지 이름으로 말한다). 쥔 쪽이 `ttl_s`(기본 30 s) 동안 아무 요청도 안 하면 만료된다 —
  V2 화면은 2 s 마다 /api/status 를 부르므로 화면이 열려 있는 한 유지되고, 노트북을 닫으면 30 s 뒤 풀린다.
- **로컬 콘솔은 언제나 이긴다.** 중계 노트북에서 낸 움직이는 명령은 제어권을 가져온다(팀원 화면은 "제어권: 중계 노트북" 으로 바뀐다).
- **멈추는 명령은 이 정책을 거치지 않는다.** 일시정지·비상정지·로봇 정지는 어디서든(OPS-1·D7 그대로).

순수 파이썬 — ROS·HTTP 에 안 기댄다. 게이트웨이(`gateway_web_server.py`)가 요청마다 이 객체에 묻는다.
"""
from __future__ import annotations

import json
import os
import time
from typing import Any, Callable, Dict, Optional, Tuple

DEFAULT_TTL_S = 30.0
LOCAL_NAME = "중계 노트북(로컬)"

Verdict = Tuple[bool, str, str]     # (ok, code, message) — code: LOCAL · ALLOWED · CONTROL_HELD · FORBIDDEN · ...


class AllowList:
    """허용 목록 파일. mtime 이 바뀌면 다시 읽는다. 없거나 깨지면 **빈 목록**(fail-closed) + 오류 문구."""

    def __init__(self, path: Optional[str]):
        self.path = path
        self._mtime: Optional[float] = None
        self._entries: Dict[str, str] = {}      # ip -> name (enabled 만)
        self._ttl_s: Optional[float] = None
        self.error: Optional[str] = None

    def _load(self) -> None:
        if not self.path:
            self._entries, self.error = {}, None
            return
        try:
            mtime = os.stat(self.path).st_mtime
        except OSError:
            self._entries, self._mtime, self.error = {}, None, None      # 파일 없음 = 닫힘, 오류는 아니다
            return
        if mtime == self._mtime:
            return
        self._mtime = mtime
        try:
            with open(self.path, encoding="utf-8") as fh:
                doc = json.load(fh)
            rows = doc.get("controllers") or []
            entries: Dict[str, str] = {}
            for row in rows:
                if not isinstance(row, dict) or not row.get("enabled"):
                    continue
                ip = str(row.get("ip") or "").strip()
                if not ip:
                    continue
                entries[ip] = str(row.get("name") or ip)
            ttl = doc.get("ttl_s")
            self._ttl_s = float(ttl) if isinstance(ttl, (int, float)) and ttl > 0 else None
            self._entries, self.error = entries, None
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            self._entries, self.error = {}, "%s: %s" % (type(exc).__name__, exc)   # 깨진 파일 = 닫힘 + 이유

    def entries(self) -> Dict[str, str]:
        self._load()
        return dict(self._entries)

    def ttl_s(self) -> Optional[float]:
        self._load()
        return self._ttl_s

    def source(self) -> str:
        return "file:%s" % self.path if self.path else "none"


class ControlPolicy:
    def __init__(self, local_ips, allow: AllowList, ttl_s: float = DEFAULT_TTL_S,
                 clock: Callable[[], float] = time.monotonic):
        self.local_ips = tuple(local_ips)
        self.allow = allow
        self._ttl_s = float(ttl_s)
        self.clock = clock
        self.holder: Optional[Dict[str, Any]] = None   # {ip, name, since, last_seen}

    # ---- 조회 ------------------------------------------------------------------------------------------------
    def ttl_s(self) -> float:
        return self.allow.ttl_s() or self._ttl_s

    def is_local(self, ip: str) -> bool:
        return ip in self.local_ips

    def allowed_name(self, ip: str) -> Optional[str]:
        """허용 목록에 있으면 그 이름, 로컬이면 LOCAL_NAME, 아니면 None."""
        if self.is_local(ip):
            return LOCAL_NAME
        return self.allow.entries().get(ip)

    def _expired(self) -> bool:
        return bool(self.holder) and (self.clock() - self.holder["last_seen"]) > self.ttl_s()

    def _current_holder(self) -> Optional[Dict[str, Any]]:
        if self.holder and self._expired():
            self.holder = None
        return self.holder

    def touch(self, ip: str) -> None:
        """어느 요청이든 — 쥔 쪽이면 살아 있다고 본다(화면 폴링이 제어권을 유지한다)."""
        h = self._current_holder()
        if h and h["ip"] == ip:
            h["last_seen"] = self.clock()

    # ---- 결정 ------------------------------------------------------------------------------------------------
    def _take(self, ip: str, name: str) -> None:
        now = self.clock()
        self.holder = {"ip": ip, "name": name, "since": now, "last_seen": now}

    def acquire(self, ip: str, name: Optional[str] = None) -> Verdict:
        """제어권 잡기. 로컬은 언제나, 허용 주소는 비어 있거나 만료됐거나 자기 것일 때."""
        who = self.allowed_name(ip)
        if who is None:
            return False, "FORBIDDEN", "허용 목록에 없는 주소다 — 중계 운영자가 configs/control_allow.json 에 넣어야 한다"
        label = name.strip() if name and name.strip() and not self.is_local(ip) else who
        h = self._current_holder()
        if self.is_local(ip) or h is None or h["ip"] == ip:
            self._take(ip, label)
            return True, "ACQUIRED", "제어권: %s" % label
        return False, "CONTROL_HELD", "제어권을 %s(%s) 이 쥐고 있다 — 놓거나 %d s 뒤 만료" % (h["name"], h["ip"], int(self.ttl_s()))

    def release(self, ip: str) -> Verdict:
        h = self._current_holder()
        if h is None:
            return True, "RELEASED", "제어권 없음"
        if h["ip"] == ip or self.is_local(ip):
            self.holder = None
            return True, "RELEASED", "제어권 놓음(%s)" % h["name"]
        return False, "CONTROL_HELD", "제어권은 %s(%s) 것이다 — 남의 것은 놓을 수 없다" % (h["name"], h["ip"])

    def may_move(self, ip: str) -> Verdict:
        """움직이는 명령을 내도 되나. 되면 제어권을 (없으면) 잡고 살아 있음을 갱신한다."""
        who = self.allowed_name(ip)
        if who is None:
            return False, "FORBIDDEN", "안전 정책: 이 주소는 움직이는 명령을 낼 수 없다(허용 목록 밖). 멈추는 명령은 된다"
        h = self._current_holder()
        if self.is_local(ip):
            if h is None or h["ip"] != ip:
                self._take(ip, LOCAL_NAME)            # 로컬 콘솔이 가져간다 — 팀원 화면에 그렇게 보인다
            else:
                h["last_seen"] = self.clock()
            return True, "LOCAL", "중계 노트북(로컬)"
        if h is None:
            self._take(ip, who)
            return True, "ALLOWED", "제어권 잡음: %s" % who
        if h["ip"] == ip:
            h["last_seen"] = self.clock()
            return True, "ALLOWED", "제어권: %s" % who
        return False, "CONTROL_HELD", "제어권을 %s(%s) 이 쥐고 있다 — 그쪽이 놓거나 %d s 동안 조용하면 만료" % (h["name"], h["ip"], int(self.ttl_s()))

    def http_code(self, code: str) -> int:
        return {"FORBIDDEN": 403, "CONTROL_HELD": 409}.get(code, 200)

    # ---- 화면용 -------------------------------------------------------------------------------------------------
    def status(self, ip: str) -> Dict[str, Any]:
        h = self._current_holder()
        who = self.allowed_name(ip)
        entries = self.allow.entries()
        return {
            "view_only": who is None,                               # 움직이는 조작을 낼 수 없는 주소
            "controller_name": who,                                 # 이 요청 주소의 이름(허용 목록/로컬), 없으면 None
            "holder": h["name"] if h else None,
            "holder_ip": h["ip"] if h else None,
            "mine": bool(h) and h["ip"] == ip,
            "holder_ttl_s": (round(self.ttl_s() - (self.clock() - h["last_seen"]), 1) if h else None),
            "ttl_s": self.ttl_s(),
            "allow_source": self.allow.source(),
            "allow_count": len(entries),
            "allow_error": self.allow.error,
        }
