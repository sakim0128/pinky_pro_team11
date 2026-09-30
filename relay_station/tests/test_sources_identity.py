# -*- coding: utf-8 -*-
"""R-3 · `tablet-relay` 거짓 이름을 **주소 → 신원**으로 고친다 + 영상 소스를 안 켜는 설정.

09-23 신원 실측: 현장망 `.2` 는 Note 3 Neo 였고 Y700 은 `.4` 였다. 퓰러는 발견이 거부되면 고정 후보로
넘어가므로(fail-open) Y700 앱이 꺼지자 옛 주소 `.2` 에 붙어 "태블릿 y700" 이름으로 Note 3 Neo 영상을 냈다
(phone2 와 발행 세션·프레임 번호까지 같았다). DHCP 는 주소를 다른 기기에 다시 준다 — 주소는 신원이 아니다.
"""
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "relay_station", "gateway_web"))

import mjpeg_puller as mp

SOURCES = os.path.join(REPO, "relay_station", "configs", "video_sources.json")
NONE = os.path.join(REPO, "relay_station", "configs", "video_sources.none.json")
FIELD_LAN = re.compile(r"//192\.168\.0\.\d+[:/]")      # 현장망 — DHCP 가 주소를 돌려 쓴다


def _by_id(path):
    return {s["id"]: s for s in mp.load_pull_sources(path)}


def test_tablet_relay_는_신원으로만_찾는다():
    s = _by_id(SOURCES)["tablet-relay"]
    assert s["urls"] == [], "주소 후보가 남아 있으면 발견이 거부될 때 다른 기기에 붙는다"
    assert s["discover"] and s["discover"]["tailnetIp"]


def test_어떤_소스도_현장망_주소를_고정_후보로_갖지_않는다():
    bad = {i: [u for u in s["urls"] if FIELD_LAN.search(u)] for i, s in _by_id(SOURCES).items()}
    assert not any(bad.values()), bad


def test_영상_소스를_안_켜는_설정은_소스가_0개():
    assert mp.load_pull_sources(NONE) == []
