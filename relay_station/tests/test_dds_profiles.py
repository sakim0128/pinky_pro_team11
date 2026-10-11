# -*- coding: utf-8 -*-
"""DDS 프로파일 — 장소가 바뀌어도 관제가 뜨는지를 고정한다.

2026-09-09 집에서 런처를 올렸더니 게이트웨이가 통째로 죽었다:

    python3: enx001122334455: does not match an available interface.
    rmw_create_node: failed to create domain  ->  RCLError  ->  trap cleanup

원인은 코드가 아니라 **설정이 장소에 묶여 있던 것**이다. 현장 설정은 교육장 유선 NIC 에
바인딩을 박아 두는데, 집에는 그 어댑터가 없다. 그래서 프로파일을 둘로 나눴다.

    ip link show enx001122334455
        ├─ 있음 ──> configs/cyclonedds.xml           (현장. 한 바이트도 안 바꿨다)
        └─ 없음 ──> configs/cyclonedds-offsite.xml   (autodetermine + loopback peers)

이 파일이 지키는 것: 누가 offsite 설정에 현장 NIC 을 다시 박거나, 런처가 분기를
잃어버리면 실패한다.
"""
import os
import re
import xml.etree.ElementTree as ET

import pytest

NS = {"c": "https://cdds.io/config"}
FIELD_NIC = "enx001122334455"

_RELAY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(_RELAY)
CONFIGS = os.path.join(_REPO, "relay_station", "configs")
FIELD_XML = os.path.join(CONFIGS, "cyclonedds.xml")
OFFSITE_XML = os.path.join(CONFIGS, "cyclonedds-offsite.xml")
LAUNCHER = os.path.join(_RELAY, "launch_master_gateway.sh")


def _interfaces(path):
    root = ET.parse(path).getroot()
    return root.findall(".//c:Interfaces/c:NetworkInterface", NS)


def _peers(path):
    root = ET.parse(path).getroot()
    return [p.get("address") for p in root.findall(".//c:Peers/c:Peer", NS)]


@pytest.mark.parametrize("path", [FIELD_XML, OFFSITE_XML])
def test_두_프로파일_모두_존재하고_파싱된다(path):
    assert os.path.exists(path), "%s 가 없다" % path
    ET.parse(path)


def test_현장_프로파일은_현장_nic_에_바인딩한다():
    """현장 동작을 바꾸지 않았다는 것을 고정한다."""
    names = [n.get("name") for n in _interfaces(FIELD_XML)]
    assert names == [FIELD_NIC]


def test_offsite_프로파일은_현장_nic_에_바인딩하지_않는다():
    """주석이 사고 이력을 설명하는 건 좋다. **설정값**으로 들어오면 집에서 또 죽는다.

    그래서 원문 문자열이 아니라 파싱된 속성·텍스트만 본다 — 검사 대상은 '언급'이
    아니라 '바인딩'이다.
    """
    root = ET.parse(OFFSITE_XML).getroot()
    for el in root.iter():
        for key, value in el.attrib.items():
            assert FIELD_NIC not in str(value), \
                "%s 의 %s 속성에 현장 NIC 이 박혀 있다" % (el.tag, key)
        assert FIELD_NIC not in (el.text or ""), "%s 텍스트에 현장 NIC 이 있다" % el.tag


def test_offsite_프로파일은_인터페이스를_자동으로_고른다():
    ifaces = _interfaces(OFFSITE_XML)
    assert len(ifaces) == 1
    assert ifaces[0].get("autodetermine") == "true"
    assert ifaces[0].get("name") is None, "이름을 박으면 자동 선택이 아니다"


def test_offsite_는_남의_네트워크로_나가지_않는다():
    """현장 밖에는 로봇이 없다. 집 공유기에 DDS 를 흘리지 않는다."""
    assert _peers(OFFSITE_XML) == ["localhost"]


def test_현장_프로파일은_로봇들을_지목한다():
    peers = _peers(FIELD_XML)
    for robot_ip in ("198.51.100.5", "198.51.100.6"):
        assert robot_ip in peers


@pytest.mark.parametrize("path", [FIELD_XML, OFFSITE_XML])
def test_도메인은_any_여야_브릿지가_여러_도메인에_참여한다(path):
    root = ET.parse(path).getroot()
    domains = root.findall("c:Domain", NS)
    assert len(domains) == 1
    assert domains[0].get("id") == "any"


def test_런처가_nic_유무로_프로파일을_고른다():
    src = open(LAUNCHER, encoding="utf-8").read()
    # 2026-09-25 관제 검수 P3: bridge_env.sh 처럼 환경변수로 바꿀 수 있다 — 기본값은 같은 NIC 여야 한다
    assert 'FIELD_NIC="${FIELD_NIC:-%s}"' % FIELD_NIC in src
    assert re.search(r'if\s+ip\s+link\s+show\s+"\$FIELD_NIC"', src), \
        "런처가 NIC 존재를 확인하지 않는다"
    assert "cyclonedds.xml" in src and "cyclonedds-offsite.xml" in src, \
        "두 프로파일 중 하나만 참조한다"
    assert "DDS_PROFILE=" in src, "어느 프로파일로 떴는지 로그에 남지 않는다"


def test_런처가_고른_프로파일을_진단출력에_보여준다():
    """어느 프로파일로 떴는지 사람이 볼 수 없으면 또 오진한다."""
    src = open(LAUNCHER, encoding="utf-8").read()
    assert "$DDS_PROFILE" in src
