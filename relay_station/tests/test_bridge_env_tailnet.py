# -*- coding: utf-8 -*-
"""R-2 · tailnet DDS 프로파일 — `configs/cyclonedds-tailnet.xml` + `RELAY_DDS_PROFILE` 세 번째 분기.

지시: `docs/REQ_20260924_RELAY_SESSION.md` R-2 (`tailscale0` 바인딩, 멀티캐스트 off,
peer = localhost + `${FARM_HOST_TAILNET_IP}` env) + `bridge_env.sh` 세 번째 분기 + 시험.

⭐ 설계 판단 — tailnet 은 **명시할 때만** 탄다. 자동으로 고르면 중계가 예고 없이 팜의 DDS 에 합류한다.
   명시했는데 조건(`tailscale0`·팜 주소)이 없으면 다른 프로파일로 조용히 떨어지지 않고 78 로 멈춘다.
⚠️ 기존 `test_bridge_env` 의 가짜 `ip` 는 어떤 NIC 를 물어도 같은 답을 준다. 여기서는 NIC 이름별로
   있고 없음을 흉내 낸다 — 현장 NIC 는 없고 tailscale0 은 있는 경우를 재야 하므로.
"""
import os
import re
import shutil
import stat
import subprocess
import xml.etree.ElementTree as ET

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ENV_SH = os.path.join(REPO, "relay_station", "domain_bridge", "bridge_env.sh")
LAUNCHER = os.path.join(REPO, "relay_station", "launch_master_gateway.sh")
XML = os.path.join(REPO, "relay_station", "configs", "cyclonedds-tailnet.xml")
FIELD = "enx001122334455"
FAKE_FARM = "127.0.0.2"          # 시험용. 실제 tailnet 주소는 레포에 두지 않는다


def _run(tmp_path, present, **env_over):
    shim = tmp_path / "shim"
    shim.mkdir(exist_ok=True)
    ip = shim / "ip"
    # `ip link show NAME` — NAME 이 PRESENT_NICS 에 있을 때만 0
    ip.write_text('#!/bin/sh\ncase ":$PRESENT_NICS:" in *":$3:"*) exit 0;; esac\nexit 1\n', encoding="utf-8")
    ip.chmod(ip.stat().st_mode | stat.S_IEXEC)
    env = {k: v for k, v in os.environ.items()
           if k not in ("RELAY_DDS_PROFILE", "FARM_HOST_TAILNET_IP", "CYCLONEDDS_URI")}
    env.update(PATH=str(shim) + os.pathsep + env.get("PATH", ""), PRESENT_NICS=":".join(present))
    env.update(env_over)
    r = subprocess.run([shutil.which("bash"), ENV_SH], cwd=REPO, env=env, capture_output=True, timeout=120)
    return r.returncode, r.stdout.decode("utf-8", "replace"), r.stderr.decode("utf-8", "replace")


def _uri(out):
    m = re.search(r"CYCLONEDDS_URI=file://(\S+)", out)
    return os.path.basename(m.group(1)) if m else None


# ---- 선택 규칙 (실제 스크립트를 돌린다) ------------------------------------------

def test_명시한_tailnet_이_조건을_갖추면_tailnet(tmp_path):
    rc, out, err = _run(tmp_path, ["tailscale0"], RELAY_DDS_PROFILE="tailnet", FARM_HOST_TAILNET_IP=FAKE_FARM)
    assert rc == 0, err
    assert _uri(out) == "cyclonedds-tailnet.xml"
    assert "tailnet" in err and FAKE_FARM not in err       # 로그에 주소를 찍지 않는다


def test_tailnet_인데_tailscale0_이_없으면_78(tmp_path):
    rc, _, err = _run(tmp_path, [], RELAY_DDS_PROFILE="tailnet", FARM_HOST_TAILNET_IP=FAKE_FARM)
    assert rc == 78 and "tailscale0" in err


def test_tailnet_인데_팜_주소가_없으면_78__빈_peer_로_조용히_뜨지_않는다(tmp_path):
    rc, _, err = _run(tmp_path, ["tailscale0"], RELAY_DDS_PROFILE="tailnet")
    assert rc == 78 and "FARM_HOST_TAILNET_IP" in err


def test_자동은_tailscale0_이_있어도_tailnet_을_고르지_않는다(tmp_path):
    rc, out, _ = _run(tmp_path, ["tailscale0"], FARM_HOST_TAILNET_IP=FAKE_FARM)
    assert rc == 0 and _uri(out) == "cyclonedds-offsite.xml"


def test_자동은_예전과_같다__현장_NIC_있으면_field(tmp_path):
    rc, out, _ = _run(tmp_path, [FIELD, "tailscale0"])
    assert rc == 0 and _uri(out) == "cyclonedds.xml"


def test_field_를_명시했는데_NIC_이_없으면_78(tmp_path):
    rc, _, err = _run(tmp_path, ["tailscale0"], RELAY_DDS_PROFILE="field")
    assert rc == 78 and FIELD in err


def test_모르는_프로파일은_78(tmp_path):
    rc, _, err = _run(tmp_path, ["tailscale0"], RELAY_DDS_PROFILE="tailnt")
    assert rc == 78 and "auto|field|offsite|tailnet" in err


# ---- 게이트웨이 런처와 같은 규칙 ---------------------------------------------------

def _case_table(path):
    """`case "$_profile_req" in … esac` 에서 라벨 → 고르는 xml 파일들."""
    src = open(path, encoding="utf-8").read()
    body = src[src.index('case "$_profile_req" in'):]
    body = body[:body.index("esac")]
    table, label = {}, None
    for line in body.splitlines():
        m = re.match(r"\s*([a-z*|]+)\)\s*$", line)
        if m:
            label = m.group(1)
            table[label] = []
            continue
        if label:
            table[label] += re.findall(r"cyclonedds[-a-z]*\.xml", line)
    return {k: sorted(v) for k, v in table.items()}


def test_브리지와_게이트웨이_런처가_같은_규칙을_갖는다():
    b, g = _case_table(ENV_SH), _case_table(LAUNCHER)
    assert b == g, (b, g)
    assert b["tailnet"] == ["cyclonedds-tailnet.xml"]
    assert b["auto"] == ["cyclonedds-offsite.xml", "cyclonedds.xml"]


# ---- 프로파일 파일 --------------------------------------------------------------

def test_프로파일은_tailscale0_멀티캐스트off_peer는_localhost와_환경변수():
    ns = {"c": "https://cdds.io/config"}
    root = ET.parse(XML).getroot()
    nics = [e.get("name") for e in root.iter("{https://cdds.io/config}NetworkInterface")]
    assert nics == ["tailscale0"]
    assert root.find(".//c:AllowMulticast", ns).text.strip() == "false"
    peers = [e.get("address") for e in root.iter("{https://cdds.io/config}Peer")]
    assert peers == ["localhost", "${FARM_HOST_TAILNET_IP}"]


def test_프로파일_파일에_주소가_박혀_있지_않다():
    """tailnet 주소는 홈랩 식별자 — 공개 레포 금지(지시서 규칙)."""
    text = open(XML, encoding="utf-8").read()
    assert not re.search(r"\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b", text)


def test_CycloneDDS_가_이_프로파일로_실제_노드를_만든다():
    """문법은 CycloneDDS 에게 묻는다 — 모르는 요소면 'unknown element' 로 도메인 생성이 실패한다."""
    if not os.path.exists("/sys/class/net/tailscale0"):
        pytest.skip("이 기계에 tailscale0 이 없다")
    try:
        import rclpy  # noqa: F401
    except Exception:
        pytest.skip("rclpy 없음")
    env = dict(os.environ, RMW_IMPLEMENTATION="rmw_cyclonedds_cpp", CYCLONEDDS_URI="file://" + XML,
               FARM_HOST_TAILNET_IP=FAKE_FARM, ROS_DOMAIN_ID="97")
    r = subprocess.run(["python3", "-c", "import rclpy; rclpy.init(); n=rclpy.create_node('r2_smoke'); "
                        "print('NODE_OK'); n.destroy_node(); rclpy.shutdown()"],
                       env=env, capture_output=True, text=True, timeout=60)
    assert "NODE_OK" in r.stdout, r.stderr[-800:]


# ---- 관제 검수 후속 (2026-09-25) --------------------------------------------------------------

@pytest.mark.parametrize("form", [
    "RELAY_DDS_PROFILE=tailnet FARM_HOST_TAILNET_IP={ip} source {env}",                 # 접두 대입
    "RELAY_DDS_PROFILE=tailnet; FARM_HOST_TAILNET_IP={ip}; source {env}",               # 셸 변수
    "export RELAY_DDS_PROFILE=tailnet FARM_HOST_TAILNET_IP={ip}; source {env}",         # export
])
def test_팜_주소가_뒤에_뜨는_ros2_의_환경까지_간다(tmp_path, form):
    """관제 검수 미검증 항목: 검사만 하고 export 안 하면 bridge_env 안에선 값이 보이는데
    뒤에 뜨는 ros2 에는 없다 → CycloneDDS 가 ${FARM_HOST_TAILNET_IP} 를 빈 값으로 편다(2026-09-25 실측)."""
    shim = tmp_path / "shim"
    shim.mkdir()
    ip = shim / "ip"
    ip.write_text('#!/bin/sh\ncase "$3" in tailscale0) exit 0;; esac\nexit 1\n', encoding="utf-8")
    ip.chmod(ip.stat().st_mode | stat.S_IEXEC)
    env = {k: v for k, v in os.environ.items() if k not in ("RELAY_DDS_PROFILE", "FARM_HOST_TAILNET_IP")}
    env["PATH"] = str(shim) + os.pathsep + env.get("PATH", "")
    cmd = form.format(ip=FAKE_FARM, env=ENV_SH) + " >/dev/null 2>&1; env | grep '^FARM_HOST_TAILNET_IP=' || echo MISSING"
    r = subprocess.run([shutil.which("bash"), "-c", cmd], cwd=REPO, env=env, capture_output=True, text=True, timeout=60)
    assert r.stdout.strip() == "FARM_HOST_TAILNET_IP=" + FAKE_FARM, (form, r.stdout, r.stderr[-400:])


def _launcher_dds_block():
    """게이트웨이 런처의 DDS 프로파일 선택 부분만 떼어 낸다(런처 전체는 게이트웨이를 띄운다)."""
    src = open(LAUNCHER, encoding="utf-8").read()
    start = src.index("# bridge_env.sh 와 같은 규칙 — 환경변수로")
    end = src.index("export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET", start)
    return src[start:end] + 'echo "URI=$CYCLONEDDS_URI"\n'


def _run_launcher_block(tmp_path, present, with_xml, xml_nic=FIELD, **env_over):
    shim = tmp_path / "shim"
    shim.mkdir(exist_ok=True)
    ip = shim / "ip"
    ip.write_text('#!/bin/sh\ncase ":$PRESENT_NICS:" in *":$3:"*) exit 0;; esac\nexit 1\n', encoding="utf-8")
    ip.chmod(ip.stat().st_mode | stat.S_IEXEC)
    root = tmp_path / "repo"
    (root / "configs").mkdir(parents=True, exist_ok=True)
    if with_xml:
        for n in ("cyclonedds-offsite.xml", "cyclonedds-tailnet.xml"):
            (root / "configs" / n).write_text("<CycloneDDS/>", encoding="utf-8")
        (root / "configs" / "cyclonedds.xml").write_text(
            '<CycloneDDS><NetworkInterface name="%s" /></CycloneDDS>' % xml_nic, encoding="utf-8")
    env = {k: v for k, v in os.environ.items()
           if k not in ("RELAY_DDS_PROFILE", "FARM_HOST_TAILNET_IP", "CYCLONEDDS_URI", "FIELD_NIC")}
    env.update(PATH=str(shim) + os.pathsep + env.get("PATH", ""), PRESENT_NICS=":".join(present),
               REPO_ROOT=str(root))
    env.update(env_over)
    r = subprocess.run([shutil.which("bash"), "-c", _launcher_dds_block()], env=env,
                       capture_output=True, text=True, timeout=60)
    return r.returncode, r.stdout, r.stderr


def test_런처도_고른_xml_이_없으면_78__게이트웨이가_통째로_죽기_전에(tmp_path):
    """관제 검수 P3: bridge_env.sh 는 78 로 멈추는데 런처는 검사가 없었다."""
    rc, out, err = _run_launcher_block(tmp_path, [], with_xml=False)
    assert rc == 78 and "DDS 프로파일 파일이 없다" in err, (rc, out, err)
    rc, out, _ = _run_launcher_block(tmp_path, [], with_xml=True)
    assert rc == 0 and out.strip().endswith("cyclonedds-offsite.xml")


def test_런처의_현장_NIC_도_환경변수로_바뀐다__bridge_env_와_같은_규칙(tmp_path):
    """관제 검수 P3: FIELD_NIC 가 bridge_env.sh 는 환경변수, 런처는 하드코딩이라 둘이 갈릴 수 있었다."""
    rc, out, _ = _run_launcher_block(tmp_path, ["eth9"], with_xml=True, xml_nic="eth9", FIELD_NIC="eth9")
    assert rc == 0 and out.strip().endswith("/cyclonedds.xml"), out
    rc, out, _ = _run_launcher_block(tmp_path, ["eth9"], with_xml=True)      # 기본 NIC 는 없다 → offsite
    assert rc == 0 and out.strip().endswith("cyclonedds-offsite.xml"), out


def test_런처도_tailnet_팜_주소를_export_한다(tmp_path):
    rc, out, err = _run_launcher_block(tmp_path, ["tailscale0"], with_xml=True, RELAY_DDS_PROFILE="tailnet",
                                       FARM_HOST_TAILNET_IP=FAKE_FARM)
    assert rc == 0, err
    block = _launcher_dds_block()
    assert "export FARM_HOST_TAILNET_IP" in block



def test_검수3_4_런처는_FIELD_NIC_와_현장_xml_의_NIC_가_다르면_78(tmp_path):
    """FIELD_NIC 만 바꾸고 configs/cyclonedds.xml 을 안 바꾸면 엉뚱한 NIC 에 선다(관제 검수 REVIEW_20260925 §3.4)."""
    rc, out, err = _run_launcher_block(tmp_path, ["eth9"], with_xml=True, FIELD_NIC="eth9")   # xml 은 기본 NIC
    assert rc == 78 and "같이 바꾼다" in err, (rc, out, err)


def test_검수3_4_bridge_env_도_FIELD_NIC_와_현장_xml_의_NIC_가_다르면_78(tmp_path):
    rc, _, err = _run(tmp_path, ["eth9"], FIELD_NIC="eth9")     # 레포의 진짜 cyclonedds.xml 은 기본 NIC 에 묶여 있다
    assert rc == 78 and "같이 바꾼다" in err, err


def test_검수3_4_기본_NIC_는_그대로_field(tmp_path):
    rc, out, _ = _run(tmp_path, [FIELD])
    assert rc == 0 and _uri(out) == "cyclonedds.xml"
