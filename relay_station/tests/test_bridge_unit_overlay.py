# -*- coding: utf-8 -*-
"""R-1 · 브리지 유닛이 레포 `install/` 을 **있으면** 얹는다 — 없어도 브리지는 뜬다.

지시: `docs/REQ_20260924_RELAY_SESSION.md` R-1 (감사 §2·§8: 브리지 환경에 `/opt/ros/jazzy` 뿐이라
`pinky_lane_msgs/msg/PoseFix` 를 못 찾아 pose_fix 가 D8 에서 끝났다).

🔴 원격 main 에 먼저 들어간 판은 `[ -f install/setup.bash ] && source … && exec ros2 run …` 였다.
   install/ 이 없는 장비에서는 `exec` 까지 못 가서 **브리지가 아예 안 뜨고** Restart=always 로 조용히 돈다
   — 유닛 주석이 경고하는 D-3 과 같은 실패다. 그래서 문자열이 아니라 **동작**으로 잰다:
   유닛의 ExecStart 명령을 꺼내 가짜 레포·가짜 ros2 로 실제로 돌린다.
"""
import os
import re
import stat
import subprocess

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
UNIT = os.path.join(REPO, "relay_station", "domain_bridge", "systemd", "pinky-domain-bridge@.service")


def _exec_start():
    text = open(UNIT, encoding="utf-8").read()
    m = re.search(r"^ExecStart=/bin/bash -lc '(.*)'\s*$", text, re.M)
    assert m, "ExecStart 를 못 읽었다"
    return m.group(1)


def _fake_repo(tmp_path, with_install):
    """유닛이 가리키는 레포 경로를 가짜로 세운다. ros2 는 받은 인자와 OVERLAY 변수를 적는다."""
    repo = tmp_path / "repo"
    (repo / "relay_station" / "domain_bridge" / "configs").mkdir(parents=True)
    (repo / "relay_station" / "domain_bridge" / "bridge_env.sh").write_text("true\n")
    ros = tmp_path / "opt_ros_setup.bash"
    ros.write_text("true\n")
    if with_install:
        (repo / "install").mkdir()
        (repo / "install" / "setup.bash").write_text("export OVERLAY=loaded\n")
    bindir = tmp_path / "bin"
    bindir.mkdir()
    out = tmp_path / "ran.txt"
    ros2 = bindir / "ros2"
    ros2.write_text('#!/bin/bash\necho "RAN overlay=${OVERLAY:-none} $*" > "%s"\n' % out)
    ros2.chmod(ros2.stat().st_mode | stat.S_IEXEC)
    return repo, ros, bindir, out


def _run(tmp_path, with_install):
    repo, ros, bindir, out = _fake_repo(tmp_path, with_install)
    cmd = _exec_start()
    real_repo = re.search(r"source (\S+)/relay_station/domain_bridge/bridge_env\.sh", cmd).group(1)
    cmd = cmd.replace(real_repo, str(repo)).replace("/opt/ros/jazzy/setup.bash", str(ros))
    cmd = cmd.replace("%i", "robot1_control")
    env = {"PATH": "%s:/usr/bin:/bin" % bindir, "HOME": str(tmp_path)}
    r = subprocess.run(["/bin/bash", "-c", cmd], env=env, capture_output=True, text=True, timeout=20)
    return r.returncode, (out.read_text().strip() if out.exists() else None)


def test_install_이_없어도_브리지는_뜬다(tmp_path):
    rc, ran = _run(tmp_path, with_install=False)
    assert ran is not None, "install/ 이 없다고 ros2 run 까지 못 갔다 — 브리지가 안 뜬다(D-3 루프)"
    assert ran.startswith("RAN overlay=none run domain_bridge domain_bridge ")
    assert ran.endswith("/configs/robot1_control.yaml")


def test_install_이_있으면_얹고_뜬다__PoseFix_타입을_찾게(tmp_path):
    rc, ran = _run(tmp_path, with_install=True)
    assert ran is not None and ran.startswith("RAN overlay=loaded ")


def test_ROS_를_먼저_불러오고_install_을_그_위에_얹는다():
    cmd = _exec_start()
    assert cmd.index("/opt/ros/jazzy/setup.bash") < cmd.index("/install/setup.bash") < cmd.index("exec ros2 run")
