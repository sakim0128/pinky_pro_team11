#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""G-D — 게이트웨이 소스가 **문법적으로 성립하고 이름이 실제로 풀리는지** 검사한다.

왜 필요한가 (2026-09-10, 내 실수로 드러났다):
    `gateway_web_server.py` 를 패치하다 다중행 import 를 깨뜨려
    `from source_registry import (, clock_alignment` 를 만들었다.
    그런데 **테스트 156개가 전부 통과했고 커밋·푸시까지 됐다.**

    이유: `tests/conftest.py` 는 `gateway_web_server.py` 를 일부러 import 하지 않는다.
    그 파일이 module-level 에서 rclpy·cv_bridge 를 부르는데 ROS 없이는 없기 때문이다.
    그래서 **테스트가 그 파일을 아예 안 본다.** 게이트웨이 본체가 사각지대였다.

    실행 중 프로세스는 이미 모듈을 로드해 두어 멀쩡했다 — 증상이 없다.
    다음 재기동에서 한 번에 터진다. 죽은 사본과 같은 종류의 침묵이다.

무엇을 보나:
    1) 문법        모든 `gateway_web/*.py` 를 ast.parse 한다
    2) 형제 이름   `from <형제모듈> import X` 의 X 가 그 모듈에 **실제로 있는지** 본다
                   ROS 가 없어도 된다 — 실행하지 않고 AST 로만 판정하기 때문이다
    3) 상대 import `from .x import` 도 같은 폴더로 본다

    4) 플릿 코디네이터  게이트웨이가 기동 때 하는 것과 **같은** import(`fleet.fleet_coordinator`, realpath 기준)를
                   따로 떠서 해 본다 — 제3자 검수 G-1(아래). 게이트웨이를 띄우는 폴더(--exec-dir, 런처는 실행본
                   농장을 준다)의 realpath 로 본다 — 제3자 재검 GW-R2

⭐ py_compile 은 **문법만** 본다. 이름이 풀리는지는 이 검사가 본다.
⭐ 외부 모듈(rclpy·cv2·numpy…)은 보지 않는다. 우리가 소유하지 않은 것을 판정하면 거짓 경보가 된다.
   단 4) 는 예외다 — 코디네이터는 `pinky_fleet_msgs`·`pinky_lane_msgs`(레포 install 오버레이)가 없으면 못 뜨고,
   게이트웨이는 그 실패를 삼킨 채 **뜬다**(카메라·화면은 살아 있어야 해서). 그래서 조용했다(제3자 검수 G-1):
   실물 게이트웨이는 09-26 까지 코디네이터 없이 돌았고, 그동안 플릿 비상정지는 받는 쪽 0 인 채 200 이었다(G-2).
   launch_master_gateway.sh 는 오버레이를 **있을 때만** source 한다 — 없으면 여기서 크게 말한다.
"""
import argparse
import ast
import os
import subprocess
import sys

def _repo_root():
    """이 스크립트가 사는 레포의 루트. HOME 이나 현재 디렉터리에 기대지 않는다.

    ⭐ `~/...` 를 박아 두면 HOME 이 다른 곳(도커 복제본 = /root)에서 검사 대상을 못 찾고
       **조용히 통과한다.** 2026-09-10 에 실제로 그랬다 — 복제본의 게이트 1/3 이
       아무것도 안 하고 있었다. 스크립트는 레포 안에 사니 자기 위치를 쓰면 된다.
    """
    env = os.environ.get("REPO_ROOT")
    if env and os.path.isdir(env):
        return env
    here = os.path.dirname(os.path.abspath(__file__))          # relay_station/scripts
    return os.path.dirname(os.path.dirname(here))              # 레포 루트


DEFAULT_GATEWAY_DIR = os.path.join(_repo_root(), "relay_station", "gateway_web")

SYNTAX = "문법오류"
MISSING_NAME = "이름없음"      # 형제 모듈에 그 이름이 없다
MISSING_MODULE = "형제없음"    # from x import 인데 x.py 가 사라졌다


def module_names(tree):
    """모듈이 최상위에서 정의·대입하는 이름 전부. `import x` 로 들어온 이름도 포함한다."""
    names = set()
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name):
                    names.add(target.id)
                elif isinstance(target, (ast.Tuple, ast.List)):
                    for elt in target.elts:
                        if isinstance(elt, ast.Name):
                            names.add(elt.id)
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            names.add(node.target.id)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                names.add(alias.asname or alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            for alias in node.names:
                names.add(alias.asname or alias.name)
        elif isinstance(node, ast.Try):
            # try: import x / except: x = None 형태를 놓치지 않는다
            for sub in node.body + node.orelse:
                if isinstance(sub, ast.Import):
                    for alias in sub.names:
                        names.add(alias.asname or alias.name.split(".")[0])
                elif isinstance(sub, ast.ImportFrom):
                    for alias in sub.names:
                        names.add(alias.asname or alias.name)
                elif isinstance(sub, ast.Assign):
                    for target in sub.targets:
                        if isinstance(target, ast.Name):
                            names.add(target.id)
    return names


def check_dir(gateway_dir):
    """(problems, checked_files) 를 돌려준다. problems = [(file, kind, detail)]"""
    problems = []
    try:
        files = sorted(f for f in os.listdir(gateway_dir) if f.endswith(".py"))
    except OSError as exc:
        return [(gateway_dir, MISSING_MODULE, str(exc))], 0

    trees = {}
    for name in files:
        path = os.path.join(gateway_dir, name)
        try:
            with open(path, "rb") as fh:
                trees[name] = ast.parse(fh.read(), filename=path)
        except SyntaxError as exc:
            problems.append((name, SYNTAX, "line %s: %s" % (exc.lineno, exc.msg)))

    exported = {n: module_names(t) for n, t in trees.items()}

    for name, tree in trees.items():
        for node in ast.walk(tree):
            if not isinstance(node, ast.ImportFrom) or node.module is None:
                continue
            sibling = node.module.split(".")[-1] + ".py"
            if sibling not in exported:
                # 형제가 아니면(외부 패키지) 보지 않는다.
                # 단 상대 import 는 반드시 형제여야 한다.
                if node.level and node.level > 0:
                    problems.append((name, MISSING_MODULE,
                                     "from %s%s import — 같은 폴더에 %s 가 없다"
                                     % ("." * node.level, node.module, sibling)))
                continue
            for alias in node.names:
                if alias.name == "*":
                    continue
                if alias.name not in exported[sibling]:
                    problems.append((name, MISSING_NAME,
                                     "from %s import %s — %s 에 그 이름이 없다 (line %s)"
                                     % (node.module, alias.name, sibling, node.lineno)))
    return problems, len(files)



def undefined_names(gateway_dir, require=False):
    """정의되지 않은 이름을 찾는다. (문제목록, 검사했나).

    ⭐ 이 검사가 왜 따로 필요한가: 위의 검사는 **import 문**만 본다.
       `from x import a` 의 a 가 x 에 있는지는 보지만, **아예 안 가져온 이름을 쓴 것**은
       못 본다. 그게 2026-09-11 에 물렸다 — SEVERITY_ORDER 를 안 가져오고 썼는데
       게이트가 통과했고, 그 줄은 레지스트리가 있을 때만 닿는 자리라 운영에서 터진다.

    pyflakes 가 없으면 (없음, False) 를 준다. require 면 호출자가 그걸 실패로 삼는다 —
    **없는 것을 통과로 읽지 않기 위해서다.**
    """
    try:
        from pyflakes.api import checkPath
        from pyflakes.reporter import Reporter
    except ImportError:
        return [], False

    class _Collect(object):
        def __init__(self):
            self.hits = []

        def unexpectedError(self, filename, msg):
            self.hits.append((filename, 0, "pyflakes 실패: %s" % msg))

        def syntaxError(self, filename, msg, lineno, offset, text):
            self.hits.append((filename, lineno or 0, "문법오류: %s" % msg))

        def flake(self, message):
            text = str(message)
            # undefined name 만 본다. 잡음이 늘면 사람이 게이트를 안 읽는다.
            if "undefined name" in text:
                self.hits.append((message.filename,
                                  getattr(message, "lineno", 0), text))

    rep = _Collect()
    for name in sorted(os.listdir(gateway_dir)):
        if not name.endswith(".py"):
            continue
        checkPath(os.path.join(gateway_dir, name), rep)
    return rep.hits, True


FLEET_IMPORT_CODE = ("import sys; sys.path.insert(0, sys.argv[1]); "
                     "from fleet.fleet_coordinator import RelayFleetCoordinator")


def fleet_import_problem(gateway_dir, env=None, timeout=30):
    """게이트웨이가 기동 때 하는 import 로 플릿 코디네이터를 찾는다. 찾으면 None, 못 찾으면 이유(문자열).

    제3자 검수 G-1: 게이트웨이의 `except Exception: RelayFleetCoordinator = None` 은 이유를 로그에만 남기고 뜬다.
    여기서는 **게이트웨이와 같은 규칙**(gateway_web_server.py 의 realpath 두 단계 위 = relay 루트)으로, 이 검사 프로세스의
    sys.path 가 섞이지 않게 따로 떠서 import 해 본다. 환경(ROS·오버레이 source 여부)은 부른 셸의 것을 그대로 쓴다.
    """
    server = os.path.join(gateway_dir, "gateway_web_server.py")
    relay_root = os.path.dirname(os.path.dirname(os.path.realpath(server)))
    if not os.path.isfile(os.path.join(relay_root, "fleet", "fleet_coordinator.py")):
        return "fleet/fleet_coordinator.py 가 없다 (relay 루트 %s)" % relay_root
    try:
        r = subprocess.run([sys.executable, "-B", "-c", FLEET_IMPORT_CODE, relay_root],
                           cwd=gateway_dir, env=env, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return "import 검사를 돌리지 못했다: %s" % exc
    if r.returncode == 0:
        return None
    lines = [l for l in (r.stderr or "").strip().splitlines() if l.strip()]
    return lines[-1].strip() if lines else "exit %d" % r.returncode


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="게이트웨이 소스의 문법과 형제 import 이름을 검사 (ROS 불필요)")
    ap.add_argument("--gateway-dir", default=DEFAULT_GATEWAY_DIR)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--require-pyflakes", action="store_true",
                    help="pyflakes 가 없으면 실패한다. 컨테이너처럼 반드시 있어야 하는 "
                         "곳에서 쓴다 - 없는 검사를 통과로 읽지 않기 위해서다")
    ap.add_argument("--require-fleet", action="store_true",
                    help="플릿 코디네이터를 import 하지 못하면 실패한다. 기본은 크게 알리기만 한다 - "
                         "게이트웨이는 카메라·화면 때문에 코디네이터 없이도 뜨기 때문이다(그때 플릿·이동 명령은 503)")
    ap.add_argument("--exec-dir", default=None,
                    help="게이트웨이를 실제로 띄우는 폴더(현장은 ~/doc/src/field_gateway_relay 심링크 농장). 플릿 "
                         "코디네이터는 이 폴더의 gateway_web_server.py realpath 로 찾는다 - 기본은 --gateway-dir. "
                         "문법·형제 이름 검사는 그대로 --gateway-dir 을 본다")
    args = ap.parse_args(argv)

    if not os.path.isdir(args.gateway_dir):
        # ⭐ 건너뛰지 않는다. 검사 대상이 없다는 건 정상이 아니라 사고다 —
        #    조용히 통과하면 게이트가 아니라 장식이다.
        print("  [imports] 🔴 검사할 폴더가 없습니다: %s" % args.gateway_dir)
        return 1

    # 제3자 검수 G-1: 플릿 코디네이터 — --quiet 여도 문제는 찍는다(조용한 실패가 이 항목의 결함이었다)
    # 제3자 재검 GW-R2: 게이트웨이가 **실제로 뜨는 폴더**로 본다. 레포 폴더만 보면 실행본 본체가 사본이 됐을 때(§14-3 G-B 회귀)
    #     게이트웨이는 fleet 을 못 찾는데 여기는 '정상' 이라 한다. 농장의 다른 파일(robot1_mission_navigator.py 등)까지
    #     소스 검사에 끌어들이지 않도록 이 검사만 옮긴다.
    fleet_problem = fleet_import_problem(args.exec_dir or args.gateway_dir)
    if fleet_problem:
        print("  [imports] 🔴 플릿 코디네이터를 import 하지 못합니다 — 게이트웨이는 뜨지만 플릿 제어·비상정지·"
              "우회 목표·미션이 전부 503 입니다")
        print("            이유: %s" % fleet_problem)
        print("            확인: 레포 install/setup.bash(pinky_fleet_msgs·pinky_lane_msgs)를 source 했나 · "
              "relay_station/fleet 이 실행본 realpath 옆에 있나")
        if args.require_fleet:
            return 1
    elif not args.quiet:
        print("  [imports] 플릿 코디네이터 import 정상")

    problems, checked = check_dir(args.gateway_dir)

    # 정의 안 된 이름 (import 문 검사가 못 보는 자리)
    undef, ran = undefined_names(args.gateway_dir, require=args.require_pyflakes)
    if undef:
        print("  [imports] 🔴 정의되지 않은 이름")
        for fn, ln, msg in undef:
            print("         %s:%s  %s" % (os.path.basename(fn), ln, msg))
        return 1
    if not ran:
        if args.require_pyflakes:
            print("  [imports] 🔴 pyflakes 가 없어 '정의 안 된 이름' 검사를 못 했습니다.")
            print("         없는 검사를 통과로 읽지 않습니다. pip install pyflakes")
            return 1
        print("  [imports] ⚠️ pyflakes 없음 — '정의 안 된 이름' 검사는 건너뜁니다")
    if not problems:
        if not args.quiet:
            print("  [imports] 게이트웨이 %d 파일 문법·형제 이름 정상" % checked)
        return 0

    print("  [imports] 🔴 게이트웨이 소스가 기동하지 못하는 상태입니다")
    for name, kind, detail in problems:
        print("            %-8s %-28s %s" % (kind, name, detail))
    print("            이 상태로 재기동하면 게이트웨이가 죽습니다. 커밋 전에 고치세요.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
