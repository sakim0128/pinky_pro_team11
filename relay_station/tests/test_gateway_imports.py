# -*- coding: utf-8 -*-
"""G-D — 게이트웨이 본체가 테스트의 사각지대였다.

2026-09-10, 내 실수로 드러났다. `gateway_web_server.py` 의 다중행 import 를 깨뜨려
`from source_registry import (, clock_alignment` 를 만들었는데 **테스트 156개가 전부 통과했고
커밋·푸시까지 됐다.** `conftest.py` 가 그 파일을 일부러 import 하지 않기 때문이다
(module-level 에서 rclpy·cv_bridge 를 부르는데 ROS 없이는 없다).

실행 중 프로세스는 이미 로드해 둔 모듈로 돌아 **증상이 없었다.** 다음 재기동에서 터진다 —
죽은 사본과 같은 종류의 침묵이다.

이 검사는 실행하지 않고 AST 로만 판정하므로 ROS 없이도 게이트웨이 본체를 본다.
여기서는 각 사고 유형마다 실제로 무는지를 하나씩 실증한다(뮤테이션).
"""
import os
import shutil
import sys
import tempfile

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_RELAY = os.path.dirname(_HERE)
_SCRIPTS = os.path.join(_RELAY, "scripts")
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from check_gateway_imports import (check_dir, module_names, main,
                                   SYNTAX, MISSING_NAME, MISSING_MODULE,
                                   DEFAULT_GATEWAY_DIR)

GOOD_SIBLING = '''# -*- coding: utf-8 -*-
CONSTANT = 1

def helper():
    return CONSTANT

class Thing:
    pass
'''

GOOD_MAIN = '''# -*- coding: utf-8 -*-
from sibling import CONSTANT, helper, Thing
'''


class Bed(object):
    def __init__(self, main_src=GOOD_MAIN, sibling_src=GOOD_SIBLING):
        self.root = tempfile.mkdtemp()
        self.write("sibling.py", sibling_src)
        self.write("main.py", main_src)

    def write(self, name, src):
        with open(os.path.join(self.root, name), "w", encoding="utf-8") as fh:
            fh.write(src)

    def check(self):
        return check_dir(self.root)[0]

    def close(self):
        shutil.rmtree(self.root, ignore_errors=True)


@pytest.fixture
def bed():
    b = Bed()
    yield b
    b.close()


def test_정상이면_문제가_없다(bed):
    assert bed.check() == []


def test_깨진_다중행_import_를_잡는다(bed):
    """이게 09-10 의 사고다. 문법이 깨졌는데 테스트가 안 봤다."""
    assert bed.check() == [], "전제: 망가뜨리기 전엔 통과해야 한다"
    bed.write("main.py", "from sibling import (, helper\n    CONSTANT,\n)\n")
    problems = bed.check()
    assert len(problems) == 1
    assert problems[0][0] == "main.py"
    assert problems[0][1] == SYNTAX


def test_형제에_없는_이름을_import_하면_잡는다(bed):
    """py_compile 은 이걸 못 본다 — 문법은 멀쩡하기 때문이다."""
    assert bed.check() == []
    bed.write("main.py", "from sibling import CONSTANT, clock_alignment\n")
    problems = bed.check()
    assert len(problems) == 1
    assert problems[0][1] == MISSING_NAME
    assert "clock_alignment" in problems[0][2]


def test_형제_모듈이_사라지면_상대_import_를_잡는다(bed):
    assert bed.check() == []
    bed.write("main.py", "from .gone import thing\n")
    problems = bed.check()
    assert len(problems) == 1
    assert problems[0][1] == MISSING_MODULE


def test_외부_모듈은_보지_않는다(bed):
    """rclpy·cv2 를 판정하면 거짓 경보가 된다. 우리가 소유하지 않은 것은 안 본다."""
    bed.write("main.py",
              "import rclpy\nfrom cv_bridge import CvBridge\n"
              "from sensor_msgs.msg import Image\nfrom sibling import helper\n")
    assert bed.check() == []


def test_try_import_로_들어온_이름도_센다(bed):
    """try: import cv2 / except: cv2 = None 형태를 못 보면 거짓 경보가 난다."""
    bed.write("sibling.py",
              "try:\n    import cv2\nexcept Exception:\n    cv2 = None\n\nVALUE = 2\n")
    bed.write("main.py", "from sibling import cv2, VALUE\n")
    assert bed.check() == []


def test_함수_안의_import_도_본다(bed):
    """지연 import 라고 이름이 안 풀려도 되는 것은 아니다. 부르는 순간 죽는다."""
    bed.write("main.py",
              "def later():\n    from sibling import nonexistent_name\n    return nonexistent_name\n")
    problems = bed.check()
    assert len(problems) == 1
    assert problems[0][1] == MISSING_NAME


def test_별표_import_는_이름을_못_보므로_넘어간다(bed):
    bed.write("main.py", "from sibling import *\n")
    assert bed.check() == []


def test_module_names_는_최상위_정의를_전부_모은다():
    import ast
    tree = ast.parse("A = 1\nB, C = 2, 3\n"
                     "def f(): pass\nclass K: pass\n"
                     "import os\nfrom x import y as z\n")
    got = module_names(tree)
    assert {"A", "B", "C", "f", "K", "os", "z"} <= got


def test_폴더가_없으면_건너뛰지_않고_실패한다():
    """⭐ 예전엔 0 을 돌려주며 건너뛰었다. 그게 2026-09-10 에 물렸다 —

    도커 복제본에서 HOME 이 /root 라 `~/pinky_pro/src/pinky_pro_team11/...` 를 못 찾고
    **조용히 통과**했다. entrypoint 의 게이트 1/3 이 아무것도 안 하고 있었던 것이다.
    검사 대상이 없다는 건 정상이 아니라 사고다 — 조용히 통과하면 게이트가 아니라 장식이다.
    """
    assert main(["--gateway-dir", "/no/such/dir"]) == 1


def test_HOME_이_달라도_자기_레포를_찾는다(monkeypatch):
    """스크립트는 레포 안에 산다. HOME 이나 현재 디렉터리에 기대지 않는다."""
    import check_gateway_imports as G
    monkeypatch.setenv("HOME", "/nonexistent-home")
    monkeypatch.delenv("REPO_ROOT", raising=False)
    root = G._repo_root()
    assert os.path.isdir(os.path.join(root, "relay_station", "gateway_web")), (
        "자기 레포를 못 찾았다: %s" % root)


@pytest.mark.live
def test_실제_게이트웨이_소스가_지금_기동_가능한_상태다():
    """이 레포의 실제 gateway_web/ 을 본다 — 이게 09-10 에 없어서 깨진 걸 푸시했다."""
    gw = os.path.join(_RELAY, "gateway_web")
    if not os.path.isdir(gw):
        pytest.skip("gateway_web 없음")
    problems, checked = check_dir(gw)
    assert problems == [], "게이트웨이가 기동하지 못하는 상태다: %r" % (problems,)
    assert checked >= 8, "검사한 파일이 %d 개뿐이다 - 대상을 못 찾은 것 아닌가" % checked


@pytest.mark.live
def test_게이트웨이_본체가_실제로_검사_대상에_들어간다():
    """대상이 비면 이 검사는 '항상 통과'가 된다 — 그게 제일 위험하다."""
    gw = os.path.join(_RELAY, "gateway_web")
    if not os.path.isdir(gw):
        pytest.skip("gateway_web 없음")
    names = [f for f in os.listdir(gw) if f.endswith(".py")]
    assert "gateway_web_server.py" in names
    assert "source_registry.py" in names
