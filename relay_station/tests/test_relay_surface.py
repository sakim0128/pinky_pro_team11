# -*- coding: utf-8 -*-
"""개편 2단계 (2026-09-29): 중계는 브리지 · 코디네이터 · 제어 문 · 영상만 한다. 남은 HTTP 면을 **목록으로 잠근다**.

주 대시보드는 팀11 live 웹(:8080). 게이트웨이(:8889)에 경로가 새로 붙거나, 지운 화면·비전 월드·캘리브레이션이
되살아나면 이 시험이 먼저 빨개진다 — 늘리려면 여기 목록을 같이 고치고 설계 문서에 이유를 적는다. ROS 없이 돈다.
"""
import ast
import io
import os
import re

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
GW = os.path.join(os.path.dirname(HERE), 'gateway_web')
SERVER = os.path.join(GW, 'gateway_web_server.py')
CONSOLE = os.path.join(GW, 'static', 'relay_console.html')

GET = {
    '/api/relay/health', '/api/control', '/api/sources', '/api/status', '/api/fleet/profiles', '/api/fleet/status',
    '/api/camera/url', '/api/camera/scan',
    '/video_feed', '/video', '/video.mjpg', '/stream.mjpg',                       # 영상 중계(같은 루프, ?src=)
    '/shot.jpg', '/snapshot.jpg', '/frame.jpg', '/current.jpg', '/image.jpg',     # 한 장
}
POST = {
    '/api/control/acquire', '/api/control/release',
    '/api/fleet/start', '/api/fleet/stop', '/api/fleet/estop', '/api/fleet/resume', '/api/fleet/assign',
    '/api/vision/pose_fix',                                                       # 열린 결정 3: 키 뒤에 남김(키 없으면 401)
    '/api/camera/url', '/api/camera/upload', '/upload', '/camera', '/image', '/video', '/frame', '/shot.jpg',  # 폰이 밀어 넣는 영상
}
TABLES = {                                                                        # 경로 표로 받는 것
    'ROBOT_STOP_PATHS': {'/api/robot1/stop', '/api/stop', '/api/nav/stop', '/api/robot2/stop'},
    'ROBOT_RESUME_PATHS': {'/api/robot1/resume', '/api/robot2/resume'},
    'PROFILE_COMMANDS': {'/api/fleet/profile', '/api/fleet/robot_maps', '/api/fleet/initial_poses'},
}
REMOVED_MODULES = ('relay_controller_gui', 'ops_view', 'vision_world', 'vision_path', 'calibration', 'drift',
                   'framing', 'masks', 'censorship', 'field_low_latency_viewer')
REMOVED_PAGES = ('index.html', 'fleet_control_v2.html', 'fleet_control_v2.js', 'fleet_control_v2.css', 'ops-deploy.html')


def _src():
    with io.open(SERVER, encoding='utf-8') as f:
        return f.read()


def _routes(fn_name):
    tree = ast.parse(_src())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == fn_name)
    out = set()
    for c in ast.walk(fn):
        if isinstance(c, ast.Compare) and isinstance(c.left, ast.Attribute) and c.left.attr == 'path':
            for cmp in c.comparators:
                for lit in ast.walk(cmp):
                    if isinstance(lit, ast.Constant) and isinstance(lit.value, str):
                        out.add(lit.value)
    return out - {'/', ''}


def _table(name):
    tree = ast.parse(_src())
    for n in tree.body:
        if isinstance(n, ast.Assign) and any(getattr(t, 'id', '') == name for t in n.targets):
            return set(ast.literal_eval(n.value).keys())
    raise AssertionError(name)


def test_GET_경로는_이_목록뿐이다():
    assert _routes('do_GET') == GET


def test_POST_경로는_이_목록뿐이다():
    assert _routes('do_POST') == POST


@pytest.mark.parametrize('name', sorted(TABLES))
def test_경로_표도_이_목록뿐이다(name):
    assert _table(name) == TABLES[name]


@pytest.mark.parametrize('mod', REMOVED_MODULES)
def test_지운_모듈은_파일도_import_도_없다(mod):
    assert not os.path.exists(os.path.join(GW, mod + '.py'))
    assert not re.search(r'^\s*(import %s\b|from %s import)' % (mod, mod), _src(), re.M)


@pytest.mark.parametrize('page', REMOVED_PAGES)
def test_지운_화면은_없다(page):
    assert not os.path.exists(os.path.join(GW, 'static', page))


def test_기본_페이지는_중계_콘솔이다():
    assert "filename = 'relay_console.html' if parsed.path in ('/', '') else" in _src()
    assert os.path.isfile(CONSOLE)


def test_콘솔은_있는_경로만_부른다():
    """콘솔이 fetch/post 하는 경로가 전부 서버에 있다 — 지운 경로를 부르면 버튼이 조용히 404 가 된다."""
    html = io.open(CONSOLE, encoding='utf-8').read()
    called = set(re.findall(r"""(?:fetch|post)\(\s*['"](/[a-z0-9_/.]+)['"]""", html))
    called |= set(re.findall(r'data-post="(/[a-z0-9_/.]+)"', html))
    called |= {'/api/robot1/stop', '/api/robot2/stop', '/api/robot1/resume', '/api/robot2/resume'} \
        if "/api/${n === 'pinky1' ? 'robot1' : 'robot2'}/stop" in html else set()
    known = GET | POST | set().union(*TABLES.values())
    assert called, '콘솔이 아무 경로도 안 부른다?'
    assert called <= known, sorted(called - known)
    assert "'/video_feed?src=' + encodeURIComponent(id)" in html


def test_콘솔은_라이브러리_없이_한_장이다():
    html = io.open(CONSOLE, encoding='utf-8').read()
    assert '<script src=' not in html and '<link rel="stylesheet"' not in html
    assert len(html.splitlines()) < 400


def test_런처는_데스크톱_GUI_를_띄우지_않는다():
    launcher = io.open(os.path.join(os.path.dirname(GW), 'launch_master_gateway.sh'), encoding='utf-8').read()
    assert 'nohup python3 "$SCRIPT_DIR/relay_controller_gui.py"' not in launcher
    assert 'launch_live_web.sh' in launcher
