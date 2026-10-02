# -*- coding: utf-8 -*-
"""게이트웨이 소스에 **정의 없이 부르는 이름**이 없다 (T0: fleet_resume_target 호출이 정의 없이 남았다).

check_gateway_imports.py 는 문법·형제 import 만 보고 undefined-name 은 pyflakes 가 있을 때만 본다(없으면 건너뜀).
호출만 남은 함수는 module-level 에서 안 터지고 요청이 그 가지에 닿는 순간 NameError 로 연결이 끊긴다.
"""
import ast
import builtins
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
GW = os.path.join(os.path.dirname(_HERE), "gateway_web")
DUNDER_OK = {"__file__", "__name__", "__doc__"}


def _bound(tree):
    b = set(dir(builtins)) | DUNDER_OK
    for n in ast.walk(tree):
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            b.add(n.name)
        if isinstance(n, ast.arg):
            b.add(n.arg)
        if isinstance(n, ast.Name) and isinstance(n.ctx, (ast.Store, ast.Del)):
            b.add(n.id)
        if isinstance(n, (ast.Import, ast.ImportFrom)):
            for al in n.names:
                b.add((al.asname or al.name).split(".")[0])
        if isinstance(n, ast.ExceptHandler) and n.name:
            b.add(n.name)
        if isinstance(n, (ast.Global, ast.Nonlocal)):
            b.update(n.names)
    return b


def _undefined(path):
    with open(path, encoding="utf-8") as fh:
        tree = ast.parse(fh.read())
    b = _bound(tree)
    return sorted({n.id for n in ast.walk(tree)
                   if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load) and n.id not in b})


def test_게이트웨이_소스에_정의_없는_이름이_없다():
    bad = {f: _undefined(os.path.join(GW, f)) for f in sorted(os.listdir(GW)) if f.endswith(".py")}
    bad = {k: v for k, v in bad.items() if v}
    assert not bad, bad


def test_탐지기가_정의_없는_호출을_잡는다():
    """⭐ 양성 대조군 — 탐지기가 죽으면 위 시험은 늘 초록이다."""
    tree = ast.parse("def f(coord):\n    return fleet_resume_target(coord)\n")
    assert "fleet_resume_target" not in _bound(tree)
