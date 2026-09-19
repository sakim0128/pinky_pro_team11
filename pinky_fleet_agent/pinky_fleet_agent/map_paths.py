"""맵 "이름" 을 로봇의 맵 파일 경로로 바꾼다.

관제 PC 와 로봇은 맵 파일 경로가 서로 다르므로(`/home/sungah/maps` vs
`/home/pinky/map`) 전체 경로 대신 이름만 주고받고, 로봇이 자기 디렉터리에서 찾는다.

이 모듈은 rclpy 에 의존하지 않는다 — 이름 검증은 네트워크로 들어온 값을 다루므로
단위 테스트로 확실히 덮어야 한다.
"""

import os

MAP_EXTENSIONS = ('.yaml', '.yml')


def normalize_map_name(map_name):
    """맵 이름에서 경로 성분과 확장자를 떼어낸 안전한 이름을 돌려준다.

    이름은 FleetCommand 로 네트워크를 타고 들어오므로 `../../etc/passwd` 같은
    값이 올 수 있다고 보고 다룬다. 반환값은 디렉터리 구분자가 없는 이름이거나 None.
    """
    if not map_name:
        return None
    name = os.path.basename(str(map_name).strip().replace('\\', '/'))
    root, ext = os.path.splitext(name)
    if ext.lower() in MAP_EXTENSIONS:
        name = root
    if not name or name in ('.', '..'):
        return None
    return name


def resolve_map_path(map_dir, map_name):
    """``(path, error)`` 를 돌려준다. error 가 None 이 아니면 path 는 None.

    - 이름에서 경로 성분을 제거하고 확장자를 `.yaml` 로 통일한다.
    - 최종 경로가 정말 ``map_dir`` 안에 있는지 realpath 로 한 번 더 확인한다
      (심볼릭 링크로 밖을 가리키는 경우까지 막는다).
    - 파일이 없으면 서비스를 호출하기 전에 걸러낸다. 로그가 명확해진다.
    """
    name = normalize_map_name(map_name)
    if name is None:
        return None, f'맵 이름이 올바르지 않습니다: {map_name!r}'

    base = os.path.realpath(os.path.expandvars(os.path.expanduser(str(map_dir))))
    path = os.path.realpath(os.path.join(base, name + '.yaml'))

    if os.path.commonpath([base, path]) != base:
        return None, f'맵 디렉터리 밖을 가리킵니다: {path}'
    if not os.path.isfile(path):
        return None, f'맵을 찾을 수 없습니다: {path}'
    return path, None
