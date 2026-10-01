"""domain_bridge 실행기 — 도메인 번호 없는 브리지 템플릿에 --from/--to 를 채워 domain_bridge 를 띄운다.

apt 의 domain_bridge(jazzy)는 yaml 을 읽는 단계에서 토픽마다 from_domain/to_domain 을 요구한다 — CLI 의 --from/--to 는
파싱 뒤에 적용돼, 도메인 없는 템플릿(config/bridge_pinkyN_{up,down}.yaml)을 그대로 주면
`YamlParsingError: missing 'from_domain'` 으로 죽는다(2026-09-30 관제 PC 실측). 그래서 여기서 채운 사본을 만들어 넘긴다.

    ros2 run pinky_lane_station bridge_runner --from 10 --to 0 <템플릿.yaml>

lane_bridge.launch.xml 이 로봇마다 업링크·다운링크로 네 번 부른다. 사본은 $XDG_RUNTIME_DIR(없으면 /tmp)/pinky_bridge/ 에 쓴다.
"""

import argparse
import os
import sys
import tempfile

import yaml


def render_config(template, from_domain, to_domain):
    """템플릿 dict → 도메인을 채운 dict. 최상위 기본값과 토픽마다 둘 다 적는다 (버전마다 읽는 곳이 달라도 맞게)."""
    data = dict(template or {})
    topics = data.get('topics') or {}
    if not topics:
        raise ValueError('topics 가 비었다')
    data['from_domain'] = int(from_domain)
    data['to_domain'] = int(to_domain)
    out = {}
    for name, cfg in topics.items():
        cfg = dict(cfg or {})
        if 'type' not in cfg:
            raise ValueError(f'{name}: type 이 없다')
        cfg['from_domain'] = int(from_domain)
        cfg['to_domain'] = int(to_domain)
        out[name] = cfg
    data['topics'] = out
    return data


def write_rendered(template_path, from_domain, to_domain, out_dir=None):
    with open(template_path, encoding='utf-8') as fh:
        data = render_config(yaml.safe_load(fh), from_domain, to_domain)
    out_dir = out_dir or os.path.join(os.environ.get('XDG_RUNTIME_DIR') or tempfile.gettempdir(), 'pinky_bridge')
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(template_path))[0]
    path = os.path.join(out_dir, f'{base}_{int(from_domain)}_to_{int(to_domain)}.yaml')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(f'# {template_path} 에서 bridge_runner 가 만든 사본 — 고치지 않는다 (다음 실행 때 덮인다)\n')
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)
    return path


def domain_bridge_command(config_path):
    try:
        from ament_index_python.packages import get_package_prefix
        exe = os.path.join(get_package_prefix('domain_bridge'), 'lib', 'domain_bridge', 'domain_bridge')
        if os.access(exe, os.X_OK):
            return [exe, config_path]
    except Exception:                                   # noqa: BLE001 — 못 찾으면 ros2 run 으로
        pass
    return ['ros2', 'run', 'domain_bridge', 'domain_bridge', config_path]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--from', dest='from_domain', type=int, required=True)
    ap.add_argument('--to', dest='to_domain', type=int, required=True)
    ap.add_argument('template')
    args, _ros_args = ap.parse_known_args(argv)           # launch 가 붙이는 --ros-args 는 버린다
    path = write_rendered(args.template, args.from_domain, args.to_domain)
    cmd = domain_bridge_command(path)
    print(f'[bridge_runner] {args.template} → {path} (도메인 {args.from_domain} → {args.to_domain})', flush=True)
    os.execvp(cmd[0], cmd)


if __name__ == '__main__':
    main(sys.argv[1:])
