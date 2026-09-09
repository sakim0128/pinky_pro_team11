"""현장 사전 점검 — 미션을 돌리기 전에 "지금 뭐가 준비돼 있나"를 한 화면에 보여준다.

로봇 도메인마다 자식 프로세스를 띄운다(rclpy.init 은 프로세스당 1회이므로).
domain_worker 와 같은 multiprocessing 패턴이다.

    ros2 run pinky_fleet preflight
    ros2 run pinky_fleet preflight --print-home     # mission.yaml 에 붙여넣을 home 블록
    ros2 run pinky_fleet preflight --domains 10     # 특정 도메인만

점검하는 것
  - 노드 목록에서 amcl / bt_navigator / map_server 를 찾고 **네임스페이스를 알아낸다**
  - /map, /amcl_pose 존재 여부와 발행자 QoS
  - navigate_to_pose 액션 서버가 응답하는지
  - 현재 /amcl_pose 좌표 (→ home 실측값)
  - **PC 와 로봇의 시계 차이** — Pinky(RPi5)는 RTC 가 없어 크게 어긋날 수 있고,
    어긋나면 TF 조회가 통째로 실패해서 Nav2 가 이상하게 군다
"""

from __future__ import annotations

import argparse
import multiprocessing as mp
import sys
import time

from pinky_fleet.mission_config import load_mission_config
from pinky_fleet.pose_utils import yaw_deg_from_quaternion
from pinky_fleet.ros_qos import amcl_pose_qos

OK = 'OK  '
NO = 'FAIL'
WARN = 'WARN'

NAV2_NODES = ('amcl', 'bt_navigator', 'map_server', 'controller_server', 'planner_server')


def probe_process(domain_id, label, wait_sec, result_q):
    """자식 프로세스: 도메인 하나를 들여다보고 결과 dict 를 큐로 올린다."""
    out = {'domain_id': domain_id, 'label': label, 'errors': []}
    try:
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient

        rclpy.init(args=[], domain_id=int(domain_id))
        node = rclpy.create_node(f'preflight_{domain_id}')

        # 디스커버리가 끝날 시간을 준다
        deadline = time.time() + wait_sec
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        # --- 노드 목록 + 네임스페이스 ---
        names = node.get_node_names_and_namespaces()
        out['nodes'] = sorted(f'{ns.rstrip("/")}/{n}' if ns != '/' else n for n, ns in names)
        found = {}
        for n, ns in names:
            if n in NAV2_NODES:
                found[n] = ns
        out['nav2_nodes'] = found
        namespaces = {ns.strip('/') for ns in found.values() if ns.strip('/')}
        out['namespace'] = sorted(namespaces)[0] if namespaces else ''

        ns_prefix = f'/{out["namespace"]}' if out['namespace'] else ''

        # --- 토픽 + 발행자 QoS ---
        topics = dict(node.get_topic_names_and_types())
        out['topics'] = sorted(topics)
        for key, topic in (('map', f'{ns_prefix}/map'),
                           ('amcl_pose', f'{ns_prefix}/amcl_pose'),
                           ('scan', f'{ns_prefix}/scan')):
            present = topic in topics
            info = ''
            if present:
                try:
                    pubs = node.get_publishers_info_by_topic(topic)
                    if pubs:
                        q = pubs[0].qos_profile
                        info = (f'{q.reliability.name.lower()} · '
                                f'{q.durability.name.lower()}')
                except Exception:
                    pass
            out[key] = {'topic': topic, 'present': present, 'qos': info}

        # --- navigate_to_pose 액션 서버 ---
        ac = ActionClient(node, NavigateToPose, f'{ns_prefix}/navigate_to_pose')
        out['action'] = {'name': f'{ns_prefix}/navigate_to_pose',
                         'ready': ac.wait_for_server(timeout_sec=5.0)}
        ac.destroy()

        # --- 현재 위치 + 시계 오차 ---
        got = {}

        def _cb(m):
            p = m.pose.pose
            got['x'] = p.position.x
            got['y'] = p.position.y
            got['yaw_deg'] = yaw_deg_from_quaternion(
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            got['stamp'] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            got['frame'] = m.header.frame_id
            c = m.pose.covariance
            got['xy_std'] = max(abs(c[0]), abs(c[7])) ** 0.5
            got['yaw_std'] = abs(c[35]) ** 0.5

        node.create_subscription(
            PoseWithCovarianceStamped, f'{ns_prefix}/amcl_pose', _cb, amcl_pose_qos())
        deadline = time.time() + 5.0
        while time.time() < deadline and 'x' not in got:
            rclpy.spin_once(node, timeout_sec=0.1)

        if 'x' in got:
            # 메시지 stamp 는 로봇 시계, now() 는 PC 시계다. 차이가 곧 시계 오차.
            now = node.get_clock().now().nanoseconds / 1e9
            got['clock_skew'] = now - got['stamp']
        out['pose'] = got

        node.destroy_node()
        rclpy.shutdown()
    except Exception as exc:  # noqa: BLE001 - 무엇이 터지든 부모에게 알려야 한다
        import traceback
        out['errors'].append(traceback.format_exc())
        out['fatal'] = f'{type(exc).__name__}: {exc}'
    result_q.put(out)


# ---------------------------------------------------------------------------
def check_pc_packages():
    """PC 쪽에 필요한 패키지가 깔려 있는지."""
    import importlib.util
    import shutil

    rows = []
    for label, kind, target in (
            ('nav2_simple_commander', 'python', 'nav2_simple_commander.robot_navigator'),
            ('domain_bridge', 'exec', 'domain_bridge'),
            ('turtlesim (L1 검증용)', 'exec', 'turtlesim_node'),
            ('rviz2', 'exec', 'rviz2')):
        if kind == 'python':
            ok = importlib.util.find_spec(target.split('.')[0]) is not None
        else:
            ok = shutil.which(target) is not None
        rows.append((label, ok))
    return rows


def render(out, cfg_robot):
    """도메인 하나의 결과를 사람이 읽을 형태로."""
    d = out['domain_id']
    print(f'\n─── 도메인 {d} · {out["label"]} ' + '─' * 34)

    if out.get('fatal'):
        print(f'  {NO}  프로세스가 죽었습니다: {out["fatal"]}')
        for e in out['errors']:
            print('        ' + e.replace('\n', '\n        '))
        return False

    ok = True
    ns = out.get('namespace', '')
    nav2 = out.get('nav2_nodes', {})
    if nav2:
        print(f'  {OK}  Nav2 노드 {len(nav2)}개: ' + ', '.join(sorted(nav2)))
    else:
        ok = False
        print(f'  {NO}  Nav2 노드를 못 찾았습니다 (amcl / bt_navigator 없음)')
        print(f'        보이는 노드: {", ".join(out.get("nodes", [])) or "(없음)"}')
        print('        → 로봇에서 pinky_navigation bringup_launch.xml 이 떠 있는지,')
        print(f'          로봇의 ROS_DOMAIN_ID 가 {d} 인지 확인하세요.')

    if ns:
        print(f'  {WARN}  네임스페이스 "{ns}" 아래에서 돌고 있습니다.')
        print(f'        → mission.yaml 의 해당 로봇에 namespace: \'{ns}\' 를 넣으세요.')
        if cfg_robot and cfg_robot.namespace != ns:
            ok = False

    for key, label in (('map', '/map'), ('amcl_pose', '/amcl_pose'), ('scan', '/scan')):
        info = out.get(key, {})
        if info.get('present'):
            print(f'  {OK}  {info["topic"]:<24} {info.get("qos", "")}')
        else:
            ok = False
            print(f'  {NO}  {info.get("topic", label)} 없음')

    act = out.get('action', {})
    if act.get('ready'):
        print(f'  {OK}  액션 서버 {act["name"]} 응답함')
    else:
        ok = False
        print(f'  {NO}  액션 서버 {act.get("name")} 무응답 → Nav2 가 아직 활성화되지 않았습니다')

    pose = out.get('pose', {})
    if pose.get('x') is not None:
        print(f'  {OK}  현재 위치  x={pose["x"]:.3f}  y={pose["y"]:.3f}  '
              f'yaw={pose["yaw_deg"]:.1f}°  (frame={pose.get("frame")})')
        print(f'        공분산 xy_std={pose["xy_std"]:.3f} m  yaw_std={pose["yaw_std"]:.3f} rad')
        skew = pose.get('clock_skew')
        if skew is not None:
            tag = OK if abs(skew) < 1.0 else NO
            if abs(skew) >= 1.0:
                ok = False
            print(f'  {tag}  시계 오차 {skew:+.2f} s  (PC now - 로봇 stamp)')
            if abs(skew) >= 1.0:
                print('        → 1초 이상 어긋나면 TF 조회가 실패해 Nav2 가 이상하게 굽니다.')
                print('          로봇에서:  sudo date -s "$(date -u +\'%Y-%m-%d %H:%M:%S\')" '
                      '   (PC 시각으로 맞춤)')
    else:
        print(f'  {WARN}  /amcl_pose 를 한 건도 못 받았습니다.')
        print('        amcl 은 로봇이 정지해 있으면 발행하지 않습니다. 아직 2D Pose Estimate 를')
        print('        한 번도 안 했다면 정상입니다 — fleet_master 가 setInitialPose 로 넣습니다.')
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='preflight', description='현장 사전 점검 — 로봇/브리지/관제 준비 상태 확인')
    ap.add_argument('--config', default='', help='mission.yaml 경로')
    ap.add_argument('--domains', nargs='*', type=int, default=None,
                    help='이 도메인만 검사 (기본: mission.yaml 의 로봇 전부 + 관제 도메인)')
    ap.add_argument('--print-home', action='store_true',
                    help='현재 위치를 mission.yaml 에 붙여넣을 형태로 출력')
    ap.add_argument('--wait', type=float, default=3.0, help='도메인별 디스커버리 대기 [s]')
    args = ap.parse_args(argv)

    try:
        mp.set_start_method('spawn')
    except RuntimeError:
        pass

    cfg = load_mission_config(args.config)
    targets = []
    if args.domains:
        by_domain = {r.domain_id: r for r in cfg.robots}
        for d in args.domains:
            r = by_domain.get(d)
            targets.append((d, r.name if r else f'domain {d}', r))
    else:
        for r in cfg.ordered_robots():
            targets.append((r.domain_id, r.name, r))
        targets.append((cfg.control_domain_id, '관제 도메인', None))

    print(f'[CONFIG] {cfg.source_path}')
    print('\n=== PC 패키지 ===')
    pc_ok = True
    for label, ok in check_pc_packages():
        print(f'  {OK if ok else NO}  {label}')
        pc_ok = pc_ok and ok
    if not pc_ok:
        print('  → 없는 것 설치:  sudo apt install ros-jazzy-domain-bridge '
              'ros-jazzy-nav2-simple-commander ros-jazzy-turtlesim')

    result_q = mp.Queue()
    procs = []
    for domain_id, label, _ in targets:
        p = mp.Process(target=probe_process,
                       args=(domain_id, label, args.wait, result_q),
                       name=f'preflight_{domain_id}', daemon=True)
        p.start()
        procs.append(p)

    results = {}
    deadline = time.time() + args.wait + 25.0
    while len(results) < len(targets) and time.time() < deadline:
        try:
            out = result_q.get(timeout=0.5)
            results[out['domain_id']] = out
        except Exception:
            continue
    for p in procs:
        p.join(2.0)
        if p.is_alive():
            p.terminate()

    all_ok = pc_ok
    for domain_id, label, robot in targets:
        out = results.get(domain_id)
        if out is None:
            all_ok = False
            print(f'\n─── 도메인 {domain_id} · {label} ' + '─' * 34)
            print(f'  {NO}  응답 없음 (프로세스 타임아웃)')
            continue
        all_ok = render(out, robot) and all_ok

    if args.print_home:
        print('\n=== mission.yaml 의 robots 블록에 붙여넣기 ===')
        print('robots:')
        for domain_id, label, robot in targets:
            out = results.get(domain_id) or {}
            pose = out.get('pose', {})
            if robot is None or 'x' not in pose:
                continue
            ns = out.get('namespace', '')
            print(f'  - name: {robot.name}')
            print(f'    domain_id: {robot.domain_id}')
            print(f"    namespace: '{ns}'")
            print(f'    home: {{x: {pose["x"]:.3f}, y: {pose["y"]:.3f}, '
                  f'yaw_deg: {pose["yaw_deg"]:.1f}}}')
        print('\n  ※ 로봇을 실제 출발 위치에 놓고, 그 위치가 맵 상에서 맞게 잡힌')
        print('     상태(RViz 에서 라이다가 벽과 일치)에서 뽑은 값이어야 합니다.')

    print('\n' + ('=' * 60))
    print('전체 결과: ' + ('준비 완료 — fleet_master 로 넘어가도 됩니다'
                       if all_ok else '문제 있음 — 위의 FAIL 항목을 먼저 해결하세요'))
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
