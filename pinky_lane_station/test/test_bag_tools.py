"""bag_report / bag_to_video 를 합성 mcap bag 으로 끝까지 돌린다 (ROS 불필요, mcap 패키지 없으면 skip)."""

import math
import os
import subprocess
import sys

import pytest

pytest.importorskip('mcap_ros2')
from mcap_ros2.writer import Writer  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOOLS = os.path.join(REPO, 'tools')

SEP = '================================================================================\n'
HEADER = 'builtin_interfaces/Time stamp\nstring frame_id\n'
TIME = 'int32 sec\nuint32 nanosec\n'


def defs(*pairs):
    """(type, body) … 첫 항목이 주 타입. 나머지는 의존 타입."""
    out = pairs[0][1]
    for name, body in pairs[1:]:
        out += SEP + f'MSG: {name}\n' + body
    return out


DEF = {
    'sensor_msgs/msg/LaserScan': defs(
        ('', 'std_msgs/Header header\nfloat32 angle_min\nfloat32 angle_max\nfloat32 angle_increment\n'
             'float32 time_increment\nfloat32 scan_time\nfloat32 range_min\nfloat32 range_max\n'
             'float32[] ranges\nfloat32[] intensities\n'),
        ('std_msgs/Header', HEADER), ('builtin_interfaces/Time', TIME)),
    'sensor_msgs/msg/Range': defs(
        ('', 'std_msgs/Header header\nuint8 radiation_type\nfloat32 field_of_view\nfloat32 min_range\n'
             'float32 max_range\nfloat32 range\n'),
        ('std_msgs/Header', HEADER), ('builtin_interfaces/Time', TIME)),
    'sensor_msgs/msg/CompressedImage': defs(
        ('', 'std_msgs/Header header\nstring format\nuint8[] data\n'),
        ('std_msgs/Header', HEADER), ('builtin_interfaces/Time', TIME)),
    'geometry_msgs/msg/Twist': defs(
        ('', 'geometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular\n'),
        ('geometry_msgs/Vector3', 'float64 x\nfloat64 y\nfloat64 z\n')),
    'nav_msgs/msg/Odometry': defs(
        ('', 'std_msgs/Header header\nstring child_frame_id\ngeometry_msgs/PoseWithCovariance pose\n'
             'geometry_msgs/TwistWithCovariance twist\n'),
        ('std_msgs/Header', HEADER), ('builtin_interfaces/Time', TIME),
        ('geometry_msgs/PoseWithCovariance', 'geometry_msgs/Pose pose\nfloat64[36] covariance\n'),
        ('geometry_msgs/Pose', 'geometry_msgs/Point position\ngeometry_msgs/Quaternion orientation\n'),
        ('geometry_msgs/Point', 'float64 x\nfloat64 y\nfloat64 z\n'),
        ('geometry_msgs/Quaternion', 'float64 x\nfloat64 y\nfloat64 z\nfloat64 w\n'),
        ('geometry_msgs/TwistWithCovariance', 'geometry_msgs/Twist twist\nfloat64[36] covariance\n'),
        ('geometry_msgs/Twist', 'geometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular\n'),
        ('geometry_msgs/Vector3', 'float64 x\nfloat64 y\nfloat64 z\n')),
    'pinky_lane_msgs/msg/LaneStatus': defs(
        ('', 'std_msgs/Header header\nstring robot_name\nuint8 drive_state\nstring state_reason\n'
             'uint32 route_seq\nint32 route_idx\nstring edge_id\nint32 clear_until_idx\nfloat32 error_x_norm\n'
             'uint8 lane_quality\nfloat32 linear_velocity\nfloat32 angular_velocity\nfloat32 path_age\n'
             'float32 lidar_min_range\nfloat32 us_range\nfloat32 odom_since_state\n'),
        ('std_msgs/Header', HEADER), ('builtin_interfaces/Time', TIME)),
}


def hdr(t):
    return {'stamp': {'sec': int(t), 'nanosec': int((t % 1) * 1e9)}, 'frame_id': 'x'}


def write_bag(path, seconds=6.0):
    import cv2
    import numpy as np
    os.makedirs(path, exist_ok=True)
    with open(os.path.join(path, 'part_0.mcap'), 'wb') as fh:
        w = Writer(fh)
        schemas = {name: w.register_msgdef(name, text) for name, text in DEF.items()}
        n = int(seconds * 10)
        ok, jpg = cv2.imencode('.jpg', np.full((120, 160, 3), 90, np.uint8))
        for i in range(n):
            t = 1000.0 + i * 0.1
            ns = int(t * 1e9)
            front = 0.3 if i == 25 else 1.0                       # 라이다 한 프레임 튐
            ranges = [front] * 360
            w.write_message('/scan', schemas['sensor_msgs/msg/LaserScan'],
                            {'header': hdr(t), 'angle_min': -math.pi, 'angle_max': math.pi,
                             'angle_increment': 2 * math.pi / 360, 'time_increment': 0.0, 'scan_time': 0.1,
                             'range_min': 0.05, 'range_max': 12.0, 'ranges': ranges, 'intensities': []},
                            log_time=ns, publish_time=ns)
            w.write_message('/us_sensor/range', schemas['sensor_msgs/msg/Range'],
                            {'header': hdr(t), 'radiation_type': 0, 'field_of_view': 0.3, 'min_range': 0.03,
                             'max_range': 3.0, 'range': 0.05 if i == 40 else 0.6},
                            log_time=ns, publish_time=ns)
            v = 0.1 if i > 5 else 0.0
            w.write_message('/cmd_vel', schemas['geometry_msgs/msg/Twist'],
                            {'linear': {'x': v, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                            log_time=ns, publish_time=ns)
            w.write_message('/odom', schemas['nav_msgs/msg/Odometry'],
                            {'header': hdr(t), 'child_frame_id': 'b',
                             'pose': {'pose': {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                               'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
                                      'covariance': [0.0] * 36},
                             'twist': {'twist': {'linear': {'x': v if i < 30 or i > 50 else 0.0, 'y': 0.0, 'z': 0.0},
                                                 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                                       'covariance': [0.0] * 36}},
                            log_time=ns, publish_time=ns)
            w.write_message('/pinky1/lane_status', schemas['pinky_lane_msgs/msg/LaneStatus'],
                            {'header': hdr(t), 'robot_name': 'pinky1', 'drive_state': 1 if i > 5 else 0,
                             'state_reason': 'ok', 'route_seq': 0, 'route_idx': 0, 'edge_id': 'lane_only',
                             'clear_until_idx': 0, 'error_x_norm': 0.1 * math.sin(i / 5), 'lane_quality': 0,
                             'linear_velocity': v, 'angular_velocity': 0.0, 'path_age': 0.15,
                             'lidar_min_range': front, 'us_range': 0.6, 'odom_since_state': 0.0},
                            log_time=ns, publish_time=ns)
            if i % 2 == 0 and not 20 <= i < 30:                 # 5 fps, 1 s 끊김
                w.write_message('/pinky1/camera/image/compressed', schemas['sensor_msgs/msg/CompressedImage'],
                                {'header': hdr(t), 'format': 'jpeg', 'data': jpg.tobytes()},
                                log_time=ns, publish_time=ns)
        w.finish()


def test_bag_report_and_video_on_synthetic_bag(tmp_path):
    bag = str(tmp_path / 'bag')
    write_bag(bag)
    r = subprocess.run([sys.executable, os.path.join(TOOLS, 'bag_report.py'), bag], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    md = (tmp_path / 'bag' / 'report.md').read_text(encoding='utf-8')
    csv_txt = (tmp_path / 'bag' / 'events.csv').read_text(encoding='utf-8')
    assert '`/scan`' in md and 'CRUISE' in md
    assert 'scan_jump' in csv_txt and 'us_jump' in csv_txt and 'vel_mismatch' in csv_txt and 'camera_gap' in csv_txt
    assert 'IDLE → CRUISE' in md
    out = str(tmp_path / 'clip.mp4')
    r = subprocess.run([sys.executable, os.path.join(TOOLS, 'bag_to_video.py'), bag, '--from', '1', '--to', '4',
                        '--out', out], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert os.path.getsize(out) > 1000
