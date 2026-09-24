"""Subscribe to live telemetry only; never publishes commands or heartbeats."""
import json
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from pinky_fleet_msgs.msg import RobotState
from pinky_fleet_msgs.msg import FleetCommand
from pinky_lane_msgs.msg import LaneStatus
from std_msgs.msg import String

from .live_http import make_server
from .live_frames import FrameStore
from .live_control import ControlQueue
from .live_map import MapAsset
from .live_state import StateStore


def message_data(msg):
    result = {key: getattr(msg, key) for key in msg.get_fields_and_field_types()
              if key != 'header'}
    result['header'] = dict(frame_id=msg.header.frame_id,
                            stamp=dict(sec=msg.header.stamp.sec,
                                       nanosec=msg.header.stamp.nanosec))
    return result


def amcl_data(msg):
    """Keep only the AMCL values the browser needs; never expose ROS objects."""
    pose = msg.pose.pose
    return dict(header=dict(frame_id=msg.header.frame_id,
                            stamp=dict(sec=msg.header.stamp.sec,
                                       nanosec=msg.header.stamp.nanosec)),
                position=dict(x=pose.position.x, y=pose.position.y),
                orientation=dict(x=pose.orientation.x, y=pose.orientation.y,
                                 z=pose.orientation.z, w=pose.orientation.w),
                covariance=list(msg.pose.covariance))


def pose_data(msg):
    pose = msg.pose
    return dict(header=dict(frame_id=msg.header.frame_id,
                            stamp=dict(sec=msg.header.stamp.sec,
                                       nanosec=msg.header.stamp.nanosec)),
                position=dict(x=pose.position.x, y=pose.position.y),
                orientation=dict(x=pose.orientation.x, y=pose.orientation.y,
                                 z=pose.orientation.z, w=pose.orientation.w))


class LiveWebNode(Node):
    def __init__(self):
        super().__init__('fleet_live_web')
        self.declare_parameter('robot_names', ['pinky1', 'pinky2'])
        self.declare_parameter('state_timeout', 2.0)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8080)
        self.declare_parameter('overhead_camera_topic', '/overhead/camera/image/compressed')
        self.declare_parameter('enable_control', False)
        self.declare_parameter('initial_poses_json', '{"pinky1":[0.11,1.08,0.0],"pinky2":[0.16,0.74,-1.57079632679]}')
        default_map = (get_package_share_directory('pinky_fleet_station') +
                       '/config/map5.yaml')
        self.declare_parameter('map_yaml', default_map)
        self.store = StateStore(self.get_parameter('robot_names').value,
                                self.get_parameter('state_timeout').value)
        self.frames = FrameStore(self.store.names, self.get_parameter('state_timeout').value)
        self.controls = ControlQueue(bool(self.get_parameter('enable_control').value))
        self._lane_control_pub = self.create_publisher(String, '/fleet/lane/control', 10)
        self._fleet_control_pubs = {name: self.create_publisher(FleetCommand, f'/{name}/command', 10)
                                    for name in self.store.names}
        try:
            raw_poses = json.loads(self.get_parameter('initial_poses_json').value)
            self._initial_poses = {name: tuple(map(float, raw_poses[name])) for name in self.store.names}
            if any(len(pose) != 3 for pose in self._initial_poses.values()):
                raise ValueError
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            raise RuntimeError('initial_poses_json must define [x,y,yaw] for every robot')
        self.subscriptions_kept = []
        for name in self.store.names:
            for kind, cls, suffix in [('state', RobotState, 'state'),
                                      ('lane', LaneStatus, 'lane_status')]:
                self.subscriptions_kept.append(self.create_subscription(
                    cls, f'/{name}/{suffix}',
                    lambda msg, key=(name, kind): self.store.update(key, message_data(msg)), 10))
            self.subscriptions_kept.append(self.create_subscription(
                PoseWithCovarianceStamped, f'/{name}/amcl_pose',
                lambda msg, key=(name, 'amcl'): self.store.update(key, amcl_data(msg)), 10))
            self.subscriptions_kept.append(self.create_subscription(
                CompressedImage, f'/{name}/camera/image/compressed',
                lambda msg, robot=name: self.on_camera(robot, msg), 10))
            self.subscriptions_kept.append(self.create_subscription(
                PoseStamped, f'/{name}/overhead_pose',
                lambda msg, key=(name, 'overhead'): self.store.update(key, pose_data(msg)), 10))
        self.subscriptions_kept.append(self.create_subscription(
            CompressedImage, self.get_parameter('overhead_camera_topic').value,
            lambda msg: self.on_camera('overhead', msg), 10))
        self.create_timer(.05, self.process_controls)
        self.subscriptions_kept.append(self.create_subscription(
            String, '/fleet/lane/status', self.on_mission, 10))

    def on_mission(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get('mission'), str):
                raise ValueError('expected object with mission string')
        except (ValueError, TypeError):
            self.get_logger().warning('Invalid /fleet/lane/status JSON; sample ignored',
                                      throttle_duration_sec=5.0)
            return
        self.store.update('mission', data)

    def on_camera(self, name, msg):
        if 'jpeg' not in msg.format.lower() and 'jpg' not in msg.format.lower():
            self.get_logger().warning('Non-JPEG camera frame ignored', throttle_duration_sec=5.0)
        elif not self.frames.update_jpeg(name, msg.data):
            self.get_logger().warning('Invalid or oversized JPEG camera frame ignored', throttle_duration_sec=5.0)

    def process_controls(self):
        while not self.controls.requests.empty():
            request = self.controls.requests.get_nowait()
            action = request['action']
            if action in {'start', 'pause', 'resume'}:
                msg = String(); msg.data = json.dumps({'cmd': {'pause': 'stop', 'resume': 'resume'}.get(action, action)})
                self._lane_control_pub.publish(msg)
            elif action == 'reset':
                msg = String(); msg.data = json.dumps({'cmd': 'stop'}); self._lane_control_pub.publish(msg)
                for name, pose in self._initial_poses.items():
                    command = FleetCommand(); command.command = FleetCommand.CMD_SET_INITIAL_POSE
                    command.x, command.y, command.yaw = pose
                    self._fleet_control_pubs[name].publish(command)
            else:
                command = FleetCommand(); command.command = FleetCommand.CMD_SET_SPEED
                command.max_linear_vel = float(request['linear']); command.max_angular_vel = float(request['angular'])
                self._fleet_control_pubs[request['robot']].publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = None
    server = None
    thread = None
    try:
        node = LiveWebNode()
        map_asset = MapAsset(node.get_parameter('map_yaml').value)
        server = make_server(node.get_parameter('host').value,
                             node.get_parameter('port').value, node.store, map_asset, node.frames, node.controls)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        node.get_logger().info(f'Live read-only web: http://{server.server_address[0]}:'
                               f'{server.server_address[1]} (no robot commands)')
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if server is not None:
            if thread is not None:
                server.shutdown()
                thread.join(timeout=3)
            server.server_close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
