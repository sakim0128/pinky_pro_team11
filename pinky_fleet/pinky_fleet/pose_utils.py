"""좌표/자세 변환 유틸.

수업자료 `주피터로 내비게이션 주행하기` p10 의 get_quaternion_from_yaw() 를 옮긴 것이다.
자료는 tf_transformations.quaternion_from_euler(0, 0, yaw) 를 썼는데, roll=pitch=0 이면
결과가 (0, 0, sin(yaw/2), cos(yaw/2)) 로 딱 떨어지므로 여기서는 의존성을 하나 줄이려고
직접 계산한다. 값은 완전히 동일하다.

모듈 최상단에서 ROS를 import하지 않는다 — 부모 프로세스와 L0 dry-run에서도 쓰기 때문.
"""

from __future__ import annotations

import math


def quaternion_from_yaw_deg(yaw_degrees: float):
    """yaw[deg] -> (x, y, z, w).  tf_transformations.quaternion_from_euler(0,0,yaw) 와 동일."""
    yaw_radians = math.radians(yaw_degrees)
    half = yaw_radians / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def yaw_deg_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """(x,y,z,w) -> yaw[deg]. roll/pitch가 0인 평면 주행 로봇 전제."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def yaw_deg_between(x1: float, y1: float, x2: float, y2: float) -> float:
    """점1에서 점2를 바라보는 각도[deg]. RViz 2-click 목적지 입력에 쓴다."""
    return math.degrees(math.atan2(y2 - y1, x2 - x1))


def normalize_deg(deg: float) -> float:
    """각도를 [-180, 180) 으로 감싼다.

    07강 정리 2번 — 각도 오차는 반드시 감싸서 계산해야 한다.
    """
    return (deg + 180.0) % 360.0 - 180.0


def distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)


def pose2d_to_pose_stamped(pose2d, frame_id: str, stamp):
    """Pose2D -> geometry_msgs/PoseStamped.

    geometry_msgs 는 이 함수 안에서만 import 한다 (ROS 없는 프로세스에서도 모듈 로드 가능).
    `stamp` 는 node.get_clock().now().to_msg() 결과.
    """
    from geometry_msgs.msg import PoseStamped

    q = quaternion_from_yaw_deg(pose2d.yaw_deg)
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x = float(pose2d.x)
    msg.pose.position.y = float(pose2d.y)
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    return msg
