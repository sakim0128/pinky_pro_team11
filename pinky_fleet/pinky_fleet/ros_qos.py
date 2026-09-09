"""ROS QoS 프로파일 모음.

모듈 최상단에서 rclpy 를 import 하지 않는다 — 부모 프로세스와 L0 dry-run 에서도
이 모듈을 로드할 수 있어야 하기 때문이다. 각 함수 안에서 늦게 import 한다.
"""

from __future__ import annotations


def amcl_pose_qos():
    """nav2 amcl 이 /amcl_pose 를 발행하는 QoS 와 정확히 같은 프로파일.

        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()

    왜 굳이 맞추는가
      amcl 은 **로봇이 정지해 있으면 /amcl_pose 를 발행하지 않는다.**
      (shouldUpdateFilter 가 update_min_d 0.25 m / update_min_a 0.2 rad 를 넘어야 갱신)
      그래서 volatile 로 구독하면 정지한 로봇의 위치를 영영 못 받는다 — 관제 화면이 빈다.
      transient_local 로 구독하면 붙는 즉시 마지막으로 발행된 값이 한 건 온다.
    """
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )

    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def map_qos():
    """nav2 map_server 의 /map 발행 QoS (reliable + transient_local)."""
    return amcl_pose_qos()
