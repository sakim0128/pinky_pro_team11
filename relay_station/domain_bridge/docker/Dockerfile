FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-domain-bridge \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-cv-bridge \
    python3-opencv \
    python3-psutil \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

COPY relay_station/domain_bridge/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
