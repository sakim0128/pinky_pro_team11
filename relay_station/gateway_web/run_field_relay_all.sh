#!/usr/bin/env bash
# =============================================================================
# Educational Field Gateway Relay - All-In-One Runner (with Gazebo Simulation)
# - Gazebo 3D Simulation Engine (pinky_factory.world)
# - Tablet Camera Single Ingest & 1:N Fan-Out Web Streaming Server
# - ROS 2 Tablet Camera Publisher (DOMAIN_ID=10, /camera/image_raw)
# - Field Control Monitor Screen Synthesizer (/control_feed)
# - Dual Robot Status API & Responsive Dashboard (port 8889)
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$HOME/pinky_pro"

# 1. Source ROS 2 environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# 2. Source workspace environment
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# 3. Set Gazebo & ROS environment
export ROS_DOMAIN_ID=10
export PYTHONUNBUFFERED=1
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix pinky_description 2>/dev/null)/share/pinky_description/..:$(ros2 pkg prefix pinky_gz_sim 2>/dev/null)/share/pinky_gz_sim/models:${HOME}/.gazebo/models:${GZ_SIM_RESOURCE_PATH}"
export LIBGL_DRI3_DISABLE=1

# 옵션 체크: --no-gz 지정 시 Gazebo 실행 건너뜀
LAUNCH_GAZEBO=true
SERVER_ARGS=()
for arg in "$@"; do
    if [ "$arg" == "--no-gz" ]; then
        LAUNCH_GAZEBO=false
    else
        SERVER_ARGS+=("$arg")
    fi
done

echo "============================================================"
echo " 🛰️  Field Gateway Relay + Gazebo System (ROS_DOMAIN_ID=10)"
echo "============================================================"
echo " - LAN IP       : 198.51.100.3"
echo " - Wi-Fi IP     : 203.0.113.150"
echo " - Tailscale IP : 100.64.0.81"
echo " - Web Port     : 8889"
echo " - Gazebo Sim   : $LAUNCH_GAZEBO"
echo "============================================================"

# 정리 함수
cleanup() {
    echo -e "\n[INFO] Shutting down Gateway Relay & Gazebo processes..."
    if [ -n "$GZ_PID" ]; then
        kill "$GZ_PID" 2>/dev/null || true
    fi
    pkill -9 -f "gz sim" >/dev/null 2>&1 || true
    pkill -9 -f "ros_gz_bridge" >/dev/null 2>&1 || true
    pkill -9 -f "ros_gz_image" >/dev/null 2>&1 || true
    fuser -k 8889/tcp >/dev/null 2>&1 || true
    echo "[INFO] Cleaned up cleanly."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# 4. Gazebo 3D 시뮬레이션 백그라운드 기동
GZ_PID=""
if [ "$LAUNCH_GAZEBO" = true ]; then
    echo "[INFO] Launching Gazebo 3D Simulation (pinky_factory.world)..."
    ros2 launch pinky_gz_sim launch_sim.launch.xml >/dev/null 2>&1 &
    GZ_PID=$!
    echo "[INFO] Gazebo Sim started (PID: $GZ_PID). Waiting 3s for physics engine..."
    sleep 3
fi

# 5. 게이트웨이 웹 서버 기동
cd "$SCRIPT_DIR"
python3 gateway_web_server.py "${SERVER_ARGS[@]}"
