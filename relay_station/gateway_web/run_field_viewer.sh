#!/usr/bin/env bash
# =============================================================================
# Field Low-Latency ROS 2 Viewer Runner (<0.01s Latency HUD)
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

export ROS_DOMAIN_ID=10
export PYTHONUNBUFFERED=1

echo "============================================================"
echo " 🖥️  Launching Field Low-Latency ROS 2 Viewer (DOMAIN 10)"
echo "============================================================"

cd "$SCRIPT_DIR"
python3 field_low_latency_viewer.py "$@"
