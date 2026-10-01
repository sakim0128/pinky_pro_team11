#!/usr/bin/env bash
# =============================================================================
# Mock Robot Publisher Runner (DOMAIN 10)
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
echo " 🤖 Launching Mock Robot Publisher (/pinky1 & /pinky2)"
echo "============================================================"

cd "$SCRIPT_DIR"
python3 mock_robot_publisher.py "$@"
