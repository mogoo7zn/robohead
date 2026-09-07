#!/usr/bin/env bash
# Launch the RoboGame 2026 mission on the Raspberry Pi.
# Requires: ROS 2 (jazzy) sourced, core installed
# (pip install -e .), cameras publishing, STM32 on the serial port.
#
# Usage: bash scripts/start_robot.sh [field_file]
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIELD_FILE="${1:-real_field.yaml}"

# sanity checks before the match
echo "== pre-flight =="
python3 "$REPO_DIR/scripts/check_config.py"

if [ ! -f "$REPO_DIR/config/$FIELD_FILE" ]; then
    echo "ERROR: config/$FIELD_FILE not found" >&2
    exit 1
fi

# ROS 2 environment
if [ -z "${ROS_DISTRO:-}" ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash || true
fi
# workspace overlay (built via make ros-build or colcon on the Pi)
WS="$REPO_DIR/ros2_ws/install/setup.bash"
if [ -f "$WS" ]; then
    # shellcheck disable=SC1091
    source "$WS"
fi

echo "== launching mission node (field=$FIELD_FILE) =="
export ROBOGAME_CONFIG_DIR="$REPO_DIR/config"
exec ros2 launch robogame_bringup mission.launch.py \
    config_dir:="$REPO_DIR" \
    field_file:="$FIELD_FILE" \
    control_period:=0.02
