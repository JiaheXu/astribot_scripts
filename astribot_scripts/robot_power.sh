#!/usr/bin/env bash
# Robot power/home only; does not launch or stop the observation stack.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source ./env.sh
exec python3 "$SCRIPT_DIR/robot_power.py" "$@"
