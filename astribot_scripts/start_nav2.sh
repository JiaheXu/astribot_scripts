#!/usr/bin/env bash
# Run alongside the existing Sensors / Rear FAST-LIO stack, in a new terminal.
set -euo pipefail
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    cat <<'HELP'
Usage: astribot_scripts/start_nav2.sh [launch arguments]
  enable_control:=false   Preview Nav2 feedback/planning without SDK motion output
  stamp_offset:=SECONDS   Verified ROS clock minus sensor clock offset (default 0)
  with_joints:=true       Receive body joints through the same SDK controller
  mount_file:=PATH        Optional measured base_from_imu / imu_from_lidar JSON
  params_file:=PATH       Nav2 Humble robot parameter overrides
Start the existing observation-only sensor publisher and rear FAST-LIO first.
HELP
    exit 0
fi
SDK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SDK_DIR"
# ROS and this repository's env.sh may expand unset environment variables.
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source ./env.sh
if [[ -f "${ROS2_WS:-$HOME/ros2_ws}/install/local_setup.bash" ]]; then
    source "${ROS2_WS:-$HOME/ros2_ws}/install/local_setup.bash"
fi
set -u
if ! ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
    printf '%s\n' 'Missing Nav2. Install: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup' >&2
    exit 1
fi
exec ros2 launch "$SDK_DIR/astribot_scripts/launch/astribot_nav2.launch.py" "$@"
