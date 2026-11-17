#!/usr/bin/env bash
# Record relayed sensors/state plus original Livox topics for KISS-ICP replay.
set -euo pipefail
if (( $# > 1 )); then
    echo "Usage: $0 [new-bag-directory]" >&2
    exit 2
fi
bag_dir="${1:-robot_data_$(date +%Y%m%d_%H%M%S)}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 bag record -o "$bag_dir" --qos-profile-overrides-path "$script_dir/record_qos.yaml" \
    --regex '^(/sdk/.*|/astribot_viz/(joint_states|current_poses|desired_poses)|/livox/(lidar|imu)_(front|back)|/astribot_whole_body/imu_sensor/astribot_chassis_imu_site)$'
