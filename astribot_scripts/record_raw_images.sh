#!/usr/bin/env bash
# Record RGB and depth images at 3 Hz per topic, with MCAP file rotation.
set -eo pipefail

usage() {
    cat <<'EOF'
Usage: record_raw_images.sh [new-bag-directory]

Records head, torso, left-wrist and right-wrist RGB and depth images to MCAP at 3 Hz per topic.
Targets files below 4 GB (4,000,000,000 bytes), rolling at 3.99 GB to reserve
10 MB for the last image/chunk and indexes. Recording continues in numbered
.mcap files. Press Ctrl-C to finalize the current file and metadata.yaml.
The output defaults to recordings/raw_rgbd_YYYYMMDD_HHMMSS under the SDK folder.
This command does not power/move the robot or start camera publishers.
EOF
}
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then usage; exit 0; fi
if (( $# > 1 )); then usage >&2; exit 2; fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
bag_dir="${1:-$SDK_DIR/recordings/raw_rgbd_$(date +%Y%m%d_%H%M%S)}"
if [[ -e "$bag_dir" ]]; then
    printf 'Output already exists; choose a new bag directory: %s\n' "$bag_dir" >&2
    exit 1
fi

source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "$SDK_DIR/env.sh"
if ! ros2 pkg prefix rosbag2_storage_mcap >/dev/null; then
    echo 'MCAP storage plugin is missing: install ros-humble-rosbag2-storage-mcap.' >&2
    exit 1
fi
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/astribot"
mkdir -p -- "$state_dir"
# The descriptor is inherited by the recorder, preventing duplicate desktop clicks.
exec 9>"$state_dir/raw-images-record.lock"
if ! flock -n 9; then
    echo 'RGB-D recording is already running. Use Ctrl-C in its terminal to stop it.' >&2
    if command -v notify-send >/dev/null; then
        notify-send -a Astribot 'RGB-D recording already running' \
            'Use Ctrl+C in the existing recording terminal to stop it.' || true
    fi
    exit 1
fi
mkdir -p -- "$(dirname -- "$bag_dir")"

printf 'Recording RGB and depth at 3 Hz per topic to %s (MCAP, 3.99 GB rollover).\n' "$bag_dir"
exec /usr/bin/python3 "$SCRIPT_DIR/record_rgbd_mcap.py" "$bag_dir"
