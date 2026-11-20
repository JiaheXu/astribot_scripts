#!/usr/bin/env bash
# Run on this Mac/Linux workstation. SSH prompts normally; no passwords stored.
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
camera_host=${CAMERA_HOST:-fourier@192.168.100.102}
gpu_host=${GPU_HOST:-ubuntu@192.168.200.102}
container=${ISAAC_ROS_CONTAINER:-isaac_ros_pose_humble}
seconds=${CHECK_SECONDS:-20}
action=${1:-ensure}
strict=${2:-}
if [[ ! $container =~ ^[a-zA-Z0-9_.-]+$ || ! $seconds =~ ^[0-9]+$ ]]; then
    echo 'Invalid container name or CHECK_SECONDS' >&2; exit 1
fi
if [[ $action != ensure && $action != check && $action != status ]] ||
   [[ -n $strict && $strict != --require-registered ]]; then
    echo "Usage: $0 {ensure|check|status} [--require-registered]" >&2; exit 1
fi
# macOS TMPDIR can exceed OpenSSH's Unix-socket path limit with %C.
control_dir=$(mktemp -d /tmp/fp-ssh.XXXXXX)
ssh_options=(-o ConnectTimeout=10 -o ServerAliveInterval=15 -o ServerAliveCountMax=3
             -o ControlMaster=auto -o ControlPersist=600 -o "ControlPath=$control_dir/%C")
cleanup() {
    ssh "${ssh_options[@]}" -O exit "$camera_host" 2>/dev/null || true
    ssh "${ssh_options[@]}" -O exit "$gpu_host" 2>/dev/null || true
    rm -rf "$control_dir"
}
trap cleanup EXIT
ssh_camera() { ssh "${ssh_options[@]}" "$camera_host" "$@"; }
ssh_gpu() { ssh "${ssh_options[@]}" "$gpu_host" "$@"; }
ssh_camera true
ssh_gpu true
if [[ $action == status ]]; then
    ssh_camera 'systemctl --user status astribot-camera-link --no-pager'
    ssh_gpu "docker inspect --format '{{.State.Status}} network={{.HostConfig.NetworkMode}}' $container; tail -8 ~/workspaces/isaac_ros_humble/logs/astribot-foundationpose.log"
    exit
fi
if [[ $action == ensure ]]; then
    # These existing, purpose-specific deployments are prerequisites. Never copy
    # local calibration over measured values on the GPU server.
    ssh_camera 'test -f ~/astribot_sdk_ros2/astribot_scripts/common/sensors.py && mkdir -p ~/.config/systemd/user'
    ssh_gpu 'test -f ~/workspaces/isaac_ros_humble/astribot_camera_inputs/astribot_scripts/config/rgbd_pointcloud.yaml'
    rsync_ssh="ssh -o ControlMaster=auto -o ControlPersist=600 -o ControlPath=$control_dir/%C"
    camera_changes=$(rsync -rtci --backup --suffix=.previous -e "$rsync_ssh" \
        "$here/camera_link.py" "$here/source_peer.xml" "$here/astribot-camera-link.service" "$here/activate_cameras.py" \
        "$camera_host:astribot_sdk_ros2/astribot_scripts/integrations/foundationpose/")
    gpu_changes=$(rsync -rtci --backup --suffix=.previous -e "$rsync_ssh" \
        "$here/inputs.py" "$here/astribot.launch.py" "$here/gpu_peer.xml" \
        "$here/run_gpu.sh" "$here/check_inputs.py" \
        "$gpu_host:workspaces/isaac_ros_humble/astribot_camera_inputs/astribot_scripts/integrations/foundationpose/")
    ssh_camera 'systemctl --user link ~/astribot_sdk_ros2/astribot_scripts/integrations/foundationpose/astribot-camera-link.service; systemctl --user daemon-reload; systemctl --user enable --now astribot-camera-link'
    if [[ -n $camera_changes ]]; then ssh_camera 'systemctl --user restart astribot-camera-link'; fi
    if [[ ${ACTIVATE_CAMERAS:-1} == 1 ]]; then
        ssh_camera 'source /opt/ros/humble/setup.bash
            export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/astribot_sdk_ros2/astribot_scripts/integrations/foundationpose/source_peer.xml
            python3 ~/astribot_sdk_ros2/astribot_scripts/integrations/foundationpose/activate_cameras.py' ||
            echo 'Camera activation did not succeed; the frame check below will determine actual availability.' >&2
    fi
    restart_gpu=0
    if [[ -n $gpu_changes ]]; then restart_gpu=1; fi
    ssh_gpu bash -s -- "$container" "$restart_gpu" <<'REMOTE'
set -euo pipefail
container=$1
[[ $(docker inspect --format '{{.HostConfig.NetworkMode}}' "$container") == host ]] || {
    echo 'FoundationPose container must use host networking' >&2; exit 1;
}
if [[ $(docker inspect --format '{{.State.Running}}' "$container") != true ]]; then docker start "$container"; fi
# Stop only the launch owned by run_gpu.sh, and only if its files changed.
docker exec -i "$container" bash -s -- "$2" <<'INNER'
set -e
lock=/tmp/astribot-foundationpose-26.lock
pidfile=/tmp/astribot-foundationpose-26.pid
if [[ $1 == 1 ]] && ! flock -n "$lock" true; then
    pid=$(cat "$pidfile")
    [[ $pid =~ ^[0-9]+$ ]] || exit 1
    cmd=$(tr '\0' ' ' <"/proc/$pid/cmdline")
    [[ $cmd == *'ros2 launch /workspaces/isaac_ros-dev/astribot_camera_inputs/astribot_scripts/integrations/foundationpose/astribot.launch.py'* ]] || {
        echo 'Refusing to signal an unrecognized process' >&2; exit 1;
    }
    kill -INT "$pid"
    for ((i=0; i<30; i++)); do
        if flock -n "$lock" true; then break; fi
        sleep 1
    done
    flock -n "$lock" true || { echo 'Previous launch did not stop; inspect its log' >&2; exit 1; }
fi
INNER
if docker exec "$container" flock -n /tmp/astribot-foundationpose-26.lock true; then
    docker exec -d "$container" bash -c '
        mkdir -p /workspaces/isaac_ros-dev/logs
        exec /workspaces/isaac_ros-dev/astribot_camera_inputs/astribot_scripts/integrations/foundationpose/run_gpu.sh >>/workspaces/isaac_ros-dev/logs/astribot-foundationpose.log 2>&1'
    # Give TensorRT/NITROS startup a bounded head start before measuring frames.
    sleep 12
fi
REMOTE
fi
ssh_gpu "docker exec -e ROS_DOMAIN_ID=26 -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/astribot_camera_inputs/astribot_scripts/integrations/foundationpose/gpu_peer.xml $container /usr/local/bin/isaac-humble-entrypoint python3 /workspaces/isaac_ros-dev/astribot_camera_inputs/astribot_scripts/integrations/foundationpose/check_inputs.py --duration $seconds --report /workspaces/isaac_ros-dev/logs/camera-input-check.json $strict"
