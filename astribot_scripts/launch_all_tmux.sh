#!/usr/bin/env bash
# One SDK connection, rear FAST-LIO, and one combined RViz window.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SESSION=astribot-lio
TMUX_CMD=(tmux -L astribot-lio -f "$SCRIPT_DIR/config/astribot.tmux.conf")
ACTION=start
RVIZ=true
DETACH=false

usage() {
    cat <<'EOF'
Usage: launch_all_tmux.sh [start|attach|status|stop|check] [--headless] [--detach]

start (default)  Start sensors, rear FAST-LIO, robot model and RViz in three panes.
attach          Attach to the existing session; Ctrl-b d detaches without stopping.
status          Show each pane's running/exited state.
stop            Send Ctrl-C to all panes, wait for cleanup, then remove the session.
check           Check local dependencies and launch arguments without robot access.
--headless      Run sensors, FAST-LIO and robot_state_publisher without RViz.
--detach        Start without attaching (automatic when no interactive terminal).

Run GUI mode from a desktop terminal with DISPLAY or WAYLAND_DISPLAY configured.
Environment overrides: ROS_SETUP=/opt/ros/humble/setup.bash, ROS2_WS=$HOME/ros2_ws.
The robot's normal controller must already be running. No home/motion is requested.
The sensor publisher uses --observe-only; it does not acquire robot control.
Sensor drivers remain active after shutdown. No ROS bag is started.
EOF
}

for arg in "$@"; do
    case "$arg" in
        start|attach|status|stop|check) ACTION="$arg" ;;
        --headless) RVIZ=false ;;
        --detach) DETACH=true ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'Unknown argument: %s\n' "$arg" >&2; usage >&2; exit 2 ;;
    esac
done
command -v tmux >/dev/null || { echo 'tmux is required: sudo apt install tmux' >&2; exit 1; }

has_session() { "${TMUX_CMD[@]}" has-session -t "=$SESSION" 2>/dev/null; }
attach() {
    # This dedicated server is separate from any tmux session the user is in.
    exec env -u TMUX "${TMUX_CMD[@]}" attach-session -t "=$SESSION"
}
case "$ACTION" in
    attach) attach ;;
    status)
        "${TMUX_CMD[@]}" list-panes -t "=$SESSION" -F '#{pane_index}: #{pane_title} dead=#{pane_dead} exit=#{pane_dead_status}'
        exit ;;
    stop)
        if ! has_session; then echo 'No Astribot session is running.'; exit 0; fi
        while read -r pane dead; do
            if [[ "$dead" == 0 ]]; then "${TMUX_CMD[@]}" send-keys -t "$pane" C-c; fi
        done < <("${TMUX_CMD[@]}" list-panes -t "=$SESSION" -F '#{pane_id} #{pane_dead}')
        for ((i=0; i<15; i++)); do
            if ! has_session; then echo 'Astribot session stopped.'; exit 0; fi
            if ! "${TMUX_CMD[@]}" list-panes -t "=$SESSION" -F '#{pane_dead}' | grep -q '^0$'; then
                "${TMUX_CMD[@]}" kill-session -t "=$SESSION"
                echo 'Astribot session stopped; sensor drivers remain active.'
                exit 0
            fi
            sleep 1
        done
        echo 'Some panes have not exited. Inspect with: launch_all_tmux.sh attach' >&2
        exit 1 ;;
esac

if [[ "$ACTION" == start ]] && has_session; then
    echo 'Astribot session already exists; no duplicate processes started.'
    if [[ "$DETACH" == false && -t 0 && -t 1 ]]; then attach; fi
    exit 0
fi

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
for setup in "$ROS_SETUP" "$ROS2_WS/install/local_setup.bash" "$SDK_DIR/env.sh"; do
    [[ -f "$setup" ]] || { printf 'Missing environment setup: %s\n' "$setup" >&2; exit 1; }
done
# Colcon setup scripts are not nounset-safe. Source once before the tmux server
# starts, so all panes inherit one environment and do not race writing DDS XML.
source "$ROS_SETUP"
source "$SDK_DIR/env.sh"
# ROS and SDK underlays are already loaded; avoid stale absolute build-machine
# underlay paths embedded in the workspace's setup.bash.
source "$ROS2_WS/install/local_setup.bash"
export PYTHONUNBUFFERED=1
for package in fast_lio robot_state_publisher; do
    ros2 pkg prefix "$package" >/dev/null
done
[[ -x "$(ros2 pkg prefix fast_lio)/lib/fast_lio/fastlio_mapping" ]] || {
    echo 'Build the patched FAST_LIO in ros2_ws first.' >&2; exit 1;
}
if [[ "$RVIZ" == true ]]; then ros2 pkg prefix rviz2 >/dev/null; fi
python3 "$SCRIPT_DIR/publish_robot_data.py" --help >/dev/null
LIO=(ros2 launch "$SCRIPT_DIR/launch/astribot_fast_lio.launch.py" sensor:=rear rviz:=false)
MODEL=(ros2 launch "$SCRIPT_DIR/launch/astribot_lio_rviz.launch.py" "rviz:=$RVIZ")
if [[ "$ACTION" == check ]]; then
    "${LIO[@]}" --show-args
    "${MODEL[@]}" --show-args
    echo 'Dependency and launch checks passed. No SDK connection or nodes started.'
    exit 0
fi
if [[ "$RVIZ" == true && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo 'No desktop display found. Run from the PC desktop, or add --headless.' >&2
    exit 1
fi

# tmux executes strings through a shell; quote every argument, including paths.
shell_command() { printf 'exec '; printf '%q ' "$@"; }
sensor_cmd="$(shell_command python3 "$SCRIPT_DIR/publish_robot_data.py" --observe-only \
    --cameras head_rgbd torso_rgbd left_wrist_rgbd right_wrist_rgbd --no-depth --color-encoding rgb8 --image-rate 3 --image-size 640 480)"
lio_cmd="$(shell_command "${LIO[@]}")"
model_cmd="$(shell_command "${MODEL[@]}")"

# Create the layout before activating sensors. The initial shell is replaced
# only after all three panes have been configured.
sensor_pane="$("${TMUX_CMD[@]}" new-session -d -P -F '#{pane_id}' -s "$SESSION" -n observation \
    -c "$SDK_DIR" -x 180 -y 48 'exec /bin/bash --noprofile --norc')"
trap '"${TMUX_CMD[@]}" kill-session -t "=$SESSION" 2>/dev/null || true' ERR
"${TMUX_CMD[@]}" select-pane -t "$sensor_pane" -T 'Sensors / observation only'
lio_pane="$("${TMUX_CMD[@]}" split-window -h -t "$sensor_pane" -P -F '#{pane_id}' -c "$SDK_DIR" "$lio_cmd")"
"${TMUX_CMD[@]}" select-pane -t "$lio_pane" -T 'Rear FAST-LIO'
model_pane="$("${TMUX_CMD[@]}" split-window -v -t "$lio_pane" -P -F '#{pane_id}' -c "$SDK_DIR" "$model_cmd")"
"${TMUX_CMD[@]}" select-pane -t "$model_pane" -T 'Combined RViz'
"${TMUX_CMD[@]}" respawn-pane -k -t "$sensor_pane" -c "$SDK_DIR" "$sensor_cmd"
"${TMUX_CMD[@]}" select-pane -t "$sensor_pane"
trap - ERR
echo 'Astribot started. Ctrl-b d detaches; launch_all_tmux.sh stop stops all three panes.'
if [[ "$DETACH" == false && -t 0 && -t 1 ]]; then attach; fi
