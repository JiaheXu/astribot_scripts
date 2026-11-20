#!/usr/bin/env bash
# Separate desktop startup: retain the terminal so completion/errors stay visible.
set -uo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/astribot"
mkdir -p -- "$state_dir" || exit 1
log_file="$state_dir/robot-start-$(date +%Y%m%d_%H%M%S)-$$.log"
: > "$log_file" || exit 1
export PYTHONUNBUFFERED=1

printf '%s\n' \
    'Robot startup: power on -> activate -> settle -> PC control -> home.' \
    'Homing moves the torso, arms and head and opens the grippers.' \
    "Startup log: $log_file" \
    ''

"$SCRIPT_DIR/robot_power.sh" start --execute 2>&1 | tee "$log_file"
exit_code=${PIPESTATUS[0]}
if (( exit_code != 0 )); then
    printf '\nRobot startup did not complete (exit %s). Review the messages above.\n' "$exit_code" >&2
else
    printf '\nRobot startup finished successfully.\n'
fi

if [[ -t 0 ]]; then
    read -r -p 'Press Enter to close this window. ' _ || true
fi
exit "$exit_code"
