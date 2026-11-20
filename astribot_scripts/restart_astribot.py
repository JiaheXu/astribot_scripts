#!/usr/bin/env python3
"""Shared desktop lifecycle; the legacy entry point starts a fresh stack."""
from datetime import datetime
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

import yaml

ROOT = Path(__file__).resolve().parent.parent
SCRIPT_DIR = ROOT / 'astribot_scripts'
SESSION = 'astribot-lio'
TMUX = ['tmux', '-L', SESSION]


def notify(title, message):
    if shutil.which('notify-send'):
        try:
            subprocess.run(['notify-send', '-a', 'Astribot', '-t', '7000', title, message],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            pass  # Desktop notifications must never interrupt the restart.


def owned_processes(pane_pids, process_rows):
    """Find this session's process tree, excluding unrelated ROS processes."""
    records = {}
    for row in process_rows.splitlines():
        fields = row.strip().split(None, 2)
        if len(fields) == 3:
            records[int(fields[0])] = (int(fields[1]), fields[2])
    owned = set(pane_pids)
    while True:
        new = {pid for pid, (parent, _) in records.items() if parent in owned}
        if new.issubset(owned):
            break
        owned |= new
    return {pid: records[pid] for pid in owned if pid in records}


def stack_processes(pane_pids, process_rows):
    """Count only processes owned by this session, never unrelated ROS nodes."""
    counts = {'sensors': 0, 'mapping': 0, 'rviz': 0}
    for _, command in owned_processes(pane_pids, process_rows).values():
        args = command.split()
        executable = Path(args[0]).name
        if any(Path(arg).name == 'publish_robot_data.py' for arg in args[1:]):
            counts['sensors'] += 1
        if executable == 'fastlio_mapping':
            counts['mapping'] += 1
        if executable == 'rviz2':
            counts['rviz'] += 1
    return counts


def stop_stack(run):
    """Stop the dedicated session and verify its original descendants exited."""
    panes = subprocess.run(TMUX + ['list-panes', '-t', '=' + SESSION, '-F', '#{pane_pid}'],
                           capture_output=True, text=True, timeout=5)
    roots = {int(pid) for pid in panes.stdout.split()} if panes.returncode == 0 else set()
    before = owned_processes(roots, subprocess.check_output(['ps', '-eo', 'pid=,ppid=,args='], text=True))
    run(['bash', str(SCRIPT_DIR / 'launch_all_tmux.sh'), 'stop'], timeout=25)
    if subprocess.run(TMUX + ['has-session', '-t', '=' + SESSION],
                      capture_output=True, timeout=5).returncode == 0:
        raise RuntimeError('Astribot tmux session is still running after Stop.')
    deadline = time.monotonic() + 5
    while True:
        # Match original command lines as well as PIDs to avoid PID reuse.
        rows = subprocess.check_output(['ps', '-eo', 'pid=,ppid=,args='], text=True)
        remaining = {pid: command for pid, (_, command) in owned_processes(set(before), rows).items()
                     if pid in before and command == before[pid][1]}
        if not remaining:
            return
        if time.monotonic() >= deadline:
            raise RuntimeError(f'Some stack processes have not exited: {sorted(remaining)}')
        time.sleep(0.2)


def wait_started(gui):
    deadline = time.monotonic() + 30
    stable_since = None
    expected = {'sensors': 1, 'mapping': 1, 'rviz': int(gui)}
    while time.monotonic() < deadline:
        result = subprocess.run(TMUX + ['list-panes', '-t', '=' + SESSION,
                                       '-F', '#{pane_pid} #{pane_dead}'], capture_output=True, text=True)
        rows = [line.split() for line in result.stdout.splitlines()]
        if result.returncode or len(rows) != 3 or any(dead != '0' for _, dead in rows):
            raise RuntimeError('The tmux session did not start all three panes. Inspect its pane output.')
        processes = subprocess.check_output(['ps', '-eo', 'pid=,ppid=,args='], text=True)
        counts = stack_processes({int(pid) for pid, _ in rows}, processes)
        if counts == expected:
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= 2:
                return counts
        else:
            stable_since = None
        time.sleep(0.25)
    raise RuntimeError(f'Timed out waiting for the stack: {counts}')


def main(action='start'):
    if action not in ('start', 'stop'):
        raise ValueError('Expected start or stop')
    state = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'astribot'
    state.mkdir(parents=True, exist_ok=True)
    # Keep the old lock filename so legacy Restart and both new buttons serialize.
    with (state / 'restart.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            if action == 'start':
                notify('Astribot action already running', 'Please wait for the current Start or Stop to finish.')
                return 0
            notify('Astribot Stop queued', 'Waiting for the current action to finish, then stopping the stack.')
            fcntl.flock(lock, fcntl.LOCK_EX)
        log_path = state / f'{action}.log'
        previous = state / f'{action}.previous.log'
        if log_path.exists():
            log_path.replace(previous)
        with log_path.open('w', buffering=1) as log:
            def run(args, timeout=60):
                print('$ ' + ' '.join(map(str, args)), file=log)
                subprocess.run(args, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                               check=True, timeout=timeout)
            try:
                print(f'{action.title()} requested at {datetime.now().astimezone().isoformat()}', file=log)
                if not shutil.which('tmux'):
                    raise RuntimeError('tmux is not installed')
                if action == 'stop':
                    notify('Stopping Astribot', 'Closing the sensor publisher, FAST-LIO and RViz.')
                    stop_stack(run)
                    print('Stop complete: session and its processes have exited.', file=log)
                    notify('Astribot stopped', 'The observation stack and its tmux session are stopped.')
                    return 0
                config = SCRIPT_DIR / 'config/astribot.tmux.yaml'
                settings = yaml.safe_load(config.read_text())
                if settings.get('session_name') != SESSION:
                    raise RuntimeError('The YAML session_name must be astribot-lio.')
                gui = str(settings.get('environment', {}).get('ASTRIBOT_RVIZ', 'true')).lower() == 'true'
                for program in ('tmux', 'tmuxp'):
                    if not shutil.which(program):
                        raise RuntimeError(f'{program} is not installed')
                # Check dependencies/display before disrupting the running stack.
                check = ['bash', str(SCRIPT_DIR / 'launch_all_tmux.sh'), 'check']
                run(check + ([] if gui else ['--headless']))
                run([sys.executable, str(SCRIPT_DIR / 'desktop_exec.py'), '--gui', str(gui).lower(), '--', '/usr/bin/true'])
                notify('Starting Astribot', 'Replacing any running stack and starting a fresh FAST-LIO map.')
                stop_stack(run)
                run(['tmuxp', 'load', '-L', SESSION, '-d', str(config)])
                counts = wait_started(gui)
                # Process creation precedes SDK discovery and sensor activation.
                # Only report success once RViz has fresh robot/map/image data.
                run(['bash', '-c',
                     'set -e; source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"; '
                     'source ./env.sh; source "${ROS2_WS:-$HOME/ros2_ws}/install/local_setup.bash"; '
                     'exec python3 ./astribot_scripts/check_stack_ready.py --timeout 100'], timeout=110)
                counts = wait_started(gui)
                print(f'Start complete: {counts}', file=log)
                notify('Astribot started', 'Sensors, rear FAST-LIO and visualization are running.')
                return 0
            except Exception as exc:
                traceback.print_exc(file=log)
                subprocess.run(TMUX + ['list-panes', '-t', '=' + SESSION, '-F', '#{pane_id}: #{pane_title} dead=#{pane_dead}'],
                               stdout=log, stderr=subprocess.STDOUT)
                notify(f'Astribot {action} failed', f'{exc}\nSee {log_path}')
                if shutil.which('zenity'):
                    subprocess.Popen(['zenity', '--error', f'--title=Astribot {action} failed',
                                      f'--text={exc}\n\nLog: {log_path}', '--width=500'],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return 1


if __name__ == '__main__':
    raise SystemExit(main())
