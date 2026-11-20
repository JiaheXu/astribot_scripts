#!/usr/bin/env python3
"""Separate robot power automation. Start/stop preview unless --execute is given."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from astribot_scripts.common.cli import positive_rate
from astribot_scripts.common.robot_lifecycle import DiagnosticMonitor, LifecycleError, RobotWeb, Sequencer


def run_home(lock_fd):
    script = Path(__file__).resolve().with_name('startup.py')
    with tempfile.TemporaryDirectory(prefix='astribot-home-') as directory:
        result = Path(directory)/'verified.json'
        # The child inherits this lock so a stuck SDK call cannot race another lifecycle action.
        child = subprocess.Popen([sys.executable, str(script), '--non-interactive',
                                  '--completion-file', str(result)],
                                 stdin=subprocess.DEVNULL, pass_fds=(lock_fd,))
        try:
            returncode = child.wait(timeout=45)
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                raise LifecycleError(f'Home helper PID {child.pid} has not exited after interrupt; '
                                     'it retains the robot-power lock. No power-off was sent.') from exc
            raise LifecycleError('Home interrupted or timed out; inspect the robot before continuing.') from exc
        if returncode:
            return returncode
        # The SDK can os._exit(0) after losing driver feedback. Exit 0 alone is
        # not proof that the helper reached its measured-home completion check.
        if not result.is_file():
            raise LifecycleError('Home helper exited without measured-home confirmation')
        confirmation = json.loads(result.read_text())
        if confirmation.get('stage') != 'home' or confirmation.get('success') is not True:
            raise LifecycleError('Home helper returned invalid completion feedback')
        return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'watch', 'start', 'stop'], nargs='?', default='status')
    parser.add_argument('--execute', action='store_true', help='send real power/drive commands and allow home motion')
    parser.add_argument('--host', default='192.168.0.10')
    parser.add_argument('--port', type=int, default=5141)
    parser.add_argument('--control-port', type=int, default=5143, help='passive vendor control-state WebSocket port')
    parser.add_argument('--timeout', type=positive_rate, default=60, help='seconds per power/drive transition')
    parser.add_argument('--activation-settle', type=positive_rate, default=5,
                        help='extra healthy seconds after activation before control handoff and home')
    parser.add_argument('--duration', type=positive_rate, default=10, help='seconds to passively watch diagnostics')
    args = parser.parse_args(argv)
    if args.execute and args.action in ('status', 'watch'):
        parser.error('--execute is only valid with start or stop')
    state = Path(os.environ.get('XDG_STATE_HOME', str(Path.home()/'.local/state'))) / 'astribot'
    state.mkdir(parents=True, exist_ok=True)
    lock = None
    monitor = None
    try:
        web = RobotWeb(args.host, args.port, control_port=args.control_port)
        if args.execute:
            lock = (state/'robot-power.lock').open('a')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise LifecycleError('Another robot-power or home operation is still running.') from exc
        monitor = DiagnosticMonitor()
        sequence = Sequencer(monitor, web, lambda: run_home(lock.fileno()), timeout=args.timeout,
                             settle_seconds=args.activation_settle,
                             report=lambda text: print(text, flush=True))
        if args.execute:
            getattr(sequence, args.action)()
            return 0
        status = sequence.wait('live state', lambda value: True)
        print(json.dumps({**status.summary(), 'sdk_control_owners': monitor.control_owners()}, indent=2), flush=True)
        if args.action == 'watch':
            deadline = time.monotonic()+args.duration
            while time.monotonic() < deadline:
                print(json.dumps(monitor.read().summary()), flush=True)
        elif args.action == 'start':
            print(f'[ROBOT] Passive control source: {web.control_source()}', flush=True)
            print('DRY RUN: power on if needed -> confirm power -> activate if needed -> confirm drives '
                  f'-> settle {args.activation_settle:g}s -> WEB-to-PC handoff -> confirm control released '
                  '-> SDK home (opens grippers) -> confirm home. Add --execute to run.', flush=True)
        elif args.action == 'stop':
            print('DRY RUN: deactivate if needed -> confirm inactive -> power off -> confirm off. '
                  'Add --execute to run. Observation buttons and nodes are separate.', flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f'[ROBOT] ERROR: {exc or "Interrupted"}. Sequence halted; no later step was sent.', file=sys.stderr, flush=True)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        if monitor is not None:
            monitor.close()
        if lock is not None:
            lock.close()


if __name__ == '__main__':
    raise SystemExit(main())
