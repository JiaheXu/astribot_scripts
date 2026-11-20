#!/usr/bin/env python3
"""Run a GUI on this Linux PC's desktop, including when invoked over SSH."""
import argparse
import os
from pathlib import Path
import re
import subprocess


def desktop_environment(environment=None, proc=Path('/proc')):
    env = dict(os.environ if environment is None else environment)
    candidates = []
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            args = (entry / 'cmdline').read_bytes().decode().split('\0')
            if Path(args[0]).name != 'Xwayland' or '-auth' not in args:
                continue
            display = next(arg for arg in args[1:] if re.fullmatch(r':\d+(?:\.\d+)?', arg))
            authority = args[args.index('-auth') + 1]
            if Path(authority).is_file():
                candidates.append((display, authority))
        except (OSError, UnicodeError, StopIteration, IndexError):
            continue
    # Prefer a local desktop already selected by the caller, never SSH X11 forwarding.
    current = env.get('DISPLAY', '')
    matching = [item for item in candidates if item[0] == current]
    if len(matching) == 1:
        candidates = matching
    if len(candidates) == 1:
        env['DISPLAY'], env['XAUTHORITY'] = candidates[0]
    elif not re.fullmatch(r':\d+(?:\.\d+)?', current):
        raise RuntimeError('No unique local desktop found. Run tmuxp from the receiving PC desktop.')
    env['XDG_RUNTIME_DIR'] = env.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')
    env['QT_QPA_PLATFORM'] = 'xcb'
    probe = subprocess.run(['xdpyinfo'], env=env, capture_output=True, timeout=5)
    if probe.returncode:
        raise RuntimeError(f'Cannot open local desktop {env.get("DISPLAY")}; check its login/X authority.')
    return env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', choices=['true', 'false'], default='true')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command[:1] == ['--']:
        command = command[1:]
    if not command:
        parser.error('supply a command after --')
    try:
        env = desktop_environment() if args.gui == 'true' else dict(os.environ)
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
        parser.exit(1, f'{exc}\n')
    if args.gui == 'true':
        print(f'Opening RViz on this PC desktop: DISPLAY={env["DISPLAY"]}', flush=True)
    os.execvpe(command[0], command, env)


if __name__ == '__main__':
    main()
