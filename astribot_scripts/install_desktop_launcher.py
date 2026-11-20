#!/usr/bin/env python3
"""Install observation, recording, robot startup and operator-guide shortcuts."""
import os
from pathlib import Path
import subprocess


def desktop_quote(value):
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('`', '\\`').replace('$', '\\$').replace('%', '%%') + '"'


def main():
    scripts = Path(__file__).resolve().parent
    desktop = Path(subprocess.check_output(['xdg-user-dir', 'DESKTOP'], text=True).strip())
    desktop.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env.setdefault('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')
    env.setdefault('DBUS_SESSION_BUS_ADDRESS', f'unix:path=/run/user/{os.getuid()}/bus')
    entries = [
        ('Start Astribot', 'start', '/usr/bin/python3', 'start_astribot.py', False,
         'Start sensors, rear FAST-LIO and RViz; replace any existing observation session'),
        ('Stop Astribot', 'stop', '/usr/bin/python3', 'stop_astribot.py', False,
         'Stop the observation stack and RViz; leave robot control and sensor drivers active'),
        ('Record RGB-D MCAP', 'record', '/usr/bin/bash', 'record_raw_images.sh', True,
         'Record RGB and depth from four cameras at 3 Hz per topic. Press Ctrl+C to finish.'),
        ('Power On & Home Robot', 'power', '/usr/bin/bash', 'start_robot_power_desktop.sh', True,
         'Power on, confirm power, activate, confirm drives, then move the robot home and open grippers'),
        ('Astribot Button Guide', 'guide', '/usr/bin/xdg-open', 'docs/desktop-buttons.html', False,
         'Read the offline guide to robot startup, observation, recording and shutdown buttons'),
    ]
    for name, action, executable, script, terminal, comment in entries:
        # Keep the existing recorder shortcut and its desktop position on upgrade.
        filename = 'Record RGB MCAP' if action == 'record' else name
        launcher = desktop / f'{filename}.desktop'
        launcher.write_text('\n'.join([
            '[Desktop Entry]', 'Type=Application', 'Version=1.0',
            f'Name={name}', f'Comment={comment}',
            f'Exec={executable} {desktop_quote(scripts / script)}',
            f'Path={scripts.parent}', f'Icon={scripts / ("config/astribot-" + action + ".svg")}',
            f'Terminal={str(terminal).lower()}', 'StartupNotify=false', 'Categories=Utility;', '',
        ]))
        launcher.chmod(0o755)
        subprocess.run(['desktop-file-validate', str(launcher)], check=True)
        subprocess.run(['gio', 'set', str(launcher), 'metadata::trusted', 'true'], env=env, check=True)
        print(launcher)
    # Remove only the old launcher belonging to this installation.
    legacy = desktop / 'Restart Astribot.desktop'
    if legacy.exists() and str(scripts / 'restart_astribot.py') in legacy.read_text():
        legacy.unlink()


if __name__ == '__main__':
    main()
