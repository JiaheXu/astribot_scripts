#!/usr/bin/env python3
"""Stop the observation stack and RViz; leave robot control and drivers active."""
if __package__:
    from .restart_astribot import main
else:
    from restart_astribot import main


if __name__ == '__main__':
    raise SystemExit(main('stop'))
