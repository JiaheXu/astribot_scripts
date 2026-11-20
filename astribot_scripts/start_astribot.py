#!/usr/bin/env python3
"""Start a fresh observation stack, gracefully replacing an existing session."""
if __package__:
    from .restart_astribot import main
else:
    from restart_astribot import main


if __name__ == '__main__':
    raise SystemExit(main('start'))
