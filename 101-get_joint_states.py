#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2024, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------
# Author: Astribot Team
# -----------------------------------------------------------------------------

"""Continuously display current and desired joint positions, velocities and torques.

--rate controls display refresh (default 1 Hz), not the robot publication rate.
Press Ctrl-C to stop. No joint commands or file writes are issued.
"""

import argparse
import math
import time

import rclpy
from tabulate import tabulate
from astribot_sdk.core.astribot_api.astribot_client import Astribot


def positive_rate(value):
    rate = float(value)
    if not math.isfinite(rate) or rate <= 0:
        raise argparse.ArgumentTypeError("rate must be finite and positive")
    return rate


def report(astribot):
    for state in ("current", "desired"):
        for quantity in ("position", "velocity", "torque"):
            values = getattr(astribot, f"get_{state}_joints_{quantity}")()
            rows = [[name, value] for name, value in zip(astribot.whole_body_names, values)]
            print(tabulate(rows, headers=["Part", f"{state} {quantity}"], tablefmt="grid"), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=positive_rate, default=1.0, help="display refresh Hz")
    args = parser.parse_args()
    try:
        astribot = Astribot()
        while rclpy.ok():
            report(astribot)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
