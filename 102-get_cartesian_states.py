#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2024, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------
# Author: Astribot Team
# -----------------------------------------------------------------------------

"""Continuously display both arms' current and desired EE poses in one frame.

Pose order: x, y, z (meters), qx, qy, qz, qw.
--frame selects chassis (default) or world. Press Ctrl-C to stop.
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


def report(astribot, frame):
    for state in ("current", "desired"):
        # Gripper names return opening values; arm names return 7D EE poses.
        poses = getattr(astribot, f"get_{state}_cartesian_pose")(names=astribot.arm_names, frame=frame)
        rows = [[name, *pose] for name, pose in zip(astribot.arm_names, poses)]
        print(f"{state} EE poses, frame={frame}", flush=True)
        print(tabulate(rows, headers=["Arm", "x(m)", "y(m)", "z(m)", "qx", "qy", "qz", "qw"], tablefmt="grid"), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=positive_rate, default=1.0, help="display refresh Hz")
    parser.add_argument("--frame", choices=("chassis", "world"), default="chassis")
    args = parser.parse_args()
    try:
        astribot = Astribot()
        while rclpy.ok():
            report(astribot, args.frame)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
