#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2024, Astribot Co., Ltd.
# All rights reserved. License: BSD 3-Clause License
"""Continuously print joint and Cartesian states using examples 101 and 102.

Print current and desired joint positions, velocities and torques for all parts,
plus Cartesian poses for chassis, torso, both arms and head. Gripper openings
are joint values, not Cartesian poses. No files, topics or motion commands are
written. --rate defaults to 1 Hz; --frame defaults to chassis. Ctrl-C stops.
"""

import argparse
import math
import time
from unittest.mock import patch

import rclpy
from tabulate import tabulate
from astribot_sdk.core.astribot_api.astribot_client import Astribot


def positive_rate(value):
    rate = float(value)
    if not math.isfinite(rate) or rate <= 0:
        raise argparse.ArgumentTypeError('rate must be finite and positive')
    return rate


def report(astribot, frame):
    names = astribot.whole_body_names
    pose_names = [name for name in names if name not in astribot.effector_names]
    for state in ('current', 'desired'):
        for quantity in ('position', 'velocity', 'torque'):
            values = getattr(astribot, f'get_{state}_joints_{quantity}')()
            if values is None or len(values) != len(names):
                raise ValueError(f'Incomplete {state} joint {quantity} data')
            rows = [[name, value] for name, value in zip(names, values)]
            print(f'\n{state} joint {quantity}', flush=True)
            print(tabulate(rows, headers=['Part', 'Joint values'], tablefmt='grid'), flush=True)

        poses = getattr(astribot, f'get_{state}_cartesian_pose')(names=pose_names, frame=frame)
        if poses is None or len(poses) != len(pose_names) or any(len(pose) != 7 for pose in poses):
            raise ValueError(f'Incomplete {state} Cartesian pose data')
        print(f'\n{state} Cartesian poses, frame={frame}', flush=True)
        rows = [[name, *pose] for name, pose in zip(pose_names, poses)]
        print(tabulate(rows, headers=['Part', 'x(m)', 'y(m)', 'z(m)', 'qx', 'qy', 'qz', 'qw'],
                       tablefmt='grid'), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rate', type=positive_rate, default=1.0, help='display refresh Hz')
    parser.add_argument('--frame', choices=('chassis', 'world'), default='chassis')
    args = parser.parse_args()
    try:
        # Answer Enter during SDK startup; do not opt into taking another controller's rights.
        with patch('builtins.input', return_value=''):
            astribot = Astribot()
        print('Reading joint and Cartesian states. Desired values are commands; '
              'separate getter calls are not a synchronized snapshot. Ctrl-C to stop.', flush=True)
        while rclpy.ok():
            report(astribot, args.frame)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
