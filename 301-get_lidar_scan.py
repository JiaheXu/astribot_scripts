#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2025, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------

"""Monitor both Astribot Livox LiDAR streams without saving scans.

The robot publishes scans on ``/livox/lidar_front`` and
``/livox/lidar_back`` as ``sensor_msgs/msg/PointCloud2``. This example
prints per-stream scan counts, point counts, and scan rates.

Run after sourcing this SDK's ROS 2 environment::

    python3 custom/301-get_lidar_scan.py
    python3 custom/301-get_lidar_scan.py --duration 60

Press ``q`` or Ctrl-C to stop. By default the existing LiDAR driver is left
untouched. Use ``--activate`` to reset and activate it, then deactivate it on exit.
"""

import argparse
import select
import signal
import sys
import termios
import threading
import time
import tty
from unittest.mock import patch

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2

# SDK Import
from astribot_sdk.core.astribot_api.astribot_client import Astribot


LIDAR_TOPICS = {
    "front": "/livox/lidar_front",
    "back": "/livox/lidar_back",
}
REPORT_INTERVAL_SEC = 1.0


class LidarMonitor:
    def __init__(self, activate=False, duration=0.0):
        # Answer the SDK control-rights prompt as Enter, only during startup.
        with patch("builtins.input", return_value=""):
            self.astribot = Astribot()
        self.node = self.astribot.astribot_interface.node
        self.activate = activate
        self.duration = duration
        self.started = time.monotonic()
        self.stop_requested = threading.Event()
        self.frame_count = {name: 0 for name in LIDAR_TOPICS}
        self.point_count = {name: 0 for name in LIDAR_TOPICS}
        self.last_report_frames = {name: 0 for name in LIDAR_TOPICS}
        self.last_report = time.monotonic()
        self.lock = threading.Lock()

        # LiDAR data is high bandwidth and the driver publishes Best Effort.
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.subscriptions = [
            self.node.create_subscription(
                PointCloud2,
                topic,
                lambda msg, name=name: self.lidar_callback(name, msg),
                qos_profile,
            )
            for name, topic in LIDAR_TOPICS.items()
        ]
        self.report_timer = self.node.create_timer(REPORT_INTERVAL_SEC, self.report)
        self.duration_timer = (
            self.node.create_timer(0.1, self.check_duration) if duration else None
        )

    def start_hardware(self):
        """Reset and activate the LiDAR driver through the SDK."""
        if not self.activate:
            print("Leaving the existing LiDAR driver state unchanged.", flush=True)
            return
        print("Ensuring LiDAR is deactivated first...", flush=True)
        self.astribot.deactivate_lidar()
        time.sleep(1.0)
        print("Activating LiDAR...", flush=True)
        result = self.astribot.activate_lidar()
        if result:
            print("LiDAR activated successfully.", flush=True)
        else:
            print("[WARN] Failed to activate LiDAR; still listening for data.", flush=True)

    def lidar_callback(self, name, msg):
        """Update human-readable stats for each incoming PointCloud2."""
        with self.lock:
            self.frame_count[name] += 1
            self.point_count[name] += msg.width * msg.height

    def report(self):
        now = time.monotonic()
        with self.lock:
            frames = dict(self.frame_count)
            points = dict(self.point_count)
        elapsed = max(now - self.last_report, 1e-9)
        print(
            "[LiDAR] "
            + ", ".join(
                f"{name}: {frames[name]} scans, {points[name]} points "
                f"({(frames[name] - self.last_report_frames[name]) / elapsed:.1f} scans/s)"
                for name in LIDAR_TOPICS
            ),
            flush=True,
        )
        self.last_report_frames = frames
        self.last_report = now

    def check_duration(self):
        if self.duration and time.monotonic() - self.started >= self.duration:
            print(f"Duration of {self.duration:g}s reached; stopping.", flush=True)
            self.stop_requested.set()

    def close(self):
        """Stop subscriptions and deactivate LiDAR if activated by this example."""
        if self.report_timer is not None:
            self.node.destroy_timer(self.report_timer)
            self.report_timer = None
        if self.duration_timer is not None:
            self.node.destroy_timer(self.duration_timer)
            self.duration_timer = None
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.subscriptions.clear()
        if self.activate:
            try:
                print("Deactivating LiDAR hardware...", flush=True)
                self.astribot.deactivate_lidar()
            except Exception as exc:
                print(f"[WARN] Could not deactivate LiDAR: {exc}", flush=True)


def nonnegative(value):
    number = float(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration",
        type=nonnegative,
        default=0.0,
        help="stop after this many seconds; 0 runs until q or Ctrl-C",
    )
    activation = parser.add_mutually_exclusive_group()
    activation.add_argument(
        "--activate", action="store_true",
        help="reset/activate the LiDAR driver and deactivate it on exit",
    )
    activation.add_argument(
        "--no-activate", dest="activate", action="store_false",
        help="leave the running LiDAR driver unchanged (default)",
    )
    parser.set_defaults(activate=False)
    args = parser.parse_args()

    monitor = None
    try:
        monitor = LidarMonitor(
            activate=args.activate,
            duration=args.duration,
        )
        signal.signal(signal.SIGINT, lambda _signum, _frame: monitor.stop_requested.set())
        signal.signal(signal.SIGTERM, lambda _signum, _frame: monitor.stop_requested.set())
        monitor.start_hardware()
        print("Monitoring front and back LiDAR scans. Press q or Ctrl-C to stop.", flush=True)
        # AstribotInterface owns the node's ROS executor thread. Keep this
        # thread alive until Ctrl-C or --duration instead of adding the same
        # node to a second executor with rclpy.spin().
        terminal_fd = None
        terminal_settings = None
        try:
            if sys.stdin.isatty():
                terminal_fd = sys.stdin.fileno()
                terminal_settings = termios.tcgetattr(terminal_fd)
                tty.setcbreak(terminal_fd)
            while rclpy.ok() and not monitor.stop_requested.is_set():
                if terminal_fd is None:
                    monitor.stop_requested.wait(0.2)
                    continue
                readable, _, _ = select.select([terminal_fd], [], [], 0.2)
                if readable and sys.stdin.read(1).lower() == "q":
                    monitor.stop_requested.set()
        finally:
            if terminal_settings is not None:
                termios.tcsetattr(terminal_fd, termios.TCSADRAIN, terminal_settings)
    except KeyboardInterrupt:
        pass
    finally:
        if monitor is not None:
            monitor.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
