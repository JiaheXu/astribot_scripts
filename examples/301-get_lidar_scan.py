#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2025, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------

"""Record both Astribot Livox LiDAR streams into a ROS 2 bag.

The robot publishes the complete scans on ``/livox/lidar_front`` and
``/livox/lidar_back`` as ``sensor_msgs/msg/PointCloud2``. This example keeps
the original message (including all Livox fields and the ROS header timestamp)
and writes it to a rosbag2 directory. The bag can be replayed directly with
``ros2 bag play`` or consumed by a lidar-odometry pipeline.

Run after sourcing this SDK's ROS 2 environment::

    python3 examples/301-get_lidar_scan.py
    python3 examples/301-get_lidar_scan.py --output /data/lidar_run
    python3 examples/301-get_lidar_scan.py --duration 60

Press ``q`` or Ctrl-C to finish the bag cleanly. By default the LiDAR driver is
reset and activated, matching the original example. Use ``--no-activate`` when
the driver is already running and must be left untouched.
"""

import argparse
from datetime import datetime
from pathlib import Path
import queue
import select
import signal
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2

try:
    import rosbag2_py
except ImportError as exc:  # pragma: no cover - depends on the ROS 2 install
    raise RuntimeError(
        "rosbag2_py is required to record LiDAR data. Source the ROS 2/SDK "
        "environment before running this example."
    ) from exc

# SDK Import
from astribot_sdk.core.astribot_api.astribot_client import Astribot


LIDAR_TOPICS = {
    "front": "/livox/lidar_front",
    "back": "/livox/lidar_back",
}
DEFAULT_BAG_ROOT = Path(__file__).resolve().parents[1] / "recordings"
REPORT_INTERVAL_SEC = 1.0


def stamp_ns(header):
    """Return a ROS message timestamp in nanoseconds."""
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def default_bag_path():
    """Create a readable default bag URI."""
    base = DEFAULT_BAG_ROOT / f"lidar_{datetime.now().astimezone():%Y-%m-%d_%H-%M-%S}"
    candidate = base
    suffix = 1
    while candidate.exists():
        candidate = base.with_name(f"{base.name}_{suffix:02d}")
        suffix += 1
    return candidate


class RosbagRecorder:
    """Serialize PointCloud2 callbacks and write them on a dedicated thread."""

    def __init__(self, bag_path):
        self.bag_path = Path(bag_path).expanduser()
        if self.bag_path.exists():
            raise FileExistsError(
                f"Bag path already exists: {self.bag_path}. Choose a new --output path."
            )
        self.bag_path.parent.mkdir(parents=True, exist_ok=True)

        storage_options = rosbag2_py.StorageOptions(
            uri=str(self.bag_path), storage_id="sqlite3"
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(storage_options, converter_options)
        for topic in LIDAR_TOPICS.values():
            self.writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=topic,
                    type="sensor_msgs/msg/PointCloud2",
                    serialization_format="cdr",
                    offered_qos_profiles="",
                )
            )

        # The queue is intentionally unbounded: a recording must not silently
        # drop scans when the rosbag writer briefly falls behind.
        self.pending = queue.Queue()
        self.lock = threading.Lock()
        self.active = True
        self.received = {name: 0 for name in LIDAR_TOPICS}
        self.written = {name: 0 for name in LIDAR_TOPICS}
        self.errors = []
        self.worker = threading.Thread(
            target=self._write_loop, name="lidar-rosbag-writer", daemon=True
        )
        self.worker.start()

    def record(self, name, msg):
        """Copy and enqueue one message without blocking the ROS callback."""
        with self.lock:
            if not self.active:
                return
            # Serialize before returning from the callback. PointCloud2 owns a
            # mutable byte sequence, so retaining the ROS object itself is unsafe.
            # Keep this under the same lock used by stop() so an in-flight
            # callback cannot enqueue a scan after the stop sentinel.
            serialized = serialize_message(msg)
            self.received[name] += 1
            self.pending.put((name, stamp_ns(msg.header), serialized))

    def _write_loop(self):
        while True:
            item = self.pending.get()
            if item is None:
                self.pending.task_done()
                return
            name, timestamp_ns, serialized = item
            try:
                self.writer.write(LIDAR_TOPICS[name], serialized, timestamp_ns)
                with self.lock:
                    self.written[name] += 1
            except Exception as exc:  # Keep the other stream alive if one write fails.
                message = f"{name}: {exc}"
                with self.lock:
                    if message not in self.errors:
                        self.errors.append(message)
                print(f"[RECORD ERROR] {message}", flush=True)
            finally:
                self.pending.task_done()

    def stop(self):
        """Drain queued messages and close the rosbag metadata cleanly."""
        with self.lock:
            if not self.active:
                return
            self.active = False
        self.pending.put(None)
        self.worker.join()
        # SequentialWriter finalizes metadata.yaml when the writer is closed or
        # destroyed. Explicit close is available in newer ROS 2 releases;
        # deleting the writer is the portable Humble fallback.
        close = getattr(self.writer, "close", None)
        if close is not None:
            close()
        self.writer = None
        print(f"ROS 2 bag saved to {self.bag_path}", flush=True)
        print(f"Saved scans: {self.written}", flush=True)
        with self.lock:
            if self.errors:
                print(f"[WARN] Bag write errors: {self.errors}", flush=True)


class LidarMonitor:
    def __init__(self, bag_path, activate=True, duration=0.0):
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
        self.recorder = RosbagRecorder(bag_path)

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
        """Record every incoming PointCloud2 and update human-readable stats."""
        with self.lock:
            self.frame_count[name] += 1
            self.point_count[name] += msg.width * msg.height
        self.recorder.record(name, msg)

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
        """Stop subscriptions, finish the bag, and deactivate LiDAR."""
        if self.report_timer is not None:
            self.node.destroy_timer(self.report_timer)
            self.report_timer = None
        if self.duration_timer is not None:
            self.node.destroy_timer(self.duration_timer)
            self.duration_timer = None
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.subscriptions.clear()
        self.recorder.stop()
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
        "--output",
        type=Path,
        default=None,
        help="rosbag2 output directory (default: recordings/lidar_<timestamp>)",
    )
    parser.add_argument(
        "--duration",
        type=nonnegative,
        default=0.0,
        help="stop after this many seconds; 0 runs until q or Ctrl-C",
    )
    parser.add_argument(
        "--no-activate",
        action="store_true",
        help="do not reset, activate, or deactivate the existing LiDAR driver",
    )
    args = parser.parse_args()
    bag_path = args.output or default_bag_path()

    monitor = None
    try:
        monitor = LidarMonitor(
            bag_path=bag_path,
            activate=not args.no_activate,
            duration=args.duration,
        )
        signal.signal(signal.SIGINT, lambda _signum, _frame: monitor.stop_requested.set())
        signal.signal(signal.SIGTERM, lambda _signum, _frame: monitor.stop_requested.set())
        monitor.start_hardware()
        print("Recording front and back LiDAR scans. Press q or Ctrl-C to stop.", flush=True)
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
