#!/usr/bin/env python3
"""Republish both Livox PointCloud2 topics in a dedicated ROS 2 process.

Terminal 1: python3 lidar_node.py --activate --activate-camera
Terminal 2: python3 camera_raw_node.py

Uses example 301's BEST_EFFORT/depth-10 subscriptions and preserves entire scans.
Activation uses the same SetBool services as the SDK, without creating its
control clients or native threads. With no flags, only subscribes/publishes.
Stopping this process leaves shared sensor drivers running.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool


class LidarRepublisher(Node):
    def __init__(self):
        super().__init__('astribot_lidar_republisher')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.started = time.monotonic()
        self.stats = {side: {'received': 0, 'published': 0, 'publish_errors': 0,
                            'last_received': None, 'max_gap_sec': 0.0,
                            'last_log': 0.0, 'last_points': 0}
                      for side in ('front', 'back')}
        self.outputs = {side: self.create_publisher(
            PointCloud2, f'/sdk/astribot_lidar/{side}_scan', qos) for side in self.stats}
        self.inputs = [self.create_subscription(
            PointCloud2, f'/livox/lidar_{side}',
            lambda msg, side=side: self.receive(side, msg), qos) for side in self.stats]
        for side in self.stats:
            print(f'/livox/lidar_{side} -> /sdk/astribot_lidar/{side}_scan', flush=True)

    def receive(self, side, msg):
        now = time.monotonic()
        stats = self.stats[side]
        stats['received'] += 1
        if stats['last_received'] is not None:
            stats['max_gap_sec'] = max(stats['max_gap_sec'], now - stats['last_received'])
        stats['last_received'] = now
        stats['last_points'] = msg.width * msg.height
        try:
            self.outputs[side].publish(msg)
        except Exception as exc:
            stats['publish_errors'] += 1
            if stats['publish_errors'] == 1:
                print(f'[ERROR] {side} publish failed: {exc}', flush=True)
            return
        stats['published'] += 1
        if now - stats['last_log'] >= 1:
            stats['last_log'] = now
            print(f'lidar/{side} published at {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}', flush=True)

    def set_driver(self, service, enabled):
        client = self.create_client(SetBool, service)
        try:
            if not client.wait_for_service(timeout_sec=5):
                print(f'[WARN] {service} unavailable.', flush=True)
                return False
            future = client.call_async(SetBool.Request(data=enabled))
            rclpy.spin_until_future_complete(self, future, timeout_sec=10)
            if not future.done():
                future.cancel()
                print(f'[WARN] {service} response timed out.', flush=True)
                return False
            response = future.result()
            print(f'{service}: success={response.success}, {response.message}', flush=True)
            return response.success
        finally:
            self.destroy_client(client)

    def snapshot(self):
        now = time.monotonic()
        streams = {}
        for side, stats in self.stats.items():
            streams[side] = {key: value for key, value in stats.items()
                             if key not in ('last_received', 'last_log')}
            streams[side]['seconds_since_last_message'] = (
                now - stats['last_received'] if stats['last_received'] is not None else None)
        return {'elapsed_seconds': now - self.started, 'streams': streams}


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError('must be finite and non-negative')
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--activate', action='store_true', help='Request LiDAR activation through the SDK driver service.')
    parser.add_argument('--reset-lidar', action='store_true', help='Deactivate LiDAR, wait 5 seconds, then activate it.')
    parser.add_argument('--activate-camera', action='store_true', help='Request camera driver activation; no images are subscribed here.')
    parser.add_argument('--duration', type=nonnegative, default=0, help='Seconds to monitor after startup; 0 runs until Ctrl-C.')
    parser.add_argument('--report', type=Path, help='Write JSON statistics on exit.')
    args = parser.parse_args()
    stop = threading.Event()
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    node = LidarRepublisher()
    try:
        if args.reset_lidar:
            node.set_driver('/astribot_lidar_control', False)
            stop.wait(5)
        if not stop.is_set() and (args.activate or args.reset_lidar):
            node.set_driver('/astribot_lidar_control', True)
        if not stop.is_set() and args.activate_camera:
            node.set_driver('/astribot_camera_control', True)
        start = last = time.monotonic()
        previous = {side: dict(stats) for side, stats in node.stats.items()}
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last >= 5:
                rates = {side: {key: round((stats[key] - previous[side][key]) / (now - last), 2)
                                for key in ('received', 'published')} for side, stats in node.stats.items()}
                print(f'[5s rates Hz] {rates}', flush=True)
                previous = {side: dict(stats) for side, stats in node.stats.items()}
                last = now
            if args.duration and now - start >= args.duration:
                break
    finally:
        result = node.snapshot()
        node.destroy_node()
        rclpy.try_shutdown()
        print(json.dumps(result, indent=2), flush=True)
        if args.report:
            args.report.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
