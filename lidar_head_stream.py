#!/usr/bin/env python3
"""Stream both Livox scans and head/wrist RGB using examples 300/301's SDK patterns.

    python3 lidar_head_stream.py
    python3 lidar_head_stream.py --camera-delay 10 --duration 60 --report /tmp/head_lidar.json
    python3 lidar_head_stream.py --no-activate  # attach to drivers already running

Outputs: /sdk/astribot_lidar/{front,back}_scan (PointCloud2) and
/sdk/astribot_camera/{head_rgbd,left_wrist_rgbd,right_wrist_rgbd}/color_image
(Image, bgr8, 320x240).
The original headers and point-cloud payloads are preserved. Resizing takes
place after reception and does not reduce robot-link bandwidth. Ctrl-C exits
the subscriber without deactivating shared robot drivers. Examples are unchanged.
"""

import argparse
import json
import logging
from pathlib import Path
import signal
import threading
import time

import cv2
import numpy as np
from rclpy.logging import LoggingSeverity
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2

from astribot_sdk.core.astribot_api.astribot_client import Astribot
from astribot_sdk.core.astribot_api.astribot_interface import AstribotInterface


CAMERAS = ('head_rgbd', 'left_wrist_rgbd', 'right_wrist_rgbd')
CAMERA_INPUTS = {
    f'/astribot_camera/{camera}/color_compress/compressed': camera for camera in CAMERAS
}


class LidarHeadStreamer:
    def __init__(self):
        # Construct a sensor-only SDK client without taking robot control rights.
        acquire = AstribotInterface.acquire_control_rights
        AstribotInterface.acquire_control_rights = lambda client, high: setattr(
            client, 'have_control_rights', False)
        try:
            logger = logging.getLogger('lidar_head_sdk')
            logger.setLevel(logging.WARNING)
            self.robot = Astribot(logger=logger)
        finally:
            AstribotInterface.acquire_control_rights = acquire
        self.node = self.robot.astribot_interface.node
        self.node.get_logger().set_level(LoggingSeverity.WARN)
        self.qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.stop_event = threading.Event()
        self.conditions = {camera: threading.Condition() for camera in CAMERAS}
        self.pending_images = dict.fromkeys(CAMERAS)
        self.lock = threading.Lock()
        self.started = time.monotonic()
        self.camera_started = None
        self.counts = dict.fromkeys(('lidar/front', 'lidar/back', *CAMERAS), 0)
        self.lidar_received = {'front': 0, 'back': 0}
        self.lidar_publish_errors = {'front': 0, 'back': 0}
        self.last_received = {}
        self.last_log = {}
        self.max_gaps = dict.fromkeys(self.counts, 0.0)
        self.camera_received = dict.fromkeys(CAMERAS, 0)
        self.camera_replaced = dict.fromkeys(CAMERAS, 0)
        self.decode_errors = dict.fromkeys(CAMERAS, 0)
        self.baseline_counts = None
        self.subscriptions = []
        self.lidar_publishers = {
            side: self.node.create_publisher(PointCloud2, f'/sdk/astribot_lidar/{side}_scan', self.qos)
            for side in ('front', 'back')
        }
        self.image_publishers = {
            camera: self.node.create_publisher(
                Image, f'/sdk/astribot_camera/{camera}/color_image', self.qos)
            for camera in CAMERAS
        }
        self.workers = {
            camera: threading.Thread(target=self.image_worker, args=(camera,),
                                     name=f'{camera}-decoder', daemon=True)
            for camera in CAMERAS
        }

    def mark_published(self, name, header):
        now = time.monotonic()
        with self.lock:
            previous = self.last_received.get(name)
            if previous is not None:
                self.max_gaps[name] = max(self.max_gaps[name], now - previous)
            self.last_received[name] = now
            self.counts[name] += 1
            log = now - self.last_log.get(name, 0) >= 1
            if log:
                self.last_log[name] = now
        if log:
            print(f'{name} published at {header.stamp.sec}.{header.stamp.nanosec:09d}', flush=True)

    def lidar_callback(self, side, msg):
        if self.stop_event.is_set():
            return
        with self.lock:
            self.lidar_received[side] += 1
        try:
            self.lidar_publishers[side].publish(msg)
        except Exception as exc:
            with self.lock:
                self.lidar_publish_errors[side] += 1
                first_error = self.lidar_publish_errors[side] == 1
            if first_error:
                print(f'[ERROR] {side} LiDAR publication failed: {exc}', flush=True)
            return
        self.mark_published(f'lidar/{side}', msg.header)

    def image_callback(self, topic, msg, width, height, array):
        # need_decode=False: the SDK passes compressed data; no decoding here.
        # All SDK registrations share one callback, so dispatch by source topic.
        camera = CAMERA_INPUTS.get(topic)
        if camera is None:
            return
        with self.conditions[camera]:
            if self.stop_event.is_set():
                return
            self.camera_received[camera] += 1
            if self.pending_images[camera] is not None:
                self.camera_replaced[camera] += 1
            self.pending_images[camera] = msg
            self.conditions[camera].notify()

    @staticmethod
    def decode_image(msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError('Invalid compressed camera image')
        frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
        image = Image()
        image.header = msg.header
        image.height, image.width = frame.shape[:2]
        image.encoding = 'bgr8'
        image.is_bigendian = 0
        image.step = image.width * 3
        image.data = frame.tobytes()
        return image

    def image_worker(self, camera):
        condition = self.conditions[camera]
        while not self.stop_event.is_set():
            with condition:
                condition.wait_for(lambda: self.pending_images[camera] is not None or self.stop_event.is_set())
                if self.stop_event.is_set():
                    return
                msg, self.pending_images[camera] = self.pending_images[camera], None
            try:
                image = self.decode_image(msg)
                self.image_publishers[camera].publish(image)
                self.mark_published(camera, image.header)
            except Exception as exc:
                with condition:
                    self.decode_errors[camera] += 1
                    first_error = self.decode_errors[camera] == 1
                if first_error:
                    print(f'[ERROR] {camera} image processing failed: {exc}', flush=True)

    def activate(self, label, method):
        for _ in range(3):
            if self.stop_event.is_set():
                return
            if method():
                print(f'{label} activated.', flush=True)
                return
            if self.stop_event.wait(2):
                return
        print(f'[WARN] {label} activation failed; listening for existing data.', flush=True)

    def start(self, activate=True, camera_delay=0):
        for worker in self.workers.values():
            worker.start()
        # Exactly the subscription order and QoS of example 301.
        for side in ('front', 'back'):
            self.subscriptions.append(self.node.create_subscription(
                PointCloud2, f'/livox/lidar_{side}',
                lambda msg, side=side: self.lidar_callback(side, msg), self.qos))
        if activate:
            if not self.robot.deactivate_lidar():
                print('[WARN] LiDAR reset request failed.', flush=True)
            if self.stop_event.wait(1):
                return
            self.activate('LiDAR', self.robot.activate_lidar)
        if camera_delay:
            print(f'LiDAR-only baseline for {camera_delay:g} seconds.', flush=True)
            if self.stop_event.wait(camera_delay):
                return
        if activate:
            self.activate('Camera', self.robot.activate_camera)
        if self.stop_event.is_set():
            return
        with self.lock:
            self.baseline_counts = dict(self.counts)
            self.camera_started = time.monotonic()
        for camera in CAMERAS:
            subscription = self.robot.register_image_callback(
                camera, 'color', self.image_callback, need_decode=False)
            if subscription is None:
                raise RuntimeError(f'{camera} SDK subscription failed')
            self.subscriptions.append(subscription)
            print(f'RGB topic: /sdk/astribot_camera/{camera}/color_image (320x240 bgr8)', flush=True)
        print('Head and both wrists subscribed alongside both LiDARs.', flush=True)
        # AstribotInterface already spins this node. Do not call rclpy.spin again.

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            result = {
                'elapsed_seconds': now - self.started,
                'counts': dict(self.counts),
                'lidar_received': dict(self.lidar_received),
                'lidar_publish_errors': dict(self.lidar_publish_errors),
                'max_arrival_gaps_seconds': dict(self.max_gaps),
                'seconds_since_last_message': {key: now - self.last_received[key]
                    if key in self.last_received else None for key in self.counts},
            }
            if self.camera_started is not None:
                elapsed = now - self.camera_started
                counts = {key: self.counts[key] - self.baseline_counts[key] for key in self.counts}
                result.update(baseline_counts=self.baseline_counts, combined_seconds=elapsed,
                              combined_counts=counts,
                              combined_hz={key: count / max(elapsed, 1e-9) for key, count in counts.items()})
        result.update(camera_received={}, camera_frames_replaced={}, decode_errors={})
        for camera, condition in self.conditions.items():
            with condition:
                result['camera_received'][camera] = self.camera_received[camera]
                result['camera_frames_replaced'][camera] = self.camera_replaced[camera]
                result['decode_errors'][camera] = self.decode_errors[camera]
        return result

    def close(self):
        self.stop_event.set()
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.subscriptions.clear()
        for camera, condition in self.conditions.items():
            with condition:
                self.pending_images[camera] = None
                condition.notify_all()
        for worker in self.workers.values():
            if worker.is_alive():
                worker.join()
        # Leave the shared camera/LiDAR drivers running for other applications.


def nonnegative(value):
    value = float(value)
    if not np.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError('must be a finite, non-negative number')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--no-activate', action='store_true', help='Subscribe without any driver control calls.')
    parser.add_argument('--camera-delay', type=nonnegative, default=0, help='Seconds to observe LiDAR before subscribing to head/wrist RGB.')
    parser.add_argument('--duration', type=nonnegative, default=0, help='Combined-stream test duration; 0 runs until Ctrl-C.')
    parser.add_argument('--report', type=Path, help='Write input/publication statistics as JSON on exit.')
    args = parser.parse_args()
    streamer = LidarHeadStreamer()
    signal.signal(signal.SIGINT, lambda *_: streamer.stop_event.set())
    signal.signal(signal.SIGTERM, lambda *_: streamer.stop_event.set())
    try:
        streamer.start(activate=not args.no_activate, camera_delay=args.camera_delay)
        started = time.monotonic()
        last = started
        previous = dict(streamer.snapshot()['counts'])
        while not streamer.stop_event.wait(0.2):
            now = time.monotonic()
            if now - last >= 5:
                counts = streamer.snapshot()['counts']
                rates = {key: round((counts[key] - previous[key]) / (now - last), 2) for key in counts}
                print(f'[5s rates Hz] {rates}', flush=True)
                previous, last = counts, now
            if args.duration and now - started >= args.duration:
                break
    finally:
        streamer.close()
        result = streamer.snapshot()
        print(json.dumps(result, indent=2), flush=True)
        if args.report:
            args.report.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
