#!/usr/bin/env python3
"""Convert existing compressed RGB topics to six 320x240 bgr8 image topics.

    python3 camera_raw_node.py
    python3 camera_raw_node.py --duration 60 --report /tmp/camera_raw.json

Inputs:  /astribot_camera/{camera}/color_compress/compressed
Outputs: /sdk/astribot_camera/{camera}/color_image
Stereo: /sdk/astribot_camera/head_stereo/{left,right}_image
Five JPEG sources produce six raw outputs; the stereo JPEG is split before resizing.
This node does not activate drivers or import the Astribot SDK. Resizing occurs
after reception; it does not reduce incoming camera bandwidth.
"""

import argparse
import json
import math
from pathlib import Path
import signal
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CompressedImage, Image


CAMERAS = ('head_rgbd', 'torso_rgbd', 'left_wrist_rgbd', 'right_wrist_rgbd', 'head_stereo')


def output_topics(camera):
    if camera == 'head_stereo':
        return {side: f'/sdk/astribot_camera/{camera}/{side}_image' for side in ('left', 'right')}
    return {'color': f'/sdk/astribot_camera/{camera}/color_image'}


class CameraRawNode(Node):
    def __init__(self, cameras=CAMERAS):
        super().__init__('astribot_camera_raw')
        self.cameras = tuple(dict.fromkeys(cameras))
        self.stopping = threading.Event()
        self.started = time.monotonic()
        self.conditions = {camera: threading.Condition() for camera in self.cameras}
        self.pending = dict.fromkeys(self.cameras)
        self.stats = {camera: {'received': 0, 'published': 0, 'replaced': 0,
                              'errors': 0, 'max_publish_gap_seconds': 0.0,
                              'output_counts': dict.fromkeys(output_topics(camera), 0)}
                      for camera in self.cameras}
        self.last_published = {}
        self.last_log = {}
        self.publishers_by_camera = {}
        self.input_subscriptions = []
        self.workers = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for camera in self.cameras:
            source = f'/astribot_camera/{camera}/color_compress/compressed'
            targets = output_topics(camera)
            self.publishers_by_camera[camera] = {
                view: self.create_publisher(Image, target, qos) for view, target in targets.items()}
            self.input_subscriptions.append(self.create_subscription(
                CompressedImage, source, lambda msg, camera=camera: self.receive(camera, msg), qos))
            worker = threading.Thread(target=self.decode_worker, args=(camera,),
                                      name=f'{camera}-decoder', daemon=True)
            self.workers.append(worker)
            worker.start()
            for target in targets.values():
                print(f'{source} -> {target} (320x240 bgr8)', flush=True)

    def receive(self, camera, msg):
        # Retain only the newest JPEG for this camera; never decode in ROS callbacks.
        with self.conditions[camera]:
            if self.stopping.is_set():
                return
            self.stats[camera]['received'] += 1
            if self.pending[camera] is not None:
                self.stats[camera]['replaced'] += 1
            self.pending[camera] = msg
            self.conditions[camera].notify()

    @staticmethod
    def decode_image(msg):
        return CameraRawNode.decode_images('head_rgbd', msg)['color']

    @staticmethod
    def decode_images(camera, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError('Invalid compressed image')
        if camera == 'head_stereo':
            if frame.shape[1] < 2 or frame.shape[1] % 2:
                raise ValueError('Stereo JPEG width must be even for left/right splitting')
            midpoint = frame.shape[1] // 2
            views = {'left': frame[:, :midpoint], 'right': frame[:, midpoint:]}
        else:
            views = {'color': frame}
        images = {}
        for view, pixels in views.items():
            pixels = cv2.resize(pixels, (320, 240), interpolation=cv2.INTER_AREA)
            image = Image()
            image.header = msg.header
            image.height, image.width = pixels.shape[:2]
            image.encoding = 'bgr8'
            image.is_bigendian = 0
            image.step = image.width * 3
            image.data = pixels.tobytes()
            images[view] = image
        return images

    def decode_worker(self, camera):
        condition = self.conditions[camera]
        while not self.stopping.is_set():
            with condition:
                condition.wait_for(lambda: self.pending[camera] is not None or self.stopping.is_set())
                if self.stopping.is_set():
                    return
                msg, self.pending[camera] = self.pending[camera], None
            try:
                images = self.decode_images(camera, msg)
                if self.stopping.is_set():
                    return
                for view, image in images.items():
                    self.publishers_by_camera[camera][view].publish(image)
                    with condition:
                        self.stats[camera]['output_counts'][view] += 1
                now = time.monotonic()
                with condition:
                    stats = self.stats[camera]
                    stats['published'] += 1
                    previous = self.last_published.get(camera)
                    if previous is not None:
                        stats['max_publish_gap_seconds'] = max(stats['max_publish_gap_seconds'], now - previous)
                    self.last_published[camera] = now
                    log = now - self.last_log.get(camera, 0) >= 1
                    if log:
                        self.last_log[camera] = now
                if log:
                    for view, image in images.items():
                        print(f'{camera}/{view} published at {image.header.stamp.sec}.{image.header.stamp.nanosec:09d}', flush=True)
            except Exception as exc:
                with condition:
                    self.stats[camera]['errors'] += 1
                    first_error = self.stats[camera]['errors'] == 1
                if first_error:
                    print(f'[ERROR] {camera} decode/publication failed: {exc}', flush=True)

    def snapshot(self):
        now = time.monotonic()
        result = {'elapsed_seconds': now - self.started, 'cameras': {}}
        for camera, condition in self.conditions.items():
            with condition:
                stats = dict(self.stats[camera])
                stats['output_counts'] = dict(stats['output_counts'])
                stats['seconds_since_last_publish'] = (now - self.last_published[camera]
                                                       if camera in self.last_published else None)
                result['cameras'][camera] = stats
        return result

    def close(self):
        self.stopping.set()
        for subscription in self.input_subscriptions:
            self.destroy_subscription(subscription)
        self.input_subscriptions.clear()
        for camera, condition in self.conditions.items():
            with condition:
                self.pending[camera] = None
                condition.notify_all()
        for worker in self.workers:
            worker.join()


def nonnegative(value):
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError('must be a finite, non-negative number')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--cameras', nargs='+', choices=CAMERAS, default=list(CAMERAS))
    parser.add_argument('--duration', type=nonnegative, default=0, help='Seconds to run; 0 runs until Ctrl-C.')
    parser.add_argument('--report', type=Path, help='Write receive/publication statistics as JSON on exit.')
    args = parser.parse_args(remove_ros_args()[1:])
    stop = threading.Event()
    previous_handlers = {sig: signal.signal(sig, lambda *_: stop.set())
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = CameraRawNode(args.cameras)
        started = last = time.monotonic()
        previous = node.snapshot()['cameras']
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last >= 5:
                current = node.snapshot()['cameras']
                rates = {camera: {'received_hz': round((stats['received'] - previous[camera]['received']) / (now - last), 2),
                                  'published_hz': round((stats['published'] - previous[camera]['published']) / (now - last), 2),
                                  'errors': stats['errors']} for camera, stats in current.items()}
                print(f'[5s camera rates] {rates}', flush=True)
                previous, last = current, now
            if args.duration and now - started >= args.duration:
                break
    finally:
        if node is not None:
            node.close()
            result = node.snapshot()
            node.destroy_node()
            print(json.dumps(result, indent=2), flush=True)
        rclpy.try_shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        if node is not None and args.report:
            args.report.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
