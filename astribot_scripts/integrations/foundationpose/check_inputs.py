#!/usr/bin/env python3
"""Verify live camera data inside the GPU container; never publish fake frames."""
import argparse
from functools import partial
import json
import math
from pathlib import Path
import time

import rclpy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

CAMERAS = ('head_rgbd', 'torso_rgbd')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--warmup', type=float, default=5)
    parser.add_argument('--min-hz', type=float, default=1)
    parser.add_argument('--max-silence', type=float, default=3)
    parser.add_argument('--require-registered', action='store_true')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if any(not math.isfinite(v) or v <= 0 for v in
           (args.duration, args.warmup, args.min_hz, args.max_silence)):
        parser.error('Timing and rate arguments must be positive and finite')
    rclpy.init()
    node = rclpy.create_node('foundationpose_input_check')
    samples, status = {}, {}
    measure_start = time.monotonic() + args.warmup
    baseline = {}

    def receive(topic, message):
        now = time.monotonic()
        if now < measure_start:
            return
        item = samples[topic]
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp > item.get('last_stamp_ns', -1):
            item['advancing_frames'] += 1
        item.update(last_stamp_ns=stamp, last_received=now, width=message.width,
                    height=message.height, frame_id=message.header.frame_id)
        item['frames'] += 1
        if isinstance(message, Image):
            item['encoding'] = message.encoding
            item['valid_bytes'] = len(message.data) == message.height * message.step

    def receive_status(message):
        try:
            payload = json.loads(message.data)
            if not baseline and time.monotonic() >= measure_start:
                baseline.update(payload.get('counts', {}))
            status.update(payload)
            status['last_received'] = time.monotonic()
        except (ValueError, TypeError):
            pass

    node.create_subscription(String, '/foundationpose/input_status', receive_status, 2)
    for camera in CAMERAS:
        prefix = '/foundationpose/' + camera
        topics = [(f'/sdk/astribot_camera/{camera}/{kind}_image', Image)
                  for kind in ('color', 'depth')]
        topics += [(prefix + '/rgb/image_rect_color', Image),
                   (prefix + '/rgb/camera_info', CameraInfo),
                   (prefix + '/depth_registered/image_rect', Image)]
        for topic, cls in topics:
            samples[topic] = dict(frames=0, advancing_frames=0)
            node.create_subscription(cls, topic, partial(receive, topic), 2)
    print(f'Checking live inputs: {args.warmup:g}s discovery + {args.duration:g}s measurement', flush=True)
    try:
        while time.monotonic() < measure_start + args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        issues, registration_issues = [], []
        for topic, item in samples.items():
            item['hz'] = round(item['advancing_frames'] / (now - measure_start), 2)
            last = item.pop('last_received', None)
            item['silence_seconds'] = round(now - last, 2) if last is not None else None
            item['publishers'] = sorted({e.node_namespace.rstrip('/') + '/' + e.node_name
                                        for e in node.get_publishers_info_by_topic(topic)})
            item['subscribers'] = sorted({e.node_namespace.rstrip('/') + '/' + e.node_name
                                         for e in node.get_subscriptions_info_by_topic(topic)})
            item['healthy'] = (item['hz'] >= args.min_hz and last is not None
                               and now - last <= args.max_silence and item.get('valid_bytes', True))
            if not item['healthy']:
                target = registration_issues if '/depth_registered/' in topic else issues
                age_text = f'{item["silence_seconds"]}s ago' if last is not None else 'never'
                target.append(f'{topic}: {item["frames"]} frames, {item["hz"]} Hz, '
                              f'last received {age_text}')
            print(f'{item["frames"]:4d} frames  {item["hz"]:5.2f} Hz  {topic}', flush=True)
        if now - status.get('last_received', 0) > args.max_silence:
            issues.append('Input adapter telemetry is absent or stale')
        for camera in CAMERAS:
            for kind in ('color', 'depth'):
                current = status.get('counts', {}).get(camera, {}).get(kind, 0)
                before = baseline.get(camera, {}).get(kind, current)
                age = status.get('seconds_since_input', {}).get(camera + '/' + kind, float('inf'))
                if current <= before or age > args.max_silence:
                    issues.append(f'Input adapter is not receiving fresh {camera}/{kind}')
            fp_node = '/foundationpose/' + camera + '/foundationpose'
            for suffix in ('rgb/image_rect_color', 'rgb/camera_info', 'depth_registered/image_rect', 'segmentation'):
                topic = '/foundationpose/' + camera + '/' + suffix
                subscriptions = node.get_subscriptions_info_by_topic(topic)
                if not any(e.node_namespace.rstrip('/') + '/' + e.node_name == fp_node for e in subscriptions):
                    issues.append(f'{fp_node} has no subscription to {topic}')
        status.pop('last_received', None)
        result = {'checked_at_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                  'raw_inputs_ok': not issues, 'registered_inputs_ok': not issues and not registration_issues,
                  'issues': issues, 'registration_issues': registration_issues,
                  'adapter_status': status, 'topics': samples}
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(result, indent=2) + '\n')
        print('RAW INPUTS: ' + ('PASS' if not issues else 'FAIL'))
        print('REGISTERED DEPTH: ' + ('PASS' if not registration_issues else 'BLOCKED'))
        for issue in issues:
            print('ERROR: ' + issue)
        for camera, error in status.get('registration_errors', {}).items():
            print(f'CALIBRATION: {camera}: {error}')
        if registration_issues:
            print('Native depth is not registered metric depth. Supply measured depth calibration before inference.')
        return 2 if issues else 3 if args.require_registered and registration_issues else 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
