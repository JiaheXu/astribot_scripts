#!/usr/bin/env python3
"""Verify live robot/map/RGB data and TF before reporting desktop restart success."""
import argparse
import json
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState, PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException

CAMERAS = ('head_rgbd', 'torso_rgbd', 'left_wrist_rgbd', 'right_wrist_rgbd')
FRAMES = ('astribot_viz_chassis', *(f'astribot_viz_{c}_optical' for c in CAMERAS))
EXPECTED = ('joints', 'odometry', 'map_cloud', *CAMERAS)


def missing_streams(received, counts, now, max_age=3.0):
    return [name for name in EXPECTED
            if counts.get(name, 0) < 2 or now - received.get(name, float('-inf')) > max_age]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=100.0)
    args = parser.parse_args()
    rclpy.init(args=['--ros-args', '-r', '/tf:=/astribot_viz/tf',
                    '-r', '/tf_static:=/astribot_viz/tf_static'])
    node = rclpy.create_node('astribot_restart_readiness')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    received, counts = {}, {}

    def receive(name, msg):
        if name in ('odometry', 'map_cloud') and msg.header.frame_id != 'map':
            return
        if name == 'odometry' and msg.child_frame_id != 'astribot_viz_chassis':
            return
        if name == 'joints' and (len(msg.name) != 20 or len(msg.position) != 20):
            return
        if name in CAMERAS and (msg.encoding != 'rgb8' or (msg.width, msg.height) != (640, 480)
                               or msg.step != 640 * 3 or len(msg.data) != 640 * 480 * 3):
            return
        if name == 'map_cloud' and not msg.width * msg.height:
            return
        received[name] = time.monotonic()
        counts[name] = counts.get(name, 0) + 1

    topics = [('joints', JointState, '/astribot_viz/joint_states'),
              ('odometry', Odometry, '/fast_lio/chassis/odometry'),
              ('map_cloud', PointCloud2, '/fast_lio/rear/cloud_registered')]
    topics += [(camera, Image, f'/sdk/astribot_camera/{camera}/color_image') for camera in CAMERAS]
    subscriptions = [node.create_subscription(kind, topic, lambda m, k=name: receive(k, m),
                                              qos_profile_sensor_data) for name, kind, topic in topics]
    deadline = time.monotonic() + args.timeout
    stable_since, last_report = None, 0.0
    missing, missing_tf = list(EXPECTED), list(FRAMES)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            missing = missing_streams(received, counts, now)
            missing_tf = []
            for frame in FRAMES:
                try:
                    transform = buffer.lookup_transform('map', frame, Time())
                    age = (node.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
                    if age > 3.0 or age < -1.0:
                        missing_tf.append(frame)
                except TransformException:
                    missing_tf.append(frame)
            if not missing and not missing_tf:
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= 3.0:
                    print('Live stack ready: ' + json.dumps({'messages': counts, 'world_frame': 'map',
                                                             'frames': list(FRAMES)}), flush=True)
                    return 0
            else:
                stable_since = None
            if now - last_report >= 5.0:
                print(f'Waiting for live data: {missing}; missing/stale map TF: {missing_tf}', flush=True)
                last_report = now
        raise RuntimeError(f'RViz data not ready. Missing/stale streams: {missing}; '
                           f'missing/stale map TF: {missing_tf}. Check the Sensors pane and robot connection.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
