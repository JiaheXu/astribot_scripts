#!/usr/bin/env python3
"""Show apriltag_utils AprilTag/ArUco poses as tag-center axes in RViz."""
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import MarkerArray

from astribot_scripts.common.apriltag_markers import TagAxes, load_sources, pose_matrix


class AprilTagRviz(Node):
    def __init__(self):
        super().__init__('astribot_apriltag_rviz')
        cameras = self.declare_parameter('cameras_config', '').value
        tags = self.declare_parameter('tags_config', '').value
        self.fixed_frame = self.declare_parameter('fixed_frame', 'map').value
        self.tf_mode = self.declare_parameter('tf_time_mode', 'latest').value
        self.tf_timeout = float(self.declare_parameter('max_tf_age', 1.0).value)
        axis_length = float(self.declare_parameter('axis_length', 0.10).value)
        lifetime = float(self.declare_parameter('marker_lifetime', 1.0).value)
        if self.tf_mode not in ('latest', 'measurement'):
            raise ValueError('tf_time_mode must be latest or measurement')
        if not np.isfinite(self.tf_timeout) or self.tf_timeout <= 0:
            raise ValueError('max_tf_age must be finite and positive')
        sources, family, lengths = load_sources(cameras, tags)
        self.axes = TagAxes(family, lengths, axis_length, lifetime)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(MarkerArray, '/astribot_viz/apriltag_axes', 10)
        self.subscriptions_ = []
        for camera, (topic, optical_frame) in sources.items():
            self.subscriptions_.append(self.create_subscription(
                Detection2DArray, topic,
                lambda msg, camera=camera, optical_frame=optical_frame: self.receive(camera, optical_frame, msg),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
        self.get_logger().info(
            f'Tag axes: family={family}, {len(sources)} cameras, frame={self.fixed_frame}, '
            f'TF mode={self.tf_mode}. Latest mode is a display overlay, not synchronized localization.')

    def receive(self, camera, optical_frame, message):
        now = self.get_clock().now()
        try:
            if message.header.frame_id != optical_frame:
                raise ValueError(f'Expected optical frame {optical_frame}, got {message.header.frame_id!r}')
            matrix = np.eye(4)
            if message.detections and self.fixed_frame != optical_frame:
                when = Time() if self.tf_mode == 'latest' else Time.from_msg(message.header.stamp)
                if self.tf_mode == 'measurement' and when.nanoseconds == 0:
                    raise ValueError('Measurement mode requires a nonzero image timestamp')
                tf = self.buffer.lookup_transform(self.fixed_frame, optical_frame, when)
                if self.tf_mode == 'latest':
                    age = (now.nanoseconds - Time.from_msg(tf.header.stamp).nanoseconds) / 1e9
                    if tf.header.stamp.sec or tf.header.stamp.nanosec:
                        if not -0.1 <= age <= self.tf_timeout:
                            raise ValueError(f'Camera TF is stale/future ({age:.3f}s)')
                matrix = pose_matrix(SimpleNamespace(position=tf.transform.translation,
                                                     orientation=tf.transform.rotation))
            # Freeze in the fixed frame at receipt: old detections do not follow
            # later camera motion while waiting for the next image.
            markers = self.axes.update(camera, message, matrix, self.fixed_frame, now.to_msg())
        except (TransformException, ValueError, TypeError) as exc:
            self.get_logger().warning(f'{camera}: cannot show tag axes: {exc}', throttle_duration_sec=2.0)
            markers = self.axes.clear(camera, message, self.fixed_frame, now.to_msg())
        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = None
    try:
        node = AprilTagRviz()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
