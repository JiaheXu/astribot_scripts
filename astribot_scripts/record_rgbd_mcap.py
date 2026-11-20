"""Sample the four RGB-D camera drivers directly into a rosbag2 MCAP."""
from functools import partial
from pathlib import Path
import sys

import cv2
import numpy as np
import rclpy
import rosbag2_py
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage

from astribot_scripts.common.sensors import to_depth_message, to_image_message


# The driver's depth_compress transport contains little-endian uint16 pixels
# without image dimensions. Validate the native payload before writing the Image.
DEPTH_SIZES = {
    'head_rgbd': (640, 576),
    'torso_rgbd': (640, 480),
    'left_wrist_rgbd': (640, 360),
    'right_wrist_rgbd': (640, 360),
}
RATE_HZ = 3
MCAP_CONFIG = Path(__file__).resolve().parent / 'config/record_raw_images_mcap.yaml'


class CameraSampler:
    def __init__(self, node, writer):
        self.node = node
        self.writer = writer
        self.latest = {}
        self.counts = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for camera in DEPTH_SIZES:
            for kind, suffix in [('color', 'color_compress/compressed'), ('depth', 'depth_compress')]:
                source = f'/astribot_camera/{camera}/{suffix}'
                topic = f'/sdk/astribot_camera/{camera}/{kind}_image'
                writer.create_topic(rosbag2_py.TopicMetadata(topic, 'sensor_msgs/msg/Image', 'cdr'))
                self.counts[topic] = 0
                node.create_subscription(CompressedImage, source,
                                         partial(self.receive, topic, camera, kind), qos)
                print(f'  {topic}', flush=True)
        # Single-threaded execution makes taking and clearing a sample atomic
        # with respect to receive(). A silent source leaves a gap in the bag.
        node.create_timer(1 / RATE_HZ, self.sample)
        node.create_timer(10, self.report)

    def receive(self, topic, camera, kind, msg):
        self.latest[topic] = (camera, kind, msg)

    def sample(self):
        timestamp = self.node.get_clock().now().nanoseconds
        for topic, (camera, kind, msg) in self.latest.items():
            try:
                if kind == 'depth':
                    image = to_depth_message(msg, *DEPTH_SIZES[camera])
                else:
                    bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                    image = to_image_message(msg.header, bgr, 'rgb8', (640, 480))
            except (ValueError, cv2.error) as exc:
                self.node.get_logger().error(f'{topic}: {exc}', throttle_duration_sec=5)
                continue
            self.writer.write(topic, serialize_message(image), timestamp)
            self.counts[topic] += 1
        self.latest.clear()

    def report(self):
        print('Frames saved (RGB/depth): ' + ', '.join(
            f'{camera} ' + '/'.join(str(self.counts[f"/sdk/astribot_camera/{camera}/{kind}_image"])
                                     for kind in ('color', 'depth'))
            for camera in DEPTH_SIZES), flush=True)


def main(bag_dir):
    cv2.setNumThreads(1)
    rclpy.init(args=[])
    node = rclpy.create_node('astribot_rgbd_recorder', enable_rosout=False)
    writer = rosbag2_py.SequentialWriter()
    try:
        writer.open(rosbag2_py.StorageOptions(
            uri=bag_dir, storage_id='mcap', max_bagfile_size=3_990_000_000,
            max_cache_size=33_554_432, storage_config_uri=str(MCAP_CONFIG)),
            rosbag2_py.ConverterOptions('', ''))
        try:
            sampler = CameraSampler(node, writer)
            print('Waiting for camera frames. Press Ctrl+C to finish recording.', flush=True)
            try:
                rclpy.spin(node)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            sampler.report()
        finally:
            writer.close()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main(sys.argv[1])
