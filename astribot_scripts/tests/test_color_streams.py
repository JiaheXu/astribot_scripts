"""Exercise compressed input -> rate-limited RGB output with real ROS messages."""
import argparse
import unittest
from unittest.mock import Mock

import cv2
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

from astribot_scripts.common.sensors import SensorPublisher, add_sensor_arguments, to_image_message


CAMERAS = ('head_rgbd', 'torso_rgbd', 'left_wrist_rgbd', 'right_wrist_rgbd')


class ColorStreamTests(unittest.TestCase):
    def setUp(self):
        self.robot = Mock()
        self.node = self.robot.astribot_interface.node
        self.publishers, self.callbacks = {}, {}
        self.node.create_publisher.side_effect = lambda kind, topic, qos: self.publishers.setdefault(topic, Mock())
        self.node.create_subscription.side_effect = (
            lambda kind, topic, callback, qos, **kwargs: self.callbacks.setdefault(topic, callback))
        self.sensor = SensorPublisher(self.robot, CAMERAS, enable_depth=False,
                                      color_encoding='rgb8', image_rate=3.0, image_size=(640, 480))
        self.sensor.start()
        self.addCleanup(self.sensor.close)

    @staticmethod
    def compressed(value, stamp):
        pixels = np.full((720, 1280, 3), (value, value + 1, value + 2), dtype=np.uint8)
        ok, encoded = cv2.imencode('.png', pixels)
        assert ok
        message = CompressedImage(format='bgr8; png compressed bgr8', data=encoded.tobytes())
        message.header.frame_id = 'source_camera_frame'
        message.header.stamp.sec = stamp
        return message

    def test_timer_and_four_inputs_are_selected_without_sdk_decode_or_depth(self):
        self.robot.register_image_callback.assert_not_called()
        self.assertAlmostEqual(self.node.create_timer.call_args.args[0], 1 / 3)
        inputs = [c for c in self.node.create_subscription.call_args_list if c.args[0] is CompressedImage]
        self.assertEqual([c.args[1] for c in inputs],
                         [f'/astribot_camera/{camera}/color_compress/compressed' for camera in CAMERAS])
        self.assertTrue(all(c.args[3].depth == 1 for c in inputs))
        self.assertIsNone(self.sensor.depth_publisher)

    def test_latest_frame_per_camera_is_resized_rgb_and_preserves_header(self):
        for index, camera in enumerate(CAMERAS):
            callback = self.callbacks[f'/astribot_camera/{camera}/color_compress/compressed']
            callback(self.compressed(5, 1))
            callback(self.compressed(10 + index, 42 + index))
        self.assertTrue(all(p.publish.call_count == 0 for p in self.sensor.image_publishers.values()))
        self.node.create_timer.call_args.args[1]()
        for index, camera in enumerate(CAMERAS):
            published = self.sensor.image_publishers[camera].publish
            published.assert_called_once()
            image = deserialize_message(serialize_message(published.call_args.args[0]), Image)
            self.assertEqual((image.width, image.height, image.step), (640, 480, 1920))
            self.assertEqual(image.encoding, 'rgb8')
            self.assertEqual(image.header.frame_id, 'source_camera_frame')
            self.assertEqual(image.header.stamp.sec, 42 + index)
            self.assertEqual(len(image.data), 640 * 480 * 3)
            pixels = np.frombuffer(image.data, np.uint8).reshape(480, 640, 3)
            self.assertTrue(np.all(pixels == np.array([12 + index, 11 + index, 10 + index])))
        # With no new input a second tick must not repeat old source frames.
        self.sensor.publish_latest_images()
        self.assertTrue(all(p.publish.call_count == 1 for p in self.sensor.image_publishers.values()))

    def test_bad_frame_does_not_block_another_camera(self):
        self.sensor.compressed_callback('head_rgbd', CompressedImage(format='jpeg', data=b'broken'))
        self.sensor.compressed_callback('torso_rgbd', self.compressed(25, 70))
        self.sensor.publish_latest_images()
        self.sensor.image_publishers['head_rgbd'].publish.assert_not_called()
        self.sensor.image_publishers['torso_rgbd'].publish.assert_called_once()
        self.sensor.compressed_callback('head_rgbd', CompressedImage(format='raw', data=b'broken'))
        self.sensor.publish_latest_images()
        self.assertEqual(self.node.get_logger().error.call_count, 2)

    def test_closing_destroys_timer_once_and_leaves_drivers_active(self):
        timer = self.sensor.image_timer
        self.sensor.close()
        self.sensor.close()
        self.node.destroy_timer.assert_called_once_with(timer)
        self.robot.deactivate_camera.assert_not_called()
        self.robot.deactivate_lidar.assert_not_called()

    def test_resize_handles_upscale_and_non_four_by_three_input(self):
        source = np.full((2, 5, 3), [1, 2, 3], dtype=np.uint8)
        result = to_image_message(Header(), source, 'rgb8', (640, 480))
        self.assertEqual((result.width, result.height), (640, 480))
        self.assertEqual(bytes(result.data), bytes([3, 2, 1]) * (640 * 480))
        np.testing.assert_array_equal(source[0, 0], [1, 2, 3])

    def test_invalid_rates_and_sizes_are_rejected(self):
        for rate in (0, -1, float('nan'), float('inf')):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                SensorPublisher(self.robot, CAMERAS, image_rate=rate)
        for size in ((0, 480), (640, -1), (1,), (640.5, 480)):
            with self.subTest(size=size), self.assertRaises(ValueError):
                SensorPublisher(self.robot, CAMERAS, image_size=size)
        parser = argparse.ArgumentParser()
        add_sensor_arguments(parser)
        args = parser.parse_args(['--image-rate', '3', '--image-size', '640', '480'])
        self.assertEqual((args.image_rate, args.image_size), (3.0, [640, 480]))


if __name__ == '__main__':
    unittest.main()
