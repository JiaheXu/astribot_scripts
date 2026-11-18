"""Validate real ROS message serialization offline; no ROS node or robot starts."""
import unittest

import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header

from astribot_scripts.common.sensors import to_image_message, to_depth_message


class ImageMessageTests(unittest.TestCase):
    convert = staticmethod(to_image_message)
    convert_depth = staticmethod(to_depth_message)

    def test_noncontiguous_pixels_and_header_survive_ros_serialization(self):
        pixels = np.arange(36, dtype=np.uint8).reshape(3, 4, 3)[:, ::2, :]
        header = Header(frame_id='camera_optical')
        header.stamp.sec, header.stamp.nanosec = 42, 123456789
        message = self.convert(header, pixels)
        restored = deserialize_message(serialize_message(message), Image)
        self.assertEqual(restored.header, header)
        self.assertEqual((restored.height, restored.width, restored.step), (3, 2, 6))
        self.assertEqual(restored.encoding, 'bgr8')
        self.assertEqual(restored.is_bigendian, 0)
        self.assertEqual(bytes(restored.data), pixels.tobytes())

    def test_raw_depth_preserves_uint16_values_and_source_header(self):
        for bigendian in (False, True):
            with self.subTest(bigendian=bigendian):
                pixels = np.array([[0, 1000], [2500, 65535]], dtype='>u2' if bigendian else '<u2')
                source = CompressedImage(format='raw', data=pixels.tobytes())
                source.header.frame_id = ''
                source.header.stamp.sec, source.header.stamp.nanosec = 42, 123
                result = self.convert_depth(source, 2, 2, bigendian)
                restored = deserialize_message(serialize_message(result), Image)
                self.assertEqual(restored.header, source.header)
                self.assertEqual((restored.height, restored.width, restored.step), (2, 2, 4))
                self.assertEqual(restored.encoding, '16UC1')
                self.assertEqual(restored.is_bigendian, int(bigendian))
                self.assertEqual(bytes(restored.data), pixels.tobytes())

    def test_raw_depth_rejects_wrong_format_size_and_dimensions(self):
        for fmt, data, width, height in (
            ('jpeg', bytes(8), 2, 2), ('raw', bytes(7), 2, 2),
            ('raw', bytes(10), 2, 2), ('raw', b'', 0, 2), ('raw', b'', -1, 2)):
            with self.subTest(fmt=fmt, length=len(data), width=width), self.assertRaises(ValueError):
                self.convert_depth(CompressedImage(format=fmt, data=data), width, height, False)

    def test_rejects_raw_compressed_bytes_and_invalid_pixel_arrays(self):
        for pixels in (None, np.zeros(10, dtype=np.uint8),
                       np.zeros((2, 2, 3), dtype=np.float32),
                       np.zeros((0, 2, 3), dtype=np.uint8)):
            with self.subTest(pixels=pixels), self.assertRaises(ValueError):
                self.convert(Header(), pixels)


if __name__ == '__main__':
    unittest.main()
