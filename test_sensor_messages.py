"""Validate real ROS message serialization offline; no ROS node or robot starts."""
import ast
from pathlib import Path
import unittest

import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class ImageMessageTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).with_name('303-get_images_and_lidar.py')
        tree = ast.parse(path.read_text())
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'to_image_message']
        self.assertEqual(len(functions), 1, 'raw Image conversion is required')
        namespace = {'np': np, 'Image': Image}
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), 'exec'), namespace)
        self.convert = namespace['to_image_message']

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

    def test_rejects_raw_compressed_bytes_and_invalid_pixel_arrays(self):
        for pixels in (None, np.zeros(10, dtype=np.uint8),
                       np.zeros((2, 2, 3), dtype=np.float32),
                       np.zeros((0, 2, 3), dtype=np.uint8)):
            with self.subTest(pixels=pixels), self.assertRaises(ValueError):
                self.convert(Header(), pixels)


if __name__ == '__main__':
    unittest.main()
