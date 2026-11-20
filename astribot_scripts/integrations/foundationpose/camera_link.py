"""Forward only head/torso RGB-D from the robot ROS domain to the GPU domain."""
import argparse
from functools import partial
import ipaddress
import json
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from astribot_scripts.common.sensors import to_depth_message, to_image_message


DEPTH_SIZES = {'head_rgbd': (640, 576), 'torso_rgbd': (640, 480)}


def robot_addresses():
    interfaces = json.loads(subprocess.check_output(['ip', '-j', '-4', 'address', 'show']))
    subnet = ipaddress.ip_network('192.168.0.0/24')
    return {address['local'] for interface in interfaces for address in interface['addr_info']
            if ipaddress.ip_address(address['local']) in subnet}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-domain', type=int, default=25)
    parser.add_argument('--output-domain', type=int, default=26)
    args = parser.parse_args()
    if args.input_domain == args.output_domain:
        parser.error('Input and output must use separate domains')
    cv2.setNumThreads(1)
    incoming_addresses = robot_addresses()
    rclpy.init(domain_id=args.input_domain)
    outgoing = Context()
    outgoing.init(domain_id=args.output_domain)
    source = rclpy.create_node('astribot_foundationpose_source')
    target = rclpy.create_node('astribot_camera_link', context=outgoing)
    latest, publishers, last_sdk, counts = {}, {}, {}, {}

    def receive(key, sdk, msg):
        now = time.monotonic()
        if sdk:
            last_sdk[key] = now
        elif now - last_sdk.get(key, float('-inf')) < 1.0:
            return
        latest[key] = msg
        counts[key]['received'] += 1

    for camera in DEPTH_SIZES:
        for kind, suffix in [('color', 'color_compress/compressed'), ('depth', 'depth_compress')]:
            key = (camera, kind)
            counts[key] = {'received': 0, 'published': 0}
            topic = f'/sdk/astribot_camera/{camera}/{kind}_image'
            publishers[key] = target.create_publisher(Image, topic, 2)
            source.create_subscription(Image, topic, partial(receive, key, True), qos_profile_sensor_data)
            # Older SDK launches publish only head depth. The driver carries both.
            source.create_subscription(CompressedImage, f'/astribot_camera/{camera}/{suffix}',
                                       partial(receive, key, False), 2)

    def publish():
        for key, msg in list(latest.items()):
            del latest[key]
            if not publishers[key].get_subscription_count():
                continue
            camera, kind = key
            try:
                if isinstance(msg, CompressedImage):
                    if kind == 'depth':
                        msg = to_depth_message(msg, *DEPTH_SIZES[camera])
                    else:
                        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                        msg = to_image_message(msg.header, bgr, 'rgb8', (640, 480))
                publishers[key].publish(msg)
                counts[key]['published'] += 1
            except (ValueError, cv2.error) as exc:
                source.get_logger().error(f'{camera} {kind}: {exc}', throttle_duration_sec=5)

    def check_link():
        if robot_addresses() != incoming_addresses:
            raise RuntimeError('Robot network address changed. Restarting DDS discovery.')

    source.create_timer(1 / 3, publish)
    source.create_timer(5.0, check_link)
    source.create_timer(10.0, lambda: source.get_logger().info(
        'Camera link totals: ' + json.dumps({'/'.join(k): v for k, v in counts.items()})))
    try:
        rclpy.spin(source)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        source.destroy_node()
        target.destroy_node()
        outgoing.try_shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
