#!/usr/bin/env python3
"""Enable only the camera service, without opening the robot-control SDK."""
import rclpy
from std_srvs.srv import SetBool


def main():
    rclpy.init(domain_id=25)
    node = rclpy.create_node('foundationpose_camera_enable')
    try:
        client = node.create_client(SetBool, '/astribot_camera_control')
        if not client.wait_for_service(timeout_sec=30):
            print('Camera-control service unavailable; verify robot connectivity.')
            return 2
        future = client.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(node, future, timeout_sec=20)
        if not future.done():
            print('Camera-control request timed out.')
            return 2
        response = future.result()
        print(f'Camera service: success={response.success}: {response.message}')
        return 0 if response.success else 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
