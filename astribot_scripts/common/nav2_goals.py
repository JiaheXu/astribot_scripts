"""RViz map clicks -> Nav2 actions, with map-frame target/path feedback."""
from copy import deepcopy
import math

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from astribot_scripts.common.nav2 import ODOM_FRAME, pose_matrix


def matrix_from_pose(pose):
    p, q = pose.position, pose.orientation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def set_pose(pose, matrix):
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = pose.orientation
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(matrix[:3, :3]).as_quat())


class RvizGoals:
    def __init__(self, node, projector, feedback_reason):
        self.node, self.projector, self.feedback_reason = node, projector, feedback_reason
        self.client = ActionClient(node, NavigateToPose, '/astribot_nav/navigate_to_pose')
        self.pending = False
        self.cancel_pending = False
        self.active = None
        self.active_target = None
        self.target = None
        self.last_feedback_second = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = node.create_publisher(MarkerArray, '/astribot_nav/goal_markers', qos)
        self.status = node.create_publisher(String, '/astribot_nav/goal_status', qos)
        self.path = node.create_publisher(Path, '/astribot_nav/plan_map', 1)
        self.subscriptions = [
            node.create_subscription(PoseStamped, '/astribot_nav/goal_pose_map', self.receive_pose, 1),
            node.create_subscription(PointStamped, '/astribot_nav/goal_point_map', self.receive_point, 1),
            node.create_subscription(Path, '/astribot_nav/plan', self.receive_path, 1)]
        self.cancel_service = node.create_service(Trigger, '/astribot_nav/cancel_goal', self.cancel)

    def report(self, text, color=(0.2, 0.8, 1.0)):
        if not self.node.enable_output:
            text = f'Preview (motion disabled): {text}'
        self.status.publish(String(data=text))
        self.node.get_logger().info(f'RViz navigation: {text}')
        if self.target is None:
            return
        markers = MarkerArray()
        for index, kind in enumerate((Marker.ARROW, Marker.TEXT_VIEW_FACING)):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns, marker.id, marker.type, marker.action = 'navigation_goal', index, kind, Marker.ADD
            marker.pose = deepcopy(self.target.pose)
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            if index == 0:
                marker.scale.x, marker.scale.y, marker.scale.z = 0.5, 0.08, 0.12
                marker.pose.position.z += 0.05
            else:
                marker.pose.position.z += 0.5
                marker.scale.z = 0.16
                p = self.target.pose.position
                marker.text = f'({p.x:.2f}, {p.y:.2f})\n{text}'
            markers.markers.append(marker)
        self.markers.publish(markers)

    def receive_point(self, message):
        if self.projector.previous is None or self.projector.odom_from_world is None:
            self.report('Goal rejected: waiting for navigation odometry', (1.0, 0.3, 0.2))
            return
        # Publish Point selects a 3D surface. Only X/Y determine the base goal.
        world_from_base = np.linalg.inv(self.projector.odom_from_world) @ self.projector.previous[1]
        yaw = math.atan2(world_from_base[1, 0], world_from_base[0, 0])
        goal = PoseStamped()
        goal.header = deepcopy(message.header)
        goal.pose.position.x, goal.pose.position.y = message.point.x, message.point.y
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.receive_pose(goal)

    def receive_pose(self, message):
        try:
            if message.header.frame_id != 'map':
                raise ValueError('set RViz Fixed Frame to map')
            reason = self.feedback_reason()
            if reason or self.projector.odom_from_world is None:
                raise ValueError(reason or 'waiting for navigation origin')
            if self.pending:
                raise ValueError('previous goal request is pending; click again after its response')
            if not self.client.server_is_ready():
                raise ValueError('Nav2 NavigateToPose server is unavailable')
            world_goal = matrix_from_pose(message.pose)
            local_goal = self.projector.odom_from_world @ world_goal
            # A map click specifies a planar goal, regardless of the picked height.
            yaw = math.atan2(local_goal[1, 0], local_goal[0, 0])
            local_goal[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
            local_goal[2, 3] = 0.0
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = ODOM_FRAME
            request.pose.header.stamp = self.node.get_clock().now().to_msg()
            set_pose(request.pose.pose, local_goal)
            target = PoseStamped()
            target.header.frame_id = 'map'
            set_pose(target.pose, np.linalg.inv(self.projector.odom_from_world) @ local_goal)
        except (ValueError, TypeError) as exc:
            suffix = '; previous goal remains active' if self.active is not None else ''
            self.report(f'Goal rejected: {exc}{suffix}', (1.0, 0.3, 0.2))
            return
        self.pending, self.cancel_pending = True, False
        self.target = target
        self.last_feedback_second = None
        self.report('Sending target to Nav2', (1.0, 0.8, 0.1))
        try:
            future = self.client.send_goal_async(request, feedback_callback=self.feedback)
            future.add_done_callback(self.goal_response)
        except Exception as exc:
            self.pending = False
            if self.active is not None:
                self.target = self.active_target
            self.report(f'Goal request failed: {exc}', (1.0, 0.3, 0.2))

    def goal_response(self, future):
        self.pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                suffix = '; previous goal remains active' if self.active is not None else ''
                if self.active is not None:
                    self.target = self.active_target
                self.report(f'Nav2 rejected target{suffix}', (1.0, 0.3, 0.2))
                return
            self.active = handle
            self.active_target = self.target
            handle.get_result_async().add_done_callback(lambda result: self.result(handle, result))
            if self.cancel_pending or self.feedback_reason() is not None:
                self.request_cancel(handle)
                self.report('Cancel requested', (1.0, 0.8, 0.1))
            else:
                self.report('Navigating')
        except Exception as exc:
            if self.active is not None:
                self.target = self.active_target
            self.report(f'Goal response failed: {exc}', (1.0, 0.3, 0.2))

    def feedback(self, message):
        if (self.pending or self.active is None
                or bytes(message.goal_id.uuid) != bytes(self.active.goal_id.uuid)):
            return
        second = self.node.get_clock().now().nanoseconds // 1_000_000_000
        if second != self.last_feedback_second:
            self.last_feedback_second = second
            self.report(f'Navigating: {message.feedback.distance_remaining:.2f} m remaining')

    def result(self, handle, future):
        if handle is not self.active:
            return  # A newer accepted Nav2 goal preempted this one.
        self.active = None
        try:
            status = future.result().status
            text = {4: 'Target reached', 5: 'Navigation canceled', 6: 'Navigation failed'}.get(
                status, f'Navigation finished with status {status}')
            self.report(text, (0.2, 1.0, 0.3) if status == 4 else (1.0, 0.3, 0.2))
        except Exception as exc:
            self.report(f'Navigation result unavailable: {exc}', (1.0, 0.3, 0.2))

    def request_cancel(self, handle):
        def completed(future):
            try:
                if not future.result().goals_canceling:
                    self.node.get_logger().warning('Nav2 did not accept cancellation; inspect goal status')
            except Exception as exc:
                self.node.get_logger().error(f'Nav2 cancel failed: {exc}')
        handle.cancel_goal_async().add_done_callback(completed)

    def cancel(self, request, response):
        self.cancel_pending = self.pending
        if self.active is not None:
            self.request_cancel(self.active)
        response.success = True
        response.message = ('Cancel requested; awaiting Nav2 result' if self.pending or self.active is not None
                            else 'No RViz navigation goal is active')
        self.report(response.message, (1.0, 0.8, 0.1))
        return response

    def receive_path(self, message):
        if self.projector.odom_from_world is None or self.feedback_reason() is not None:
            return
        try:
            if message.header.frame_id != ODOM_FRAME:
                raise ValueError(f'Expected path in {ODOM_FRAME}')
            world_from_odom = np.linalg.inv(self.projector.odom_from_world)
            output = deepcopy(message)
            output.header.frame_id = 'map'
            for item in output.poses:
                if item.header.frame_id not in ('', ODOM_FRAME):
                    raise ValueError('Unexpected path pose frame')
                set_pose(item.pose, world_from_odom @ matrix_from_pose(item.pose))
                item.header.frame_id = 'map'
        except (ValueError, TypeError) as exc:
            self.node.get_logger().warning(f'Cannot display Nav2 path: {exc}', throttle_duration_sec=2.0)
            return
        self.path.publish(output)
