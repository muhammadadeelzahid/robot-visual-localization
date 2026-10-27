#!/usr/bin/env python3
"""Send a fixed set of waypoints to Nav2 and follow them in order.

Waypoints are a flat [x, y, yaw, ...] list parameter in the map frame.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


DEFAULT_WAYPOINTS = [
    6.0, 0.0, 0.0,
    8.0, 5.5, 1.57,
    -8.0, 5.5, 3.14,
    -8.0, -5.5, -1.57,
    8.0, -5.5, 0.0,
    0.0, 0.0, 1.57,
]


class RouteFollower(Node):

    def __init__(self):
        super().__init__('follow_route')
        flat = self.declare_parameter('waypoints', DEFAULT_WAYPOINTS).value
        self.poses = self._build_poses(flat)
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def _build_poses(self, flat):
        poses = []
        for i in range(0, len(flat) - 2, 3):
            x, y, yaw = flat[i], flat[i + 1], flat[i + 2]
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(p)
        return poses

    def run(self):
        self.get_logger().info('waiting for Nav2 follow_waypoints action...')
        self.client.wait_for_server()
        stamp = self.get_clock().now().to_msg()
        for p in self.poses:
            p.header.stamp = stamp
        goal = FollowWaypoints.Goal()
        goal.poses = self.poses
        self.get_logger().info(f'sending {len(self.poses)} waypoints')
        send = self.client.send_goal_async(
            goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error('waypoints rejected')
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        missed = result_future.result().result.missed_waypoints
        self.get_logger().info(f'route complete; missed waypoints: {list(missed)}')

    def _feedback(self, msg):
        self.get_logger().info(
            f'heading to waypoint {msg.feedback.current_waypoint}')


def main(argv=None):
    rclpy.init(args=argv)
    node = RouteFollower()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
