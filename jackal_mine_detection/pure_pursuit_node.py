#!/usr/bin/env python3
"""
pure_pursuit_node.py
Pure Pursuit 기반 Local Planner (Nav2 미사용)

구독:
  /path                    (nav_msgs/Path)
  /j100_0915/platform/odom (nav_msgs/Odometry)
  /tag_detected            (std_msgs/Bool)

발행:
  /j100_0915/cmd_vel       (geometry_msgs/Twist)
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.lookahead_dist = 0.5
        self.linear_speed = 0.3
        self.goal_tolerance = 0.3

        self.path = []
        self.tag_detected = False

        self.path_sub = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/j100_0915/platform/odom',
            self.odom_callback,
            10
        )

        self.tag_sub = self.create_subscription(
            Bool,
            '/tag_detected',
            self.tag_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/j100_0915/cmd_vel',
            10
        )

        self.get_logger().info('Pure Pursuit Local Planner started.')

    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.get_logger().info(f'Received path: {len(self.path)} poses')

    def tag_callback(self, msg: Bool):
        self.tag_detected = msg.data

        if self.tag_detected:
            self.get_logger().info('Tag detected. Stop robot.')
            self.publish_stop()

    def odom_callback(self, msg: Odometry):
        if self.tag_detected:
            self.publish_stop()
            return

        if not self.path:
            self.publish_stop()
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.follow_path(x, y, yaw)

    def follow_path(self, x, y, yaw):
        goal = self.path[-1].pose.position
        dist_to_goal = math.hypot(goal.x - x, goal.y - y)

        if dist_to_goal <= self.goal_tolerance:
            self.get_logger().info('Goal reached. Stop robot.')
            self.publish_stop()
            return

        target = None

        for pose in self.path:
            px = pose.pose.position.x
            py = pose.pose.position.y

            dist = math.hypot(px - x, py - y)

            if dist >= self.lookahead_dist:
                target = pose.pose.position
                break

        if target is None:
            self.publish_stop()
            return

        dx = target.x - x
        dy = target.y - y

        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        angular_z = 2.0 * self.linear_speed * math.sin(alpha) / self.lookahead_dist

        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = angular_z

        self.cmd_pub.publish(twist)

    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()