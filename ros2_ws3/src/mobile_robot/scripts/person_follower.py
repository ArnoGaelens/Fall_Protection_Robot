#!/usr/bin/env python3
"""
Person follower node — reactive controller.
Directly publishes /cmd_vel based on person position and LiDAR obstacle avoidance.
No Nav2 global planner — fast, smooth, responsive.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import math


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        self.declare_parameter('target_distance', 0.4)   # desired following distance (m)
        self.declare_parameter('max_linear_vel', 0.4)    # m/s
        self.declare_parameter('max_angular_vel', 1.0)   # rad/s
        self.declare_parameter('obstacle_distance', 0.3) # stop if obstacle closer than this (m)
        self.declare_parameter('obstacle_angle', 45.0)   # degrees to check in front

        self.target_dist = self.get_parameter('target_distance').value
        self.max_lin = self.get_parameter('max_linear_vel').value
        self.max_ang = self.get_parameter('max_angular_vel').value
        self.obs_dist = self.get_parameter('obstacle_distance').value
        self.obs_angle = math.radians(self.get_parameter('obstacle_angle').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/person_pose', self.person_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self._person_x = None
        self._person_y = None
        self._obstacle_ahead = False

        self.create_timer(0.1, self.control_loop)   # 10 Hz control loop
        self.get_logger().info('Person follower (reactive) started')

    def person_callback(self, msg: PoseStamped):
        self._person_x = msg.pose.position.x
        self._person_y = msg.pose.position.y

    def scan_callback(self, msg: LaserScan):
        # check for obstacles in front cone, but ignore the person's direction
        person_angle = math.atan2(self._person_y, self._person_x) if self._person_x is not None else 0.0
        person_ignore_width = math.radians(30.0)  # ignore 30° around person direction

        self._obstacle_ahead = False
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > self.obs_angle:
                continue
            # skip the sector where the person is (that's not an obstacle)
            if abs(angle - person_angle) < person_ignore_width:
                continue
            if msg.range_min < r < self.obs_dist:
                self._obstacle_ahead = True
                return

    def control_loop(self):
        if self._person_x is None:
            return

        twist = Twist()

        # angle and distance to person (in robot/odom frame)
        angle_to_person = math.atan2(self._person_y, self._person_x)
        dist_to_person = math.sqrt(self._person_x**2 + self._person_y**2)

        # angular velocity — proportional to heading error
        twist.angular.z = max(-self.max_ang, min(self.max_ang, 1.5 * angle_to_person))

        # linear velocity — proportional to distance error, zero if obstacle ahead
        if self._obstacle_ahead:
            twist.linear.x = 0.0
            self.get_logger().warn('Obstacle ahead — stopping', throttle_duration_sec=1.0)
        else:
            dist_error = dist_to_person - self.target_dist
            if dist_error > 0.1:   # only move forward if person is farther than target
                speed = min(self.max_lin, 0.5 * dist_error)
                twist.linear.x = speed
            else:
                twist.linear.x = 0.0

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
