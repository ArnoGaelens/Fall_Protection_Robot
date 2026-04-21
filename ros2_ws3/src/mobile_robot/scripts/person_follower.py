#!/usr/bin/env python3
"""
Person follower node — reactive controller.
Computes velocity commands directly from person position relative to robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        self.declare_parameter('target_distance', 0.3)
        self.declare_parameter('max_linear_vel',  1.5)
        self.declare_parameter('max_angular_vel', 1.2)
        self.declare_parameter('obstacle_distance', 0.9)  # start braking
        self.declare_parameter('obstacle_stop',    0.35)  # hard stop / back up
        self.declare_parameter('obstacle_angle',   90.0)
        self.declare_parameter('person_ignore_deg', 40.0) # cone around person

        self.target_dist   = self.get_parameter('target_distance').value
        self.max_lin       = self.get_parameter('max_linear_vel').value
        self.max_ang       = self.get_parameter('max_angular_vel').value
        self.obs_dist      = self.get_parameter('obstacle_distance').value
        self.obs_stop      = self.get_parameter('obstacle_stop').value
        self.obs_angle     = math.radians(self.get_parameter('obstacle_angle').value)
        self.person_ignore = math.radians(self.get_parameter('person_ignore_deg').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/person_pose', self.person_cb, 10)
        self.create_subscription(Odometry,    '/odom',        self.odom_cb,   10)
        self.create_subscription(LaserScan,   '/scan',        self.scan_cb,   10)

        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0
        self._person_x  = None
        self._person_y  = None
        self._obstacle_ahead    = False
        self._obstacle_critical = False
        self._obstacle_steer    = 0.0
        self._closest_obs       = float('inf')
        self._stuck_time        = 0.0

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Person follower started')

    def odom_cb(self, msg: Odometry):
        self._robot_x   = msg.pose.pose.position.x
        self._robot_y   = msg.pose.pose.position.y
        self._robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def person_cb(self, msg: PoseStamped):
        self._person_x = msg.pose.position.x
        self._person_y = msg.pose.position.y

    def scan_cb(self, msg: LaserScan):
        person_angle_robot = 0.0
        person_dist = float('inf')
        if self._person_x is not None:
            dx = self._person_x - self._robot_x
            dy = self._person_y - self._robot_y
            person_dist = math.hypot(dx, dy)
            person_angle_world = math.atan2(dy, dx)
            person_angle_robot = math.atan2(
                math.sin(person_angle_world - self._robot_yaw),
                math.cos(person_angle_world - self._robot_yaw))

        ignore_width = self.person_ignore

        self._obstacle_ahead    = False
        self._obstacle_critical = False
        self._obstacle_steer    = 0.0
        self._closest_obs       = float('inf')
        left_threat  = 0.0
        right_threat = 0.0

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > self.obs_angle:
                continue
            if abs(angle - person_angle_robot) < ignore_width:
                continue
            if not (msg.range_min < r < self.obs_dist):
                continue
            if r < self._closest_obs:
                self._closest_obs = r
            threat = (self.obs_dist - r) / self.obs_dist
            if angle > 0:
                left_threat  += threat
            else:
                right_threat += threat
            self._obstacle_ahead = True
            if r < self.obs_stop:
                self._obstacle_critical = True

        if self._obstacle_ahead:
            self._obstacle_steer = -1.0 if left_threat >= right_threat else 1.0

    def control_loop(self):
        if self._person_x is None:
            return

        dx = self._person_x - self._robot_x
        dy = self._person_y - self._robot_y
        dist = math.sqrt(dx*dx + dy*dy)

        angle_world = math.atan2(dy, dx)
        angle_robot = math.atan2(
            math.sin(angle_world - self._robot_yaw),
            math.cos(angle_world - self._robot_yaw))

        twist = Twist()

        # angular: steer toward person, higher gain when far off-axis
        if abs(angle_robot) < math.radians(4):
            ang_cmd = 0.0
        else:
            ang_cmd = 1.0 * angle_robot
        twist.angular.z = max(-self.max_ang, min(self.max_ang, ang_cmd))

        # linear
        if self._obstacle_critical:
            self._stuck_time += 0.1
            twist.linear.x  = -0.2
            twist.angular.z = self.max_ang * self._obstacle_steer
            self.get_logger().warn('Too close — backing up', throttle_duration_sec=1.0)

        elif self._obstacle_ahead:
            self._stuck_time += 0.1
            if self._stuck_time > 2.0:
                twist.linear.x  = -0.2
                twist.angular.z = self.max_ang * self._obstacle_steer
                self.get_logger().warn('Stuck — backing up', throttle_duration_sec=1.0)
            else:
                brake = (self._closest_obs - self.obs_stop) / (self.obs_dist - self.obs_stop)
                brake = max(0.0, min(1.0, brake))
                twist.linear.x  = min(self.max_lin * brake * 0.35, 0.2)
                twist.angular.z = self.max_ang * self._obstacle_steer
                self.get_logger().warn('Obstacle — braking', throttle_duration_sec=1.0)

        else:
            self._stuck_time = 0.0
            dist_error = dist - self.target_dist
            if dist_error > 0.8:
                twist.linear.x = self.max_lin
            elif dist_error > 0.05:
                twist.linear.x = max(0.3, self.max_lin * (dist_error / 0.8))
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
