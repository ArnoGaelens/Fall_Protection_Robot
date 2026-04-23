#!/usr/bin/env python3
"""
Person follower node — reactive controller.
Computes velocity commands directly from person position relative to robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Float64MultiArray
import math


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        self.declare_parameter('target_distance',   0.1)
        self.declare_parameter('max_linear_vel',    0.8)
        self.declare_parameter('max_angular_vel',   1.5)
        self.declare_parameter('obstacle_distance', 1.5)   # repulsion starts here
        self.declare_parameter('obstacle_stop',     0.4)   # back up zone
        self.declare_parameter('speed_slow_dist',   0.7)   # speed reduction starts here
        self.declare_parameter('obstacle_angle',    100.0)
        self.declare_parameter('person_ignore_deg', 50.0)

        self.target_dist   = self.get_parameter('target_distance').value
        self.max_lin       = self.get_parameter('max_linear_vel').value
        self.max_ang       = self.get_parameter('max_angular_vel').value
        self.obs_dist      = self.get_parameter('obstacle_distance').value
        self.obs_stop      = self.get_parameter('obstacle_stop').value
        self.speed_slow    = self.get_parameter('speed_slow_dist').value
        self.obs_angle     = math.radians(self.get_parameter('obstacle_angle').value)
        self.person_ignore = math.radians(self.get_parameter('person_ignore_deg').value)

        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)
        self.swivel_pub = self.create_publisher(
            Float64MultiArray, '/upper_swivel_velocity_controller/commands', 10)
        self.create_subscription(PoseStamped, '/person_pose',  self.person_cb,      10)
        self.create_subscription(Odometry,    '/odom',         self.odom_cb,        10)
        self.create_subscription(LaserScan,   '/scan',         self.scan_cb,        10)
        self.create_subscription(JointState,  '/joint_states', self.joint_state_cb, 10)

        self._robot_x   = 0.0
        self._robot_y   = 0.0
        self._robot_yaw = 0.0
        self._person_x   = None
        self._person_y   = None
        self._person_yaw = 0.0

        self.declare_parameter('rear_offset', 0.6)   # metres behind person
        self.declare_parameter('left_offset', 0.35)  # metres to person's left
        self.rear_offset = self.get_parameter('rear_offset').value
        self.left_offset = self.get_parameter('left_offset').value

        self._avoid_ang   = 0.0   # repulsive steering from potential field
        self._closest_obs = float('inf')
        self._critical    = False
        self._swivel_angle = 0.0

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Person follower started')

    def joint_state_cb(self, msg: JointState):
        if 'Upper_swivel' in msg.name:
            idx = msg.name.index('Upper_swivel')
            self._swivel_angle = msg.position[idx]

    def odom_cb(self, msg: Odometry):
        self._robot_x   = msg.pose.pose.position.x
        self._robot_y   = msg.pose.pose.position.y
        self._robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def person_cb(self, msg: PoseStamped):
        self._person_x   = msg.pose.position.x
        self._person_y   = msg.pose.position.y
        self._person_yaw = yaw_from_quaternion(msg.pose.orientation)

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

        self._closest_obs = float('inf')
        self._critical    = False
        repulsion         = 0.0

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > self.obs_angle:
                continue
            if abs(angle - person_angle_robot) < self.person_ignore and r > person_dist * 0.7:
                continue
            if r >= self.obs_dist:
                continue

            if r < self._closest_obs:
                self._closest_obs = r
            if r < self.obs_stop:
                self._critical = True

            # potential field: obstacle at angle pushes robot in opposite direction
            # weight = proximity squared (stronger when very close)
            weight = ((self.obs_dist - r) / self.obs_dist) ** 2
            # forward-facing obstacles matter more than side ones
            forward_factor = math.cos(angle) * 0.5 + 0.5
            repulsion += -math.sin(angle) * weight * forward_factor

        # scale and store
        self._avoid_ang = max(-self.max_ang, min(self.max_ang, 2.5 * repulsion))

    def control_loop(self):
        if self._person_x is None:
            return

        # compute rear-left target position based on person heading
        py = self._person_yaw
        target_x = self._person_x \
            - self.rear_offset * math.cos(py) \
            - self.left_offset * math.sin(py)
        target_y = self._person_y \
            - self.rear_offset * math.sin(py) \
            + self.left_offset * math.cos(py)

        dx = target_x - self._robot_x
        dy = target_y - self._robot_y
        dist = math.sqrt(dx * dx + dy * dy)
        angle_robot = math.atan2(
            math.sin(math.atan2(dy, dx) - self._robot_yaw),
            math.cos(math.atan2(dy, dx) - self._robot_yaw))

        twist = Twist()

        if self._critical:
            # dangerously close — back up
            twist.linear.x  = -0.2
            twist.angular.z = self._avoid_ang if self._avoid_ang != 0 else self.max_ang
            self.get_logger().warn('Too close — backing up', throttle_duration_sec=1.0)
        else:
            # person-following steering
            person_ang = 0.0 if abs(angle_robot) < math.radians(2) else \
                max(-self.max_ang, min(self.max_ang, 2.0 * angle_robot))

            # blend: avoid_ang takes over as obstacle gets closer
            proximity = max(0.0, 1.0 - (self._closest_obs / self.obs_dist))
            twist.angular.z = (1.0 - proximity) * person_ang + proximity * self._avoid_ang

            # slow down only when closer than speed_slow_dist (steering reacts earlier at obs_dist)
            speed_scale = min(1.0, (self._closest_obs - self.obs_stop) /
                              (self.speed_slow - self.obs_stop)) if self._closest_obs < self.speed_slow else 1.0
            dist_error = dist - self.target_dist
            if dist_error > 0.05:
                twist.linear.x = min(self.max_lin * speed_scale, max(0.15, dist_error * self.max_lin))
            elif dist_error < -0.05:
                twist.linear.x = max(-0.2, dist_error * self.max_lin)

        self.cmd_pub.publish(twist)

        # swivel always faces person
        swivel_error = math.atan2(
            math.sin(angle_robot - self._swivel_angle),
            math.cos(angle_robot - self._swivel_angle))
        swivel_msg = Float64MultiArray()
        swivel_msg.data = [max(-4.0, min(4.0, 5.0 * swivel_error))]
        self.swivel_pub.publish(swivel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
