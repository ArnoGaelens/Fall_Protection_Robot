#!/usr/bin/env python3
"""
Simulated person node — publishes a moving /person_pose (PoseStamped)
and moves a Gazebo cylinder model to match.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import subprocess


class PersonSim(Node):
    def __init__(self):
        super().__init__('person_sim')

        self.declare_parameter('radius', 2.5)
        self.declare_parameter('speed', 0.3)      # rad/s
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)

        self.radius = self.get_parameter('radius').value
        self.speed = self.get_parameter('speed').value
        self.cx = self.get_parameter('center_x').value
        self.cy = self.get_parameter('center_y').value

        self.pub = self.create_publisher(PoseStamped, '/person_pose', 10)
        self.timer = self.create_timer(0.1, self.tick)   # 10 Hz
        self.angle = 0.0
        self._gz_tick = 0

        self.get_logger().info('Person sim started — walking in a circle')

    def tick(self):
        self.angle += self.speed * 0.1

        px = self.cx + self.radius * math.cos(self.angle)
        py = self.cy + self.radius * math.sin(self.angle)

        # publish ROS2 pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = px
        msg.pose.position.y = py
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

        # update Gazebo model every 5 ticks (2 Hz) to avoid overloading
        self._gz_tick += 1
        if self._gz_tick >= 5:
            self._gz_tick = 0
            req = f'name: "person" position: {{x: {px:.3f} y: {py:.3f} z: 0.9}}'
            subprocess.Popen(
                ['gz', 'service', '-s', '/world/empty/set_pose',
                 '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean',
                 '--req', req,
                 '--timeout', '200'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )


def main(args=None):
    rclpy.init(args=args)
    node = PersonSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
