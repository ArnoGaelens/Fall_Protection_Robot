import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Wait for the publisher to connect
        time.sleep(1.0)
        self.run_path()

    def _drive(self, linear_x, angular_z, duration):
        """Send a constant velocity command for a given duration (seconds)."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        end = time.time() + duration
        while time.time() < end:
            self._pub.publish(msg)
            time.sleep(0.05)
        # Stop
        self._pub.publish(Twist())
        time.sleep(0.2)

    def run_path(self):
        self.get_logger().info('Starting path...')

        # Drive forward 2 metres (~4 seconds at 0.5 m/s)
        self._drive(linear_x=0.5, angular_z=0.0, duration=4.0)

        # Turn 90 degrees left (~3.1 seconds at 0.5 rad/s)
        self._drive(linear_x=0.0, angular_z=0.5, duration=3.1)

        # Drive forward 2 metres
        self._drive(linear_x=0.5, angular_z=0.0, duration=4.0)

        # Turn 90 degrees left again
        self._drive(linear_x=0.0, angular_z=0.5, duration=3.1)

        # Drive forward back to roughly start
        self._drive(linear_x=0.5, angular_z=0.0, duration=4.0)

        self.get_logger().info('Path complete.')


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
