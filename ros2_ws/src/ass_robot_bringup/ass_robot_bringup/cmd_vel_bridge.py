import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gz.transport13 import Node as GzNode
from gz.msgs10.twist_pb2 import Twist as GzTwist


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self._gz_node = GzNode()
        self._pub = self._gz_node.advertise('/cmd_vel', GzTwist)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self.get_logger().info('cmd_vel bridge ready')

    def _cb(self, msg: Twist):
        gz_msg = GzTwist()
        gz_msg.linear.x = msg.linear.x
        gz_msg.linear.y = msg.linear.y
        gz_msg.linear.z = msg.linear.z
        gz_msg.angular.x = msg.angular.x
        gz_msg.angular.y = msg.angular.y
        gz_msg.angular.z = msg.angular.z
        self._pub.publish(gz_msg)


def main():
    rclpy.init()
    rclpy.spin(CmdVelBridge())
    rclpy.shutdown()
