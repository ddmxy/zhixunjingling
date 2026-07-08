#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class CommunicationCore(Node):
    def __init__(self):
        super().__init__('communication_core')
        self.sub = self.create_subscription(PoseStamped, '/qr_target_for_car', self.callback, 10)
        self.car_pub = self.create_publisher(PoseStamped, '/car/qr_goal', 10)

    def callback(self, msg):
        self.get_logger().info(f"Send to car: {msg.pose.position}")
        self.car_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommunicationCore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()