#!/usr/bin/env python3
import rclpy
import math  # 移到最顶部全局导入
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MockQrPublisher(Node):
    def __init__(self):
        super().__init__('mock_qr_publisher')
        self.qr_pub = self.create_publisher(PoseStamped, '/qr_pose', 10)

        # 订阅无人机实时位置
        self.create_subscription(
            PoseStamped, '/drone/current_pose', self.pose_cb, 10
        )

        # 触发条件：飞到(2.0, 2.0)半径0.5m范围内触发
        self.trigger_x = 2.0
        self.trigger_y = 2.0
        self.trigger_radius = 0.5
        self.has_triggered = False

        self.get_logger().info("Mock 二维码仿真节点已启动")

    def pose_cb(self, msg):
        if self.has_triggered:
            return

        dx = msg.pose.position.x - self.trigger_x
        dy = msg.pose.position.y - self.trigger_y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < self.trigger_radius:
            self.has_triggered = True
            # 连续发布3帧，匹配主控防误检逻辑
            qr_msg = PoseStamped()
            qr_msg.header.stamp = self.get_clock().now().to_msg()
            qr_msg.header.frame_id = "camera_link"
            qr_msg.pose.position.x = 0.2
            qr_msg.pose.position.y = 0.1
            qr_msg.pose.position.z = 1.8
            qr_msg.pose.orientation.w = 1.0

            for _ in range(3):
                self.qr_pub.publish(qr_msg)
            self.get_logger().info("模拟识别到二维码，已发布 /qr_pose")

def main(args=None):
    rclpy.init(args=args)
    node = MockQrPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()