#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from hbm_img_msgs.msg import HbmMsg1080P
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2

class HbmToImage(Node):
    def __init__(self):
        super().__init__('hbm_to_image')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/image_raw', 10)

        # 使用 BEST_EFFORT 以匹配 mipi_cam 的发布策略
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.sub = self.create_subscription(
            HbmMsg1080P, '/hbmem_img', self.callback, qos
        )
        self.get_logger().info('HBM → image_raw 转换节点已启动（QoS: BEST_EFFORT）')

    def callback(self, msg):
        raw_data = bytes(msg.data)
        data_len = msg.data_size
        width = msg.width
        height = msg.height

        nv12 = raw_data[:data_len]
        y_size = width * height
        Y = np.frombuffer(nv12[:y_size], dtype=np.uint8).reshape(height, width)
        UV = np.frombuffer(nv12[y_size:], dtype=np.uint8).reshape(height // 2, width // 2, 2)

        U = cv2.resize(UV[:, :, 0], (width, height), interpolation=cv2.INTER_LINEAR)
        V = cv2.resize(UV[:, :, 1], (width, height), interpolation=cv2.INTER_LINEAR)

        yuv = cv2.merge([Y, U, V])
        frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera'
        self.pub.publish(img_msg)

def main():
    rclpy.init()
    node = HbmToImage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()