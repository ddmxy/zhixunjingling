#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        # 订阅 /image_raw（由 hbm_to_raw.py 发布）
        self.sub = self.create_subscription(Image, '/image_raw', self.callback, 10)
        self.bridge = CvBridge()
        self.saved = False

    def callback(self, msg):
        if self.saved:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite('/tmp/official_camera-2.jpg', frame)
            self.get_logger().info('已保存 /tmp/official_camera.jpg，请下载到电脑查看')
            self.saved = True
            # 保存一张后主动退出，避免占用资源
            raise SystemExit
        except Exception as e:
            self.get_logger().error(f'转换失败: {e}')

def main():
    rclpy.init()
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()