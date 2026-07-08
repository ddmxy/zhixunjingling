#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import threading
import sys

SAVE_DIR = "./calib_images"
IMAGE_TOPIC = "/camera/image_raw"

class CalibCmdCollector(Node):
    def __init__(self):
        super().__init__('calib_cmd')
        self.bridge = CvBridge()
        self.count = 0
        self.latest_frame = None
        self.running = True

        os.makedirs(SAVE_DIR, exist_ok=True)

        self.img_sub = self.create_subscription(
            Image, IMAGE_TOPIC, self.image_callback, 10
        )

        # 启动键盘监听线程
        self.key_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.key_thread.start()

        self.get_logger().info("命令行手动采集已启动")
        self.get_logger().info("按 回车键 保存一张图片，输入 q 回车 退出")

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"图像转换失败: {e}")

    def keyboard_listener(self):
        while self.running:
            cmd = sys.stdin.readline().strip()
            if cmd.lower() == 'q':
                self.running = False
                rclpy.shutdown()
                return
            # 回车（空字符串）触发保存
            if self.latest_frame is not None:
                save_path = f"{SAVE_DIR}/calib_{self.count:02d}.jpg"
                cv2.imwrite(save_path, self.latest_frame)
                self.count += 1
                self.get_logger().info(f"[{self.count}] 已保存: {save_path}")
            else:
                self.get_logger().warn("尚未收到图像帧，请检查话题是否正常")

    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CalibCmdCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()