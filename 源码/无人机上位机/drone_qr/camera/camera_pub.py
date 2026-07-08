#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from hobot_vio import libsrcampy

class MipiCameraPublisher(Node):
    def __init__(self):
        super().__init__('mipi_camera_pub')
        self.bridge = CvBridge()
        self.video_index = 0
        self.width = 1920
        self.height = 1080
        self.fps = 30

        self.cam = libsrcampy.Camera()
        ret = self.cam.open_cam(0, self.video_index, self.fps, self.width, self.height)
        if ret != 0:
            self.get_logger().error(f"摄像头打开失败，错误码: {ret}")
            raise RuntimeError("摄像头初始化失败")

        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)
        self.get_logger().info(f"IMX219 已启动，格式 NV16 (U first)")

    def timer_callback(self):
        img_bytes = self.cam.get_img(0)
        if not isinstance(img_bytes, bytes) or len(img_bytes) == 0:
            return

        y_size = self.width * self.height
        Y = np.frombuffer(img_bytes[:y_size], dtype=np.uint8).reshape((self.height, self.width))
        UV = np.frombuffer(img_bytes[y_size:], dtype=np.uint8).reshape((self.height, self.width // 2, 2))

        # NV16 (U 在前)
        U = cv2.resize(UV[:, :, 0], (self.width, self.height), interpolation=cv2.INTER_LINEAR)
        V = cv2.resize(UV[:, :, 1], (self.width, self.height), interpolation=cv2.INTER_LINEAR)

        yuv = cv2.merge([Y, U, V])
        frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        self.img_pub.publish(msg)

    def destroy_node(self):
        self.cam.close_cam()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MipiCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()