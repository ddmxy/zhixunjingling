#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
from pyzbar.pyzbar import decode, ZBarSymbol

class QrCore(Node):
    def __init__(self):
        super().__init__('qr_core')
        self.bridge = CvBridge()
        self.frame_count = 0

        self.pose_pub = self.create_publisher(PoseStamped, '/qr_pose', 10)
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )

        # ========== 相机标定参数 ==========
        self.camera_matrix = np.array([
            [2.43135718e+03, 0.00000000e+00, 9.76415382e+02],
            [0.00000000e+00, 2.41167159e+03, 5.12394348e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ], dtype=np.float64)
        self.dist_coeffs = np.array([
            -2.49138306e-02, 3.82854750e-01, -1.82692374e-03,
            5.53843113e-04, -1.03991247e+00
        ], dtype=np.float64)

        # 二维码真实边长，单位：米【已修正为15cm = 0.15m】
        self.qr_side_length = 0.15

        self.get_logger().info("二维码识别定位节点已启动（弱光优化版）")
        self.get_logger().info("订阅图像话题: /image_raw")

    def image_callback(self, msg):
        self.frame_count += 1

        # 1. ROS图像转OpenCV格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # 2. 弱光图像预处理
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # 高斯模糊降噪，压制画面颗粒感
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        # 自适应直方图均衡化，提升暗部对比度，强化二维码边界
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)

        # 3. 仅识别QR二维码，跳过所有其他条码，消除警告+提速
        qr_results = decode(gray, symbols=[ZBarSymbol.QRCODE])

        # 未识别到直接返回
        if not qr_results:
            return

        # 4. 位姿解算
        qr = qr_results[0]
        qr_data = qr.data.decode('utf-8')

        # 使用float32格式，适配OpenCV cornerSubPix输入要求
        corner_pts = np.array(qr.polygon, dtype=np.float32)

        # 校验角点数量，异常直接跳过当前帧，避免后续崩溃
        if len(corner_pts) != 4:
            self.get_logger().warn("二维码角点数量异常，跳过当前帧")
            return

        # 调整维度为 (N, 1, 2)，严格匹配 cornerSubPix 输入格式
        corner_pts = corner_pts.reshape(-1, 1, 2)

        #亚像素细化增加异常兜底，失败则使用原始角点，不中断程序
        try:
            corner_pts = cv2.cornerSubPix(
                gray, corner_pts, (5, 5), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
            )
        except cv2.error as e:
            self.get_logger().warn(f"亚像素细化失败，使用原始角点: {e}")

        # 恢复为 (4, 2) 格式，供后续 PnP 位姿解算使用
        corner_pts = corner_pts.reshape(-1, 2)

        # 构建物体坐标系三维点（原点在二维码中心）
        half_len = self.qr_side_length / 2.0
        object_pts = np.array([
            [-half_len, -half_len, 0.0],
            [ half_len, -half_len, 0.0],
            [ half_len,  half_len, 0.0],
            [-half_len,  half_len, 0.0]
        ], dtype=np.float64)

        # PnP解算三维位姿
        success, rvec, tvec = cv2.solvePnP(
            object_pts, corner_pts,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            self.get_logger().warn("PnP位姿解算失败")
            return

        # 旋转向量转四元数
        q = self._rvec_to_quaternion(rvec)

        # 发布位姿消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_link"
        pose_msg.pose.position.x = tvec[0][0]
        pose_msg.pose.position.y = tvec[1][0]
        pose_msg.pose.position.z = tvec[2][0]
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.pose_pub.publish(pose_msg)
        self.get_logger().info(
            f"识别成功 | 内容: {qr_data} | "
            f"位置: ({tvec[0][0]:.3f}, {tvec[1][0]:.3f}, {tvec[2][0]:.3f}) m"
        )

    def _rvec_to_quaternion(self, rvec):
        """旋转向量转四元数 [x, y, z, w]"""
        theta = cv2.norm(rvec)
        if theta < 1e-6:
            return np.array([0.0, 0.0, 0.0, 1.0])
        axis = rvec.flatten() / theta
        sin_half = math.sin(theta / 2.0)
        cos_half = math.cos(theta / 2.0)
        return np.array([
            axis[0] * sin_half,
            axis[1] * sin_half,
            axis[2] * sin_half,
            cos_half
        ])

def main(args=None):
    rclpy.init(args=args)
    node = QrCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()