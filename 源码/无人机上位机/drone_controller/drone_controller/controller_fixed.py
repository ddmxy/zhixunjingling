#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from drone_interfaces.srv import DroneMavros
import numpy as np
import math


class FixedWaypointController(Node):
    def __init__(self):
        super().__init__('drone_controller_fixed')

        # 服务客户端 
        self.drone_client = self.create_client(DroneMavros, '/mavros_to_controller')
        self.drone_client.wait_for_service()
        self.get_logger().info("飞控服务已连接")

        # 订阅者 
        # 无人机当前全局位姿
        self.current_pose = None
        self.create_subscription(
            PoseStamped, '/drone/current_pose', self.pose_callback, 10
        )
        # 二维码识别结果（相机坐标系）
        self.qr_consecutive_count = 0  # 连续检测计数
        self.qr_valid = False
        self.qr_camera_pose = None
        self.create_subscription(
            PoseStamped, '/qr_pose', self.qr_callback, 10
        )

        # 发布者
        # 二维码全局位置发布
        self.qr_global_pub = self.create_publisher(
            PoseStamped, '/qr_global_pose', 10
        )

        # 任务参数
        self.target_altitude = 0.8  # 飞行高度
        self.position_threshold = 0.25  # 到达航点判断阈值
        self.qr_min_consecutive = 3  # 二维码连续检测帧数

        # 打点（世界坐标系，x, y, z）
        self.waypoints = [
            (2.0, 0.0, self.target_altitude),
            (2.0, 2.0, self.target_altitude),
            (0.0, 2.0, self.target_altitude),
            (0.0, 0.0, self.target_altitude)
        ]
        self.current_waypoint_idx = 0

        # ====================== 状态机 ======================
        # IDLE: 待机  TAKEOFF: 起飞中  CRUISE: 巡航中
        # QR_FOUND: 识别到二维码  LANDING: 降落中  FINISHED: 任务结束
        self.task_state = "IDLE"

        # 主循环 10Hz
        self.create_timer(0.1, self.main_loop)
        self.get_logger().info("固定航点巡航主控已启动")

    # 订阅回调
    def pose_callback(self, msg):
        self.current_pose = msg

    def qr_callback(self, msg):
        """二维码回调：连续多帧检测到才判定为有效"""
        self.qr_consecutive_count += 1
        self.qr_camera_pose = msg
        if self.qr_consecutive_count >= self.qr_min_consecutive and not self.qr_valid:
            self.qr_valid = True
            self.get_logger().info("有效二维码已确认")

    # ====================== 工具方法 ======================
    def get_distance_to_target(self, target_x, target_y, target_z):
        """计算无人机当前位置到目标点的距离"""
        if self.current_pose is None:
            return float('inf')
        dx = target_x - self.current_pose.pose.position.x
        dy = target_y - self.current_pose.pose.position.y
        dz = target_z - self.current_pose.pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def set_drone_target(self, x, y, z):
        """非阻塞设置无人机目标点，立刻返回"""
        req = DroneMavros.Request()
        req.command = 6
        req.input_num_1 = x
        req.input_num_2 = y
        req.input_num_3 = z
        self.drone_client.call_async(req)

    def calculate_qr_global_pose(self):
        """
        近似计算二维码世界坐标系位置
        相机与实体机固连、朝向一致，直接叠加相对位置
        """
        if self.current_pose is None or self.qr_camera_pose is None:
            return None

        # 提取无人机当前位置
        drone_x = self.current_pose.pose.position.x
        drone_y = self.current_pose.pose.position.y
        drone_z = self.current_pose.pose.position.z

        # 提取二维码相对相机的位置
        rel_x = self.qr_camera_pose.pose.position.x
        rel_y = self.qr_camera_pose.pose.position.y
        rel_z = self.qr_camera_pose.pose.position.z

        # 叠加
        global_pose = PoseStamped()
        global_pose.header.stamp = self.get_clock().now().to_msg()
        global_pose.header.frame_id = "map"
        global_pose.pose.position.x = drone_x + rel_x
        global_pose.pose.position.y = drone_y + rel_y
        global_pose.pose.position.z = drone_z - rel_z  # 相机朝下时z取反，根据实际安装调整
        global_pose.pose.orientation.w = 1.0
        return global_pose

    # 主循环
    def main_loop(self):
        if self.task_state == "FINISHED":
            return

        # ！识别到二维码，中断所有任务 
        if self.qr_valid and self.task_state not in ["LANDING", "FINISHED"]:
            self.get_logger().warn("检测到有效二维码，中断巡航，准备降落")
            # 悬停
            req = DroneMavros.Request()
            req.command = 5
            self.drone_client.call_async(req)
            # 计算并发布二维码全局位置
            qr_global = self.calculate_qr_global_pose()
            if qr_global:
                self.qr_global_pub.publish(qr_global)
                self.get_logger().info(
                    f"二维码全局位置已发布: "
                    f"({qr_global.pose.position.x:.2f}, "
                    f"{qr_global.pose.position.y:.2f}, "
                    f"{qr_global.pose.position.z:.2f}) m"
                )
            # 切换到降落状态
            self.task_state = "QR_FOUND"
            # 调用降落服务
            req = DroneMavros.Request()
            req.command = 2
            self.drone_client.call_async(req)
            self.task_state = "LANDING"
            return

        # state1：待机 → 执行起飞 
        if self.task_state == "IDLE":
            self.get_logger().info("开始任务：起飞")
            req = DroneMavros.Request()
            req.command = 0
            req.input_num_1 = self.target_altitude
            self.drone_client.call_async(req)
            self.task_state = "TAKEOFF"

        # state2：起飞中 → 到达高度后开始巡航
        elif self.task_state == "TAKEOFF":
            if self.current_pose and self.current_pose.pose.position.z >= self.target_altitude - 0.2:
                self.get_logger().info("起飞完成，开始巡航")
                # 设置第一个航点
                x, y, z = self.waypoints[0]
                self.set_drone_target(x, y, z)
                self.task_state = "CRUISE"

        # state3：巡航中 → 逐个飞航点
        elif self.task_state == "CRUISE":
            current_target = self.waypoints[self.current_waypoint_idx]
            dist = self.get_distance_to_target(*current_target)

            if dist < self.position_threshold:
                # 当前航点到达，飞下一个
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx >= len(self.waypoints):
                    # 所有航点飞完，未找到二维码，返航降落
                    self.get_logger().info("所有航点巡航完成，未发现二维码，返航降落")
                    req = DroneMavros.Request()
                    req.command = 2
                    self.drone_client.call_async(req)
                    self.task_state = "LANDING"
                else:
                    # 设置下一个航点
                    x, y, z = self.waypoints[self.current_waypoint_idx]
                    self.set_drone_target(x, y, z)
                    self.get_logger().info(f"飞往第 {self.current_waypoint_idx+1} 个航点: ({x}, {y}, {z})")

        # State4：降落中 → 任务结束 
        elif self.task_state == "LANDING":
            # 简单判定：高度低于0.2米视为降落完成
            if self.current_pose and self.current_pose.pose.position.z < 0.2:
                self.get_logger().info("降落完成，任务结束")
                self.task_state = "FINISHED"


def main(args=None):
    rclpy.init(args=args)
    controller = FixedWaypointController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().warn("手动中断，紧急降落")
        req = DroneMavros.Request()
        req.command = 2
        controller.drone_client.call_async(req)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()