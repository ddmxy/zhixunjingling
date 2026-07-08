#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from drone_interfaces.srv import DroneMavros
import math

class MockMavros(Node):
    def __init__(self):
        super().__init__('mock_mavros')

        # 物理模拟参数
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.fly_speed = 0.5
        self.position_threshold = 0.1

        # 飞行状态标志
        self.offboard_active = False
        self.is_armed = False
        self.is_landing = False

        # 飞行目标点
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        # 位置发布话题
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/current_pose', 10)
        self.create_timer(0.05, self._update_and_publish_pose)

        # 控制服务
        self.create_service(DroneMavros, '/mavros_to_controller', self.control_callback)
        self.get_logger().info("Mock MAVROS 仿真节点已启动，服务: /mavros_to_controller ")

    def _update_and_publish_pose(self):
        """后台匀速仿真移动并发布位置"""
        if self.offboard_active and not self.is_landing:
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            dz = self.target_z - self.current_z
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

            if dist > self.position_threshold:
                step = self.fly_speed * 0.05
                ratio = min(step / dist, 1.0)
                self.current_x += dx * ratio
                self.current_y += dy * ratio
                self.current_z += dz * ratio

        # 降落逻辑
        if self.is_landing:
            if self.current_z > 0.05:
                self.current_z -= 0.3 * 0.05
            else:
                self.current_z = 0.0
                self.is_landing = False
                self.offboard_active = False
                self.is_armed = False

        # 发布位姿
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.current_x
        msg.pose.position.y = self.current_y
        msg.pose.position.z = self.current_z
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def control_callback(self, request, response):
        cmd = request.command
        self.get_logger().info(f"[Command {cmd}] Received")

        # 0 = 起飞 input_num_1:高度
        if cmd == 0:
            alt = request.input_num_1
            if self.current_z > 0.1:
                response.success = False
                response.feedback = "Already in air"
            else:
                self.target_x = 0.0
                self.target_y = 0.0
                self.target_z = alt
                self.offboard_active = True
                self.is_armed = True
                self.is_landing = False
                response.success = True
                response.feedback = "Takeoff target set"

        # 1 = 阻塞飞往定点 input_num1/2/3 xyz
        elif cmd == 1:
            if not self.offboard_active:
                response.success = False
                response.feedback = "Not OFFBOARD"
            else:
                self.target_x = request.input_num_1
                self.target_y = request.input_num_2
                self.target_z = request.input_num_3
                response.success = True
                response.feedback = "Goto target updated"

        # 2 = 降落
        elif cmd == 2:
            self.is_landing = True
            self.offboard_active = False
            response.success = True
            response.feedback = "Landing trigger set"

        # 3 = 解锁
        elif cmd == 3:
            self.is_armed = True
            response.success = True
            response.feedback = "Armed"

        # 4 = 上锁
        elif cmd == 4:
            self.is_armed = False
            self.offboard_active = False
            response.success = True
            response.feedback = "Disarmed"

        # 5 = 悬停（锁定当前位置）
        elif cmd == 5:
            if not self.offboard_active:
                response.success = False
                response.feedback = "Not OFFBOARD"
            else:
                self.target_x = self.current_x
                self.target_y = self.current_y
                self.target_z = self.current_z
                response.success = True
                response.feedback = "Hover activated"

        # 6 = 非阻塞设置目标点
        elif cmd == 6:
            if not self.offboard_active:
                response.success = False
                response.feedback = "Not OFFBOARD"
            else:
                self.target_x = request.input_num_1
                self.target_y = request.input_num_2
                self.target_z = request.input_num_3
                response.success = True
                response.feedback = f"Target set ({request.input_num_1:.2f},{request.input_num_2:.2f},{request.input_num_3:.2f})"

        # 未知指令
        else:
            response.success = False
            response.feedback = f"Invalid command: {cmd}"

        self.get_logger().info(f"[Command {cmd}] Result: {response.feedback}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockMavros()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()