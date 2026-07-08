#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode

# 导入统一服务类型
from drone_interfaces.srv import DroneMavros

class MavrosCommander(Node):
    def __init__(self):
        super().__init__('mavros_commander')

        # ========== 发布者：位置设定点 ==========
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )

        # ========== 订阅者：当前位置反馈 ==========
        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )

        # ========== MAVROS 基础服务客户端 ==========
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.get_logger().info("Waiting for MAVROS services...")
        self.arming_client.wait_for_service()
        self.mode_client.wait_for_service()
        self.get_logger().info("MAVROS services connected.")

        # ========== 控制参数 ==========
        self.control_rate = 20.0
        self.position_threshold = 0.3
        self.takeoff_timeout = 15.0
        self.goto_timeout = 30.0

        # ========== 对外唯一服务入口（和机械臂对齐） ==========
        self.srv_control = self.create_service(
            DroneMavros, 'mavros_to_controller', self.control_callback
        )
        self.get_logger().info("Drone control service [mavros_to_controller] ready.")

    # ====================== 订阅回调 ======================
    def pose_callback(self, msg):
        self.current_pose = msg

    # ====================== 工具方法 ======================
    def get_distance_to_target(self, target_x, target_y, target_z):
        if self.current_pose is None:
            return float('inf')
        dx = target_x - self.current_pose.pose.position.x
        dy = target_y - self.current_pose.pose.position.y
        dz = target_z - self.current_pose.pose.position.z
        return (dx**2 + dy**2 + dz**2) ** 0.5

    def publish_single_setpoint(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    # ====================== 底层飞控操作 ======================
    def set_mode(self, mode='OFFBOARD'):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def disarm(self):
        req = CommandBool.Request()
        req.value = False
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    # ====================== 核心动作实现（内部方法） ======================
    def _takeoff_impl(self, altitude):
        """起飞：input_num_1 = 目标高度"""
        if self.current_pose is None:
            return False, "No position feedback"

        rate = self.create_rate(self.control_rate)
        start_time = self.get_clock().now()

        # 预发布设定点（必须在切OFFBOARD之前）
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < 1.5:
            self.publish_single_setpoint(0.0, 0.0, altitude)
            rate.sleep()
            rclpy.spin_once(self, timeout_sec=0.001)

        if not self.set_mode('OFFBOARD'):
            return False, "Failed to set OFFBOARD mode"

        if not self.arm():
            self.set_mode('POSCTL')
            return False, "Arming failed"

        # 等待到达目标高度
        while rclpy.ok():
            self.publish_single_setpoint(0.0, 0.0, altitude)
            current_dist = self.get_distance_to_target(0.0, 0.0, altitude)
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9

            if current_dist < self.position_threshold:
                return True, f"Takeoff complete, altitude {altitude}m"

            if elapsed > self.takeoff_timeout:
                self._land_impl()
                return False, "Takeoff timeout"

            rate.sleep()
            rclpy.spin_once(self, timeout_sec=0.001)

    def _goto_impl(self, x, y, z):
        """飞点：input_num_1=x, input_num_2=y, input_num_3=z"""
        if self.current_pose is None:
            return False, "No position feedback"

        rate = self.create_rate(self.control_rate)
        start_time = self.get_clock().now()

        while rclpy.ok():
            self.publish_single_setpoint(x, y, z)
            current_dist = self.get_distance_to_target(x, y, z)
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9

            if current_dist < self.position_threshold:
                return True, f"Reached target ({x}, {y}, {z})"

            if elapsed > self.goto_timeout:
                self._land_impl()
                return False, "Goto timeout"

            rate.sleep()
            rclpy.spin_once(self, timeout_sec=0.001)

    def _land_impl(self):
        """降落：无额外参数"""
        if self.set_mode('AUTO.LAND'):
            return True, "Switched to AUTO.LAND mode"
        return False, "Failed to set land mode"

    # ====================== 服务回调：按command分发（和机械臂完全同构） ======================
    def control_callback(self, request, response):
        """
        command 约定：
        0 = 起飞      input_num_1 = 目标高度
        1 = 飞定点    input_num_1=x, input_num_2=y, input_num_3=z
        2 = 降落
        3 = 解锁
        4 = 上锁
        """
        cmd = request.command
        self.get_logger().info(f"[Command {cmd}] Received")

        # 和机械臂 call_back_1 完全一致的分发逻辑
        if cmd == 0:
            # 起飞
            success, msg = self._takeoff_impl(request.input_num_1)
        elif cmd == 1:
            # 飞往目标点
            success, msg = self._goto_impl(
                request.input_num_1,
                request.input_num_2,
                request.input_num_3
            )
        elif cmd == 2:
            # 降落
            success, msg = self._land_impl()
        elif cmd == 3:
            # 手动解锁
            success = self.arm()
            msg = "Armed" if success else "Arm failed"
        elif cmd == 4:
            # 手动上锁
            success = self.disarm()
            msg = "Disarmed" if success else "Disarm failed"
        else:
            success = False
            msg = f"Unknown command: {cmd}"

        response.success = success
        response.feedback = msg
        self.get_logger().info(f"[Command {cmd}] Result: {msg}")
        return response


def main(args=None):
    rclpy.init(args=args)
    commander = MavrosCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()