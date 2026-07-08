#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from drone_interfaces.srv import DroneMavros

class MavrosCommander(Node):
    def __init__(self):
        """
            初始化节点和服务
            1. 创建发布器：用于发布目标点到 /mavros/setpoint_position/local
            2. 创建订阅器：用于订阅无人机当前位置 /mavros/local_position/pose
            3. 创建服务客户端：用于调用解锁/上锁和模式切换服务
            4. 设置一些参数：发布频率、到达阈值、超时时间等
            5. 等待服务连接完成
        """
        super().__init__('mavros_core') 

        # 创建发布器
        self.setpoint_pub = self.create_publisher(
            PoseStamped, 
            '/mavros/setpoint_position/local', 
            10)
        # 创建订阅器
        self.current_pose = None  # 保存无人机实时位置
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )

        # 创建服务
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming') # 解锁 / 上锁服务
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode') # 飞行模式切换服务

        # 一些参数
        self.control_rate = 20.0        # 设定点发布频率 20Hz
        self.position_threshold = 0.3   # 到达判断阈值：距离目标点小于0.3米视为到达
        self.takeoff_timeout = 15.0     # 起飞超时时间（s）
        self.goto_timeout = 30.0        # 定点飞行超时时间（s）

        # ========== 新增：全局目标点 + OFFBOARD状态标志 ==========
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.offboard_active = False  # 是否处于OFFBOARD模式，激活后定时器持续发设定点

        # ========== 新增：后台定时器，统一循环发布设定点 ==========
        self.create_timer(1.0 / self.control_rate, self._setpoint_publish_loop)

        # 等待服务
        self.get_logger().info("Waiting for MAVROS services...")
        self.arming_client.wait_for_service()
        self.mode_client.wait_for_service()
        self.get_logger().info("Services connected.")

        # 创建对外服务接口
        self.srv_control = self.create_service(
            DroneMavros, '/mavros_to_controller', self.control_callback
        )
        self.get_logger().info("Drone control service [/mavros_to_controller] ready.")


# ============ TOOLS AREA ============
    def pose_callback(self, msg):
        """
            订阅无人机当前位置，实时更新成员变量
        """
        self.current_pose = msg

    def get_distance_to_target(self, target_x, target_y, target_z):
        """
            计算当前位置到目标点的距离，单位：米
        """
        if self.current_pose is None:
            return float('inf')  # 还没收到位置数据，返回无穷大
        dx = target_x - self.current_pose.pose.position.x
        dy = target_y - self.current_pose.pose.position.y
        dz = target_z - self.current_pose.pose.position.z
        return (dx**2 + dy**2 + dz**2) ** 0.5

    def publish_single_setpoint(self, x, y, z):
        """
            发布单次位置设定点
            x y z: E N U 坐标系下的目标位置，单位：米
            原点定义:飞控初始化完成瞬间机身中心位置
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        # 姿态默认保持水平，航向角0；如需控制航向可补充四元数设置
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    # ========== 新增：定时器回调，OFFBOARD激活后自动循环发布设定点 ==========
    def _setpoint_publish_loop(self):
        """后台持续发布当前目标点，无需每个函数单独写循环"""
        if self.offboard_active:
            self.publish_single_setpoint(self.target_x, self.target_y, self.target_z)

# ========= MAVROS SERVICE AREA =========
    def set_mode(self, mode='OFFBOARD'):
        """
            设置飞行模式
        """
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent

    def arm(self):
        """
            解锁飞控
        """
        req = CommandBool.Request()
        req.value = True 
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
    
    def disarm(self):
        """
            上锁飞控
        """
        req = CommandBool.Request()
        req.value = False 
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

# ============ MAIN ACTION AREA ============
    def takeoff(self, altitude):
        """
        标准PX4 OFFBOARD起飞流程:
        1. 先持续发送设定点（满足飞控准入条件）
        2. 切换OFFBOARD模式
        3. 解锁无人机
        4. 等待到达目标高度
        --input_num_1 = altitude
        """
        # 前置检查：确保已经收到位置反馈
        if self.current_pose is None:
            self.get_logger().error("No position feedback! Check MAVROS connection.")
            return False, "No position feedback"

        rate = self.create_rate(self.control_rate)
        start_time = self.get_clock().now()

        # 预发布设定点，必须在切模式之前 
        ## PX4要求：切入OFFBOARD前必须至少有1秒的设定点流，否则直接拒绝
        self.get_logger().info("Publishing setpoints before entering OFFBOARD...")
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < 1.5:
            self.publish_single_setpoint(0.0, 0.0, altitude)
            rate.sleep()
            rclpy.spin_once(self, timeout_sec=0.001)  # 处理回调，保证位置更新

        # 切换OFFBOARD模式
        if not self.set_mode('OFFBOARD'):
            self.get_logger().error("Failed to set OFFBOARD mode")
            return False, "Failed to set OFFBOARD mode"
        self.get_logger().info("OFFBOARD mode set")

        # 解锁无人机 
        if not self.arm():
            self.get_logger().error("Arming failed")
            self.set_mode('POSCTL')  # 失败切回位置模式
            return False, "Arming failed"
        self.get_logger().info("Drone armed, taking off...")

        # ========== 修改：激活后台发布，更新全局目标点 ==========
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = altitude
        self.offboard_active = True

        # 等待到达目标高度（发布由定时器负责，此处只做闭环判断）
        while rclpy.ok():
            current_dist = self.get_distance_to_target(0.0, 0.0, altitude)
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9

            # 到达目标高度
            if current_dist < self.position_threshold:
                self.get_logger().info(f"Takeoff complete, reached altitude: {altitude}m")
                return True, f"Takeoff complete, reached altitude: {altitude}m"

            # 超时保护
            if elapsed > self.takeoff_timeout:
                self.get_logger().error("Takeoff timeout!")
                self.land()
                return False, "Takeoff timeout"

            rate.sleep()
            rclpy.spin_once(self, timeout_sec=0.001)

    def goto_xyz(self, x, y, z):
        """
        飞往指定坐标（阻塞式），闭环逻辑：
        1.更新全局目标点，后台定时器持续发布
        2.实时订阅当前位置计算距离
        3.距离小于阈值视为到达，正常返回
        4.超时未到达则触发降落保护
        --input_num_1=x, input_num_2=y, input_num_3=z
        """
        if self.current_pose is None or not self.offboard_active:
            self.get_logger().error("No position feedback or not in OFFBOARD, cannot execute goto")
            return False, "Invalid state for goto"

        self.get_logger().info(f"Flying to target: ({x}, {y}, {z})")
        rate = self.create_rate(self.control_rate)
        start_time = self.get_clock().now()

        # 更新全局目标点，定时器自动持续发布
        self.target_x = x
        self.target_y = y
        self.target_z = z

        while rclpy.ok():
            # 计算与目标点的距离
            current_dist = self.get_distance_to_target(x, y, z)
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9

            # 到达判断
            if current_dist < self.position_threshold:
                self.get_logger().info(f"Reached target ({x}, {y}, {z}), error: {current_dist:.2f}m")
                return True, f"Reached target ({x}, {y}, {z}), error: {current_dist:.2f}m"

            # 超时保护
            if elapsed > self.goto_timeout:
                self.get_logger().error(f"Goto timeout after {self.goto_timeout}s")
                self.land()
                return False, f"Goto timeout after {self.goto_timeout}s"
            rate.sleep()
            rclpy.spin_once(self, timeout_sec=0.001)

    def land(self):
        """切换自动降落模式，停止OFFBOARD设定点发布"""
        self.offboard_active = False  # 停止后台设定点发布
        if self.set_mode('AUTO.LAND'):
            self.get_logger().info("Switched to AUTO.LAND mode")
            return True, "Switched to AUTO.LAND mode"
        return False, "Failed to set land mode"

    # ========== 新增：悬停功能 ==========
    def hover(self):
        """
        悬停：锁定当前位置为目标点，持续发布实现悬停
        非阻塞，调用后立刻返回
        """
        if not self.offboard_active or self.current_pose is None:
            return False, "Not in OFFBOARD mode or no position data"
        
        # 将目标点设为当前实时位置
        self.target_x = self.current_pose.pose.position.x
        self.target_y = self.current_pose.pose.position.y
        self.target_z = self.current_pose.pose.position.z
        return True, "Hover mode activated"

    # ========== 新增：非阻塞设置目标点 ==========
    def set_target_nonblock(self, x, y, z):
        """
        非阻塞更新飞行目标，调用后立刻返回
        无人机后台自动飞向目标，到达判断由上层主控负责
        """
        if not self.offboard_active:
            return False, "Not in OFFBOARD mode"
        
        self.target_x = x
        self.target_y = y
        self.target_z = z
        return True, f"Target updated to ({x:.2f}, {y:.2f}, {z:.2f})"

# =========== SERVICE CALLBACK AREA ===========
    def control_callback(self, request, response):
        """
        command 约定：
        0 = 起飞          input_num_1 = 目标高度（阻塞）
        1 = 飞定点        input_num_1=x, input_num_2=y, input_num_3=z（阻塞）
        2 = 降落          （阻塞到触发降落指令）
        3 = 解锁
        4 = 上锁
        5 = 悬停          （非阻塞，立刻返回）
        6 = 设目标点      input_num_1=x, input_num_2=y, input_num_3=z（非阻塞，立刻返回）
        """
        cmd = request.command
        self.get_logger().info(f"[Command {cmd}] Received")
        if cmd == 0: # 起飞
            success, msg = self.takeoff(request.input_num_1)
        elif cmd == 1: # 飞往目标点（阻塞）
            success, msg = self.goto_xyz(
                request.input_num_1,
                request.input_num_2,
                request.input_num_3
            )
        elif cmd == 2: # 着陆
            success, msg = self.land()
        elif cmd == 3: # 解锁
            success = self.arm()
            msg = "Armed" if success else "Arming failed"
        elif cmd == 4: # 上锁
            success = self.disarm()
            msg = "Disarmed" if success else "Disarming failed"
        elif cmd == 5: # 悬停（非阻塞）
            success, msg = self.hover()
        elif cmd == 6: # 非阻塞设置目标点
            success, msg = self.set_target_nonblock(
                request.input_num_1,
                request.input_num_2,
                request.input_num_3
            )
        else:   
            success = False
            msg = f"Invalid command: {cmd}"

        response.success = success
        response.feedback = msg
        self.get_logger().info(f"[Command {cmd}] Result: {msg}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MavrosCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()