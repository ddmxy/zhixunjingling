"""
Convert heading (yaw, rad) Float32 to sensor_msgs/Imu orientation.

Input:
- std_msgs/Float32 on topic chassis/heading_rad (configurable)

Output:
- sensor_msgs/Imu on topic imu/data (configurable)
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class HeadingImuPublisher(Node):
    def __init__(self) -> None:
        super().__init__("heading_imu_publisher")

        self.declare_parameter("heading_topic", "chassis/heading_rad")
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("yaw_offset_rad", 0.0)

        heading_topic = self.get_parameter("heading_topic").get_parameter_value().string_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._yaw_offset = self.get_parameter("yaw_offset_rad").get_parameter_value().double_value

        self._pub = self.create_publisher(Imu, imu_topic, 10)
        self.create_subscription(Float32, heading_topic, self._on_heading, 10)

        self.get_logger().info(f"heading_imu_publisher: {heading_topic} -> {imu_topic} (frame_id={self._frame_id})")

    def _on_heading(self, msg: Float32) -> None:
        yaw = float(msg.data) + float(self._yaw_offset)
        if not (yaw == yaw):  # NaN
            return

        # Normalize to [-pi, pi] for neat logging/consumers (not strictly required).
        yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi
        half = 0.5 * yaw

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self._frame_id

        # Quaternion for yaw-only rotation.
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = math.sin(half)
        imu.orientation.w = math.cos(half)

        # Unknown angular velocity / linear acceleration.
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0

        self._pub.publish(imu)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = HeadingImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
