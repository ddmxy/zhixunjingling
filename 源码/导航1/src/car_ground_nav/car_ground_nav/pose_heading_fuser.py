"""
Fuse UWB XY position with chassis heading (yaw) to produce PoseStamped with orientation.

This keeps the navigation stack happy: position from UWB, yaw from IMU/heading.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


class PoseHeadingFuser(Node):
    def __init__(self) -> None:
        super().__init__("pose_heading_fuser")

        self.declare_parameter("pose_in_topic", "uwb_pose_raw")
        self.declare_parameter("heading_topic", "chassis/heading_rad")
        self.declare_parameter("pose_out_topic", "uwb_pose_fused")
        self.declare_parameter("yaw_offset_rad", 0.0)

        pose_in = self.get_parameter("pose_in_topic").get_parameter_value().string_value
        heading_topic = self.get_parameter("heading_topic").get_parameter_value().string_value
        pose_out = self.get_parameter("pose_out_topic").get_parameter_value().string_value
        self._yaw_offset = self.get_parameter("yaw_offset_rad").get_parameter_value().double_value

        self._yaw: Optional[float] = None
        self._pub = self.create_publisher(PoseStamped, pose_out, 50)
        self.create_subscription(PoseStamped, pose_in, self._on_pose, 50)
        self.create_subscription(Float32, heading_topic, self._on_heading, 10)

        self.get_logger().info(f"pose_heading_fuser: {pose_in} + {heading_topic} -> {pose_out}")

    def _on_heading(self, msg: Float32) -> None:
        yaw = float(msg.data) + float(self._yaw_offset)
        if not (yaw == yaw):  # NaN
            return
        self._yaw = yaw

    def _on_pose(self, msg: PoseStamped) -> None:
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose

        if self._yaw is not None:
            yaw = (self._yaw + math.pi) % (2.0 * math.pi) - math.pi
            half = 0.5 * yaw
            out.pose.orientation.x = 0.0
            out.pose.orientation.y = 0.0
            out.pose.orientation.z = math.sin(half)
            out.pose.orientation.w = math.cos(half)
        else:
            out.pose.orientation.x = 0.0
            out.pose.orientation.y = 0.0
            out.pose.orientation.z = 0.0
            out.pose.orientation.w = 1.0

        self._pub.publish(out)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PoseHeadingFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
