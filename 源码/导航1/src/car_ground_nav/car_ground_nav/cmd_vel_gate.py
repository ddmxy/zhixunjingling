"""
Gate velocity commands: keep the robot stopped until a goal is received.

Typical wiring:
- Nav2 publishes Twist on /cmd_vel
- This node subscribes /cmd_vel and republishes to /cmd_vel_gated
- chassis_serial_node subscribes /cmd_vel_gated and sends to MCU

Enable condition:
- receive any PoseStamped on /uav_goal (LoRa goal receiver output)
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist


class CmdVelGate(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_gate")

        self.declare_parameter("in_topic", "cmd_vel")
        self.declare_parameter("out_topic", "cmd_vel_gated")
        self.declare_parameter("enable_topic", "uav_goal")
        self.declare_parameter("enabled_on_start", False)
        self.declare_parameter("publish_zero_when_disabled", True)
        self.declare_parameter("zero_publish_rate_hz", 10.0)
        self.declare_parameter("max_v_m_s", 0.5)
        self.declare_parameter("max_w_rad_s", 1.0)

        in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value
        enable_topic = self.get_parameter("enable_topic").get_parameter_value().string_value

        self._enabled = bool(self.get_parameter("enabled_on_start").get_parameter_value().bool_value)
        self._pub_zero = bool(self.get_parameter("publish_zero_when_disabled").get_parameter_value().bool_value)
        rate = float(self.get_parameter("zero_publish_rate_hz").get_parameter_value().double_value)
        if rate <= 0.0:
            rate = 10.0

        self._max_v = float(self.get_parameter("max_v_m_s").get_parameter_value().double_value)
        self._max_w = float(self.get_parameter("max_w_rad_s").get_parameter_value().double_value)
        if self._max_v <= 0.0:
            self._max_v = 0.5
        if self._max_w <= 0.0:
            self._max_w = 1.0

        self._pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._on_cmd, 10)
        self.create_subscription(PoseStamped, enable_topic, self._on_goal, 10)

        if self._pub_zero:
            self.create_timer(1.0 / rate, self._tick_zero)

        self.get_logger().info(
            f"cmd_vel_gate: {in_topic} -> {out_topic} | enable on {enable_topic} | enabled={self._enabled} | "
            f"limits: v<= {self._max_v:.3f} m/s, w<= {self._max_w:.3f} rad/s"
        )

    @staticmethod
    def _clamp(x: float, lo: float, hi: float) -> float:
        if x < lo:
            return lo
        if x > hi:
            return hi
        return x

    def _on_goal(self, _: PoseStamped) -> None:
        if not self._enabled:
            self._enabled = True
            self.get_logger().info("cmd_vel_gate enabled by goal")

    def _on_cmd(self, msg: Twist) -> None:
        if self._enabled:
            out = Twist()
            v = float(msg.linear.x)
            w = float(msg.angular.z)
            out.linear.x = self._clamp(v, -self._max_v, self._max_v)
            out.angular.z = self._clamp(w, -self._max_w, self._max_w)
            self._pub.publish(out)
        elif self._pub_zero:
            self._pub.publish(Twist())

    def _tick_zero(self) -> None:
        if not self._enabled and self._pub_zero:
            self._pub.publish(Twist())


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

