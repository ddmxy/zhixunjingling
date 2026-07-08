"""Forward PoseStamped goals (e.g. from LoRa) to Nav2 NavigateToPose action."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class Nav2GoalBridge(Node):
    def __init__(self) -> None:
        super().__init__("nav2_goal_bridge")

        self.declare_parameter("goal_topic", "uav_goal")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("cancel_previous", True)
        self.declare_parameter("min_goal_interval_sec", 0.5)
        self.declare_parameter("max_nav_goals", 3)

        goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self._cancel_previous = self.get_parameter("cancel_previous").get_parameter_value().bool_value
        self._min_interval = self.get_parameter("min_goal_interval_sec").get_parameter_value().double_value
        self._max_nav_goals = self.get_parameter("max_nav_goals").get_parameter_value().integer_value
        if self._max_nav_goals <= 0:
            self._max_nav_goals = 3

        self._client = ActionClient(self, NavigateToPose, action_name)
        self._goal_handle = None
        self._last_send_time: Optional[rclpy.time.Time] = None
        self._sent_goals = 0

        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
        self.get_logger().info(
            f"nav2_goal_bridge: {goal_topic} -> NavigateToPose({action_name}), max_nav_goals={self._max_nav_goals}"
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        if self._sent_goals >= self._max_nav_goals:
            return

        now = self.get_clock().now()
        if self._last_send_time is not None:
            dt = (now - self._last_send_time).nanoseconds * 1e-9
            if dt < self._min_interval:
                return
        self._last_send_time = now

        if not self._client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warning("NavigateToPose action server not available; skipping goal")
            return

        if self._cancel_previous and self._goal_handle is not None:
            _ = self._client.cancel_goal_async(self._goal_handle)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_sent)

    def _on_goal_sent(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("NavigateToPose goal rejected")
            return
        self._goal_handle = goal_handle
        self._sent_goals += 1
        self.get_logger().info(f"NavigateToPose goal accepted ({self._sent_goals}/{self._max_nav_goals})")


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = Nav2GoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
