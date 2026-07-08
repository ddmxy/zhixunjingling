import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

ALL_JOINTS = [
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5",
    "joint_6", "joint_10", "joint_7", "joint_11", "joint_8", "joint_9",
]

DEFAULT_POSITIONS = {
    "joint_1": 0.0, "joint_2": 0.0, "joint_3": 0.0,
    "joint_4": -1.57, "joint_5": 0.0, "joint_6": 0.0,
    "joint_7": 0.0, "joint_8": 0.0, "joint_9": 0.0,
    "joint_10": 0.0, "joint_11": 0.0,
}


def point_time_sec(point):
    return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9


class TrajectoryBridgeNode(Node):
    def __init__(self):
        super().__init__("trajectory_bridge_node")
        self.positions = dict(DEFAULT_POSITIONS)
        self.lock = threading.Lock()
        self.executing = False
        self.publish_rate = self.declare_parameter("publish_rate", 50.0).value

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)

        callback_group = ReentrantCallbackGroup()
        self.arm_action_server = ActionServer(
            self, FollowJointTrajectory,
            "arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_arm_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group,
        )
        self.hand_action_server = ActionServer(
            self, FollowJointTrajectory,
            "hand_controller/follow_joint_trajectory",
            execute_callback=self.execute_hand_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group,
        )
        self.get_logger().info("Trajectory bridge ready")

    def goal_callback(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def publish_joint_states(self):
        with self.lock:
            if self.executing:
                return
            self._publish_now()

    def _publish_now(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        msg.position = [self.positions[name] for name in ALL_JOINTS]
        self.joint_pub.publish(msg)

    def _run_trajectory(self, goal_handle, trajectory):
        joint_names = list(trajectory.joint_names)
        points = list(trajectory.points)
        if not joint_names or not points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        start_positions = {n: self.positions.get(n, 0.0) for n in joint_names}
        start_time = time.monotonic()
        point_index = 0

        while point_index < len(points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            now = time.monotonic() - start_time
            while point_index < len(points) and point_time_sec(points[point_index]) <= now:
                point_index += 1
            if point_index >= len(points):
                break

            target = points[point_index]
            target_time = max(point_time_sec(target), 1e-6)
            prev_time = point_time_sec(points[point_index - 1]) if point_index > 0 else 0.0
            alpha = min(max((now - prev_time) / max(target_time - prev_time, 1e-6), 0.0), 1.0)

            if point_index == 0:
                from_positions = start_positions
            else:
                from_positions = {n: points[point_index - 1].positions[i] for i, n in enumerate(joint_names)}

            with self.lock:
                for idx, name in enumerate(joint_names):
                    start_val = from_positions.get(name, 0.0)
                    end_val = target.positions[idx]
                    self.positions[name] = start_val + alpha * (end_val - start_val)
                self._publish_now()
            time.sleep(1.0 / self.publish_rate)

        with self.lock:
            final = points[-1]
            for idx, name in enumerate(joint_names):
                self.positions[name] = final.positions[idx]
            self._publish_now()

        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    async def execute_arm_callback(self, goal_handle):
        with self.lock:
            self.executing = True
        try:
            return self._run_trajectory(goal_handle, goal_handle.request.trajectory)
        finally:
            with self.lock:
                self.executing = False

    async def execute_hand_callback(self, goal_handle):
        with self.lock:
            self.executing = True
        try:
            return self._run_trajectory(goal_handle, goal_handle.request.trajectory)
        finally:
            with self.lock:
                self.executing = False


def main():
    rclpy.init()
    node = TrajectoryBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
