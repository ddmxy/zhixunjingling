#!/usr/bin/env python3
"""Phase-1: Nav2 to saved waypoint, then arm_look for box camera view.

START ORDER (base must be up first):
  1) bash ~/Desktop/run_navigation.sh          # chassis + lidar + Nav2
  2) 2D Pose Estimate in RViz (home)
  3) ros2 launch wheeltec_arm_bridge arm_serial_only.launch.py port:=/dev/ttyACM1
  4) python3 ~/Desktop/nav_arm_look_test.py

Optional: record points first with save_mission_points.py
"""
from __future__ import annotations

import math
import os
import subprocess
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

DEFAULT_WAYPOINTS = os.path.expanduser("~/mission_waypoints.yaml")
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def load_point(path: str, name: str) -> dict:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    pts = data.get("points", {})
    if name not in pts:
        raise KeyError(f"'{name}' not in {path}; record with save_mission_points.py")
    return pts[name]


def lifecycle_active(node: Node, srv: str) -> bool:
    cmd = ["ros2", "lifecycle", "get", srv]
    try:
        out = subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT, timeout=8)
        return "active [3]" in out
    except (subprocess.SubprocessError, FileNotFoundError) as exc:
        node.get_logger().warn(f"lifecycle check {srv}: {exc}")
        return False


class NavArmLookTest(Node):
    def __init__(self, wp_path: str, target: str, fold_before_nav: bool) -> None:
        super().__init__("nav_arm_look_test")
        self.wp_path = wp_path
        self.target = target
        self.fold_before_nav = fold_before_nav
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._odom_seen = False
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)

    def _on_odom(self, _msg: Odometry) -> None:
        self._odom_seen = True

    def wait_base(self, timeout: float = 60.0) -> None:
        self.get_logger().info("waiting for /odom (base_driver must be running)...")
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._odom_seen:
                self.get_logger().info("base OK (/odom publishing)")
                return
        raise RuntimeError("no /odom — launch navigation first (base_driver in launch)")

    def wait_nav2(self, timeout: float = 90.0) -> None:
        self.get_logger().info("waiting for bt_navigator active...")
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if lifecycle_active(self, "/bt_navigator"):
                self.get_logger().info("Nav2 OK (bt_navigator active)")
                return
            time.sleep(2.0)
        raise RuntimeError("bt_navigator not active — check Nav2 launch / 2D Pose Estimate")

    def wait_nav_action(self, timeout: float = 30.0) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("navigate_to_pose action not available")

    def navigate(self, point: dict) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = point.get("frame_id", "map")
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(point["x"])
        goal.pose.pose.position.y = float(point["y"])
        q = yaw_to_quat(float(point["yaw"]))
        goal.pose.pose.orientation.x, goal.pose.pose.orientation.y = q[0], q[1]
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = q[2], q[3]

        self.get_logger().info(
            f"Nav2 -> {self.target} x={point['x']:.3f} y={point['y']:.3f} "
            f"yaw={point.get('yaw_deg', math.degrees(point['yaw'])):.1f}°"
        )
        send = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("Nav2 goal rejected")

        result = handle.get_result_async()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            if result.done():
                break
        res = result.result()
        if res is None:
            raise RuntimeError("Nav2 result missing")
        if res.status != 4:  # SUCCEEDED
            raise RuntimeError(f"Nav2 failed status={res.status}")
        self.get_logger().info("Nav2 arrived")

    def run_arm_pose(self, pose: str) -> None:
        cmd = [sys.executable, os.path.join(SCRIPT_DIR, "arm_pose_cmd.py"), pose]
        self.get_logger().info(f"arm: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Nav to waypoint then arm_look")
    parser.add_argument("--waypoints", default=DEFAULT_WAYPOINTS)
    parser.add_argument("--point", default="box_approach", help="home | box_approach")
    parser.add_argument("--no-fold", action="store_true", help="skip arm_home before nav")
    parser.add_argument("--stay-home", action="store_true", help="after look, return arm_home")
    args = parser.parse_args()

    point = load_point(args.waypoints, args.point)

    rclpy.init()
    node = NavArmLookTest(args.waypoints, args.point, fold_before_nav=not args.no_fold)
    try:
        node.wait_base()
        node.wait_nav2()
        node.wait_nav_action()
        if not args.no_fold:
            node.run_arm_pose("arm_home")
        node.navigate(point)
        node.run_arm_pose("arm_look")
        if args.stay_home:
            node.run_arm_pose("arm_home")
        node.get_logger().info("Phase-1 done — check arm camera view for box edge")
    except Exception as exc:
        node.get_logger().error(str(exc))
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
