#!/usr/bin/env python3
"""Record goal vs AMCL heading error after Nav2 reaches a goal.

Nav2 does NOT publish /goal_pose — goals come from the global /plan path.

Usage (navigation running):
  source ~/Desktop/ros2_ws/install/setup.bash
  python3 ~/Desktop/nav_yaw_probe.py

Send Nav2 Goal in RViz, wait until the car stops. Each stop logs signed yaw error.
Press Enter to print summary + suggested laser_mount_yaw_rad correction.

Log file: ~/yaw_error_log.csv
"""
from __future__ import annotations

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node

DEFAULT_LOG = os.path.expanduser("~/yaw_error_log.csv")
CALIB_PATH = os.path.expanduser(
    "~/Desktop/ros2_ws/src/car_description/config/sensor_calibration.yaml"
)


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_deg(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def load_errors(path: str) -> list[float]:
    if not os.path.isfile(path):
        return []
    errs: list[float] = []
    with open(path, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                errs.append(float(row["error_deg"]))
            except (KeyError, ValueError):
                continue
    return errs


def print_summary(path: str, current_laser_yaw: float = 0.0) -> None:
    errs = load_errors(path)
    if not errs:
        print(f"No records in {path}")
        return
    mean = sum(errs) / len(errs)
    suggested = current_laser_yaw + math.radians(-mean)
    print(f"\n=== yaw error summary ({len(errs)} samples) ===")
    print(f"log: {path}")
    for i, e in enumerate(errs, 1):
        print(f"  #{i:02d}  error={e:+.1f} deg")
    print(f"mean error     : {mean:+.2f} deg  (+ = car stopped LEFT of goal heading)")
    print(f"current laser_mount_yaw_rad : {current_laser_yaw:+.4f} rad ({math.degrees(current_laser_yaw):+.1f} deg)")
    print(f"suggested laser_mount_yaw_rad: {suggested:+.4f} rad ({math.degrees(suggested):+.1f} deg)")
    print("\nApply: edit sensor_calibration.yaml -> laser_mount_yaw_rad, then:")
    print("  colcon build --packages-select car_description car_bringup")
    print("  restart navigation.launch.py")


def read_current_laser_yaw() -> float:
    if not os.path.isfile(CALIB_PATH):
        return 0.0
    try:
        import yaml

        with open(CALIB_PATH, encoding="utf-8") as f:
            return float((yaml.safe_load(f) or {}).get("laser_mount_yaw_rad", 0.0))
    except Exception:
        return 0.0


class NavYawProbe(Node):
    def __init__(self, log_path: str) -> None:
        super().__init__("nav_yaw_probe")
        self.log_path = log_path
        self.goal_yaw: float | None = None
        self.goal_set_t = 0.0
        self.last_amcl: PoseWithCovarianceStamped | None = None
        self.last_cmd_vel_t = time.monotonic()
        self.reported = False
        self.sample_n = len(load_errors(log_path))

        self._ensure_log_header()
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(3.0, self.heartbeat)
        self.create_timer(0.5, self.tick)

        self.get_logger().info(f"Logging to {log_path} ({self.sample_n} prior samples)")
        self.get_logger().info("Send Nav2 Goal in RViz — listening on /plan")

    def _ensure_log_header(self) -> None:
        if os.path.isfile(self.log_path):
            return
        os.makedirs(os.path.dirname(self.log_path) or ".", exist_ok=True)
        with open(self.log_path, "w", newline="", encoding="utf-8") as f:
            csv.DictWriter(
                f,
                fieldnames=["time", "goal_deg", "amcl_deg", "error_deg", "correction_rad"],
            ).writeheader()

    def _append_log(self, goal_deg: float, amcl_deg: float, err_deg: float) -> None:
        correction = math.radians(-err_deg)
        with open(self.log_path, "a", newline="", encoding="utf-8") as f:
            csv.DictWriter(
                f,
                fieldnames=["time", "goal_deg", "amcl_deg", "error_deg", "correction_rad"],
            ).writerow(
                {
                    "time": datetime.now().isoformat(timespec="seconds"),
                    "goal_deg": f"{goal_deg:.2f}",
                    "amcl_deg": f"{amcl_deg:.2f}",
                    "error_deg": f"{err_deg:.2f}",
                    "correction_rad": f"{correction:+.4f}",
                }
            )
        self.sample_n += 1

    def on_plan(self, msg: Path) -> None:
        if len(msg.poses) < 2:
            return
        q = msg.poses[-1].pose.orientation
        new_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        # /plan republishes every planner cycle — only latch when goal yaw actually changes
        if self.goal_yaw is not None:
            if abs(wrap_deg(math.degrees(new_yaw - self.goal_yaw))) < 3.0:
                return
        self.goal_yaw = new_yaw
        self.goal_set_t = time.monotonic()
        self.reported = False
        self.get_logger().info(
            f"New goal yaw={math.degrees(self.goal_yaw):+.1f} deg"
        )

    def on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.last_amcl = msg

    def on_cmd(self, msg: Twist) -> None:
        if abs(msg.linear.x) > 0.02 or abs(msg.angular.z) > 0.03:
            self.last_cmd_vel_t = time.monotonic()

    def heartbeat(self) -> None:
        if self.last_amcl is None:
            self.get_logger().warn("No /amcl_pose yet — did you 2D Pose Estimate?")
            return
        if self.goal_yaw is None or self.reported:
            q = self.last_amcl.pose.pose.orientation
            yaw = math.degrees(yaw_from_quat(q.x, q.y, q.z, q.w))
            self.get_logger().info(f"amcl_yaw={yaw:+.1f} deg  (waiting for Nav2 goal...)")

    def tick(self) -> None:
        if self.reported or self.goal_yaw is None or self.last_amcl is None:
            return
        if time.monotonic() - self.goal_set_t < 3.0:
            return
        if time.monotonic() - self.last_cmd_vel_t < 1.5:
            return

        q = self.last_amcl.pose.pose.orientation
        amcl_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        goal_deg = math.degrees(self.goal_yaw)
        amcl_deg = math.degrees(amcl_yaw)
        err_deg = wrap_deg(goal_deg - amcl_deg)
        correction = math.radians(-err_deg)

        self._append_log(goal_deg, amcl_deg, err_deg)
        self.get_logger().info(
            f">>> RECORDED #{self.sample_n} | goal={goal_deg:+.1f} deg  "
            f"amcl={amcl_deg:+.1f} deg  error={err_deg:+.1f} deg  "
            f"laser_mount_yaw_rad += {correction:+.3f}"
        )
        self.reported = True


def main() -> None:
    import sys
    import threading

    if "--summary" in sys.argv:
        print_summary(DEFAULT_LOG, read_current_laser_yaw())
        return

    log_path = DEFAULT_LOG
    for i, arg in enumerate(sys.argv[1:], 1):
        if arg == "--log" and i < len(sys.argv) - 1:
            log_path = sys.argv[i + 1]

    rclpy.init()
    node = NavYawProbe(log_path)

    def input_thread() -> None:
        while rclpy.ok():
            try:
                input()
                print_summary(log_path, read_current_laser_yaw())
            except EOFError:
                break

    threading.Thread(target=input_thread, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print_summary(log_path, read_current_laser_yaw())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
