#!/usr/bin/env python3
"""Append a waypoint on every RViz 2D Pose Estimate or Nav2 Goal.

  2D Pose Estimate  ->  record kind=estimate
  Nav2 Goal         ->  record kind=goal  (from /plan endpoint)

Each click prints one line and appends to ~/mission_waypoints.yaml.
Use --fresh to clear old list and start from #1.

Usage:
  python3 ~/Desktop/save_mission_points.py
  python3 ~/Desktop/save_mission_points.py --fresh
"""
from __future__ import annotations

import math
import os
from datetime import datetime

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node

DEFAULT_PATH = os.path.expanduser("~/mission_waypoints.yaml")


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_deg(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


class MissionPointSaver(Node):
    def __init__(self, out_path: str, fresh: bool) -> None:
        super().__init__("save_mission_points")
        self.out_path = out_path
        self._last_plan_xyyaw: tuple[float, float, float] | None = None

        if fresh or not os.path.isfile(out_path):
            self.data: dict = {"frame_id": "map", "waypoints": []}
        else:
            with open(out_path, encoding="utf-8") as f:
                loaded = yaml.safe_load(f) or {}
            self.data = {
                "frame_id": loaded.get("frame_id", "map"),
                "waypoints": list(loaded.get("waypoints", [])),
            }

        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.on_estimate, 10
        )
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal_pose, 10)
        self.create_subscription(Path, "/plan", self.on_plan, 10)

        n = len(self.data["waypoints"])
        self.get_logger().info(f"Log -> {out_path}  ({n} existing)")
        self.get_logger().info("2D Pose Estimate -> +estimate | Nav2 Goal -> +goal")

    def _next_id(self) -> int:
        wps = self.data["waypoints"]
        return 1 if not wps else int(wps[-1]["id"]) + 1

    def _write(self) -> None:
        self.data["saved_at"] = datetime.now().isoformat(timespec="seconds")
        with open(self.out_path, "w", encoding="utf-8") as f:
            yaml.dump(self.data, f, allow_unicode=True, sort_keys=False)

    def _append(self, kind: str, x: float, y: float, yaw: float, frame: str, via: str) -> None:
        wid = self._next_id()
        entry = {
            "id": wid,
            "kind": kind,
            "x": round(x, 4),
            "y": round(y, 4),
            "yaw": round(yaw, 4),
            "yaw_deg": round(math.degrees(yaw), 2),
            "frame_id": frame,
            "via": via,
            "time": datetime.now().isoformat(timespec="seconds"),
        }
        self.data["waypoints"].append(entry)
        self._write()
        tag = "estimate" if kind == "estimate" else "goal"
        self.get_logger().info(
            f">>> #{wid:02d} [{tag}]  x={x:.3f}  y={y:.3f}  "
            f"yaw={math.degrees(yaw):+.1f}°  ({via})"
        )
        self.get_logger().info(f">>> total {len(self.data['waypoints'])} waypoints")

    def on_estimate(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        q = p.orientation
        self._append(
            "estimate",
            p.position.x,
            p.position.y,
            yaw_from_quat(q.x, q.y, q.z, q.w),
            msg.header.frame_id or "map",
            "2D Pose Estimate",
        )

    def on_goal_pose(self, msg: PoseStamped) -> None:
        p = msg.pose
        q = p.orientation
        self._append(
            "goal",
            p.position.x,
            p.position.y,
            yaw_from_quat(q.x, q.y, q.z, q.w),
            msg.header.frame_id or "map",
            "2D Goal Pose",
        )

    def on_plan(self, msg: Path) -> None:
        if len(msg.poses) < 2:
            return
        p = msg.poses[-1].pose
        q = p.orientation
        x, y = p.position.x, p.position.y
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        if self._last_plan_xyyaw is not None:
            lx, ly, lyaw = self._last_plan_xyyaw
            if math.hypot(x - lx, y - ly) < 0.15 and abs(wrap_deg(math.degrees(yaw - lyaw))) < 5.0:
                return

        self._last_plan_xyyaw = (x, y, yaw)
        self._append(
            "goal",
            x,
            y,
            yaw,
            msg.header.frame_id or "map",
            "Nav2 Goal",
        )


def main() -> None:
    import sys

    args = sys.argv[1:]
    fresh = "--fresh" in args
    out = DEFAULT_PATH
    for i, a in enumerate(args):
        if a == "--out" and i + 1 < len(args):
            out = args[i + 1]

    rclpy.init()
    node = MissionPointSaver(out, fresh=fresh)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
