#!/usr/bin/env python3
"""Segment stop drift test — does error grow after each stop?

Use case: "I drive a bit, stop, and alignment error gets worse."

Procedure:
  1) Mark start line on floor (tape). Align car heading with tape.
  2) Run this script + mapping_teleop in another terminal.
  3) Drive straight slowly to 0.5m, 1.0m, 1.5m, 2.0m marks; press k to stop at each.
  4) In THIS terminal, press Enter at each stop and type tape distance (e.g. 0.5).
  5) Script records odom straight / yaw drift vs your ground truth each time.

Usage:
  source ~/Desktop/ros2_ws/install/setup.bash
  python3 ~/Desktop/segment_stop_drift_test.py
"""
from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


@dataclass
class Checkpoint:
    physical_m: float
    odom_straight_m: float
    odom_yaw_deg: float
    err_linear_m: float
    err_yaw_deg: float


class SegmentDriftNode(Node):
    def __init__(self) -> None:
        super().__init__("segment_stop_drift_test")
        self.x0: Optional[float] = None
        self.y0: Optional[float] = None
        self.yaw0: Optional[float] = None
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        self.checkpoints: List[Checkpoint] = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 100)

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        if self.x0 is None:
            self.x0, self.y0, self.yaw0 = p.x, p.y, yaw
        self.cur_x, self.cur_y, self.cur_yaw = p.x, p.y, yaw

    def straight(self) -> float:
        if self.x0 is None:
            return 0.0
        return math.hypot(self.cur_x - self.x0, self.cur_y - self.y0)

    def yaw_deg(self) -> float:
        if self.yaw0 is None:
            return 0.0
        return math.degrees(wrap(self.cur_yaw - self.yaw0))

    def snapshot(self, physical_m: float) -> Checkpoint:
        odom_s = self.straight()
        odom_y = self.yaw_deg()
        cp = Checkpoint(
            physical_m=physical_m,
            odom_straight_m=odom_s,
            odom_yaw_deg=odom_y,
            err_linear_m=physical_m - odom_s,
            err_yaw_deg=-odom_y,
        )
        self.checkpoints.append(cp)
        return cp


def spin_bg(node: SegmentDriftNode) -> None:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> int:
    rclpy.init()
    node = SegmentDriftNode()
    t = threading.Thread(target=spin_bg, args=(node,), daemon=True)
    t.start()

    print("=" * 60)
    print("SEGMENT STOP DRIFT TEST")
    print("=" * 60)
    print("1) Align car at start tape. 2) Teleop: i forward, k stop at each mark.")
    print("3) Here: press Enter at EACH stop, then type tape distance (m).")
    print("   Example stops: 0.5  1.0  1.5  2.0")
    print("4) Empty line + Enter when done.")
    print("=" * 60)

    time.sleep(1.0)
    if node.x0 is None:
        print("ERROR: no /odom yet. Is base_driver running?")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    while True:
        try:
            line = input("\nAt STOP: Enter tape distance (m), or blank to finish: ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            break
        try:
            physical = float(line)
        except ValueError:
            print("  invalid number, try again")
            continue
        cp = node.snapshot(physical)
        print(
            f"  physical={cp.physical_m:.3f} m  odom_straight={cp.odom_straight_m:.3f} m  "
            f"linear_err={cp.err_linear_m:+.3f} m  yaw_drift={cp.odom_yaw_deg:+.2f} deg"
        )

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    if not node.checkpoints:
        print("No checkpoints recorded.")
    else:
        print(f"{'phys':>6} {'odom':>6} {'lin_err':>8} {'yaw_deg':>8}")
        for cp in node.checkpoints:
            print(
                f"{cp.physical_m:6.3f} {cp.odom_straight_m:6.3f} "
                f"{cp.err_linear_m:+8.3f} {cp.odom_yaw_deg:+8.2f}"
            )
        print("")
        if len(node.checkpoints) >= 2:
            first = node.checkpoints[0]
            last = node.checkpoints[-1]
            lin_growth = abs(last.err_linear_m) - abs(first.err_linear_m)
            yaw_growth = abs(last.odom_yaw_deg) - abs(first.odom_yaw_deg)
            print(f"linear |error| growth (first->last): {lin_growth:+.3f} m")
            print(f"yaw |drift| growth (first->last)    : {yaw_growth:+.2f} deg")
            if yaw_growth > 2.0:
                print("=> yaw drift ACCUMULATES with distance (heading problem).")
            elif abs(lin_growth) < 0.02:
                print("=> linear error stable (wheel scale OK).")
    print("=" * 60)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
