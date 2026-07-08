#!/usr/bin/env python3
"""Stationary drift test for odom / IMU / TF.

Purpose:
  Keep robot fully still, then quantify whether odom pose/yaw and IMU yaw-rate drift.
  This separates "sensor/model drift" from "mapping/localization issues".

Usage (RDK):
  # Terminal 1: bringup sensors + base_driver (no need to move)
  ros2 launch car_bringup mapping.launch.py use_rviz:=false

  # Terminal 2:
  source ~/Desktop/ros2_ws/install/setup.bash
  python3 ~/Desktop/stationary_odom_imu_drift_test.py --duration 120
"""
from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


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
class OdomSample:
    t: float
    x: float
    y: float
    yaw: float
    vx: float
    wz: float


class StationaryDriftNode(Node):
    def __init__(self, duration_s: float) -> None:
        super().__init__("stationary_odom_imu_drift_test")
        self.duration_s = duration_s
        self.start_mono = time.monotonic()
        self.odom_samples: List[OdomSample] = []
        self.imu_wz_samples: List[float] = []
        self.tf_xyyaw_samples: List[Tuple[float, float, float]] = []
        self.tf_fail = 0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, "/odom", self.on_odom, 100)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 100)
        self.timer = self.create_timer(0.1, self.tick_tf)

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.odom_samples.append(
            OdomSample(
                t=time.monotonic() - self.start_mono,
                x=float(p.x),
                y=float(p.y),
                yaw=float(yaw),
                vx=float(msg.twist.twist.linear.x),
                wz=float(msg.twist.twist.angular.z),
            )
        )

    def on_imu(self, msg: Imu) -> None:
        self.imu_wz_samples.append(float(msg.angular_velocity.z))

    def tick_tf(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            self.tf_xyyaw_samples.append((float(t.x), float(t.y), float(yaw_from_quat(q.x, q.y, q.z, q.w))))
        except TransformException:
            self.tf_fail += 1

    @staticmethod
    def stats(vals: List[float]) -> Tuple[float, float, float]:
        if not vals:
            return 0.0, 0.0, 0.0
        vmin = min(vals)
        vmax = max(vals)
        mean = sum(vals) / len(vals)
        return vmin, vmax, mean

    def report(self) -> str:
        lines: List[str] = []
        lines.append("=" * 62)
        lines.append("STATIONARY ODOM/IMU DRIFT REPORT")
        lines.append("=" * 62)
        lines.append(f"duration_s            : {time.monotonic() - self.start_mono:.1f}")
        lines.append(f"odom_samples          : {len(self.odom_samples)}")
        lines.append(f"imu_samples           : {len(self.imu_wz_samples)}")
        lines.append(f"tf_samples            : {len(self.tf_xyyaw_samples)} (fail={self.tf_fail})")
        lines.append("")

        if len(self.odom_samples) >= 2:
            first = self.odom_samples[0]
            last = self.odom_samples[-1]
            dx = last.x - first.x
            dy = last.y - first.y
            dxy = math.hypot(dx, dy)
            dyaw = wrap(last.yaw - first.yaw)
            dt = max(1e-6, last.t - first.t)

            vx_vals = [s.vx for s in self.odom_samples]
            wz_vals = [s.wz for s in self.odom_samples]
            vx_min, vx_max, vx_mean = self.stats(vx_vals)
            wz_min, wz_max, wz_mean = self.stats(wz_vals)

            lines.extend(
                [
                    "[ODOM drift while stationary]",
                    f"delta position        : {dxy:.4f} m (dx={dx:.4f}, dy={dy:.4f})",
                    f"delta yaw             : {math.degrees(dyaw):.2f} deg",
                    f"drift rate            : {dxy / dt * 60.0:.4f} m/min, {math.degrees(abs(dyaw)) / dt * 60.0:.2f} deg/min",
                    f"twist linear.x min/max/mean : {vx_min:.4f} / {vx_max:.4f} / {vx_mean:.4f} m/s",
                    f"twist angular.z min/max/mean: {wz_min:.4f} / {wz_max:.4f} / {wz_mean:.4f} rad/s",
                    "",
                ]
            )
        else:
            lines.append("Not enough /odom samples.")
            lines.append("")

        if self.imu_wz_samples:
            wz_min, wz_max, wz_mean = self.stats(self.imu_wz_samples)
            lines.extend(
                [
                    "[IMU angular velocity z while stationary]",
                    f"imu wz min/max/mean   : {wz_min:.4f} / {wz_max:.4f} / {wz_mean:.4f} rad/s",
                    "",
                ]
            )
        else:
            lines.append("No /imu/data samples.")
            lines.append("")

        if len(self.tf_xyyaw_samples) >= 2:
            x0, y0, yaw0 = self.tf_xyyaw_samples[0]
            x1, y1, yaw1 = self.tf_xyyaw_samples[-1]
            tf_dx = x1 - x0
            tf_dy = y1 - y0
            tf_dxy = math.hypot(tf_dx, tf_dy)
            tf_dyaw = wrap(yaw1 - yaw0)
            lines.extend(
                [
                    "[TF odom->base_footprint drift]",
                    f"tf delta position     : {tf_dxy:.4f} m (dx={tf_dx:.4f}, dy={tf_dy:.4f})",
                    f"tf delta yaw          : {math.degrees(tf_dyaw):.2f} deg",
                    "",
                ]
            )
        else:
            lines.append("Not enough TF samples for odom->base_footprint.")
            lines.append("")

        lines.append("Reference thresholds (stationary, 120s):")
        lines.append("- odom delta position <= 0.03 m (good)")
        lines.append("- odom delta yaw      <= 2.0 deg (good)")
        lines.append("- imu wz mean close to 0 (|mean| <= 0.01 rad/s preferable)")
        lines.append("")

        lines.append("If exceeded:")
        lines.append("- check IMU yaw bias / wz gain in MCU")
        lines.append("- check wheel velocity zero bias / slip")
        lines.append("- enable nonholonomic + deadband + feedback timeout in base_driver (already added)")
        lines.append("=" * 62)
        return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Stationary odom/imu drift test.")
    p.add_argument("--duration", type=float, default=120.0, help="Test duration in seconds.")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = StationaryDriftNode(args.duration)
    try:
        end_t = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < end_t:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        print(node.report(), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

