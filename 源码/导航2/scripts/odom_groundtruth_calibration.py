#!/usr/bin/env python3
"""Ground-truth odom calibration: compare odom against a tape measure / floor marks.

Why: imu_wz_step_test only checks IMU against ITSELF (integrate wz vs nav_yaw).
If the gyro scale is wrong, both are wrong equally and the ratio still looks ~1.0.
This tool exposes the TRUE scale error by comparing odom to the physical world.

It accumulates from /odom:
  - path_len   : sum of |position deltas| (drive distance)
  - straight   : straight-line displacement start->current
  - yaw_unwrap : unwrapped heading change (handles multi-turn, e.g. 360 deg)
and integrates /imu wz as an independent yaw estimate.

Procedure
  LINEAR scale:
    1) Mark start. Run this script.
    2) Push/teleop the car straight for a measured distance (e.g. 2.000 m).
    3) Stop, Ctrl+C. Compare straight vs your tape measure.
       wheel_scale = physical / odom_straight   (apply in MCU wheel calc)

  YAW scale (most important for turning drift):
    1) Mark heading. Run this script.
    2) Rotate the car in place exactly 360 deg (or 720 for accuracy).
    3) Stop, Ctrl+C. Compare yaw_unwrap (odom) and imu_yaw_unwrap.
       gyro_scale = physical_deg / odom_yaw_deg  (apply to IMU gyro scale in MCU)

Usage (RDK, bringup running):
  source ~/Desktop/ros2_ws/install/setup.bash
  python3 ~/Desktop/odom_groundtruth_calibration.py
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


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


class Calib(Node):
    def __init__(self) -> None:
        super().__init__("odom_groundtruth_calibration")
        self.x0: Optional[float] = None
        self.y0: Optional[float] = None
        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.last_yaw: Optional[float] = None
        self.path_len = 0.0
        self.yaw_unwrap = 0.0
        self.cur_x = 0.0
        self.cur_y = 0.0

        self.imu_yaw_unwrap = 0.0
        self.imu_last_t: Optional[float] = None

        self.create_subscription(Odometry, "/odom", self.on_odom, 100)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 100)
        self.create_timer(1.0, self.live)
        self.start = time.monotonic()

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        if self.x0 is None:
            self.x0, self.y0 = p.x, p.y
            self.last_x, self.last_y, self.last_yaw = p.x, p.y, yaw
            return
        self.path_len += math.hypot(p.x - self.last_x, p.y - self.last_y)
        self.yaw_unwrap += wrap(yaw - self.last_yaw)
        self.last_x, self.last_y, self.last_yaw = p.x, p.y, yaw
        self.cur_x, self.cur_y = p.x, p.y

    def on_imu(self, msg: Imu) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.imu_last_t is not None:
            dt = t - self.imu_last_t
            if 0.0 < dt < 0.5:
                self.imu_yaw_unwrap += msg.angular_velocity.z * dt
        self.imu_last_t = t

    def straight(self) -> float:
        if self.x0 is None:
            return 0.0
        return math.hypot(self.cur_x - self.x0, self.cur_y - self.y0)

    def live(self) -> None:
        print(
            f"[{time.monotonic()-self.start:5.1f}s] "
            f"path={self.path_len:6.3f} m  straight={self.straight():6.3f} m  "
            f"odom_yaw={math.degrees(self.yaw_unwrap):+7.1f} deg  "
            f"imu_yaw={math.degrees(self.imu_yaw_unwrap):+7.1f} deg",
            flush=True,
        )

    def report(self) -> str:
        L = ["", "=" * 60, "GROUND-TRUTH CALIBRATION RESULT", "=" * 60]
        L.append(f"odom path length     : {self.path_len:.3f} m")
        L.append(f"odom straight disp.  : {self.straight():.3f} m")
        L.append(f"odom yaw (unwrapped) : {math.degrees(self.yaw_unwrap):+.2f} deg")
        L.append(f"imu  yaw (integ wz)  : {math.degrees(self.imu_yaw_unwrap):+.2f} deg")
        L.append("")
        L.append("Now compare to the physical world you measured:")
        L.append("  LINEAR: wheel_scale = physical_meters / odom_straight")
        L.append("  YAW   : gyro_scale  = physical_degrees / odom_yaw_deg")
        L.append("")
        L.append("Guidance:")
        L.append("  - If odom_yaw and imu_yaw agree but BOTH differ from physical,")
        L.append("    the gyro SCALE is wrong -> fix IMU gyro scale in MCU firmware.")
        L.append("  - If odom_straight differs from tape measure,")
        L.append("    the wheel radius / ticks-per-rev is wrong -> fix in MCU.")
        L.append("  - A 10% yaw scale error alone can ruin nav after a few turns.")
        L.append("=" * 60)
        return "\n".join(L)


def main() -> int:
    argparse.ArgumentParser().parse_args()
    rclpy.init()
    node = Calib()
    print("Recording... drive a MEASURED distance or rotate a KNOWN angle, then Ctrl+C.")
    interrupted = False
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except KeyboardInterrupt:
                interrupted = True
                break
    except KeyboardInterrupt:
        interrupted = True
    finally:
        print(node.report(), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if interrupted else 0


if __name__ == "__main__":
    raise SystemExit(main())
