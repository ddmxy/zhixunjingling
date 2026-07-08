#!/usr/bin/env python3
"""One-shot sensor + TF health check (~15 s).

Catches the silent problems that degrade SLAM/nav over time:
  - /scan, /odom, /imu/data rates and timestamp age (clock skew / latency)
  - scan angular span + valid-range ratio (reflective/soft walls)
  - TF chain map->odom->base_footprint->laser existence
  - sign/magnitude consistency between odom.twist.wz and imu wz

Usage (RDK, with bringup running):
  source ~/Desktop/ros2_ws/install/setup.bash
  python3 ~/Desktop/sensor_health_check.py --duration 15
"""
from __future__ import annotations

import argparse
import math
import time
from typing import List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class HealthCheck(Node):
    def __init__(self, duration_s: float) -> None:
        super().__init__("sensor_health_check")
        self.duration_s = duration_s
        self.start = time.monotonic()

        self.scan_recv: List[float] = []
        self.scan_age: List[float] = []
        self.scan_valid_ratio: List[float] = []
        self.scan_span: Optional[float] = None
        self.scan_n: Optional[int] = None

        self.odom_recv: List[float] = []
        self.odom_age: List[float] = []
        self.odom_vx: List[float] = []
        self.odom_wz: List[float] = []

        self.imu_recv: List[float] = []
        self.imu_wz: List[float] = []

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_ob_ok = 0
        self.tf_ob_fail = 0

        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 100)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 100)
        self.create_timer(0.1, self.tick_tf)

    def _age(self, stamp) -> float:
        now = self.get_clock().now().nanoseconds * 1e-9
        s = stamp.sec + stamp.nanosec * 1e-9
        return now - s

    def on_scan(self, msg: LaserScan) -> None:
        self.scan_recv.append(time.monotonic())
        self.scan_age.append(self._age(msg.header.stamp))
        if self.scan_span is None:
            self.scan_span = msg.angle_max - msg.angle_min
            self.scan_n = len(msg.ranges)
        if msg.ranges:
            valid = sum(
                1 for r in msg.ranges
                if math.isfinite(r) and msg.range_min < r < msg.range_max
            )
            self.scan_valid_ratio.append(valid / len(msg.ranges))

    def on_odom(self, msg: Odometry) -> None:
        self.odom_recv.append(time.monotonic())
        self.odom_age.append(self._age(msg.header.stamp))
        self.odom_vx.append(float(msg.twist.twist.linear.x))
        self.odom_wz.append(float(msg.twist.twist.angular.z))

    def on_imu(self, msg: Imu) -> None:
        self.imu_recv.append(time.monotonic())
        self.imu_wz.append(float(msg.angular_velocity.z))

    def tick_tf(self) -> None:
        try:
            self.tf_buffer.lookup_transform(
                "odom", "base_footprint", rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
            self.tf_ob_ok += 1
        except TransformException:
            self.tf_ob_fail += 1

    @staticmethod
    def _rate(stamps: List[float]) -> float:
        if len(stamps) < 2:
            return 0.0
        return (len(stamps) - 1) / max(1e-6, stamps[-1] - stamps[0])

    @staticmethod
    def _avg(v: List[float]) -> float:
        return sum(v) / len(v) if v else 0.0

    def _tf_chain(self) -> List[str]:
        checks = [("map", "odom"), ("odom", "base_footprint"),
                  ("base_footprint", "laser"), ("map", "laser")]
        out = []
        for tgt, src in checks:
            try:
                self.tf_buffer.lookup_transform(tgt, src, rclpy.time.Time(), timeout=Duration(seconds=0.1))
                out.append(f"  {tgt} <- {src}: OK")
            except TransformException as e:
                out.append(f"  {tgt} <- {src}: FAIL ({str(e).splitlines()[0][:70]})")
        return out

    def report(self) -> str:
        L = ["=" * 60, "SENSOR / TF HEALTH CHECK", "=" * 60]
        L.append(f"duration_s        : {time.monotonic() - self.start:.1f}")
        L.append("")
        L.append("[Rates]")
        L.append(f"  /scan     : {self._rate(self.scan_recv):6.2f} Hz  (n={len(self.scan_recv)})  [expect ~10]")
        L.append(f"  /odom     : {self._rate(self.odom_recv):6.2f} Hz  (n={len(self.odom_recv)})  [expect ~50]")
        L.append(f"  /imu/data : {self._rate(self.imu_recv):6.2f} Hz  (n={len(self.imu_recv)})  [expect ~50]")
        L.append(f"  tf o->b   : ok={self.tf_ob_ok} fail={self.tf_ob_fail}")
        L.append("")
        L.append("[Timestamp age = now - header.stamp]  (small & stable is good)")
        if self.scan_age:
            L.append(f"  /scan age : avg {self._avg(self.scan_age)*1000:6.1f} ms  "
                     f"(min {min(self.scan_age)*1000:.1f}, max {max(self.scan_age)*1000:.1f})")
        else:
            L.append("  /scan age : no data")
        if self.odom_age:
            L.append(f"  /odom age : avg {self._avg(self.odom_age)*1000:6.1f} ms  "
                     f"(min {min(self.odom_age)*1000:.1f}, max {max(self.odom_age)*1000:.1f})")
        else:
            L.append("  /odom age : no data")
        L.append("")
        L.append("[Scan geometry]")
        if self.scan_span is not None:
            L.append(f"  angular span : {math.degrees(self.scan_span):.1f} deg over {self.scan_n} beams")
            L.append(f"  valid ratio  : {self._avg(self.scan_valid_ratio)*100:.1f}%  "
                     f"[low => reflective/soft walls or range too large]")
        else:
            L.append("  no scan received")
        L.append("")
        L.append("[Motion signs]  (turn the car during test to check)")
        L.append(f"  odom vx avg : {self._avg(self.odom_vx):+.4f} m/s")
        L.append(f"  odom wz avg : {self._avg(self.odom_wz):+.4f} rad/s")
        L.append(f"  imu  wz avg : {self._avg(self.imu_wz):+.4f} rad/s")
        L.append("  hint: odom wz and imu wz must share SIGN & similar magnitude when turning.")
        L.append("")
        L.append("[TF chain]")
        L.extend(self._tf_chain())
        L.append("")
        L.append("[Flags]")
        flagged = False
        if self.scan_age and self._avg(self.scan_age) > 0.15:
            L.append("  WARN: /scan stamp age large -> SLAM/AMCL TF lookups may fail."); flagged = True
        if self.odom_age and self._avg(self.odom_age) > 0.10:
            L.append("  WARN: /odom stamp age large -> check base_driver clock."); flagged = True
        if self.scan_valid_ratio and self._avg(self.scan_valid_ratio) < 0.5:
            L.append("  WARN: low valid scan ratio -> reflective/soft env or range too big."); flagged = True
        if self._rate(self.odom_recv) < 30.0:
            L.append("  WARN: /odom slower than expected 50 Hz."); flagged = True
        if self._rate(self.scan_recv) < 7.0:
            L.append("  WARN: /scan slower than expected 10 Hz."); flagged = True
        if not flagged:
            L.append("  none")
        L.append("=" * 60)
        return "\n".join(L)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=15.0)
    args = p.parse_args()
    rclpy.init()
    node = HealthCheck(args.duration)
    try:
        end = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < end:
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
