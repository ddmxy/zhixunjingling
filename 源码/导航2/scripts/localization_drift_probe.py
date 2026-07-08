#!/usr/bin/env python3
"""Quantify localization drift growth during navigation.

Focus: whether map->odom correction keeps growing (symptom of odom drift).

Usage:
  source ~/Desktop/ros2_ws/install/setup.bash
  python3 ~/Desktop/localization_drift_probe.py --duration 120
"""
from __future__ import annotations

import argparse
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
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
class Sample:
    t: float
    x: float
    y: float
    yaw: float


class DriftProbe(Node):
    def __init__(self, duration_s: float) -> None:
        super().__init__("localization_drift_probe")
        self.duration_s = duration_s
        self.start = time.monotonic()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom: Optional[Odometry] = None
        self.samples: Deque[Sample] = deque(maxlen=4000)
        self.tf_fail = 0

        self.create_subscription(Odometry, "/odom", self.on_odom, 100)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def tick(self) -> None:
        if time.monotonic() - self.start > self.duration_s:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "odom", rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException:
            self.tf_fail += 1
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        self.samples.append(
            Sample(time.monotonic() - self.start, float(t.x), float(t.y), yaw_from_quat(q.x, q.y, q.z, q.w))
        )

    def summarize(self) -> str:
        if len(self.samples) < 10:
            return "No enough map->odom samples. Is AMCL running and active?"
        first = self.samples[0]
        last = self.samples[-1]
        dx = last.x - first.x
        dy = last.y - first.y
        dpos = math.hypot(dx, dy)
        dyaw = wrap(last.yaw - first.yaw)
        dt = max(1e-6, last.t - first.t)

        # robust "short term jump" metric over 1 second window
        jump_pos = 0.0
        jump_yaw = 0.0
        for i in range(len(self.samples)):
            j = i
            while j + 1 < len(self.samples) and (self.samples[j + 1].t - self.samples[i].t) < 1.0:
                j += 1
            if j == i:
                continue
            ddx = self.samples[j].x - self.samples[i].x
            ddy = self.samples[j].y - self.samples[i].y
            ddp = math.hypot(ddx, ddy)
            ddyaw = abs(wrap(self.samples[j].yaw - self.samples[i].yaw))
            jump_pos = max(jump_pos, ddp)
            jump_yaw = max(jump_yaw, ddyaw)

        lines = [
            "=" * 56,
            "LOCALIZATION DRIFT PROBE (map->odom)",
            "=" * 56,
            f"duration_s               : {dt:.1f}",
            f"tf_lookup_fail_count     : {self.tf_fail}",
            f"map->odom drift total    : {dpos:.3f} m, {math.degrees(dyaw):.2f} deg",
            f"map->odom drift rate     : {dpos / dt * 60.0:.3f} m/min, {math.degrees(abs(dyaw)) / dt * 60.0:.2f} deg/min",
            f"max 1s correction jump   : {jump_pos:.3f} m, {math.degrees(jump_yaw):.2f} deg",
            "",
            "Interpretation:",
        ]
        if dpos / dt > 0.005 or abs(dyaw) / dt > math.radians(0.5):
            lines.append("- High drift rate: odom drift is significant; AMCL keeps compensating.")
        else:
            lines.append("- Drift rate is modest: localization backbone is stable.")
        if jump_pos > 0.15 or jump_yaw > math.radians(8.0):
            lines.append("- Large jump(s): likely relocalization events or unstable scan matching.")
        else:
            lines.append("- No large correction jumps.")

        if self.odom is not None:
            vx = self.odom.twist.twist.linear.x
            wz = self.odom.twist.twist.angular.z
            lines.append(f"- Last odom twist: vx={vx:.3f} m/s, wz={wz:.3f} rad/s")
        lines.append("=" * 56)
        return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=120.0, help="Probe duration in seconds.")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = DriftProbe(args.duration)
    try:
        end_t = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < end_t:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        print(node.summarize(), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

