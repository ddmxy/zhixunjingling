#!/usr/bin/env python3
"""Check whether wheel+IMU odometry agrees with lidar scans (no SLAM required).

What it does
  1. Verifies /scan, /odom rates and TF (odom -> laser) at each scan stamp.
  2. Between consecutive scans, uses TF motion (or /odom fallback) to predict
     the next scan from the previous one in a static environment.
  3. Reports RMSE / inlier ratio overall and for straight vs rotate segments.

Prerequisites (on RDK, separate from this script):
  - base_driver_node  (/odom + TF odom->base_footprint)
  - lslidar_driver    (/scan)
  - robot_state_publisher (TF base_footprint->laser)

Usage:
  # Terminal 1 — sensors + odom only (no SLAM needed):
  ros2 launch car_bringup mapping.launch.py use_rviz:=false
  # or any bringup that starts base_driver + lidar + robot_state_publisher

  # Terminal 2 — teleop while testing:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

  # Terminal 3 — run this script (~60 s):
  source ~/Desktop/ros2_ws/install/setup.bash
  python3 ~/Desktop/odom_laser_consistency_test.py --duration 60

  Copy the final REPORT block and send it for analysis.

Dependencies: ROS2 Humble Python packages (rclpy, tf2_ros, sensor_msgs, nav_msgs).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def wrap_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def valid_range(r: float, scan: LaserScan, max_range: float) -> bool:
    return (
        math.isfinite(r)
        and r > scan.range_min + 1e-3
        and r < min(scan.range_max, max_range)
    )


def lookup_tf(
    tf_buffer: Buffer,
    target: str,
    source: str,
    stamp,
    timeout_s: float,
) -> Tuple[Optional[object], Optional[str]]:
    try:
        tf = tf_buffer.lookup_transform(
            target,
            source,
            stamp,
            timeout=Duration(seconds=timeout_s),
        )
        return tf, None
    except TransformException as exc:
        return None, str(exc)


def laser_pose_in_odom(
    tf_buffer: Buffer,
    stamp,
    timeout_s: float,
) -> Tuple[Optional[Tuple[float, float, float]], Optional[str]]:
    tf, err = lookup_tf(tf_buffer, "odom", "laser", stamp, timeout_s)
    if tf is None:
        return None, err

    t = tf.transform.translation
    q = tf.transform.rotation
    yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
    return (t.x, t.y, yaw), None


def relative_pose_2d(
    p1: Tuple[float, float, float],
    p2: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Motion of frame-2 origin expressed in frame-1 (x forward, y left)."""
    x1, y1, yaw1 = p1
    x2, y2, yaw2 = p2
    dx_w = x2 - x1
    dy_w = y2 - y1
    dyaw = wrap_rad(yaw2 - yaw1)
    c = math.cos(yaw1)
    s = math.sin(yaw1)
    dx_b = c * dx_w + s * dy_w
    dy_b = -s * dx_w + c * dy_w
    return dx_b, dy_b, dyaw


def scan_points(scan: LaserScan, max_range: float) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    angle = scan.angle_min
    for r in scan.ranges:
        if valid_range(r, scan, max_range):
            pts.append((r * math.cos(angle), r * math.sin(angle)))
        angle += scan.angle_increment
    return pts


def predict_bins_from_prev(
    prev: LaserScan,
    dx: float,
    dy: float,
    dyaw: float,
    curr: LaserScan,
    max_range: float,
) -> dict:
    """Project prev-scan points into curr-scan angular bins after motion (dx,dy,dyaw)."""
    bins: dict = {}
    c = math.cos(dyaw)
    s = math.sin(dyaw)
    angle = prev.angle_min
    for r in prev.ranges:
        if valid_range(r, prev, max_range):
            px = r * math.cos(angle)
            py = r * math.sin(angle)
            qx = c * (px - dx) + s * (py - dy)
            qy = -s * (px - dx) + c * (py - dy)
            qa = math.atan2(qy, qx)
            qr = math.hypot(qx, qy)
            if qa < curr.angle_min or qa > curr.angle_max:
                angle += prev.angle_increment
                continue
            idx = int(round((qa - curr.angle_min) / curr.angle_increment))
            if 0 <= idx < len(curr.ranges):
                bins.setdefault(idx, []).append(qr)
        angle += prev.angle_increment
    return {k: sorted(v)[len(v) // 2] for k, v in bins.items()}


def compare_scans(
    prev: LaserScan,
    curr: LaserScan,
    dx: float,
    dy: float,
    dyaw: float,
    max_range: float,
    inlier_m: float,
) -> Tuple[Optional[float], Optional[float], int]:
    """Return (rmse, inlier_ratio, num_pairs)."""
    pred = predict_bins_from_prev(prev, dx, dy, dyaw, curr, max_range)
    errs: List[float] = []
    inliers = 0
    for idx, pred_r in pred.items():
        meas_r = curr.ranges[idx]
        if not valid_range(meas_r, curr, max_range):
            continue
        err = abs(pred_r - meas_r)
        errs.append(err)
        if err <= inlier_m:
            inliers += 1

    if len(errs) < 8:
        return None, None, len(errs)

    rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
    ratio = inliers / len(errs)
    return rmse, ratio, len(errs)


@dataclass
class SegmentStats:
    name: str
    rmse_list: List[float] = field(default_factory=list)
    inlier_list: List[float] = field(default_factory=list)
    pairs: int = 0

    def add(self, rmse: float, inlier: float, n: int) -> None:
        self.rmse_list.append(rmse)
        self.inlier_list.append(inlier)
        self.pairs += n

    def summary(self) -> str:
        if not self.rmse_list:
            return f"{self.name}: no samples"
        rmse_avg = sum(self.rmse_list) / len(self.rmse_list)
        inl_avg = sum(self.inlier_list) / len(self.inlier_list)
        return (
            f"{self.name}: pairs={self.pairs}, samples={len(self.rmse_list)}, "
            f"rmse_avg={rmse_avg:.3f} m, inlier_avg={inl_avg * 100:.1f}%"
        )


class OdomLaserConsistencyNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("odom_laser_consistency_test")
        self.args = args
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_scan: Optional[LaserScan] = None
        self.last_laser_pose: Optional[Tuple[float, float, float]] = None
        self.last_odom: Optional[Odometry] = None

        self.scan_count = 0
        self.odom_count = 0
        self.tf_ok = 0
        self.tf_fail = 0
        self.tf_latest_ok = 0
        self.tf_errors: Counter = Counter()
        self.last_stamp_ok = False

        self.overall = SegmentStats("overall")
        self.straight = SegmentStats("straight")
        self.rotate = SegmentStats("rotate")
        self.mixed = SegmentStats("mixed")

        self.odom_history: Deque[Tuple[float, float, float, float]] = deque(maxlen=200)

        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 100)

        self.start_time = time.monotonic()
        self.get_logger().info(
            "odom_laser_consistency_test started. Drive slowly: straight, then rotate in place."
        )

    def on_odom(self, msg: Odometry) -> None:
        self.odom_count += 1
        self.last_odom = msg
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_history.append((t, p.x, p.y, yaw))

    def motion_label(self) -> str:
        if self.last_odom is None:
            return "mixed"
        vx = self.last_odom.twist.twist.linear.x
        wz = self.last_odom.twist.twist.angular.z
        if abs(wz) < 0.12 and abs(vx) > 0.04:
            return "straight"
        if abs(wz) > 0.12 and abs(vx) < 0.08:
            return "rotate"
        return "mixed"

    def on_scan(self, msg: LaserScan) -> None:
        if time.monotonic() - self.start_time > self.args.duration:
            return

        self.scan_count += 1
        stamp = msg.header.stamp

        pose, err = laser_pose_in_odom(self.tf_buffer, stamp, self.args.tf_timeout)
        stamp_ok = pose is not None
        if not stamp_ok:
            self.tf_fail += 1
            if err:
                self.tf_errors[err.split("\n")[0][:120]] += 1
            pose, _ = laser_pose_in_odom(
                self.tf_buffer, rclpy.time.Time(), self.args.tf_timeout
            )
            if pose is not None:
                self.tf_latest_ok += 1
        else:
            self.tf_ok += 1

        if pose is None:
            self.last_scan = msg
            self.last_laser_pose = None
            self.last_stamp_ok = False
            return

        can_compare = (
            self.last_scan is not None
            and self.last_laser_pose is not None
            and stamp_ok == self.last_stamp_ok
        )
        if can_compare:
            dt = (
                stamp.sec + stamp.nanosec * 1e-9
                - self.last_scan.header.stamp.sec
                - self.last_scan.header.stamp.nanosec * 1e-9
            )
            if 0.05 < dt < 1.5:
                dx, dy, dyaw = relative_pose_2d(self.last_laser_pose, pose)
                motion = abs(dx) + abs(dy) + abs(dyaw)
                if motion >= self.args.min_motion:
                    result = compare_scans(
                        self.last_scan,
                        msg,
                        dx,
                        dy,
                        dyaw,
                        self.args.max_range,
                        self.args.inlier_m,
                    )
                    rmse, inlier, n_pairs = result
                    if rmse is not None and inlier is not None:
                        self.overall.add(rmse, inlier, n_pairs)
                        label = self.motion_label()
                        if label == "straight":
                            self.straight.add(rmse, inlier, n_pairs)
                        elif label == "rotate":
                            self.rotate.add(rmse, inlier, n_pairs)
                        else:
                            self.mixed.add(rmse, inlier, n_pairs)

        self.last_scan = msg
        self.last_laser_pose = pose
        self.last_stamp_ok = stamp_ok

    def probe_tf_chain(self) -> List[str]:
        latest = rclpy.time.Time()
        checks = [
            ("odom", "base_footprint"),
            ("base_footprint", "laser"),
            ("odom", "laser"),
        ]
        lines: List[str] = []
        for target, source in checks:
            _, err = lookup_tf(self.tf_buffer, target, source, latest, self.args.tf_timeout)
            if err:
                lines.append(f"  {target} <- {source} (latest): FAIL | {err.split(chr(10))[0][:100]}")
            else:
                lines.append(f"  {target} <- {source} (latest): OK")
        return lines

    def build_report(self) -> str:
        elapsed = time.monotonic() - self.start_time
        scan_hz = self.scan_count / max(elapsed, 1e-3)
        odom_hz = self.odom_count / max(elapsed, 1e-3)
        tf_total = self.tf_ok + self.tf_fail
        tf_rate = (100.0 * self.tf_ok / tf_total) if tf_total else 0.0

        lines = [
            "=" * 60,
            "ODOM vs LASER CONSISTENCY REPORT",
            "=" * 60,
            f"duration_s       : {elapsed:.1f}",
            f"scan_hz          : {scan_hz:.2f}  (count={self.scan_count})",
            f"odom_hz          : {odom_hz:.2f}  (count={self.odom_count})  [expect ~50]",
            f"tf_at_scan_stamp : {self.tf_ok}/{tf_total} ({tf_rate:.1f}%)",
            f"tf_latest_fallback: {self.tf_latest_ok}/{tf_total}",
            "",
            "tf_chain_probe (latest time):",
        ]
        lines.extend(self.probe_tf_chain())
        if self.tf_errors:
            lines.append("")
            lines.append("top_tf_errors_at_scan_stamp:")
            for msg, cnt in self.tf_errors.most_common(3):
                lines.append(f"  x{cnt}: {msg}")
        lines.extend([
            "",
            self.overall.summary(),
            self.straight.summary(),
            self.rotate.summary(),
            self.mixed.summary(),
            "",
        ])

        if odom_hz < 15.0:
            lines.append(
                "WARN: /odom too slow — base_driver_node likely not running at 50 Hz."
            )
            lines.append("      Check: ros2 node list | grep base_driver")
            lines.append("      Check: ros2 topic info /odom -v")
            lines.append("")

        if tf_rate < 90.0:
            lines.append("VERDICT: TF stamp mismatch — rebuild lslidar_driver (scan timestamp fix).")
            if self.overall.rmse_list:
                lines.append("  (consistency used latest-TF fallback; see overall rmse below)")
        elif not self.overall.rmse_list:
            lines.append(
                "VERDICT: Not enough motion/samples — drive straight AND rotate slowly."
            )
        else:
            rmse_avg = sum(self.overall.rmse_list) / len(self.overall.rmse_list)
            inl_avg = sum(self.overall.inlier_list) / len(self.overall.inlier_list)
            if rmse_avg < 0.15 and inl_avg > 0.65:
                lines.append("VERDICT: GOOD — odom and laser agree well for SLAM.")
            elif rmse_avg < 0.30 and inl_avg > 0.45:
                lines.append(
                    "VERDICT: MARGINAL — usable but tune IMU/wheel gain or drive slower."
                )
            else:
                lines.append(
                    "VERDICT: BAD — odom and laser disagree; fix base_driver / IMU gain before SLAM."
                )
                if self.straight.rmse_list and self.rotate.rmse_list:
                    s_rmse = sum(self.straight.rmse_list) / len(self.straight.rmse_list)
                    r_rmse = sum(self.rotate.rmse_list) / len(self.rotate.rmse_list)
                    if r_rmse > s_rmse * 1.4:
                        lines.append("  hint: rotation segment worse -> check IMU yaw / wz gain.")
                    if s_rmse > r_rmse * 1.4:
                        lines.append("  hint: straight segment worse -> check wheel velocity / slip.")

        lines.append("=" * 60)
        return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Odom vs laser consistency test (no SLAM).")
    p.add_argument("--duration", type=float, default=60.0, help="Test length in seconds.")
    p.add_argument("--max-range", type=float, default=12.0, help="Max lidar range used.")
    p.add_argument("--inlier-m", type=float, default=0.15, help="Inlier threshold in meters.")
    p.add_argument("--min-motion", type=float, default=0.02,
                   help="Min |dx|+|dy|+|dyaw| between scans to score a pair.")
    p.add_argument("--tf-timeout", type=float, default=0.05, help="TF lookup timeout per scan.")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = OdomLaserConsistencyNode(args)

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        while rclpy.ok() and (time.monotonic() - node.start_time) < args.duration:
            executor.spin_once(timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        report = node.build_report()
        print(report, flush=True)
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
