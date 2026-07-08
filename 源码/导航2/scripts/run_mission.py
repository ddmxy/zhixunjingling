#!/usr/bin/env python3
"""Nav mission: Nav2 coarse -> chassis cmd_vel fine-tune at each waypoint.

  wp_09 -> wp_10 -> wp_11 -> box_approach -> dwell 20s -> home

START:
  ros2 launch car_bringup navigation.launch.py map:=/home/sunrise/maps/arena_map_v4.yaml
  python3 ~/Desktop/run_mission.py
"""
from __future__ import annotations

import math
import os
import subprocess
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

DEFAULT_WAYPOINTS = os.path.expanduser("~/mission_waypoints.yaml")
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROUTE = ["wp_09", "wp_10", "wp_11", "box_approach"]
DEFAULT_MISSION = {
    "coarse_handoff_xy_m": 0.22,
    "coarse_handoff_yaw_deg": 18.0,
    "fine_at_via": True,
    "fine_xy_m": 0.08,
    "fine_yaw_deg": 6.0,
    "fine_timeout_s": 25.0,
    "fine_max_vx": 0.11,
    "fine_max_wz": 0.45,
    "fine_min_vx": 0.04,
    "fine_min_wz": 0.16,
    "final_coarse_handoff_xy_m": 0.12,
    "final_xy_ok_m": 0.09,
    "final_xy_severe_m": 0.09,
    "final_yaw_deg": 1.5,
    "final_rescue_vx": 0.06,
    "final_yaw_timeout_s": 90.0,
    "final_rescue_timeout_s": 20.0,
    "home_arrive_xy_m": 0.30,
    "home_nav_timeout_s": 180.0,
    "settle_time_s": 0.3,
    "final_dwell_s": 20.0,
    "return_home": True,
    "escape_before_home": True,
    "escape_forward_m": 0.15,
    "escape_vx": 0.08,
    "escape_min_vx": 0.06,
    "escape_rotate_deg": 90.0,
    "escape_rotate_wz": 0.48,
    "escape_rotate_sign": -1.0,
    "escape_rotate_tol_deg": 8.0,
    "escape_rotate_max_s": 12.0,
    "escape_settle_s": 0.8,
    "escape_max_s": 15.0,
    "grab_enabled": True,
    "grab_target_color": "green",
    "grab_pick_kill_max_s": 14.0,
    "grab_pick_home_hold_s": 0.6,
    "grab_pick_wait_s": 28.0,
    "grab_settle_s": 6.0,
    "grab_timeout_s": 150.0,
    "final_point": "box_approach",
    "via_points": ["wp_09", "wp_10", "wp_11"],
}


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_deg(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def wrap_rad(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def load_mission(path: str) -> tuple[dict, list[str], dict]:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    points = data.get("points", {})
    if not points:
        raise KeyError(f"no 'points' in {path}")
    route = data.get("route", DEFAULT_ROUTE)
    for name in route:
        if name not in points:
            raise KeyError(f"route item '{name}' missing in points")
    if "home" not in points:
        raise KeyError("'home' point required")
    mission = {**DEFAULT_MISSION, **(data.get("mission") or {})}
    return points, route, mission


def lifecycle_active(node: Node, srv: str) -> bool:
    cmd = ["ros2", "lifecycle", "get", srv]
    try:
        out = subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT, timeout=8)
        return "active [3]" in out
    except (subprocess.SubprocessError, FileNotFoundError) as exc:
        node.get_logger().warn(f"lifecycle check {srv}: {exc}")
        return False


class MissionRunner(Node):
    def __init__(self) -> None:
        super().__init__("run_mission")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom_seen = False
        self._odom_xy: tuple[float, float, float] | None = None
        self._last_amcl: PoseWithCovarianceStamped | None = None
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_seen = True
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._odom_xy = (
            p.x,
            p.y,
            yaw_from_quat(q.x, q.y, q.z, q.w),
        )

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._last_amcl = msg

    def wait_base(self, timeout: float = 60.0) -> None:
        self.get_logger().info("waiting for /odom...")
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._odom_seen:
                self.get_logger().info("base OK")
                return
        raise RuntimeError("no /odom — start navigation first")

    def wait_nav2(self, timeout: float = 90.0) -> None:
        self.get_logger().info("waiting for bt_navigator active...")
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if lifecycle_active(self, "/bt_navigator"):
                self.get_logger().info("Nav2 OK")
                return
            time.sleep(2.0)
        raise RuntimeError("bt_navigator not active")

    def wait_nav_action(self, timeout: float = 30.0) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("navigate_to_pose not available")

    def stop_robot(self) -> None:
        z = Twist()
        for _ in range(10):
            self.cmd_pub.publish(z)
            time.sleep(0.05)

    def pose_error(self, point: dict) -> tuple[float, float] | None:
        if self._last_amcl is None:
            return None
        p = self._last_amcl.pose.pose
        q = p.orientation
        ax, ay = p.position.x, p.position.y
        ayaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        tx, ty, tyaw = float(point["x"]), float(point["y"]), float(point["yaw"])
        xy_err = math.hypot(ax - tx, ay - ty)
        yaw_err = abs(wrap_deg(math.degrees(ayaw - tyaw)))
        return xy_err, yaw_err

    def _cancel_nav(self, handle) -> None:
        if handle is None:
            return
        fut = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        time.sleep(0.4)
        self.stop_robot()

    def fine_adjust(
        self,
        name: str,
        point: dict,
        xy_tol: float,
        yaw_tol_deg: float,
        max_time: float,
        max_vx: float,
        max_wz: float,
        min_vx: float = 0.05,
        min_wz: float = 0.16,
    ) -> None:
        """Short fine-tune: turn toward point, drive, then fix yaw."""
        tx = float(point["x"])
        ty = float(point["y"])
        tyaw = float(point["yaw"])
        self.get_logger().info(
            f"FINE [{name}]  xy<{xy_tol}m yaw<{yaw_tol_deg}°  max {max_time:.0f}s"
        )
        end = time.monotonic() + max_time
        next_log = time.monotonic()
        dt = 0.1
        phase = "pos"
        ok_hits = 0

        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._last_amcl is None:
                time.sleep(dt)
                continue

            p = self._last_amcl.pose.pose
            q = p.orientation
            cx, cy = p.position.x, p.position.y
            cyaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            dx, dy = tx - cx, ty - cy
            dist = math.hypot(dx, dy)
            yaw_err_deg = wrap_deg(math.degrees(tyaw - cyaw))
            bearing = wrap_rad(math.atan2(dy, dx) - cyaw)
            bear_deg = abs(math.degrees(bearing))

            if phase == "pos" and dist <= xy_tol:
                phase = "yaw"
                self.stop_robot()

            cmd = Twist()
            if phase == "pos":
                if bear_deg > 12.0:
                    wz = max(-max_wz, min(max_wz, 2.0 * bearing))
                    cmd.angular.z = math.copysign(max(min_wz, abs(wz)), wz)
                else:
                    cmd.linear.x = min(max_vx, max(min_vx, 0.9 * dist))
                    if bear_deg > 3.0:
                        cmd.angular.z = max(-max_wz, min(max_wz, bearing))
            else:
                if abs(yaw_err_deg) <= yaw_tol_deg:
                    ok_hits += 1
                    self.stop_robot()
                    if ok_hits >= 2:
                        self.get_logger().info(f"FINE OK [{name}]")
                        return
                    time.sleep(dt)
                    continue
                ok_hits = 0
                wz = max(-max_wz, min(max_wz, math.radians(3.5 * yaw_err_deg)))
                cmd.angular.z = math.copysign(max(min_wz, abs(wz)), wz)

            self.cmd_pub.publish(cmd)
            if time.monotonic() >= next_log:
                self.get_logger().info(
                    f"  fine [{name}/{phase}] xy={dist:.2f}m yaw={yaw_err_deg:.0f}° "
                    f"vx={cmd.linear.x:.2f} wz={cmd.angular.z:.2f}"
                )
                next_log = time.monotonic() + 2.0
            time.sleep(dt)

        self.stop_robot()
        self.get_logger().warn(f"FINE timeout [{name}] — continue")

    def fine_adjust_final(self, name: str, point: dict, mission: dict) -> None:
        """Final point: yaw only; xy left to vision unless severely off."""
        tyaw = float(point["yaw"])
        tx, ty = float(point["x"]), float(point["y"])
        yaw_tol = float(mission.get("final_yaw_deg", 1.5))
        severe_xy = float(mission["final_xy_severe_m"])
        max_wz = float(mission["fine_max_wz"])
        rescue_vx = float(mission["final_rescue_vx"])
        min_wz = float(mission.get("fine_min_wz", 0.12))
        dead_wz = max(0.04, min_wz * 0.35)
        dt = 0.1
        ok_hits = 0

        self.get_logger().info(
            f"FINE final [{name}]  yaw<{yaw_tol}° strict  "
            f"xy<={float(mission['final_xy_ok_m']):.2f}m ok  "
            f">{severe_xy:.2f}m才救"
        )
        end = time.monotonic() + float(mission["final_yaw_timeout_s"])
        next_log = time.monotonic()

        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._last_amcl is None:
                time.sleep(dt)
                continue
            p = self._last_amcl.pose.pose
            q = p.orientation
            cyaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            dist = math.hypot(tx - p.position.x, ty - p.position.y)
            yaw_err_deg = wrap_deg(math.degrees(tyaw - cyaw))

            if abs(yaw_err_deg) <= yaw_tol:
                ok_hits += 1
                self.stop_robot()
                if ok_hits >= 5:
                    self.get_logger().info(
                        f"yaw OK [{name}]  yaw={yaw_err_deg:.1f}°  xy={dist:.3f}m"
                    )
                    break
                time.sleep(dt)
                continue
            ok_hits = 0

            wz = math.radians(min(40.0, 3.0 * abs(yaw_err_deg)))
            wz = math.copysign(wz, yaw_err_deg if yaw_err_deg != 0.0 else 1.0)
            cmd = Twist()
            if abs(wz) < dead_wz:
                cmd.angular.z = 0.0
            else:
                cmd.angular.z = max(-max_wz, min(max_wz, wz))
                if abs(cmd.angular.z) < min_wz:
                    cmd.angular.z = math.copysign(min_wz, cmd.angular.z)
            self.cmd_pub.publish(cmd)
            if time.monotonic() >= next_log:
                self.get_logger().info(
                    f"  final yaw [{name}]  yaw={yaw_err_deg:.1f}°  xy={dist:.3f}m  "
                    f"wz={cmd.angular.z:.2f}"
                )
                next_log = time.monotonic() + 2.0
            time.sleep(dt)
        else:
            self.stop_robot()
            self.get_logger().warn(f"yaw timeout [{name}] — continue")

        err = self.pose_error(point)
        if err is None:
            return
        xy_err, yaw_err = err
        if xy_err <= severe_xy:
            ok = float(mission.get("final_xy_ok_m", 0.09))
            self.get_logger().info(
                f"FINAL [{name}] hold  xy={xy_err:.3f}m (ok<={ok:.2f}m, vision)"
            )
            self.stop_robot()
            return

        self.get_logger().warn(
            f"xy severe [{name}] {xy_err:.2f}m > {severe_xy:.2f}m — careful rescue"
        )
        end2 = time.monotonic() + float(mission["final_rescue_timeout_s"])
        target_xy = float(mission.get("final_xy_ok_m", 0.09))
        next_log = time.monotonic()

        while rclpy.ok() and time.monotonic() < end2:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._last_amcl is None:
                time.sleep(dt)
                continue
            p = self._last_amcl.pose.pose
            q = p.orientation
            cx, cy = p.position.x, p.position.y
            cyaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            dx, dy = tx - cx, ty - cy
            dist = math.hypot(dx, dy)
            yaw_err_deg = wrap_deg(math.degrees(tyaw - cyaw))

            if dist <= target_xy:
                self.stop_robot()
                self.get_logger().info(
                    f"rescue OK [{name}]  xy={dist:.3f}m  yaw={yaw_err_deg:.1f}°"
                )
                return

            cmd = Twist()
            if abs(yaw_err_deg) > yaw_tol + 2.0:
                cmd.angular.z = max(
                    -max_wz * 0.6,
                    min(max_wz * 0.6, math.radians(yaw_err_deg)),
                )
            else:
                bearing = wrap_rad(math.atan2(dy, dx) - cyaw)
                if abs(math.degrees(bearing)) > 12.0:
                    cmd.angular.z = max(-max_wz * 0.5, min(max_wz * 0.5, bearing))
                else:
                    cmd.linear.x = max(0.0, min(rescue_vx, 0.5 * dist))
            self.cmd_pub.publish(cmd)
            if time.monotonic() >= next_log:
                self.get_logger().info(
                    f"  rescue [{name}]  xy={dist:.3f}m  yaw={yaw_err_deg:.1f}°"
                )
                next_log = time.monotonic() + 2.0
            time.sleep(dt)

        self.stop_robot()
        err = self.pose_error(point)
        if err:
            self.get_logger().warn(
                f"rescue timeout [{name}]  xy={err[0]:.3f}m — hold for vision"
            )

    def navigate_coarse(
        self,
        name: str,
        point: dict,
        handoff_xy: float,
        handoff_yaw_deg: float,
        timeout: float = 120.0,
    ) -> None:
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
            f"COARSE Nav2 -> [{name}]  x={point['x']:.3f}  y={point['y']:.3f}  "
            f"yaw={point.get('yaw_deg', math.degrees(point['yaw'])):.1f}°"
        )
        send = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"Nav2 goal rejected at [{name}]")

        result = handle.get_result_async()
        t0 = time.monotonic()
        next_log = t0 + 4.0
        stall_t: float | None = None
        last_xy = 999.0
        handed_off = False

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            now = time.monotonic()
            err = self.pose_error(point)

            if err and now >= next_log:
                self.get_logger().info(
                    f"  coarse [{name}]  xy={err[0]:.2f}m  yaw={err[1]:.1f}°"
                )
                next_log = now + 4.0

            if err:
                xy, yaw = err
                if xy <= handoff_xy and yaw <= handoff_yaw_deg:
                    self.get_logger().info(f"coarse close [{name}] -> fine-tune")
                    self._cancel_nav(handle)
                    handed_off = True
                    break
                if xy < 0.22:
                    if abs(xy - last_xy) < 0.012:
                        stall_t = stall_t or now
                        if now - stall_t > 6.0:
                            self.get_logger().info(
                                f"coarse stuck [{name}] xy={xy:.2f}m -> fine-tune"
                            )
                            self._cancel_nav(handle)
                            handed_off = True
                            break
                    else:
                        stall_t = None
                    last_xy = xy

            if result.done():
                break
            if now - t0 > timeout:
                self.get_logger().warn(f"coarse timeout [{name}] -> fine-tune")
                self._cancel_nav(handle)
                handed_off = True
                break

        if not handed_off:
            if result.done():
                res = result.result()
                st = getattr(res, "status", None)
                self.get_logger().info(f"Nav2 finished [{name}] status={st}")
            self._cancel_nav(handle)

    def navigate_home(self, name: str, point: dict, arrive_xy: float, timeout: float) -> None:
        """Return home: Nav2 drives to xy only (goal yaw = current heading). No fine-tune."""
        deadline = time.monotonic() + 5.0
        while self._last_amcl is None and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_yaw = float(point["yaw"])
        if self._last_amcl is not None:
            q = self._last_amcl.pose.pose.orientation
            goal_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = point.get("frame_id", "map")
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(point["x"])
        goal.pose.pose.position.y = float(point["y"])
        q = yaw_to_quat(goal_yaw)
        goal.pose.pose.orientation.x, goal.pose.pose.orientation.y = q[0], q[1]
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = q[2], q[3]

        self.get_logger().info(
            f"HOME Nav2 -> [{name}]  x={point['x']:.3f} y={point['y']:.3f}  "
            f"(position only, arrive<{arrive_xy:.2f}m)"
        )
        send = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("Nav2 home goal rejected")

        result = handle.get_result_async()
        t0 = time.monotonic()
        next_log = t0 + 4.0
        last_xy = 999.0
        stall_t: float | None = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            now = time.monotonic()
            if self._last_amcl is None:
                continue
            p = self._last_amcl.pose.pose
            xy = math.hypot(float(point["x"]) - p.position.x, float(point["y"]) - p.position.y)

            if now >= next_log:
                self.get_logger().info(f"  home [{name}]  xy={xy:.2f}m")
                next_log = now + 4.0

            if xy <= arrive_xy:
                self.get_logger().info(f"HOME OK [{name}]  xy={xy:.2f}m")
                self._cancel_nav(handle)
                return

            if abs(xy - last_xy) < 0.015:
                stall_t = stall_t or now
                if now - stall_t > 12.0 and xy < 0.55:
                    self.get_logger().warn(
                        f"home stuck xy={xy:.2f}m — accept and stop"
                    )
                    self._cancel_nav(handle)
                    return
            else:
                stall_t = None
            last_xy = xy

            if result.done():
                res = result.result()
                st = getattr(res, "status", None)
                if xy <= arrive_xy * 1.2:
                    self.get_logger().info(f"HOME OK (Nav2 done) xy={xy:.2f}m status={st}")
                    return
                self.get_logger().warn(f"Nav2 home done but xy={xy:.2f}m status={st}")
                self._cancel_nav(handle)
                return

            if now - t0 > timeout:
                self.get_logger().warn(f"home timeout xy={xy:.2f}m — stop")
                self._cancel_nav(handle)
                return

    def go_to_point(self, name: str, point: dict, mission: dict, mode: str) -> None:
        if mode == "coarse_only":
            self.navigate_home(
                name,
                point,
                float(mission.get("home_arrive_xy_m", 0.30)),
                float(mission.get("home_nav_timeout_s", 180.0)),
            )
            self.stop_robot()
            return

        handoff_xy = float(mission["coarse_handoff_xy_m"])
        if mode == "final":
            handoff_xy = float(mission.get("final_coarse_handoff_xy_m", 0.12))
        self.navigate_coarse(
            name,
            point,
            handoff_xy,
            float(mission["coarse_handoff_yaw_deg"]),
        )
        time.sleep(0.3)
        self.stop_robot()
        if mode == "final":
            self.fine_adjust_final(name, point, mission)
        elif mission.get("fine_at_via", False):
            self.fine_adjust(
                name,
                point,
                float(mission["fine_xy_m"]),
                float(mission["fine_yaw_deg"]),
                float(mission["fine_timeout_s"]),
                float(mission["fine_max_vx"]),
                float(mission["fine_max_wz"]),
                float(mission.get("fine_min_vx", 0.05)),
                float(mission.get("fine_min_wz", 0.16)),
            )
        else:
            self.get_logger().info(f"[{name}] via — Nav2 only, skip fine")
        pause = float(mission["settle_time_s"])
        if pause > 0:
            self.stop_robot()
            time.sleep(pause)

    def dwell(self, seconds: float, label: str) -> None:
        self.stop_robot()
        self.get_logger().info(f"DWELL {label}: {seconds:.0f}s")
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            self.stop_robot()
        self.get_logger().info(f"DWELL done ({label})")

    def escape_forward(self, mission: dict) -> None:
        """Alias: forward creep + rotate before home."""
        self.escape_before_home(mission)

    def escape_before_home(self, mission: dict) -> None:
        if not mission.get("escape_before_home", True):
            return
        # Always forward first (away from box), then rotate — never rotate while against box.
        self.get_logger().info("ESCAPE step 1/2: forward (no rotation)")
        self._escape_forward_creep(mission)
        rot_deg = float(mission.get("escape_rotate_deg", 0.0))
        if rot_deg > 0:
            self.get_logger().info("ESCAPE step 2/2: rotate in place")
            time.sleep(0.5)
            self._escape_rotate_in_place(mission)

    def _escape_forward_creep(self, mission: dict) -> None:
        dist_m = float(mission.get("escape_forward_m", 0.15))
        if dist_m <= 0:
            self.get_logger().warn("escape: forward distance 0 — skip")
            return
        vx = float(mission.get("escape_vx", 0.08))
        min_vx = float(mission.get("escape_min_vx", 0.06))
        settle = float(mission.get("escape_settle_s", 0.5))
        max_t = float(mission.get("escape_max_s", 15.0))

        self.stop_robot()
        time.sleep(settle)

        deadline = time.monotonic() + 2.0
        while self._odom_xy is None and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self._odom_xy is None:
            self.get_logger().warn("escape: no odom — skip forward creep")
            return

        sx, sy, syaw = self._odom_xy
        self.get_logger().info(
            f"ESCAPE forward {dist_m:.2f}m @ vx={vx:.2f} heading={math.degrees(syaw):.0f}°"
        )
        cmd = Twist()
        cmd.linear.x = max(vx, min_vx)
        cmd.angular.z = 0.0
        end = time.monotonic() + max_t
        dt = 0.1
        next_log = time.monotonic()

        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._odom_xy is None:
                time.sleep(dt)
                continue
            cx, cy, _ = self._odom_xy
            traveled = math.hypot(cx - sx, cy - sy)
            if traveled >= dist_m:
                self.get_logger().info(f"ESCAPE forward done  traveled={traveled:.3f}m")
                break
            self.cmd_pub.publish(cmd)
            if time.monotonic() >= next_log:
                self.get_logger().info(f"  escape fwd  {traveled:.3f}/{dist_m:.2f}m")
                next_log = time.monotonic() + 1.0
            time.sleep(dt)
        else:
            if self._odom_xy is not None:
                cx, cy, _ = self._odom_xy
                traveled = math.hypot(cx - sx, cy - sy)
                self.get_logger().warn(f"escape forward timeout  traveled={traveled:.3f}m")
        self.stop_robot()
        time.sleep(0.4)

    def _escape_rotate_in_place(self, mission: dict) -> None:
        deg = float(mission.get("escape_rotate_deg", 90.0))
        wz = float(mission.get("escape_rotate_wz", 0.48))
        sign = float(mission.get("escape_rotate_sign", -1.0))
        tol = math.radians(float(mission.get("escape_rotate_tol_deg", 8.0)))
        max_t = float(mission.get("escape_rotate_max_s", 12.0))
        target_rad = math.radians(deg) * sign

        deadline = time.monotonic() + 2.0
        while self._odom_xy is None and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._odom_xy is None:
            self.get_logger().warn("escape: no odom — skip rotate")
            return

        _, _, syaw = self._odom_xy
        self.get_logger().info(f"ESCAPE rotate {deg:.0f}° @ wz={wz:.2f}")
        end = time.monotonic() + max_t
        dt = 0.08
        cmd = Twist()
        next_log = time.monotonic()

        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._odom_xy is None:
                time.sleep(dt)
                continue
            _, _, cyaw = self._odom_xy
            turned = wrap_rad(cyaw - syaw)
            remaining = target_rad - turned
            if abs(remaining) <= tol:
                self.get_logger().info(
                    f"ESCAPE rotate done  turned={math.degrees(turned):.1f}°"
                )
                break
            cmd.angular.z = wz if remaining > 0 else -wz
            self.cmd_pub.publish(cmd)
            if time.monotonic() >= next_log:
                self.get_logger().info(
                    f"  escape rot  {math.degrees(turned):.1f}° / {deg:.0f}°"
                )
                next_log = time.monotonic() + 1.0
            time.sleep(dt)
        else:
            if self._odom_xy is not None:
                _, _, cyaw = self._odom_xy
                turned = math.degrees(wrap_rad(cyaw - syaw))
                self.get_logger().warn(f"escape rotate timeout  turned={turned:.1f}°")
        self.stop_robot()
        time.sleep(0.5)

    def set_initial_pose(self, point: dict) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = point.get("frame_id", "map")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(point["x"])
        msg.pose.pose.position.y = float(point["y"])
        q = yaw_to_quat(float(point["yaw"]))
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = q[0], q[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[2], q[3]
        for i in range(36):
            msg.pose.covariance[i] = 0.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        for _ in range(3):
            self.initial_pub.publish(msg)
            time.sleep(0.3)
        self.get_logger().info(
            f"AMCL <- home  x={point['x']:.3f} y={point['y']:.3f} "
            f"yaw={point.get('yaw_deg', math.degrees(point['yaw'])):.1f}°"
        )
        time.sleep(2.0)

    def run_arm_pose(self, pose: str) -> None:
        cmd = [sys.executable, os.path.join(SCRIPT_DIR, "arm_pose_cmd.py"), pose]
        self.get_logger().info(f"arm: {pose}")
        subprocess.run(cmd, check=True)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Run multi-waypoint mission")
    parser.add_argument("--waypoints", default=DEFAULT_WAYPOINTS)
    parser.add_argument("--no-set-pose", action="store_true")
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--no-return", action="store_true")
    parser.add_argument("--no-escape", action="store_true",
                        help="skip forward creep before return home")
    parser.add_argument("--escape-m", type=float, default=None,
                        help="override escape_forward_m (meters)")
    args = parser.parse_args()

    points, route, mission = load_mission(args.waypoints)
    final_name = str(mission["final_point"])
    do_return = bool(mission["return_home"]) and not args.no_return

    rclpy.init()
    node = MissionRunner()
    try:
        node.wait_base()
        if not args.no_set_pose:
            node.set_initial_pose(points["home"])
        node.wait_nav2()
        node.wait_nav_action()

        if args.arm:
            node.run_arm_pose("arm_home")

        for i, name in enumerate(route, 1):
            mode = "final" if name == final_name else "via"
            node.get_logger().info(f"=== leg {i}/{len(route)}: {name} ({mode}) ===")
            node.go_to_point(name, points[name], mission, mode)

        node.dwell(float(mission["final_dwell_s"]), final_name)

        if do_return:
            if not args.no_escape:
                if args.escape_m is not None:
                    mission = {**mission, "escape_forward_m": args.escape_m}
                node.escape_forward(mission)
            node.get_logger().info("=== return home (Nav2 only, no fine) ===")
            node.go_to_point("home", points["home"], mission, "coarse_only")

        if args.arm:
            node.run_arm_pose("arm_home")

        node.get_logger().info("MISSION DONE")
    except Exception as exc:
        node.get_logger().error(str(exc))
        sys.exit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
