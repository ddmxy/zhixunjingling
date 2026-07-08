#!/usr/bin/env python3
"""Chassis align helper — vision + grab stay on official pick_color.launch.

Flow (matches your 3 steps):
  1. pick_color.launch: arm_look + camera (color_pick moves arm on start)
  2. grab_mission chassis:
     (a) green visible but outside green box -> move chassis toward ROI center
     (b) green in box -> stop when official /color_ik_result (find_color DETECTED)
     (c) no green in frame -> black tape edge guides approach (too far / offset)
  3. color_pick_node grabs (same as pick_color.launch alone)

Official (you tuned this — do not duplicate):
  source /opt/ros/humble/setup.bash
  source ~/wheeltec_arm/install/setup.bash
  ros2 launch wheeltec_color_sort pick_color.launch.py target_color:=green port:=/dev/ttyACM0

Run:
  T1 base_driver (chassis /dev/ttyUSB0)
  T2 pick_color.launch (camera + find_color + arm + color_pick)
  T3 python3 grab_mission.py (chassis only)
"""
from __future__ import annotations

import argparse
import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

try:
    from a150_arm_msgs.msg import ColorIkResult
except ImportError as exc:
    raise SystemExit(
        "a150_arm_msgs required — source ~/wheeltec_arm/install/setup.bash"
    ) from exc

# find_color_node crop + green HSV (chassis hint only — grab uses official nodes)
PIC_W, PIC_H = 640, 480
CROP_X1 = PIC_W // 16 * 4
CROP_X2 = PIC_W // 16 * 15
CROP_Y1 = PIC_H // 16
CROP_Y2 = PIC_H // 16 * 12
PROC_W, PROC_H = 320, 240
GREEN_HSV = (50, 46, 24, 80, 255, 255)

# Official color_pick_node poses (read-only reference for /joint_states)
ARM_HOME_JOINTS = (0.0, 0.0, 0.0, 0.0, 0.0)
ARM_HOME_TOL = 0.18
GRIP_CLOSE_THRESH = 0.45
GRIP_OPEN_THRESH = -0.05


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_from_quat(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def imgmsg_to_bgr(msg: Image):
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = (msg.encoding or "").lower()
    step = msg.step or (msg.width * 3)
    try:
        arr = buf.reshape(msg.height, step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    except ValueError:
        arr = buf.reshape(msg.height, msg.width, -1)
    if enc in ("rgb8", "rgb"):
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if enc in ("bgr8", "bgr", ""):
        return arr[:, :, :3].copy()
    return cv2.cvtColor(arr[:, :, :3], cv2.COLOR_RGB2BGR)


def roi_norm(w: int, h: int):
    return (CROP_X1 / w, CROP_Y1 / h, CROP_X2 / w, CROP_Y2 / h)


def green_centroid(bgr):
    """Chassis hint: green block center in full camera frame (same HSV as find_color).

    Searches the whole image so chassis can move when the block is visible but
    still outside the official ROI green box.
    """
    h, w = bgr.shape[:2]
    proc = cv2.resize(bgr, (PROC_W, PROC_H), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(proc, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    hsv = cv2.dilate(cv2.erode(hsv, kernel, iterations=1), kernel, iterations=1)
    lo, hi = GREEN_HSV[:3], GREEN_HSV[3:]
    mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    mask = cv2.erode(mask, None, iterations=4)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    # Block looks smaller in full frame when far — lower threshold than in-box crop
    if cv2.contourArea(c) < 350:
        return None
    cx, cy = cv2.minAreaRect(c)[0]
    return cx / PROC_W, cy / PROC_H


def green_in_roi(green, roi) -> bool:
    if green is None:
        return False
    cx, cy = green
    x0, y0, x1, y1 = roi
    return x0 <= cx <= x1 and y0 <= cy <= y1


def detect_box_tape(bgr, s_max, v_max, min_pix):
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    black = cv2.inRange(hsv, np.array([0, 0, 0], np.uint8),
                        np.array([180, s_max, v_max], np.uint8))
    x0, x1, y0, y1 = int(0.05 * w), int(0.95 * w), int(0.02 * h), int(0.98 * h)
    m = np.zeros_like(black)
    m[y0:y1, x0:x1] = black[y0:y1, x0:x1]
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    if cv2.countNonZero(m) < min_pix:
        return None
    ys, xs = np.nonzero(m)
    rows = [np.nonzero(m[:, c])[0].max() for c in range(x0, x1, 3)
            if np.nonzero(m[:, c])[0].size]
    near_y = float(np.percentile(rows, 80)) / h if rows else float(ys.max()) / h
    return {"cx": float(xs.mean()) / w, "near_y": near_y}


class GrabMission(Node):
    ALIGN = "ALIGN"
    HANDOFF = "HANDOFF"
    RETURN = "RETURN"
    DONE = "DONE"

    def __init__(self, args):
        super().__init__("grab_mission")
        self.a = args
        self.bgr = None
        self.last_img_t = 0.0
        self.odom = None
        self.home = None
        self.state = self.ALIGN
        self.t_state = time.monotonic()
        self.t_last = self.t_state
        self.forward_dist = 0.0
        self.return_phase = 0
        self.official_ik_count = 0
        self.last_official_ik_t = 0.0
        self.t_first_ik: float | None = None
        self.arm_joints: list[float] | None = None
        self.joint_10: float | None = None
        self.saw_grip_closed = False
        self.t_arm_home_stable: float | None = None
        self.smooth_green = None
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.last_vx_dir = 0

        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.create_subscription(Image, args.image_topic, self._on_img, qos_profile_sensor_data)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, qos_profile_sensor_data)
        # ONLY listen — never publish /color_ik_result (official find_color does that)
        self.create_subscription(
            ColorIkResult, "/color_ik_result", self._on_official_ik, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint, 10)

        self.show_win = not args.no_window
        if self.show_win:
            os.environ.setdefault("DISPLAY", ":0")
            try:
                cv2.startWindowThread()
                cv2.namedWindow("grab_mission", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("grab_mission", 960, 720)
            except Exception as e:
                self.get_logger().warn(f"no window: {e}")
                self.show_win = False

        self.create_timer(1.0 / args.rate, self._tick)
        self.stop()
        self.get_logger().info(
            "grab_mission = CHASSIS ONLY. Start pick_color.launch first (find_color + color_pick).")
        if args.no_drive:
            self.get_logger().warn("--no-drive: chassis disabled")

    def _on_img(self, msg):
        b = imgmsg_to_bgr(msg)
        if b is not None:
            self.bgr = b
            self.last_img_t = time.monotonic()

    def _on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y,
                     yaw_from_quat(p.orientation.x, p.orientation.y,
                                   p.orientation.z, p.orientation.w))

    def _on_official_ik(self, msg: ColorIkResult):
        if msg.color != self.a.target_color:
            return
        self.official_ik_count += 1
        self.last_official_ik_t = time.monotonic()
        if self.official_ik_count >= self.a.handoff_ik_frames and self.t_first_ik is None:
            self.t_first_ik = time.monotonic()
        if self.official_ik_count == 1:
            self.get_logger().info(
                ">>> official find_color DETECTED (/color_ik_result) — stop chassis, "
                "color_pick will grab <<<")

    def _on_joint(self, msg: JointState):
        pos = {n: float(p) for n, p in zip(msg.name, msg.position)}
        if any(f"joint_{i}" in pos for i in range(1, 6)):
            self.arm_joints = [pos.get(f"joint_{i}", 0.0) for i in range(1, 6)]
        if "joint_10" in pos:
            self.joint_10 = pos["joint_10"]

    def _grip_closed(self) -> bool:
        return self.joint_10 is not None and self.joint_10 >= GRIP_CLOSE_THRESH

    def _grip_opening_again(self) -> bool:
        return self.joint_10 is not None and self.joint_10 <= GRIP_OPEN_THRESH

    def _arm_near_home(self) -> bool:
        if not self.arm_joints:
            return False
        return all(
            abs(a - h) < ARM_HOME_TOL for a, h in zip(self.arm_joints, ARM_HOME_JOINTS)
        )

    def _pick_done_reason(self) -> str | None:
        """True end of one pick = closed grip + arm_home, before official reset opens claw."""
        a = self.a
        age = self._age()

        if self._grip_closed():
            self.saw_grip_closed = True

        if self.saw_grip_closed and self._grip_opening_again() and age > 4.0:
            return "gripper re-opening — kill NOW"

        if self.saw_grip_closed and self._arm_near_home():
            if self.t_arm_home_stable is None:
                self.t_arm_home_stable = time.monotonic()
            elif time.monotonic() - self.t_arm_home_stable >= a.pick_home_hold_s:
                return "grip closed + arm_home stable"
        else:
            self.t_arm_home_stable = None

        if a.pick_kill_max_s > 0 and age >= a.pick_kill_max_s:
            return f"handoff max {a.pick_kill_max_s:.0f}s (safety kill)"

        return None

    def stop(self):
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        try:
            self.cmd_pub.publish(Twist())
        except Exception as exc:
            self.get_logger().warn(f"stop publish: {exc}")

    def _go(self, s):
        if s != self.state:
            self.get_logger().info(f"--> {s}")
            if s == self.HANDOFF:
                self.saw_grip_closed = False
                self.t_arm_home_stable = None
        self.state = s
        self.t_state = time.monotonic()

    def _age(self):
        return time.monotonic() - self.t_state

    def _filter_green(self, green):
        if green is None:
            self.smooth_green = None
            return None
        if self.smooth_green is None:
            self.smooth_green = green
        else:
            a = self.a.green_ema
            self.smooth_green = (
                a * green[0] + (1.0 - a) * self.smooth_green[0],
                a * green[1] + (1.0 - a) * self.smooth_green[1],
            )
        return self.smooth_green

    def _p_speed(self, err: float, k: float, vmax: float, dead: float, gain_sign: float) -> float:
        if abs(err) <= dead:
            return 0.0
        direction = 1.0 if err > 0 else -1.0
        v = gain_sign * direction * k * (abs(err) - dead)
        return max(-vmax, min(vmax, v))

    def _apply_align_cmd(self, cmd, ex, ey, dead_x, dead_y, sequential, dt, box_vx=False):
        a = self.a
        # Block above tgt (ey<0) -> +vx forward; below -> back. Use --vx-sign -1 if still reversed.
        vx_gain = a.vx_sign
        raw_wz = self._p_speed(ex, a.k_wz, a.max_wz, dead_x, a.wz_sign * -1.0)
        raw_vx = self._p_speed(ey, a.k_vx, a.max_vx, dead_y, vx_gain)
        if raw_vx > 0 and self.forward_dist >= a.max_forward:
            raw_vx = 0.0

        # Hysteresis: avoid forward/back ping-pong near target
        if abs(ey) > dead_y and raw_vx != 0.0:
            new_dir = 1 if raw_vx > 0 else -1
            if self.last_vx_dir != 0 and new_dir != self.last_vx_dir:
                if abs(ey) < a.vx_flip_hyst:
                    raw_vx = 0.0
                else:
                    self.last_vx_dir = new_dir
            else:
                self.last_vx_dir = new_dir
        elif abs(ey) <= dead_y:
            self.last_vx_dir = 0

        if sequential:
            if abs(ex) > dead_x:
                raw_vx = 0.0
            elif abs(ey) <= dead_y:
                raw_vx = 0.0
                raw_wz = 0.0

        tgt_wz = self._clamp_wz(raw_wz, ex if abs(ex) > dead_x else None, dead_x)
        tgt_vx = raw_vx if abs(ey) > dead_y else 0.0
        # Heavy chassis: ensure cmd exceeds static friction when we decide to move
        if abs(tgt_vx) > 1e-6 and abs(tgt_vx) < a.min_vx:
            tgt_vx = math.copysign(a.min_vx, tgt_vx)
        if abs(tgt_wz) > 1e-6 and abs(tgt_wz) < a.min_wz:
            tgt_wz = math.copysign(a.min_wz, tgt_wz)
        vx, wz = self._slew_cmd(tgt_vx, tgt_wz, dt)
        cmd.linear.x = vx
        cmd.angular.z = wz

    def _slew_cmd(self, target_vx: float, target_wz: float, dt: float) -> tuple[float, float]:
        dvx = self.a.slew_vx * dt
        dwz = self.a.slew_wz * dt
        self.cmd_vx += max(-dvx, min(dvx, target_vx - self.cmd_vx))
        self.cmd_wz += max(-dwz, min(dwz, target_wz - self.cmd_wz))
        if abs(self.cmd_vx) < self.a.cmd_eps:
            self.cmd_vx = 0.0
        if abs(self.cmd_wz) < self.a.cmd_eps:
            self.cmd_wz = 0.0
        return self.cmd_vx, self.cmd_wz

    def _clamp_wz(self, wz, err_x=None, dead=None):
        dead = dead if dead is not None else self.a.dead_x
        if err_x is not None and abs(err_x) < dead:
            return 0.0
        wz = max(-self.a.max_wz, min(self.a.max_wz, wz))
        if 0 < abs(wz) < self.a.min_wz and err_x is not None and abs(err_x) > dead * 2.5:
            wz = math.copysign(self.a.min_wz, wz)
        return wz

    def _align_target(self, roi):
        x0, y0, x1, y1 = roi
        # Yellow cross: x=center, y=align_y_frac from ROI top (default 1/4 down)
        return (
            (x0 + x1) / 2.0 + self.a.align_cx_offset,
            y0 + self.a.align_y_frac * (y1 - y0) + self.a.align_cy_offset,
        )

    def _align_met(self, green, roi) -> bool:
        if green is None:
            return False
        tgt_cx, tgt_cy = self._align_target(roi)
        return (abs(green[0] - tgt_cx) <= self.a.dead_x
                and abs(green[1] - tgt_cy) <= self.a.dead_y)

    def _drive(self, cmd, dt):
        if self.a.no_drive:
            self.stop()
            return
        if abs(cmd.linear.x) < 1e-6 and abs(cmd.angular.z) < 1e-6:
            self.stop()
            self.cmd_vx = 0.0
            self.cmd_wz = 0.0
            return
        if cmd.linear.x > 0:
            self.forward_dist += cmd.linear.x * dt
        elif cmd.linear.x < 0:
            self.forward_dist = max(0.0, self.forward_dist + cmd.linear.x * dt)
        try:
            self.cmd_pub.publish(cmd)
        except Exception as exc:
            self.get_logger().warn(f"cmd_vel publish: {exc}")

    def _official_ready(self, green=None, roi=None) -> bool:
        if self.official_ik_count < self.a.handoff_ik_frames:
            return False
        if self.a.require_align_before_handoff and not self._align_met(green, roi):
            return False
        return True

    def _ctrl_align(self, cmd, green, box, roi, img_ok, dt):
        a = self.a

        if self._official_ready(green, roi):
            self.stop()
            self._go(self.HANDOFF)
            return "DETECTED + aligned -> handoff to color_pick"

        if (self.official_ik_count >= a.handoff_ik_frames and green is not None
                and a.require_align_before_handoff and not self._align_met(green, roi)):
            tgt_cx, tgt_cy = self._align_target(roi)
            cx, cy = green
            # Official saw block but chassis target not met — keep fine approach
            self._apply_align_cmd(cmd, cx - tgt_cx, cy - tgt_cy, a.dead_x, a.dead_y,
                                  a.sequential_align, dt)
            return (f"ik seen, align y@1/4 ({cx:.2f},{cy:.2f}) tgt=({tgt_cx:.2f},{tgt_cy:.2f})")

        if self._age() < a.settle_s:
            self.stop()
            return f"settle {a.settle_s - self._age():.1f}s (wait pick_color arm_look)..."

        if not img_ok:
            return "waiting /image_raw (from pick_color.launch usb_cam)..."

        x0, y0, x1, y1 = roi
        tgt_cx, tgt_cy = self._align_target(roi)
        dead_x, dead_y = a.dead_x, a.dead_y

        if green is not None:
            cx, cy = green
            ex, ey = cx - tgt_cx, cy - tgt_cy
            in_roi = green_in_roi(green, roi)
            self._apply_align_cmd(cmd, ex, ey, dead_x, dead_y, a.sequential_align, dt)
            if in_roi and abs(ex) <= dead_x and abs(ey) <= dead_y:
                self.cmd_vx = 0.0
                self.cmd_wz = 0.0
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            tag = "chassis OFF" if a.no_drive else ("hold" if abs(ex) <= dead_x and abs(ey) <= dead_y
                                                      else ("in ROI" if in_roi else "green->ROI"))
            ik_n = self.official_ik_count
            return (f"{tag} ({cx:.2f},{cy:.2f}) tgt=({tgt_cx:.2f},{tgt_cy:.2f}) "
                    f"ik={ik_n}/{a.handoff_ik_frames} fwd={self.forward_dist:.2f}m")

        if box is not None and green is None:
            ex = box["cx"] - tgt_cx
            ed = a.box_near_y - box["near_y"]
            self._apply_align_cmd(cmd, ex, ed, dead_x, dead_y, a.sequential_align, dt, box_vx=True)
            return f"no green -> box edge rim_y={box['near_y']:.2f}"

        return "waiting green (official find_color in pick_color.launch)..."

    def _ctrl_handoff(self):
        self.stop()
        a = self.a
        reason = self._pick_done_reason()
        if reason:
            self.get_logger().info(f"one-pick done ({reason}) -> DONE, kill pick_color NOW")
            self._go(self.DONE)
            return reason
        if self._age() >= a.pick_wait_s:
            self.get_logger().info("pick wait done (max)")
            if a.return_home and self.home is not None and not a.exit_on_done:
                self.return_phase = 0
                self._go(self.RETURN)
            else:
                self._go(self.DONE)
            return f"handoff done ({a.pick_wait_s:.0f}s) -> next"
        left = a.pick_wait_s - self._age()
        closed = "Y" if self.saw_grip_closed else "n"
        home = "Y" if self._arm_near_home() else "n"
        hold_left = -1.0
        if self.t_arm_home_stable is not None:
            hold_left = max(0.0, a.pick_home_hold_s - (time.monotonic() - self.t_arm_home_stable))
        kill_left = max(0.0, a.pick_kill_max_s - self._age()) if a.pick_kill_max_s > 0 else -1.0
        j10 = f"{self.joint_10:.2f}" if self.joint_10 is not None else "?"
        return (
            f"HANDOFF: closed={closed} home={home} j10={j10} "
            f"hold={hold_left:.1f}s kill_in={kill_left:.0f}s ({left:.0f}s max)"
        )

    def _ctrl_return(self, cmd):
        a = self.a
        if self.odom is None or self.home is None:
            return "no odom"
        x, y, yaw = self.odom
        hx, hy, hyaw = self.home
        dx, dy = hx - x, hy - y
        dist = math.hypot(dx, dy)
        self.forward_dist = 0.0
        if self.return_phase == 0:
            if dist < a.home_xy_tol:
                self.return_phase = 2
            else:
                e = wrap(math.atan2(dy, dx) - yaw)
                if abs(e) > a.home_yaw_tol:
                    cmd.angular.z = self._clamp_wz(2.0 * e)
                    return f"turn home {math.degrees(e):.0f} deg"
                self.return_phase = 1
        if self.return_phase == 1:
            if dist < a.home_xy_tol:
                self.return_phase = 2
            else:
                cmd.linear.x = min(a.max_vx, a.k_home_vx * dist)
                return f"drive home d={dist:.2f}m"
        e = wrap(hyaw - yaw)
        if abs(e) > a.home_yaw_tol:
            cmd.angular.z = self._clamp_wz(2.0 * e)
            return f"fix yaw {math.degrees(e):.0f} deg"
        self.stop()
        self._go(self.DONE)
        return "home reached"

    def _tick(self):
        now = time.monotonic()
        dt = max(1e-3, now - self.t_last)
        self.t_last = now

        img_ok = self.bgr is not None and (now - self.last_img_t) < 0.7
        bgr = self.bgr.copy() if img_ok else None
        roi = roi_norm(640, 480)
        green_raw = green_centroid(bgr) if bgr is not None else None
        if green_raw is None:
            self.smooth_green = None
            green = None
        else:
            green = self._filter_green(green_raw)
        box = detect_box_tape(bgr, self.a.black_s_max, self.a.black_v_max,
                              self.a.tape_min_pix) if bgr is not None else None

        if self.state == self.ALIGN and self.odom is not None and self.home is None:
            self.home = self.odom

        cmd = Twist()
        note = ""

        if self.state == self.ALIGN:
            note = self._ctrl_align(cmd, green, box, roi, img_ok, dt)
            self._drive(cmd, dt)
        elif self.state == self.HANDOFF:
            note = self._ctrl_handoff()
        elif self.state == self.RETURN:
            note = self._ctrl_return(cmd)
            self._drive(cmd, dt)
        elif self.state == self.DONE:
            self.stop()
            note = "DONE"

        self._show(bgr, roi, green, note, self._align_target(roi) if img_ok else None)

    def _show(self, bgr, roi, green, note, align_tgt=None):
        if bgr is None:
            if self.show_win:
                blank = np.zeros((480, 640, 3), np.uint8)
                cv2.putText(blank, "waiting /image_raw from pick_color...", (20, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imshow("grab_mission", blank)
                cv2.waitKey(1)
            return
        h, w = bgr.shape[:2]
        vis = bgr.copy()
        x0, y0, x1, y1 = roi
        cv2.rectangle(vis, (int(x0 * w), int(y0 * h)), (int(x1 * w), int(y1 * h)), (0, 255, 0), 2)
        cv2.putText(vis, "official ROI (find_color)", (int(x0 * w), max(16, int(y0 * h) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        if align_tgt is not None:
            tx, ty = align_tgt
            cv2.drawMarker(vis, (int(tx * w), int(ty * h)), (0, 255, 255),
                           cv2.MARKER_CROSS, 22, 2)
            cv2.putText(vis, "align tgt (1/4 from top)", (int(tx * w) + 8, int(ty * h) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            # Guide line: 1/4 height from ROI top
            x0, y0, x1, y1 = roi
            guide_y = int((y0 + self.a.align_y_frac * (y1 - y0)) * h)
            cv2.line(vis, (int(x0 * w), guide_y), (int(x1 * w), guide_y), (0, 255, 255), 1)
        if green is not None:
            cv2.circle(vis, (int(green[0] * w), int(green[1] * h)), 8, (0, 0, 255), -1)
        drive = "OFF" if self.a.no_drive else "ON"
        cv2.putText(vis, f"[{self.state}|chassis {drive}|ik={self.official_ik_count}]",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(vis, "vision+grab = pick_color.launch (NOT this script)", (8, 44),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1)
        cv2.putText(vis, note, (8, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)
        if self.show_win:
            cv2.imshow("grab_mission", vis)
            cv2.waitKey(1)


def build_parser():
    p = argparse.ArgumentParser(
        description="Chassis align only; vision+grab via pick_color.launch")
    p.add_argument("--image-topic", default="/image_raw")
    p.add_argument("--cmd-topic", default="/cmd_vel")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--target-color", default="green")
    p.add_argument("--rate", type=float, default=10.0,
                   help="control loop Hz (lower = smoother)")
    p.add_argument("--green-ema", type=float, default=0.35,
                   help="green centroid smoothing 0..1 (lower = smoother)")
    p.add_argument("--sequential-align", action=argparse.BooleanOptionalAction, default=True,
                   help="turn first then forward (reduces shake)")
    p.add_argument("--slew-vx", type=float, default=0.15,
                   help="max m/s per tick change on linear.x")
    p.add_argument("--slew-wz", type=float, default=0.55,
                   help="max rad/s per tick change on angular.z")
    p.add_argument("--cmd-eps", type=float, default=0.003,
                   help="cmd below this treated as zero")
    p.add_argument("--no-drive", action="store_true")
    p.add_argument("--return-home", action="store_true")
    p.add_argument("--no-window", action="store_true")
    p.add_argument("--exit-on-done", action="store_true",
                   help="exit after one-pick handoff (for run_full_mission)")
    p.add_argument("--pick-kill-max-s", type=float, default=14.0,
                   help="max seconds in HANDOFF before force-kill pick_color")
    p.add_argument("--pick-home-hold-s", type=float, default=0.6,
                   help="seconds grip closed + arm_home stable before exit")
    p.add_argument("--pick-once-s", type=float, default=None,
                   help="deprecated alias for --pick-kill-max-s")
    p.add_argument("--handoff-ik-frames", type=int, default=1,
                   help="stop chassis after N official /color_ik_result (DETECTED)")
    p.add_argument("--pick-wait-s", type=float, default=25.0,
                   help="seconds to hold still while color_pick grabs")
    p.add_argument("--settle-s", type=float, default=5.0,
                   help="wait after start for pick_color arm_look to finish")
    p.add_argument("--require-align-before-handoff", action=argparse.BooleanOptionalAction,
                   default=True,
                   help="wait until green at align tgt even if /color_ik_result seen")
    p.add_argument("--vx-flip-hyst", type=float, default=0.08,
                   help="min |ey| before reversing forward/back (anti ping-pong)")
    p.add_argument("--align-y-frac", type=float, default=0.25,
                   help="align target y: fraction of ROI height from top (0.25=1/4 down)")
    p.add_argument("--align-cx-offset", type=float, default=0.0,
                   help="chassis align target shift right (+) in normalized image")
    p.add_argument("--align-cy-offset", type=float, default=0.0,
                   help="fine tune align y on top of align-y-frac")
    p.add_argument("--black-s-max", type=int, default=100)
    p.add_argument("--black-v-max", type=int, default=75)
    p.add_argument("--tape-min-pix", type=int, default=800)
    p.add_argument("--box-near-y", type=float, default=0.80)
    p.add_argument("--move-threshold", type=float, default=0.07)
    p.add_argument("--k-wz", type=float, default=1.2)
    p.add_argument("--k-vx", type=float, default=0.24)
    p.add_argument("--max-wz", type=float, default=0.45)
    p.add_argument("--min-wz", type=float, default=0.20)
    p.add_argument("--max-vx", type=float, default=0.10)
    p.add_argument("--min-vx", type=float, default=0.055)
    p.add_argument("--dead-x", type=float, default=0.04)
    p.add_argument("--dead-y", type=float, default=0.04)
    p.add_argument("--wz-sign", type=float, default=1.0)
    p.add_argument("--vx-sign", type=float, default=1.0)
    p.add_argument("--max-forward", type=float, default=0.50)
    p.add_argument("--home-xy-tol", type=float, default=0.06)
    p.add_argument("--home-yaw-tol", type=float, default=0.09)
    p.add_argument("--k-home-vx", type=float, default=0.4)
    return p


def main():
    args = build_parser().parse_args()
    if args.pick_once_s is not None:
        args.pick_kill_max_s = float(args.pick_once_s)
    rclpy.init()
    node = GrabMission(args)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.exit_on_done and node.state == GrabMission.DONE:
                node.get_logger().info("exit-on-done")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        if not args.no_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
