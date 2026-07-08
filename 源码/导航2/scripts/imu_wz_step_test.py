#!/usr/bin/env python3
"""IMU + angular velocity test aligned with base_driver navigation.

Yaw handling matches base_driver_node.cpp:
  - use_first_yaw_as_zero (default true): first 0x04 yaw -> bias, nav_yaw = wrap(raw - bias + offset)
  - wz from 0x04 is used as-is (same as /odom twist.angular.z)

Also checks:
  - static gyro bias / yaw drift
  - wz step response (v=0 pure rotation)
  - yaw change vs integrated wz (gyro scale sanity)

MCU note: CAR_YAW_TO_WHEEL_GAIN in Car.h (currently 1.2) scales wheel differential only;
act_wz from IMU is reported as-is — compare sign/ratio, not exact match to cmd_w.

Usage on RDK:
  pkill -9 -f base_driver
  pip install pyserial
  python3 imu_wz_step_test.py --port /dev/ttyUSB0

Usage on Windows:
  python imu_wz_step_test.py --port COM3
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass
from typing import List, Tuple

try:
    import serial
except ImportError:
    print("Missing dependency: pip install pyserial")
    sys.exit(1)

SOF = 0xFF
EOF = 0xFE
CMD_TX = 0x02
CMD_RX_VEL = 0x03
CMD_RX_YAW = 0x04

MAX_LINEAR = 0.5
MAX_ANGULAR = 1.5

# Pure rotation: v=0; steps match nav2 max_vel_theta=0.5
DEFAULT_STEPS: List[Tuple[float, float, str]] = [
    (0.0, 0.0, "stop"),
    (0.0, 0.3, "wz +0.3"),
    (0.0, 0.5, "wz +0.5"),
    (0.0, -0.3, "wz -0.3"),
    (0.0, 0.0, "stop"),
]

# Same as Car.h (inform user if firmware still 0.75)
MCU_YAW_TO_WHEEL_GAIN = 1.45


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


def wrap_rad(a: float) -> float:
    """Same as base_driver_node normalizeAngle()."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def make_cmd_frame(v: float, w: float) -> bytes:
    v = clamp(v, -MAX_LINEAR, MAX_LINEAR)
    w = clamp(w, -MAX_ANGULAR, MAX_ANGULAR)
    return bytes([SOF, CMD_TX]) + struct.pack("<ff", v, w) + bytes([EOF])


@dataclass
class Feedback:
    raw_yaw: float = 0.0
    nav_yaw: float = 0.0
    wz: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vel_rx_count: int = 0
    yaw_rx_count: int = 0
    parse_resync: int = 0


class ImuTester:
    def __init__(
        self,
        port: str,
        baud: int,
        use_first_yaw_as_zero: bool,
        yaw_offset_rad: float,
    ):
        self.use_first_yaw_as_zero = use_first_yaw_as_zero
        self.yaw_offset_rad = yaw_offset_rad
        self.yaw_zero_bias = None
        self.yaw_zero_initialized = False
        self.raw_rx_bytes = 0

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.fb = Feedback()
        self._state = 0
        self._cmd = 0
        self._payload = bytearray()

    def apply_yaw_nav(self, raw_yaw: float) -> float:
        """Mirror base_driver onValidFrame() for 0x04."""
        if self.use_first_yaw_as_zero and not self.yaw_zero_initialized:
            self.yaw_zero_bias = raw_yaw
            self.yaw_zero_initialized = True
            print(
                f"[YAW] zero bias set from first frame: "
                f"{raw_yaw:+.4f} rad ({rad2deg(raw_yaw):+.1f} deg) "
                f"(same as base_driver use_first_yaw_as_zero)"
            )
        bias = self.yaw_zero_bias if self.yaw_zero_bias is not None else 0.0
        return wrap_rad(raw_yaw - bias + self.yaw_offset_rad)

    def poll_rx(self) -> int:
        data = self.ser.read(256)
        self.raw_rx_bytes += len(data)
        for b in data:
            self._on_byte(b)
        return len(data)

    def sniff_raw(self, duration_s: float = 2.0) -> None:
        """Print first bytes on wire when protocol parse fails."""
        self.ser.reset_input_buffer()
        t_end = time.time() + duration_s
        buf = bytearray()
        while time.time() < t_end:
            self.send(0.0, 0.0)
            chunk = self.ser.read(256)
            buf.extend(chunk)
            time.sleep(0.05)
        if not buf:
            print("[SNIFF] no bytes at all -> wrong port, STM32 off, or cable")
            return
        show = buf[:64]
        hexs = " ".join(f"{b:02X}" for b in show)
        print(f"[SNIFF] {len(buf)} bytes in {duration_s:.0f}s, first bytes: {hexs}")
        if 0xFF in buf:
            print("[SNIFF] saw 0xFF SOF marker -> likely STM32 binary stream on this port")
        else:
            print("[SNIFF] no 0xFF -> probably NOT the STM32 chassis port")

    def handshake(self, timeout_s: float = 10.0) -> bool:
        print("[COMM] handshake: send zero-speed, wait for 0x03/0x04 upload")
        t_end = time.time() + timeout_s
        vel0 = self.fb.vel_rx_count
        yaw0 = self.fb.yaw_rx_count
        self.raw_rx_bytes = 0

        while time.time() < t_end:
            self.send(0.0, 0.0)
            self.poll_rx()
            if self.fb.yaw_rx_count > yaw0:
                print(
                    f"[COMM] OK (0x04) raw_yaw={self.fb.raw_yaw:+.3f} rad  "
                    f"nav_yaw={rad2deg(self.fb.nav_yaw):+.1f} deg  wz={self.fb.wz:+.3f} rad/s"
                )
                return True
            if self.fb.vel_rx_count > vel0 and (time.time() + 2.0) < t_end:
                # Have 0x03 but not 0x04 yet — keep waiting (TIM5 alternates)
                pass
            time.sleep(0.05)

        print(f"[COMM] FAIL within {timeout_s:.0f}s")
        print(
            f"       vel_frames(0x03)={self.fb.vel_rx_count}  "
            f"yaw_frames(0x04)={self.fb.yaw_rx_count}  "
            f"raw_bytes={self.raw_rx_bytes}  resync={self.fb.parse_resync}"
        )
        if self.fb.vel_rx_count > 0 and self.fb.yaw_rx_count == 0:
            print("       Got 0x03 but NO 0x04 -> firmware missing Car_Send_Yaw(), reflash STM32")
        elif self.raw_rx_bytes == 0:
            print("       No serial data -> check power, USB cable, port (/dev/ttyUSB0 vs ACM0)")
        else:
            print("       Bytes received but no valid frames -> port conflict or wrong baud")
        self.sniff_raw(2.0)
        return False

    def _on_byte(self, b: int) -> None:
        if self._state == 0:
            self._state = 1 if b == SOF else 0
            return
        if self._state == 1:
            if b in (CMD_RX_VEL, CMD_RX_YAW):
                self._cmd = b
                self._payload.clear()
                self._state = 2
            else:
                self._state = 1 if b == SOF else 0
            return
        if self._state == 2:
            self._payload.append(b)
            if len(self._payload) >= 8:
                self._state = 3
            return
        if self._state == 3:
            if b == EOF and len(self._payload) == 8:
                if self._cmd == CMD_RX_VEL:
                    self.fb.vx, self.fb.vy = struct.unpack("<ff", self._payload)
                    self.fb.vel_rx_count += 1
                elif self._cmd == CMD_RX_YAW:
                    raw_yaw, wz = struct.unpack("<ff", self._payload)
                    self.fb.raw_yaw = raw_yaw
                    self.fb.wz = wz
                    self.fb.nav_yaw = self.apply_yaw_nav(raw_yaw)
                    self.fb.yaw_rx_count += 1
            else:
                self.fb.parse_resync += 1
            self._state = 1 if b == SOF else 0

    def send(self, v: float, w: float) -> None:
        self.ser.write(make_cmd_frame(v, w))
        self.ser.flush()

    def static_check(self, duration_s: float, hz: float) -> None:
        print(f"\n[STATIC] car still on ground for {duration_s:.0f}s ...")
        self.send(0.0, 0.0)
        time.sleep(0.5)

        wz_samples: List[float] = []
        yaw_nav0 = self.fb.nav_yaw
        t_end = time.time() + duration_s
        next_send = time.time()

        while time.time() < t_end:
            if time.time() >= next_send:
                self.send(0.0, 0.0)
                next_send += 1.0 / hz
            self.poll_rx()
            wz_samples.append(self.fb.wz)
            time.sleep(0.01)

        wz_mean = sum(wz_samples) / max(1, len(wz_samples))
        wz_peak = max(wz_samples, key=abs) if wz_samples else 0.0
        yaw_drift = rad2deg(wrap_rad(self.fb.nav_yaw - yaw_nav0))

        print(
            f"[STATIC] nav_yaw_drift={yaw_drift:+.1f} deg  "
            f"wz_mean={wz_mean:+.4f} rad/s ({rad2deg(wz_mean):+.2f} deg/s)  "
            f"wz_peak={wz_peak:+.4f}"
        )

    def run_wz_profile(
        self,
        steps: List[Tuple[float, float, str]],
        step_s: float,
        settle_s: float,
        hz: float,
        live_hz: float,
    ) -> None:
        print()
        print("yaw: nav_yaw = wrap(raw_yaw - first_bias + offset)  [same as /odom orientation.z]")
        print(f"MCU CAR_YAW_TO_WHEEL_GAIN expected {MCU_YAW_TO_WHEEL_GAIN} "
              f"(effective body w ≈ cmd_w * gain)")
        print()
        print("time_s  label          cmd_w  act_wz  err_w  nav_yaw  d_yaw  yaw_hz")
        print("-" * 76)

        t0 = time.time()
        for v_tgt, w_tgt, label in steps:
            step_start = time.time()
            yaw_count_start = self.fb.yaw_rx_count
            yaw_nav_start = self.fb.nav_yaw
            wz_int = 0.0
            d_yaw_unwrapped_deg = 0.0
            last_yaw_nav = self.fb.nav_yaw
            last_t = step_start

            self.send(v_tgt, w_tgt)
            time.sleep(settle_s)

            samples_w: List[float] = []
            next_send = time.time()
            next_live = time.time()

            while time.time() - step_start < step_s:
                now = time.time()
                dt = now - last_t
                last_t = now

                if now >= next_send:
                    self.send(v_tgt, w_tgt)
                    next_send = now + 1.0 / hz
                if now >= next_live:
                    err = self.fb.wz - w_tgt
                    d_yaw = rad2deg(wrap_rad(self.fb.nav_yaw - yaw_nav_start))
                    print(
                        f"  [live] cmd_w={w_tgt:+.2f} act_wz={self.fb.wz:+.3f} "
                        f"err={err:+.3f} nav_yaw={rad2deg(self.fb.nav_yaw):+.1f}deg "
                        f"d_yaw={d_yaw:+.1f}deg"
                    )
                    next_live = now + 1.0 / live_hz

                self.poll_rx()
                dy = wrap_rad(self.fb.nav_yaw - last_yaw_nav)
                d_yaw_unwrapped_deg += rad2deg(dy)
                last_yaw_nav = self.fb.nav_yaw
                wz_int += self.fb.wz * dt
                samples_w.append(self.fb.wz)
                time.sleep(0.01)

            yaw_frames = self.fb.yaw_rx_count - yaw_count_start
            yaw_hz = yaw_frames / max(step_s, 1e-3)
            tail = samples_w[-30:] if samples_w else [0.0]
            act_w = sum(tail) / len(tail)
            err_w = act_w - w_tgt
            d_yaw_deg = d_yaw_unwrapped_deg
            d_yaw_from_wz = rad2deg(wz_int)
            elapsed = time.time() - t0
            follow = act_w / w_tgt if abs(w_tgt) > 0.05 else 0.0

            print(
                f"{elapsed:6.1f}  {label:13s} "
                f"{w_tgt:+.2f}  {act_w:+.3f}  {err_w:+.3f}  "
                f"{rad2deg(self.fb.nav_yaw):+6.1f}  {d_yaw_deg:+5.1f}  {yaw_hz:5.1f}"
            )
            if abs(w_tgt) > 0.05 and step_s > 5.0:
                ratio = d_yaw_deg / d_yaw_from_wz if abs(d_yaw_from_wz) > 1.0 else 0.0
                print(
                    f"         yaw_check: integrate(wz)={d_yaw_from_wz:+.1f}deg  "
                    f"nav_yaw_delta={d_yaw_deg:+.1f}deg  ratio={ratio:.2f}  "
                    f"follow=act_w/cmd_w={follow:.2f}"
                )
                if abs(ratio) < 0.6:
                    print(
                        "         WARN: |ratio|<0.6 -> check IMU_GYRO_DPS_FS (gyro scale)"
                    )
                elif follow < 0.35:
                    print(
                        "         WARN: follow low -> raise CAR_YAW_TO_WHEEL_GAIN in Car.h"
                    )

        self.send(0.0, 0.0)
        print()
        print(
            f"[STAT] yaw_frames={self.fb.yaw_rx_count} vel_frames={self.fb.vel_rx_count} "
            f"resync={self.fb.parse_resync}"
        )
        print("[PASS] cmd_w>0: act_wz>0 and d_yaw>0; cmd_w<0: both negative")
        print("[PASS] |ratio| about 0.8~1.2 (integrate wz vs nav_yaw)")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="IMU test aligned with base_driver yaw zeroing + wz step response"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--use-first-yaw-as-zero",
        dest="use_first_yaw_as_zero",
        action="store_true",
        default=True,
        help="same as base_driver (default on)",
    )
    parser.add_argument(
        "--no-use-first-yaw-as-zero",
        dest="use_first_yaw_as_zero",
        action="store_false",
        help="disable first-frame yaw zeroing",
    )
    parser.add_argument(
        "--yaw-offset-rad",
        type=float,
        default=0.0,
        help="same as base_driver yaw_offset_rad",
    )
    parser.add_argument("--static-s", type=float, default=8.0)
    parser.add_argument("--step-s", type=float, default=12.0)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--live-hz", type=float, default=1.0)
    parser.add_argument("--handshake-timeout", type=float, default=10.0)
    parser.add_argument("--skip-static", action="store_true")
    args = parser.parse_args()

    print(f"Port={args.port} Baud={args.baud}")
    print(
        f"Yaw policy: use_first_yaw_as_zero={args.use_first_yaw_as_zero}  "
        f"yaw_offset_rad={args.yaw_offset_rad}"
    )
    tester = ImuTester(
        args.port, args.baud, args.use_first_yaw_as_zero, args.yaw_offset_rad
    )
    try:
        if not tester.handshake(timeout_s=args.handshake_timeout):
            return
        if not args.skip_static:
            tester.static_check(args.static_s, args.hz)
        tester.run_wz_profile(
            DEFAULT_STEPS, args.step_s, args.settle_s, args.hz, args.live_hz
        )
    except KeyboardInterrupt:
        tester.send(0.0, 0.0)
        print("\nStopped.")
    finally:
        tester.ser.close()


if __name__ == "__main__":
    main()
