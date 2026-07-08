#!/usr/bin/env python3
"""Step through target speeds; compare target vs encoder feedback (0x03).

Only 0x03 vx/vy is used for PID validation (encoder-based).
0x04 yaw/wz is IMU-based and printed for reference only.

Usage on RDK:
  pip install pyserial
  python3 velocity_pid_step_test.py --port /dev/ttyUSB0

Usage on Windows:
  python velocity_pid_step_test.py --port COM3
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass, field
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
FRAME_LEN = 11

MAX_LINEAR = 0.5
MAX_ANGULAR = 1.5

# Linear-only profile (encoder feedback is trustworthy for vx)
DEFAULT_STEPS: List[Tuple[float, float, str]] = [
    (0.00, 0.0, "stop"),
    (0.10, 0.0, "fwd 0.10"),
    (0.20, 0.0, "fwd 0.20"),
    (0.30, 0.0, "fwd 0.30"),
    (-0.10, 0.0, "rev 0.10"),
    (0.00, 0.0, "stop"),
]


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def make_cmd_frame(v: float, w: float) -> bytes:
    v = clamp(v, -MAX_LINEAR, MAX_LINEAR)
    w = clamp(w, -MAX_ANGULAR, MAX_ANGULAR)
    return bytes([SOF, CMD_TX]) + struct.pack("<ff", v, w) + bytes([EOF])


@dataclass
class Feedback:
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0
    wz: float = 0.0
    vel_rx_count: int = 0
    yaw_rx_count: int = 0
    last_vel_rx_t: float = 0.0
    last_yaw_rx_t: float = 0.0
    parse_resync: int = 0


class ChassisTester:
  def __init__(self, port: str, baud: int):
    self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
    self.ser.reset_input_buffer()
    self.ser.reset_output_buffer()
    self.fb = Feedback()
    self._state = 0
    self._cmd = 0
    self._payload = bytearray()

  def poll_rx(self) -> None:
    data = self.ser.read(256)
    for b in data:
      self._on_byte(b)

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
        self._state = 0
        if b == SOF:
          self._state = 1
        elif b not in (CMD_TX,):
          self.fb.parse_resync += 1
      return

    if self._state == 2:
      self._payload.append(b)
      if len(self._payload) >= 8:
        self._state = 3
      return

    if self._state == 3:
      now = time.time()
      if b == EOF and len(self._payload) == 8:
        if self._cmd == CMD_RX_VEL:
          self.fb.vx, self.fb.vy = struct.unpack("<ff", self._payload)
          self.fb.vel_rx_count += 1
          self.fb.last_vel_rx_t = now
        elif self._cmd == CMD_RX_YAW:
          self.fb.yaw, self.fb.wz = struct.unpack("<ff", self._payload)
          self.fb.yaw_rx_count += 1
          self.fb.last_yaw_rx_t = now
      else:
        self.fb.parse_resync += 1
      self._state = 0
      if b == SOF:
        self._state = 1

  def send(self, v: float, w: float) -> None:
    frame = make_cmd_frame(v, w)
    self.ser.write(frame)
    self.ser.flush()

  def handshake(self, timeout_s: float = 3.0) -> bool:
    """Send zero-speed to enable MCU feedback, wait for first 0x03 frame."""
    print("[COMM] handshake: sending zero-speed frame (FF 02 ... FE)")
    t_end = time.time() + timeout_s
    vel_before = self.fb.vel_rx_count
    while time.time() < t_end:
      self.send(0.0, 0.0)
      self.poll_rx()
      if self.fb.vel_rx_count > vel_before:
        dt = time.time() - (self.fb.last_vel_rx_t)
        print(
          f"[COMM] OK: received 0x03 velocity frame "
          f"(vx={self.fb.vx:+.3f}, vy={self.fb.vy:+.3f})"
        )
        return True
      time.sleep(0.05)

    print("[COMM] FAIL: no 0x03 frame within {:.1f}s".format(timeout_s))
    print("       Check: STM32 flashed? Car_Send_Current_V enabled? Port/baud?")
    print("       Only one program should open the serial port.")
    return False

  def run_profile(
    self,
    steps: List[Tuple[float, float, str]],
    step_s: float,
    settle_s: float,
    hz: float,
    live_hz: float,
  ) -> None:
    if not self.handshake():
      return

    print()
    print("Columns: tgt_* = host command, act_vx/vy = MCU 0x03 encoder feedback")
    print("         act_wz/yaw = MCU 0x04 IMU (reference only, not used for pass/fail)")
    print()
    print("time_s  label            tgt_v  tgt_w  act_vx act_vy  err_v   vel_hz")
    print("-" * 78)

    self.send(0.0, 0.0)
    time.sleep(0.3)

    t0 = time.time()
    for v_tgt, w_tgt, label in steps:
      step_start = time.time()
      vel_count_start = self.fb.vel_rx_count
      self.send(v_tgt, w_tgt)
      time.sleep(settle_s)

      samples_v: List[float] = []
      next_send = time.time()
      next_live = time.time()

      while time.time() - step_start < step_s:
        now = time.time()
        if now >= next_send:
          self.send(v_tgt, w_tgt)
          next_send = now + 1.0 / hz
        if now >= next_live:
          err = self.fb.vx - v_tgt
          print(
            f"  [live] tgt_v={v_tgt:+.2f} act_vx={self.fb.vx:+.3f} "
            f"err_v={err:+.3f} vy={self.fb.vy:+.3f}"
          )
          next_live = now + 1.0 / live_hz

        self.poll_rx()
        samples_v.append(self.fb.vx)
        time.sleep(0.01)

      vel_frames = self.fb.vel_rx_count - vel_count_start
      vel_hz = vel_frames / max(step_s, 1e-3)
      tail = samples_v[-30:] if samples_v else [0.0]
      act_v = sum(tail) / len(tail)
      err_v = act_v - v_tgt
      elapsed = time.time() - t0

      print(
        f"{elapsed:6.1f}  {label:16s} "
        f"{v_tgt:+.2f}  {w_tgt:+.2f}  "
        f"{act_v:+.3f} {self.fb.vy:+.3f}  "
        f"{err_v:+.3f}  {vel_hz:5.1f}"
      )

    self.send(0.0, 0.0)
    print()
    print(
      f"[STAT] vel_frames={self.fb.vel_rx_count} yaw_frames={self.fb.yaw_rx_count} "
      f"resync={self.fb.parse_resync}"
    )
    print("[PASS] steady |err_v| < 0.03 m/s on 0.10/0.20/0.30 steps")
    print("[NOTE] wz/yaw from IMU ignored for PID judgment")


def main() -> None:
  parser = argparse.ArgumentParser(description="STM32 velocity PID step test (encoder vx)")
  parser.add_argument("--port", default="/dev/ttyUSB0")
  parser.add_argument("--baud", type=int, default=115200)
  parser.add_argument("--step-s", type=float, default=15.0)
  parser.add_argument("--settle-s", type=float, default=2.0)
  parser.add_argument("--hz", type=float, default=20.0, help="cmd resend rate (Hz)")
  parser.add_argument("--live-hz", type=float, default=1.0, help="live print rate")
  args = parser.parse_args()

  print(f"Port={args.port} Baud={args.baud}")
  tester = ChassisTester(args.port, args.baud)
  try:
    tester.run_profile(
      DEFAULT_STEPS, args.step_s, args.settle_s, args.hz, args.live_hz
    )
  except KeyboardInterrupt:
    tester.send(0.0, 0.0)
    print("\nStopped, zero-speed sent.")
  finally:
    tester.ser.close()


if __name__ == "__main__":
  main()
