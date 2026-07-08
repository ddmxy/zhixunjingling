#!/usr/bin/env python3
import argparse
import math
import struct
import sys
import threading
import time
from typing import Optional, Tuple

try:
    import msvcrt  # Windows console input
except ImportError:
    msvcrt = None

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial")
    print("Install with: pip install pyserial")
    sys.exit(1)


SOF = 0xFF
EOF = 0xFE
CMD_TX = 0x02
CMD_RX_VEL = 0x03
CMD_RX_YAW = 0x04

FRAME_LEN = 11
MAX_LINEAR = 0.5
MAX_ANGULAR = 1.5


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class SerialProbe:
    def __init__(self, port: str, baud: int, hold_timeout: float):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)
        self.hold_timeout = hold_timeout

        self.level = 3
        self.level_to_v = {
            1: 0.10,
            2: 0.20,
            3: 0.30,
            4: 0.40,
            5: 0.50,
        }
        self.level_to_w = {
            1: 0.30,
            2: 0.60,
            3: 0.90,
            4: 1.20,
            5: 1.50,
        }

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_press_t = 0.0
        self.last_sent: Optional[Tuple[float, float]] = None
        self.stop_flag = False

        self.rx_state = 0
        self.rx_cmd = 0
        self.rx_payload = bytearray()

        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_yaw = 0.0
        self.last_w = 0.0

        # Odom state (for quick validation before ROS migration)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_odom_t = time.time()
        self.last_odom_print_t = 0.0

    def make_frame(self, v: float, w: float) -> bytes:
        v = clamp(v, -MAX_LINEAR, MAX_LINEAR)
        w = clamp(w, -MAX_ANGULAR, MAX_ANGULAR)
        return bytes([SOF, CMD_TX]) + struct.pack("<f", v) + struct.pack("<f", w) + bytes([EOF])

    def send_cmd(self, v: float, w: float, force: bool = False) -> None:
        v = clamp(v, -MAX_LINEAR, MAX_LINEAR)
        w = clamp(w, -MAX_ANGULAR, MAX_ANGULAR)
        pair = (round(v, 4), round(w, 4))
        if (not force) and self.last_sent == pair:
            return

        frame = self.make_frame(v, w)
        self.ser.write(frame)
        self.last_sent = pair
        print(f"[TX] v={v:+.3f} m/s, w={w:+.3f} rad/s")

    def on_key(self, key: str) -> None:
        now = time.time()

        if key in ("q", "Q"):
            self.stop_flag = True
            return

        if key in ("r", "R"):
            self.reset_odom()
            print("[ODOM] reset to x=0.000, y=0.000, theta=0.000")
            return

        if key in ("1", "2", "3", "4", "5"):
            self.level = int(key)
            print(
                f"[MODE] level={self.level}, "
                f"linear={self.level_to_v[self.level]:.2f} m/s, "
                f"angular={self.level_to_w[self.level]:.2f} rad/s"
            )
            return

        v_mag = self.level_to_v[self.level]
        w_mag = self.level_to_w[self.level]

        # Hold-to-run: rely on keyboard repeat while key is held.
        # If no repeat within hold_timeout, command returns to zero.
        if key in ("i", "I"):
            self.cmd_v = +v_mag
            self.cmd_w = 0.0
            self.last_press_t = now
        elif key in ("m", "M"):
            self.cmd_v = -v_mag
            self.cmd_w = 0.0
            self.last_press_t = now
        elif key in ("<", ","):
            self.cmd_v = 0.0
            self.cmd_w = +w_mag
            self.last_press_t = now
        elif key in (">", "."):
            self.cmd_v = 0.0
            self.cmd_w = -w_mag
            self.last_press_t = now
        elif key == " ":
            self.cmd_v = 0.0
            self.cmd_w = 0.0
            self.last_press_t = now

    def keyboard_loop(self) -> None:
        if msvcrt is None:
            print("This script expects Windows console (msvcrt not available).")
            self.stop_flag = True
            return

        while not self.stop_flag:
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                self.on_key(ch)
            time.sleep(0.005)

    def wrap_angle(self, ang: float) -> float:
        while ang > 3.141592653589793:
            ang -= 6.283185307179586
        while ang < -3.141592653589793:
            ang += 6.283185307179586
        return ang

    def reset_odom(self) -> None:
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_odom_t = time.time()

    def update_odom(self, now: float) -> None:
        dt = now - self.last_odom_t
        if dt <= 0.0:
            return
        self.last_odom_t = now

        # Use IMU yaw as heading to reduce drift in short-term tests.
        self.odom_theta = self.wrap_angle(self.last_yaw)
        c = math.cos(self.odom_theta)
        s = math.sin(self.odom_theta)
        self.odom_x += (self.last_vx * c - self.last_vy * s) * dt
        self.odom_y += (self.last_vx * s + self.last_vy * c) * dt

        if now - self.last_odom_print_t > 0.2:  # 5 Hz print
            self.last_odom_print_t = now
            print(
                f"[ODOM] x={self.odom_x:+.3f} m, y={self.odom_y:+.3f} m, "
                f"theta={self.odom_theta:+.3f} rad"
            )

    def parse_rx_byte(self, b: int) -> None:
        if self.rx_state == 0:
            if b == SOF:
                self.rx_state = 1
            return

        if self.rx_state == 1:
            if b in (CMD_RX_VEL, CMD_RX_YAW):
                self.rx_cmd = b
                self.rx_payload.clear()
                self.rx_state = 2
            else:
                self.rx_state = 0
            return

        if self.rx_state == 2:
            self.rx_payload.append(b)
            if len(self.rx_payload) >= 8:
                self.rx_state = 3
            return

        if self.rx_state == 3:
            if b == EOF and len(self.rx_payload) == 8:
                a = struct.unpack("<f", self.rx_payload[0:4])[0]
                c = struct.unpack("<f", self.rx_payload[4:8])[0]
                if self.rx_cmd == CMD_RX_VEL:
                    self.last_vx = a
                    self.last_vy = c
                else:
                    self.last_yaw = a
                    self.last_w = c
                print(
                    f"[RX] vx={self.last_vx:+.3f} m/s, vy={self.last_vy:+.3f} m/s, "
                    f"w={self.last_w:+.3f} rad/s, yaw={self.last_yaw:+.3f} rad"
                )
            self.rx_state = 0

    def run(self) -> None:
        print("==== Serial Teleop Probe ====")
        print(f"Port={self.ser.port}, Baud={self.ser.baudrate}")
        print("Keys:")
        print("  i = forward, m = backward")
        print("  < or , = rotate left, > or . = rotate right")
        print("  1..5 = speed level, r = reset odom, space = stop, q = quit")
        print("Hold key -> keep speed; release -> zero")
        print("")

        # Start handshake: let MCU begin feedback upload.
        self.send_cmd(0.0, 0.0, force=True)

        kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        kb_thread.start()

        try:
            while not self.stop_flag:
                now = time.time()
                if now - self.last_press_t > self.hold_timeout:
                    self.cmd_v = 0.0
                    self.cmd_w = 0.0

                self.send_cmd(self.cmd_v, self.cmd_w)

                data = self.ser.read(128)
                for x in data:
                    self.parse_rx_byte(x)

                self.update_odom(now)

                time.sleep(0.02)
        finally:
            try:
                self.send_cmd(0.0, 0.0, force=True)
            except Exception:
                pass
            self.ser.close()
            print("Exit: sent zero-speed and closed serial.")


def main() -> None:
    parser = argparse.ArgumentParser(description="STM32 serial teleop + packet probe")
    parser.add_argument("--port", default="COM3", help="Serial port, e.g. COM3")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate")
    parser.add_argument(
        "--hold-timeout",
        type=float,
        default=0.18,
        help="Seconds before auto-stop after key repeat stops",
    )
    args = parser.parse_args()

    probe = SerialProbe(port=args.port, baud=args.baud, hold_timeout=args.hold_timeout)
    probe.run()


if __name__ == "__main__":
    main()

