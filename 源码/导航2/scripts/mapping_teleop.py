#!/usr/bin/env python3
"""Slow keyboard teleop for SLAM mapping — continuous /cmd_vel at 10 Hz.

Press i/,/j/l once to start motion; k or space to stop. Do NOT arc: straight, stop, turn, stop.

Usage on RDK (mapping launch running):
  source ~/Desktop/ros2_ws/install/setup.bash
  python3 ~/Desktop/mapping_teleop.py
"""
from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = """
Mapping teleop (slow, continuous 10 Hz)
---------------------------------------
   i   : forward (until stop)
   ,   : backward
   j/l : turn left / right IN PLACE (linear forced to 0)
   k / space : STOP
   q/z : +/- linear speed step
   u/o : +/- angular speed step
   h   : help
   Ctrl+C : quit

IMPORTANT for SLAM:
   1) Only ONE motion at a time: straight -> STOP -> turn -> STOP
   2) First lap only; if walls look OK, do ONE more lap max
   3) Keep angular <= 0.18 rad/s while mapping
"""

LIN_STEP = 0.02
ANG_STEP = 0.03
LIN_MAX = 0.20
ANG_MAX = 0.25
LIN_DEFAULT = 0.10
ANG_DEFAULT = 0.15
PUB_HZ = 10.0


class MappingTeleop(Node):
    def __init__(self) -> None:
        super().__init__("mapping_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.lin = LIN_DEFAULT
        self.ang = ANG_DEFAULT
        self.vx_cmd = 0.0
        self.wz_cmd = 0.0
        self.create_timer(1.0 / PUB_HZ, self._tick)

    def _tick(self) -> None:
        msg = Twist()
        msg.linear.x = self.vx_cmd
        msg.angular.z = self.wz_cmd
        self.pub.publish(msg)

    def stop(self) -> None:
        self.vx_cmd = 0.0
        self.wz_cmd = 0.0
        self._tick()

    def status(self) -> str:
        mode = "STOP"
        if self.vx_cmd > 0.0:
            mode = "FWD"
        elif self.vx_cmd < 0.0:
            mode = "BACK"
        elif self.wz_cmd > 0.0:
            mode = "TURN-L"
        elif self.wz_cmd < 0.0:
            mode = "TURN-R"
        return (
            f"[{mode}]  target linear={self.lin:.2f} m/s  "
            f"angular={self.ang:.2f} rad/s  (pub {PUB_HZ:.0f} Hz)"
        )


def read_key(timeout_s: float = 0.05) -> str | None:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if not ready:
            return None
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> int:
    rclpy.init()
    node = MappingTeleop()
    print(HELP)
    print(node.status())

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(0.05)
            if key is None:
                continue
            if key in ("\x03", "\x1b"):
                break
            if key == "h":
                print(HELP)
                print(node.status())
                continue
            if key in ("k", " "):
                node.stop()
                print("stop")
                continue
            if key == "q":
                node.lin = min(LIN_MAX, node.lin + LIN_STEP)
            elif key == "z":
                node.lin = max(0.04, node.lin - LIN_STEP)
            elif key == "u":
                node.ang = min(ANG_MAX, node.ang + ANG_STEP)
            elif key == "o":
                node.ang = max(0.08, node.ang - ANG_STEP)
            elif key == "i":
                node.vx_cmd = node.lin
                node.wz_cmd = 0.0
            elif key == ",":
                node.vx_cmd = -node.lin
                node.wz_cmd = 0.0
            elif key == "j":
                node.vx_cmd = 0.0
                node.wz_cmd = node.ang
            elif key == "l":
                node.vx_cmd = 0.0
                node.wz_cmd = -node.ang
            else:
                continue
            print(node.status())
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
