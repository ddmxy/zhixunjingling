#!/usr/bin/env python3
"""Publish named arm poses to /joint_states (needs arm_serial_node running).

Poses match wheeltec_color_sort color_pick_node (direct mode, no MoveIt).

Usage:
  ros2 launch wheeltec_arm_bridge arm_serial_only.launch.py port:=/dev/ttyACM1
  python3 arm_pose_cmd.py arm_home     # 待机/运输
  python3 arm_pose_cmd.py arm_look     # 低头观察(视觉对准前)
  python3 arm_pose_cmd.py hand_open    # 张爪
  python3 arm_pose_cmd.py hand_close   # 合爪(夹一次)
  python3 arm_pose_cmd.py grab         # 手动抓取: 合爪 -> 抬回 arm_home
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ALL_JOINTS = [
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5",
    "joint_6", "joint_10", "joint_7", "joint_11", "joint_8", "joint_9",
]

# wheeltec_table_arm init_joint_states / arm_serial_node INIT_ANGLES
ARM_SERIAL_INIT = [0.0, 0.0, 0.0, -1.57, 0.0]
ARM_POSES = {
    "arm_home": [0.0, 0.0, 0.0, 0.0, 0.0],
    "arm_look": [0.0, 0.42, -1.57, -1.57, 0.0],
}

HAND_POSES = {
    "hand_open": {
        "joint_10": -0.2, "joint_11": 0.2, "joint_6": 0.2, "joint_7": 0.2,
        "joint_9": -0.2, "joint_8": 0.2,
    },
    "hand_close": {
        "joint_10": 0.8, "joint_11": -0.8, "joint_6": 0.8, "joint_7": 0.8,
        "joint_9": 0.8, "joint_8": 0.8,
    },
}


class ArmPoseCmd(Node):
    def __init__(self, publish_rate: float = 50.0, move_duration: float = 3.0) -> None:
        super().__init__("arm_pose_cmd")
        self.publish_rate = publish_rate
        self.move_duration = move_duration
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        # Match STM32 wake pose (joint_4=-90 deg). Do NOT start all-zero.
        self.positions = {name: 0.0 for name in ALL_JOINTS}
        for i, v in enumerate(ARM_SERIAL_INIT, 1):
            self.positions[f"joint_{i}"] = v
        self.positions.update({k: float(v) for k, v in HAND_POSES["hand_open"].items()})

    def _publish_once(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        msg.position = [float(self.positions[name]) for name in ALL_JOINTS]
        self.pub.publish(msg)

    def _hold(self, duration: float) -> None:
        end = time.monotonic() + duration
        period = 1.0 / self.publish_rate
        while time.monotonic() < end and rclpy.ok():
            self._publish_once()
            time.sleep(period)

    def move_arm(self, pose_name: str) -> None:
        if pose_name not in ARM_POSES:
            raise ValueError(f"unknown pose {pose_name}, choose {list(ARM_POSES)}")
        target = ARM_POSES[pose_name]
        start = [self.positions[f"joint_{i}"] for i in range(1, 6)]
        steps = max(int(self.move_duration * self.publish_rate), 1)
        period = 1.0 / self.publish_rate
        self.get_logger().info(f"moving -> {pose_name} {target}")
        for step in range(1, steps + 1):
            t = step / steps
            for i in range(5):
                self.positions[f"joint_{i + 1}"] = start[i] + (target[i] - start[i]) * t
            self._publish_once()
            time.sleep(period)
        self._hold(1.5)
        self.get_logger().info(f"done {pose_name}")

    def set_hand(self, hand_name: str, hold: float = 1.5) -> None:
        self.positions.update({k: float(v) for k, v in HAND_POSES[hand_name].items()})
        self.get_logger().info(f"hand -> {hand_name}")
        hold_t = 2.0 if hand_name == "hand_close" else hold
        self._hold(hold_t)

    def grab(self) -> None:
        """手动抓取: 在当前(arm_look/已下探)姿态合爪 -> 抬回 arm_home 保持夹持."""
        self.set_hand("hand_close", hold=2.0)
        self.move_arm("arm_home")
        self._hold(1.0)
        self.get_logger().info("grab done (夹住并回到 arm_home)")


def main() -> None:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "arm_look"
    rclpy.init()
    node = ArmPoseCmd()
    time.sleep(0.5)  # let arm_serial_node subscribe
    try:
        if cmd == "grab":
            node.grab()
        elif cmd in HAND_POSES:
            node.set_hand(cmd)
        else:
            node.move_arm(cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
