import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
import serial

FRAME_HEAD = 0xAA
FRAME_TAIL = 0xBB
MODE_DEFAULT = 1
MODE_FOLLOWER = 2
# Official wheeltec_table_arm init pose on startup
INIT_ANGLES = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0]
# Serial frame carries 6 angles: arm joint_1..5 + joint_6 (gripper/wrist)
SERIAL_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def xor_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def build_frame(angles, mode=1):
    frame = bytearray(16)
    frame[0] = FRAME_HEAD
    for index, angle in enumerate(angles):
        value = int(angle * 1000)
        frame[1 + index * 2] = (value >> 8) & 0xFF
        frame[2 + index * 2] = value & 0xFF
    frame[13] = mode
    frame[14] = xor_checksum(frame[:14])
    frame[15] = FRAME_TAIL
    return frame


class ArmSerialNode(Node):
    def __init__(self):
        super().__init__("arm_serial_node")
        self.port = self.declare_parameter("port", "/dev/ttyACM0").value
        self.baud = self.declare_parameter("baud", 115200).value
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.last_angles = list(INIT_ANGLES)
        self.rx_count = 0
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(JointState, "/joint_states", self.joint_callback, qos)
        self.get_logger().info(f"Serial open: {self.port} @ {self.baud}, listening on /joint_states")
        self._send_init_pose()

    def _write_frame(self, angles, mode):
        frame = build_frame(angles, mode=mode)
        self.ser.write(frame)
        self.ser.flush()
        return frame

    def _send_init_pose(self):
        """Match official wheeltec_table_arm::init_joint_states() wake-up frame."""
        frame = self._write_frame(INIT_ANGLES, MODE_FOLLOWER)
        hex_preview = frame.hex(" ")
        self.get_logger().info(
            f"sent STM32 init frame (mode={MODE_FOLLOWER}): {hex_preview}"
        )
        time.sleep(0.2)

    def joint_callback(self, msg):
        self.rx_count += 1
        if self.rx_count == 1:
            self.get_logger().info(
                f"first /joint_states: {[round(float(p), 3) for p in msg.position[:6]]}"
            )
        elif self.rx_count % 200 == 0:
            self.get_logger().info(f"/joint_states active, frames={self.rx_count}")
        name_to_pos = {n: msg.position[i] for i, n in enumerate(msg.name)}
        angles = [float(name_to_pos.get(name, self.last_angles[i])) for i, name in enumerate(SERIAL_JOINTS)]
        self.last_angles = angles
        try:
            if self.rx_count == 1:
                hex_preview = self._write_frame(angles, MODE_DEFAULT).hex(" ")
                self.get_logger().info(f"first command frame (mode={MODE_DEFAULT}): {hex_preview}")
            else:
                self._write_frame(angles, MODE_DEFAULT)
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write failed: {exc}")


def main():
    rclpy.init()
    node = ArmSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
