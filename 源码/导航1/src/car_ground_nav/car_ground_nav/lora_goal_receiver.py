"""Subscribe to LoRa serial (same 11-byte protocol as uwb_qr_lora air side) and publish UAV goal pose."""

from __future__ import annotations

import struct
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

try:
    import serial
except ImportError as e:  # pragma: no cover
    serial = None  # type: ignore
    _IMPORT_ERROR = e
else:
    _IMPORT_ERROR = None

HEADER = bytes([0xFF, 0x01])
FOOTER = bytes([0xFE])
FRAME_LEN = 11


class LoraGoalReceiver(Node):
    def __init__(self) -> None:
        super().__init__("lora_goal_receiver")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("endian", "little")
        self.declare_parameter("max_goal_publishes", 3)

        if _IMPORT_ERROR is not None:
            self.get_logger().fatal(f"pyserial missing: {_IMPORT_ERROR}. Install: pip install pyserial")
            raise RuntimeError("pyserial required") from _IMPORT_ERROR

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("baudrate").get_parameter_value().integer_value
        if baud <= 0:
            baud = 115200
        self._max_publishes = self.get_parameter("max_goal_publishes").get_parameter_value().integer_value
        if self._max_publishes <= 0:
            self._max_publishes = 3
        self._publish_count = 0

        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        endian = self.get_parameter("endian").get_parameter_value().string_value
        self._fmt = "<ff" if endian == "little" else ">ff"

        self._pub = self.create_publisher(PoseStamped, "uav_goal", 10)
        self._buffer = bytearray()

        try:
            self._ser = serial.Serial(port, baud, timeout=0.05)
        except serial.SerialException as ex:
            self.get_logger().error(f"Cannot open {port}: {ex}")
            raise

        self.get_logger().info(
            f"LoRa goal receiver on {port} @ {baud}, uav_goal frame_id={self._frame_id}, "
            f"max_goal_publishes={self._max_publishes}"
        )
        self.create_timer(0.02, self._poll_serial)

    def _try_parse_one(self, buf: bytes) -> Optional[tuple[float, float]]:
        if len(buf) < FRAME_LEN:
            return None
        if buf[:2] != HEADER:
            return None
        if buf[10:11] != FOOTER:
            return None
        x, y = struct.unpack(self._fmt, buf[2:10])
        if not (x == x and y == y):  # NaN check
            return None
        return float(x), float(y)

    def _feed(self, data: bytes) -> None:
        self._buffer.extend(data)
        while True:
            start = self._buffer.find(HEADER)
            if start < 0:
                if len(self._buffer) > 2:
                    self._buffer = self._buffer[-2:]
                break
            if start > 0:
                del self._buffer[:start]
            if len(self._buffer) < FRAME_LEN:
                break
            chunk = bytes(self._buffer[:FRAME_LEN])
            parsed = self._try_parse_one(chunk)
            if parsed is not None:
                x, y = parsed
                if self._publish_count < self._max_publishes:
                    msg = PoseStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self._frame_id
                    msg.pose.position.x = x
                    msg.pose.position.y = y
                    msg.pose.position.z = 0.0
                    msg.pose.orientation.w = 1.0
                    self._pub.publish(msg)
                    self._publish_count += 1
                    self.get_logger().info(
                        f"uav_goal [{self._publish_count}/{self._max_publishes}] x={x:.3f} y={y:.3f}"
                    )
                del self._buffer[:FRAME_LEN]
            else:
                del self._buffer[0:1]

    def _poll_serial(self) -> None:
        try:
            if self._ser.in_waiting:
                data = self._ser.read(self._ser.in_waiting)
                if data:
                    self._feed(data)
        except serial.SerialException as ex:
            self.get_logger().error(f"serial read error: {ex}")

    def destroy_node(self) -> bool:
        try:
            self._ser.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = LoraGoalReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
