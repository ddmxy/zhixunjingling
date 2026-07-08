"""
Single serial to lower controller (115200).

Protocol is defined by the STM32 code in HARDWARE/Car.c and USER/main.c:

TX (upper -> lower):
- FF 02 v(float32) w(float32) FE

RX (lower -> upper):
- FF 03 vx(float32) vy(float32) FE
- FF 04 yaw(float32) w(float32) FE

One port — avoid double-open.
"""

from __future__ import annotations

import struct
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

try:
    import serial
except ImportError as e:  # pragma: no cover
    serial = None  # type: ignore
    _IMPORT_ERROR = e
else:
    _IMPORT_ERROR = None

TX_HEADER = bytes([0xFF, 0x02])
FOOTER = bytes([0xFE])
HDR_VEL = bytes([0xFF, 0x03])
HDR_YAW = bytes([0xFF, 0x04])
LEN_RX_FRAME = 11  # FF + type + 8 payload + FE


class ChassisSerialNode(Node):
    def __init__(self) -> None:
        super().__init__("chassis_serial_node")

        self.declare_parameter("serial_port", "/dev/ttyUSB1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("min_send_interval_sec", 0.02)

        if _IMPORT_ERROR is not None:
            self.get_logger().fatal(f"pyserial missing: {_IMPORT_ERROR}")
            raise RuntimeError("pyserial required") from _IMPORT_ERROR

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("baudrate").get_parameter_value().integer_value
        if baud <= 0:
            baud = 115200
        self._min_interval = self.get_parameter("min_send_interval_sec").get_parameter_value().double_value
        topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value

        self._ser = serial.Serial(port, baud, timeout=0.05)
        self._buf = bytearray()
        self._last_send: Optional[rclpy.time.Time] = None

        self.create_subscription(Twist, topic, self._on_twist, 10)
        self._pub_speed = self.create_publisher(Float32, "chassis/speed_m_s", 10)
        self._pub_vx = self.create_publisher(Float32, "chassis/vx_m_s", 10)
        self._pub_vy = self.create_publisher(Float32, "chassis/vy_m_s", 10)
        self._pub_heading = self.create_publisher(Float32, "chassis/heading_rad", 10)
        self._pub_w = self.create_publisher(Float32, "chassis/w_rad_s", 10)

        self.create_timer(0.01, self._poll_serial)

        self.get_logger().info(
            f"chassis_serial_node: {port} @ {baud} | sub {topic} -> FF02 v w FE | "
            f"RX FF03 vx+vy, FF04 yaw+w"
        )

    def _pack_cmd(self, v_m_s: float, w_rad_s: float) -> bytes:
        return TX_HEADER + struct.pack("<ff", float(v_m_s), float(w_rad_s)) + FOOTER

    def _on_twist(self, msg: Twist) -> None:
        now = self.get_clock().now()
        if self._last_send is not None:
            dt = (now - self._last_send).nanoseconds * 1e-9
            if dt < self._min_interval:
                return
        self._last_send = now
        pkt = self._pack_cmd(float(msg.linear.x), float(msg.angular.z))
        self._ser.write(pkt)

    def _parse_buffer(self) -> None:
        while len(self._buf) >= 3:
            if self._buf[0] != 0xFF:
                del self._buf[0]
                continue
            if len(self._buf) < LEN_RX_FRAME:
                return
            if self._buf[LEN_RX_FRAME - 1] != 0xFE:
                del self._buf[0]
                continue

            typ = self._buf[1]
            payload = bytes(self._buf[2:10])

            if typ == 0x03:
                vx, vy = struct.unpack("<ff", payload)
                self._pub_vx.publish(Float32(data=float(vx)))
                self._pub_vy.publish(Float32(data=float(vy)))
                speed = float((vx * vx + vy * vy) ** 0.5)
                self._pub_speed.publish(Float32(data=speed))
                del self._buf[:LEN_RX_FRAME]
                continue

            if typ == 0x04:
                yaw, w = struct.unpack("<ff", payload)
                self._pub_heading.publish(Float32(data=float(yaw)))
                self._pub_w.publish(Float32(data=float(w)))
                del self._buf[:LEN_RX_FRAME]
                continue

            del self._buf[0]

    def _poll_serial(self) -> None:
        try:
            n = self._ser.in_waiting
            if n:
                data = self._ser.read(n)
                if data:
                    self._buf.extend(data)
                    self._parse_buffer()
        except serial.SerialException as ex:
            self.get_logger().error(f"serial: {ex}")

    def destroy_node(self) -> bool:
        try:
            self._ser.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ChassisSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
