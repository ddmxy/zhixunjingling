"""
UWB serial: same protocol as uwb_qr_lora (HEADER CmdM:4, 101-byte frames) -> 2D position.
Publishes geometry_msgs/PoseStamped on uwb_pose_raw for uwb_filtered_odom.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

try:
    import serial
except ImportError as e:  # pragma: no cover
    serial = None  # type: ignore
    _IMPORT_ERROR = e
else:
    _IMPORT_ERROR = None

# --- copied from uwb_qr_lora/app.py ---
HEADER = b"CmdM:4"
HEADER_LEN = 6
PAYLOAD_LEN = 91
FRAME_LEN = 101
FOOTER = b"\r\n"


@dataclass
class UsbFrame:
    seq: int
    mask: int
    raw_ranges_mm: List[int]
    kalman_enable: bool
    kalman_ranges_mm: List[int]


class UsbStreamParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    @staticmethod
    def _u32_le(b: bytes) -> int:
        return int.from_bytes(b, "little", signed=False)

    def _parse_one(self, buf: bytes) -> Optional[UsbFrame]:
        if len(buf) < FRAME_LEN:
            return None
        if buf[:HEADER_LEN] != HEADER:
            return None
        if buf[6] != PAYLOAD_LEN:
            return None
        if buf[99:101] != FOOTER:
            return None

        idx = 7
        idx += 4
        idx += 2
        idx += 2
        seq = buf[idx]
        idx += 1
        mask = buf[idx]
        idx += 1

        raw = []
        for _ in range(8):
            raw.append(self._u32_le(buf[idx : idx + 4]))
            idx += 4

        kalman_enable = buf[idx] != 0
        idx += 1

        kalman = []
        for _ in range(8):
            kalman.append(self._u32_le(buf[idx : idx + 4]))
            idx += 4

        return UsbFrame(
            seq=seq,
            mask=mask,
            raw_ranges_mm=raw,
            kalman_enable=kalman_enable,
            kalman_ranges_mm=kalman,
        )

    def feed(self, data: bytes) -> List[UsbFrame]:
        self.buffer.extend(data)
        out: List[UsbFrame] = []

        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                if len(self.buffer) > HEADER_LEN:
                    self.buffer = self.buffer[-(HEADER_LEN - 1) :]
                break

            if start > 0:
                self.buffer = self.buffer[start:]

            if len(self.buffer) < FRAME_LEN:
                break

            frame_bytes = bytes(self.buffer[:FRAME_LEN])
            frame = self._parse_one(frame_bytes)
            if frame is not None:
                out.append(frame)
                self.buffer = self.buffer[FRAME_LEN:]
            else:
                self.buffer = self.buffer[1:]

        return out


class Uwb2DLocalizer:
    def __init__(
        self,
        anchors: Dict[int, Tuple[float, float]],
        fixed_triplet: Tuple[int, int, int],
        use_kalman: bool = True,
        calib_a: float = 1.0,
        calib_b_mm: float = 0.0,
    ) -> None:
        self.anchors = anchors
        self.fixed_triplet = fixed_triplet
        self.use_kalman = use_kalman
        self.calib_a = calib_a
        self.calib_b_mm = calib_b_mm

    def _mm_to_m(self, d_mm: int) -> float:
        return (self.calib_a * float(d_mm) + self.calib_b_mm) / 1000.0

    def solve(self, frame: UsbFrame) -> Optional[Tuple[float, float, float]]:
        ranges = frame.kalman_ranges_mm if (self.use_kalman and frame.kalman_enable) else frame.raw_ranges_mm
        i, j, k = self.fixed_triplet
        ids = [i, j, k]

        for anchor_id in ids:
            if ((frame.mask >> anchor_id) & 0x01) == 0:
                return None
            if anchor_id not in self.anchors:
                return None
            if ranges[anchor_id] <= 0:
                return None

        d1 = self._mm_to_m(ranges[i])
        d2 = self._mm_to_m(ranges[j])
        d3 = self._mm_to_m(ranges[k])

        x1, y1 = self.anchors[i]
        x2, y2 = self.anchors[j]
        x3, y3 = self.anchors[k]

        a1 = 2.0 * (x1 - x3)
        b1 = 2.0 * (y1 - y3)
        r1 = x1 * x1 - x3 * x3 + y1 * y1 - y3 * y3 + d3 * d3 - d1 * d1

        a2 = 2.0 * (x2 - x3)
        b2 = 2.0 * (y2 - y3)
        r2 = x2 * x2 - x3 * x3 + y2 * y2 - y3 * y3 + d3 * d3 - d2 * d2

        det = a1 * b2 - a2 * b1
        if abs(det) < 1e-9:
            return None

        x = (r1 * b2 - r2 * b1) / det
        y = (r2 * a1 - r1 * a2) / det

        if not math.isfinite(x) or not math.isfinite(y):
            return None

        residual = 0.0
        for anchor_id in ids:
            ax, ay = self.anchors[anchor_id]
            est = math.hypot(x - ax, y - ay)
            real = self._mm_to_m(ranges[anchor_id])
            residual += abs(est - real)
        residual /= 3.0

        return x, y, residual


def _load_anchors_from_json(path: str) -> Tuple[Dict[int, Tuple[float, float]], Tuple[int, int, int], bool, float, float]:
    p = Path(path)
    with p.open("r", encoding="utf-8") as f:
        cfg = json.load(f)
    anchors = {int(k): (float(v[0]), float(v[1])) for k, v in cfg["anchors"].items()}
    tri = tuple(cfg.get("fixed_triplet", [0, 1, 2]))
    if len(tri) != 3:
        raise ValueError("fixed_triplet must have 3 elements")
    ft = (int(tri[0]), int(tri[1]), int(tri[2]))
    use_k = bool(cfg.get("use_kalman", True))
    ca = float(cfg.get("calib_a", 1.0))
    cb = float(cfg.get("calib_b_mm", 0.0))
    return anchors, ft, use_k, ca, cb


class UwbSerialLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("uwb_serial_localizer")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("anchor_config_file", "")
        self.declare_parameter(
            "anchors_json",
            '{"0":[0.0,0.0],"1":[4.0,0.0],"2":[0.0,3.0]}',
        )
        self.declare_parameter("fixed_triplet", [0, 1, 2])
        self.declare_parameter("use_kalman", True)
        self.declare_parameter("calib_a", 1.0)
        self.declare_parameter("calib_b_mm", 0.0)
        self.declare_parameter("publish_residual_topic", True)

        if _IMPORT_ERROR is not None:
            self.get_logger().fatal(f"pyserial missing: {_IMPORT_ERROR}")
            raise RuntimeError("pyserial required") from _IMPORT_ERROR

        cfg_path = self.get_parameter("anchor_config_file").get_parameter_value().string_value
        if cfg_path:
            anchors, ft, use_k, ca, cb = _load_anchors_from_json(cfg_path)
            self.get_logger().info(f"Loaded UWB anchors from {cfg_path}")
        else:
            js = self.get_parameter("anchors_json").get_parameter_value().string_value
            data = json.loads(js)
            anchors = {int(k): (float(v[0]), float(v[1])) for k, v in data.items()}
            ft_list = list(self.get_parameter("fixed_triplet").get_parameter_value().integer_array_value)
            if len(ft_list) != 3:
                raise ValueError("fixed_triplet must have 3 integers")
            ft = (int(ft_list[0]), int(ft_list[1]), int(ft_list[2]))
            use_k = self.get_parameter("use_kalman").get_parameter_value().bool_value
            ca = self.get_parameter("calib_a").get_parameter_value().double_value
            cb = self.get_parameter("calib_b_mm").get_parameter_value().double_value

        self._solver = Uwb2DLocalizer(
            anchors=anchors,
            fixed_triplet=ft,
            use_kalman=use_k,
            calib_a=ca,
            calib_b_mm=cb,
        )
        self._parser = UsbStreamParser()

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("baudrate").get_parameter_value().integer_value
        if baud <= 0:
            baud = 115200
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self._pub = self.create_publisher(PoseStamped, "uwb_pose_raw", 50)
        self._pub_res = None
        if self.get_parameter("publish_residual_topic").get_parameter_value().bool_value:
            self._pub_res = self.create_publisher(Float32, "uwb_residual_m", 10)

        self._ser = serial.Serial(port, baud, timeout=0.05)
        self.get_logger().info(
            f"uwb_serial_localizer: {port} @ {baud} -> uwb_pose_raw (frame={self._frame_id})"
        )
        self.create_timer(0.02, self._poll)

    def _poll(self) -> None:
        try:
            if self._ser.in_waiting:
                data = self._ser.read(self._ser.in_waiting)
                if not data:
                    return
                for frame in self._parser.feed(data):
                    solved = self._solver.solve(frame)
                    if solved is None:
                        continue
                    x, y, res = solved
                    msg = PoseStamped()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self._frame_id
                    msg.pose.position.x = x
                    msg.pose.position.y = y
                    msg.pose.position.z = 0.0
                    msg.pose.orientation.w = 1.0
                    self._pub.publish(msg)
                    if self._pub_res is not None:
                        self._pub_res.publish(Float32(data=float(res)))
        except serial.SerialException as ex:
            self.get_logger().error(f"uwb serial: {ex}")

    def destroy_node(self) -> bool:
        try:
            self._ser.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = UwbSerialLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
