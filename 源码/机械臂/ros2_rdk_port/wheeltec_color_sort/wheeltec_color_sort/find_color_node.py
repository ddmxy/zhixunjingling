#!/usr/bin/env python3
"""Detect one configured color and publish /color_ik_result."""
import math
import os
import socket
import threading
import time
from http import server
from math import acos, atan, degrees, radians, sqrt

import cv2
import numpy as np
import rclpy
from a150_arm_msgs.msg import ColorIkResult
from rclpy.node import Node
from sensor_msgs.msg import Image

COLOR_RANGES = {
    "green": (50, 46, 24, 80, 255, 255),
    "blue": (80, 33, 12, 127, 255, 255),
    "yellow": (20, 90, 110, 39, 255, 255),
}

COLOR_BGR = {
    "green": (0, 255, 0),
    "blue": (255, 128, 0),
    "yellow": (0, 255, 255),
}

PACKAGE_VERSION = "0.2.12"


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def imgmsg_to_bgr8(msg: Image) -> np.ndarray:
    if msg.encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"unsupported image encoding: {msg.encoding}")
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img.copy()


class _MjpegHandler(server.BaseHTTPRequestHandler):
    provider = None

    def log_message(self, _format, *_args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            page = (
                b"<html><head><title>Arm Vision</title></head>"
                b"<body style='margin:0;background:#000'>"
                b"<img src='/stream.mjpg' style='width:100%'></body></html>"
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(page)
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        while rclpy.ok():
            jpeg = _MjpegHandler.provider.get_jpeg() if _MjpegHandler.provider else None
            if jpeg is None:
                time.sleep(0.05)
                continue
            try:
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                break
            time.sleep(0.033)


def bgr8_to_imgmsg(img: np.ndarray, stamp, frame_id: str = "camera_frame") -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = img.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = img.tobytes()
    return msg


class FindColorNode(Node):
    def __init__(self):
        super().__init__("find_color")
        self.picture_height = 480
        self.picture_width = 640
        self.process_width = 320
        self.process_height = 240
        self.crop_y1 = int(self.picture_height / 16)
        self.crop_y2 = int(self.picture_height / 16 * 12)
        self.crop_x1 = int(self.picture_width / 16 * 4)
        self.crop_x2 = int(self.picture_width / 16 * 15)
        self.tan_horizontal = math.tan(0.5235987755982988)
        self.tan_vertical = math.tan(0.43196898986859655)

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("show_image", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("enable_web_view", False)
        self.declare_parameter("web_port", 8080)
        self.declare_parameter("target_color", "green")
        self.declare_parameter("link_a", 0.105)
        self.declare_parameter("link_b", 0.100)
        self.declare_parameter("link_c", 0.170)
        self.declare_parameter("link_h", 0.105)
        self.declare_parameter("auxiliary_angle", 0.20)
        self.declare_parameter("color_x_offset", 0.006)
        self.declare_parameter("color_y_offset", -0.005)
        self.declare_parameter("vision_x_scale", -0.354)
        self.declare_parameter("vision_x_base", 0.105)
        self.declare_parameter("vision_y_scale", 0.307)
        self.declare_parameter("vision_y_base", 0.17)
        self.declare_parameter("max_angle_x", 0.65)
        self.declare_parameter("max_angle_y", 0.55)
        self.declare_parameter("use_block_rotation", True)
        self.declare_parameter("hand_angle_offset", 0.0)
        self.declare_parameter("edge_x_gain", 0.0)
        self.declare_parameter("edge_y_gain", 0.0)

        self.target_color = self.get_parameter("target_color").get_parameter_value().string_value
        if self.target_color not in COLOR_RANGES:
            raise ValueError(f"target_color must be green|blue|yellow, got: {self.target_color}")

        self.link_a = self.get_parameter("link_a").value
        self.link_b = self.get_parameter("link_b").value
        self.link_c = self.get_parameter("link_c").value
        self.link_h = self.get_parameter("link_h").value
        self.auxiliary_angle = self.get_parameter("auxiliary_angle").value
        self.x_offset = self.get_parameter("color_x_offset").value
        self.y_offset = self.get_parameter("color_y_offset").value
        self.vision_x_scale = self.get_parameter("vision_x_scale").value
        self.vision_x_base = self.get_parameter("vision_x_base").value
        self.vision_y_scale = self.get_parameter("vision_y_scale").value
        self.vision_y_base = self.get_parameter("vision_y_base").value
        self.max_angle_x = float(self.get_parameter("max_angle_x").value)
        self.max_angle_y = float(self.get_parameter("max_angle_y").value)
        self.use_block_rotation = as_bool(self.get_parameter("use_block_rotation").value)
        self.hand_angle_offset = float(self.get_parameter("hand_angle_offset").value)
        self.edge_x_gain = float(self.get_parameter("edge_x_gain").value)
        self.edge_y_gain = float(self.get_parameter("edge_y_gain").value)
        self.show_image = as_bool(self.get_parameter("show_image").value)
        self.publish_debug_image = as_bool(self.get_parameter("publish_debug_image").value)
        self.enable_web_view = as_bool(self.get_parameter("enable_web_view").value)
        self.web_port = int(self.get_parameter("web_port").value)
        self.basic_angle = acos((self.link_c - self.link_h) / self.link_a)

        self.last_status = "waiting..."
        self.last_metrics = ""
        self._jpeg_lock = threading.Lock()
        self._latest_jpeg = None
        self._http_server = None

        image_topic = self.get_parameter("image_topic").value
        self.ik_pub = self.create_publisher(ColorIkResult, "/color_ik_result", 10)
        self.debug_pub = self.create_publisher(Image, "find_color/debug_image", 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.no_target_count = 0
        self.reject_count = 0
        if self.enable_web_view:
            self._start_web_server()
        if self.show_image:
            if not self._setup_opencv_window():
                self.get_logger().warn("OpenCV window failed, enable web view on port %d" % self.web_port)
                self.enable_web_view = True
                self._start_web_server()
        self.get_logger().info(
            f"find_color v{PACKAGE_VERSION} ready, target={self.target_color}, "
            f"image={image_topic}, show_image={self.show_image}, web={self.enable_web_view}, "
            f"use_block_rotation={self.use_block_rotation}"
        )

    def _setup_opencv_window(self) -> bool:
        os.environ.setdefault("DISPLAY", ":0")
        try:
            cv2.startWindowThread()
            cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Camera", 960, 720)
            test = np.zeros((100, 100, 3), dtype=np.uint8)
            cv2.imshow("Camera", test)
            cv2.waitKey(1)
            self.get_logger().info(f"OpenCV window OK, DISPLAY={os.environ.get('DISPLAY', '')}")
            return True
        except Exception as exc:
            self.get_logger().error(f"OpenCV window failed: {exc}")
            return False

    def get_jpeg(self):
        with self._jpeg_lock:
            return self._latest_jpeg

    def _update_web_frame(self, img: np.ndarray):
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        with self._jpeg_lock:
            self._latest_jpeg = buf.tobytes()

    def _guess_ip(self) -> str:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.connect(("8.8.8.8", 80))
            ip = sock.getsockname()[0]
            sock.close()
            return ip
        except OSError:
            return "127.0.0.1"

    def _start_web_server(self):
        _MjpegHandler.provider = self
        self._http_server = server.ThreadingHTTPServer(("0.0.0.0", self.web_port), _MjpegHandler)
        threading.Thread(target=self._http_server.serve_forever, daemon=True).start()
        ip = self._guess_ip()
        self.get_logger().info(f"WEB VIEW >>> http://{ip}:{self.web_port}/  (open in PC browser)")

    def image_callback(self, msg):
        image0 = imgmsg_to_bgr8(msg)
        image = image0[self.crop_y1 : self.crop_y2, self.crop_x1 : self.crop_x2]
        image = cv2.resize(image, (self.process_width, self.process_height), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)
        hsv = cv2.dilate(cv2.erode(hsv, kernel, iterations=1), kernel, iterations=1)

        lower = COLOR_RANGES[self.target_color][:3]
        upper = COLOR_RANGES[self.target_color][3:]
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=4)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        detected = False
        proc_vis = image.copy()
        box_pts = None
        center = None

        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            if cv2.contourArea(contour) < 800:
                break
            center, size, rotation = cv2.minAreaRect(contour)
            box_pts = cv2.boxPoints((center, size, rotation)).astype(int)
            detected = True
            self.no_target_count = 0
            self.reject_count = 0
            # Official WHEELTEC formula: crop pixel + full-frame size (empirically calibrated)
            ax = self.calculate_angle_x(center[0])
            ay = self.calculate_angle_y(center[1])
            hand_deg = self.block_rotation_to_hand_deg(rotation)
            metrics, status = self.publish_arm_angle(ax, ay, hand_deg, center)
            self.last_status = status
            self.last_metrics = metrics or f"ang_x={ax:.2f} ang_y={ay:.2f}"
            if metrics is None and "DETECTED" not in status:
                self.reject_count += 1
                if self.reject_count % 30 == 1:
                    self.get_logger().warn(f"block seen but not ready: {status}")
            break

        if not detected:
            self._log_no_target()
            self.last_status = f"no {self.target_color} block"
            self.last_metrics = ""

        if detected and box_pts is not None and center is not None:
            cv2.polylines(proc_vis, [box_pts], True, COLOR_BGR[self.target_color], 2)
            cx, cy = int(center[0]), int(center[1])
            cv2.circle(proc_vis, (cx, cy), 6, (0, 0, 255), -1)
            cv2.drawMarker(proc_vis, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 18, 2)
            if self.use_block_rotation:
                self._draw_text(proc_vis, f"rot={hand_deg:.0f}", (cx + 8, cy - 8), (0, 255, 255))

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        panel = np.hstack([proc_vis, mask_bgr])
        self._draw_text(panel, self.last_status, (8, 24), (0, 255, 0) if "DETECTED" in self.last_status else (0, 165, 255))
        if self.last_metrics:
            self._draw_text(panel, self.last_metrics, (8, 50), (255, 255, 255))

        full_vis = image0.copy()
        cv2.rectangle(full_vis, (self.crop_x1, self.crop_y1), (self.crop_x2, self.crop_y2), (0, 255, 0), 2)
        self._draw_text(full_vis, f"target: {self.target_color}", (10, 28), COLOR_BGR[self.target_color])
        self._draw_text(full_vis, self.last_status, (10, 56), (0, 255, 0) if "DETECTED" in self.last_status else (0, 165, 255))
        if detected and center is not None:
            scale_x = (self.crop_x2 - self.crop_x1) / self.process_width
            scale_y = (self.crop_y2 - self.crop_y1) / self.process_height
            fx = int(self.crop_x1 + center[0] * scale_x)
            fy = int(self.crop_y1 + center[1] * scale_y)
            cv2.drawMarker(full_vis, (fx, fy), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
            cv2.circle(full_vis, (fx, fy), 10, (0, 255, 255), 2)

        pip = cv2.resize(panel, (320, 120))
        full_vis[360:480, 0:320] = pip
        cv2.rectangle(full_vis, (0, 360), (320, 480), (255, 255, 255), 1)

        stamp = self.get_clock().now().to_msg()
        if self.publish_debug_image:
            self.debug_pub.publish(bgr8_to_imgmsg(full_vis, stamp))
        if self.enable_web_view:
            self._update_web_frame(full_vis)
        if self.show_image:
            try:
                cv2.imshow("Camera", full_vis)
                cv2.waitKey(1)
            except Exception as exc:
                self.get_logger().error(f"imshow failed: {exc}")

    def _draw_text(self, img, text, pos, color):
        cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

    def _log_no_target(self):
        self.no_target_count += 1
        if self.no_target_count % 50 == 0:
            self.get_logger().warn(f"no {self.target_color} block in camera view")

    def calculate_angle_x(self, crop_x):
        displacement = 2 * crop_x / self.picture_width - 1
        return -math.atan(displacement * self.tan_horizontal)

    def calculate_angle_y(self, crop_y):
        displacement = 2 * crop_y / self.picture_height - 1
        return -math.atan(displacement * self.tan_vertical)

    def block_rotation_to_hand_deg(self, rotation):
        """Wrist angle for joint_5 (same as official find_color.py)."""
        if not self.use_block_rotation:
            return self.hand_angle_offset
        hand = float(rotation) + 90.0
        if hand > 45.0:
            hand -= 90.0
        return hand + self.hand_angle_offset

    def publish_arm_angle(self, x, y, hand_deg, center=None):
        if abs(x) > self.max_angle_x or abs(y) > self.max_angle_y:
            return None, f"angle limit (move block to center) x={x:.2f} y={y:.2f}"

        true_x = x * self.vision_x_scale + self.vision_x_base + self.x_offset
        true_y = y * self.vision_y_scale + self.vision_y_base + self.y_offset
        if center is not None and (self.edge_x_gain != 0.0 or self.edge_y_gain != 0.0):
            norm_x = center[0] / self.process_width - 0.5
            norm_y = center[1] / self.process_height - 0.5
            true_x += norm_x * self.edge_x_gain
            true_y += norm_y * self.edge_y_gain
        if true_y < 0.05:
            return None, f"too close to arm base, y={true_y:.3f}m"

        pedestal_angle = degrees(atan(abs(true_x / true_y)))
        if true_x <= 0:
            pedestal_angle = -pedestal_angle

        calc_a = self.link_a * math.sin(self.basic_angle) + math.sin(self.auxiliary_angle) * self.link_c
        calc_b = self.link_a * math.cos(self.basic_angle) + math.cos(self.auxiliary_angle) * self.link_c
        calc_c = sqrt(true_x ** 2 + true_y ** 2) - self.link_b
        reach = sqrt(calc_a ** 2 + calc_b ** 2)
        if reach < 1e-6:
            return None, "IK error: zero reach"
        cos_d = calc_c / reach
        if cos_d < -1.0 or cos_d > 1.0:
            dist = sqrt(true_x ** 2 + true_y ** 2)
            return None, f"arm cannot reach dist={dist:.3f}m (move block closer)"

        calc_d = acos(cos_d)
        calc_e = atan(calc_b / calc_a)
        arm_angle = calc_e - calc_d

        preview = (
            f"x={true_x:.3f}m y={true_y:.3f}m "
            f"ang_x={x:.2f} ang_y={y:.2f} hand={hand_deg:.1f}"
        )

        msg = ColorIkResult()
        msg.pedestal_angle = float(radians(pedestal_angle))
        msg.arm_angle = float(arm_angle)
        msg.hand_angle = float(radians(hand_deg))
        msg.color = self.target_color
        self.ik_pub.publish(msg)

        metrics = f"{preview} ped={msg.pedestal_angle:.2f} arm={msg.arm_angle:.2f}"
        self.get_logger().info(f">>> DETECTED {msg.color.upper()} <<< {metrics}")
        return metrics, f"DETECTED {self.target_color.upper()}"


def main():
    os.environ.setdefault("DISPLAY", ":0")
    rclpy.init()
    node = FindColorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node._http_server is not None:
                node._http_server.shutdown()
            cv2.destroyAllWindows()
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
