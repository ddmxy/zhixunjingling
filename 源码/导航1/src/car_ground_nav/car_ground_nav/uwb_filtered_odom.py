"""Subscribe to raw UWB pose, apply low-pass + jump limit, publish nav_msgs/Odometry on /odom."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry


class UwbFilteredOdom(Node):
    def __init__(self) -> None:
        super().__init__("uwb_filtered_odom")

        self.declare_parameter("input_topic", "uwb_pose_raw")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("alpha", 0.35)
        self.declare_parameter("max_step_m", 2.0)
        self.declare_parameter("publish_tf", False)

        self._odom_frame = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame_id").get_parameter_value().string_value
        self._alpha = self.get_parameter("alpha").get_parameter_value().double_value
        self._max_step = self.get_parameter("max_step_m").get_parameter_value().double_value
        self._publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value

        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value

        self._fx: Optional[float] = None
        self._fy: Optional[float] = None
        self._prev_fx: Optional[float] = None
        self._prev_fy: Optional[float] = None
        self._prev_time: Optional[rclpy.time.Time] = None

        self._odom_pub = self.create_publisher(Odometry, "odom", 10)

        self._tf_broadcaster = None
        self._TransformStamped = None
        if self._publish_tf:
            from tf2_ros import TransformBroadcaster
            from geometry_msgs.msg import TransformStamped

            self._tf_broadcaster = TransformBroadcaster(self)
            self._TransformStamped = TransformStamped

        self.create_subscription(PoseStamped, in_topic, self._on_pose, 10)
        self.get_logger().info(
            f"uwb_filtered_odom: sub {in_topic} -> odom "
            f"(frame={self._odom_frame}, alpha={self._alpha}, max_step={self._max_step}m)"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)

        if self._fx is None:
            self._fx, self._fy = x, y
        else:
            dx = x - self._fx
            dy = y - self._fy
            step = math.hypot(dx, dy)
            if step > self._max_step:
                self.get_logger().warning(
                    f"UWB jump {step:.2f} m > max_step; accepting raw sample"
                )
                self._fx, self._fy = x, y
            else:
                a = self._alpha
                self._fx = a * x + (1.0 - a) * self._fx
                self._fy = a * y + (1.0 - a) * self._fy

        assert self._fx is not None and self._fy is not None
        now = self.get_clock().now()

        vx = 0.0
        vy = 0.0
        if self._prev_fx is not None and self._prev_time is not None:
            dt = (now - self._prev_time).nanoseconds * 1e-9
            if dt > 1e-6:
                vx = (self._fx - self._prev_fx) / dt
                vy = (self._fy - self._prev_fy) / dt

        self._prev_fx = self._fx
        self._prev_fy = self._fy
        self._prev_time = now

        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._fx
        odom.pose.pose.position.y = self._fy
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = msg.pose.orientation
        odom.twist.twist = Twist(linear=Vector3(x=vx, y=vy, z=0.0))
        self._odom_pub.publish(odom)

        if self._publish_tf and self._tf_broadcaster is not None and self._TransformStamped is not None:
            t = self._TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = self._fx
            t.transform.translation.y = self._fy
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(t)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = UwbFilteredOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
