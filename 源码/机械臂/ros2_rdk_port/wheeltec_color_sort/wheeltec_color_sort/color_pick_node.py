#!/usr/bin/env python3
"""Pick one configured color block."""
import math
import statistics
import threading
import time

import rclpy
from a150_arm_msgs.msg import ColorIkResult
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

PACKAGE_VERSION = "0.2.12"

ALL_JOINTS = [
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5",
    "joint_6", "joint_10", "joint_7", "joint_11", "joint_8", "joint_9",
]

ARM_POSES = {
    "arm_home": [0.0, 0.0, 0.0, 0.0, 0.0],
    "arm_look": [0.0, 0.42, -1.57, -1.57, 0.0],
    "color_put_interval": [1.2, 0.0, -0.63, -1.3, 0.0],
    "green_put": [1.30, -0.54, -1.02, -1.10, -0.28],
    "blue_put": [1.23, -0.47, -1.12, -1.41, -0.38],
    "yellow_put": [1.57, -0.55, -1.04, -1.17, 0.0],
}

HAND_POSES = {
    "hand_open": {"joint_10": -0.2, "joint_11": 0.2, "joint_6": 0.2, "joint_7": 0.2, "joint_9": -0.2, "joint_8": 0.2},
    "hand_half_open": {"joint_10": 0.3, "joint_11": -0.3, "joint_6": -0.3, "joint_7": -0.3, "joint_9": 0.3, "joint_8": -0.3},
    "hand_close": {"joint_10": 0.8, "joint_11": -0.8, "joint_6": -0.8, "joint_7": -0.8, "joint_9": 0.8, "joint_8": -0.8},
}


class ColorPickNode(Node):
    def __init__(self):
        super().__init__("color_pick_execute")
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter("target_color", "green")
        self.declare_parameter("put_pose", "green_put")
        self.declare_parameter("auxiliary_angle", 0.20)
        self.declare_parameter("link_a", 0.105)
        self.declare_parameter("link_c", 0.170)
        self.declare_parameter("link_h", 0.105)
        self.declare_parameter("confirm_count", 6)
        self.declare_parameter("max_sample_spread", 0.04)
        self.declare_parameter("do_put", False)
        self.declare_parameter("return_home_after_pick", True)
        self.declare_parameter("use_moveit", False)
        self.declare_parameter("move_duration", 3.0)
        self.declare_parameter("grip_hold_duration", 2.0)
        self.declare_parameter("pick_cooldown", 2.0)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("grab_y_adjust", 0.0)

        self.target_color = self.get_parameter("target_color").get_parameter_value().string_value
        self.put_pose = self.get_parameter("put_pose").get_parameter_value().string_value
        self.confirm_count = int(self.get_parameter("confirm_count").value)
        self.max_sample_spread = float(self.get_parameter("max_sample_spread").value)
        self.do_put = bool(self.get_parameter("do_put").value)
        self.return_home_after_pick = bool(self.get_parameter("return_home_after_pick").value)
        self.use_moveit = bool(self.get_parameter("use_moveit").value)
        self.move_duration = float(self.get_parameter("move_duration").value)
        self.grip_hold_duration = float(self.get_parameter("grip_hold_duration").value)
        self.pick_cooldown = float(self.get_parameter("pick_cooldown").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.grab_y_adjust = float(self.get_parameter("grab_y_adjust").value)
        self.auxiliary_angle = self.get_parameter("auxiliary_angle").value
        self.link_a = self.get_parameter("link_a").value
        self.link_c = self.get_parameter("link_c").value
        self.link_h = self.get_parameter("link_h").value
        self.base_angle = math.acos((self.link_c - self.link_h) / self.link_a)

        if self.do_put and self.put_pose not in ARM_POSES:
            raise ValueError(f"Unknown put_pose: {self.put_pose}")

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.move_client = None
        if self.use_moveit:
            self.move_client = ActionClient(
                self, MoveGroup, "/move_action", callback_group=self.callback_group
            )
            self.get_logger().info("Waiting for move_group...")
            self.move_client.wait_for_server()

        self.create_subscription(
            ColorIkResult,
            "/color_ik_result",
            self.color_callback,
            10,
            callback_group=self.callback_group,
        )

        self.samples = []
        self.busy = False
        self.init_done = False
        self.pick_lock = threading.Lock()
        self.positions = {name: 0.0 for name in ALL_JOINTS}
        self.positions["joint_4"] = -1.57
        self._apply_hand_pose(HAND_POSES["hand_open"])
        self._apply_arm_pose(ARM_POSES["arm_look"])

        mode = "grab+home" if not self.do_put else "grab+put"
        self.get_logger().info(
            f"color_pick v{PACKAGE_VERSION} mode={mode}, target={self.target_color}, "
            f"confirm={self.confirm_count}, spread<{self.max_sample_spread}"
        )
        threading.Thread(target=self._init_pose, daemon=True).start()

    def _apply_arm_pose(self, arm_joints):
        for i, name in enumerate(ALL_JOINTS[:5]):
            self.positions[name] = float(arm_joints[i])

    def _apply_hand_pose(self, hand_map):
        self.positions.update({k: float(v) for k, v in hand_map.items()})

    def _init_pose(self):
        try:
            time.sleep(1.0)
            self.get_logger().info("moving to arm_look...")
            self.move_named_arm("arm_look")
            self.move_named_hand("hand_open")
            self.init_done = True
            self.get_logger().info("ready at arm_look — waiting for stable target")
        except Exception as exc:
            self.get_logger().error(f"init pose failed: {exc}")

    def _sample_spread(self, samples):
        ped = [s[0] for s in samples]
        arm = [s[1] for s in samples]
        hand = [s[2] for s in samples]
        return max(
            max(ped) - min(ped),
            max(arm) - min(arm),
            max(hand) - min(hand),
        )

    def _average_sample(self, samples):
        return [
            statistics.mean(s[0] for s in samples),
            statistics.mean(s[1] for s in samples),
            statistics.mean(s[2] for s in samples),
        ]

    def color_callback(self, msg: ColorIkResult):
        if msg.color != self.target_color:
            return
        if not self.init_done:
            return
        with self.pick_lock:
            if self.busy:
                return
            self.samples.append([msg.pedestal_angle, msg.arm_angle, msg.hand_angle])
            n = len(self.samples)
            self.get_logger().info(
                f"sample {n}/{self.confirm_count} "
                f"ped={msg.pedestal_angle:.3f} arm={msg.arm_angle:.3f}"
            )
            if n < self.confirm_count:
                return

            spread = self._sample_spread(self.samples)
            if spread > self.max_sample_spread:
                self.get_logger().warn(
                    f"unstable target spread={spread:.3f}, reset samples "
                    f"(keep block still in center)"
                )
                self.samples = self.samples[-2:]
                return

            target = self._average_sample(self.samples)
            self.samples = []
            self.busy = True

        self.get_logger().info(
            f"STABLE TARGET ped={target[0]:.3f} arm={target[1]:.3f} hand={target[2]:.3f}"
        )
        threading.Thread(target=self._pick_worker, args=(target,), daemon=True).start()

    def _pick_worker(self, target):
        self.get_logger().info(f"START PICK: color={self.target_color}")
        try:
            self.pick_sequence(target)
            self.get_logger().info(f"PICK DONE: color={self.target_color}")
        except Exception as exc:
            self.get_logger().error(f"pick failed: {exc}")
        finally:
            time.sleep(self.pick_cooldown)
            try:
                self.get_logger().info("reset: open gripper, back to arm_look")
                self.move_named_hand("hand_open")
                self.move_named_arm("arm_look")
            except Exception as exc:
                self.get_logger().error(f"reset failed: {exc}")
            with self.pick_lock:
                self.busy = False

    def pick_sequence(self, target):
        pedestal, arm_angle, hand_angle = target
        if self.grab_y_adjust:
            arm_angle = max(0.01, arm_angle + self.grab_y_adjust)

        joints = [
            pedestal,
            -1.57 - arm_angle + self.base_angle,
            arm_angle - self.base_angle,
            -1.57 + arm_angle + self.auxiliary_angle,
            hand_angle,
        ]
        self.get_logger().info(f"STEP 1 move to block, joints={[round(v, 3) for v in joints]}")
        self.move_arm_joints(joints)
        time.sleep(0.3)

        self.get_logger().info("STEP 2 close gripper")
        self.move_named_hand("hand_close")
        time.sleep(0.3)

        if self.do_put:
            self._do_put_sequence()
        elif self.return_home_after_pick:
            self.get_logger().info("STEP 3 return home (holding block)")
            self.move_named_arm("arm_home")
        else:
            self.get_logger().info("STEP 3 lift to arm_look (holding block)")
            self.move_named_arm("arm_look")

    def _do_put_sequence(self):
        self.get_logger().info("STEP 3 lift to arm_look")
        self.move_named_arm("arm_look")
        self.get_logger().info("STEP 4 move to put interval")
        self.move_named_arm("color_put_interval")
        self.get_logger().info(f"STEP 5 move to {self.put_pose}")
        self.move_named_arm(self.put_pose)
        self.get_logger().info("STEP 6 release block")
        self.move_named_hand("hand_half_open")
        time.sleep(0.5)
        self.move_named_arm("color_put_interval")
        self.move_named_arm("arm_look")
        self.move_named_hand("hand_open")
        if self.return_home_after_pick:
            self.move_named_arm("arm_home")

    def move_named_arm(self, pose_name):
        self.move_arm_joints(ARM_POSES[pose_name])

    def move_named_hand(self, pose_name):
        self.move_hand_joints(HAND_POSES[pose_name], hold_long=(pose_name == "hand_close"))

    def move_arm_joints(self, joints):
        if self.use_moveit:
            self._move_arm_moveit(joints)
        else:
            self._move_arm_direct(joints)

    def _publish_once(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        msg.position = [float(self.positions[name]) for name in ALL_JOINTS]
        self.joint_pub.publish(msg)

    def _hold_publish(self, duration=0.5):
        end = time.monotonic() + duration
        period = 1.0 / self.publish_rate
        while time.monotonic() < end and rclpy.ok():
            self._publish_once()
            time.sleep(period)

    def _move_arm_direct(self, joints):
        start = [self.positions[f"joint_{i}"] for i in range(1, 6)]
        end = [float(v) for v in joints]
        steps = max(int(self.move_duration * self.publish_rate), 1)
        period = 1.0 / self.publish_rate
        for step in range(1, steps + 1):
            t = step / steps
            for i in range(5):
                self.positions[f"joint_{i + 1}"] = start[i] + (end[i] - start[i]) * t
            self._publish_once()
            time.sleep(period)
        self._hold_publish(1.0)
        self.get_logger().info(f"arm at {[round(v, 3) for v in end]}")

    def move_hand_joints(self, joint_map, hold_long=False):
        self._apply_hand_pose(joint_map)
        hold = self.grip_hold_duration if hold_long else 1.0
        self._hold_publish(hold)
        self.get_logger().info(f"hand: {list(joint_map.keys())}")

    def _move_arm_moveit(self, joints):
        constraints = Constraints()
        for index, value in enumerate(joints, start=1):
            joint = JointConstraint()
            joint.joint_name = f"joint_{index}"
            joint.position = float(value)
            joint.tolerance_above = 0.02
            joint.tolerance_below = 0.02
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnect"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 15.0
        goal.request.goal_constraints.append(constraints)

        future = self.move_client.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveGroup goal rejected for group arm")

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.05)
        result = result_future.result()
        if result is None:
            raise RuntimeError("MoveGroup returned no result")
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveGroup failed for group arm, code={result.result.error_code.val}")
        self._apply_arm_pose(joints)
        time.sleep(0.5)


def main():
    rclpy.init()
    node = ColorPickNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
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
