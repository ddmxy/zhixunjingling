from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("car_ground_nav")
    params = os.path.join(pkg, "config", "params.yaml")

    return LaunchDescription(
        [
            Node(
                package="car_ground_nav",
                executable="uwb_serial_localizer",
                name="uwb_serial_localizer",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="car_ground_nav",
                executable="lora_goal_receiver",
                name="lora_goal_receiver",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="car_ground_nav",
                executable="pose_heading_fuser",
                name="pose_heading_fuser",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="car_ground_nav",
                executable="heading_imu_publisher",
                name="heading_imu_publisher",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="car_ground_nav",
                executable="uwb_filtered_odom",
                name="uwb_filtered_odom",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="car_ground_nav",
                executable="nav2_goal_bridge",
                name="nav2_goal_bridge",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="car_ground_nav",
                executable="cmd_vel_gate",
                name="cmd_vel_gate",
                parameters=[params],
                output="screen",
            ),
            Node(
                package="car_ground_nav",
                executable="chassis_serial_node",
                name="chassis_serial_node",
                parameters=[params],
                output="screen",
            ),
        ]
    )
