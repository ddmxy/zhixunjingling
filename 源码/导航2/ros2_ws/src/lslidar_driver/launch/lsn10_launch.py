#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    driver_dir = os.path.join(
        get_package_share_directory("lslidar_driver"),
        "params",
        "lidar_uart_ros2",
        "lsn10.yaml",
    )

    lidar_port = LaunchConfiguration("lidar_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_port",
                default_value="/dev/ttyACM0",
                description="LSLIDAR serial port (default ACM0 on RDK)",
            ),
            Node(
                package="lslidar_driver",
                executable="lslidar_driver_node",
                name="lslidar_driver_node",
                output="screen",
                emulate_tty=True,
                parameters=[driver_dir, {"serial_port_": lidar_port}],
            ),
        ]
    )
