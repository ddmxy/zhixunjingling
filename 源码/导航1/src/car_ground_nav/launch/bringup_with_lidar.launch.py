import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("car_ground_nav")
    params = os.path.join(pkg, "config", "params.yaml")

    # Car-side comms + localization + goal bridge
    car_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "ground_nav.launch.py"))
    )

    # LSLidar driver (from 2.ROS2_SDK, typically built in same colcon workspace).
    # Default config publishes /scan with frame_id=laser.
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("lslidar_driver"), "launch", "lsn10_launch.py")
        )
    )

    # Static TF: base_link -> laser. Adjust xyz/rpy for your installation.
    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_laser",
        arguments=["0.0", "0.0", "0.15", "0.0", "0.0", "0.0", "base_link", "laser"],
        output="screen",
        parameters=[params],
    )

    return LaunchDescription(
        [
            car_nodes,
            lidar_launch,
            static_tf_laser,
        ]
    )

