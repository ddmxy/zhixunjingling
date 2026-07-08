import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("car_ground_nav")

    # Bring up: car nodes + lidar + static TF + SLAM toolbox + Nav2
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "slam_nav_bringup.launch.py"))
    )

    rviz_cfg = os.path.join(pkg, "rviz", "nav2_2d_goal_test.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    return LaunchDescription([bringup, rviz])

