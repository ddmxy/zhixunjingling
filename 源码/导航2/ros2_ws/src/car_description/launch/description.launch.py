from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("car_description")
    default_urdf = os.path.join(pkg_share, "urdf", "car.urdf")

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(default_urdf, "r", encoding="utf-8").read()}],
    )

    return LaunchDescription([rsp])

