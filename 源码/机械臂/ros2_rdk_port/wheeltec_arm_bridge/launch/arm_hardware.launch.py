"""Serial bridge + MoveIt trajectory action servers."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            Node(
                package="wheeltec_arm_bridge",
                executable="trajectory_bridge_node",
                output="screen",
            ),
            Node(
                package="wheeltec_arm_bridge",
                executable="arm_serial_node",
                output="screen",
                parameters=[{"port": LaunchConfiguration("port")}],
            ),
        ]
    )
