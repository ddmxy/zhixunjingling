from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("base_driver")
    default_params = os.path.join(pkg_share, "config", "base_driver.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to base driver parameter file",
    )
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Nav chassis STM32 on ttyUSB0 (arm camera: ttyUSB1)",
    )

    node = Node(
        package="base_driver",
        executable="base_driver_node",
        name="base_driver_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"port": LaunchConfiguration("port")},
        ],
    )

    return LaunchDescription([params_file_arg, port_arg, node])

