import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_driver_share = get_package_share_directory("base_driver")
    car_desc_share = get_package_share_directory("car_description")
    car_nav_share = get_package_share_directory("car_navigation")
    lslidar_share = get_package_share_directory("lslidar_driver")
    slam_share = get_package_share_directory("slam_toolbox")

    default_base_driver_params = os.path.join(base_driver_share, "config", "base_driver.yaml")
    default_slam_params = os.path.join(car_nav_share, "config", "slam_toolbox_mapping.yaml")
    default_rviz_cfg = os.path.join(car_nav_share, "rviz", "mapping.rviz")
    default_lidar_launch = os.path.join(lslidar_share, "launch", "lsn10_launch.py")
    default_urdf = os.path.join(car_desc_share, "urdf", "car.urdf")
    default_sensor_cal = os.path.join(car_desc_share, "config", "sensor_calibration.yaml")

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from urdf_loader import load_robot_description

    base_driver_params = LaunchConfiguration("base_driver_params")
    base_driver_port = LaunchConfiguration("base_driver_port")
    lidar_port = LaunchConfiguration("lidar_port")
    slam_params = LaunchConfiguration("slam_params")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    lidar_launch = LaunchConfiguration("lidar_launch")

    args = [
        DeclareLaunchArgument("base_driver_params", default_value=default_base_driver_params),
        DeclareLaunchArgument(
            "base_driver_port",
            default_value="/dev/ttyUSB0",
            description="Nav chassis STM32 (ttyUSB0). Arm cam uses ttyUSB1.",
        ),
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyACM0",
            description="LSLIDAR serial (ttyACM0). Arm MCU uses ttyACM1.",
        ),
        DeclareLaunchArgument("slam_params", default_value=default_slam_params),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_cfg),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("lidar_launch", default_value=default_lidar_launch),
    ]

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch),
        launch_arguments={"lidar_port": lidar_port}.items(),
    )

    base_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_driver_share, "launch", "base_driver.launch.py")
        ),
        launch_arguments={
            "params_file": base_driver_params,
            "port": base_driver_port,
        }.items(),
    )

    robot_description = load_robot_description(default_urdf, default_sensor_cal)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "online_async_launch.py")),
        launch_arguments={"slam_params_file": slam_params, "use_sim_time": "false"}.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [lidar, base_driver, rsp, slam, rviz])

