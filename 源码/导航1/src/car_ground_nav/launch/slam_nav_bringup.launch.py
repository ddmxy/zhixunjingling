import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory("car_ground_nav")

    # 1) Car comms + UWB odom TF + LoRa goal bridge
    car_with_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "bringup_with_lidar.launch.py"))
    )

    # 2) SLAM toolbox (online async mapping) -> publishes map->odom and /map
    slam_params = os.path.join(pkg, "config", "slam_toolbox_params.yaml")
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
        ),
        launch_arguments={"slam_params_file": slam_params}.items(),
    )

    # 3) Nav2 navigation stack (no AMCL; SLAM provides map->odom)
    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
        ),
        launch_arguments={"params_file": nav2_params, "use_sim_time": "false"}.items(),
    )

    return LaunchDescription(
        [
            car_with_lidar,
            slam,
            nav2,
        ]
    )

