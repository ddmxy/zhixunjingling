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

    default_base_driver_params = os.path.join(base_driver_share, "config", "base_driver.yaml")
    default_nav2_params = os.path.join(car_nav_share, "config", "nav2_params.yaml")
    default_rviz_cfg = os.path.join(car_nav_share, "rviz", "navigation.rviz")
    default_lidar_launch = os.path.join(lslidar_share, "launch", "lsn10_launch.py")
    default_urdf = os.path.join(car_desc_share, "urdf", "car.urdf")
    default_sensor_cal = os.path.join(car_desc_share, "config", "sensor_calibration.yaml")

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from urdf_loader import load_robot_description

    base_driver_params = LaunchConfiguration("base_driver_params")
    base_driver_port = LaunchConfiguration("base_driver_port")
    lidar_port = LaunchConfiguration("lidar_port")
    map_yaml = LaunchConfiguration("map")
    # NOTE: this MUST NOT be called "params_file" — base_driver.launch.py also declares a
    # LaunchConfiguration with that exact name (default = base_driver.yaml). Because launch
    # arguments are global, the IncludeLaunchDescription(base_driver) below silently
    # overwrites our "params_file" with base_driver.yaml, and then every nav2 node we hand
    # parameters=[params_file] to ends up loading base_driver.yaml instead of
    # nav2_params.yaml. Symptom: controller_server runs at default 20Hz with no DWB critics.
    nav_params_file = LaunchConfiguration("nav_params_file")
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
        DeclareLaunchArgument(
            "map",
            default_value="/home/sunrise/maps/arena_map_v4.yaml",
            description="Full path to map yaml (e.g. /home/sunrise/maps/my_map.yaml)",
        ),
        DeclareLaunchArgument("nav_params_file", default_value=default_nav2_params),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_cfg),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("lidar_launch", default_value=default_lidar_launch),
    ]

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch),
        launch_arguments={"lidar_port": lidar_port}.items(),
    )

    base_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(base_driver_share, "launch", "base_driver.launch.py")),
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

    # ---------------- Localization ----------------
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[nav_params_file, {"use_sim_time": False, "yaml_filename": map_yaml}],
    )

    # Force /scan: multi-node YAML sometimes does not merge amcl.scan_topic onto /amcl,
    # leaving the node's default "scan" (no leading slash) so AMCL never subscribes to /scan.
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav_params_file, {"use_sim_time": False, "scan_topic": "/scan"}],
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    # ---------------- Navigation ----------------
    # IMPORTANT (Humble): we deliberately DO NOT include nav2_bringup/navigation_launch.py
    # (its RewrittenYaml indirection is fragile). Each Nav2 node below is launched directly
    # with parameters=[nav_params_file, ...] so the user's nav2_params.yaml is passed as-is
    # via --params-file. Do not rename nav_params_file to "params_file" — that name is
    # already taken by base_driver.launch.py and IncludeLaunchDescription will silently
    # overwrite it with base_driver.yaml, leaving every controller/planner/bt node at
    # built-in defaults (20Hz controller_server, no DWB critics, etc.).
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav_params_file, {"use_sim_time": False}],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav_params_file, {"use_sim_time": False}],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav_params_file, {"use_sim_time": False}],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav_params_file, {"use_sim_time": False}],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav_params_file, {"use_sim_time": False}],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav_params_file, {"use_sim_time": False}],
    )

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"bond_timeout": 20.0},
            {
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                ]
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        args
        + [
            lidar,
            base_driver,
            rsp,
            map_server,
            amcl,
            lifecycle_manager_localization,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            lifecycle_manager_navigation,
            rviz,
        ]
    )
