"""Pick one configured color block."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DISPLAY_ENV = {"DISPLAY": ":0"}


def as_bool(text: str) -> bool:
    return str(text).strip().lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    port = LaunchConfiguration("port").perform(context)
    video_device = LaunchConfiguration("video_device").perform(context)
    target_color = LaunchConfiguration("target_color").perform(context)
    put_pose = LaunchConfiguration("put_pose").perform(context)
    visualize = as_bool(LaunchConfiguration("visualize").perform(context))
    run_pick = as_bool(LaunchConfiguration("run_pick").perform(context))
    use_moveit = as_bool(LaunchConfiguration("use_moveit").perform(context))
    web_port = int(LaunchConfiguration("web_port").perform(context))

    port_missing = not os.path.exists(port)

    color_share = get_package_share_directory("wheeltec_color_sort")
    pick_yaml = os.path.join(color_share, "config", "pick_color.yaml")

    nodes = []
    if run_pick and port_missing:
        nodes.append(
            LogInfo(
                msg=(
                    f"WARN: {port} not found, camera-only mode. "
                    "Plug arm USB, then: ls /dev/ttyACM*"
                )
            )
        )
        run_pick = False

    nodes.extend(
        [
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            output="screen",
            parameters=[
                {
                    "video_device": video_device,
                    "image_width": 640,
                    "image_height": 480,
                    "framerate": 30.0,
                    "pixel_format": "mjpeg2rgb",
                }
            ],
        ),
        Node(
            package="wheeltec_color_sort",
            executable="find_color_node",
            output="screen",
            additional_env=DISPLAY_ENV,
            parameters=[
                pick_yaml,
                {
                    "image_topic": "/image_raw",
                    "show_image": visualize,
                    "publish_debug_image": True,
                    "enable_web_view": False,
                    "web_port": web_port,
                    "target_color": target_color,
                },
            ],
        ),
        ]
    )

    if run_pick and os.path.exists(port):
        bridge_launch = (
            "arm_hardware.launch.py" if use_moveit else "arm_serial_only.launch.py"
        )
        nodes.extend(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            get_package_share_directory("wheeltec_arm_bridge"),
                            f"/launch/{bridge_launch}",
                        ]
                    ),
                    launch_arguments={"port": port}.items(),
                ),
            ]
        )
        if use_moveit:
            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            get_package_share_directory("a150_moveit_config"),
                            "/launch/move_group_only.launch.py",
                        ]
                    ),
                )
            )
        nodes.append(
                Node(
                    package="wheeltec_color_sort",
                    executable="color_pick_node",
                    output="screen",
                    parameters=[
                        pick_yaml,
                        {
                            "target_color": target_color,
                            "put_pose": put_pose,
                            "use_moveit": use_moveit,
                        },
                    ],
                ),
            )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("target_color", default_value="green"),
            DeclareLaunchArgument("put_pose", default_value="green_put"),
            DeclareLaunchArgument("visualize", default_value="true"),
            DeclareLaunchArgument("web_port", default_value="8080"),
            DeclareLaunchArgument("run_pick", default_value="true"),
            DeclareLaunchArgument("use_moveit", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
