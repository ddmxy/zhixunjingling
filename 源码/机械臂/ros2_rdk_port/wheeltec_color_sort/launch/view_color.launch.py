"""Camera + color detection only (no arm)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DISPLAY_ENV = {"DISPLAY": ":0"}


def as_bool(text: str) -> bool:
    return str(text).strip().lower() in ("1", "true", "yes", "on")


def launch_setup(context, *args, **kwargs):
    color_share = get_package_share_directory("wheeltec_color_sort")
    pick_yaml = os.path.join(color_share, "config", "pick_color.yaml")
    visualize = as_bool(LaunchConfiguration("visualize").perform(context))

    return [
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            output="screen",
            parameters=[
                {
                    "video_device": LaunchConfiguration("video_device").perform(context),
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
                    "target_color": LaunchConfiguration("target_color").perform(context),
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("target_color", default_value="green"),
            DeclareLaunchArgument("visualize", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
