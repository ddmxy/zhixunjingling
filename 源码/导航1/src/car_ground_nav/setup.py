from setuptools import find_packages, setup

package_name = "car_ground_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/ground_nav.launch.py",
                "launch/bringup_with_lidar.launch.py",
                "launch/slam_nav_bringup.launch.py",
                "launch/rviz_2d_goal_test.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/params.yaml",
                "config/uwb_anchors.example.json",
                "config/slam_toolbox_params.yaml",
                "config/nav2_params.yaml",
            ],
        ),
        ("share/" + package_name + "/rviz", ["rviz/nav2_2d_goal_test.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo",
    description="LoRa UAV goal + UWB filtered odom for car navigation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lora_goal_receiver = car_ground_nav.lora_goal_receiver:main",
            "uwb_filtered_odom = car_ground_nav.uwb_filtered_odom:main",
            "nav2_goal_bridge = car_ground_nav.nav2_goal_bridge:main",
            "chassis_serial_node = car_ground_nav.chassis_serial_node:main",
            "uwb_serial_localizer = car_ground_nav.uwb_serial_localizer:main",
            "heading_imu_publisher = car_ground_nav.heading_imu_publisher:main",
            "pose_heading_fuser = car_ground_nav.pose_heading_fuser:main",
            "cmd_vel_gate = car_ground_nav.cmd_vel_gate:main",
        ],
    },
)
