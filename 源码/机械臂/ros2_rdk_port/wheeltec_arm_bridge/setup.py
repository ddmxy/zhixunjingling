from setuptools import setup
from glob import glob

package_name = "wheeltec_arm_bridge"

setup(
    name=package_name,
    version="0.2.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Serial bridge for WHEELTEC A150",
    license="BSD",
    entry_points={
        "console_scripts": [
            "arm_serial_node = wheeltec_arm_bridge.arm_serial_node:main",
            "trajectory_bridge_node = wheeltec_arm_bridge.trajectory_bridge_node:main",
        ],
    },
)
