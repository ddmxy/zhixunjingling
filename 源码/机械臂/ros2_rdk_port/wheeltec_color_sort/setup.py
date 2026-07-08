from setuptools import setup
from glob import glob

package_name = "wheeltec_color_sort"

setup(
    name=package_name,
    version="0.2.12",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Single-color pick for WHEELTEC A150",
    license="BSD",
    entry_points={
        "console_scripts": [
            "find_color_node = wheeltec_color_sort.find_color_node:main",
            "color_pick_node = wheeltec_color_sort.color_pick_node:main",
        ],
    },
)
