#!/bin/bash
set -e
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS=~/wheeltec_arm

touch "$SRC_DIR/COLCON_IGNORE"
rsync -a "$SRC_DIR/a150_arm_msgs/" "$WS/src/a150_arm_msgs/"
rsync -a "$SRC_DIR/wheeltec_color_sort/" "$WS/src/wheeltec_color_sort/"
rsync -a "$SRC_DIR/wheeltec_arm_bridge/" "$WS/src/wheeltec_arm_bridge/"

cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select a150_arm_msgs wheeltec_arm_bridge wheeltec_color_sort --executor sequential
source install/setup.bash

echo "DONE v0.2.13"
echo "  ros2 launch wheeltec_color_sort pick_color.launch.py target_color:=green port:=/dev/ttyACM0"
