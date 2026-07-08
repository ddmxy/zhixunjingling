#!/bin/bash
eval "$(python3 ~/Desktop/load_ports.py --bash "$@")"
ARM="$ARM_PORT"

echo "=== full mission ==="
echo "arm=$ARM"
echo "ports_file=${PORTS_FILE:-未找到yaml，用的是内置默认}"

set +u
source /opt/ros/humble/setup.bash
source ~/Desktop/ros2_ws/install/setup.bash
source ~/wheeltec_arm/install/setup.bash

python3 ~/Desktop/run_full_mission.py --waypoints ~/mission_waypoints.yaml --arm-port "$ARM" "$@"
