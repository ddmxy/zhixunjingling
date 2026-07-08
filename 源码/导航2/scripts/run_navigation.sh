#!/bin/bash
MAP="/home/sunrise/maps/arena_map_v5.yaml"
PORT_ARGS=()
for arg in "$@"; do
  if [[ "$arg" == *.yaml ]]; then
    MAP="$arg"
  else
    PORT_ARGS+=("$arg")
  fi
done

eval "$(python3 ~/Desktop/load_ports.py --bash "${PORT_ARGS[@]}")"
CHASSIS="$CHASSIS_PORT"
LIDAR="$LIDAR_PORT"

echo "=== navigation ==="
echo "map=$MAP"
echo "chassis=$CHASSIS  lidar=$LIDAR"
echo "ports_file=${PORTS_FILE:-未找到yaml，用的是内置默认}"

set +u
source /opt/ros/humble/setup.bash

cd ~/Desktop/ros2_ws
colcon build --symlink-install --packages-select lslidar_driver base_driver car_bringup car_navigation
source install/setup.bash

sudo fuser -k "$CHASSIS" "$LIDAR" 2>/dev/null || true
sleep 1

ros2 launch car_bringup navigation.launch.py map:="$MAP" base_driver_port:="$CHASSIS" lidar_port:="$LIDAR"
