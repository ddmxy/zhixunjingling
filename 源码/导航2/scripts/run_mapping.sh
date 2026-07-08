#!/usr/bin/env bash
# SLAM mapping — same port layout as navigation.
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=load_ports.sh
source "${SCRIPT_DIR}/load_ports.sh"

PORT_ARGS=()
POSITIONAL=()
USE_RVIZ="true"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --chassis) PORT_ARGS+=(--chassis "$2"); shift 2 ;;
    --lidar) PORT_ARGS+=(--lidar "$2"); shift 2 ;;
    --ports-file|-f) PORT_ARGS+=(--ports-file "$2"); shift 2 ;;
    --no-rviz) USE_RVIZ="false"; shift ;;
    -h|--help)
      echo "Usage: run_mapping.sh [--chassis DEV] [--lidar DEV] [--no-rviz]"
      exit 0
      ;;
    --*) echo "Unknown: $1" >&2; exit 1 ;;
    *) POSITIONAL+=("$1"); shift ;;
  esac
done

[[ ${#POSITIONAL[@]} -ge 1 ]] && PORT_ARGS+=(--lidar "${POSITIONAL[0]}")
[[ ${#POSITIONAL[@]} -ge 2 ]] && PORT_ARGS+=(--chassis "${POSITIONAL[1]}")

load_device_ports "${PORT_ARGS[@]}"

echo "=== run_mapping.sh chassis=${CHASSIS_PORT} lidar=${LIDAR_PORT} ==="

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/tros/humble/setup.bash
cd ~/Desktop/ros2_ws && source install/setup.bash

sudo fuser -k "${CHASSIS_PORT}" "${LIDAR_PORT}" 2>/dev/null || true
sleep 1

exec ros2 launch car_bringup mapping.launch.py \
  "base_driver_port:=${CHASSIS_PORT}" \
  "lidar_port:=${LIDAR_PORT}" \
  "use_rviz:=${USE_RVIZ}"
