#!/usr/bin/env bash
# Phase-1 mission helpers — BASE (chassis) must start before arm.
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=load_ports.sh
source "${SCRIPT_DIR}/load_ports.sh"

MAP=""
PORT_ARGS=()
POSITIONAL=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    nav|record|arm|run|mission|deploy-waypoints|help) break ;;
    --map|-m) MAP="$2"; shift 2 ;;
    --chassis|--lidar|--arm|--ports-file|-f)
      PORT_ARGS+=("$1" "$2"); shift 2 ;;
    --*) shift ;;
    *) POSITIONAL+=("$1"); shift ;;
  esac
done

[[ ${#POSITIONAL[@]} -ge 1 ]] && MAP="${POSITIONAL[0]}"
[[ ${#POSITIONAL[@]} -ge 2 ]] && PORT_ARGS+=(--lidar "${POSITIONAL[1]}")
[[ ${#POSITIONAL[@]} -ge 3 ]] && PORT_ARGS+=(--arm "${POSITIONAL[2]}")

load_device_ports "${PORT_ARGS[@]}"

MAP="${MAP:-/home/sunrise/maps/arena_map_v5.yaml}"
DESKTOP="${HOME}/Desktop"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/tros/humble/setup.bash
[ -f "${DESKTOP}/ros2_ws/install/setup.bash" ] && source "${DESKTOP}/ros2_ws/install/setup.bash"
[ -f "${HOME}/ros2_ws/install/setup.bash" ] && source "${HOME}/ros2_ws/install/setup.bash"

cmd="${1:-help}"
shift || true

case "${cmd}" in
  nav)
    echo "=== [1] Navigation chassis=${CHASSIS_PORT} lidar=${LIDAR_PORT} ==="
    bash "${DESKTOP}/run_navigation.sh" "${MAP}" --lidar "${LIDAR_PORT}" --chassis "${CHASSIS_PORT}"
    ;;
  record)
    python3 "${DESKTOP}/save_mission_points.py"
    ;;
  arm)
    arm_port="${1:-${ARM_PORT}}"
    echo "=== Arm serial: ${arm_port} ==="
    ros2 launch wheeltec_arm_bridge arm_serial_only.launch.py "port:=${arm_port}"
    ;;
  run|mission)
    echo "=== Full / partial mission arm=${ARM_PORT} ==="
    python3 "${DESKTOP}/run_full_mission.py" --waypoints "${HOME}/mission_waypoints.yaml" \
      --arm-port "${ARM_PORT}" "$@"
    ;;
  deploy-waypoints)
    if [ -f "${DESKTOP}/mission_waypoints.yaml" ]; then
      cp "${DESKTOP}/mission_waypoints.yaml" "${HOME}/mission_waypoints.yaml"
    elif [ -f "${DESKTOP}/ros2_ws/src/car_navigation/config/mission_waypoints.yaml" ]; then
      cp "${DESKTOP}/ros2_ws/src/car_navigation/config/mission_waypoints.yaml" "${HOME}/mission_waypoints.yaml"
    else
      echo "mission_waypoints.yaml not found" >&2
      exit 1
    fi
    echo "OK -> ~/mission_waypoints.yaml"
    ;;
  help|*)
    echo "Usage:"
    echo "  bash ~/Desktop/run_mission_phase1.sh [MAP] nav"
    echo "  bash ~/Desktop/run_mission_phase1.sh [MAP] run"
    echo "  bash ~/Desktop/run_mission_phase1.sh --arm /dev/ttyACM0 arm"
    echo ""
    echo "Ports: ~/device_ports.yaml  or  --chassis / --lidar / --arm"
    python3 "${SCRIPT_DIR}/load_ports.py" --show
    ;;
esac
