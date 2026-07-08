#!/usr/bin/env bash
# Show configured + present USB/serial devices.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=load_ports.sh
source "${SCRIPT_DIR}/load_ports.sh"
load_device_ports "$@"

echo "===== Configured ports ====="
python3 "${SCRIPT_DIR}/load_ports.py" --show "${@}"
echo ""

echo "===== Present devices ====="
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/video* 2>/dev/null || true
echo ""

for dev in "${CHASSIS_PORT}" "${LIDAR_PORT}" "${ARM_PORT}" "${ARM_CAMERA}"; do
  if [[ -e "$dev" ]]; then
    holder=$(sudo fuser "$dev" 2>/dev/null || true)
    echo "  $dev  exists  holder: ${holder:-free}"
  else
    echo "  $dev  MISSING"
  fi
done

echo ""
echo "Tip: edit ~/device_ports.yaml when plug order changes."
echo "     Or: bash run_navigation.sh --chassis ... --lidar ..."
