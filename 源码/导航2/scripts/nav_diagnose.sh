#!/usr/bin/env bash
# Quick Nav2 health check — run while navigation.launch.py is running.
set -euo pipefail

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/tros/humble/setup.bash
source ~/Desktop/ros2_ws/install/setup.bash 2>/dev/null || true

echo "===== Action (must exist for Nav2 Goal) ====="
ros2 action list 2>/dev/null | grep -E 'navigate_to_pose|NavigateToPose' || echo "MISSING navigate_to_pose"

echo ""
echo "===== Lifecycle (all must be active [3]) ====="
for n in amcl map_server bt_navigator controller_server planner_server behavior_server; do
  printf "  %-20s " "$n"
  ros2 lifecycle get "/$n" 2>/dev/null || echo "NODE NOT RUNNING"
done

echo ""
echo "===== TF map -> odom (need output after Pose Estimate) ====="
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | head -6 || echo "NO map->odom TF"

echo ""
echo "===== Topics ====="
ros2 topic hz /scan --window 3 2>/dev/null || true
ros2 topic hz /cmd_vel --window 3 2>/dev/null || true

echo ""
echo "If bt_navigator is NOT active [3], Nav2 Goal will do nothing."
echo "Fix: pkill -9 -f ros2; sleep 3; relaunch navigation."
