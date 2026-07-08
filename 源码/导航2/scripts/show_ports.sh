#!/bin/bash
# 改 ~/device_ports.yaml 后跑这个，确认各脚本会用什么口
echo "========== 读取的配置文件 =========="
python3 ~/Desktop/load_ports.py --show
echo ""
eval "$(python3 ~/Desktop/load_ports.py --bash)"
echo "========== 各脚本实际会用 =========="
echo "run_navigation.sh   chassis=$CHASSIS_PORT  lidar=$LIDAR_PORT"
echo "run_full_mission.sh arm=$ARM_PORT"
echo "摄像头              $ARM_CAMERA"
echo ""
echo "启动时脚本第一行会打印上述口，请对照确认。"
