#!/bin/bash
# 逐个口试：谁是雷达、谁是机械臂
# 用法: bash ~/Desktop/identify_ports.sh

echo "========== 1. 当前插着的口 =========="
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/video* 2>/dev/null || echo "(无)"

echo ""
echo "========== 2. USB 身份 (看名字区分) =========="
for dev in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0; do
  if [[ -e "$dev" ]]; then
    echo "--- $dev ---"
    udevadm info -q property -n "$dev" 2>/dev/null | grep -E 'ID_VENDOR|ID_MODEL|ID_SERIAL_SHORT' || true
  fi
done

echo ""
echo "========== 3. 怎么判断 =========="
cat <<'EOF'
最靠谱：一次只插一个设备，看出现 ttyACM0 还是 ttyACM1

  只插雷达  -> 记下是哪个 /dev/ttyACM?
  只插机械臂 -> 记下是哪个 /dev/ttyACM?
  底盘一般是 /dev/ttyUSB0 (CH340/CP210x 等)

插上后实测：
  雷达口：导航起来后  ros2 topic hz /scan  有频率(约5-10Hz)就是雷达
  机械臂口：
    source /opt/ros/humble/setup.bash
    source ~/wheeltec_arm/install/setup.bash
    ros2 launch wheeltec_color_sort pick_color.launch.py target_color:=green port:=/dev/ttyACM?
    机械臂会抬起来 -> 这个口就是 arm_mcu

改配置只改一处：  nano ~/device_ports.yaml
  lidar:   /dev/ttyACM?   # 导航用
  arm_mcu: /dev/ttyACM?   # 抓取用

然后两个终端都重启。
EOF

echo ""
echo "========== 4. 当前 device_ports.yaml =========="
python3 ~/Desktop/load_ports.py --show 2>/dev/null || cat ~/device_ports.yaml 2>/dev/null || echo "未找到配置"
