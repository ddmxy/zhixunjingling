# RDK_Navigation

基于 **RDK X5 + STM32 + LSLIDAR N10** 的 2D 室内导航项目。

> 上位机（RDK X5 上的 ROS 2 Humble）负责把 STM32 当成"传感器+执行器"，把激光雷达的 `/scan` 与下位机的 `/odom` 一起喂进 SLAM/Nav2，最终在 RViz 中点选目标点完成自动导航。

## 目录结构

```
RDK_Navigation/
├── README.md                          ← 本文件
├── docs/                              ← 设计文档
│   ├── 00_整体方案与思路.md             ← ★ 先读这个
│   └── 01_下位机串口协议.md             ← 串口协议（已对齐 STM32 源码）
├── ros2_ws/                           ← ROS 2 工作空间（待填充）
│   └── src/
│       ├── lslidar_driver/            ← 雷达厂家驱动（步骤 2）
│       ├── lslidar_msgs/
│       ├── base_driver/               ← 自己写的串口⇄ROS 桥接（步骤 3）
│       ├── car_description/           ← URDF（步骤 4）
│       ├── car_bringup/               ← 一键 launch（步骤 5/6）
│       └── car_navigation/            ← Nav2 参数（步骤 6）
└── scripts/                           ← udev 规则、烧录脚本等
```

## 快速开始（当前版本）

```bash
# 在 RDK X5 上：
cd ~/RDK_Navigation/ros2_ws
colcon build
source install/setup.bash

# 模式 A：建图（已提供）
ros2 launch car_bringup mapping.launch.py

# 键盘遥控（另开一个终端）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 导航前安装一次（RViz 使用 Nav2 Goal 动作，需此包）
sudo apt install ros-humble-nav2-rviz-plugins
```

**导航 RViz 要点：** 保留 **「Navigation 2」** 侧栏；用 **Nav2 Goal** 在地图上**拖箭头**松开后，看 **Navigation 2** 里状态/或运行 `navigation.launch.py` 的终端日志。若代价地图在 RViz 里报警，请**勿在 RViz 里改完就长期保留带 `*` 的未保存配置**，否则与 `install/share` 里已修复的 `navigation.rviz` 不一致；改完可 **File → Save** 到工作区 `src/.../navigation.rviz` 再 `colcon build`。

`base_driver` 默认会：
- 启动即发一帧 `v=0,w=0`（触发下位机开始回传）
- 使用首帧 IMU yaw 作为零点（`use_first_yaw_as_zero: true`）
- 在终端周期打印 `vx/vy/wz/yaw` 接收值

## 当前进度

- [x] 步骤 0：方案设计（`docs/00_整体方案与思路.md`）
- [x] 步骤 1：补全协议文档 `docs/01_下位机串口协议.md`
- [x] 步骤 2：N10 雷达驱动源码接入（`lslidar_driver` + `lslidar_msgs`）
- [x] 步骤 3：`base_driver` 节点初版完成（串口收发、日志、`/odom`、`/imu/data`、TF）
- [x] 步骤 4：URDF + `car_description` 完成（`base_footprint -> base_link -> laser/imu_link`）
- [x] 步骤 5：建图 launch + slam_toolbox 参数完成（`car_bringup/mapping.launch.py`）
- [ ] 步骤 6：导航（Nav2）
- [ ] 步骤 7：联调

## 相关材料位置

- 下位机源码：`../USER/`、`../HARDWARE/`、`../SYSTEM/`、`../CORE/`、`../STM32F10x_FWLib/`（STM32F103，Keil 工程 `../USER/TIMER.uvprojx`）
- 雷达资料：`D:\Ksoftware\32\N10系列激光雷达附送资料\N10系列雷达客户资料V5.0_20240822\`
- 雷达 ROS 2 SDK：`...\2.ROS2_SDK\LSLIDAR_X_ROS2-20240228\`
