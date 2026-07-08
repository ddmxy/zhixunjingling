# car_ground_nav（小车上位机 / ROS 2）

本包用于“空地协同”项目的小车上位机侧，把 **LoRa 接收到的无人机目标坐标** 转成 **Nav2 目标点**，同时完成 **UWB 定位**、**航向角/IMU 准备**、以及 **与底盘下位机串口通信**。

## 已实现链路总览

- **LoRa 目标坐标**：串口接收 `FF 01 X(float) Y(float) FE` → 发布 `/uav_goal`（`PoseStamped`）
- **Nav2 目标桥接**：订阅 `/uav_goal` → 调用 Nav2 `NavigateToPose` action
- **UWB 定位**：解析 `CmdM:4` 101 字节帧 → 解算 2D `(x,y)` → 发布 `/uwb_pose_raw`（`PoseStamped`）与 `/uwb_residual_m`（可选）
- **位置+航向融合**：`/uwb_pose_raw` + `/chassis/heading_rad` → `/uwb_pose_fused`（带 yaw 四元数）
- **UWB 里程计**：订阅 `/uwb_pose_fused` → 低通 + 跳变限制 → 发布 `/odom`（`nav_msgs/Odometry`）
- **航向角 → IMU**：订阅 `/chassis/heading_rad` → 发布 `/imu/data`（`sensor_msgs/Imu`，yaw-only）
- **底盘串口**：`/cmd_vel` → 下发速度角速度；同时解析回传速度/航向角/激光最近距离

## 数据协议核对（已做完）

### 1) 无人机 → 小车（LoRa）
- **包格式**：`FF 01 X(float32) Y(float32) FE`
- **帧长**：11 字节
- **端序**：默认 little-endian（对应 `struct.pack("<ff", x, y)`）
- **接收节点**：`car_ground_nav/lora_goal_receiver.py`
- **输出话题**：`/uav_goal`（`geometry_msgs/PoseStamped`）

### 2) 小车上位机 ↔ 下位机（底盘串口）
- **下发 cmd_vel**：`FF 02 v(float32) w(float32) FE`
- **回传当前速度（车体速度）**：`FF 03 vx(float32) vy(float32) FE`
  - `vx/vy` → `/chassis/vx_m_s`、`/chassis/vy_m_s`
  - `speed = hypot(vx,vy)` → `/chassis/speed_m_s`
- **回传姿态（航向角 + 角速度）**：`FF 04 yaw(float32) w(float32) FE`
  - `yaw` → `/chassis/heading_rad`
  - `w` → `/chassis/w_rad_s`

对应节点：`car_ground_nav/chassis_serial_node.py`

### 3) UWB USB 串口（定位）
- 解析头：`CmdM:4`
- 固定帧长：101 字节（校验 payload_len 与 `\r\n` 结尾）
- 3 基站一次解算 2D，输出 `(x,y,residual)`

对应节点：`car_ground_nav/uwb_serial_localizer.py`

## 雷达（LSLidar ROS2 SDK）

你们的雷达驱动在 `Car_ Navigation_accordingtofly/2.ROS2_SDK/LSLIDAR_X_ROS2-20240228/`。

- 默认发布：`/scan`（`sensor_msgs/LaserScan`）
- 默认 `frame_id`：`laser`
- 串口设备名（示例 yaml）：`/dev/wheeltec_laser`

本包提供启动整合：`bringup_with_lidar.launch.py`，会同时启动：
- `car_ground_nav` 的所有节点
- `lslidar_driver`（默认 `lsn10_launch.py`）
- 静态 TF：`base_link -> laser`（需要你按安装位置修改 xyz/rpy）

## 启动方式

### 只启动小车上位机核心节点（不含雷达）

```bash
ros2 launch car_ground_nav ground_nav.launch.py
```

### 启动核心节点 + 雷达 + 静态 TF

```bash
ros2 launch car_ground_nav bringup_with_lidar.launch.py
```

### 边建图边导航（SLAM Toolbox + Nav2）

前提：
- 雷达已能发布 `/scan`（本工程已集成 LSLidar 驱动）
- 本包能提供 `odom->base_link`（已在 `params.yaml` 中打开 `uwb_filtered_odom.publish_tf=true`）

启动：

```bash
ros2 launch car_ground_nav slam_nav_bringup.launch.py
```

说明：
- `slam_toolbox` 会发布 `map->odom`（边走边建图）
- Nav2 使用 `map` 作为全局坐标系、`odom` 作为局部坐标系

## 目前还缺什么（导航真正跑起来必须具备）

- **坐标系闭环（TF）**：
  - `odom -> base_link`：由 `uwb_filtered_odom` 发布（已默认打开）
  - `map -> odom`：由 `slam_toolbox` 发布（边建图边导航）
- **雷达安装位姿 TF**：`base_link -> laser` 的静态 TF 需要按实机标定改对
- **定位融合策略**：
  - 当前是“UWB 给位置、底盘给 yaw”，已融合成 `/uwb_pose_fused` 和 `/odom`
  - 如需更稳，可接 `robot_localization` 做 EKF（后续可加）

