# 空地协同智能抓取系统 — 软件技术文档

> **项目类型**：空地协同（UAV + UGV + 机械臂 + 双视觉）  
> **地面平台**：RDK X5 · ROS 2 Humble · Nav2 · WHEELTEC A150  
> **空中平台**：PX4 飞控 · MAVLink · MAVROS · RDK X5（机载）  
> **文档范围**：`RDK_Navigation` 地面栈 + `无人机/` 空中栈 + 协同接口 + 仿真验证

---

## 1. 项目概述

本系统实现 **「无人机巡航侦察 → 识别地面目标 → 向小车下发坐标 → 小车自主导航 → 机械臂抓取 → 双方返航」** 的完整空地协同闭环。

### 1.1 协同任务定义

场地布置（与 `air_ground_sim` 一致）：

| 元素 | 说明 |
|------|------|
| 台子 | 场地中央放置平台 |
| 小球 | 置于台顶，为机械臂抓取目标 |
| 地面 QR | 贴在台子正前方地面，供无人机俯视识别 |
| 小车任务 | 导航到 **QR 地面坐标**（台子正前方），再抓取台顶小球 |

### 1.2 子系统分工

| 子系统 | 硬件 | 软件职责 | 工程目录 |
|--------|------|----------|----------|
| **无人机** | PX4 多旋翼 + 机载 RDK + MIPI 相机 | 定点巡航、QR 识别、位姿解算、MAVLink 飞控、向地面发目标 | `../无人机/drone_ws` |
| **小车** | RDK + STM32 底盘 + N10 激光 | SLAM/Nav2 导航、多航点任务、脱困回家 | `RDK_Navigation/ros2_ws` |
| **机械臂** | WHEELTEC A150 + USB 相机 | 官方视觉找色 + 一次抓取（不自研） | `~/wheeltec_arm`（官方） |
| **视觉** | 无人机 MIPI + 机械臂 USB _cam | 空中：QR+PnP；地面：HSV 找色+IK | `无人机/drone_qr` + `wheeltec_color_sort` |

### 1.3 设计原则

1. **官方包不改源码**：`wheeltec_arm` / `wheeltec_color_sort` 仅 launch 调用  
2. **端口集中配置**：地面 `~/device_ports.yaml`；空中相机/串口独立配置  
3. **职责分离**：`grab_mission.py` 只管底盘对齐，抓取动作由官方 `color_pick` 完成  
4. **一次抓取硬停**：识别到合爪+`arm_home` 后立即 kill `pick_color`，防止二次开爪  

---

## 2. 系统总体架构

```
                    ┌─────────────────────────────────────┐
                    │         空中子系统（无人机板）        │
                    │  drone_ws @ RDK X5 (TROS/Humble)    │
                    │                                     │
                    │  mipi_cam → /image_raw              │
                    │       ↓                             │
                    │  drone_qr (qr_core) → /qr_pose      │
                    │       ↓                             │
                    │  drone_controller (航点巡航状态机)    │
                    │       ↓                             │
                    │  drone_mavros (mavros_core)         │
                    │       ↓ MAVLink                     │
                    │  PX4 飞控 ← 起飞/巡航/悬停/降落      │
                    │       ↓                             │
                    │  drone_communication                │
                    │    /qr_target_for_car               │
                    │         → /car/qr_goal              │
                    └──────────────┬──────────────────────┘
                                   │ 无线/局域网 ROS 或桥接
                    ┌──────────────▼──────────────────────┐
                    │         地面子系统（小车板）          │
                    │  RDK_Navigation @ RDK X5            │
                    │                                     │
                    │  T1 run_navigation.sh               │
                    │    base_driver + lslidar + Nav2     │
                    │                                     │
                    │  T2 run_full_mission.sh             │
                    │    ├─ PHASE1 多航点 Nav2            │
                    │    ├─ PHASE2 pick_color + grab      │
                    │    └─ PHASE3 脱困 + 回家            │
                    │                                     │
                    │  pick_color.launch                  │
                    │    usb_cam → find_color → color_pick│
                    │    grab_mission（仅底盘对齐）         │
                    └─────────────────────────────────────┘
```

---

## 3. 协同任务流程

### 3.1 端到端时序

```
无人机                          通信                     小车+机械臂
  │                              │                          │
  ├─ 起飞至巡航高度               │                          │
  ├─ 定点航点巡航                 │                          │  (T1 Nav2 待命)
  ├─ 俯视识别地面 QR              │                          │
  ├─ PnP 解算 + 转全局坐标        │                          │
  ├─ 发布 /qr_global_pose        │                          │
  ├─ 悬停等待 ──────────────────►│ /car/qr_goal ───────────►│ 接收目标（待接入）
  │                              │                          ├─ Nav2 至 QR 地面点
  │                              │                          ├─ 底盘视觉对齐
  │                              │                          ├─ 官方 color_pick 抓球
  │                              │                          ├─ 脱困 + 回 home
  ├─ 收到完成信号 ◄──────────────│◄─────────────────────────┤
  └─ 返航降落                     │                          │
```

### 3.2 地面三段任务（已实现）

| 阶段 | 脚本 | 内容 |
|------|------|------|
| PHASE 1 | `run_mission.py` | `home → wp_09 → wp_10 → wp_11 → box_approach` |
| PHASE 2 | `run_full_mission.py` + `grab_mission.py` | 启 `pick_color` → 底盘对齐 → 一次抓取 → kill 视觉栈 |
| PHASE 3 | `run_mission.py` | 直行脱困 → 转 90° → Nav2 回 `home` |

### 3.3 空中任务状态机（`drone_controller`）

| 状态 | 行为 |
|------|------|
| `IDLE` | 发送起飞指令 `command=0` |
| `TAKEOFF` | 等待到达目标高度 |
| `CRUISE` | 依次飞往预设航点 `(x,y,z)` |
| `QR_FOUND` | 中断巡航，发布全局 QR 坐标 |
| `LANDING` | 降落 `command=2` |
| `FINISHED` | 任务结束 |

识别到有效 QR（连续 ≥3 帧）时 **中断巡航**，悬停并发布 `/qr_global_pose`。

---

## 4. 地面子系统 — 小车导航

### 4.1 硬件接口

```yaml
# ~/device_ports.yaml（RDK 上唯一生效副本）
device_ports:
  chassis: /dev/ttyUSB0    # STM32 底盘
  lidar: /dev/ttyACM0      # LSLIDAR N10
  arm_mcu: /dev/ttyACM1    # 机械臂控制器
  arm_camera: /dev/video0  # 机械臂 USB 相机
```

```bash
python3 ~/Desktop/load_ports.py --show
```

### 4.2 ROS 2 包结构

```
ros2_ws/src/
├── base_driver/        # 底盘串口桥：/cmd_vel ↔ STM32，发布 /odom /imu/data
├── lslidar_driver/     # N10 雷达 → /scan
├── lslidar_msgs/
├── car_description/    # URDF + TF（base_link → laser）
├── car_bringup/        # mapping.launch.py / navigation.launch.py
└── car_navigation/     # Nav2 参数 nav2_params.yaml、地图配置
```

### 4.3 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 速度指令 → 底盘 |
| `/odom` | `nav_msgs/Odometry` | 里程计（底盘积分） |
| `/scan` | `sensor_msgs/LaserScan` | 2D 激光 |
| `/map` | `nav_msgs/OccupancyGrid` | 栅格地图 |

### 4.4 启动方式

**T1 — 导航栈（常开）**

```bash
bash ~/Desktop/run_navigation.sh
# 地图默认：/home/sunrise/maps/arena_map_v5.yaml
```

**T2 — 全任务**

```bash
bash ~/Desktop/run_full_mission.sh
```

### 4.5 航点与任务参数

| 文件 | RDK 路径 | 说明 |
|------|----------|------|
| `config/mission_waypoints_v5.yaml` | `~/mission_waypoints.yaml` | 航点坐标 + mission 调参 |
| `config/device_ports.yaml` | `~/device_ports.yaml` | 端口模板 |

默认路线：`wp_09 → wp_10 → wp_11 → box_approach`（抓取前站位）

精调参数：`fine_at_via`、`fine_xy_m`、`fine_yaw_deg`、`final_coarse_handoff_xy_m` 等。

脱困参数：`escape_forward_m=0.15`、`escape_rotate_deg=90`、`escape_rotate_sign=-1.0`。

---

## 5. 机械臂子系统

### 5.1 官方软件栈（不修改）

```bash
source ~/wheeltec_arm/install/setup.bash
ros2 launch wheeltec_color_sort pick_color.launch.py \
  target_color:=green \
  port:=/dev/ttyACM1 \
  video_device:=/dev/video0
```

| 节点 | 职责 |
|------|------|
| `usb_cam_node` | 发布 `/image_raw` |
| `find_color_node` | HSV 找绿色块，发布 `/color_ik_result` |
| `color_pick_node` | 收到稳定 IK 后：下探 → 合爪 → `arm_home` |

### 5.2 自研底盘对齐 — `grab_mission.py`

**只做底盘**，不重复官方抓取逻辑：

| 状态 | 行为 |
|------|------|
| `ALIGN` | 绿块进 ROI / 黑边引导靠近 |
| `HANDOFF` | 收到 `/color_ik_result` 后停车，交给 `color_pick` |
| 退出条件 | `/joint_states` 检测 **合爪 + arm_home 稳定** → DONE |

`run_full_mission.py` 在退出后立即 `kill_pick_color_stack()`，防止官方 reset 开爪。

### 5.3 抓取完成判定（关节状态）

| 信号 | 判定条件 |
|------|----------|
| 合爪 | `joint_10 ≥ 0.45` |
| arm_home | `joint_1~5` 均接近 `0`（容差 0.18 rad） |
| 稳定 | 持续 `grab_pick_home_hold_s`（默认 0.6 s） |
| 兜底 | `grab_pick_kill_max_s`（默认 14 s）强制杀节点 |

---

## 6. 视觉子系统

本项目的视觉分为 **空中 QR** 与 **地面找色** 两条独立链路。

### 6.1 无人机视觉 — QR 识别 + PnP

**包**：`无人机/drone_ws/src/drone_qr`（`qr_core.py`）

| 项目 | 说明 |
|------|------|
| 相机 | MIPI IMX219，`mipi_cam.launch.py` |
| 图像话题 | `/image_raw`（BEST_EFFORT QoS） |
| 检测库 | `pyzbar` + OpenCV 预处理（CLAHE、亚像素角点） |
| 输出 | `/qr_pose`（`geometry_msgs/PoseStamped`，`camera_link` 系） |
| 标定 | IMX219 内参（重投影误差 ~1.03 px） |

**相机内参（第一版标定）**：

```
K = [[2431.36, 0, 976.42],
     [0, 2411.67, 512.39],
     [0, 0, 1]]
dist = [-0.025, 0.383, -0.0018, 0.00055, -1.040]
```

**启动相机**：

```bash
source /opt/tros/humble/setup.bash
ros2 launch mipi_cam mipi_cam.launch.py \
  mipi_channel:=0 mipi_sensor_type:=imx219 \
  mipi_image_width:=1920 mipi_image_height:=1080 gdc_enable:=false
```

**调试**：

```bash
ros2 topic echo /qr_pose
```

QR 内容可编码 JSON：`type=pick_target, qr_x, qr_y, ball_x, ball_y, ball_z`，与仿真 `air_ground_sim` 格式一致。

### 6.2 地面视觉 — 官方找色抓取

| 项目 | 说明 |
|------|------|
| 相机 | USB，`video_device` 来自 `device_ports.yaml` |
| 检测 | HSV 绿色块 + 稳定性确认（`confirm_count`） |
| 关键话题 | `/color_ik_result`（`a150_arm_msgs/ColorIkResult`） |
| 抓取 | 官方 `color_pick_node` 完整序列 |

### 6.3 地面 QR 独立测试（可选）

目录 `../rdk_qr_test/`：通用 USB 摄像头 QR 测试，不依赖机械臂/ROS：

```bash
python3 qr_core.py --device /dev/video0 --hold-s 5
```

---

## 7. 无人机子系统 — MAVLink / PX4 / MAVROS

### 7.1 工程结构

```
无人机/
├── README.md
└── drone_ws/                    # 机载 ROS 2 工作空间
    └── src/
        ├── drone_interfaces/      # 自定义服务 DroneMavros.srv
        ├── drone_mavros/          # mavros_core.py — PX4 通信层
        ├── drone_qr/              # qr_core.py — QR 识别
        ├── drone_controller/      # controller_fixed.py — 航点巡航主控
        ├── drone_communication/   # communication_core.py — 空地通信
        └── drone_navigation/      # （弃用，改打点巡航）
```

> 注：仓库中可能仅含 `build/` 编译产物，完整源码以机载 `~/drone_ws` 为准。

### 7.2 飞控通信架构

```
drone_controller
      │  ROS 2 Service: /mavros_to_controller
      │  (drone_interfaces/DroneMavros)
      ▼
drone_mavros (mavros_core.py)
      │  MAVROS API
      │  /mavros/setpoint_position/local
      │  /mavros/state, /mavros/local_position/pose
      ▼
PX4 飞控 (MAVLink)
```

### 7.3 `DroneMavros` 服务接口

**服务名**：`/mavros_to_controller`  
**类型**：`drone_interfaces/srv/DroneMavros`

**Request**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `command` | int32 | 指令码（见下表） |
| `input1` | string | 扩展字符串参数 |
| `input_num_1` | float | 数值参数 1 |
| `input_num_2` | float | 数值参数 2 |
| `input_num_3` | float | 数值参数 3 |

**Response**：`success` (bool), `feedback` (string)

**指令码**（`drone_controller` 使用）：

| command | 含义 | 参数 |
|---------|------|------|
| `0` | 起飞 | `input_num_1` = 目标高度 (m) |
| `2` | 降落 | — |
| `5` | 悬停 | — |
| `6` | 飞往局部坐标 | `input_num_1/2/3` = x, y, z |

### 7.4 无人机关键话题

| 话题 | 方向 | 说明 |
|------|------|------|
| `/image_raw` | 相机 → QR | MIPI 图像 |
| `/qr_pose` | QR → 主控 | 相机系下 QR 位姿 |
| `/drone/current_pose` | MAVROS → 主控 | 无人机全局位姿 |
| `/qr_global_pose` | 主控 → 通信 | QR 世界坐标（近似叠加） |
| `/qr_target_for_car` | 主控 → 通信 | 下发给地面的目标 |
| `/car/qr_goal` | 通信 → 地面 | 小车导航目标点 |

### 7.5 机载启动流程（参考）

```bash
# 1. 相机
source /opt/tros/humble/setup.bash
ros2 launch mipi_cam mipi_cam.launch.py ...

# 2. MAVROS + PX4（按飞控连接方式配置 fcu_url）
# ros2 launch mavros px4.launch ...

# 3. 编译并启动 drone 栈
cd ~/drone_ws
colcon build
source install/setup.bash

ros2 run drone_mavros mavros_core          # 或对应 launch
ros2 run drone_qr qr_core
ros2 run drone_controller controller_fixed
ros2 run drone_communication communication_core
```

### 7.6 航点巡航配置

`controller_fixed.py` 预设航点（局部坐标系，单位 m）：

```python
waypoints = [
    (2.0, 0.0, 2.0),
    (2.0, 2.0, 2.0),
    (0.0, 2.0, 2.0),
    (0.0, 0.0, 2.0),
]
target_altitude = 2.0
position_threshold = 0.3   # 到点判定 (m)
qr_min_consecutive = 3     # QR 连续帧确认
```

---

## 8. 空地通信

### 8.1 当前实现

`drone_communication/communication_core.py`：

```python
订阅: /qr_target_for_car  (PoseStamped)
发布: /car/qr_goal         (PoseStamped)
```

将无人机识别到的 QR 全局坐标转发给地面小车。

### 8.2 地面接入（规划/待完善）

地面 `RDK_Navigation` 侧建议：

1. 新增节点订阅 `/car/qr_goal`（或等效 MQTT/UDP 桥接）
2. 将 `(x, y)` 写入 `mission_waypoints.yaml` 临时目标，或调用 `MissionRunner.go_to_point()`
3. 小车到达后触发既有 PHASE 2 抓取流程
4. 完成后向无人机发布 `mission_complete` 话题

### 8.3 与仿真的对应关系

`air_ground_sim/` 用内存消息模拟上述链路：

| 仿真 | 实机 |
|------|------|
| `TargetReport` | `/qr_pose` + 坐标变换 |
| `DRONE_SEND_TARGET` | `/car/qr_goal` |
| `MissionComplete` | 地面完成回调 |

```bash
cd air_ground_sim
python run_sim.py          # Pygame 可视化仿真
python run_sim.py --3d     # PyBullet 3D（可选）
```

---

## 9. 仓库目录总览

```
大创/
├── RDK_Navigation/           # ★ 地面小车+机械臂任务（本文档主工程）
│   ├── README.md
│   ├── config/
│   │   ├── device_ports.yaml
│   │   └── mission_waypoints_v5.yaml
│   ├── scripts/
│   │   ├── run_navigation.sh       # T1
│   │   ├── run_full_mission.sh     # T2
│   │   ├── run_full_mission.py
│   │   ├── run_mission.py
│   │   ├── grab_mission.py
│   │   └── load_ports.py
│   ├── ros2_ws/
│   └── docs/
│       ├── 00_整体方案与思路.md
│       └── 01_下位机串口协议.md
│
├── 无人机/                      # ★ 空中 PX4+MAVROS+QR 栈
│   ├── README.md
│   └── drone_ws/ (src + build)
│
├── air_ground_sim/              # 空地协同仿真验证
└── rdk_qr_test/                 # 独立 QR 摄像头测试
```

---

## 10. 部署与同步

### 10.1 地面 RDK 文件同步（PC → 小车板）

```powershell
scp "...\RDK_Navigation\scripts\run_navigation.sh" sunrise@192.168.43.205:~/Desktop/
scp "...\RDK_Navigation\scripts\run_full_mission.sh" sunrise@192.168.43.205:~/Desktop/
scp "...\RDK_Navigation\scripts\run_full_mission.py" sunrise@192.168.43.205:~/Desktop/
scp "...\RDK_Navigation\scripts\run_mission.py" sunrise@192.168.43.205:~/Desktop/
scp "...\RDK_Navigation\scripts\grab_mission.py" sunrise@192.168.43.205:~/Desktop/
scp "...\RDK_Navigation\scripts\load_ports.py" sunrise@192.168.43.205:~/Desktop/
scp "...\RDK_Navigation\config\mission_waypoints_v5.yaml" sunrise@192.168.43.205:~/mission_waypoints.yaml
scp "...\RDK_Navigation\config\device_ports.yaml" sunrise@192.168.43.205:~/device_ports.yaml
scp "...\RDK_Navigation\ros2_ws\src\car_navigation\config\nav2_params.yaml" sunrise@192.168.43.205:~/Desktop/ros2_ws/src/car_navigation/config/
```

### 10.2 机载无人机板

在无人机 RDK 上维护 `~/drone_ws`，按 §7.5 编译运行。确保 PX4 与 MAVROS 的 `fcu_url` 正确（USB/UDP 视飞控连接而定）。

### 10.3 依赖

| 平台 | 关键依赖 |
|------|----------|
| 地面 | ROS 2 Humble, Nav2, `wheeltec_arm`, `wheeltec_color_sort` |
| 空中 | TROS/Humble, MAVROS, PX4, `pyzbar`, OpenCV, `mipi_cam` |

---

## 11. 调参与排障

### 11.1 地面导航

| 现象 | 处理 |
|------|------|
| 转弯抖、慢 | 降低 `nav2_params.yaml` 中 `max_vel_theta` |
| 到点绕圈 | 调 `progress_checker.required_movement_radius` |
| 改 Nav2 参数无效 | 需重启 T1 `run_navigation.sh` |

### 11.2 机械臂 / 相机

| 现象 | 处理 |
|------|------|
| 摄像头白屏 | 确认 `arm_camera`；关 VNC Camera 窗口；`pick_color` 需传 `video_device` |
| 夹爪抓后又开 | 缩短 `grab_pick_kill_max_s`；确认 joint 判定后 kill |
| 未抬完就杀 | 勿用固定 IK 计时；依赖 `/joint_states` 合爪+home |

### 11.3 无人机

| 现象 | 处理 |
|------|------|
| `/qr_pose` 无输出 | 检查 MIPI 相机、光照、QR 尺寸参数 `qr_side_length` |
| 定位偏差大 | 重新标定相机内参；检查 QR 边长是否与实际一致 |
| 飞控无响应 | 检查 MAVROS 连接、`/mavros/state` 是否 `connected` |
| 误识别 QR | 增大 `qr_min_consecutive` |

### 11.4 端口占用

```bash
sudo fuser -k /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyACM1
pkill -f usb_cam; pkill -f pick_color
```

---

## 12. 版本记录

| 模块 | 版本/备注 |
|------|-----------|
| 地图 | `arena_map_v5` |
| 航点 | `mission_waypoints_v5` |
| 地面 ROS | Humble |
| 空中 ROS | TROS Humble（机载） |
| 仿真 | `air_ground_sim` v1（定点巡航+QR+抓取） |
| 官方机械臂 | `wheeltec_arm` + `wheeltec_color_sort`（不修改） |

---

## 13. 相关文档

| 文档 | 路径 |
|------|------|
| 小车建图导航方案 | `docs/00_整体方案与思路.md` |
| STM32 串口协议 | `docs/01_下位机串口协议.md` |
| 无人机 README | `../无人机/README.md` |
| 空地仿真 | `../air_ground_sim/run_sim.py` |

---

*西北工业大学 · 空地协同智能抓取 · 软件技术文档*
