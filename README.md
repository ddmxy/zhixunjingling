# Air\-Ground Cooperative Intelligent Grasping System — Software Technical Document

> **Project Type**: Air\-Ground Cooperation \(UAV \+ UGV \+ Robotic Arm \+ Dual Vision\)
> **Ground Platform**: RDK X5 · ROS 2 Humble · Nav2 · WHEELTEC A150
> **Airborne Platform**: PX4 Flight Controller · MAVLink · MAVROS · RDK X5 \(Onboard\)
> **Document Scope**: `RDK_Navigation` Ground Stack \+ `drone/` Air Stack \+ Cooperative Interface \+ Simulation Verification

---

## 1\. Project Overview

This system implements a complete air\-ground cooperative closed loop:
**UAV Cruise Reconnaissance → Ground Target Recognition → Coordinate Dispatch to UGV → Autonomous UGV Navigation → Robotic Arm Grasping → Dual Platform Return Home**

### 1\.1 Cooperative Task Definition

Field layout \(consistent with `air_ground_sim`\):

| Element        | Description                                                                                                   |
| -------------- | ------------------------------------------------------------------------------------------------------------- |
| Platform       | Placed at the center of the field                                                                             |
| Small Sphere   | On top of the platform, the grasping target for the robotic arm                                               |
| Ground QR Code | Pasted on the ground directly in front of the platform for downward UAV recognition                           |
| UGV Mission    | Navigate to the **QR ground coordinate** \(front of the platform\), then grasp the sphere on the platform top |

### 1\.2 Subsystem Division of Labor

| Subsystem       | Hardware                                     | Software Responsibilities                                                                                           | Project Directory                         |
| --------------- | -------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- | ----------------------------------------- |
| **UAV**         | PX4 Multirotor \+ Onboard RDK \+ MIPI Camera | Fixed\-point cruise, QR recognition, pose solving, MAVLink flight control, target coordinate transmission to ground | `../drone/drone_ws`                       |
| **UGV**         | RDK \+ STM32 Chassis \+ N10 LiDAR            | SLAM/Nav2 navigation, multi\-waypoint tasks, escape \& homing                                                       | `RDK_Navigation/ros2_ws`                  |
| **Robotic Arm** | WHEELTEC A150 \+ USB Camera                  | Official color detection grasping logic \(no self\-developed algorithm\)                                            | `~/wheeltec_arm` \(Official Package\)     |
| Vision Module   | UAV MIPI Camera \+ Robotic Arm USB Camera    | Airside: QR \+ PnP; Groundside: HSV color segmentation \+ IK Solver                                                 | `drone/drone_qr` \+ `wheeltec_color_sort` |

### 1\.3 Design Principles

1. **No Modification to Official Packages**: Only launch files are called for `wheeltec_arm` / `wheeltec_color_sort`, source code remains untouched

2. **Centralized Port Configuration**: Groundside `~/device_ports.yaml`; independent config for airborne cameras and serial ports

3. **Separation of Responsibilities**: `grab_mission.py` only handles chassis alignment; grasping actions are delegated to official `color_pick` node

4. **Single Grasp Hard Stop**: Once claw closure \+ `arm_home` state is detected, immediately kill `pick_color` to prevent unintended re\-opening of the gripper

---

## 2\. Overall System Architecture

```Plain Text
┌─────────────────────────────────────┐
                    │         Airborne Subsystem (Drone Board)        │
                    │  drone_ws @ RDK X5 (TROS/Humble)    │
                    │                                     │
                    │  mipi_cam → /image_raw              │
                    │       ↓                             │
                    │  drone_qr (qr_core) → /qr_pose      │
                    │       ↓                             │
                    │  drone_controller (Waypoint Cruise State Machine)    │
                    │       ↓                             │
                    │  drone_mavros (mavros_core)         │
                    │       ↓ MAVLink                     │
                    │  PX4 Flight Controller ← Takeoff/Cruise/Hover/Landing      │
                    │       ↓                             │
                    │  drone_communication                │
                    │    /qr_target_for_car               │
                    │         → /car/qr_goal              │
                    └──────────────┬──────────────────────┘
                                   │ Wireless/LAN ROS Bridge
                    ┌──────────────▼──────────────────────┐
                    │         Ground Subsystem (UGV Board)          │
                    │  RDK_Navigation @ RDK X5            │
                    │                                     │
                    │  T1 run_navigation.sh               │
                    │    base_driver + lslidar + Nav2     │
                    │                                     │
                    │  T2 run_full_mission.sh             │
                    │    ├─ PHASE1 Multi-waypoint Nav2            │
                    │    ├─ PHASE2 pick_color + grab      │
                    │    └─ PHASE3 Escape + Homing            │
                    │                                     │
                    │  pick_color.launch                  │
                    │    usb_cam → find_color → color_pick│
                    │    grab_mission (Chassis Alignment Only)         │
                    └─────────────────────────────────────┘
```

---

## 3\. Cooperative Task Workflow

### 3\.1 End\-to\-End Timing Sequence

```Plain Text
UAV                          Communication                     UGV + Robotic Arm
  │                              │                          │
  ├─ Takeoff to cruising height               │                          │
  ├─ Fixed-point waypoint cruise                 │                          │  (T1 Nav2 Standby)
  ├─ Downward view capture & ground QR detection              │                          │
  ├─ PnP Solver + Global Coordinate Transformation        │                          │
  ├─ Publish /qr_global_pose        │                          │
  ├─ Hover & Wait ──────────────────►│ /car/qr_goal ───────────►│ Receive navigation target (Pending Integration)
  │                              │                          ├─ Nav2 navigate to QR ground point
  │                              │                          ├─ Chassis visual fine alignment
  │                              │                          ├─ Official color_pick sphere grasping
  │                              │                          ├─ Obstacle escape + Return to home
  ├─ Receive Mission Complete Signal ◄──────────────│◄─────────────────────────┤
  └─ Return & Land                     │                          │
```

### 3\.2 Three\-Stage Ground Task \(Implemented\)

| Stage   | Script                                     | Content                                                                         |
| ------- | ------------------------------------------ | ------------------------------------------------------------------------------- |
| PHASE 1 | `run_mission.py`                           | `home → wp_09 → wp_10 → wp_11 → box_approach`                                   |
| PHASE 2 | `run_full_mission.py` \+ `grab_mission.py` | Launch `pick_color` → Chassis alignment → Single grasp → Terminate vision stack |
| PHASE 3 | `run_mission.py`                           | Straight\-line escape → 90° rotation → Nav2 return to `home`                    |

### 3\.3 Airborne Task State Machine \(`drone_controller`\)

| State      | Behavior                                           |
| ---------- | -------------------------------------------------- |
| `IDLE`     | Send takeoff command `command=0`                   |
| `TAKEOFF`  | Wait until target altitude is reached              |
| `CRUISE`   | Fly to predefined waypoints `(x,y,z)` sequentially |
| `QR_FOUND` | Interrupt cruise, publish global QR coordinates    |
| `LANDING`  | Trigger landing `command=2`                        |
| `FINISHED` | Mission terminated                                 |

Cruise will be interrupted only when valid QR codes are detected for **≥3 consecutive frames**, then the drone hovers and publishes `/qr_global_pose`\.

---

## 4\. Ground Subsystem — UGV Navigation

### 4\.1 Hardware Interfaces

```yaml
# ~/device_ports.yaml (Only valid copy on RDK)
device_ports:
  chassis: /dev/ttyUSB0 # STM32 Chassis Controller
  lidar: /dev/ttyACM0 # LSLIDAR N10 LiDAR
  arm_mcu: /dev/ttyACM1 # Robotic Arm Controller
  arm_camera: /dev/video0 # Robotic Arm USB Camera
```

```bash
python3 ~/Desktop/load_ports.py --show
```

### 4\.2 ROS 2 Package Structure

```Plain Text
ros2_ws/src/
├── base_driver/        # Chassis serial bridge: /cmd_vel ↔ STM32, publish /odom /imu/data
├── lslidar_driver/     # N10 LiDAR driver → /scan topic
├── lslidar_msgs/
├── car_description/    # URDF + TF Tree (base_link → laser)
├── car_bringup/        # mapping.launch.py / navigation.launch.py
└── car_navigation/     # Nav2 parameter nav2_params.yaml, map configuration
```

### 4\.3 Core ROS Topics

| Topic      | Message Type             | Description                            |
| ---------- | ------------------------ | -------------------------------------- |
| `/cmd_vel` | `geometry_msgs/Twist`    | Velocity command from Nav2 to chassis  |
| `/odom`    | `nav_msgs/Odometry`      | Wheel odometry \(chassis integration\) |
| `/scan`    | `sensor_msgs/LaserScan`  | 2D LiDAR scan data                     |
| `/map`     | `nav_msgs/OccupancyGrid` | Occupancy grid navigation map          |

### 4\.4 Startup Scripts

**T1 — Persistent Navigation Stack**

```bash
bash ~/Desktop/run_navigation.sh
# Default map: /home/sunrise/maps/arena_map_v5.yaml
```

**T2 — Full Autonomous Mission**

```bash
bash ~/Desktop/run_full_mission.sh
```

### 4\.5 Waypoint \& Mission Parameters

| File                               | RDK Path                   | Description                                       |
| ---------------------------------- | -------------------------- | ------------------------------------------------- |
| `config/mission_waypoints_v5.yaml` | `~/mission_waypoints.yaml` | Waypoint coordinates \+ mission tuning parameters |
| `config/device_ports.yaml`         | `~/device_ports.yaml`      | Serial port template                              |

Default route: `wp_09 → wp_10 → wp_11 → box_approach` \(pre\-grasp parking position\)
Fine\-tuning parameters: `fine_at_via`, `fine_xy_m`, `fine_yaw_deg`, `final_coarse_handoff_xy_m`, etc\.
Escape parameters: `escape_forward_m=0.15`, `escape_rotate_deg=90`, `escape_rotate_sign=-1.0`\.

---

## 5\. Robotic Arm Subsystem

### 5\.1 Official Software Stack \(Unmodified\)

```bash
source ~/wheeltec_arm/install/setup.bash
ros2 launch wheeltec_color_sort pick_color.launch.py \
  target_color:=green \
  port:=/dev/ttyACM1 \
  video_device:=/dev/video0
```

| Node              | Responsibility                                                      |
| ----------------- | ------------------------------------------------------------------- |
| `usb_cam_node`    | Publish raw image stream `/image_raw`                               |
| `find_color_node` | HSV green blob detection, output `/color_ik_result`                 |
| `color_pick_node` | Upon stable IK result: Descend → Close gripper → Move to `arm_home` |

### 5\.2 Self\-developed Chassis Alignment — `grab_mission.py`

This module only controls the mobile chassis and does not replicate official grasping logic:

| State          | Behavior                                                                              |
| -------------- | ------------------------------------------------------------------------------------- |
| `ALIGN`        | Guide UGV to center green blob within ROI via black border feedback                   |
| `HANDOFF`      | Stop UGV after receiving `/color_ik_result`, hand over control to `color_pick`        |
| Exit Condition | Detect stable gripper closed \+ arm home state via `/joint_states` → Mission Complete |

After mission exit, `run_full_mission.py` executes `kill_pick_color_stack()` immediately to prevent official package from resetting and opening the gripper again\.

### 5\.3 Grasp Completion Judgment \(Joint States\)

| Signal            | Criterion                                                              |
| ----------------- | ---------------------------------------------------------------------- |
| Gripper Closed    | `joint_10 ≥ 0.45`                                                      |
| Arm Home Position | `joint_1~5` all close to 0 rad \(tolerance 0\.18 rad\)                 |
| Stable Hold       | Maintain state for `grab_pick_home_hold_s` \(default 0\.6 s\)          |
| Fallback Timeout  | Force\-kill vision stack after `grab_pick_kill_max_s` \(default 14 s\) |

---

## 6\. Vision Subsystem

Vision pipeline is split into two independent branches: **Airborne QR Detection** and **Groundside Color Grasping**\.

### 6\.1 UAV Vision — QR Recognition \+ PnP Pose Estimation

**Package**: `drone/drone_ws/src/drone_qr` \(`qr_core.py`\)

| Item              | Details                                                              |
| ----------------- | -------------------------------------------------------------------- |
| Camera            | MIPI IMX219, launched via `mipi_cam.launch.py`                       |
| Image Topic       | `/image_raw` \(BEST_EFFORT QoS\)                                     |
| Detection Library | pyzbar \+ OpenCV preprocessing \(CLAHE, subpixel corner refinement\) |
| Output Topic      | `/qr_pose` \(`geometry_msgs/PoseStamped`, camera_link frame\)        |
| Calibration       | IMX219 intrinsic parameters, reprojection error \~1\.03 px           |

**First\-version Camera Intrinsic Matrix**:

```Plain Text
K = [[2431.36, 0, 976.42],
     [0, 2411.67, 512.39],
     [0, 0, 1]]
dist = [-0.025, 0.383, -0.0018, 0.00055, -1.040]
```

**Camera Launch Command**:

```bash
source /opt/tros/humble/setup.bash
ros2 launch mipi_cam mipi_cam.launch.py \
  mipi_channel:=0 mipi_sensor_type:=imx219 \
  mipi_image_width:=1920 mipi_image_height:=1080 gdc_enable:=false
```

**Debugging Command**:

```bash
ros2 topic echo /qr_pose
```

QR code payload supports JSON encoding: `type=pick_target, qr_x, qr_y, ball_x, ball_y, ball_z`, matching the format used in `air_ground_sim`\.

### 6\.2 Ground Vision — Official Color\-Based Grasping

| Item               | Details                                                                |
| ------------------ | ---------------------------------------------------------------------- |
| Camera             | USB camera, `video_device` loaded from `device_ports.yaml`             |
| Detection Pipeline | HSV green blob filtering \+ stability confirmation \(`confirm_count`\) |
| Key Topic          | `/color_ik_result` \(`a150_arm_msgs/ColorIkResult`\)                   |
| Grasp Execution    | Full motion sequence handled by official `color_pick_node`             |

### 6\.3 Standalone Ground QR Test \(Optional\)

Directory `../rdk_qr_test/`: General\-purpose USB camera QR test, independent of robotic arm and ROS:

```bash
python3 qr_core.py --device /dev/video0 --hold-s 5
```

---

## 7\. UAV Subsystem — MAVLink / PX4 / MAVROS

### 7\.1 Project Structure

```Plain Text
drone/
├── README.md
└── drone_ws/                    # Onboard ROS 2 Workspace
    └── src/
        ├── drone_interfaces/      # Custom Service DroneMavros.srv
        ├── drone_mavros/          # mavros_core.py — PX4 Communication Layer
        ├── drone_qr/              # qr_core.py — QR Detection Module
        ├── drone_controller/      # controller_fixed.py — Waypoint Cruise Main Controller
        ├── drone_communication/   # communication_core.py — Air-Ground Data Bridge
        └── drone_navigation/      # Deprecated; replaced with fixed-point cruise logic
```

> Note: Repository may only contain compiled `build/` artifacts; full source code is stored on onboard RDK `~/drone_ws`\.

### 7\.2 Flight Control Communication Architecture

```Plain Text
drone_controller
      │  ROS 2 Service: /mavros_to_controller
      │  (drone_interfaces/DroneMavros)
      ▼
drone_mavros (mavros_core.py)
      │  MAVROS API
      │  /mavros/setpoint_position/local
      │  /mavros/state, /mavros/local_position/pose
      ▼
PX4 Flight Controller (MAVLink Protocol)
```

### 7\.3 `DroneMavros` Service Interface

**Service Name**: `/mavros_to_controller`
**Type**: `drone_interfaces/srv/DroneMavros`

**Request Fields**:

| Field         | Type   | Description                          |
| ------------- | ------ | ------------------------------------ |
| `command`     | int32  | Instruction code \(see table below\) |
| `input1`      | string | Extended string parameter            |
| `input_num_1` | float  | Numeric parameter 1                  |
| `input_num_2` | float  | Numeric parameter 2                  |
| `input_num_3` | float  | Numeric parameter 3                  |

**Response**: `success` \(bool\), `feedback` \(string\)

**Instruction Codes \(Used by drone_controller\)**:

| command | Function                | Parameters                            |
| ------- | ----------------------- | ------------------------------------- |
| `0`     | Takeoff                 | `input_num_1` = Target altitude \(m\) |
| `2`     | Land                    | None                                  |
| `5`     | Hover                   | None                                  |
| `6`     | Fly to Local Coordinate | `input_num_1/2/3` = x, y, z           |

### 7\.4 Key UAV ROS Topics

| Topic                 | Direction                              | Description                           |
| --------------------- | -------------------------------------- | ------------------------------------- |
| `/image_raw`          | Camera → QR Module                     | MIPI raw image stream                 |
| `/qr_pose`            | QR Module → Main Controller            | QR pose under camera frame            |
| `/drone/current_pose` | MAVROS → Main Controller               | Global UAV pose                       |
| `/qr_global_pose`     | Main Controller → Communication Module | QR target world coordinate            |
| `/qr_target_for_car`  | Main Controller → Communication Module | Target coordinate forwarded to ground |
| `/car/qr_goal`        | Communication Module → Ground UGV      | Navigation target for UGV             |

### 7\.5 Onboard Startup Flow \(Reference\)

```bash
# 1. Start Camera
source /opt/tros/humble/setup.bash
ros2 launch mipi_cam mipi_cam.launch.py ...
# 2. MAVROS + PX4 (Configure fcu_url based on flight controller connection)
# ros2 launch mavros px4.launch ...
# 3. Build & Launch UAV Software Stack
cd ~/drone_ws
colcon build
source install/setup.bash
ros2 run drone_mavros mavros_core          # Or corresponding launch file
ros2 run drone_qr qr_core
ros2 run drone_controller controller_fixed
ros2 run drone_communication communication_core
```

### 7\.6 Waypoint Cruise Configuration

Predefined waypoints in `controller_fixed.py` \(local coordinate frame, unit: meter\):

```python
waypoints = [
    (2.0, 0.0, 2.0),
    (2.0, 2.0, 2.0),
    (0.0, 2.0, 2.0),
    (0.0, 0.0, 2.0),
]
target_altitude = 2.0
position_threshold = 0.3   # Waypoint arrival tolerance (m)
qr_min_consecutive = 3     # Minimum consecutive frames for valid QR detection
```

---

## 8\. Air\-Ground Communication

### 8\.1 Current Implementation

`drone_communication/communication_core.py`:

```python
Subscriber: /qr_target_for_car  (PoseStamped)
Publisher: /car/qr_goal         (PoseStamped)
```

Forwards global QR coordinates detected by UAV to ground UGV\.

### 8\.2 Groundside Integration \(Planned / Pending Optimization\)

Proposed modification for ground `RDK_Navigation`:

1. Add new ROS node to subscribe to `/car/qr_goal` \(or equivalent MQTT/UDP bridge\)

2. Write target `(x, y)` to temporary entry in `mission_waypoints.yaml`, or call `MissionRunner.go_to_point()`

3. After UGV reaches target, trigger existing PHASE 2 grasping workflow

4. Publish `mission_complete` topic to notify UAV upon full task completion

### 8\.3 Mapping with Simulation Environment

`air_ground_sim/` uses in\-memory message bus to simulate the above communication pipeline:

| Simulation Module   | Real Hardware Equivalent                |
| ------------------- | --------------------------------------- |
| `TargetReport`      | `/qr_pose` \+ coordinate transformation |
| `DRONE_SEND_TARGET` | `/car/qr_goal`                          |
| `MissionComplete`   | Ground task completion callback         |

```bash
cd air_ground_sim
python run_sim.py          # Pygame 2D visualization simulation
python run_sim.py --3d     # PyBullet 3D physical simulation (optional)
```

---

## 9\. Full Repository Directory Overview

```Plain Text
InnovationProject/
├── RDK_Navigation/           # ★ Ground UGV + Robotic Arm Main Project (This Document)
│   ├── README.md
│   ├── config/
│   │   ├── device_ports.yaml
│   │   └── mission_waypoints_v5.yaml
│   ├── scripts/
│   │   ├── run_navigation.sh       # T1 Navigation Stack Launcher
│   │   ├── run_full_mission.sh     # T2 Full Mission Launcher
│   │   ├── run_full_mission.py
│   │   ├── run_mission.py
│   │   ├── grab_mission.py
│   │   └── load_ports.py
│   ├── ros2_ws/
│   └── docs/
│       ├── 00_Overall_Solution_Design.md
│       └── 01_Lower_Machine_Serial_Protocol.md
│
├── drone/                      # ★ Airborne PX4+MAVROS+QR Vision Stack
│   ├── README.md
│   └── drone_ws/ (src + build directories)
│
├── air_ground_sim/              # Air-Ground Cooperative Simulation Framework
└── rdk_qr_test/                 # Standalone QR Camera Test Tool
```

---

## 10\. Deployment \& File Synchronization

### 10\.1 Ground RDK File Sync \(PC → UGV Board\)

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

### 10\.2 Onboard UAV RDK

Maintain `~/drone_ws` on UAV RDK, compile and run following Section §7\.5\. Ensure correct `fcu_url` for PX4 \& MAVROS \(USB/UDP depends on flight controller physical connection\)\.

### 10\.3 Software Dependencies

| Platform     | Core Dependencies                                           |
| ------------ | ----------------------------------------------------------- |
| Ground UGV   | ROS 2 Humble, Nav2, `wheeltec_arm`, `wheeltec_color_sort`   |
| Airborne UAV | TROS/Humble, MAVROS, PX4, pyzbar, OpenCV, `mipi_cam` driver |

---

## 11\. Parameter Tuning \& Troubleshooting

### 11\.1 UGV Navigation Issues

| Phenomenon                           | Solution                                           |
| ------------------------------------ | -------------------------------------------------- |
| Jittery / Slow Turning               | Reduce `max_vel_theta` in `nav2_params.yaml`       |
| Loitering around target waypoint     | Adjust `progress_checker.required_movement_radius` |
| Nav2 parameter changes not effective | Restart T1 `run_navigation.sh`                     |

### 11\.2 Robotic Arm \& Camera Issues

| Phenomenon                                        | Solution                                                                                                        |
| ------------------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| Blank camera feed                                 | Verify `arm_camera` port; close VNC camera preview window; pass `video_device` to pick_color launch             |
| Gripper reopens after grasping                    | Shorten `grab_pick_kill_max_s`; confirm vision stack kill after closed\+home joint state                        |
| Vision stack terminated before arm fully retracts | Do not rely on fixed timing for IK; trigger kill only when `/joint_states` confirms closed gripper \+ home pose |

### 11\.3 UAV Issues

| Phenomenon                     | Solution                                                                                      |
| ------------------------------ | --------------------------------------------------------------------------------------------- |
| No output on `/qr_pose`        | Check MIPI camera, lighting condition, QR side length calibration parameter `qr_side_length`  |
| Large positioning offset       | Re\-calibrate camera intrinsic parameters; verify physical QR size matches code configuration |
| Flight controller unresponsive | Check MAVROS connection status, confirm `/mavros/state` shows `connected`                     |
| False QR detection             | Increase `qr_min_consecutive` frame threshold                                                 |

### 11\.4 Serial Port Occupancy Clearance

```bash
sudo fuser -k /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyACM1
pkill -f usb_cam; pkill -f pick_color
```

---

## 12\. Version Log

| Module                    | Version \& Remarks                                                  |
| ------------------------- | ------------------------------------------------------------------- |
| Navigation Map            | `arena_map_v5`                                                      |
| Mission Waypoints         | `mission_waypoints_v5`                                              |
| Ground ROS Distribution   | Humble                                                              |
| Airborne ROS Distribution | TROS Humble \(Onboard\)                                             |
| Simulation Environment    | `air_ground_sim` v1 \(Fixed Cruise \+ QR Detection \+ Grasp Logic\) |
| Official Robotic Arm SDK  | `wheeltec_arm` \+ `wheeltec_color_sort` \(Source Code Unmodified\)  |

---

## 13\. Related Documents

| Document                            | Path                                       |
| ----------------------------------- | ------------------------------------------ |
| UGV Mapping \& Navigation Solution  | `docs/00_Overall_Solution_Design.md`       |
| STM32 Serial Communication Protocol | `docs/01_Lower_Machine_Serial_Protocol.md` |
| UAV Readme                          | `../drone/README.md`                       |
| Air\-Ground Simulation Script       | `../air_ground_sim/run_sim.py`             |

---

_Northwestern Polytechnical University · Air\-Ground Cooperative Intelligent Grasping System · Software Technical Document_
