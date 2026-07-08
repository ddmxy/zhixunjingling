# Sync navigation code from Windows PC to RDK X5.
# Usage (PowerShell):  cd RDK_Navigation\scripts ; .\sync_to_rdk.ps1
# Requires: PC and RDK on same network; SSH password for sunrise@<IP>

$RDK_IP = "192.168.43.205"
$RDK_USER = "sunrise"
$NAV = Split-Path $PSScriptRoot -Parent
$REMOTE = "${RDK_USER}@${RDK_IP}"

Write-Host "=== Sync to $REMOTE ===" -ForegroundColor Cyan
Write-Host "Project: $NAV"

$files = @(
    "ros2_ws\src\car_navigation\config\nav2_params.yaml",
    "ros2_ws\src\car_bringup\launch\mapping.launch.py",
    "ros2_ws\src\car_bringup\launch\navigation.launch.py",
    "ros2_ws\src\car_bringup\config\device_ports.yaml",
    "ros2_ws\src\lslidar_driver\launch\lsn10_launch.py",
    "ros2_ws\src\lslidar_driver\src\lslidar_driver.cc",
    "ros2_ws\src\lslidar_driver\params\lidar_uart_ros2\lsn10.yaml",
    "ros2_ws\src\base_driver\config\base_driver.yaml",
    "ros2_ws\src\base_driver\launch\base_driver.launch.py"
)

foreach ($rel in $files) {
    $local = Join-Path $NAV $rel
    $remotePath = "~/Desktop/ros2_ws/src/" + ($rel -replace '\\','/' -replace '^ros2_ws/src/','')
    if (-not (Test-Path $local)) {
        Write-Warning "SKIP (missing): $local"
        continue
    }
    Write-Host "  -> $rel"
    scp $local "${REMOTE}:${remotePath}"
}

Write-Host "`n=== Scripts ===" -ForegroundColor Cyan
$portScripts = @(
    "load_ports.py",
    "load_ports.sh",
    "check_usb_ports.sh",
    "run_navigation.sh",
    "run_mapping.sh",
    "run_full_mission.sh"
)
foreach ($name in $portScripts) {
    scp (Join-Path $NAV "scripts\$name") "${REMOTE}:~/Desktop/"
}
scp (Join-Path $NAV "config\device_ports.yaml") "${REMOTE}:~/device_ports.yaml"
scp -r (Join-Path $NAV "scripts\udev") "${REMOTE}:~/Desktop/"

Write-Host "`n=== Mission scripts ===" -ForegroundColor Cyan
$missionScripts = @(
    "save_mission_points.py",
    "arm_pose_cmd.py",
    "grab_mission.py",
    "run_full_mission.py",
    "run_full_mission.sh",
    "run_full_mission.py",
    "run_mission.py",
    "run_mission_phase1.sh",
    "waypoint_recorder.py"
)
foreach ($name in $missionScripts) {
    $local = Join-Path $NAV "scripts\$name"
    if (Test-Path $local) {
        scp $local "${REMOTE}:~/Desktop/"
    }
}

Write-Host "`nDone. On RDK run:" -ForegroundColor Green
Write-Host @"
  ssh $REMOTE
  cd ~/Desktop/ros2_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-select lslidar_driver base_driver car_bringup car_navigation
  source install/setup.bash
  bash ~/Desktop/check_usb_ports.sh
  python3 ~/Desktop/load_ports.py --show
  # edit ~/device_ports.yaml when USB order changes, then:
  bash ~/Desktop/run_navigation.sh
"@
