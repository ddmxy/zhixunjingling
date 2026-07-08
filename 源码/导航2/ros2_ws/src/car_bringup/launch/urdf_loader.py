"""Load car.urdf with optional sensor_calibration.yaml substitutions."""
from __future__ import annotations

import os

try:
    import yaml
except ImportError:
    yaml = None


def load_robot_description(urdf_path: str, calibration_path: str | None = None) -> str:
    with open(urdf_path, encoding="utf-8") as handle:
        urdf = handle.read()

    laser_yaw = 0.0
    if calibration_path and os.path.isfile(calibration_path) and yaml is not None:
        with open(calibration_path, encoding="utf-8") as handle:
            cal = yaml.safe_load(handle) or {}
        laser_yaw = float(cal.get("laser_mount_yaw_rad", 0.0))

    return urdf.replace("LASER_MOUNT_YAW", str(laser_yaw))
