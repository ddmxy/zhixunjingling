#!/usr/bin/env python3
"""Load USB/serial ports from ~/device_ports.yaml with CLI/env overrides."""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path

DEFAULT_PORTS = {
    "chassis": "/dev/ttyUSB0",
    "lidar": "/dev/ttyACM0",
    "arm_mcu": "/dev/ttyACM1",
    "arm_camera": "/dev/video0",
}

ALIASES = {
    "chassis": "chassis",
    "base": "chassis",
    "base_driver": "chassis",
    "lidar": "lidar",
    "arm": "arm_mcu",
    "arm_mcu": "arm_mcu",
    "arm_port": "arm_mcu",
    "grab_arm_port": "arm_mcu",
    "camera": "arm_camera",
    "arm_camera": "arm_camera",
}


def find_ports_file(explicit: str | None) -> Path | None:
    if explicit:
        p = Path(explicit).expanduser()
        return p if p.is_file() else None
    for candidate in (
        Path.home() / "device_ports.yaml",
        Path.home() / "Desktop" / "device_ports.yaml",
        Path(__file__).resolve().parent.parent / "config" / "device_ports.yaml",
    ):
        if candidate.is_file():
            return candidate
    return None


def parse_yaml_ports(path: Path) -> dict[str, str]:
    ports: dict[str, str] = {}
    in_block = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if re.match(r"^\s*device_ports\s*:", line):
            in_block = True
            continue
        if in_block:
            if re.match(r"^\S", line) and not line.lstrip().startswith("#"):
                break
            m = re.match(
                r"^\s+(chassis|lidar|arm_mcu|arm_camera|arm_camera_serial)\s*:\s*(\S+)",
                line,
            )
            if m:
                key = "arm_camera" if m.group(1) == "arm_camera_serial" else m.group(1)
                ports[key] = m.group(2).strip("\"'")
    return ports


def load_ports(ports_file: str | None = None, **overrides: str | None) -> dict[str, str]:
    result = dict(DEFAULT_PORTS)
    pf = find_ports_file(ports_file)
    if pf:
        result.update(parse_yaml_ports(pf))
    for raw_key, value in overrides.items():
        if value is None:
            continue
        key = ALIASES.get(raw_key, raw_key)
        if key in result:
            result[key] = value
    for env, key in (
        ("CHASSIS_PORT", "chassis"),
        ("LIDAR_PORT", "lidar"),
        ("ARM_PORT", "arm_mcu"),
        ("ARM_MCU_PORT", "arm_mcu"),
        ("ARM_CAMERA", "arm_camera"),
    ):
        if os.environ.get(env):
            result[key] = os.environ[env]
    return result


def bash_export(ports: dict[str, str], ports_file: Path | None) -> str:
    src = str(ports_file) if ports_file else ""
    lines = [
        f'export PORTS_FILE="{src}"',
        f'export CHASSIS_PORT="{ports["chassis"]}"',
        f'export LIDAR_PORT="{ports["lidar"]}"',
        f'export ARM_PORT="{ports["arm_mcu"]}"',
        f'export ARM_CAMERA="{ports["arm_camera"]}"',
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Load device serial/video ports")
    parser.add_argument("--ports-file", "-f", default=None, help="YAML with device_ports:")
    parser.add_argument("--chassis", default=None)
    parser.add_argument("--lidar", default=None)
    parser.add_argument("--arm", dest="arm_mcu", default=None)
    parser.add_argument("--camera", dest="arm_camera", default=None)
    parser.add_argument("--bash", action="store_true", help="print bash export statements")
    parser.add_argument("--show", action="store_true", help="human-readable summary")
    args = parser.parse_args()

    pf = find_ports_file(args.ports_file)
    ports = load_ports(
        args.ports_file,
        chassis=args.chassis,
        lidar=args.lidar,
        arm_mcu=args.arm_mcu,
        arm_camera=args.arm_camera,
    )

    if args.bash:
        if pf is None:
            print(
                'echo "WARNING: 未找到 ~/device_ports.yaml，使用 load_ports.py 内置默认值"',
                file=sys.stderr,
            )
        print(bash_export(ports, pf))
    elif args.show:
        print(f"ports_file: {pf or '(built-in defaults)'}")
        for key, value in ports.items():
            print(f"  {key}: {value}")
    else:
        print(json.dumps(ports))


if __name__ == "__main__":
    main()
