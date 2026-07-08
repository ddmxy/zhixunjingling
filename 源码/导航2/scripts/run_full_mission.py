#!/usr/bin/env python3
"""Full mission: multi-waypoint Nav2 -> vision grab (once) -> escape -> home.

Prerequisites (RDK):
  T1: bash ~/Desktop/run_navigation.sh
  T2: bash ~/Desktop/run_full_mission.sh

Ports: edit ~/device_ports.yaml  OR  --chassis / --lidar / --arm on launch scripts.
Do NOT run pick_color.launch manually — this script starts/stops it.
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time

import rclpy

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from load_ports import find_ports_file, load_ports
from run_mission import DEFAULT_WAYPOINTS, MissionRunner, load_mission

GRAB_SCRIPT = os.path.join(SCRIPT_DIR, "grab_mission.py")


def _bash_lc(cmd: str, **popen_kw) -> subprocess.Popen:
    return subprocess.Popen(
        ["bash", "-lc", cmd],
        preexec_fn=os.setsid,
        **popen_kw,
    )


def kill_pick_color_stack() -> None:
    """Hard-stop official vision/grab nodes so claw cannot open a second time."""
    subprocess.run(
        [
            "bash", "-lc",
            "pkill -9 -f 'color_pick' 2>/dev/null; "
            "pkill -9 -f 'find_color' 2>/dev/null; "
            "pkill -9 -f 'pick_color.launch' 2>/dev/null; "
            "pkill -9 -f 'wheeltec_color_sort' 2>/dev/null; "
            "true",
        ],
        check=False,
    )


def stop_process_group(proc: subprocess.Popen | None, name: str, timeout: float = 12.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=timeout)
        print(f"[full_mission] stopped {name}")
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    except Exception as exc:
        print(f"[full_mission] warn stop {name}: {exc}")


def start_pick_color(port: str, color: str) -> subprocess.Popen:
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source ~/wheeltec_arm/install/setup.bash && "
        "export DISPLAY=:0 && "
        f"ros2 launch wheeltec_color_sort pick_color.launch.py "
        f"target_color:={color} port:={port}"
    )
    print(f"[full_mission] starting pick_color.launch port={port}")
    return _bash_lc(cmd)


def run_grab_phase(
    pick_kill_max_s: float,
    pick_home_hold_s: float,
    pick_wait_s: float,
    settle_s: float,
    grab_timeout_s: float,
    target_color: str,
) -> int:
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source ~/Desktop/ros2_ws/install/setup.bash && "
        "source ~/wheeltec_arm/install/setup.bash && "
        "export DISPLAY=:0 && "
        f"python3 {GRAB_SCRIPT} --exit-on-done --no-window "
        f"--target-color {target_color} "
        f"--pick-kill-max-s {pick_kill_max_s:.0f} "
        f"--pick-home-hold-s {pick_home_hold_s:.1f} "
        f"--pick-wait-s {pick_wait_s:.0f} --settle-s {settle_s:.0f}"
    )
    print("[full_mission] starting grab_mission (chassis align + handoff)")
    proc = _bash_lc(cmd)
    try:
        return proc.wait(timeout=grab_timeout_s)
    except subprocess.TimeoutExpired:
        print("[full_mission] grab_mission timeout — terminating")
        stop_process_group(proc, "grab_mission")
        return -1


def run_nav_legs(node: MissionRunner, points: dict, route: list, mission: dict) -> None:
    final_name = str(mission["final_point"])
    for i, name in enumerate(route, 1):
        mode = "final" if name == final_name else "via"
        node.get_logger().info(f"=== leg {i}/{len(route)}: {name} ({mode}) ===")
        node.go_to_point(name, points[name], mission, mode)
    node.dwell(float(mission.get("final_dwell_s", 5.0)), final_name)


def main() -> None:
    parser = argparse.ArgumentParser(description="Nav + one grab + escape + home")
    parser.add_argument("--waypoints", default=DEFAULT_WAYPOINTS)
    parser.add_argument("--no-set-pose", action="store_true")
    parser.add_argument("--no-grab", action="store_true", help="nav only, skip vision")
    parser.add_argument("--no-return", action="store_true")
    parser.add_argument("--no-escape", action="store_true")
    parser.add_argument("--escape-m", type=float, default=None)
    parser.add_argument("--ports-file", "-f", default=None, help="device_ports.yaml path")
    parser.add_argument("--chassis-port", default=None, help="logged only; set in run_navigation.sh")
    parser.add_argument("--lidar-port", default=None, help="logged only; set in run_navigation.sh")
    parser.add_argument("--arm-port", default=None, help="arm MCU for pick_color")
    parser.add_argument("--grab-timeout", type=float, default=None)
    args = parser.parse_args()

    points, route, mission = load_mission(args.waypoints)
    do_return = bool(mission.get("return_home", True)) and not args.no_return
    do_grab = bool(mission.get("grab_enabled", True)) and not args.no_grab

    ports_file = args.ports_file or mission.get("device_ports_file")
    ports = load_ports(
        ports_file,
        chassis=args.chassis_port,
        lidar=args.lidar_port,
        arm_mcu=args.arm_port or mission.get("grab_arm_port"),
    )
    arm_port = ports["arm_mcu"]
    pf = find_ports_file(ports_file)
    if pf is None:
        print(
            "[full_mission] WARNING: 未找到 device_ports.yaml，使用内置默认口",
            file=sys.stderr,
        )
    print(
        f"[full_mission] ports: chassis={ports['chassis']} lidar={ports['lidar']} "
        f"arm={arm_port} file={pf or '内置默认'}"
    )
    target_color = str(mission.get("grab_target_color", "green"))
    pick_kill_max_s = float(
        mission.get("grab_pick_kill_max_s", mission.get("grab_pick_once_s", 14.0))
    )
    pick_home_hold_s = float(mission.get("grab_pick_home_hold_s", 0.6))
    pick_wait_s = float(mission.get("grab_pick_wait_s", 28.0))
    grab_settle_s = float(mission.get("grab_settle_s", 6.0))
    grab_timeout_s = float(args.grab_timeout or mission.get("grab_timeout_s", 150.0))

    if args.escape_m is not None:
        mission = {**mission, "escape_forward_m": args.escape_m}

    rclpy.init()
    nav = MissionRunner()
    pick_proc: subprocess.Popen | None = None

    try:
        nav.get_logger().info("=== PHASE 1: multi-waypoint navigation ===")
        nav.wait_base()
        if not args.no_set_pose:
            nav.set_initial_pose(points["home"])
        nav.wait_nav2()
        nav.wait_nav_action()
        run_nav_legs(nav, points, route, mission)

        if do_grab:
            nav.get_logger().info("=== PHASE 2: vision + grab (once) ===")
            nav.stop_robot()
            time.sleep(1.0)
            pick_proc = start_pick_color(arm_port, target_color)
            time.sleep(2.0)
            rc = run_grab_phase(
                pick_kill_max_s, pick_home_hold_s,
                pick_wait_s, grab_settle_s, grab_timeout_s, target_color
            )
            stop_process_group(pick_proc, "pick_color")
            pick_proc = None
            kill_pick_color_stack()
            time.sleep(0.2)
            kill_pick_color_stack()
            if rc != 0:
                nav.get_logger().warn(f"grab phase exit code {rc} — continue escape/home")
            nav.get_logger().info(
                "grab phase done — pick_color killed, no second grab"
            )
            time.sleep(1.0)

        if do_return:
            nav.get_logger().info("=== PHASE 3: escape + return home ===")
            if not args.no_escape:
                nav.escape_before_home(mission)
            nav.go_to_point("home", points["home"], mission, "coarse_only")

        nav.get_logger().info("=== FULL MISSION DONE ===")
    except Exception as exc:
        nav.get_logger().error(str(exc))
        sys.exit(1)
    finally:
        stop_process_group(pick_proc, "pick_color")
        kill_pick_color_stack()
        nav.stop_robot()
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
