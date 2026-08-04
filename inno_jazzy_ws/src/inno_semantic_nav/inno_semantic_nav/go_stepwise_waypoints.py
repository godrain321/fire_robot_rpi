#!/usr/bin/env python3
"""Step-by-step waypoint driving using map-frame semantic points and ESP32 encoder feedback.

Example:
  ros2 run inno_semantic_nav go_stepwise_waypoints --names p1,p2,p3,p4,p5,p6,p7,p8,p9 \
    --semantic-file /home/gosunwoo/fire_robot_rpi/maps/semantic_points.yaml \
    --serial /dev/ttyUSB0

Behavior:
- Load points from semantic_points.yaml (map frame coordinates).
- Move from p1 -> p2 -> p3 -> ... -> p9 sequentially.
- After each segment, pause and wait for Enter so the operator can verify.
- During motion, read ENC_ABS messages from the ESP32 and stop when the target distance is reached.
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import List, Tuple

import serial
import yaml


WHEEL_DIAMETER_M = 0.08
STEPS_PER_REV = 1600.0
TRACK_WIDTH_M = 0.18
MAX_LINEAR_MPS = 0.25
MAX_ROTATION_RAD_S = 0.6
BAUDRATE = 115200


def load_semantic(path: str) -> dict:
    p = Path(path).expanduser()
    if not p.exists():
        raise FileNotFoundError(f"semantic file not found: {p}")
    doc = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    return doc.get("poses", {})


def resolve_point(name_or_coord: str, poses: dict) -> Tuple[float, float, float]:
    if name_or_coord in poses:
        entry = poses[name_or_coord]
        return float(entry["x"]), float(entry["y"]), float(entry.get("yaw", 0.0))
    parts = [p.strip() for p in name_or_coord.split(",")]
    if len(parts) >= 2:
        x = float(parts[0])
        y = float(parts[1])
        yaw = float(parts[2]) if len(parts) > 2 else 0.0
        return x, y, yaw
    raise ValueError(f"Unknown point: {name_or_coord}")


def pose_to_wheel_sps(linear_mps: float, angular_rps: float, wheel_diameter: float, track_width: float, steps_per_rev: float) -> Tuple[float, float]:
    v_l = linear_mps - angular_rps * track_width / 2.0
    v_r = linear_mps + angular_rps * track_width / 2.0
    revs_l = v_l / (math.pi * wheel_diameter)
    revs_r = v_r / (math.pi * wheel_diameter)
    return revs_l * steps_per_rev, revs_r * steps_per_rev


class Esp32Driver:
    def __init__(self, port: str, baudrate: int = BAUDRATE) -> None:
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(0.2)
        self.seq = 1

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    def send_cmd(self, line: str) -> None:
        self.ser.write((line + "\n").encode("ascii"))
        self.ser.flush()

    def send_motor(self, left_sps: float, right_sps: float) -> None:
        line = f"M,{self.seq},{left_sps:.1f},{right_sps:.1f}"
        self.send_cmd(line)
        self.seq += 1

    def stop(self) -> None:
        self.send_cmd(f"STOP,{self.seq}")
        self.seq += 1

    def read_line(self) -> str | None:
        line = self.ser.readline()
        if not line:
            return None
        return line.decode("ascii", errors="ignore").strip()


def parse_abs_encoder(line: str) -> Tuple[float, float] | None:
    if not line.startswith("ENC_ABS,"):
        return None
    parts = line.split(",")
    if len(parts) < 8:
        return None
    try:
        left_m = float(parts[6])
        right_m = float(parts[7])
    except ValueError:
        return None
    return left_m, right_m


def rotate_in_place(driver: Esp32Driver, angle_rad: float, speed_rad_s: float = 0.45, timeout_s: float = 5.0) -> None:
    sign = 1.0 if angle_rad >= 0 else -1.0
    angular_speed = min(abs(speed_rad_s), MAX_ROTATION_RAD_S) * sign
    left_sps, right_sps = pose_to_wheel_sps(0.0, angular_speed, WHEEL_DIAMETER_M, TRACK_WIDTH_M, STEPS_PER_REV)
    driver.send_motor(left_sps, right_sps)
    start = time.time()
    while time.time() - start < timeout_s:
        line = driver.read_line()
        if line is not None and line.startswith("ERR,"):
            break
        if abs(angle_rad) <= 0.02:
            break
        time.sleep(0.02)
    driver.stop()
    time.sleep(0.1)


def drive_distance(driver: Esp32Driver, distance_m: float, speed_mps: float = 0.15, timeout_s: float = 10.0) -> float:
    if distance_m <= 0.001:
        return 0.0
    left_sps, right_sps = pose_to_wheel_sps(speed_mps, 0.0, WHEEL_DIAMETER_M, TRACK_WIDTH_M, STEPS_PER_REV)
    driver.send_motor(left_sps, right_sps)
    baseline_left = None
    baseline_right = None
    start = time.time()
    travelled = 0.0
    while time.time() - start < timeout_s:
        line = driver.read_line()
        if line is None:
            time.sleep(0.02)
            continue
        if line.startswith("ERR,"):
            break
        parsed = parse_abs_encoder(line)
        if parsed is None:
            continue
        left_m, right_m = parsed
        if baseline_left is None or baseline_right is None:
            baseline_left, baseline_right = left_m, right_m
            continue
        delta_left = abs(left_m - baseline_left)
        delta_right = abs(right_m - baseline_right)
        travelled = 0.5 * (delta_left + delta_right)
        print(f"encoder: left={left_m:.3f} m right={right_m:.3f} m travelled={travelled:.3f} m")
        if travelled >= distance_m:
            break
        time.sleep(0.02)
    driver.stop()
    time.sleep(0.1)
    return travelled


def run_waypoints(names: List[str], semantic_file: str, serial_port: str, speed_mps: float, step_mode: bool) -> None:
    poses = load_semantic(semantic_file)
    points = [resolve_point(name, poses) for name in names]
    if len(points) < 2:
        raise ValueError("Need at least two waypoints")

    print(f"Loaded {len(points)} waypoints from {semantic_file}")
    driver = Esp32Driver(serial_port)
    try:
        prev_x, prev_y, prev_yaw = points[0]
        for idx in range(1, len(points)):
            target_x, target_y, target_yaw = points[idx]
            dx = target_x - prev_x
            dy = target_y - prev_y
            target_dist = math.hypot(dx, dy)
            target_heading = math.atan2(dy, dx)
            print(f"Segment {idx}: {names[idx-1]} -> {names[idx]}  dist={target_dist:.3f} m")
            travelled = drive_distance(driver, target_dist, speed_mps=speed_mps)
            print(f"completed segment: travelled={travelled:.3f} m")
            # After driving to the target point, rotate in place to match the target heading.
            angle_diff = math.atan2(math.sin(target_heading - prev_yaw), math.cos(target_heading - prev_yaw))
            if abs(angle_diff) > 0.01:
                print(f"rotate to heading {target_heading:.3f} rad (diff={angle_diff:.3f})")
                rotate_in_place(driver, angle_diff)
            prev_x, prev_y, prev_yaw = target_x, target_y, target_heading
            if step_mode and idx < len(points) - 1:
                input("Press Enter to continue to the next waypoint...")
        print("All waypoints completed.")
    finally:
        driver.stop()
        driver.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Sequential waypoint driving with encoder feedback")
    parser.add_argument("--names", default="p1,p2,p3,p4,p5,p6,p7,p8,p9", help="Comma-separated waypoint names, e.g. p1,p2,p3")
    parser.add_argument("--semantic-file", default="/home/gosunwoo/fire_robot_rpi/maps/semantic_points.yaml")
    parser.add_argument("--serial", default="/dev/ttyUSB0")
    parser.add_argument("--speed", type=float, default=0.15)
    parser.add_argument("--step-mode", action="store_true", help="Pause after each segment until Enter is pressed")
    args = parser.parse_args()

    names = [n.strip() for n in args.names.split(",") if n.strip()]
    run_waypoints(names, args.semantic_file, args.serial, args.speed, args.step_mode)


if __name__ == "__main__":
    main()
