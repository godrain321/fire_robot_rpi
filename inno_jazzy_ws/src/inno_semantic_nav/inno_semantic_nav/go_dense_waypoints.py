#!/usr/bin/env python3
"""Generate dense waypoints between two semantic points and drive robot via serial.

Usage (example):
ros2 run inno_semantic_nav go_dense_waypoints p1 p3 --spacing 0.5 --serial /dev/ttyUSB0

Behavior:
- Load semantic file from this repository's inno_jazzy_ws/maps directory
- Resolve names p1 and p3 to coordinates (x,y,yaw if available)
- Generate intermediate points along straight line at `spacing` meters
- For each segment: rotate in place to face next point, then drive straight distance
- Commands are sent over serial using the ESP32 ASCII protocol (M,seq,left_sps,right_sps)

Notes:
- This node implements a simple open-loop motion primitive sequence. It assumes
  the ESP32 firmware will report `ENC_ABS` messages for closed-loop odometry fusion.
- Tunable parameters: wheel diameter, steps_per_rev, track_width, max_speed_mps

"""

import argparse
import math
import serial
import time
import yaml
from pathlib import Path

from .project_paths import project_path

# Default hardware params - keep editable
# User-supplied values:
# - wheel diameter: 80 mm -> 0.08 m
# - encoder: AS5048A, 14-bit -> 16384 counts/rev
# - track width: approximate wheel-center distance; adjust if measured value differs
WHEEL_DIAMETER_M = 0.08
STEPS_PER_REV = 1600.0  # 200 full steps/rev * 8 microsteps
TRACK_WIDTH_M = 0.30
MAX_SPEED_MPS = 0.4
LEFT_SIGN = -1
RIGHT_SIGN = 1

BAUDRATE = 115200
SEQ = 1


def load_semantic(path):
    p = Path(path).expanduser()
    if not p.exists():
        raise FileNotFoundError(f"semantic file not found: {p}")
    doc = yaml.safe_load(p.read_text())
    poses = doc.get('poses', {})
    landmarks = doc.get('landmarks', {})
    return poses, landmarks


def resolve_point(name_or_coord, poses):
    # name like 'E1' -> poses dict; or explicit "x,y[,yaw]"
    if name_or_coord in poses:
        entry = poses[name_or_coord]
        return float(entry['x']), float(entry['y']), float(entry.get('yaw', 0.0))
    parts = name_or_coord.split(',')
    if len(parts) >= 2:
        x = float(parts[0]); y = float(parts[1]); yaw = float(parts[2]) if len(parts) > 2 else 0.0
        return x, y, yaw
    raise ValueError(f"Unknown point: {name_or_coord}")


def interp_points(p1, p2, spacing):
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]
    dx = x2 - x1
    dy = y2 - y1
    dist = math.hypot(dx, dy)
    if dist <= 0.0:
        return []
    n = max(1, int(math.ceil(dist / spacing)))
    pts = []
    for i in range(1, n+1):
        t = i / n
        pts.append((x1 + dx * t, y1 + dy * t))
    return pts


def pose_to_wheel_sps(linear_mps, angular_rps, wheel_diameter=WHEEL_DIAMETER_M, track=TRACK_WIDTH_M, steps_per_rev=STEPS_PER_REV):
    # skid-steer differential velocities
    v_l = linear_mps - angular_rps * track / 2.0
    v_r = linear_mps + angular_rps * track / 2.0
    # convert to revolutions per second
    revs_l = v_l / (math.pi * wheel_diameter)
    revs_r = v_r / (math.pi * wheel_diameter)
    # steps per second
    sps_l = revs_l * steps_per_rev
    sps_r = revs_r * steps_per_rev
    return sps_l * LEFT_SIGN, sps_r * RIGHT_SIGN


def send_M(ser, left_sps, right_sps):
    global SEQ
    line = f"M,{SEQ},{left_sps:.1f},{right_sps:.1f}\n"
    ser.write(line.encode('ascii'))
    SEQ += 1


def send_stop(ser):
    global SEQ
    line = f"STOP,{SEQ}\n"
    ser.write(line.encode('ascii'))
    SEQ += 1


def stream_motor_for(ser, left_sps, right_sps, duration_s, period_s=0.1):
    """Refresh M commands faster than the ESP32's 0.5 s watchdog."""
    deadline = time.monotonic() + max(0.0, duration_s)
    while time.monotonic() < deadline:
        send_M(ser, left_sps, right_sps)
        remaining = deadline - time.monotonic()
        time.sleep(min(period_s, max(0.0, remaining)))


def rotate_in_place(ser, angle_rad, angular_speed_rad_s=0.5):
    # positive angle -> CCW; for skid-steer, left = -right
    # compute needed angular velocity sign
    sign = 1.0 if angle_rad >= 0 else -1.0
    angular_speed = angular_speed_rad_s * sign
    left_sps, right_sps = pose_to_wheel_sps(0.0, angular_speed)
    # estimate duration = |angle| / angular_speed
    dur = abs(angle_rad) / abs(angular_speed) if angular_speed != 0 else 0
    stream_motor_for(ser, left_sps, right_sps, dur)
    send_stop(ser)
    time.sleep(0.05)


def drive_straight(ser, distance_m, speed_mps=0.2):
    # drive forward distance at given speed
    left_sps, right_sps = pose_to_wheel_sps(speed_mps, 0.0)
    dur = abs(distance_m) / speed_mps if speed_mps > 0 else 0
    stream_motor_for(ser, left_sps, right_sps, dur)
    send_stop(ser)
    time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('start')
    parser.add_argument('goal')
    parser.add_argument(
        '--semantic-file',
        default=project_path('inno_jazzy_ws', 'maps', 'semantic_points.yaml'),
    )
    parser.add_argument('--spacing', type=float, default=0.5)
    parser.add_argument('--serial', default='/dev/ttyUSB0')
    parser.add_argument('--speed', type=float, default=0.2)
    args = parser.parse_args()

    poses, landmarks = load_semantic(args.semantic_file)
    start = resolve_point(args.start, poses)
    goal = resolve_point(args.goal, poses)

    points = interp_points(start, goal, args.spacing)
    if not points:
        print('No intermediate points (already at goal)')
        return

    # open serial
    ser = serial.Serial(args.serial, BAUDRATE, timeout=0.1)
    time.sleep(0.2)

    # initial orientation: rotate to face first point from start
    cur_x, cur_y, cur_yaw = start
    for px, py in points:
        desired_yaw = math.atan2(py - cur_y, px - cur_x)
        # smallest angle diff
        da = desired_yaw - cur_yaw
        da = math.atan2(math.sin(da), math.cos(da))
        if abs(da) > 1e-3:
            rotate_in_place(ser, da)
            cur_yaw = desired_yaw
        # distance to next point
        dist = math.hypot(px - cur_x, py - cur_y)
        if dist > 1e-4:
            drive_straight(ser, dist, speed_mps=args.speed)
            cur_x, cur_y = px, py
    print('Reached goal (open-loop).')


if __name__ == '__main__':
    main()
