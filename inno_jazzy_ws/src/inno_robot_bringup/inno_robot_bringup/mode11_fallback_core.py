"""Mode 11 planar dead reckoning: wheel STEP deltas for distance, gyro z for yaw."""

from dataclasses import dataclass
from enum import Enum
import math


class Mode11State(Enum):
    NORMAL = 'NORMAL'
    BLACKOUT = 'BLACKOUT'


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def normalize_yaw(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class EncoderImuFallback:
    def __init__(self, *, wheel_radius=0.04, full_steps=200,
                 microsteps=8, gear_ratio=1.0, left_sign=-1,
                 right_sign=1, maximum_steps_per_sec=2400.0):
        if min(wheel_radius, full_steps, microsteps, gear_ratio,
               maximum_steps_per_sec) <= 0:
            raise ValueError('wheel geometry and step rate must be positive')
        if left_sign not in (-1, 1) or right_sign not in (-1, 1):
            raise ValueError('wheel signs must be -1 or 1')
        self.metres_per_step = (
            2.0 * math.pi * wheel_radius / (full_steps * microsteps * gear_ratio)
        )
        self.left_sign = left_sign
        self.right_sign = right_sign
        self.maximum_steps_per_sec = maximum_steps_per_sec
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.last_ticks = None
        self.last_tick_time = None
        self.last_imu_stamp = None
        self.last_distance_yaw = 0.0
        self.active = False
        self.linear_speed = 0.0
        self.angular_speed = 0.0

    def start(self, rf2o_pose, ticks, tick_time, imu_stamp):
        try:
            values = (rf2o_pose.x, rf2o_pose.y, rf2o_pose.yaw,
                      tick_time, imu_stamp, ticks[0], ticks[1])
            valid = all(math.isfinite(value) for value in values)
        except (AttributeError, IndexError, TypeError, ValueError):
            valid = False
        if not valid:
            raise ValueError('non-finite transition state')
        self.pose = Pose2D(rf2o_pose.x, rf2o_pose.y, normalize_yaw(rf2o_pose.yaw))
        self.last_ticks = (int(ticks[0]), int(ticks[1]))
        self.last_tick_time = tick_time
        self.last_imu_stamp = imu_stamp
        self.last_distance_yaw = self.pose.yaw
        self.active = True
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        return self.pose

    def on_imu(self, angular_z, stamp, encoder_fresh=True):
        if not self.active or not all(math.isfinite(v) for v in (angular_z, stamp)):
            return False
        dt = stamp - self.last_imu_stamp
        self.last_imu_stamp = stamp
        if not encoder_fresh or not 0.0 < dt <= 0.25:
            self.angular_speed = 0.0
            return False
        self.pose = Pose2D(self.pose.x, self.pose.y,
                           normalize_yaw(self.pose.yaw + angular_z * dt))
        self.angular_speed = angular_z
        return True

    def on_ticks(self, ticks, receipt_time, imu_fresh):
        if not self.active or not math.isfinite(receipt_time):
            return False
        try:
            if not all(math.isfinite(value) for value in ticks[:2]):
                return False
            pair = (int(ticks[0]), int(ticks[1]))
        except (TypeError, ValueError, IndexError):
            return False
        dt = receipt_time - self.last_tick_time
        left_delta = (pair[0] - self.last_ticks[0]) * self.left_sign
        right_delta = (pair[1] - self.last_ticks[1]) * self.right_sign
        self.last_ticks = pair
        self.last_tick_time = receipt_time
        if (not imu_fresh or not 0.0 < dt <= 0.6
                or max(abs(left_delta), abs(right_delta)) >
                self.maximum_steps_per_sec * dt + 2):
            self.last_distance_yaw = self.pose.yaw
            self.linear_speed = 0.0
            return False
        distance = 0.5 * (left_delta + right_delta) * self.metres_per_step
        self.linear_speed = distance / dt
        heading = normalize_yaw(
            self.last_distance_yaw
            + 0.5 * normalize_yaw(self.pose.yaw - self.last_distance_yaw)
        )
        self.pose = Pose2D(
            self.pose.x + distance * math.cos(heading),
            self.pose.y + distance * math.sin(heading),
            self.pose.yaw,
        )
        self.last_distance_yaw = self.pose.yaw
        return True
