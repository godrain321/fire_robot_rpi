"""Pure geometry and state management for the thermal cost layer."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

import numpy as np


def thermal_stream_is_stale(
    last_received_ns: int, now_ns: int, timeout_sec: float
) -> bool:
    """Return whether a ROS-clock sensor stream has exceeded its timeout."""
    if not math.isfinite(timeout_sec) or timeout_sec < 0.0:
        raise ValueError("thermal data timeout must be finite and non-negative")
    if last_received_ns < 0 or now_ns < 0:
        raise ValueError("ROS timestamps must be non-negative")
    timeout_ns = int(round(timeout_sec * 1_000_000_000))
    return now_ns < last_received_ns or now_ns - last_received_ns > timeout_ns


@dataclass(frozen=True)
class GridGeometry:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_z: float = 0.0
    origin_qx: float = 0.0
    origin_qy: float = 0.0
    origin_qz: float = 0.0
    origin_qw: float = 1.0
    frame_id: str = ""

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid width and height must be positive")
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("grid resolution must be finite and positive")
        values = (
            self.origin_x, self.origin_y, self.origin_z, self.origin_qx,
            self.origin_qy, self.origin_qz, self.origin_qw,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("grid origin must contain only finite values")
        quaternion_to_yaw(
            self.origin_qx, self.origin_qy, self.origin_qz, self.origin_qw
        )


def _normalized_quaternion(qx: float, qy: float, qz: float, qw: float):
    values = tuple(float(value) for value in (qx, qy, qz, qw))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain only finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError("quaternion norm must be non-zero")
    return tuple(value / norm for value in values)


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Return yaw from a normalized or non-normalized quaternion."""
    x, y, z, w = _normalized_quaternion(qx, qy, qz, qw)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def transform_point(
    point,
    translation,
    quaternion,
) -> tuple[float, float, float]:
    """Apply a full 3-D quaternion rotation followed by translation."""
    px, py, pz = (float(value) for value in point)
    tx, ty, tz = (float(value) for value in translation)
    if not all(math.isfinite(value) for value in (px, py, pz, tx, ty, tz)):
        raise ValueError("point and translation must contain only finite values")
    qx, qy, qz, qw = _normalized_quaternion(*quaternion)

    # Quaternion rotation matrix, then target-frame translation.
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    rx = (1.0 - 2.0 * (yy + zz)) * px + 2.0 * (xy - wz) * py + 2.0 * (xz + wy) * pz
    ry = 2.0 * (xy + wz) * px + (1.0 - 2.0 * (xx + zz)) * py + 2.0 * (yz - wx) * pz
    rz = 2.0 * (xz - wy) * px + 2.0 * (yz + wx) * py + (1.0 - 2.0 * (xx + yy)) * pz
    return rx + tx, ry + ty, rz + tz


def world_to_grid(
    world_x: float, world_y: float, geometry: GridGeometry
) -> tuple[int, int] | None:
    """Convert world coordinates through inverse origin yaw into a grid cell."""
    if not math.isfinite(world_x) or not math.isfinite(world_y):
        raise ValueError("world coordinates must be finite")
    dx = float(world_x) - geometry.origin_x
    dy = float(world_y) - geometry.origin_y
    yaw = quaternion_to_yaw(
        geometry.origin_qx,
        geometry.origin_qy,
        geometry.origin_qz,
        geometry.origin_qw,
    )
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    grid_x = math.floor(local_x / geometry.resolution)
    grid_y = math.floor(local_y / geometry.resolution)
    if not (0 <= grid_x < geometry.width and 0 <= grid_y < geometry.height):
        return None
    return int(grid_x), int(grid_y)


def temperature_to_cost(
    temperature_c: float,
    safe_temperature_c: float,
    blocked_temperature_c: float,
    temperature_power: float,
) -> int:
    """Map a finite Celsius value to OccupancyGrid thermal cost 0..100."""
    values = (
        temperature_c, safe_temperature_c, blocked_temperature_c,
        temperature_power,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("temperature cost inputs must be finite")
    if blocked_temperature_c <= safe_temperature_c:
        raise ValueError("blocked_temperature_c must exceed safe_temperature_c")
    if temperature_power <= 0.0:
        raise ValueError("temperature_power must be positive")
    if temperature_c <= safe_temperature_c:
        return 0
    if temperature_c >= blocked_temperature_c:
        return 100
    ratio = (temperature_c - safe_temperature_c) / (
        blocked_temperature_c - safe_temperature_c
    )
    risk = min(1.0, max(0.0, ratio)) ** temperature_power
    return max(1, min(99, round(99.0 * risk)))


def aggregate_cell_costs(cell_cost_pairs) -> dict[tuple[int, int], int]:
    """Retain the greatest cost for duplicate cells within one frame."""
    aggregated: dict[tuple[int, int], int] = {}
    for cell, cost in cell_cost_pairs:
        numeric_cost = int(cost)
        if not 0 <= numeric_cost <= 100:
            raise ValueError("thermal cell cost must be in [0, 100]")
        aggregated[cell] = max(aggregated.get(cell, 0), numeric_cost)
    return aggregated


def inflate_cell_costs(
    cell_costs: Mapping[tuple[int, int], int],
    geometry: GridGeometry,
    inflation_radius_m: float,
) -> dict[tuple[int, int], int]:
    """Expand costs with Euclidean, monotonically decreasing falloff."""
    if not math.isfinite(inflation_radius_m) or inflation_radius_m < 0.0:
        raise ValueError("inflation_radius_m must be finite and non-negative")
    result = aggregate_cell_costs(cell_costs.items())
    if inflation_radius_m == 0.0:
        return result
    radius_cells = int(math.ceil(inflation_radius_m / geometry.resolution))
    falloff_extent = inflation_radius_m + geometry.resolution
    for (center_x, center_y), center_cost in cell_costs.items():
        if center_cost <= 0:
            continue
        for offset_y in range(-radius_cells, radius_cells + 1):
            for offset_x in range(-radius_cells, radius_cells + 1):
                grid_x = center_x + offset_x
                grid_y = center_y + offset_y
                if not (0 <= grid_x < geometry.width and 0 <= grid_y < geometry.height):
                    continue
                distance = math.hypot(offset_x, offset_y) * geometry.resolution
                if distance > inflation_radius_m + 1e-12:
                    continue
                if distance == 0.0:
                    inflated_cost = int(center_cost)
                else:
                    factor = max(0.0, 1.0 - distance / falloff_extent)
                    inflated_cost = max(1, min(99, round(center_cost * factor)))
                cell = (grid_x, grid_y)
                result[cell] = max(result.get(cell, 0), inflated_cost)
    return result


class ThermalCostState:
    """ROS-independent full-grid storage with per-cell ROS-time timestamps."""

    def __init__(self, observation_timeout_sec: float, inflation_radius_m: float):
        if not math.isfinite(observation_timeout_sec) or observation_timeout_sec < 0.0:
            raise ValueError("observation_timeout_sec must be finite and non-negative")
        if not math.isfinite(inflation_radius_m) or inflation_radius_m < 0.0:
            raise ValueError("inflation_radius_m must be finite and non-negative")
        self.timeout_ns = int(round(observation_timeout_sec * 1_000_000_000))
        self.inflation_radius_m = float(inflation_radius_m)
        self.geometry: GridGeometry | None = None
        self.costs = np.zeros((0, 0), dtype=np.int8)
        self.last_observed_ns: dict[tuple[int, int], int] = {}

    def set_geometry(self, geometry: GridGeometry) -> bool:
        changed = geometry != self.geometry
        if changed:
            self.geometry = geometry
            self.costs = np.zeros((geometry.height, geometry.width), dtype=np.int8)
            self.last_observed_ns.clear()
        return changed

    def apply_frame(
        self, frame_costs: Mapping[tuple[int, int], int], now_ns: int
    ) -> None:
        if self.geometry is None:
            raise RuntimeError("thermal state has no static grid geometry")
        if now_ns < 0:
            raise ValueError("ROS time must be non-negative")
        if self.timeout_ns == 0:
            self.clear()
        expanded = inflate_cell_costs(
            frame_costs, self.geometry, self.inflation_radius_m
        )
        for (grid_x, grid_y), cost in expanded.items():
            if not (0 <= grid_x < self.geometry.width and 0 <= grid_y < self.geometry.height):
                continue
            self.costs[grid_y, grid_x] = int(cost)
            if cost > 0:
                self.last_observed_ns[(grid_x, grid_y)] = int(now_ns)
            else:
                self.last_observed_ns.pop((grid_x, grid_y), None)

    def expire(self, now_ns: int) -> int:
        if self.geometry is None:
            return 0
        cutoff = int(now_ns) - self.timeout_ns
        expired = [
            cell for cell, observed_ns in self.last_observed_ns.items()
            if self.timeout_ns == 0 or observed_ns > now_ns or observed_ns < cutoff
        ]
        for grid_x, grid_y in expired:
            self.costs[grid_y, grid_x] = 0
            self.last_observed_ns.pop((grid_x, grid_y), None)
        return len(expired)

    def clear(self) -> int:
        count = int(np.count_nonzero(self.costs))
        self.costs.fill(0)
        self.last_observed_ns.clear()
        return count

    def flattened(self) -> list[int]:
        if self.geometry is None:
            return []
        expected = self.geometry.width * self.geometry.height
        result = self.costs.reshape(-1).astype(int).tolist()
        if len(result) != expected:
            raise RuntimeError("thermal grid data length does not match geometry")
        return result
