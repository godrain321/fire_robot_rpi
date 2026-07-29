"""Dependency-free helpers for planar yaw and quaternion conversion."""

import math
from typing import Tuple


def normalize_yaw(yaw: float) -> float:
    """Normalize an angle to the inclusive range [-pi, pi]."""
    yaw = float(yaw)
    if not math.isfinite(yaw):
        raise ValueError('yaw must be a finite number')
    normalized = math.atan2(math.sin(yaw), math.cos(yaw))
    if abs(normalized) < 1.0e-15:
        return 0.0
    return normalized


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Return a normalized (x, y, z, w) quaternion for a planar yaw."""
    half_yaw = normalize_yaw(yaw) * 0.5
    z = math.sin(half_yaw)
    w = math.cos(half_yaw)
    norm = math.hypot(z, w)
    return (0.0, 0.0, z / norm, w / norm)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw after normalizing the supplied quaternion."""
    values = tuple(float(value) for value in (x, y, z, w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion components must be finite numbers')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-15:
        raise ValueError('zero-length quaternion is invalid')
    x, y, z, w = (value / norm for value in values)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return normalize_yaw(math.atan2(sin_yaw, cos_yaw))
