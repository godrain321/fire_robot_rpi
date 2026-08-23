"""Hardware- and ROS-independent thermal image geometry helpers."""

from __future__ import annotations

import math

import numpy as np


THERMAL_HEIGHT = 24
THERMAL_WIDTH = 32


def validate_temperature_array(temperature_array) -> np.ndarray:
    """Return a float32 24x32 view, rejecting the wrong shape or infinities."""
    values = np.asarray(temperature_array, dtype=np.float32)
    expected = (THERMAL_HEIGHT, THERMAL_WIDTH)
    if values.shape != expected:
        raise ValueError(f"temperature array shape must be {expected}, got {values.shape}")
    if np.isinf(values).any():
        raise ValueError("temperature array contains an infinite value")
    return values


def apply_orientation(
    temperature_array,
    *,
    flip_horizontal: bool = False,
    flip_vertical: bool = False,
    rotate_180: bool = False,
) -> np.ndarray:
    """Apply isolated mounting corrections while preserving a 24x32 shape."""
    values = validate_temperature_array(temperature_array)
    if flip_horizontal:
        values = np.fliplr(values)
    if flip_vertical:
        values = np.flipud(values)
    if rotate_180:
        values = np.rot90(values, 2)
    return np.ascontiguousarray(values, dtype=np.float32)


def compute_column_max(temperature_array) -> np.ndarray:
    """Return each column's maximum, ignoring isolated NaNs.

    A column containing only NaNs makes the frame geometrically unusable and
    raises ``ValueError``. The sensor node applies the stricter policy of
    dropping any frame containing a non-finite sample.
    """
    values = validate_temperature_array(temperature_array)
    all_nan_columns = np.isnan(values).all(axis=0)
    if all_nan_columns.any():
        columns = np.flatnonzero(all_nan_columns).tolist()
        raise ValueError(f"temperature columns contain only NaN values: {columns}")
    return np.nanmax(values, axis=0).astype(np.float32, copy=False)


def compute_column_angles(width: int, horizontal_fov_deg: float) -> np.ndarray:
    """Return pixel-centre angles, positive toward camera/ROS left (+y)."""
    if isinstance(width, bool) or not isinstance(width, (int, np.integer)) or width < 1:
        raise ValueError("width must be a positive integer")
    if not math.isfinite(horizontal_fov_deg) or not 0.0 < horizontal_fov_deg < 180.0:
        raise ValueError("horizontal_fov_deg must be finite and in (0, 180)")
    indices = np.arange(width, dtype=np.float64)
    horizontal_fov_rad = math.radians(float(horizontal_fov_deg))
    return (0.5 - (indices + 0.5) / width) * horizontal_fov_rad


def project_columns_to_arc(
    column_temperatures,
    horizontal_fov_deg: float,
    distance_m: float,
) -> np.ndarray:
    """Project column temperatures to ``[x, y, z, intensity]`` arc points."""
    temperatures = np.asarray(column_temperatures, dtype=np.float32)
    if temperatures.ndim != 1 or temperatures.size < 1:
        raise ValueError("column_temperatures must be a non-empty 1-D array")
    if not np.isfinite(temperatures).all():
        raise ValueError("column_temperatures must contain only finite values")
    if not math.isfinite(distance_m) or distance_m <= 0.0:
        raise ValueError("distance_m must be finite and positive")

    angles = compute_column_angles(temperatures.size, horizontal_fov_deg)
    points = np.empty((temperatures.size, 4), dtype=np.float32)
    points[:, 0] = float(distance_m) * np.cos(angles)
    points[:, 1] = float(distance_m) * np.sin(angles)
    points[:, 2] = 0.0
    points[:, 3] = temperatures
    return points
