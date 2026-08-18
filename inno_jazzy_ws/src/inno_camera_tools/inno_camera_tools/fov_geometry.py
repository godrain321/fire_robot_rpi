"""Small, ROS-independent helpers for camera field-of-view measurements."""

import math
from typing import Tuple


def field_of_view_deg(
    width_px: float,
    height_px: float,
    focal_x_px: float,
    focal_y_px: float,
) -> Tuple[float, float]:
    """Return horizontal and vertical pinhole field of view in degrees."""

    values = (width_px, height_px, focal_x_px, focal_y_px)
    if not all(
        math.isfinite(value) and value > 0.0 for value in values
    ):
        raise ValueError(
            'image size and focal lengths must be finite and positive'
        )
    horizontal = math.degrees(2.0 * math.atan(width_px / (2.0 * focal_x_px)))
    vertical = math.degrees(2.0 * math.atan(height_px / (2.0 * focal_y_px)))
    return horizontal, vertical


def plane_coverage_m(
    distance_m: float,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
) -> Tuple[float, float]:
    """Return the visible width and height on a plane normal to the camera."""

    values = (distance_m, horizontal_fov_deg, vertical_fov_deg)
    if not all(
        math.isfinite(value) and value > 0.0 for value in values
    ):
        raise ValueError(
            'distance and field of view must be finite and positive'
        )
    if horizontal_fov_deg >= 180.0 or vertical_fov_deg >= 180.0:
        raise ValueError('field of view must be less than 180 degrees')
    width = 2.0 * distance_m * math.tan(math.radians(horizontal_fov_deg) * 0.5)
    height = 2.0 * distance_m * math.tan(math.radians(vertical_fov_deg) * 0.5)
    return width, height


def object_size_px(
    distance_m: float,
    object_width_m: float,
    object_height_m: float,
    focal_x_px: float,
    focal_y_px: float,
) -> Tuple[float, float]:
    """Project a front-facing reference object's size with a pinhole model."""

    values = (
        distance_m,
        object_width_m,
        object_height_m,
        focal_x_px,
        focal_y_px,
    )
    if not all(
        math.isfinite(value) and value > 0.0 for value in values
    ):
        raise ValueError(
            'distance, object size, and focal lengths must be positive'
        )
    return (
        focal_x_px * object_width_m / distance_m,
        focal_y_px * object_height_m / distance_m,
    )


def scaled_focal_lengths(
    focal_x_px: float,
    focal_y_px: float,
    calibration_width_px: float,
    calibration_height_px: float,
    frame_width_px: float,
    frame_height_px: float,
) -> Tuple[float, float]:
    """Scale calibrated focal lengths to the current frame dimensions."""

    values = (
        focal_x_px,
        focal_y_px,
        calibration_width_px,
        calibration_height_px,
        frame_width_px,
        frame_height_px,
    )
    if not all(
        math.isfinite(value) and value > 0.0 for value in values
    ):
        raise ValueError('focal lengths and image dimensions must be positive')
    return (
        focal_x_px * frame_width_px / calibration_width_px,
        focal_y_px * frame_height_px / calibration_height_px,
    )
