#!/usr/bin/env python3
"""Live metric ranging from a Rational8 camera and a planar LaserScan.

The program deliberately refuses stale or poorly synchronized data.  It uses
the raw camera image, rectifies it with ``alpha=0``, transforms every valid
2D LaserScan sample with the calibrated ``lidar -> camera`` transform, and
projects the result with the rectified camera matrix.  A click is accepted
only when a projected scan sample exists within 20 native-image pixels.

This is a visualization/measurement tool, not an object detector.  A 2D lidar
only measures the horizontal plane swept by its beam, so image objects that do
not intersect that plane have no lidar distance support.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import math
import os
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import cv2
import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CAMERA_INFO = (
    PROJECT_ROOT / "outputs/pi_camera3_wide_intrinsic/camera_info.yaml"
)
DEFAULT_EXTRINSIC = (
    PROJECT_ROOT
    / "outputs/pi_camera3_wide_extrinsic/lidar_camera_extrinsic.yaml"
)
DEFAULT_SCREENSHOTS = (
    PROJECT_ROOT / "outputs/pi_camera3_wide_extrinsic/distance_screenshots"
)

TRANSFORM_CONVENTION = (
    "p_camera = R_camera_lidar * p_lidar + t_camera_lidar_m"
)
DISTORTION_MODEL = "rational_polynomial"
DISTORTION_COEFFICIENT_COUNT = 8
MAX_STAMP_SKEW_SEC = 0.10
MAX_STALE_SEC = 0.50
FUTURE_STAMP_TOLERANCE_SEC = 0.01
CLICK_RADIUS_PX = 20.0
FOREGROUND_CLUSTER_TOLERANCE_M = 0.10
WINDOW_NAME = "RPLIDAR C1 + Camera distance"


try:
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan

    _ROS_IMPORT_ERROR: Exception | None = None
except ImportError as exc:  # Allows non-ROS unit tests to import pure helpers.
    rclpy = None
    CvBridge = None
    qos_profile_sensor_data = None
    Image = None
    LaserScan = None
    _ROS_IMPORT_ERROR = exc

    class Node:  # type: ignore[no-redef]
        """Import-only placeholder used when ROS 2 is not sourced."""


@dataclass(frozen=True)
class RationalCameraCalibration:
    """Strictly validated ROS CameraInfo Rational8 parameters."""

    path: Path
    camera_name: str
    image_size: tuple[int, int]
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray


@dataclass(frozen=True)
class LidarCameraExtrinsic:
    """Strictly validated transform from the lidar to the optical camera."""

    path: Path
    camera_frame: str
    lidar_frame: str
    transform: np.ndarray
    rotation: np.ndarray
    translation: np.ndarray
    quality: Mapping[str, Any]


@dataclass(frozen=True)
class PairState:
    """Validity and timing diagnostics for an Image/LaserScan pair."""

    valid: bool
    reason: str
    skew_sec: float = math.nan
    image_age_sec: float = math.nan
    scan_age_sec: float = math.nan


@dataclass(frozen=True)
class ProjectedScan:
    """Visible scan samples in rectified pixels and camera coordinates."""

    pixels: np.ndarray
    lidar_ranges_m: np.ndarray
    camera_points_m: np.ndarray

    @classmethod
    def empty(cls) -> "ProjectedScan":
        return cls(
            np.empty((0, 2), dtype=np.float64),
            np.empty((0,), dtype=np.float64),
            np.empty((0, 3), dtype=np.float64),
        )


@dataclass(frozen=True)
class ClickMeasurement:
    """Nearest lidar-backed metric measurement at an image click."""

    index: int
    support_count: int
    pixel: tuple[float, float]
    pixel_distance: float
    lidar_range_cm: float
    camera_forward_z_cm: float
    camera_euclidean_cm: float


def _load_yaml_mapping(path: Path | str, label: str) -> tuple[Path, Mapping[str, Any]]:
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"{label} does not exist: {source}")
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f"could not read {label}: {source}: {exc}") from exc
    if not isinstance(payload, Mapping):
        raise ValueError(f"{label} root must be a YAML mapping")
    return source, payload


def _positive_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{label} must be a positive integer")
    return value


def _finite_array(value: Any, shape: tuple[int, ...], label: str) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain numeric values") from exc
    if array.shape != shape:
        raise ValueError(f"{label} must have exact shape {shape}, got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{label} must contain only finite values")
    return array.copy()


def _camera_info_matrix(
    payload: Mapping[str, Any],
    field: str,
    shape: tuple[int, int],
) -> np.ndarray:
    entry = payload.get(field)
    if not isinstance(entry, Mapping):
        raise ValueError(f"camera_info.yaml is missing mapping '{field}'")
    if entry.get("rows") != shape[0] or entry.get("cols") != shape[1]:
        raise ValueError(
            f"{field} must declare rows={shape[0]} and cols={shape[1]}"
        )
    values = _finite_array(
        entry.get("data"),
        (shape[0] * shape[1],),
        f"{field}.data",
    )
    return values.reshape(shape)


def load_rational_camera_info(
    path: Path | str,
) -> RationalCameraCalibration:
    """Load CameraInfo and require the exact eight-term Rational model."""

    source, payload = _load_yaml_mapping(path, "camera_info.yaml")
    if payload.get("distortion_model") != DISTORTION_MODEL:
        raise ValueError(
            f"distortion_model must be exactly '{DISTORTION_MODEL}'"
        )
    width = _positive_int(payload.get("image_width"), "image_width")
    height = _positive_int(payload.get("image_height"), "image_height")

    matrix = _camera_info_matrix(payload, "camera_matrix", (3, 3))
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("camera_matrix fx and fy must be positive")
    if not np.allclose(matrix[2], (0.0, 0.0, 1.0), atol=1.0e-12):
        raise ValueError("camera_matrix last row must be [0, 0, 1]")

    distortion = _camera_info_matrix(
        payload,
        "distortion_coefficients",
        (1, DISTORTION_COEFFICIENT_COUNT),
    ).reshape(DISTORTION_COEFFICIENT_COUNT)
    camera_name = payload.get("camera_name", "camera")
    if not isinstance(camera_name, str) or not camera_name.strip():
        raise ValueError("camera_name must be a non-empty string")

    return RationalCameraCalibration(
        path=source,
        camera_name=camera_name.strip(),
        image_size=(width, height),
        camera_matrix=matrix,
        distortion_coefficients=distortion,
    )


def _validate_rotation(rotation: np.ndarray, label: str) -> None:
    identity = rotation.T @ rotation
    if not np.allclose(identity, np.eye(3), atol=1.0e-6, rtol=0.0):
        raise ValueError(f"{label} must be orthonormal")
    determinant = float(np.linalg.det(rotation))
    if not math.isclose(determinant, 1.0, abs_tol=1.0e-6):
        raise ValueError(f"{label} determinant must be +1, got {determinant}")


def _validate_quality(payload: Mapping[str, Any]) -> Mapping[str, Any]:
    quality = payload.get("quality")
    if not isinstance(quality, Mapping):
        raise ValueError("extrinsic quality must be a mapping")
    if quality.get("passed") is not True:
        raise ValueError("extrinsic quality.passed must be true")

    for field in ("pose_count", "normal_rank", "jacobian_rank"):
        value = quality.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"extrinsic quality.{field} must be a non-negative integer")
    for field in (
        "jacobian_condition_number",
        "rmse_m",
        "median_abs_residual_m",
        "max_abs_residual_m",
    ):
        value = quality.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"extrinsic quality.{field} must be numeric")
        if not math.isfinite(float(value)) or float(value) < 0.0:
            raise ValueError(
                f"extrinsic quality.{field} must be finite and non-negative"
            )
    return dict(quality)


def load_lidar_camera_extrinsic(path: Path | str) -> LidarCameraExtrinsic:
    """Load a calibrated lidar-to-camera transform without guessing direction.

    The canonical 4x4 ``T_camera_lidar`` representation and the equivalent
    ``R_camera_lidar``/``t_camera_lidar_m`` pair are accepted.  When both are
    present they must agree numerically.
    """

    source, payload = _load_yaml_mapping(path, "extrinsic YAML")
    version = payload.get("schema_version")
    if isinstance(version, bool) or version != 1:
        raise ValueError("extrinsic schema_version must be integer 1")
    if payload.get("transform_convention") != TRANSFORM_CONVENTION:
        raise ValueError(
            "extrinsic transform_convention must be exactly: "
            f"{TRANSFORM_CONVENTION}"
        )

    frames = payload.get("frames")
    if not isinstance(frames, Mapping):
        raise ValueError("extrinsic frames must be a mapping")
    camera_frame = frames.get("camera")
    lidar_frame = frames.get("lidar")
    if not isinstance(camera_frame, str) or not camera_frame.strip():
        raise ValueError("extrinsic frames.camera must be a non-empty string")
    if lidar_frame != "laser":
        raise ValueError("extrinsic frames.lidar must be exactly 'laser'")
    camera_frame = camera_frame.strip()

    has_transform = "T_camera_lidar" in payload
    has_rotation = "R_camera_lidar" in payload
    has_translation = "t_camera_lidar_m" in payload
    if has_rotation != has_translation:
        raise ValueError(
            "R_camera_lidar and t_camera_lidar_m must be provided together"
        )
    if not has_transform and not has_rotation:
        raise ValueError(
            "extrinsic YAML needs T_camera_lidar or the R_camera_lidar/"
            "t_camera_lidar_m pair"
        )

    transform: np.ndarray | None = None
    transform_rotation: np.ndarray | None = None
    transform_translation: np.ndarray | None = None
    if has_transform:
        transform = _finite_array(
            payload.get("T_camera_lidar"),
            (4, 4),
            "T_camera_lidar",
        )
        if not np.allclose(
            transform[3], (0.0, 0.0, 0.0, 1.0), atol=1.0e-9, rtol=0.0
        ):
            raise ValueError("T_camera_lidar bottom row must be [0, 0, 0, 1]")
        transform_rotation = transform[:3, :3].copy()
        transform_translation = transform[:3, 3].copy()
        _validate_rotation(transform_rotation, "T_camera_lidar rotation")

    pair_rotation: np.ndarray | None = None
    pair_translation: np.ndarray | None = None
    if has_rotation:
        pair_rotation = _finite_array(
            payload.get("R_camera_lidar"),
            (3, 3),
            "R_camera_lidar",
        )
        pair_translation = _finite_array(
            payload.get("t_camera_lidar_m"),
            (3,),
            "t_camera_lidar_m",
        )
        _validate_rotation(pair_rotation, "R_camera_lidar")

    if transform is not None and pair_rotation is not None:
        if not np.allclose(
            transform_rotation, pair_rotation, atol=1.0e-9, rtol=0.0
        ):
            raise ValueError("T_camera_lidar rotation disagrees with R_camera_lidar")
        if not np.allclose(
            transform_translation, pair_translation, atol=1.0e-9, rtol=0.0
        ):
            raise ValueError(
                "T_camera_lidar translation disagrees with t_camera_lidar_m"
            )

    if transform is None:
        assert pair_rotation is not None and pair_translation is not None
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = pair_rotation
        transform[:3, 3] = pair_translation
        transform_rotation = pair_rotation
        transform_translation = pair_translation

    assert transform_rotation is not None and transform_translation is not None
    quality = _validate_quality(payload)
    return LidarCameraExtrinsic(
        path=source,
        camera_frame=camera_frame,
        lidar_frame=lidar_frame,
        transform=transform,
        rotation=transform_rotation,
        translation=transform_translation,
        quality=quality,
    )


def build_alpha_zero_rectification(
    calibration: RationalCameraCalibration,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[int, int, int, int]]:
    """Precompute raw-to-rectified maps and the matching alpha=0 matrix."""

    width, height = calibration.image_size
    new_matrix, roi = cv2.getOptimalNewCameraMatrix(
        calibration.camera_matrix,
        calibration.distortion_coefficients.reshape(1, -1),
        (width, height),
        0.0,
        (width, height),
    )
    new_matrix = np.asarray(new_matrix, dtype=np.float64)
    if new_matrix.shape != (3, 3) or not np.all(np.isfinite(new_matrix)):
        raise ValueError("OpenCV produced an invalid alpha=0 camera matrix")
    if new_matrix[0, 0] <= 0.0 or new_matrix[1, 1] <= 0.0:
        raise ValueError("OpenCV produced non-positive alpha=0 focal lengths")
    map_x, map_y = cv2.initUndistortRectifyMap(
        calibration.camera_matrix,
        calibration.distortion_coefficients.reshape(1, -1),
        None,
        new_matrix,
        (width, height),
        cv2.CV_32FC1,
    )
    return (
        map_x,
        map_y,
        new_matrix,
        tuple(int(value) for value in roi),
    )


def stamp_to_seconds(stamp: Any) -> float:
    """Convert a ROS builtin_interfaces/Time-like object to seconds."""

    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if isinstance(sec, bool) or not isinstance(sec, int):
        return math.nan
    if isinstance(nanosec, bool) or not isinstance(nanosec, int):
        return math.nan
    if nanosec < 0 or nanosec >= 1_000_000_000:
        return math.nan
    return float(sec) + float(nanosec) * 1.0e-9


def validate_pair(
    image_message: Any,
    scan_message: Any,
    now_sec: float,
    expected_image_size: tuple[int, int],
    expected_camera_frame: str,
    expected_lidar_frame: str,
) -> PairState:
    """Apply strict frame, resolution, stamp-skew, and freshness gates."""

    if image_message is None:
        return PairState(False, "WAITING_FOR_IMAGE")
    if scan_message is None:
        return PairState(False, "WAITING_FOR_SCAN")

    width, height = expected_image_size
    if (
        getattr(image_message, "width", None) != width
        or getattr(image_message, "height", None) != height
    ):
        return PairState(False, f"IMAGE_SIZE_NOT_{width}x{height}")

    image_header = getattr(image_message, "header", None)
    scan_header = getattr(scan_message, "header", None)
    image_frame = getattr(image_header, "frame_id", None)
    scan_frame = getattr(scan_header, "frame_id", None)
    if image_frame != expected_camera_frame:
        return PairState(False, f"CAMERA_FRAME_NOT_{expected_camera_frame}")
    if scan_frame != expected_lidar_frame:
        return PairState(False, f"LIDAR_FRAME_NOT_{expected_lidar_frame}")

    increment = float(getattr(scan_message, "angle_increment", math.nan))
    range_min = float(getattr(scan_message, "range_min", math.nan))
    range_max = float(getattr(scan_message, "range_max", math.nan))
    if not math.isfinite(increment) or abs(increment) <= 1.0e-12:
        return PairState(False, "INVALID_SCAN_ANGLE_INCREMENT")
    if (
        not math.isfinite(range_min)
        or not math.isfinite(range_max)
        or range_min < 0.0
        or range_max <= range_min
    ):
        return PairState(False, "INVALID_SCAN_RANGE_LIMITS")

    image_stamp = stamp_to_seconds(getattr(image_header, "stamp", None))
    scan_stamp = stamp_to_seconds(getattr(scan_header, "stamp", None))
    if (
        not math.isfinite(now_sec)
        or not math.isfinite(image_stamp)
        or not math.isfinite(scan_stamp)
        or image_stamp <= 0.0
        or scan_stamp <= 0.0
    ):
        return PairState(False, "INVALID_TIMESTAMP")

    image_age = now_sec - image_stamp
    scan_age = now_sec - scan_stamp
    skew = abs(image_stamp - scan_stamp)
    diagnostics = {
        "skew_sec": skew,
        "image_age_sec": image_age,
        "scan_age_sec": scan_age,
    }
    if (
        image_age < -FUTURE_STAMP_TOLERANCE_SEC
        or scan_age < -FUTURE_STAMP_TOLERANCE_SEC
    ):
        return PairState(False, "FUTURE_TIMESTAMP", **diagnostics)
    if image_age > MAX_STALE_SEC or scan_age > MAX_STALE_SEC:
        return PairState(False, "STALE_GT_0.50S", **diagnostics)
    if skew > MAX_STAMP_SKEW_SEC:
        return PairState(False, "STAMP_SKEW_GT_0.10S", **diagnostics)
    return PairState(True, "VALID", **diagnostics)


def project_laser_scan(
    scan_message: Any,
    extrinsic: LidarCameraExtrinsic,
    rectified_matrix: np.ndarray,
    image_size: tuple[int, int],
) -> ProjectedScan:
    """Transform a planar scan (z_lidar=0) and project visible samples."""

    ranges = np.asarray(getattr(scan_message, "ranges", ()), dtype=np.float64)
    if ranges.ndim != 1 or ranges.size == 0:
        return ProjectedScan.empty()
    angle_min = float(getattr(scan_message, "angle_min", math.nan))
    angle_increment = float(getattr(scan_message, "angle_increment", math.nan))
    range_min = float(getattr(scan_message, "range_min", math.nan))
    range_max = float(getattr(scan_message, "range_max", math.nan))
    if not all(
        math.isfinite(value)
        for value in (angle_min, angle_increment, range_min, range_max)
    ):
        return ProjectedScan.empty()

    angles = angle_min + np.arange(ranges.size, dtype=np.float64) * angle_increment
    valid = (
        np.isfinite(ranges)
        & (ranges >= range_min)
        & (ranges <= range_max)
        & np.isfinite(angles)
    )
    if not np.any(valid):
        return ProjectedScan.empty()

    selected_ranges = ranges[valid]
    selected_angles = angles[valid]
    lidar_points = np.column_stack(
        (
            selected_ranges * np.cos(selected_angles),
            selected_ranges * np.sin(selected_angles),
            np.zeros(selected_ranges.size, dtype=np.float64),
        )
    )
    camera_points = (
        extrinsic.rotation @ lidar_points.T
    ).T + extrinsic.translation.reshape(1, 3)
    forward = camera_points[:, 2]
    visible = np.isfinite(camera_points).all(axis=1) & (forward > 1.0e-6)
    if not np.any(visible):
        return ProjectedScan.empty()
    selected_ranges = selected_ranges[visible]
    camera_points = camera_points[visible]

    normalized_x = camera_points[:, 0] / camera_points[:, 2]
    normalized_y = camera_points[:, 1] / camera_points[:, 2]
    pixels_x = (
        rectified_matrix[0, 0] * normalized_x
        + rectified_matrix[0, 1] * normalized_y
        + rectified_matrix[0, 2]
    )
    pixels_y = (
        rectified_matrix[1, 0] * normalized_x
        + rectified_matrix[1, 1] * normalized_y
        + rectified_matrix[1, 2]
    )
    width, height = image_size
    in_image = (
        np.isfinite(pixels_x)
        & np.isfinite(pixels_y)
        & (pixels_x >= 0.0)
        & (pixels_x < width)
        & (pixels_y >= 0.0)
        & (pixels_y < height)
    )
    if not np.any(in_image):
        return ProjectedScan.empty()
    return ProjectedScan(
        pixels=np.column_stack((pixels_x[in_image], pixels_y[in_image])),
        lidar_ranges_m=selected_ranges[in_image],
        camera_points_m=camera_points[in_image],
    )


def measurement_near_click(
    projected: ProjectedScan,
    click_pixel: tuple[float, float],
    radius_px: float = CLICK_RADIUS_PX,
) -> ClickMeasurement | None:
    """Return a foreground-safe scan sample if click support exists.

    Camera projection can place a foreground edge and a farther background in
    the same click neighborhood. First retain the cluster no farther than
    10 cm behind the minimum lidar range, then choose the pixel-nearest sample
    only within that foreground cluster.
    """

    if radius_px <= 0.0 or projected.pixels.shape[0] == 0:
        return None
    click = np.asarray(click_pixel, dtype=np.float64)
    if click.shape != (2,) or not np.all(np.isfinite(click)):
        return None
    squared = np.sum((projected.pixels - click.reshape(1, 2)) ** 2, axis=1)
    supported = squared <= radius_px * radius_px
    support_count = int(np.count_nonzero(supported))
    if support_count == 0:
        return None
    supported_indices = np.flatnonzero(supported)
    minimum_range = float(np.min(projected.lidar_ranges_m[supported_indices]))
    foreground = supported_indices[
        projected.lidar_ranges_m[supported_indices]
        <= minimum_range + FOREGROUND_CLUSTER_TOLERANCE_M
    ]
    support_count = int(foreground.size)
    index = int(foreground[np.argmin(squared[foreground])])
    camera_point = projected.camera_points_m[index]
    return ClickMeasurement(
        index=index,
        support_count=support_count,
        pixel=(
            float(projected.pixels[index, 0]),
            float(projected.pixels[index, 1]),
        ),
        pixel_distance=float(math.sqrt(float(squared[index]))),
        lidar_range_cm=float(projected.lidar_ranges_m[index] * 100.0),
        camera_forward_z_cm=float(camera_point[2] * 100.0),
        camera_euclidean_cm=float(np.linalg.norm(camera_point) * 100.0),
    )


def _projection_layer(
    projected: ProjectedScan,
    image_size: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray]:
    """Build a cached 3x3 colored-dot layer for a projected scan."""

    width, height = image_size
    layer = np.zeros((height, width, 3), dtype=np.uint8)
    mask = np.zeros((height, width), dtype=bool)
    if projected.pixels.shape[0] == 0:
        return layer, mask

    pixels = np.rint(projected.pixels).astype(np.int32)
    ranges = projected.lidar_ranges_m
    normalized = np.clip((ranges - 0.2) / 4.8, 0.0, 1.0)
    colors = np.column_stack(
        (
            np.full(ranges.size, 40.0),
            255.0 * normalized,
            255.0 * (1.0 - normalized),
        )
    ).astype(np.uint8)
    for offset_y in (-1, 0, 1):
        for offset_x in (-1, 0, 1):
            x_values = pixels[:, 0] + offset_x
            y_values = pixels[:, 1] + offset_y
            valid = (
                (x_values >= 0)
                & (x_values < width)
                & (y_values >= 0)
                & (y_values < height)
            )
            layer[y_values[valid], x_values[valid]] = colors[valid]
            mask[y_values[valid], x_values[valid]] = True
    return layer, mask


def _put_text(
    image: np.ndarray,
    text: str,
    position: tuple[int, int],
    color: tuple[int, int, int],
    scale: float = 0.55,
) -> None:
    cv2.putText(
        image,
        text,
        position,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        2,
        cv2.LINE_AA,
    )


class LidarCameraDistanceNode(Node):
    """ROS 2 subscriptions, strict synchronization gates, and OpenCV GUI."""

    def __init__(
        self,
        camera: RationalCameraCalibration,
        extrinsic: LidarCameraExtrinsic,
        image_topic: str,
        scan_topic: str,
        screenshot_dir: Path,
        window_scale: float,
    ) -> None:
        if _ROS_IMPORT_ERROR is not None:
            raise RuntimeError(
                "ROS 2 Python modules are unavailable; source /opt/ros/jazzy/"
                f"setup.bash first ({_ROS_IMPORT_ERROR})"
            )
        super().__init__("lidar_camera_distance")
        self.camera = camera
        self.extrinsic = extrinsic
        self.image_topic = image_topic
        self.scan_topic = scan_topic
        self.screenshot_dir = screenshot_dir.expanduser().resolve()
        self.window_scale = window_scale
        self.bridge = CvBridge()

        (
            self.map_x,
            self.map_y,
            self.rectified_matrix,
            self.valid_roi,
        ) = build_alpha_zero_rectification(camera)

        self.latest_image_message: Any = None
        self.latest_raw_bgr: np.ndarray | None = None
        self.latest_image_error: str | None = None
        self.latest_scan_message: Any = None
        self.click_pixel: tuple[float, float] | None = None
        self.last_annotated: np.ndarray | None = None
        self.last_pair_state = PairState(False, "WAITING_FOR_IMAGE")
        self.last_measurement: ClickMeasurement | None = None
        self.cached_scan_message: Any = None
        self.cached_projection = ProjectedScan.empty()
        self.cached_layer = np.zeros(
            (camera.image_size[1], camera.image_size[0], 3), dtype=np.uint8
        )
        self.cached_layer_mask = np.zeros(
            (camera.image_size[1], camera.image_size[0]), dtype=bool
        )

        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / 30.0, self._draw)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW_NAME, self._mouse_callback)
        self.get_logger().info(f"Rational8: {camera.path}")
        self.get_logger().info(f"Extrinsic: {extrinsic.path}")
        self.get_logger().info(
            f"Transform: {extrinsic.lidar_frame} -> {extrinsic.camera_frame}"
        )
        self.get_logger().info(f"Image: {image_topic} (raw, alpha=0 rectified)")
        self.get_logger().info(f"LaserScan: {scan_topic}")
        self.get_logger().info(
            "Validity: stamp skew <= 0.10 s and both ages <= 0.50 s"
        )
        self.get_logger().info("Click distance | c screenshot | q exit")

    def _image_callback(self, message: Any) -> None:
        self.latest_image_message = message
        try:
            converted = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
            self.latest_raw_bgr = np.asarray(converted).copy()
            self.latest_image_error = None
        except Exception as exc:  # cv_bridge uses several exception classes.
            self.latest_raw_bgr = None
            self.latest_image_error = f"IMAGE_DECODE_FAILED: {exc}"

    def _scan_callback(self, message: Any) -> None:
        self.latest_scan_message = message

    def _mouse_callback(
        self,
        event: int,
        x_value: int,
        y_value: int,
        _flags: int,
        _parameter: Any,
    ) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        native_x = float(x_value) / self.window_scale
        native_y = float(y_value) / self.window_scale
        width, height = self.camera.image_size
        if 0.0 <= native_x < width and 0.0 <= native_y < height:
            self.click_pixel = (native_x, native_y)

    def _current_projection(self) -> ProjectedScan:
        scan = self.latest_scan_message
        if scan is self.cached_scan_message:
            return self.cached_projection
        self.cached_scan_message = scan
        self.cached_projection = project_laser_scan(
            scan,
            self.extrinsic,
            self.rectified_matrix,
            self.camera.image_size,
        )
        self.cached_layer, self.cached_layer_mask = _projection_layer(
            self.cached_projection,
            self.camera.image_size,
        )
        return self.cached_projection

    def _base_image(self) -> tuple[np.ndarray, str | None]:
        width, height = self.camera.image_size
        if self.latest_raw_bgr is None:
            return np.zeros((height, width, 3), dtype=np.uint8), self.latest_image_error
        image = self.latest_raw_bgr
        if image.ndim != 3 or image.shape[2] != 3:
            return np.zeros((height, width, 3), dtype=np.uint8), "IMAGE_NOT_BGR8"
        if (image.shape[1], image.shape[0]) != (width, height):
            return np.zeros((height, width, 3), dtype=np.uint8), (
                f"IMAGE_ARRAY_SIZE_NOT_{width}x{height}"
            )
        rectified = cv2.remap(
            image,
            self.map_x,
            self.map_y,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        return rectified, None

    def _annotate(
        self,
        display: np.ndarray,
        state: PairState,
        projected: ProjectedScan,
        measurement: ClickMeasurement | None,
    ) -> None:
        height, width = display.shape[:2]
        shaded = display.copy()
        top_height = min(height, 150)
        bottom_start = max(0, height - 68)
        cv2.rectangle(shaded, (0, 0), (width, top_height), (0, 0, 0), -1)
        cv2.rectangle(
            shaded, (0, bottom_start), (width, height), (0, 0, 0), -1
        )
        cv2.addWeighted(shaded, 0.70, display, 0.30, 0.0, dst=display)

        _put_text(
            display,
            "RPLIDAR C1 + CAMERA METRIC RANGE | RAW -> RATIONAL8 alpha=0",
            (14, 25),
            (255, 255, 255),
            0.54,
        )
        if state.valid:
            state_text = (
                f"VALID | skew={state.skew_sec:.3f}s "
                f"image_age={state.image_age_sec:.3f}s "
                f"scan_age={state.scan_age_sec:.3f}s "
                f"projected={projected.pixels.shape[0]}"
            )
            state_color = (70, 255, 70)
        else:
            state_text = f"INVALID | {state.reason}"
            state_color = (30, 30, 255)
        _put_text(display, state_text, (14, 52), state_color, 0.56)
        _put_text(
            display,
            f"frames: {self.extrinsic.lidar_frame} -> "
            f"{self.extrinsic.camera_frame} | support radius: 20 px",
            (14, 79),
            (220, 220, 220),
            0.51,
        )

        if not state.valid:
            selection_text = "distance: INVALID"
            selection_color = (30, 30, 255)
        elif self.click_pixel is None:
            selection_text = "distance: CLICK A PROJECTED LIDAR POINT"
            selection_color = (0, 255, 255)
        elif measurement is None:
            selection_text = "distance: NO_LIDAR_SUPPORT"
            selection_color = (0, 190, 255)
        else:
            selection_text = (
                f"lidar_range_cm={measurement.lidar_range_cm:.1f} | "
                f"camera_forward_z_cm={measurement.camera_forward_z_cm:.1f} | "
                f"camera_euclidean_cm={measurement.camera_euclidean_cm:.1f} | "
                f"support={measurement.support_count}"
            )
            selection_color = (80, 255, 80)
        _put_text(display, selection_text, (14, 108), selection_color, 0.55)
        _put_text(
            display,
            "Distance is valid only for the nearest projected scan sample.",
            (14, 136),
            (210, 210, 210),
            0.47,
        )

        if self.click_pixel is not None:
            center = tuple(int(round(value)) for value in self.click_pixel)
            cv2.circle(
                display,
                center,
                int(round(CLICK_RADIUS_PX)),
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.drawMarker(
                display,
                center,
                (255, 255, 255),
                cv2.MARKER_CROSS,
                11,
                1,
                cv2.LINE_AA,
            )
        if measurement is not None:
            selected = tuple(int(round(value)) for value in measurement.pixel)
            cv2.circle(display, selected, 8, (255, 255, 255), 2, cv2.LINE_AA)

        _put_text(
            display,
            "2D SCAN-PLANE LIMITATION: only surfaces intersecting lidar z=0 are measurable.",
            (14, max(18, height - 38)),
            (0, 190, 255),
            0.50,
        )
        _put_text(
            display,
            "Click dot: live distance | c: save annotated screenshot | q: exit",
            (14, max(18, height - 12)),
            (230, 230, 230),
            0.48,
        )

    def _draw(self) -> None:
        display, image_error = self._base_image()
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        if image_error is not None:
            state = PairState(False, image_error)
        else:
            state = validate_pair(
                self.latest_image_message,
                self.latest_scan_message,
                now_sec,
                self.camera.image_size,
                self.extrinsic.camera_frame,
                self.extrinsic.lidar_frame,
            )

        projected = ProjectedScan.empty()
        measurement = None
        if state.valid:
            projected = self._current_projection()
            display[self.cached_layer_mask] = self.cached_layer[
                self.cached_layer_mask
            ]
            if self.click_pixel is not None:
                measurement = measurement_near_click(
                    projected,
                    self.click_pixel,
                    CLICK_RADIUS_PX,
                )
        self.last_pair_state = state
        self.last_measurement = measurement
        self._annotate(display, state, projected, measurement)
        self.last_annotated = display.copy()

        shown = display
        if not math.isclose(self.window_scale, 1.0):
            shown = cv2.resize(
                display,
                (
                    max(1, int(round(display.shape[1] * self.window_scale))),
                    max(1, int(round(display.shape[0] * self.window_scale))),
                ),
                interpolation=(
                    cv2.INTER_AREA if self.window_scale < 1.0 else cv2.INTER_LINEAR
                ),
            )
        try:
            cv2.imshow(WINDOW_NAME, shown)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as exc:
            self.get_logger().error(f"OpenCV GUI failed: {exc}")
            rclpy.shutdown()
            return
        if key == ord("q"):
            rclpy.shutdown()
        elif key == ord("c"):
            self._save_screenshot()

    def _save_screenshot(self) -> None:
        if self.last_annotated is None:
            self.get_logger().warning("No rendered frame is available for screenshot")
            return
        self.screenshot_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = self.screenshot_dir / f"distance_{timestamp}.png"
        success, encoded = cv2.imencode(".png", self.last_annotated)
        if not success:
            self.get_logger().error(f"Could not encode screenshot: {path}")
            return
        try:
            encoded.tofile(path)
        except OSError as exc:
            self.get_logger().error(f"Could not save screenshot: {path}: {exc}")
            return
        self.get_logger().info(f"Screenshot: {path.resolve()}")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Rectify /camera/image_raw with Rational8 intrinsics, project /scan "
            "through the calibrated lidar-to-camera transform, and report "
            "lidar-backed metric distance at a click."
        )
    )
    parser.add_argument(
        "--camera-info",
        type=Path,
        default=DEFAULT_CAMERA_INFO,
        help=f"Rational8 CameraInfo YAML (default: {DEFAULT_CAMERA_INFO})",
    )
    parser.add_argument(
        "--extrinsic",
        type=Path,
        default=DEFAULT_EXTRINSIC,
        help=f"lidar-to-camera extrinsic YAML (default: {DEFAULT_EXTRINSIC})",
    )
    parser.add_argument(
        "--image-topic",
        default="/camera/image_raw",
        help="raw sensor_msgs/Image topic (default: /camera/image_raw)",
    )
    parser.add_argument(
        "--scan-topic",
        default="/scan",
        help="sensor_msgs/LaserScan topic (default: /scan)",
    )
    parser.add_argument(
        "--screenshot-dir",
        type=Path,
        default=DEFAULT_SCREENSHOTS,
        help=f"directory used by the c key (default: {DEFAULT_SCREENSHOTS})",
    )
    parser.add_argument(
        "--window-scale",
        type=float,
        default=1.0,
        help="GUI scale only; click radius remains 20 native pixels (default: 1.0)",
    )
    return parser


def _validate_arguments(args: argparse.Namespace) -> None:
    if not isinstance(args.image_topic, str) or not args.image_topic.startswith("/"):
        raise ValueError("--image-topic must be an absolute ROS topic")
    if not isinstance(args.scan_topic, str) or not args.scan_topic.startswith("/"):
        raise ValueError("--scan-topic must be an absolute ROS topic")
    if (
        not math.isfinite(args.window_scale)
        or args.window_scale < 0.25
        or args.window_scale > 2.0
    ):
        raise ValueError("--window-scale must be finite and between 0.25 and 2.0")


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        _validate_arguments(args)
        camera = load_rational_camera_info(args.camera_info)
        extrinsic = load_lidar_camera_extrinsic(args.extrinsic)
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))

    if _ROS_IMPORT_ERROR is not None:
        parser.error(
            "ROS 2 Python modules are unavailable. Source /opt/ros/jazzy/"
            f"setup.bash first: {_ROS_IMPORT_ERROR}"
        )
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error(
            "no GUI display is available (DISPLAY/WAYLAND_DISPLAY is unset)"
        )

    node: LidarCameraDistanceNode | None = None
    rclpy.init(args=[])
    try:
        node = LidarCameraDistanceNode(
            camera=camera,
            extrinsic=extrinsic,
            image_topic=args.image_topic,
            scan_topic=args.scan_topic,
            screenshot_dir=args.screenshot_dir,
            window_scale=args.window_scale,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
