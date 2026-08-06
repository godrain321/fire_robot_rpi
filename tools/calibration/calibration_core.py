"""Core routines for offline checkerboard Rational Polynomial calibration.

The module deliberately has no ROS dependency.  It implements one camera model:
OpenCV's eight-coefficient ``CALIB_RATIONAL_MODEL`` in the coefficient order
``[k1, k2, p1, p2, k3, k4, k5, k6]``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import glob
import json
import logging
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import cv2
import numpy as np

try:  # PyYAML is present on ROS 2 desktop installs, but keep imports optional.
    import yaml
except ImportError:  # pragma: no cover - exercised only on minimal installations
    yaml = None


LOGGER = logging.getLogger(__name__)

RATIONAL_COEFFICIENT_NAMES = (
    "k1",
    "k2",
    "p1",
    "p2",
    "k3",
    "k4",
    "k5",
    "k6",
)
POSE_DESCRIPTOR_NAMES = (
    "center_x_normalized",
    "center_y_normalized",
    "convex_hull_area_ratio",
    "top_edge_angle_normalized",
    "left_right_length_ratio",
    "top_bottom_length_ratio",
    "perspective_change",
)

# No thin-prism, tilted-sensor, zero-tangent, fixed-principal-point, or fixed
# aspect-ratio flag is included here.  Keeping this constant in one place makes
# accidental changes to the deployed model easy to review.
RATIONAL_CALIBRATION_FLAGS = int(
    cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL
)
DEFAULT_CORNER_FLAGS = int(
    cv2.CALIB_CB_NORMALIZE_IMAGE
    | cv2.CALIB_CB_EXHAUSTIVE
    | cv2.CALIB_CB_ACCURACY
)


@dataclass(frozen=True)
class CheckerboardConfig:
    """Physical checkerboard description using *inner* corner counts."""

    inner_corners_cols: int = 8
    inner_corners_rows: int = 9
    square_size_m: float = 0.070

    def __post_init__(self) -> None:
        if isinstance(self.inner_corners_cols, bool) or self.inner_corners_cols < 2:
            raise ValueError("inner_corners_cols must be an integer >= 2")
        if isinstance(self.inner_corners_rows, bool) or self.inner_corners_rows < 2:
            raise ValueError("inner_corners_rows must be an integer >= 2")
        if not math.isfinite(float(self.square_size_m)) or self.square_size_m <= 0:
            raise ValueError("square_size_m must be a finite value greater than zero")

    @property
    def pattern_size(self) -> tuple[int, int]:
        """Return OpenCV's ``(columns, rows)`` pattern size."""

        return (int(self.inner_corners_cols), int(self.inner_corners_rows))

    @property
    def point_count(self) -> int:
        """Return the number of internal checkerboard corners."""

        return int(self.inner_corners_cols * self.inner_corners_rows)

    # Short aliases are convenient in formulas and preserve an unambiguous
    # canonical name for YAML/CLI I/O.
    @property
    def cols(self) -> int:
        return self.inner_corners_cols

    @property
    def rows(self) -> int:
        return self.inner_corners_rows

    def to_dict(self) -> dict[str, int | float]:
        """Return a JSON/YAML-safe representation."""

        return {
            "inner_corners_cols": int(self.inner_corners_cols),
            "inner_corners_rows": int(self.inner_corners_rows),
            "square_size_m": float(self.square_size_m),
        }


@dataclass(frozen=True)
class CalibrationConfig:
    """Numerical, splitting, and robust-rejection calibration settings."""

    validation_ratio: float = 0.20
    max_iterations: int = 200
    epsilon: float = 1.0e-12
    mad_multiplier: float = 3.0
    max_rejection_ratio: float = 0.15
    minimum_training_views: int = 20
    cv_folds: int = 5
    max_outlier_iterations: int = 3
    duplicate_distance_threshold: float = 0.08
    coverage_grid_cols: int = 8
    coverage_grid_rows: int = 6

    def __post_init__(self) -> None:
        if not 0.0 <= float(self.validation_ratio) < 1.0:
            raise ValueError("validation_ratio must be in [0, 1)")
        if isinstance(self.max_iterations, bool) or self.max_iterations <= 0:
            raise ValueError("max_iterations must be a positive integer")
        if not math.isfinite(float(self.epsilon)) or self.epsilon <= 0:
            raise ValueError("epsilon must be finite and greater than zero")
        if not math.isfinite(float(self.mad_multiplier)) or self.mad_multiplier < 0:
            raise ValueError("mad_multiplier must be finite and non-negative")
        if not 0.0 <= float(self.max_rejection_ratio) < 1.0:
            raise ValueError("max_rejection_ratio must be in [0, 1)")
        if (
            isinstance(self.minimum_training_views, bool)
            or self.minimum_training_views < 3
        ):
            raise ValueError("minimum_training_views must be an integer >= 3")
        if isinstance(self.cv_folds, bool) or self.cv_folds < 2:
            raise ValueError("cv_folds must be an integer >= 2")
        if (
            isinstance(self.max_outlier_iterations, bool)
            or not 0 <= self.max_outlier_iterations <= 3
        ):
            raise ValueError("max_outlier_iterations must be in [0, 3]")
        if (
            not math.isfinite(float(self.duplicate_distance_threshold))
            or self.duplicate_distance_threshold < 0
        ):
            raise ValueError("duplicate_distance_threshold must be non-negative")
        if self.coverage_grid_cols <= 0 or self.coverage_grid_rows <= 0:
            raise ValueError("coverage grid dimensions must be positive")

    @property
    def calibration_flags(self) -> int:
        """Return the fixed Rational Polynomial optimization flags."""

        return RATIONAL_CALIBRATION_FLAGS

    @property
    def termination_criteria(self) -> tuple[int, int, float]:
        """Return the OpenCV iterative optimization termination criteria."""

        return (
            int(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER),
            int(self.max_iterations),
            float(self.epsilon),
        )

    def to_dict(self) -> dict[str, int | float]:
        """Return a JSON/YAML-safe representation."""

        return {
            "validation_ratio": float(self.validation_ratio),
            "max_iterations": int(self.max_iterations),
            "epsilon": float(self.epsilon),
            "mad_multiplier": float(self.mad_multiplier),
            "max_rejection_ratio": float(self.max_rejection_ratio),
            "minimum_training_views": int(self.minimum_training_views),
            "cv_folds": int(self.cv_folds),
            "max_outlier_iterations": int(self.max_outlier_iterations),
            "duplicate_distance_threshold": float(
                self.duplicate_distance_threshold
            ),
            "coverage_grid_cols": int(self.coverage_grid_cols),
            "coverage_grid_rows": int(self.coverage_grid_rows),
        }


@dataclass
class DetectedView:
    """Detection result and image metadata for one input file.

    ``corners`` uses OpenCV's ``(N, 1, 2)`` float32 convention whenever a
    detection is accepted.  Rejected/corrupt files remain in the returned list
    so reports can account for every input path.
    """

    path: Path
    width: int = 0
    height: int = 0
    read_success: bool = True
    detection_success: bool = True
    corners: np.ndarray | None = None
    corner_count: int = 0
    center: tuple[float, float] | None = None
    area_ratio: float = 0.0
    blur_score: float = math.nan
    exclusion_reason: str | None = None
    pose_descriptor: np.ndarray | None = None
    duplicate_of: Path | None = None

    def __post_init__(self) -> None:
        self.path = Path(self.path)
        self.width = int(self.width)
        self.height = int(self.height)
        if self.width < 0 or self.height < 0:
            raise ValueError("image width and height cannot be negative")
        if self.corners is not None:
            self.corners = _as_image_points(self.corners)
            if self.corner_count == 0:
                self.corner_count = int(self.corners.shape[0])
        self.corner_count = int(self.corner_count)
        if self.pose_descriptor is not None:
            descriptor = np.asarray(self.pose_descriptor, dtype=np.float64).reshape(-1)
            if descriptor.size != len(POSE_DESCRIPTOR_NAMES):
                raise ValueError(
                    "pose_descriptor must contain exactly "
                    f"{len(POSE_DESCRIPTOR_NAMES)} values"
                )
            if not np.all(np.isfinite(descriptor)):
                raise ValueError("pose_descriptor contains NaN or Inf")
            self.pose_descriptor = descriptor
        if self.duplicate_of is not None:
            self.duplicate_of = Path(self.duplicate_of)

    @property
    def image_size(self) -> tuple[int, int]:
        """Return ``(width, height)``."""

        return (self.width, self.height)

    @property
    def accepted(self) -> bool:
        """Whether this view is eligible for calibration."""

        return bool(
            self.read_success
            and self.detection_success
            and self.corners is not None
            and not self.exclusion_reason
        )

    @property
    def found(self) -> bool:
        """Compatibility alias for ``detection_success``."""

        return self.detection_success

    @property
    def descriptor(self) -> np.ndarray | None:
        """Compatibility alias for ``pose_descriptor``."""

        return self.pose_descriptor

    def to_metadata_dict(self) -> dict[str, Any]:
        """Return serializable per-image metadata for CSV/JSON reports."""

        center_x, center_y = self.center or (math.nan, math.nan)
        row: dict[str, Any] = {
            "path": str(self.path),
            "width": self.width,
            "height": self.height,
            "read_success": self.read_success,
            "detection_success": self.detection_success,
            "corner_count": self.corner_count,
            "center_x": float(center_x),
            "center_y": float(center_y),
            "area_ratio": float(self.area_ratio),
            "blur_score": float(self.blur_score),
            "exclusion_reason": self.exclusion_reason or "",
            "duplicate_of": str(self.duplicate_of) if self.duplicate_of else "",
        }
        if self.pose_descriptor is not None:
            row.update(
                {
                    name: float(value)
                    for name, value in zip(
                        POSE_DESCRIPTOR_NAMES, self.pose_descriptor, strict=True
                    )
                }
            )
        return row


@dataclass
class CalibrationResult:
    """Complete output from ``cv2.calibrateCameraExtended``."""

    rms: float
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    rvecs: list[np.ndarray]
    tvecs: list[np.ndarray]
    std_deviations_intrinsics: np.ndarray
    std_deviations_extrinsics: np.ndarray
    per_view_errors: np.ndarray
    views: list[DetectedView]
    image_size: tuple[int, int]
    warnings: list[str] = field(default_factory=list)
    extra_distortion_coefficients: np.ndarray = field(
        default_factory=lambda: np.empty(0, dtype=np.float64)
    )

    def __post_init__(self) -> None:
        self.rms = float(self.rms)
        self.camera_matrix = np.asarray(self.camera_matrix, dtype=np.float64).reshape(3, 3)
        self.distortion_coefficients = np.asarray(
            self.distortion_coefficients, dtype=np.float64
        ).reshape(-1)
        if self.distortion_coefficients.size != 8:
            raise ValueError("Rational distortion_coefficients must have length 8")
        self.rvecs = [np.asarray(value, dtype=np.float64).reshape(3, 1) for value in self.rvecs]
        self.tvecs = [np.asarray(value, dtype=np.float64).reshape(3, 1) for value in self.tvecs]
        self.std_deviations_intrinsics = np.asarray(
            self.std_deviations_intrinsics, dtype=np.float64
        ).reshape(-1)
        self.std_deviations_extrinsics = np.asarray(
            self.std_deviations_extrinsics, dtype=np.float64
        ).reshape(-1)
        self.per_view_errors = np.asarray(self.per_view_errors, dtype=np.float64).reshape(-1)
        self.extra_distortion_coefficients = np.asarray(
            self.extra_distortion_coefficients, dtype=np.float64
        ).reshape(-1)
        self.views = list(self.views)
        self.image_size = (int(self.image_size[0]), int(self.image_size[1]))
        if not math.isfinite(self.rms):
            raise ValueError("calibration RMS is NaN or Inf")
        if not np.all(np.isfinite(self.camera_matrix)):
            raise ValueError("camera matrix contains NaN or Inf")
        if not np.all(np.isfinite(self.distortion_coefficients)):
            raise ValueError("distortion coefficients contain NaN or Inf")
        if self.per_view_errors.size != len(self.views):
            raise ValueError("per_view_errors length does not match calibrated views")

    @property
    def K(self) -> np.ndarray:
        """Alias for the 3x3 camera matrix."""

        return self.camera_matrix

    @property
    def D(self) -> np.ndarray:
        """Alias for the ordered eight Rational coefficients."""

        return self.distortion_coefficients

    @property
    def overall_rms(self) -> float:
        """Alias used by result writers."""

        return self.rms

    @property
    def std_intrinsics(self) -> np.ndarray:
        return self.std_deviations_intrinsics

    @property
    def std_extrinsics(self) -> np.ndarray:
        return self.std_deviations_extrinsics

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-safe calibration payload."""

        return {
            "overall_rms": self.rms,
            "K": self.camera_matrix.tolist(),
            "D": self.distortion_coefficients.tolist(),
            "coefficient_order": list(RATIONAL_COEFFICIENT_NAMES),
            "rvecs": [value.reshape(-1).tolist() for value in self.rvecs],
            "tvecs": [value.reshape(-1).tolist() for value in self.tvecs],
            "std_deviations_intrinsics": self.std_deviations_intrinsics.tolist(),
            "std_deviations_extrinsics": self.std_deviations_extrinsics.tolist(),
            "per_view_rms": self.per_view_errors.tolist(),
            "views": [str(view.path) for view in self.views],
            "image_width": self.image_size[0],
            "image_height": self.image_size[1],
            "warnings": list(self.warnings),
        }


@dataclass
class ValidationResult:
    """Model-fixed validation statistics shared by CLI/reporting modules."""

    rms: float
    mean: float
    median: float
    std: float
    p90: float
    p95: float
    p99: float
    maximum: float
    center_region_rms: float
    edge_region_rms: float
    four_corner_region_rms: float
    per_view_rms: dict[str, float] = field(default_factory=dict)
    per_corner_errors: list[dict[str, Any]] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-safe validation summary."""

        return {
            "validation_rms": float(self.rms),
            "mean": float(self.mean),
            "median": float(self.median),
            "std": float(self.std),
            "p90": float(self.p90),
            "p95": float(self.p95),
            "p99": float(self.p99),
            "maximum": float(self.maximum),
            "center_region_rms": float(self.center_region_rms),
            "edge_region_rms": float(self.edge_region_rms),
            "four_corner_region_rms": float(self.four_corner_region_rms),
            "per_view_rms": dict(self.per_view_rms),
            "warnings": list(self.warnings),
        }


@dataclass(frozen=True)
class MadStatistics:
    """Median absolute deviation threshold components."""

    median: float
    mad: float
    robust_sigma: float
    threshold: float


@dataclass(frozen=True)
class RejectedView:
    """Audit record for one view removed by robust rejection."""

    path: Path
    per_view_rms: float
    robust_threshold: float
    iteration: int
    coverage_impact: Mapping[str, float | int | bool]
    reason: str = "per_view_rms_above_mad_threshold"

    def to_dict(self) -> dict[str, Any]:
        """Return a CSV/JSON-safe record."""

        return {
            "path": str(self.path),
            "per_view_rms": float(self.per_view_rms),
            "robust_threshold": float(self.robust_threshold),
            "iteration": int(self.iteration),
            "coverage_impact": json.dumps(dict(self.coverage_impact), sort_keys=True),
            "reason": self.reason,
        }


def load_config(path: str | Path | None) -> tuple[CheckerboardConfig, CalibrationConfig]:
    """Load board and calibration settings, applying code defaults first.

    CLI overrides are intentionally not handled here; callers should apply them
    to the returned dataclasses (for example with ``dataclasses.replace``), which
    yields the required CLI > YAML > code-default precedence.
    """

    if path is None:
        return CheckerboardConfig(), CalibrationConfig()
    config_path = Path(path)
    if not config_path.is_file():
        raise FileNotFoundError(f"calibration config does not exist: {config_path}")
    if yaml is None:
        raise RuntimeError("PyYAML is required to read the calibration config")
    try:
        payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ValueError(f"failed to parse calibration config {config_path}: {exc}") from exc
    if payload is None:
        payload = {}
    if not isinstance(payload, Mapping):
        raise ValueError("calibration config root must be a mapping")

    board_payload = payload.get("board", {})
    calibration_payload = payload.get("calibration", {})
    if not isinstance(board_payload, Mapping):
        raise ValueError("config 'board' must be a mapping")
    if not isinstance(calibration_payload, Mapping):
        raise ValueError("config 'calibration' must be a mapping")

    board_keys = {"inner_corners_cols", "inner_corners_rows", "square_size_m"}
    calibration_keys = {
        "validation_ratio",
        "max_iterations",
        "epsilon",
        "mad_multiplier",
        "max_rejection_ratio",
        "minimum_training_views",
        "cv_folds",
        "max_outlier_iterations",
        "duplicate_distance_threshold",
        "coverage_grid_cols",
        "coverage_grid_rows",
    }
    calibration_runtime_keys = {
        "strict_resolution",
        "remove_duplicates",
        "sample_undistort_count",
    }
    _warn_unknown_keys("board", board_payload, board_keys)
    _warn_unknown_keys(
        "calibration",
        calibration_payload,
        calibration_keys | calibration_runtime_keys,
    )
    board_values = {key: board_payload[key] for key in board_keys if key in board_payload}
    calibration_values = {
        key: calibration_payload[key]
        for key in calibration_keys
        if key in calibration_payload
    }
    try:
        board = CheckerboardConfig(**board_values)
        calibration = CalibrationConfig(**calibration_values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid calibration config {config_path}: {exc}") from exc
    return board, calibration


def _warn_unknown_keys(
    section: str, payload: Mapping[str, Any], allowed: set[str]
) -> None:
    unknown = sorted(str(key) for key in payload if key not in allowed)
    if unknown:
        LOGGER.warning("Ignoring unknown %s config keys: %s", section, ", ".join(unknown))


def discover_images(pattern: str | Path | Sequence[str | Path]) -> list[Path]:
    """Expand one or more glob patterns and return unique paths in stable order.

    A literal directory means all common image files directly inside it.  An
    empty result is an error rather than a silent no-op.
    """

    patterns: Sequence[str | Path]
    if isinstance(pattern, (str, Path)):
        patterns = [pattern]
    else:
        patterns = pattern
    discovered: dict[str, Path] = {}
    image_suffixes = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}
    for raw_pattern in patterns:
        text = str(raw_pattern)
        literal = Path(text)
        if literal.is_dir():
            matches = [item for item in literal.iterdir() if item.is_file()]
        elif literal.is_file():
            matches = [literal]
        else:
            matches = [Path(item) for item in glob.glob(text, recursive=True)]
        for candidate in matches:
            if candidate.is_file() and candidate.suffix.lower() in image_suffixes:
                discovered[str(candidate)] = candidate
    result = sorted(discovered.values(), key=lambda value: str(value).casefold())
    if not result:
        rendered = ", ".join(str(value) for value in patterns)
        raise FileNotFoundError(f"no input images matched: {rendered}")
    return result


def create_object_points(board: CheckerboardConfig) -> np.ndarray:
    """Create row-major planar checkerboard points with metre units.

    The returned shape is ``(rows * columns, 3)`` and all Z coordinates are
    exactly zero.
    """

    object_points = np.zeros((board.point_count, 3), dtype=np.float32)
    object_points[:, :2] = (
        np.mgrid[0 : board.cols, 0 : board.rows]
        .T.reshape(-1, 2)
        .astype(np.float32)
        * np.float32(board.square_size_m)
    )
    return object_points


# Explicit alternate name used in a few calibration codebases.
generate_object_points = create_object_points


def detect_checkerboards(
    paths: Iterable[str | Path],
    board: CheckerboardConfig,
    strict_resolution: bool = False,
    *,
    corner_flags: int = DEFAULT_CORNER_FLAGS,
) -> tuple[list[DetectedView], tuple[int, int]]:
    """Read images and run strict full-board ``findChessboardCornersSB``.

    The first readable image establishes the reference resolution.  Resolution
    mismatches are retained as rejected metadata by default or raise immediately
    in strict mode.  The returned list accounts for corrupt files as well.
    """

    sorted_paths = sorted((Path(path) for path in paths), key=lambda p: str(p).casefold())
    if not sorted_paths:
        raise ValueError("no image paths were provided for checkerboard detection")

    views: list[DetectedView] = []
    image_size: tuple[int, int] | None = None
    for path in sorted_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None or image.size == 0:
            LOGGER.warning("Could not read image: %s", path)
            views.append(
                DetectedView(
                    path=path,
                    read_success=False,
                    detection_success=False,
                    exclusion_reason="image_read_failed_or_corrupt",
                )
            )
            continue
        height, width = image.shape[:2]
        current_size = (int(width), int(height))
        if image_size is None:
            image_size = current_size
        if current_size != image_size:
            message = (
                f"resolution mismatch for {path}: got {current_size[0]}x{current_size[1]}, "
                f"expected {image_size[0]}x{image_size[1]}"
            )
            if strict_resolution:
                raise ValueError(message)
            LOGGER.warning(message)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            views.append(
                DetectedView(
                    path=path,
                    width=width,
                    height=height,
                    read_success=True,
                    detection_success=False,
                    blur_score=_blur_score(gray),
                    exclusion_reason="resolution_mismatch",
                )
            )
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur_score = _blur_score(gray)
        try:
            found, corners = cv2.findChessboardCornersSB(
                gray, board.pattern_size, flags=int(corner_flags)
            )
        except cv2.error as exc:
            LOGGER.warning("OpenCV checkerboard detection failed for %s: %s", path, exc)
            views.append(
                DetectedView(
                    path=path,
                    width=width,
                    height=height,
                    read_success=True,
                    detection_success=False,
                    blur_score=blur_score,
                    exclusion_reason=f"opencv_detection_error: {exc}",
                )
            )
            continue

        valid, reason, normalized = _validate_detected_corners(
            bool(found), corners, board, current_size
        )
        if not valid or normalized is None:
            views.append(
                DetectedView(
                    path=path,
                    width=width,
                    height=height,
                    read_success=True,
                    detection_success=False,
                    corners=None,
                    corner_count=0 if corners is None else int(np.asarray(corners).shape[0]),
                    blur_score=blur_score,
                    exclusion_reason=reason,
                )
            )
            continue

        flat = normalized.reshape(-1, 2)
        center_array = np.mean(flat, axis=0)
        hull_area = abs(float(cv2.contourArea(cv2.convexHull(flat.astype(np.float32)))))
        area_ratio = hull_area / float(width * height)
        descriptor = compute_pose_descriptor(normalized, current_size, board)
        views.append(
            DetectedView(
                path=path,
                width=width,
                height=height,
                read_success=True,
                detection_success=True,
                corners=normalized,
                corner_count=normalized.shape[0],
                center=(float(center_array[0]), float(center_array[1])),
                area_ratio=float(area_ratio),
                blur_score=blur_score,
                pose_descriptor=descriptor,
            )
        )

    if image_size is None:
        raise ValueError("none of the input image files could be read")
    return views, image_size


def _blur_score(gray: np.ndarray) -> float:
    score = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    return score if math.isfinite(score) else math.nan


def _validate_detected_corners(
    found: bool,
    corners: np.ndarray | None,
    board: CheckerboardConfig,
    image_size: tuple[int, int],
) -> tuple[bool, str | None, np.ndarray | None]:
    if not found:
        return False, "checkerboard_not_found", None
    if corners is None:
        return False, "detector_returned_no_corners", None
    try:
        normalized = _as_image_points(corners)
    except ValueError as exc:
        return False, f"invalid_corner_shape: {exc}", None
    if normalized.shape[0] != board.point_count:
        return (
            False,
            f"incomplete_checkerboard: expected_{board.point_count}_got_{normalized.shape[0]}",
            None,
        )
    flat = normalized.reshape(-1, 2)
    if not np.all(np.isfinite(flat)):
        return False, "corner_coordinates_not_finite", None
    width, height = image_size
    inside = (
        (flat[:, 0] >= 0.0)
        & (flat[:, 0] < float(width))
        & (flat[:, 1] >= 0.0)
        & (flat[:, 1] < float(height))
    )
    if not bool(np.all(inside)):
        return False, "corner_coordinates_outside_image", None
    return True, None, normalized


def _as_image_points(points: np.ndarray) -> np.ndarray:
    array = np.asarray(points, dtype=np.float32)
    if array.ndim == 2 and array.shape[1] == 2:
        array = array.reshape(-1, 1, 2)
    if array.ndim != 3 or array.shape[1:] != (1, 2):
        raise ValueError(f"image points must have shape (N, 2) or (N, 1, 2), got {array.shape}")
    return np.ascontiguousarray(array, dtype=np.float32)


def compute_pose_descriptor(
    corners: np.ndarray,
    image_size: tuple[int, int],
    board: CheckerboardConfig,
) -> np.ndarray:
    """Describe checkerboard position, scale, rotation, and perspective.

    Angle is divided by pi.  Opposing side ratios preserve direction, while the
    final non-negative perspective term is the Euclidean magnitude of their log
    ratios.  This gives splitting code a useful scalar for foreshortening.
    """

    points = _as_image_points(corners).reshape(-1, 2).astype(np.float64)
    if points.shape[0] != board.point_count:
        raise ValueError(
            f"pose descriptor expected {board.point_count} corners, got {points.shape[0]}"
        )
    width, height = (int(image_size[0]), int(image_size[1]))
    if width <= 0 or height <= 0:
        raise ValueError("image_size must contain positive width and height")
    grid = points.reshape(board.rows, board.cols, 2)
    top_left = grid[0, 0]
    top_right = grid[0, -1]
    bottom_left = grid[-1, 0]
    bottom_right = grid[-1, -1]
    top_vector = top_right - top_left
    top_length = float(np.linalg.norm(top_vector))
    bottom_length = float(np.linalg.norm(bottom_right - bottom_left))
    left_length = float(np.linalg.norm(bottom_left - top_left))
    right_length = float(np.linalg.norm(bottom_right - top_right))
    epsilon = np.finfo(np.float64).eps
    if min(top_length, bottom_length, left_length, right_length) <= epsilon:
        raise ValueError("checkerboard outer edges have zero pixel length")

    center = np.mean(points, axis=0)
    hull = cv2.convexHull(points.astype(np.float32))
    area_ratio = abs(float(cv2.contourArea(hull))) / float(width * height)
    angle_normalized = math.atan2(float(top_vector[1]), float(top_vector[0])) / math.pi
    left_right_ratio = left_length / right_length
    top_bottom_ratio = top_length / bottom_length
    perspective = math.hypot(
        math.log(left_right_ratio), math.log(top_bottom_ratio)
    )
    descriptor = np.array(
        [
            center[0] / width,
            center[1] / height,
            area_ratio,
            angle_normalized,
            left_right_ratio,
            top_bottom_ratio,
            perspective,
        ],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(descriptor)):
        raise ValueError("computed pose descriptor contains NaN or Inf")
    return descriptor


def filter_accepted_views(views: Iterable[DetectedView]) -> list[DetectedView]:
    """Return calibration-eligible views while preserving input order."""

    return [view for view in views if view.accepted]


def compute_coverage(
    views: Iterable[DetectedView],
    image_size: tuple[int, int],
    grid_cols: int = 8,
    grid_rows: int = 6,
) -> np.ndarray:
    """Count observed corners in an image-space ``grid_rows x grid_cols`` map."""

    width, height = (int(image_size[0]), int(image_size[1]))
    if width <= 0 or height <= 0:
        raise ValueError("image_size must contain positive width and height")
    if grid_cols <= 0 or grid_rows <= 0:
        raise ValueError("coverage grid dimensions must be positive")
    counts = np.zeros((grid_rows, grid_cols), dtype=np.int64)
    for view in filter_accepted_views(views):
        assert view.corners is not None  # narrowed by accepted
        points = view.corners.reshape(-1, 2).astype(np.float64)
        if not np.all(np.isfinite(points)):
            raise ValueError(f"non-finite corners in accepted view {view.path}")
        columns = np.floor(points[:, 0] * grid_cols / width).astype(np.int64)
        rows = np.floor(points[:, 1] * grid_rows / height).astype(np.int64)
        columns = np.clip(columns, 0, grid_cols - 1)
        rows = np.clip(rows, 0, grid_rows - 1)
        np.add.at(counts, (rows, columns), 1)
    return counts


def coverage_warnings(counts: np.ndarray) -> list[str]:
    """Describe center-heavy or corner-poor checkerboard coverage."""

    array = np.asarray(counts)
    if array.ndim != 2 or array.size == 0:
        raise ValueError("coverage counts must be a non-empty 2-D array")
    warnings: list[str] = []
    total = int(np.sum(array))
    if total == 0:
        return ["No accepted checkerboard corners contribute to coverage."]
    rows, cols = array.shape
    r0, r1 = rows // 4, rows - rows // 4
    c0, c1 = cols // 4, cols - cols // 4
    center_fraction = float(np.sum(array[r0:r1, c0:c1])) / total
    center_area_fraction = ((r1 - r0) * (c1 - c0)) / float(rows * cols)
    if center_fraction > max(0.65, center_area_fraction * 1.8):
        warnings.append("Checkerboard observations are concentrated near the image center.")

    corner_h = max(1, rows // 3)
    corner_w = max(1, cols // 3)
    regions = {
        "top-left": array[:corner_h, :corner_w],
        "top-right": array[:corner_h, -corner_w:],
        "bottom-left": array[-corner_h:, :corner_w],
        "bottom-right": array[-corner_h:, -corner_w:],
    }
    expected_uniform = total * (corner_h * corner_w) / float(rows * cols)
    for name, region in regions.items():
        count = int(np.sum(region))
        if count == 0:
            warnings.append(f"No checkerboard corners were observed in the {name} region.")
        elif count < 0.2 * expected_uniform:
            warnings.append(f"Checkerboard corner coverage is sparse in the {name} region.")
    return warnings


def pose_distance(first: DetectedView, second: DetectedView) -> float:
    """Return a fixed-scale descriptor distance suitable for duplicate warnings."""

    features = _pose_features(np.vstack([_descriptor_for_view(first), _descriptor_for_view(second)]))
    return float(np.linalg.norm(features[0] - features[1]))


def mark_pose_duplicates(
    views: Iterable[DetectedView], threshold: float = 0.08
) -> list[tuple[Path, Path, float]]:
    """Mark near-identical consecutive accepted views without deleting them."""

    if threshold < 0 or not math.isfinite(float(threshold)):
        raise ValueError("duplicate threshold must be finite and non-negative")
    accepted = filter_accepted_views(views)
    duplicates: list[tuple[Path, Path, float]] = []
    for previous, current in zip(accepted, accepted[1:]):
        distance = pose_distance(previous, current)
        if distance <= threshold:
            current.duplicate_of = previous.path
            duplicates.append((current.path, previous.path, distance))
            LOGGER.warning(
                "Near-duplicate consecutive pose: %s resembles %s (distance %.5f)",
                current.path,
                previous.path,
                distance,
            )
    return duplicates


def remove_marked_duplicates(views: Iterable[DetectedView]) -> list[DetectedView]:
    """Optionally remove views marked by :func:`mark_pose_duplicates`."""

    return [view for view in filter_accepted_views(views) if view.duplicate_of is None]


def pose_diverse_split(
    views: Sequence[DetectedView],
    validation_ratio: float,
    seed: int,
    *,
    duplicate_threshold: float = 0.08,
) -> tuple[list[DetectedView], list[DetectedView]]:
    """Split views using grouped greedy farthest-point sampling.

    Consecutive near-duplicate views form an indivisible group, preventing one
    capture burst from leaking into both development subsets.  The RNG chooses
    the first farthest-point seed and resolves ties, making the split both seeded
    and exactly reproducible.
    """

    if not 0.0 <= float(validation_ratio) < 1.0:
        raise ValueError("validation_ratio must be in [0, 1)")
    accepted = filter_accepted_views(views)
    if validation_ratio == 0.0:
        return accepted, []
    if len(accepted) < 2:
        raise ValueError("at least two accepted views are required for a split")
    target = max(1, int(round(len(accepted) * float(validation_ratio))))
    target = min(target, len(accepted) - 1)

    descriptors = np.vstack([_descriptor_for_view(view) for view in accepted])
    features = _pose_features(descriptors)
    groups: list[list[int]] = [[0]]
    for index in range(1, len(accepted)):
        distance = float(np.linalg.norm(features[index] - features[index - 1]))
        if distance <= duplicate_threshold:
            groups[-1].append(index)
        else:
            groups.append([index])

    # A completely static sequence cannot be split without leakage.  Keep the
    # split usable, but make the unavoidable fallback explicit in logs.
    if len(groups) == 1:
        LOGGER.warning(
            "All accepted views form one near-duplicate group; splitting individual views."
        )
        groups = [[index] for index in range(len(accepted))]

    group_features = np.vstack([np.mean(features[group], axis=0) for group in groups])
    group_features = _range_normalize(group_features)
    rng = np.random.default_rng(int(seed))
    selected_groups: list[int] = []
    candidates = set(range(len(groups)))
    first = int(rng.integers(0, len(groups)))
    if len(groups[first]) < len(accepted):
        selected_groups.append(first)
        candidates.remove(first)

    while sum(len(groups[index]) for index in selected_groups) < target and candidates:
        if selected_groups:
            chosen_vectors = group_features[selected_groups]
            scored: list[tuple[float, float, int]] = []
            for candidate in candidates:
                minimum_distance = float(
                    np.min(np.linalg.norm(chosen_vectors - group_features[candidate], axis=1))
                )
                # The second component is deterministic for a given seed and
                # resolves geometric ties without filename bias.
                scored.append((minimum_distance, float(rng.random()), candidate))
            scored.sort(reverse=True)
            ordered = [entry[2] for entry in scored]
        else:
            ordered = list(candidates)
            rng.shuffle(ordered)

        chosen: int | None = None
        for candidate in ordered:
            selected_count = sum(len(groups[index]) for index in selected_groups)
            if selected_count + len(groups[candidate]) < len(accepted):
                chosen = candidate
                break
        if chosen is None:
            break
        selected_groups.append(chosen)
        candidates.remove(chosen)

    validation_indices = {
        index for group_index in selected_groups for index in groups[group_index]
    }
    if not validation_indices:
        validation_indices.add(int(rng.integers(0, len(accepted))))
    if len(validation_indices) == len(accepted):
        validation_indices.remove(max(validation_indices))
    training = [view for index, view in enumerate(accepted) if index not in validation_indices]
    validation = [view for index, view in enumerate(accepted) if index in validation_indices]
    return training, validation


def _descriptor_for_view(view: DetectedView) -> np.ndarray:
    if view.pose_descriptor is not None:
        return np.asarray(view.pose_descriptor, dtype=np.float64)
    if view.corners is None or view.width <= 0 or view.height <= 0:
        raise ValueError(f"accepted view lacks a pose descriptor and usable corners: {view.path}")
    points = view.corners.reshape(-1, 2).astype(np.float64)
    center = np.mean(points, axis=0)
    area = abs(float(cv2.contourArea(cv2.convexHull(points.astype(np.float32)))))
    rectangle = cv2.minAreaRect(points.astype(np.float32))
    (_, _), (side_a, side_b), angle_degrees = rectangle
    ratio = max(float(side_a), np.finfo(float).eps) / max(float(side_b), np.finfo(float).eps)
    return np.array(
        [
            center[0] / view.width,
            center[1] / view.height,
            area / (view.width * view.height),
            math.radians(float(angle_degrees)) / math.pi,
            ratio,
            1.0 / ratio,
            abs(math.log(max(ratio, np.finfo(float).eps))),
        ],
        dtype=np.float64,
    )


def _pose_features(descriptors: np.ndarray) -> np.ndarray:
    descriptors = np.asarray(descriptors, dtype=np.float64)
    angle = descriptors[:, 3] * math.pi
    ratios = np.maximum(descriptors[:, 4:6], np.finfo(np.float64).tiny)
    return np.column_stack(
        [
            descriptors[:, 0],
            descriptors[:, 1],
            np.sqrt(np.maximum(descriptors[:, 2], 0.0)),
            np.sin(angle),
            np.cos(angle),
            np.log(ratios[:, 0]),
            np.log(ratios[:, 1]),
            descriptors[:, 6],
        ]
    )


def _range_normalize(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float64)
    minimum = np.min(values, axis=0)
    span = np.max(values, axis=0) - minimum
    span[span <= np.finfo(np.float64).eps] = 1.0
    return (values - minimum) / span


def validate_point_sets(
    object_points: Sequence[np.ndarray],
    image_points: Sequence[np.ndarray],
    expected_points_per_view: int,
) -> None:
    """Validate list lengths, per-view point counts, shapes, and finiteness."""

    if len(object_points) != len(image_points):
        raise ValueError("object_points and image_points view counts do not match")
    if not object_points:
        raise ValueError("at least one point set is required")
    for index, (objects, images) in enumerate(zip(object_points, image_points, strict=True)):
        object_array = np.asarray(objects)
        image_array = _as_image_points(images)
        if object_array.shape != (expected_points_per_view, 3):
            raise ValueError(
                f"object_points[{index}] must have shape "
                f"({expected_points_per_view}, 3), got {object_array.shape}"
            )
        if image_array.shape[0] != expected_points_per_view:
            raise ValueError(
                f"image_points[{index}] contains {image_array.shape[0]} points; "
                f"expected {expected_points_per_view}"
            )
        if not np.all(np.isfinite(object_array)) or not np.all(np.isfinite(image_array)):
            raise ValueError(f"point set {index} contains NaN or Inf")


def calibration_point_sets(
    views: Iterable[DetectedView], board: CheckerboardConfig
) -> tuple[list[np.ndarray], list[np.ndarray], list[DetectedView]]:
    """Build validated object/image lists for accepted views."""

    accepted = filter_accepted_views(views)
    template = create_object_points(board)
    object_points = [template.copy() for _ in accepted]
    image_points: list[np.ndarray] = []
    for view in accepted:
        assert view.corners is not None
        image_points.append(np.ascontiguousarray(view.corners, dtype=np.float32))
    validate_point_sets(object_points, image_points, board.point_count)
    return object_points, image_points, accepted


def calibrate_rational(
    views: Sequence[DetectedView],
    board: CheckerboardConfig,
    image_size: tuple[int, int],
    config: CalibrationConfig,
) -> CalibrationResult:
    """Calibrate with ``calibrateCameraExtended`` and the fixed 8-term model."""

    width, height = (int(image_size[0]), int(image_size[1]))
    if width <= 0 or height <= 0:
        raise ValueError("image_size must contain positive width and height")
    object_points, image_points, accepted = calibration_point_sets(views, board)
    if len(accepted) < config.minimum_training_views:
        raise ValueError(
            f"Rational calibration needs at least {config.minimum_training_views} "
            f"accepted training views; got {len(accepted)}"
        )
    for view in accepted:
        if view.image_size != (width, height):
            raise ValueError(
                f"accepted view {view.path} has resolution {view.width}x{view.height}, "
                f"expected {width}x{height}"
            )

    camera_matrix_initial = cv2.initCameraMatrix2D(
        object_points, image_points, (width, height), aspectRatio=0
    )
    camera_matrix_initial = np.asarray(camera_matrix_initial, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(camera_matrix_initial)):
        raise RuntimeError("initCameraMatrix2D returned NaN or Inf")
    if camera_matrix_initial[0, 0] <= 0 or camera_matrix_initial[1, 1] <= 0:
        raise RuntimeError("initCameraMatrix2D returned a non-positive focal length")
    distortion_initial = np.zeros((8, 1), dtype=np.float64)
    try:
        result = cv2.calibrateCameraExtended(
            object_points,
            image_points,
            (width, height),
            camera_matrix_initial,
            distortion_initial,
            flags=config.calibration_flags,
            criteria=config.termination_criteria,
        )
    except cv2.error as exc:
        raise RuntimeError(f"OpenCV Rational calibration failed: {exc}") from exc
    if len(result) != 8:
        raise RuntimeError(
            f"calibrateCameraExtended returned {len(result)} values; expected 8"
        )
    (
        rms,
        camera_matrix,
        distortion,
        rvecs,
        tvecs,
        std_intrinsics,
        std_extrinsics,
        per_view_errors,
    ) = result
    flattened_distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
    if flattened_distortion.size < 8:
        raise RuntimeError(
            "CALIB_RATIONAL_MODEL returned fewer than eight distortion coefficients"
        )
    extras = flattened_distortion[8:].copy()
    warnings: list[str] = []
    if extras.size and not np.allclose(extras, 0.0, rtol=0.0, atol=1.0e-12):
        warning = (
            "OpenCV returned non-zero distortion terms after k6 even though thin-prism "
            "and tilted models were disabled: " + np.array2string(extras, precision=6)
        )
        LOGGER.warning(warning)
        warnings.append(warning)

    return CalibrationResult(
        rms=float(rms),
        camera_matrix=np.asarray(camera_matrix, dtype=np.float64),
        distortion_coefficients=flattened_distortion[:8],
        rvecs=list(rvecs),
        tvecs=list(tvecs),
        std_deviations_intrinsics=np.asarray(std_intrinsics, dtype=np.float64),
        std_deviations_extrinsics=np.asarray(std_extrinsics, dtype=np.float64),
        per_view_errors=np.asarray(per_view_errors, dtype=np.float64),
        views=accepted,
        image_size=(width, height),
        warnings=warnings,
        extra_distortion_coefficients=extras,
    )


def compute_mad_statistics(
    errors: Sequence[float] | np.ndarray, mad_multiplier: float = 3.0
) -> MadStatistics:
    """Compute the robust per-view RMS threshold ``median + m * 1.4826*MAD``."""

    values = np.asarray(errors, dtype=np.float64).reshape(-1)
    if values.size == 0:
        raise ValueError("cannot compute a MAD threshold from no errors")
    if not np.all(np.isfinite(values)):
        raise ValueError("per-view errors contain NaN or Inf")
    if not math.isfinite(float(mad_multiplier)) or mad_multiplier < 0:
        raise ValueError("mad_multiplier must be finite and non-negative")
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    robust_sigma = 1.4826 * mad
    return MadStatistics(
        median=median,
        mad=mad,
        robust_sigma=robust_sigma,
        threshold=median + float(mad_multiplier) * robust_sigma,
    )


def robust_mad_threshold(
    errors: Sequence[float] | np.ndarray, mad_multiplier: float = 3.0
) -> float:
    """Return only the robust MAD threshold."""

    return compute_mad_statistics(errors, mad_multiplier).threshold


# Clear aliases for tests/callers that use either common naming convention.
mad_outlier_threshold = robust_mad_threshold
calculate_mad_threshold = robust_mad_threshold


def reject_outlier_views(
    views: Sequence[DetectedView],
    board: CheckerboardConfig,
    image_size: tuple[int, int],
    config: CalibrationConfig,
    *,
    grid_cols: int | None = None,
    grid_rows: int | None = None,
    max_iterations: int | None = None,
) -> tuple[CalibrationResult, list[DetectedView], list[RejectedView]]:
    """Iteratively remove at most one worst MAD outlier per calibration run.

    Removal stops at three passes, the configured rejection ratio, the minimum
    training count, or a serious image-space coverage loss.  It never rejects a
    view merely because that view observes the image periphery.
    """

    retained = filter_accepted_views(views)
    initial_count = len(retained)
    result = calibrate_rational(retained, board, image_size, config)
    records: list[RejectedView] = []
    columns = config.coverage_grid_cols if grid_cols is None else int(grid_cols)
    rows = config.coverage_grid_rows if grid_rows is None else int(grid_rows)
    iteration_limit = (
        config.max_outlier_iterations if max_iterations is None else int(max_iterations)
    )
    iteration_limit = min(3, max(0, iteration_limit))
    maximum_rejections = int(math.floor(initial_count * config.max_rejection_ratio + 1e-12))

    for iteration in range(1, iteration_limit + 1):
        statistics = compute_mad_statistics(
            result.per_view_errors, config.mad_multiplier
        )
        worst_index = int(np.argmax(result.per_view_errors))
        worst_error = float(result.per_view_errors[worst_index])
        tolerance = np.finfo(np.float64).eps * max(1.0, abs(statistics.threshold))
        if worst_error <= statistics.threshold + tolerance:
            break
        if len(records) >= maximum_rejections:
            result.warnings.append(
                "MAD outlier remains, but max_rejection_ratio prevents another removal."
            )
            break
        if len(retained) - 1 < config.minimum_training_views:
            result.warnings.append(
                "MAD outlier remains, but minimum_training_views prevents removal."
            )
            break

        candidate = retained[worst_index]
        proposed = retained[:worst_index] + retained[worst_index + 1 :]
        impact = _coverage_impact(
            retained, proposed, image_size, grid_cols=columns, grid_rows=rows
        )
        if bool(impact["serious_loss"]):
            result.warnings.append(
                f"MAD outlier {candidate.path} retained because removal would cause "
                "serious coverage loss."
            )
            break

        records.append(
            RejectedView(
                path=candidate.path,
                per_view_rms=worst_error,
                robust_threshold=statistics.threshold,
                iteration=iteration,
                coverage_impact=impact,
            )
        )
        retained = proposed
        result = calibrate_rational(retained, board, image_size, config)
    return result, retained, records


def _coverage_impact(
    before_views: Sequence[DetectedView],
    after_views: Sequence[DetectedView],
    image_size: tuple[int, int],
    *,
    grid_cols: int,
    grid_rows: int,
) -> dict[str, float | int | bool]:
    before = compute_coverage(before_views, image_size, grid_cols, grid_rows)
    after = compute_coverage(after_views, image_size, grid_cols, grid_rows)
    occupied_before = int(np.count_nonzero(before))
    occupied_after = int(np.count_nonzero(after))
    lost_cells = max(0, occupied_before - occupied_after)
    lost_fraction = lost_cells / max(1, occupied_before)

    corner_h = max(1, grid_rows // 3)
    corner_w = max(1, grid_cols // 3)
    slices = (
        (slice(0, corner_h), slice(0, corner_w)),
        (slice(0, corner_h), slice(grid_cols - corner_w, grid_cols)),
        (slice(grid_rows - corner_h, grid_rows), slice(0, corner_w)),
        (
            slice(grid_rows - corner_h, grid_rows),
            slice(grid_cols - corner_w, grid_cols),
        ),
    )
    corners_lost = sum(
        int(np.sum(before[row_slice, col_slice]) > 0 and np.sum(after[row_slice, col_slice]) == 0)
        for row_slice, col_slice in slices
    )
    serious_loss = bool(lost_fraction > 0.10 or corners_lost > 0)
    return {
        "occupied_cells_before": occupied_before,
        "occupied_cells_after": occupied_after,
        "lost_cells": lost_cells,
        "lost_cell_fraction": float(lost_fraction),
        "corner_regions_lost": int(corners_lost),
        "serious_loss": serious_loss,
    }


def camera_info_dict(
    camera_name: str,
    image_size: tuple[int, int],
    camera_matrix: np.ndarray | CalibrationResult,
    distortion_coefficients: np.ndarray | Sequence[float] | None = None,
) -> dict[str, Any]:
    """Build a ROS-compatible monocular CameraInfo YAML mapping.

    The projection matrix is the original K extended to 3x4.  It is not an
    alpha-dependent rectified/new camera matrix used only for visualization.
    """

    if isinstance(camera_matrix, CalibrationResult):
        if distortion_coefficients is not None:
            raise ValueError("do not supply D separately when passing CalibrationResult")
        distortion_coefficients = camera_matrix.distortion_coefficients
        matrix = camera_matrix.camera_matrix
    else:
        matrix = np.asarray(camera_matrix, dtype=np.float64)
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f"camera_matrix must have shape (3, 3), got {matrix.shape}")
    if distortion_coefficients is None:
        raise ValueError("distortion_coefficients are required")
    distortion = np.asarray(distortion_coefficients, dtype=np.float64).reshape(-1)
    if distortion.size != 8:
        raise ValueError(
            "camera_info Rational distortion vector must contain exactly 8 values "
            "in [k1,k2,p1,p2,k3,k4,k5,k6] order"
        )
    width, height = (int(image_size[0]), int(image_size[1]))
    if width <= 0 or height <= 0:
        raise ValueError("image_size must contain positive width and height")
    if not camera_name or not str(camera_name).strip():
        raise ValueError("camera_name cannot be empty")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion)):
        raise ValueError("camera_info K and D must contain only finite values")

    projection = np.zeros((3, 4), dtype=np.float64)
    projection[:, :3] = matrix
    rectification = np.eye(3, dtype=np.float64)
    return {
        "image_width": width,
        "image_height": height,
        "camera_name": str(camera_name),
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [float(value) for value in matrix.reshape(-1)],
        },
        "distortion_model": "rational_polynomial",
        "distortion_coefficients": {
            "rows": 1,
            "cols": 8,
            "data": [float(value) for value in distortion],
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [float(value) for value in rectification.reshape(-1)],
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [float(value) for value in projection.reshape(-1)],
        },
    }


def write_camera_info_yaml(
    output_path: str | Path,
    camera_name: str,
    image_size: tuple[int, int],
    camera_matrix: np.ndarray | CalibrationResult,
    distortion_coefficients: np.ndarray | Sequence[float] | None = None,
) -> dict[str, Any]:
    """Write and return a ROS 2 ``rational_polynomial`` CameraInfo mapping."""

    payload = camera_info_dict(
        camera_name,
        image_size,
        camera_matrix,
        distortion_coefficients,
    )
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if yaml is not None:
        rendered = yaml.safe_dump(payload, sort_keys=False, default_flow_style=False)
    else:  # JSON is valid YAML 1.2 and retains all required ROS fields.
        rendered = json.dumps(payload, indent=2) + "\n"
    path.write_text(rendered, encoding="utf-8")
    return payload


__all__ = [
    "CalibrationConfig",
    "CalibrationResult",
    "CheckerboardConfig",
    "DEFAULT_CORNER_FLAGS",
    "DetectedView",
    "MadStatistics",
    "POSE_DESCRIPTOR_NAMES",
    "RATIONAL_CALIBRATION_FLAGS",
    "RATIONAL_COEFFICIENT_NAMES",
    "RejectedView",
    "ValidationResult",
    "calculate_mad_threshold",
    "calibrate_rational",
    "calibration_point_sets",
    "camera_info_dict",
    "compute_coverage",
    "compute_mad_statistics",
    "compute_pose_descriptor",
    "coverage_warnings",
    "create_object_points",
    "detect_checkerboards",
    "discover_images",
    "filter_accepted_views",
    "generate_object_points",
    "load_config",
    "mad_outlier_threshold",
    "mark_pose_duplicates",
    "pose_distance",
    "pose_diverse_split",
    "reject_outlier_views",
    "remove_marked_duplicates",
    "robust_mad_threshold",
    "validate_point_sets",
    "write_camera_info_yaml",
]
