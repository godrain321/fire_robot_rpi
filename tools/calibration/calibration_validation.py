"""Validation and numerical quality checks for Rational camera calibration.

This module deliberately implements only OpenCV's eight-coefficient Rational
Polynomial model, in the order ``k1, k2, p1, p2, k3, k4, k5, k6``.  The
functions do not depend on ROS and need only OpenCV, NumPy, and the Python
standard library.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

import cv2
import numpy as np


RATIONAL_PARAMETER_NAMES: tuple[str, ...] = (
    "k1",
    "k2",
    "p1",
    "p2",
    "k3",
    "k4",
    "k5",
    "k6",
)
INTRINSIC_PARAMETER_NAMES: tuple[str, ...] = ("fx", "fy", "cx", "cy")
ALL_PARAMETER_NAMES: tuple[str, ...] = (
    *INTRINSIC_PARAMETER_NAMES,
    *RATIONAL_PARAMETER_NAMES,
)


@dataclass
class ValidationResult:
    """Fixed-intrinsics validation results and overlay-ready point arrays.

    ``per_view`` and ``per_corner`` contain only JSON/CSV-friendly scalar
    values.  The NumPy dictionaries preserve the exact points for plotting.
    Keys are full image paths so files with the same basename remain distinct.
    """

    summary: dict[str, Any]
    per_view: list[dict[str, Any]]
    per_corner: list[dict[str, Any]]
    projected_points: dict[str, np.ndarray] = field(default_factory=dict)
    detected_points: dict[str, np.ndarray] = field(default_factory=dict)
    errors_by_view: dict[str, np.ndarray] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)

    @property
    def validation_rms(self) -> float | None:
        """Return the aggregate validation RMS in pixels."""

        value = self.summary.get("validation_rms")
        return None if value is None else float(value)

    @property
    def metrics(self) -> dict[str, Any]:
        """Compatibility alias for callers that refer to summary metrics."""

        return self.summary


def _get_value(item: Any, *names: str, default: Any = None) -> Any:
    """Read the first available attribute or mapping key from ``item``."""

    for name in names:
        if isinstance(item, Mapping) and name in item:
            return item[name]
        if hasattr(item, name):
            return getattr(item, name)
    return default


def _view_path(view: Any, index: int) -> Path:
    value = _get_value(view, "path", "image_path", "file_path")
    return Path(value) if value is not None else Path(f"view_{index:04d}")


def _view_points(view: Any) -> np.ndarray:
    value = _get_value(view, "corners", "image_points", "points")
    if value is None:
        raise ValueError("validation view has no detected checkerboard corners")
    points = np.asarray(value, dtype=np.float64).reshape(-1, 2)
    if points.size == 0 or not np.all(np.isfinite(points)):
        raise ValueError("validation image points must be non-empty and finite")
    return points


def _camera_matrix(camera_matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(camera_matrix, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f"camera matrix must have shape (3, 3), got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("camera matrix contains NaN or Inf")
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("camera focal lengths fx and fy must be positive")
    if abs(float(np.linalg.det(matrix))) < np.finfo(np.float64).eps:
        raise ValueError("camera matrix is singular")
    return matrix.copy()


def _rational_coefficients(distortion: np.ndarray) -> np.ndarray:
    coefficients = np.asarray(distortion, dtype=np.float64).reshape(-1)
    if coefficients.size < 8:
        raise ValueError(
            "Rational Polynomial validation requires at least eight distortion "
            f"coefficients, got {coefficients.size}"
        )
    if not np.all(np.isfinite(coefficients)):
        raise ValueError("distortion coefficients contain NaN or Inf")
    if coefficients.size > 8 and np.any(np.abs(coefficients[8:]) > 1.0e-12):
        raise ValueError(
            "non-zero coefficients after k6 indicate an unsupported thin-prism "
            "or tilted model"
        )
    return coefficients[:8].copy()


def _image_size(image_size: Sequence[int]) -> tuple[int, int]:
    if len(image_size) != 2:
        raise ValueError("image_size must be (width, height)")
    width, height = int(image_size[0]), int(image_size[1])
    if width <= 0 or height <= 0:
        raise ValueError("image width and height must be positive")
    return width, height


def _expand_object_points(
    object_points: np.ndarray | Sequence[np.ndarray], view_count: int
) -> list[np.ndarray]:
    """Return one validated ``(N, 3)`` object-point array per view."""

    if isinstance(object_points, np.ndarray):
        array = np.asarray(object_points, dtype=np.float32)
        if array.ndim == 2:
            arrays = [array.copy() for _ in range(view_count)]
        elif array.ndim == 3 and array.shape[0] == view_count:
            arrays = [array[index].copy() for index in range(view_count)]
        else:
            raise ValueError(
                "object_points must have shape (N, 3) or (views, N, 3)"
            )
    else:
        source = list(object_points)
        if len(source) == 1 and view_count > 1:
            arrays = [np.asarray(source[0], dtype=np.float32).copy() for _ in range(view_count)]
        elif len(source) == view_count:
            arrays = [np.asarray(points, dtype=np.float32).copy() for points in source]
        else:
            raise ValueError(
                "object_points sequence length must equal the number of views"
            )

    validated: list[np.ndarray] = []
    for points in arrays:
        reshaped = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        if reshaped.size == 0 or not np.all(np.isfinite(reshaped)):
            raise ValueError("object points must be non-empty and finite")
        validated.append(reshaped)
    return validated


def _rms(errors: np.ndarray) -> float | None:
    values = np.asarray(errors, dtype=np.float64).reshape(-1)
    if values.size == 0:
        return None
    return float(np.sqrt(np.mean(np.square(values))))


def _error_statistics(errors: np.ndarray) -> dict[str, float | int | None]:
    values = np.asarray(errors, dtype=np.float64).reshape(-1)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {
            "count": 0,
            "rms": None,
            "mean": None,
            "median": None,
            "standard_deviation": None,
            "p90": None,
            "p95": None,
            "p99": None,
            "maximum": None,
        }
    return {
        "count": int(values.size),
        "rms": _rms(values),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "standard_deviation": float(np.std(values)),
        "p90": float(np.percentile(values, 90.0)),
        "p95": float(np.percentile(values, 95.0)),
        "p99": float(np.percentile(values, 99.0)),
        "maximum": float(np.max(values)),
    }


def validate_fixed_intrinsics(
    views: Sequence[Any],
    object_points: np.ndarray | Sequence[np.ndarray],
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    image_size: Sequence[int],
) -> ValidationResult:
    """Evaluate held-out views while keeping the Rational ``K`` and ``D`` fixed.

    A pose is estimated independently for each view with
    :func:`cv2.solvePnP`, followed by projection with
    :func:`cv2.projectPoints`.  Errors are Euclidean pixel distances.  The edge
    region is everything outside the central 25--75 percent rectangle; the
    four-corner region is the subset that lies in both an outer horizontal and
    outer vertical quarter.
    """

    view_list = list(views)
    if not view_list:
        raise ValueError("at least one validation view is required")
    width, height = _image_size(image_size)
    matrix = _camera_matrix(camera_matrix)
    coefficients = _rational_coefficients(distortion_coefficients)
    objects = _expand_object_points(object_points, len(view_list))

    per_view: list[dict[str, Any]] = []
    per_corner: list[dict[str, Any]] = []
    projected_by_view: dict[str, np.ndarray] = {}
    detected_by_view: dict[str, np.ndarray] = {}
    errors_by_view: dict[str, np.ndarray] = {}
    all_errors: list[np.ndarray] = []
    center_errors: list[float] = []
    edge_errors: list[float] = []
    corner_region_errors: list[float] = []
    warnings: list[str] = []

    for view_index, (view, object_array) in enumerate(zip(view_list, objects)):
        path = _view_path(view, view_index)
        path_key = str(path)
        try:
            detected = _view_points(view)
            if detected.shape[0] != object_array.shape[0]:
                raise ValueError(
                    f"point count mismatch: {object_array.shape[0]} object points "
                    f"and {detected.shape[0]} image points"
                )
            success, rotation, translation = cv2.solvePnP(
                object_array,
                detected.reshape(-1, 1, 2),
                matrix,
                coefficients.reshape(-1, 1),
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not success:
                raise RuntimeError("cv2.solvePnP returned false")
            projected, _ = cv2.projectPoints(
                object_array,
                rotation,
                translation,
                matrix,
                coefficients.reshape(-1, 1),
            )
            projected = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
            if not np.all(np.isfinite(projected)):
                raise RuntimeError("cv2.projectPoints produced NaN or Inf")
            errors = np.linalg.norm(detected - projected, axis=1)
            view_rms = _rms(errors)

            projected_by_view[path_key] = projected
            detected_by_view[path_key] = detected
            errors_by_view[path_key] = errors
            all_errors.append(errors)

            for corner_index, (actual, estimate, error) in enumerate(
                zip(detected, projected, errors)
            ):
                x, y = float(actual[0]), float(actual[1])
                in_center = (
                    0.25 * width <= x <= 0.75 * width
                    and 0.25 * height <= y <= 0.75 * height
                )
                in_corner = (
                    (x < 0.25 * width or x > 0.75 * width)
                    and (y < 0.25 * height or y > 0.75 * height)
                )
                region = "center" if in_center else ("corner" if in_corner else "edge")
                if in_center:
                    center_errors.append(float(error))
                else:
                    edge_errors.append(float(error))
                if in_corner:
                    corner_region_errors.append(float(error))
                per_corner.append(
                    {
                        "view_index": view_index,
                        "path": path_key,
                        "corner_index": corner_index,
                        "detected_x": x,
                        "detected_y": y,
                        "projected_x": float(estimate[0]),
                        "projected_y": float(estimate[1]),
                        "error_px": float(error),
                        "region": region,
                        "is_edge_region": not in_center,
                        "is_four_corner_region": in_corner,
                    }
                )

            per_view.append(
                {
                    "view_index": view_index,
                    "path": path_key,
                    "status": "ok",
                    "corner_count": int(errors.size),
                    "rms_px": view_rms,
                    "mean_px": float(np.mean(errors)),
                    "median_px": float(np.median(errors)),
                    "p95_px": float(np.percentile(errors, 95.0)),
                    "maximum_px": float(np.max(errors)),
                    "rvec": np.asarray(rotation, dtype=np.float64).reshape(-1).tolist(),
                    "tvec": np.asarray(translation, dtype=np.float64).reshape(-1).tolist(),
                    "error": "",
                }
            )
        except (cv2.error, ValueError, RuntimeError) as exc:
            message = f"{path}: validation pose failed: {exc}"
            warnings.append(message)
            per_view.append(
                {
                    "view_index": view_index,
                    "path": path_key,
                    "status": "failed",
                    "corner_count": 0,
                    "rms_px": None,
                    "mean_px": None,
                    "median_px": None,
                    "p95_px": None,
                    "maximum_px": None,
                    "rvec": None,
                    "tvec": None,
                    "error": str(exc),
                }
            )

    if not all_errors:
        raise RuntimeError("pose estimation failed for every validation view")

    aggregate = np.concatenate(all_errors)
    statistics = _error_statistics(aggregate)
    summary: dict[str, Any] = {
        "quality_status": "passed" if not warnings else "warning",
        "image_width": width,
        "image_height": height,
        "view_count": len(view_list),
        "successful_view_count": len(all_errors),
        "failed_view_count": len(view_list) - len(all_errors),
        "corner_count": int(aggregate.size),
        "validation_rms": statistics["rms"],
        "mean": statistics["mean"],
        "median": statistics["median"],
        "standard_deviation": statistics["standard_deviation"],
        "p90": statistics["p90"],
        "p95": statistics["p95"],
        "p99": statistics["p99"],
        "maximum": statistics["maximum"],
        "center_region_rms": _rms(np.asarray(center_errors)),
        "center_region_corner_count": len(center_errors),
        "edge_region_rms": _rms(np.asarray(edge_errors)),
        "edge_region_corner_count": len(edge_errors),
        "four_corner_region_rms": _rms(np.asarray(corner_region_errors)),
        "four_corner_region_corner_count": len(corner_region_errors),
        "warnings": warnings,
    }
    return ValidationResult(
        summary=summary,
        per_view=per_view,
        per_corner=per_corner,
        projected_points=projected_by_view,
        detected_points=detected_by_view,
        errors_by_view=errors_by_view,
        warnings=warnings,
    )


def _finite_or_none(value: Any) -> Any:
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(float(value)) else None
    return value


def _json_ready(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [_json_ready(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return _finite_or_none(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    if isinstance(value, float):
        return _finite_or_none(value)
    return value


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fields: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _json_ready(row.get(key)) for key in fields})


def write_validation_outputs(
    result: ValidationResult,
    output_dir: Path | str,
    *,
    views: Sequence[Any] | None = None,
    image_size: Sequence[int] | None = None,
) -> dict[str, Path]:
    """Write validation JSON, CSV tables, plots, and optional view overlays."""

    try:
        from .calibration_visualization import (
            save_error_heatmap,
            save_error_histogram,
            save_validation_overlay,
        )
    except ImportError:  # Direct execution with tools/calibration on sys.path.
        from calibration_visualization import (  # type: ignore
            save_error_heatmap,
            save_error_histogram,
            save_validation_overlay,
        )

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    summary_path = destination / "validation_summary.json"
    per_view_path = destination / "validation_per_view.csv"
    per_corner_path = destination / "validation_per_corner.csv"
    histogram_path = destination / "validation_error_histogram.png"
    heatmap_path = destination / "validation_error_heatmap.png"

    summary_path.write_text(
        json.dumps(_json_ready(result.summary), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    _write_csv(
        per_view_path,
        result.per_view,
        (
            "view_index",
            "path",
            "status",
            "corner_count",
            "rms_px",
            "mean_px",
            "median_px",
            "p95_px",
            "maximum_px",
            "rvec",
            "tvec",
            "error",
        ),
    )
    _write_csv(
        per_corner_path,
        result.per_corner,
        (
            "view_index",
            "path",
            "corner_index",
            "detected_x",
            "detected_y",
            "projected_x",
            "projected_y",
            "error_px",
            "region",
            "is_edge_region",
            "is_four_corner_region",
        ),
    )

    all_errors = (
        np.concatenate(list(result.errors_by_view.values()))
        if result.errors_by_view
        else np.empty(0, dtype=np.float64)
    )
    save_error_histogram(all_errors, histogram_path)
    points = np.asarray(
        [[row["detected_x"], row["detected_y"]] for row in result.per_corner],
        dtype=np.float64,
    ).reshape(-1, 2)
    errors = np.asarray(
        [row["error_px"] for row in result.per_corner], dtype=np.float64
    )
    size = image_size or (
        int(result.summary["image_width"]),
        int(result.summary["image_height"]),
    )
    save_error_heatmap(points, errors, size, heatmap_path)

    if views is not None:
        overlay_dir = destination / "validation_overlays"
        overlay_dir.mkdir(parents=True, exist_ok=True)
        for index, view in enumerate(views):
            path = _view_path(view, index)
            key = str(path)
            if key not in result.projected_points:
                continue
            rms = _rms(result.errors_by_view[key]) or 0.0
            overlay_path = overlay_dir / f"{index:04d}_{path.stem}.png"
            save_validation_overlay(
                path,
                result.detected_points[key],
                result.projected_points[key],
                rms,
                overlay_path,
            )

    return {
        "summary": summary_path,
        "per_view": per_view_path,
        "per_corner": per_corner_path,
        "histogram": histogram_path,
        "heatmap": heatmap_path,
    }


def check_rational_stability(
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    image_size: Sequence[int],
    *,
    pixel_grid: tuple[int, int] = (81, 61),
    radial_sample_count: int = 2048,
    denominator_epsilon: float = 1.0e-3,
    maximum_amplification: float = 100.0,
) -> dict[str, Any]:
    """Check the Rational denominator and radial mapping over the full image.

    Pixel samples are transformed with the inverse camera matrix.  The radial
    curve is then sampled continuously from the optical axis to the largest
    normalized radius in the image.  Tangential terms are intentionally not
    included in this radial sanity check.
    """

    matrix = _camera_matrix(camera_matrix)
    coefficients = _rational_coefficients(distortion_coefficients)
    width, height = _image_size(image_size)
    grid_width, grid_height = int(pixel_grid[0]), int(pixel_grid[1])
    if grid_width < 2 or grid_height < 2:
        raise ValueError("pixel_grid dimensions must both be at least two")
    if radial_sample_count < 16:
        raise ValueError("radial_sample_count must be at least 16")
    if denominator_epsilon <= 0.0 or maximum_amplification <= 0.0:
        raise ValueError("stability thresholds must be positive")

    u, v = np.meshgrid(
        np.linspace(0.0, float(width - 1), grid_width),
        np.linspace(0.0, float(height - 1), grid_height),
    )
    pixels = np.stack((u.reshape(-1), v.reshape(-1), np.ones(u.size)), axis=0)
    normalized_h = np.linalg.inv(matrix) @ pixels
    normalized = normalized_h[:2] / normalized_h[2:3]
    grid_radii = np.hypot(normalized[0], normalized[1])
    maximum_radius = float(np.max(grid_radii))

    radius = np.linspace(0.0, maximum_radius, radial_sample_count, dtype=np.float64)
    radius2 = np.square(radius)
    radius4 = np.square(radius2)
    radius6 = radius4 * radius2
    k1, k2, _p1, _p2, k3, k4, k5, k6 = coefficients
    numerator = 1.0 + k1 * radius2 + k2 * radius4 + k3 * radius6
    denominator = 1.0 + k4 * radius2 + k5 * radius4 + k6 * radius6
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        amplification = numerator / denominator
        mapped_radius = radius * amplification
        derivative = np.gradient(mapped_radius, radius, edge_order=2)

    grid_r2 = np.square(grid_radii)
    grid_denominator = 1.0 + k4 * grid_r2 + k5 * np.square(grid_r2) + k6 * grid_r2**3
    finite = bool(
        np.all(np.isfinite(grid_denominator))
        and np.all(np.isfinite(numerator))
        and np.all(np.isfinite(denominator))
        and np.all(np.isfinite(amplification))
        and np.all(np.isfinite(mapped_radius))
        and np.all(np.isfinite(derivative))
    )
    absolute_denominator = np.abs(denominator)
    minimum_denominator_abs = float(np.nanmin(absolute_denominator))
    near_zero = absolute_denominator <= denominator_epsilon
    sign_change = bool(np.any(denominator <= 0.0))
    non_monotonic = np.asarray(derivative <= 0.0)
    finite_amplification = np.abs(amplification[np.isfinite(amplification)])
    maximum_abs_amplification = (
        float(np.max(finite_amplification)) if finite_amplification.size else math.inf
    )
    diverges = bool(
        not finite
        or maximum_abs_amplification > maximum_amplification
        or np.any(np.abs(mapped_radius[np.isfinite(mapped_radius)]) > maximum_amplification * max(maximum_radius, 1.0))
    )

    issues: list[str] = []
    if not finite:
        issues.append("NaN or Inf occurs in the Rational radial mapping")
    if np.any(near_zero):
        issues.append(
            "Rational denominator approaches zero inside the image domain"
        )
    if sign_change:
        issues.append("Rational denominator changes sign inside the image domain")
    if diverges:
        issues.append("Rational radial mapping diverges inside the image domain")
    if np.any(non_monotonic):
        issues.append("Rational radial mapping is non-monotonic inside the image domain")

    valid_derivative = derivative[np.isfinite(derivative)]
    result: dict[str, Any] = {
        "quality_status": "passed" if not issues else "failed_quality_check",
        "stable": not issues,
        "image_width": width,
        "image_height": height,
        "pixel_grid_columns": grid_width,
        "pixel_grid_rows": grid_height,
        "maximum_normalized_radius": maximum_radius,
        "denominator_epsilon": denominator_epsilon,
        "denominator_minimum": float(np.nanmin(denominator)),
        "denominator_maximum": float(np.nanmax(denominator)),
        "denominator_minimum_absolute": minimum_denominator_abs,
        "denominator_near_zero_count": int(np.count_nonzero(near_zero)),
        "denominator_sign_change": sign_change,
        "all_values_finite": finite,
        "maximum_absolute_radial_amplification": _finite_or_none(maximum_abs_amplification),
        "radial_mapping_diverges": diverges,
        "minimum_radial_derivative": (
            float(np.min(valid_derivative)) if valid_derivative.size else None
        ),
        "non_monotonic_sample_count": int(np.count_nonzero(non_monotonic)),
        "issues": issues,
        "coefficients": dict(zip(RATIONAL_PARAMETER_NAMES, coefficients.tolist())),
        "samples": {
            "radius": radius.tolist(),
            "mapped_radius": mapped_radius.tolist(),
            "radial_amplification": amplification.tolist(),
            "denominator": denominator.tolist(),
            "derivative": derivative.tolist(),
        },
    }
    return result


def write_rational_stability_outputs(
    stability: Mapping[str, Any], output_dir: Path | str
) -> dict[str, Path]:
    """Write ``rational_stability.json`` and ``rational_radial_curve.png``."""

    try:
        from .calibration_visualization import save_radial_curve
    except ImportError:  # Direct script imports.
        from calibration_visualization import save_radial_curve  # type: ignore

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    json_path = destination / "rational_stability.json"
    plot_path = destination / "rational_radial_curve.png"
    json_path.write_text(
        json.dumps(_json_ready(stability), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    save_radial_curve(stability, plot_path)
    return {"json": json_path, "plot": plot_path}


def _config_value(config: Any, name: str, default: Any) -> Any:
    if isinstance(config, Mapping):
        if name in config:
            return config[name]
        nested = config.get("calibration")
        if isinstance(nested, Mapping) and name in nested:
            return nested[name]
    if hasattr(config, name):
        return getattr(config, name)
    return default


def _descriptor_for_view(view: Any, image_size: tuple[int, int]) -> np.ndarray:
    descriptor = _get_value(view, "pose_descriptor", "descriptor")
    if descriptor is not None:
        values = np.asarray(descriptor, dtype=np.float64).reshape(-1)
        if values.size:
            return values

    points = _view_points(view)
    width, height = image_size
    center = np.mean(points, axis=0)
    hull = cv2.convexHull(points.astype(np.float32))
    area_ratio = float(cv2.contourArea(hull)) / float(width * height)
    extent = np.ptp(points, axis=0)
    covariance = np.cov(points.T) if points.shape[0] > 1 else np.eye(2)
    eigenvalues = np.linalg.eigvalsh(covariance)
    anisotropy = float(eigenvalues[-1] / max(eigenvalues[0], 1.0e-12))
    return np.asarray(
        [
            center[0] / width,
            center[1] / height,
            area_ratio,
            extent[0] / width,
            extent[1] / height,
            math.log1p(anisotropy),
        ],
        dtype=np.float64,
    )


def _normalise_descriptors(descriptors: Sequence[np.ndarray]) -> np.ndarray:
    dimension = max(values.size for values in descriptors)
    matrix = np.full((len(descriptors), dimension), np.nan, dtype=np.float64)
    for index, values in enumerate(descriptors):
        matrix[index, : values.size] = values
    for column_index in range(dimension):
        column = matrix[:, column_index]
        finite = np.isfinite(column)
        fill = float(np.median(column[finite])) if np.any(finite) else 0.0
        column[~finite] = fill
        matrix[:, column_index] = column
    minimum = np.min(matrix, axis=0)
    span = np.ptp(matrix, axis=0)
    span[span < 1.0e-12] = 1.0
    return (matrix - minimum) / span


def _duplicate_groups(views: Sequence[Any], descriptors: np.ndarray) -> list[list[int]]:
    """Group near-identical poses so no duplicate group crosses CV folds."""

    count = len(views)
    parent = list(range(count))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    paths = {str(_view_path(view, index)): index for index, view in enumerate(views)}
    basenames = {_view_path(view, index).name: index for index, view in enumerate(views)}
    for index, view in enumerate(views):
        duplicate_of = _get_value(view, "duplicate_of")
        if duplicate_of:
            match = paths.get(str(duplicate_of), basenames.get(Path(duplicate_of).name))
            if match is not None:
                union(index, match)

    threshold = 0.06 * math.sqrt(max(1, descriptors.shape[1]))
    for left in range(count):
        for right in range(left + 1, count):
            if float(np.linalg.norm(descriptors[left] - descriptors[right])) <= threshold:
                union(left, right)

    grouped: dict[int, list[int]] = {}
    for index in range(count):
        grouped.setdefault(find(index), []).append(index)
    return sorted(grouped.values(), key=lambda group: group[0])


def _farthest_group_order(
    centroids: np.ndarray, seed: int
) -> list[int]:
    generator = np.random.default_rng(seed)
    first = int(generator.integers(0, len(centroids)))
    selected = [first]
    remaining = set(range(len(centroids))) - {first}
    nearest = np.linalg.norm(centroids - centroids[first], axis=1)
    while remaining:
        farthest_distance = max(float(nearest[index]) for index in remaining)
        tied = sorted(
            index
            for index in remaining
            if abs(float(nearest[index]) - farthest_distance) <= 1.0e-12
        )
        chosen = tied[int(generator.integers(0, len(tied)))]
        selected.append(chosen)
        remaining.remove(chosen)
        nearest = np.minimum(
            nearest, np.linalg.norm(centroids - centroids[chosen], axis=1)
        )
    return selected


def _pose_group_folds(
    views: Sequence[Any], image_size: tuple[int, int], folds: int, seed: int
) -> tuple[list[list[int]], list[int]]:
    descriptors = _normalise_descriptors(
        [_descriptor_for_view(view, image_size) for view in views]
    )
    groups = _duplicate_groups(views, descriptors)
    if len(groups) < 2:
        raise ValueError("cross-validation needs at least two distinct pose groups")
    fold_count = min(folds, len(groups))
    centroids = np.asarray(
        [np.mean(descriptors[group], axis=0) for group in groups], dtype=np.float64
    )
    order = _farthest_group_order(centroids, seed)
    assignments: list[list[int]] = [[] for _ in range(fold_count)]
    sizes = [0] * fold_count
    for position, group_index in enumerate(order):
        minimum_size = min(sizes)
        candidates = [index for index, size in enumerate(sizes) if size == minimum_size]
        fold_index = candidates[position % len(candidates)]
        assignments[fold_index].extend(groups[group_index])
        sizes[fold_index] += len(groups[group_index])
    return [sorted(indices) for indices in assignments], [len(group) for group in groups]


def _calibrate_arrays(
    objects: Sequence[np.ndarray],
    images: Sequence[np.ndarray],
    image_size: tuple[int, int],
    max_iterations: int,
    epsilon: float,
) -> tuple[float, np.ndarray, np.ndarray, list[str]]:
    if len(objects) != len(images) or not objects:
        raise ValueError("calibration object/image point lists must be non-empty and aligned")
    object_arrays = [np.asarray(value, dtype=np.float32).reshape(-1, 3) for value in objects]
    image_arrays = [np.asarray(value, dtype=np.float32).reshape(-1, 1, 2) for value in images]
    for object_array, image_array in zip(object_arrays, image_arrays):
        if object_array.shape[0] != image_array.shape[0]:
            raise ValueError("object and image point counts differ in a CV training view")
    initial_matrix = cv2.initCameraMatrix2D(
        object_arrays, image_arrays, image_size, aspectRatio=0
    )
    initial_distortion = np.zeros((8, 1), dtype=np.float64)
    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        int(max_iterations),
        float(epsilon),
    )
    output = cv2.calibrateCameraExtended(
        object_arrays,
        image_arrays,
        image_size,
        initial_matrix,
        initial_distortion,
        flags=cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL,
        criteria=criteria,
    )
    rms, matrix, distortion = output[:3]
    flat_distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
    if flat_distortion.size < 8:
        raise RuntimeError("OpenCV returned fewer than eight Rational coefficients")
    warnings: list[str] = []
    if flat_distortion.size > 8 and np.any(np.abs(flat_distortion[8:]) > 1.0e-12):
        warnings.append("OpenCV returned unexpected non-zero coefficients after k6")
    if not np.isfinite(rms) or not np.all(np.isfinite(matrix)) or not np.all(
        np.isfinite(flat_distortion[:8])
    ):
        raise RuntimeError("cross-validation calibration produced NaN or Inf")
    return float(rms), np.asarray(matrix, dtype=np.float64), flat_distortion[:8], warnings


def _parameter_values(matrix: np.ndarray, distortion: np.ndarray) -> dict[str, float]:
    values = {
        "fx": float(matrix[0, 0]),
        "fy": float(matrix[1, 1]),
        "cx": float(matrix[0, 2]),
        "cy": float(matrix[1, 2]),
    }
    values.update(dict(zip(RATIONAL_PARAMETER_NAMES, distortion.tolist())))
    return values


def cross_validate(
    views: Sequence[Any],
    object_points: np.ndarray | Sequence[np.ndarray],
    image_size: Sequence[int],
    config: Any,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Run pose-diverse grouped cross-validation with the fixed Rational model.

    Near-identical pose descriptors are grouped before fold assignment, so a
    duplicate group cannot leak into both training and validation.  Group
    centroids are traversed by seeded farthest-point sampling and distributed
    between folds while balancing the number of views.
    """

    view_list = list(views)
    if len(view_list) < 4:
        raise ValueError("cross-validation requires at least four accepted views")
    size = _image_size(image_size)
    objects = _expand_object_points(object_points, len(view_list))
    images = [_view_points(view) for view in view_list]
    for object_array, image_array in zip(objects, images):
        if object_array.shape[0] != image_array.shape[0]:
            raise ValueError("object/image point count mismatch in cross-validation input")

    requested_folds = int(_config_value(config, "cv_folds", 5))
    if requested_folds < 2:
        raise ValueError("cv_folds must be at least two")
    seed = int(_config_value(config, "seed", 42))
    max_iterations = int(_config_value(config, "max_iterations", 200))
    epsilon = float(_config_value(config, "epsilon", 1.0e-12))
    fold_indices, group_sizes = _pose_group_folds(
        view_list, size, requested_folds, seed
    )

    rows: list[dict[str, Any]] = []
    fold_warnings: list[str] = []
    for fold_index, validation_indices in enumerate(fold_indices):
        validation_set = set(validation_indices)
        training_indices = [
            index for index in range(len(view_list)) if index not in validation_set
        ]
        row: dict[str, Any] = {
            "fold": fold_index,
            "status": "failed",
            "training_view_count": len(training_indices),
            "validation_view_count": len(validation_indices),
            "training_views": ";".join(
                str(_view_path(view_list[index], index)) for index in training_indices
            ),
            "validation_views": ";".join(
                str(_view_path(view_list[index], index)) for index in validation_indices
            ),
            "training_rms": None,
            "validation_rms": None,
            "error": "",
        }
        try:
            train_rms, matrix, distortion, warnings = _calibrate_arrays(
                [objects[index] for index in training_indices],
                [images[index] for index in training_indices],
                size,
                max_iterations,
                epsilon,
            )
            held_views = [view_list[index] for index in validation_indices]
            held_objects = [objects[index] for index in validation_indices]
            validation = validate_fixed_intrinsics(
                held_views, held_objects, matrix, distortion, size
            )
            row.update(
                {
                    "status": "ok",
                    "training_rms": train_rms,
                    "validation_rms": validation.validation_rms,
                    **_parameter_values(matrix, distortion),
                }
            )
            fold_warnings.extend(f"fold {fold_index}: {warning}" for warning in warnings)
            fold_warnings.extend(
                f"fold {fold_index}: {warning}" for warning in validation.warnings
            )
        except (cv2.error, ValueError, RuntimeError) as exc:
            row["error"] = str(exc)
            fold_warnings.append(f"fold {fold_index} failed: {exc}")
        rows.append(row)

    successful = [row for row in rows if row["status"] == "ok"]
    parameters: dict[str, dict[str, Any]] = {}
    stability_warnings: list[str] = []
    for name in ALL_PARAMETER_NAMES:
        values = np.asarray(
            [row[name] for row in successful if row.get(name) is not None],
            dtype=np.float64,
        )
        if values.size == 0:
            parameters[name] = {
                "mean": None,
                "standard_deviation": None,
                "coefficient_of_variation": None,
                "coefficient_of_variation_percent": None,
                "values": [],
            }
            continue
        mean = float(np.mean(values))
        standard_deviation = float(np.std(values))
        coefficient = (
            standard_deviation / abs(mean) if abs(mean) > 1.0e-12 else None
        )
        parameters[name] = {
            "mean": mean,
            "standard_deviation": standard_deviation,
            "coefficient_of_variation": coefficient,
            "coefficient_of_variation_percent": (
                100.0 * coefficient if coefficient is not None else None
            ),
            "minimum": float(np.min(values)),
            "maximum": float(np.max(values)),
            "values": values.tolist(),
        }

        if name in ("fx", "fy") and coefficient is not None and coefficient > 0.05:
            stability_warnings.append(
                f"{name} varies by {100.0 * coefficient:.1f}% across folds"
            )
        if name == "cx" and standard_deviation / size[0] > 0.03:
            stability_warnings.append("cx varies by more than 3% of image width")
        if name == "cy" and standard_deviation / size[1] > 0.03:
            stability_warnings.append("cy varies by more than 3% of image height")
        if name in RATIONAL_PARAMETER_NAMES and abs(mean) > 1.0e-3:
            if coefficient is not None and coefficient > 1.0:
                stability_warnings.append(
                    f"{name} coefficient of variation exceeds 100%"
                )

    validation_values = np.asarray(
        [row["validation_rms"] for row in successful], dtype=np.float64
    )
    if validation_values.size > 1:
        validation_cv = float(np.std(validation_values)) / max(
            abs(float(np.mean(validation_values))), 1.0e-12
        )
        if validation_cv > 0.5:
            stability_warnings.append(
                "validation RMS varies by more than 50% across folds"
            )
    else:
        validation_cv = None

    all_warnings = [*fold_warnings, *stability_warnings]
    if not successful:
        quality_status = "failed_quality_check"
    elif stability_warnings or len(successful) != len(rows):
        quality_status = "warning"
    else:
        quality_status = "passed"
    assignments = {
        str(_view_path(view_list[index], index)): fold_index
        for fold_index, indices in enumerate(fold_indices)
        for index in indices
    }
    summary: dict[str, Any] = {
        "quality_status": quality_status,
        "seed": seed,
        "requested_folds": requested_folds,
        "actual_folds": len(fold_indices),
        "successful_folds": len(successful),
        "pose_group_count": len(group_sizes),
        "pose_group_sizes": group_sizes,
        "fold_assignments": assignments,
        "training_rms_mean": (
            float(np.mean([row["training_rms"] for row in successful]))
            if successful
            else None
        ),
        "validation_rms_mean": (
            float(np.mean(validation_values)) if validation_values.size else None
        ),
        "validation_rms_standard_deviation": (
            float(np.std(validation_values)) if validation_values.size else None
        ),
        "validation_rms_coefficient_of_variation": validation_cv,
        "parameters": parameters,
        "warnings": all_warnings,
    }
    return rows, summary


def write_cross_validation_outputs(
    rows: Sequence[Mapping[str, Any]],
    summary: Mapping[str, Any],
    output_dir: Path | str,
) -> dict[str, Path]:
    """Write cross-validation CSV, stability JSON, and stability plot."""

    try:
        from .calibration_visualization import save_parameter_stability
    except ImportError:  # Direct script imports.
        from calibration_visualization import save_parameter_stability  # type: ignore

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    csv_path = destination / "cross_validation_results.csv"
    json_path = destination / "parameter_stability.json"
    plot_path = destination / "parameter_stability.png"
    fields = (
        "fold",
        "status",
        "training_view_count",
        "validation_view_count",
        "training_rms",
        "validation_rms",
        *ALL_PARAMETER_NAMES,
        "training_views",
        "validation_views",
        "error",
    )
    _write_csv(csv_path, rows, fields)
    json_path.write_text(
        json.dumps(_json_ready(summary), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    save_parameter_stability(summary, plot_path)
    return {"csv": csv_path, "json": json_path, "plot": plot_path}


__all__ = [
    "ALL_PARAMETER_NAMES",
    "RATIONAL_PARAMETER_NAMES",
    "ValidationResult",
    "check_rational_stability",
    "cross_validate",
    "validate_fixed_intrinsics",
    "write_cross_validation_outputs",
    "write_rational_stability_outputs",
    "write_validation_outputs",
]
