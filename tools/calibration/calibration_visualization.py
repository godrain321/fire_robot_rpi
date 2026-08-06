"""OpenCV-only plots and overlays for checkerboard calibration.

The project intentionally avoids Matplotlib so the offline command works on a
minimal Raspberry Pi/OpenCV installation.  Every public function creates its
parent directory and returns the path it wrote (except the undistortion batch,
which returns detailed output metadata).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import cv2
import numpy as np


_FONT = cv2.FONT_HERSHEY_SIMPLEX
_AA = cv2.LINE_AA


def _get_value(item: Any, *names: str, default: Any = None) -> Any:
    for name in names:
        if isinstance(item, Mapping) and name in item:
            return item[name]
        if hasattr(item, name):
            return getattr(item, name)
    return default


def _to_uint8(image: np.ndarray) -> np.ndarray:
    array = np.asarray(image)
    if array.dtype == np.uint8:
        return array.copy()
    if np.issubdtype(array.dtype, np.integer):
        maximum = float(np.iinfo(array.dtype).max)
        return np.clip(array.astype(np.float64) * (255.0 / maximum), 0, 255).astype(
            np.uint8
        )
    finite = array[np.isfinite(array)]
    if finite.size == 0:
        return np.zeros(array.shape, dtype=np.uint8)
    low, high = float(np.min(finite)), float(np.max(finite))
    if high <= low:
        return np.zeros(array.shape, dtype=np.uint8)
    return np.clip((array - low) * (255.0 / (high - low)), 0, 255).astype(np.uint8)


def _to_bgr(image: np.ndarray) -> np.ndarray:
    array = _to_uint8(image)
    if array.ndim == 2:
        return cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
    if array.ndim != 3:
        raise ValueError(f"image must have two or three dimensions, got {array.shape}")
    if array.shape[2] == 1:
        return cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
    if array.shape[2] == 3:
        return array.copy()
    if array.shape[2] == 4:
        return cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
    raise ValueError(f"unsupported image channel count: {array.shape[2]}")


def _read_image(source: Path | str | np.ndarray) -> np.ndarray:
    if isinstance(source, np.ndarray):
        return _to_bgr(source)
    path = Path(source)
    try:
        encoded = np.fromfile(path, dtype=np.uint8)
    except OSError as exc:
        raise FileNotFoundError(f"cannot read image {path}: {exc}") from exc
    if encoded.size == 0:
        raise ValueError(f"image file is empty: {path}")
    image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"OpenCV could not decode image: {path}")
    return _to_bgr(image)


def _write_image(destination: Path | str, image: np.ndarray) -> Path:
    path = Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix.lower() or ".png"
    if not path.suffix:
        path = path.with_suffix(suffix)
    success, encoded = cv2.imencode(suffix, _to_uint8(image))
    if not success:
        raise OSError(f"OpenCV could not encode output image: {path}")
    try:
        encoded.tofile(path)
    except OSError as exc:
        raise OSError(f"cannot write image {path}: {exc}") from exc
    return path


def _fit_text_scale(text: str, maximum_width: int, base: float = 0.7) -> float:
    scale = base
    while scale > 0.32:
        width = cv2.getTextSize(text, _FONT, scale, 1)[0][0]
        if width <= maximum_width:
            break
        scale -= 0.05
    return scale


def _draw_label(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    *,
    color: tuple[int, int, int] = (255, 255, 255),
    background: tuple[int, int, int] = (24, 24, 24),
    scale: float = 0.55,
) -> None:
    size, baseline = cv2.getTextSize(text, _FONT, scale, 1)
    x, y = origin
    cv2.rectangle(
        image,
        (max(0, x - 4), max(0, y - size[1] - 5)),
        (min(image.shape[1] - 1, x + size[0] + 4), min(image.shape[0] - 1, y + baseline + 3)),
        background,
        cv2.FILLED,
    )
    cv2.putText(image, text, (x, y), _FONT, scale, color, 1, _AA)


def _board_shape(board: Any) -> tuple[int, int]:
    columns = _get_value(
        board, "inner_corners_cols", "board_cols", "columns", "cols"
    )
    rows = _get_value(board, "inner_corners_rows", "board_rows", "rows")
    if columns is None or rows is None:
        if isinstance(board, Sequence) and not isinstance(board, (str, bytes)):
            columns, rows = board[:2]
        else:
            raise ValueError("board must provide inner corner columns and rows")
    columns, rows = int(columns), int(rows)
    if columns <= 1 or rows <= 1:
        raise ValueError("checkerboard columns and rows must exceed one")
    return columns, rows


def save_detection_overlay(
    view: Any, board: Any, destination: Path | str
) -> Path:
    """Draw detected corners, their order, board outline, center, and metadata."""

    source = _get_value(view, "image")
    path_value = _get_value(view, "path", "image_path", "file_path")
    path = Path(path_value) if path_value is not None else Path("unknown_image")
    if source is None:
        try:
            image = _read_image(path)
        except (FileNotFoundError, ValueError):
            width = max(640, int(_get_value(view, "width", default=0) or 0))
            height = max(360, int(_get_value(view, "height", default=0) or 0))
            image = np.full((height, width, 3), 36, dtype=np.uint8)
            cv2.putText(
                image,
                "IMAGE UNREADABLE",
                (30, height // 2),
                _FONT,
                1.1,
                (60, 80, 255),
                2,
                _AA,
            )
    else:
        image = _read_image(np.asarray(source))

    columns, rows = _board_shape(board)
    expected = columns * rows
    corners_value = _get_value(view, "corners", "image_points", "points")
    corners = (
        np.asarray(corners_value, dtype=np.float32).reshape(-1, 2)
        if corners_value is not None
        else np.empty((0, 2), dtype=np.float32)
    )
    detected_flag = bool(
        _get_value(view, "detection_success", "found", "accepted", default=False)
    )
    success = bool(
        detected_flag
        and corners.shape[0] == expected
        and np.all(np.isfinite(corners))
    )

    if corners.size and np.all(np.isfinite(corners)):
        cv2.drawChessboardCorners(
            image, (columns, rows), corners.reshape(-1, 1, 2), success
        )
        label_scale = max(0.32, min(0.58, min(image.shape[:2]) / 1500.0))
        for index, point in enumerate(corners):
            x, y = int(round(float(point[0]))), int(round(float(point[1])))
            cv2.putText(
                image,
                str(index),
                (x + 3, y - 3),
                _FONT,
                label_scale,
                (255, 255, 255),
                1,
                _AA,
            )
        if corners.shape[0] == expected:
            outer_indices = (0, columns - 1, expected - 1, (rows - 1) * columns)
            outline = np.asarray([corners[index] for index in outer_indices], np.int32)
            cv2.polylines(image, [outline], True, (0, 200, 255), 3, _AA)
        center = np.mean(corners, axis=0)
        center_point = tuple(np.rint(center).astype(int))
        cv2.drawMarker(
            image, center_point, (255, 255, 0), cv2.MARKER_CROSS, 22, 2, _AA
        )
        area_ratio = float(cv2.contourArea(cv2.convexHull(corners))) / float(
            image.shape[0] * image.shape[1]
        )
    else:
        center_point = None
        area_ratio = float(_get_value(view, "area_ratio", default=0.0) or 0.0)

    status = "ACCEPTED" if success else "REJECTED"
    status_color = (70, 230, 70) if success else (60, 80, 255)
    header_height = max(68, int(image.shape[0] * 0.08))
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (image.shape[1], header_height), (10, 10, 10), cv2.FILLED)
    cv2.addWeighted(overlay, 0.72, image, 0.28, 0.0, image)
    title = f"{path.name}  [{status}]"
    title_scale = _fit_text_scale(title, image.shape[1] - 30, 0.8)
    cv2.putText(image, title, (15, 28), _FONT, title_scale, status_color, 2, _AA)
    detail = (
        f"corners {corners.shape[0]}/{expected} | area {100.0 * area_ratio:.2f}%"
    )
    if center_point is not None:
        detail += f" | center ({center_point[0]}, {center_point[1]})"
    cv2.putText(image, detail, (15, 56), _FONT, 0.55, (230, 230, 230), 1, _AA)
    reason = str(_get_value(view, "exclusion_reason", default="") or "")
    if reason:
        _draw_label(
            image,
            f"reason: {reason}",
            (15, image.shape[0] - 16),
            color=(120, 160, 255),
        )
    return _write_image(destination, image)


def save_coverage_heatmap(counts: np.ndarray, destination: Path | str) -> Path:
    """Save an annotated checkerboard-corner coverage grid."""

    values = np.asarray(counts, dtype=np.float64)
    if values.ndim != 2 or values.size == 0:
        raise ValueError("coverage counts must be a non-empty 2-D array")
    if not np.all(np.isfinite(values)) or np.any(values < 0.0):
        raise ValueError("coverage counts must be finite and non-negative")
    rows, columns = values.shape
    cell = int(np.clip(720 / max(rows, columns), 54, 120))
    normalized = (
        np.rint(255.0 * values / np.max(values)).astype(np.uint8)
        if np.max(values) > 0.0
        else np.zeros(values.shape, dtype=np.uint8)
    )
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    grid = cv2.resize(colored, (columns * cell, rows * cell), interpolation=cv2.INTER_NEAREST)
    for row in range(rows):
        for column in range(columns):
            x0, y0 = column * cell, row * cell
            x1, y1 = x0 + cell, y0 + cell
            cv2.rectangle(grid, (x0, y0), (x1, y1), (235, 235, 235), 1)
            text = str(int(values[row, column])) if values[row, column].is_integer() else f"{values[row, column]:.1f}"
            scale = _fit_text_scale(text, cell - 8, 0.62)
            text_size = cv2.getTextSize(text, _FONT, scale, 1)[0]
            origin = (
                x0 + (cell - text_size[0]) // 2,
                y0 + (cell + text_size[1]) // 2,
            )
            intensity = int(normalized[row, column])
            color = (10, 10, 10) if 70 < intensity < 220 else (255, 255, 255)
            cv2.putText(grid, text, origin, _FONT, scale, color, 1, _AA)

    margin_top, margin_left, margin_bottom = 65, 48, 42
    canvas = np.full(
        (grid.shape[0] + margin_top + margin_bottom, grid.shape[1] + margin_left + 16, 3),
        245,
        dtype=np.uint8,
    )
    canvas[margin_top : margin_top + grid.shape[0], margin_left : margin_left + grid.shape[1]] = grid
    cv2.putText(canvas, "Corner coverage counts", (margin_left, 34), _FONT, 0.85, (25, 25, 25), 2, _AA)
    cv2.putText(
        canvas,
        f"total observations: {int(np.sum(values))}   maximum cell: {int(np.max(values))}",
        (margin_left, 57),
        _FONT,
        0.48,
        (55, 55, 55),
        1,
        _AA,
    )
    for column in range(columns):
        cv2.putText(
            canvas,
            str(column),
            (margin_left + column * cell + cell // 2 - 4, margin_top + grid.shape[0] + 27),
            _FONT,
            0.45,
            (45, 45, 45),
            1,
            _AA,
        )
    for row in range(rows):
        cv2.putText(
            canvas,
            str(row),
            (14, margin_top + row * cell + cell // 2 + 5),
            _FONT,
            0.45,
            (45, 45, 45),
            1,
            _AA,
        )
    return _write_image(destination, canvas)


def save_validation_overlay(
    image_path: Path | str | np.ndarray,
    detected: np.ndarray,
    projected: np.ndarray,
    rms: float,
    destination: Path | str,
) -> Path:
    """Draw observed/projected corners and their reprojection error vectors."""

    image = _read_image(image_path)
    actual = np.asarray(detected, dtype=np.float64).reshape(-1, 2)
    estimate = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
    if actual.shape != estimate.shape or actual.size == 0:
        raise ValueError("detected and projected points must have the same non-empty shape")
    if not np.all(np.isfinite(actual)) or not np.all(np.isfinite(estimate)):
        raise ValueError("overlay points contain NaN or Inf")
    errors = np.linalg.norm(actual - estimate, axis=1)
    vector_scale = max(1.0, min(image.shape[:2]) / 900.0)
    for observed, reprojection in zip(actual, estimate):
        p_observed = tuple(np.rint(observed).astype(int))
        p_projected = tuple(np.rint(reprojection).astype(int))
        cv2.arrowedLine(
            image,
            p_observed,
            p_projected,
            (0, 220, 255),
            max(1, int(round(vector_scale))),
            _AA,
            tipLength=0.28,
        )
        cv2.circle(image, p_observed, 4, (60, 230, 60), cv2.FILLED, _AA)
        cv2.drawMarker(
            image, p_projected, (255, 80, 220), cv2.MARKER_TILTED_CROSS, 8, 1, _AA
        )

    source_name = Path(image_path).name if not isinstance(image_path, np.ndarray) else "image"
    header = image.copy()
    cv2.rectangle(header, (0, 0), (image.shape[1], 74), (8, 8, 8), cv2.FILLED)
    cv2.addWeighted(header, 0.75, image, 0.25, 0.0, image)
    cv2.putText(
        image,
        f"{source_name} | RMS {float(rms):.4f} px | max {float(np.max(errors)):.4f} px",
        (14, 29),
        _FONT,
        _fit_text_scale(source_name, image.shape[1] - 30, 0.65),
        (245, 245, 245),
        2,
        _AA,
    )
    cv2.putText(image, "detected", (14, 57), _FONT, 0.48, (60, 230, 60), 1, _AA)
    cv2.putText(image, "projected", (105, 57), _FONT, 0.48, (255, 80, 220), 1, _AA)
    cv2.putText(image, "error vector", (210, 57), _FONT, 0.48, (0, 220, 255), 1, _AA)
    return _write_image(destination, image)


def _empty_plot(title: str, message: str, destination: Path | str) -> Path:
    canvas = np.full((600, 1000, 3), 250, dtype=np.uint8)
    cv2.putText(canvas, title, (55, 70), _FONT, 1.05, (30, 30, 30), 2, _AA)
    cv2.putText(canvas, message, (55, 320), _FONT, 0.8, (90, 90, 90), 2, _AA)
    return _write_image(destination, canvas)


def save_error_histogram(errors: np.ndarray, destination: Path | str) -> Path:
    """Save an OpenCV-rendered histogram of Euclidean corner errors."""

    values = np.asarray(errors, dtype=np.float64).reshape(-1)
    values = values[np.isfinite(values) & (values >= 0.0)]
    if values.size == 0:
        return _empty_plot("Validation reprojection error", "No finite errors", destination)
    bins = int(np.clip(round(math.sqrt(values.size)), 12, 48))
    maximum = max(float(np.max(values)), np.finfo(np.float64).eps)
    counts, edges = np.histogram(values, bins=bins, range=(0.0, maximum))
    canvas = np.full((680, 1120, 3), 250, dtype=np.uint8)
    left, top, right, bottom = 90, 100, 1060, 570
    cv2.rectangle(canvas, (left, top), (right, bottom), (55, 55, 55), 1)
    plot_width, plot_height = right - left, bottom - top
    count_max = max(int(np.max(counts)), 1)
    for index, count in enumerate(counts):
        x0 = left + int(index * plot_width / bins)
        x1 = left + int((index + 1) * plot_width / bins) - 1
        y = bottom - int(float(count) * plot_height / count_max)
        cv2.rectangle(canvas, (x0, y), (x1, bottom - 1), (220, 125, 45), cv2.FILLED)
        cv2.rectangle(canvas, (x0, y), (x1, bottom - 1), (130, 75, 25), 1)

    percentiles = ((50.0, (60, 180, 60)), (90.0, (40, 170, 230)), (95.0, (50, 90, 235)), (99.0, (170, 60, 220)))
    legend_x = 620
    for offset, (percentile, color) in enumerate(percentiles):
        value = float(np.percentile(values, percentile))
        x = left + int(np.clip(value / maximum, 0.0, 1.0) * plot_width)
        cv2.line(canvas, (x, top), (x, bottom), color, 2, _AA)
        cv2.putText(
            canvas,
            f"p{int(percentile)}={value:.3f}",
            (legend_x + 110 * offset, 82),
            _FONT,
            0.43,
            color,
            1,
            _AA,
        )
    for tick in range(6):
        fraction = tick / 5.0
        x = left + int(fraction * plot_width)
        y = bottom - int(fraction * plot_height)
        cv2.line(canvas, (x, bottom), (x, bottom + 6), (45, 45, 45), 1)
        cv2.putText(canvas, f"{fraction * maximum:.2f}", (x - 15, bottom + 25), _FONT, 0.42, (55, 55, 55), 1, _AA)
        cv2.line(canvas, (left - 6, y), (left, y), (45, 45, 45), 1)
        cv2.putText(canvas, str(int(fraction * count_max)), (35, y + 5), _FONT, 0.42, (55, 55, 55), 1, _AA)
    cv2.putText(canvas, "Validation reprojection error histogram", (left, 45), _FONT, 0.9, (25, 25, 25), 2, _AA)
    cv2.putText(
        canvas,
        f"n={values.size}  RMS={math.sqrt(float(np.mean(values**2))):.4f} px  mean={float(np.mean(values)):.4f} px",
        (left, 74),
        _FONT,
        0.5,
        (65, 65, 65),
        1,
        _AA,
    )
    cv2.putText(canvas, "Euclidean error (pixels)", (440, 625), _FONT, 0.55, (45, 45, 45), 1, _AA)
    cv2.putText(canvas, "count", (12, 345), _FONT, 0.5, (45, 45, 45), 1, _AA)
    return _write_image(destination, canvas)


def save_error_heatmap(
    points: np.ndarray,
    errors: np.ndarray,
    image_size: Sequence[int],
    destination: Path | str,
    *,
    grid_size: tuple[int, int] = (64, 48),
) -> Path:
    """Save a smoothed, count-weighted spatial reprojection-error heatmap."""

    width, height = int(image_size[0]), int(image_size[1])
    if width <= 0 or height <= 0:
        raise ValueError("image_size must contain positive width and height")
    coordinates = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    values = np.asarray(errors, dtype=np.float64).reshape(-1)
    if coordinates.shape[0] != values.size:
        raise ValueError("heatmap points and errors must have the same length")
    valid = (
        np.all(np.isfinite(coordinates), axis=1)
        & np.isfinite(values)
        & (values >= 0.0)
        & (coordinates[:, 0] >= 0.0)
        & (coordinates[:, 0] < width)
        & (coordinates[:, 1] >= 0.0)
        & (coordinates[:, 1] < height)
    )
    coordinates, values = coordinates[valid], values[valid]
    if values.size == 0:
        return _empty_plot("Validation spatial error", "No finite in-image errors", destination)
    grid_width, grid_height = int(grid_size[0]), int(grid_size[1])
    if grid_width < 4 or grid_height < 4:
        raise ValueError("heatmap grid dimensions must both be at least four")
    sums = np.zeros((grid_height, grid_width), dtype=np.float32)
    counts = np.zeros_like(sums)
    x_index = np.minimum((coordinates[:, 0] * grid_width / width).astype(int), grid_width - 1)
    y_index = np.minimum((coordinates[:, 1] * grid_height / height).astype(int), grid_height - 1)
    np.add.at(sums, (y_index, x_index), values.astype(np.float32))
    np.add.at(counts, (y_index, x_index), 1.0)
    smooth_sums = cv2.GaussianBlur(sums, (0, 0), 2.2)
    smooth_counts = cv2.GaussianBlur(counts, (0, 0), 2.2)
    mean_grid = np.divide(
        smooth_sums,
        smooth_counts,
        out=np.zeros_like(smooth_sums),
        where=smooth_counts > 1.0e-6,
    )
    scale_max = max(float(np.percentile(values, 99.0)), np.finfo(np.float32).eps)
    normalized = np.clip(mean_grid * (255.0 / scale_max), 0, 255).astype(np.uint8)
    color = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    display_width = min(width, 1200)
    display_height = max(1, int(round(display_width * height / width)))
    heatmap = cv2.resize(color, (display_width, display_height), interpolation=cv2.INTER_CUBIC)
    margin_top, margin_right, margin_bottom = 74, 96, 44
    canvas = np.full(
        (display_height + margin_top + margin_bottom, display_width + margin_right, 3),
        245,
        dtype=np.uint8,
    )
    canvas[margin_top : margin_top + display_height, :display_width] = heatmap
    cv2.rectangle(canvas, (0, margin_top), (display_width - 1, margin_top + display_height - 1), (30, 30, 30), 1)
    bar_x0, bar_x1 = display_width + 24, display_width + 52
    gradient = np.arange(255, -1, -1, dtype=np.uint8).reshape(256, 1)
    gradient = cv2.applyColorMap(gradient, cv2.COLORMAP_TURBO)
    gradient = cv2.resize(gradient, (bar_x1 - bar_x0, display_height))
    canvas[margin_top : margin_top + display_height, bar_x0:bar_x1] = gradient
    cv2.putText(canvas, f"{scale_max:.3f}", (bar_x1 + 5, margin_top + 8), _FONT, 0.4, (45, 45, 45), 1, _AA)
    cv2.putText(canvas, "0", (bar_x1 + 5, margin_top + display_height), _FONT, 0.4, (45, 45, 45), 1, _AA)
    cv2.putText(canvas, "px", (bar_x0, margin_top + display_height + 25), _FONT, 0.43, (45, 45, 45), 1, _AA)
    cv2.putText(canvas, "Validation reprojection error heatmap", (18, 35), _FONT, 0.86, (25, 25, 25), 2, _AA)
    cv2.putText(
        canvas,
        f"count-weighted smoothing | color maximum = p99 ({scale_max:.4f} px) | n={values.size}",
        (18, 61),
        _FONT,
        0.48,
        (65, 65, 65),
        1,
        _AA,
    )
    return _write_image(destination, canvas)


def _draw_xy_plot(
    canvas: np.ndarray,
    rectangle: tuple[int, int, int, int],
    series: Sequence[tuple[np.ndarray, np.ndarray, tuple[int, int, int], str]],
    title: str,
    x_label: str,
    y_label: str,
) -> None:
    left, top, right, bottom = rectangle
    finite_x = np.concatenate([x[np.isfinite(x)] for x, _, _, _ in series])
    finite_y = np.concatenate([y[np.isfinite(y)] for _, y, _, _ in series])
    if finite_x.size == 0 or finite_y.size == 0:
        return
    x_min, x_max = float(np.min(finite_x)), float(np.max(finite_x))
    y_min, y_max = float(np.min(finite_y)), float(np.max(finite_y))
    if x_max <= x_min:
        x_max = x_min + 1.0
    if y_max <= y_min:
        y_min -= 0.5
        y_max += 0.5
    padding = 0.08 * (y_max - y_min)
    y_min, y_max = y_min - padding, y_max + padding
    cv2.rectangle(canvas, (left, top), (right, bottom), (60, 60, 60), 1)
    for tick in range(6):
        fraction = tick / 5.0
        x = left + int(fraction * (right - left))
        y = bottom - int(fraction * (bottom - top))
        cv2.line(canvas, (x, top), (x, bottom), (225, 225, 225), 1)
        cv2.line(canvas, (left, y), (right, y), (225, 225, 225), 1)
        cv2.putText(canvas, f"{x_min + fraction * (x_max - x_min):.2f}", (x - 16, bottom + 20), _FONT, 0.36, (70, 70, 70), 1, _AA)
        cv2.putText(canvas, f"{y_min + fraction * (y_max - y_min):.2f}", (left - 55, y + 4), _FONT, 0.36, (70, 70, 70), 1, _AA)
    for values_x, values_y, color, _label in series:
        valid = np.isfinite(values_x) & np.isfinite(values_y)
        indices = np.flatnonzero(valid)
        if indices.size < 2:
            continue
        points = np.column_stack(
            (
                left + (values_x[indices] - x_min) * (right - left) / (x_max - x_min),
                bottom - (values_y[indices] - y_min) * (bottom - top) / (y_max - y_min),
            )
        )
        cv2.polylines(canvas, [np.rint(points).astype(np.int32)], False, color, 2, _AA)
    cv2.putText(canvas, title, (left, top - 18), _FONT, 0.68, (30, 30, 30), 2, _AA)
    cv2.putText(canvas, x_label, ((left + right) // 2 - 65, bottom + 44), _FONT, 0.46, (55, 55, 55), 1, _AA)
    cv2.putText(canvas, y_label, (left - 62, top - 4), _FONT, 0.44, (55, 55, 55), 1, _AA)
    legend_x = right - 160 * len(series)
    for index, (_, _, color, label) in enumerate(series):
        x = legend_x + index * 160
        cv2.line(canvas, (x, top - 18), (x + 25, top - 18), color, 3, _AA)
        cv2.putText(canvas, label, (x + 31, top - 13), _FONT, 0.4, color, 1, _AA)


def save_radial_curve(stability: Mapping[str, Any], destination: Path | str) -> Path:
    """Plot Rational radial mapping, denominator, and numerical derivative."""

    samples = stability.get("samples", {})
    radius = np.asarray(samples.get("radius", []), dtype=np.float64)
    mapped = np.asarray(samples.get("mapped_radius", []), dtype=np.float64)
    denominator = np.asarray(samples.get("denominator", []), dtype=np.float64)
    derivative = np.asarray(samples.get("derivative", []), dtype=np.float64)
    if not (radius.size and radius.shape == mapped.shape == denominator.shape == derivative.shape):
        return _empty_plot("Rational radial stability", "No curve samples", destination)
    canvas = np.full((900, 1280, 3), 248, dtype=np.uint8)
    status = str(stability.get("quality_status", "unknown"))
    status_color = (45, 160, 45) if status == "passed" else (40, 65, 220)
    cv2.putText(canvas, "Rational Polynomial radial stability", (60, 44), _FONT, 0.95, (25, 25, 25), 2, _AA)
    cv2.putText(canvas, f"status: {status}", (970, 44), _FONT, 0.64, status_color, 2, _AA)
    _draw_xy_plot(
        canvas,
        (90, 100, 1200, 430),
        (
            (radius, mapped, (220, 100, 30), "rd(r)"),
            (radius, radius, (80, 155, 80), "identity"),
        ),
        "Radial mapping",
        "normalized radius r",
        "mapped radius",
    )
    _draw_xy_plot(
        canvas,
        (90, 535, 1200, 810),
        (
            (radius, denominator, (200, 90, 35), "q(r)"),
            (radius, derivative, (150, 55, 190), "d rd / dr"),
            (radius, np.zeros_like(radius), (80, 80, 80), "zero"),
        ),
        "Denominator and radial derivative",
        "normalized radius r",
        "value",
    )
    return _write_image(destination, canvas)


def save_parameter_stability(
    summary: Mapping[str, Any], destination: Path | str
) -> Path:
    """Save twelve fold-parameter cards with values, mean, standard deviation, CV."""

    parameters = summary.get("parameters", {})
    names = ("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6")
    canvas = np.full((1040, 1520, 3), 246, dtype=np.uint8)
    status = str(summary.get("quality_status", "unknown"))
    status_color = (40, 155, 40) if status == "passed" else (40, 90, 220)
    cv2.putText(canvas, "Cross-validation parameter stability", (45, 48), _FONT, 1.0, (25, 25, 25), 2, _AA)
    cv2.putText(canvas, f"status: {status}", (1210, 48), _FONT, 0.65, status_color, 2, _AA)
    columns, rows = 4, 3
    gap, card_width, card_height = 18, 350, 282
    start_x, start_y = 40, 80
    palette = ((210, 95, 40), (60, 160, 65), (170, 65, 190), (35, 145, 210))
    for index, name in enumerate(names):
        column, row = index % columns, index // columns
        x0 = start_x + column * (card_width + gap)
        y0 = start_y + row * (card_height + gap)
        x1, y1 = x0 + card_width, y0 + card_height
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (215, 215, 215), cv2.FILLED)
        cv2.rectangle(canvas, (x0 + 2, y0 + 2), (x1 - 2, y1 - 2), (255, 255, 255), cv2.FILLED)
        data = parameters.get(name, {}) if isinstance(parameters, Mapping) else {}
        values = np.asarray(data.get("values", []), dtype=np.float64)
        values = values[np.isfinite(values)]
        mean = data.get("mean")
        std = data.get("standard_deviation")
        cv_percent = data.get("coefficient_of_variation_percent")
        cv2.putText(canvas, name, (x0 + 18, y0 + 38), _FONT, 0.82, (30, 30, 30), 2, _AA)
        if mean is None or std is None or values.size == 0:
            cv2.putText(canvas, "no successful folds", (x0 + 18, y0 + 138), _FONT, 0.55, (90, 90, 90), 1, _AA)
            continue
        cv2.putText(canvas, f"mean {float(mean):.6g}", (x0 + 18, y0 + 70), _FONT, 0.47, (65, 65, 65), 1, _AA)
        cv2.putText(canvas, f"std   {float(std):.6g}", (x0 + 18, y0 + 94), _FONT, 0.47, (65, 65, 65), 1, _AA)
        cv_text = "undefined" if cv_percent is None else f"{float(cv_percent):.2f}%"
        cv_color = (50, 150, 50) if cv_percent is not None and float(cv_percent) <= 10.0 else (40, 80, 215)
        cv2.putText(canvas, f"CV    {cv_text}", (x0 + 18, y0 + 118), _FONT, 0.47, cv_color, 1, _AA)
        plot_left, plot_top = x0 + 22, y0 + 145
        plot_right, plot_bottom = x1 - 22, y1 - 25
        cv2.rectangle(canvas, (plot_left, plot_top), (plot_right, plot_bottom), (225, 225, 225), 1)
        minimum, maximum = float(np.min(values)), float(np.max(values))
        if maximum <= minimum:
            minimum -= 0.5
            maximum += 0.5
        padding = 0.15 * (maximum - minimum)
        minimum, maximum = minimum - padding, maximum + padding
        mean_y = plot_bottom - int((float(mean) - minimum) * (plot_bottom - plot_top) / (maximum - minimum))
        cv2.line(canvas, (plot_left, mean_y), (plot_right, mean_y), (150, 150, 150), 1, _AA)
        color = palette[index % len(palette)]
        plot_points: list[tuple[int, int]] = []
        for fold, value in enumerate(values):
            x = plot_left + int((fold + 0.5) * (plot_right - plot_left) / values.size)
            y = plot_bottom - int((float(value) - minimum) * (plot_bottom - plot_top) / (maximum - minimum))
            plot_points.append((x, y))
            cv2.circle(canvas, (x, y), 5, color, cv2.FILLED, _AA)
            cv2.putText(canvas, str(fold), (x - 4, plot_bottom + 17), _FONT, 0.34, (85, 85, 85), 1, _AA)
        if len(plot_points) > 1:
            cv2.polylines(canvas, [np.asarray(plot_points, dtype=np.int32)], False, color, 2, _AA)
    return _write_image(destination, canvas)


def save_undistorted_samples(
    paths: Sequence[Path | str],
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    image_size: Sequence[int],
    output_dir: Path | str,
) -> dict[str, Any]:
    """Undistort samples for alpha 0 and 1, reusing one map per alpha.

    The original ``K`` and ``D`` are copied and never modified.  Images whose
    resolution differs from the calibrated resolution are skipped explicitly;
    resizing them would invalidate the geometry.  Outputs are visual checks,
    not replacements for the original calibration matrices in CameraInfo.
    """

    width, height = int(image_size[0]), int(image_size[1])
    if width <= 0 or height <= 0:
        raise ValueError("image_size must contain positive width and height")
    matrix = np.asarray(camera_matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("camera matrix must be a finite (3, 3) array")
    coefficients = np.asarray(distortion_coefficients, dtype=np.float64).reshape(-1)
    if coefficients.size < 8 or not np.all(np.isfinite(coefficients)):
        raise ValueError("undistortion requires at least eight finite Rational coefficients")
    if coefficients.size > 8 and np.any(np.abs(coefficients[8:]) > 1.0e-12):
        raise ValueError("unsupported non-zero coefficients occur after Rational k6")
    coefficients = coefficients[:8].copy()
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)

    maps: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    metadata: dict[str, Any] = {"skipped": []}
    for alpha in (0.0, 1.0):
        key = f"alpha_{int(alpha)}"
        alpha_dir = destination / key
        alpha_dir.mkdir(parents=True, exist_ok=True)
        new_matrix, roi = cv2.getOptimalNewCameraMatrix(
            matrix.copy(), coefficients.reshape(-1, 1), (width, height), alpha, (width, height)
        )
        map_x, map_y = cv2.initUndistortRectifyMap(
            matrix.copy(),
            coefficients.reshape(-1, 1),
            None,
            new_matrix,
            (width, height),
            cv2.CV_32FC1,
        )
        maps[key] = (map_x, map_y)
        metadata[key] = {
            "alpha": alpha,
            "new_camera_matrix": np.asarray(new_matrix).tolist(),
            "roi": [int(value) for value in roi],
            "files": [],
        }

    decoded: list[tuple[int, Path, np.ndarray]] = []
    for index, source_value in enumerate(paths):
        source = Path(source_value)
        try:
            image = _read_image(source)
        except (FileNotFoundError, ValueError) as exc:
            metadata["skipped"].append({"path": str(source), "reason": str(exc)})
            continue
        if (image.shape[1], image.shape[0]) != (width, height):
            metadata["skipped"].append(
                {
                    "path": str(source),
                    "reason": (
                        f"resolution {image.shape[1]}x{image.shape[0]} differs from "
                        f"calibration {width}x{height}"
                    ),
                }
            )
            continue
        decoded.append((index, source, image))

    for key, (map_x, map_y) in maps.items():
        alpha_dir = destination / key
        for index, source, image in decoded:
            corrected = cv2.remap(
                image, map_x, map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
            )
            output = alpha_dir / f"{index:04d}_{source.stem}.png"
            written = _write_image(output, corrected)
            metadata[key]["files"].append(str(written))
    return metadata


__all__ = [
    "save_coverage_heatmap",
    "save_detection_overlay",
    "save_error_heatmap",
    "save_error_histogram",
    "save_parameter_stability",
    "save_radial_curve",
    "save_undistorted_samples",
    "save_validation_overlay",
]
