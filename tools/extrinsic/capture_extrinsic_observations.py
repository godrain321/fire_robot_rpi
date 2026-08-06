#!/usr/bin/env python3
"""Interactive camera/2-D-LiDAR checkerboard observation collector.

This tool deliberately accepts live ROS 2 topics only.  Each saved pose contains
the checkerboard plane in the rectified camera frame and the robustly selected
LiDAR returns on that plane.  The companion extrinsic solver can then estimate
``p_camera = R_camera_lidar * p_lidar + t_camera_lidar_m``.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Sequence
import warnings

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.calibration.view_rational_undistortion import (  # noqa: E402
    RationalCameraInfo,
    load_rational_camera_info,
)


SCHEMA_VERSION = 1
TRANSFORM_CONVENTION = (
    "p_camera = R_camera_lidar * p_lidar + t_camera_lidar_m"
)
CAMERA_WINDOW = "Extrinsic calibration - rectified camera"
LIDAR_WINDOW = "Extrinsic calibration - LiDAR ROI (drag mouse)"
SCAN_MEDIAN_COUNT = 5
SYNC_BUFFER_COUNT = 60
PREFERRED_STAMP_SKEW_S = 0.15
# Calibration poses are static. A wider hard gate tolerates the startup
# callback backlog seen on Raspberry Pi, while the buffered matcher below
# still picks the closest timestamp pair and reports the actual skew.
MAX_STAMP_SKEW_S = 1.00
MAX_REPROJECTION_RMS_PX = 1.0
MIN_LINE_INLIERS = 8
MIN_LINE_SPAN_M = 0.15
MAX_LINE_RMS_M = 0.015


@dataclass(frozen=True)
class ImageSample:
    image: np.ndarray
    stamp_s: float
    frame_id: str


@dataclass(frozen=True)
class ScanSample:
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    stamp_s: float
    frame_id: str


@dataclass(frozen=True)
class BoardPose:
    corners_px: np.ndarray
    rvec: np.ndarray
    tvec_m: np.ndarray
    normal_camera: np.ndarray
    offset_camera_m: float
    reprojection_rms_px: float


@dataclass(frozen=True)
class MedianScan:
    points_xy_m: np.ndarray
    stamps_s: tuple[float, ...]
    reference_stamp_s: float
    frame_id: str


@dataclass(frozen=True)
class FrozenObservation:
    rectified_image: np.ndarray
    annotated_camera: np.ndarray
    image_stamp_s: float
    camera_frame_id: str
    board: BoardPose
    scan: MedianScan
    stamp_skew_s: float


@dataclass(frozen=True)
class LineFit:
    selected_points_xy_m: np.ndarray
    inlier_points_xy_m: np.ndarray
    centroid_xy_m: np.ndarray
    direction_xy: np.ndarray
    normal_xy: np.ndarray
    offset_m: float
    endpoints_xy_m: np.ndarray
    span_m: float
    rms_m: float
    mad_m: float
    threshold_m: float
    accepted: bool
    reason: str


def ros_stamp_seconds(stamp: Any) -> float:
    """Convert a builtin_interfaces/Time-like object to seconds."""

    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def image_message_to_bgr(message: Any) -> np.ndarray:
    """Decode common uncompressed sensor_msgs/Image encodings without cv_bridge."""

    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    encoding = str(message.encoding).lower()
    if height <= 0 or width <= 0 or step <= 0:
        raise ValueError("Image height, width, and step must be positive")

    formats = {
        "bgr8": (3, None),
        "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR),
        "8uc1": (1, cv2.COLOR_GRAY2BGR),
    }
    if encoding not in formats:
        raise ValueError(
            f"unsupported Image encoding '{message.encoding}'; use bgr8/rgb8/mono8"
        )
    channels, conversion = formats[encoding]
    used_bytes = width * channels
    if step < used_bytes:
        raise ValueError(f"Image step {step} is smaller than one row ({used_bytes})")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < height * step:
        raise ValueError("Image data is shorter than height * step")
    rows = raw[: height * step].reshape(height, step)[:, :used_bytes]
    if channels == 1:
        decoded = rows.reshape(height, width)
    else:
        decoded = rows.reshape(height, width, channels)
    if conversion is not None:
        decoded = cv2.cvtColor(decoded, conversion)
    return np.ascontiguousarray(decoded).copy()


def rectify_alpha_zero(
    image: np.ndarray, camera_info: RationalCameraInfo
) -> tuple[np.ndarray, np.ndarray, tuple[int, int, int, int]]:
    """Rectify at the calibrated resolution with all eight Rational terms."""

    width, height = camera_info.image_size
    if image.shape[:2] != (height, width):
        raise ValueError(
            f"live image is {image.shape[1]}x{image.shape[0]}, but camera_info.yaml "
            f"was calibrated at {width}x{height}; resizing is forbidden"
        )
    new_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_info.camera_matrix,
        camera_info.distortion_coefficients.reshape(-1, 1),
        (width, height),
        0.0,
        (width, height),
    )
    map_x, map_y = cv2.initUndistortRectifyMap(
        camera_info.camera_matrix,
        camera_info.distortion_coefficients.reshape(-1, 1),
        None,
        new_matrix,
        (width, height),
        cv2.CV_32FC1,
    )
    rectified = cv2.remap(
        image,
        map_x,
        map_y,
        cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    return rectified, np.asarray(new_matrix, dtype=np.float64), tuple(
        int(value) for value in roi
    )


def checkerboard_object_points(cols: int, rows: int, square_size_m: float) -> np.ndarray:
    points = np.zeros((cols * rows, 3), dtype=np.float32)
    points[:, :2] = (
        np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float32)
        * float(square_size_m)
    )
    return points


def estimate_board_pose(
    rectified: np.ndarray,
    rectified_matrix: np.ndarray,
    cols: int,
    rows: int,
    square_size_m: float,
) -> BoardPose:
    """Detect a complete SB grid and estimate its camera-frame plane."""

    gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    flags = (
        cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
    )
    found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=flags)
    expected = cols * rows
    if not found or corners is None:
        raise ValueError(f"complete {cols}x{rows} checkerboard was not detected")
    corners = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
    if corners.shape[0] != expected or not np.all(np.isfinite(corners)):
        raise ValueError(f"checkerboard must contain exactly {expected} finite corners")

    object_points = checkerboard_object_points(cols, rows, square_size_m)
    zero_distortion = np.zeros((5, 1), dtype=np.float64)
    solved, rvec, tvec = cv2.solvePnP(
        object_points,
        corners,
        rectified_matrix,
        zero_distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not solved:
        raise ValueError("solvePnP failed")
    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(
            object_points,
            corners,
            rectified_matrix,
            zero_distortion,
            rvec,
            tvec,
        )
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, rectified_matrix, zero_distortion
    )
    residuals = projected.reshape(-1, 2) - corners.reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
    if not math.isfinite(rms) or rms > MAX_REPROJECTION_RMS_PX:
        raise ValueError(
            f"PnP reprojection RMS {rms:.3f}px exceeds {MAX_REPROJECTION_RMS_PX:.1f}px"
        )

    rotation, _ = cv2.Rodrigues(rvec)
    normal = np.asarray(rotation[:, 2], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    translation = np.asarray(tvec, dtype=np.float64).reshape(3)
    # The same geometric plane has two signs.  Point the normal from the camera
    # origin toward the board so that offset is consistently non-positive.
    if float(np.dot(normal, translation)) < 0.0:
        normal = -normal
    offset = -float(np.dot(normal, translation))
    return BoardPose(
        corners_px=corners.reshape(-1, 2).astype(np.float64),
        rvec=np.asarray(rvec, dtype=np.float64).reshape(3),
        tvec_m=translation,
        normal_camera=normal,
        offset_camera_m=offset,
        reprojection_rms_px=rms,
    )


def annotate_board(
    rectified: np.ndarray,
    pose: BoardPose,
    matrix: np.ndarray,
    cols: int,
    rows: int,
    square_size_m: float,
) -> np.ndarray:
    canvas = rectified.copy()
    corners = pose.corners_px.astype(np.float32).reshape(-1, 1, 2)
    cv2.drawChessboardCorners(canvas, (cols, rows), corners, True)
    projected, _ = cv2.projectPoints(
        checkerboard_object_points(cols, rows, square_size_m),
        pose.rvec.reshape(3, 1),
        pose.tvec_m.reshape(3, 1),
        matrix,
        np.zeros((5, 1), dtype=np.float64),
    )
    for point in projected.reshape(-1, 2):
        cv2.drawMarker(
            canvas,
            tuple(np.rint(point).astype(int)),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=5,
            thickness=1,
        )
    _put_lines(
        canvas,
        [
            f"FROZEN: checkerboard {cols}x{rows}, square {square_size_m:.3f} m",
            f"PnP reprojection RMS: {pose.reprojection_rms_px:.3f} px (limit 1.000)",
            "Drag an ROI around the board returns in the LiDAR window",
        ],
        color=(40, 255, 40),
    )
    return canvas


def median_scan(scans: Sequence[ScanSample]) -> MedianScan:
    """Convert an element-wise median of exactly the latest five scans to XY."""

    if len(scans) != SCAN_MEDIAN_COUNT:
        raise ValueError(f"exactly {SCAN_MEDIAN_COUNT} scans are required")
    latest = list(scans)[-SCAN_MEDIAN_COUNT:]
    first = latest[0]
    if first.stamp_s <= 0.0 or any(scan.stamp_s <= 0.0 for scan in latest):
        raise ValueError("camera and scan messages must contain non-zero ROS stamps")
    if any(scan.frame_id != first.frame_id for scan in latest):
        raise ValueError("the latest five LaserScan messages changed frame_id")
    size = first.ranges.size
    if size == 0 or any(scan.ranges.size != size for scan in latest):
        raise ValueError("the latest five LaserScan messages have incompatible sizes")
    for scan in latest[1:]:
        if not np.isclose(scan.angle_min, first.angle_min, atol=1.0e-7) or not np.isclose(
            scan.angle_increment, first.angle_increment, atol=1.0e-9
        ):
            raise ValueError("the latest five LaserScan angle grids differ")

    stack = np.stack([scan.ranges.astype(np.float64, copy=True) for scan in latest])
    for index, scan in enumerate(latest):
        valid = (
            np.isfinite(stack[index])
            & (stack[index] >= scan.range_min)
            & (stack[index] <= scan.range_max)
        )
        stack[index, ~valid] = np.nan
    valid_counts = np.count_nonzero(np.isfinite(stack), axis=0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        ranges = np.nanmedian(stack, axis=0)
    ranges[valid_counts < 3] = np.nan
    angles = first.angle_min + np.arange(size, dtype=np.float64) * first.angle_increment
    valid = np.isfinite(ranges)
    points = np.column_stack(
        (ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid]))
    )
    if points.size == 0:
        raise ValueError("five-scan median contains no valid ranges")
    return MedianScan(
        points_xy_m=points,
        stamps_s=tuple(float(scan.stamp_s) for scan in latest),
        # The element-wise median represents the middle of the acquisition
        # interval better than the final scan timestamp.
        reference_stamp_s=float(np.median([scan.stamp_s for scan in latest])),
        frame_id=first.frame_id,
    )


def closest_synchronized_observation(
    images: Sequence[ImageSample], scans: Sequence[ScanSample]
) -> tuple[ImageSample, MedianScan, float]:
    """Select the closest image and five-scan median from recent buffers."""

    if not images:
        raise ValueError("no camera images are buffered")
    if len(scans) < SCAN_MEDIAN_COUNT:
        raise ValueError(f"need at least {SCAN_MEDIAN_COUNT} buffered scans")

    best: tuple[tuple[float, float], ImageSample, MedianScan] | None = None
    image_list = list(images)
    scan_list = list(scans)
    for end in range(SCAN_MEDIAN_COUNT, len(scan_list) + 1):
        candidate_scan = median_scan(scan_list[end - SCAN_MEDIAN_COUNT:end])
        image = min(
            image_list,
            key=lambda item: abs(item.stamp_s - candidate_scan.reference_stamp_s),
        )
        skew = abs(image.stamp_s - candidate_scan.reference_stamp_s)
        # Prefer minimum skew; for an equal skew prefer the most recent pair.
        key = (skew, -candidate_scan.reference_stamp_s)
        if best is None or key < best[0]:
            best = (key, image, candidate_scan)
    assert best is not None
    return best[1], best[2], best[0][0]


def _principal_line(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    center = np.mean(points, axis=0)
    covariance = np.cov((points - center).T, bias=True)
    values, vectors = np.linalg.eigh(covariance)
    direction = vectors[:, int(np.argmax(values))]
    direction = direction / np.linalg.norm(direction)
    if direction[0] < 0.0 or (abs(direction[0]) < 1.0e-12 and direction[1] < 0.0):
        direction = -direction
    normal = np.array([-direction[1], direction[0]], dtype=np.float64)
    return center, direction, normal


def fit_line_pca_mad(selected_points_xy_m: np.ndarray) -> LineFit:
    """Fit a line using PCA, MAD outlier rejection, and strict quality gates."""

    points = np.asarray(selected_points_xy_m, dtype=np.float64).reshape(-1, 2)
    if points.shape[0] < MIN_LINE_INLIERS:
        return _rejected_line(points, f"need at least {MIN_LINE_INLIERS} selected points")
    if not np.all(np.isfinite(points)):
        return _rejected_line(points, "selected points contain NaN/Inf")

    center, _, normal = _principal_line(points)
    signed = (points - center) @ normal
    signed_median = float(np.median(signed))
    mad = float(np.median(np.abs(signed - signed_median)))
    robust_sigma = 1.4826 * mad
    threshold = max(3.0 * robust_sigma, 0.002)
    mask = np.abs(signed - signed_median) <= threshold
    inliers = points[mask]
    if inliers.shape[0] < MIN_LINE_INLIERS:
        return _rejected_line(
            points,
            f"MAD kept {inliers.shape[0]} points; need {MIN_LINE_INLIERS}",
            inliers,
            mad,
            threshold,
        )

    center, direction, normal = _principal_line(inliers)
    residuals = (inliers - center) @ normal
    rms = float(np.sqrt(np.mean(residuals * residuals)))
    projection = (inliers - center) @ direction
    span = float(np.max(projection) - np.min(projection))
    endpoints = np.stack(
        (center + np.min(projection) * direction, center + np.max(projection) * direction)
    )
    if float(np.dot(normal, center)) < 0.0:
        normal = -normal
    offset = -float(np.dot(normal, center))
    reasons: list[str] = []
    if inliers.shape[0] < MIN_LINE_INLIERS:
        reasons.append(f"inliers {inliers.shape[0]} < {MIN_LINE_INLIERS}")
    if span < MIN_LINE_SPAN_M:
        reasons.append(f"span {span:.3f} m < {MIN_LINE_SPAN_M:.3f} m")
    if not math.isfinite(rms) or rms > MAX_LINE_RMS_M:
        reasons.append(f"RMS {rms:.4f} m > {MAX_LINE_RMS_M:.4f} m")
    accepted = not reasons
    return LineFit(
        selected_points_xy_m=points,
        inlier_points_xy_m=inliers,
        centroid_xy_m=center,
        direction_xy=direction,
        normal_xy=normal,
        offset_m=offset,
        endpoints_xy_m=endpoints,
        span_m=span,
        rms_m=rms,
        mad_m=mad,
        threshold_m=threshold,
        accepted=accepted,
        reason="accepted" if accepted else "; ".join(reasons),
    )


def _rejected_line(
    selected: np.ndarray,
    reason: str,
    inliers: np.ndarray | None = None,
    mad: float = math.nan,
    threshold: float = math.nan,
) -> LineFit:
    empty = np.empty((0, 2), dtype=np.float64)
    return LineFit(
        selected_points_xy_m=np.asarray(selected, dtype=np.float64).reshape(-1, 2),
        inlier_points_xy_m=empty if inliers is None else inliers,
        centroid_xy_m=np.full(2, np.nan),
        direction_xy=np.full(2, np.nan),
        normal_xy=np.full(2, np.nan),
        offset_m=math.nan,
        endpoints_xy_m=np.empty((0, 2), dtype=np.float64),
        span_m=0.0,
        rms_m=math.inf,
        mad_m=mad,
        threshold_m=threshold,
        accepted=False,
        reason=reason,
    )


class TopDownRenderer:
    """Map LaserScan XY points to a stable, mouse-selectable top-down canvas."""

    def __init__(self, width: int = 900, height: int = 760, metres: float = 4.0):
        self.width = width
        self.height = height
        self.metres = metres
        self.origin = np.array([width * 0.5, height * 0.53], dtype=np.float64)
        self._update_scale()

    def _update_scale(self) -> None:
        self.scale = min(self.width, self.height) * 0.43 / self.metres

    def zoom(self, factor: float) -> None:
        self.metres = float(np.clip(self.metres * factor, 1.0, 12.0))
        self._update_scale()

    def xy_to_uv(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        return np.column_stack(
            (self.origin[0] - points[:, 1] * self.scale,
             self.origin[1] - points[:, 0] * self.scale)
        )

    def points_in_rectangle(
        self, points: np.ndarray, start: tuple[int, int], end: tuple[int, int]
    ) -> np.ndarray:
        uv = self.xy_to_uv(points)
        lo = np.minimum(start, end)
        hi = np.maximum(start, end)
        mask = (
            (uv[:, 0] >= lo[0])
            & (uv[:, 0] <= hi[0])
            & (uv[:, 1] >= lo[1])
            & (uv[:, 1] <= hi[1])
        )
        return np.asarray(points)[mask]

    def draw(
        self,
        points: np.ndarray | None,
        roi_start: tuple[int, int] | None,
        roi_end: tuple[int, int] | None,
        line: LineFit | None,
        status: Sequence[str],
    ) -> np.ndarray:
        canvas = np.full((self.height, self.width, 3), 22, dtype=np.uint8)
        for distance in range(1, int(self.metres) + 1):
            radius = int(round(distance * self.scale))
            cv2.circle(canvas, tuple(np.rint(self.origin).astype(int)), radius, (55, 55, 55), 1)
            cv2.putText(
                canvas,
                f"{distance}m",
                (int(self.origin[0] + 5), int(self.origin[1] - radius + 16)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (130, 130, 130),
                1,
                cv2.LINE_AA,
            )
        cv2.line(
            canvas,
            (int(self.origin[0]), int(self.origin[1])),
            (int(self.origin[0]), int(self.origin[1] - self.metres * self.scale)),
            (100, 100, 100),
            1,
        )
        cv2.putText(canvas, "X forward (UP)", (int(self.origin[0]) + 8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 170, 170), 1, cv2.LINE_AA)
        cv2.putText(canvas, "Y left", (20, int(self.origin[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 170, 170), 1, cv2.LINE_AA)
        if points is not None:
            uv = self.xy_to_uv(points)
            visible = (
                (uv[:, 0] >= 0) & (uv[:, 0] < self.width)
                & (uv[:, 1] >= 0) & (uv[:, 1] < self.height)
            )
            for u, v in np.rint(uv[visible]).astype(int):
                cv2.circle(canvas, (u, v), 3, (0, 255, 255), -1, cv2.LINE_AA)
        if roi_start is not None and roi_end is not None:
            cv2.rectangle(canvas, roi_start, roi_end, (255, 0, 255), 2)
            if roi_start == roi_end:
                cv2.circle(canvas, roi_start, 6, (255, 0, 255), -1)
        if line is not None:
            inlier_uv = self.xy_to_uv(line.inlier_points_xy_m)
            for u, v in np.rint(inlier_uv).astype(int):
                if 0 <= u < self.width and 0 <= v < self.height:
                    cv2.circle(canvas, (u, v), 4, (0, 255, 0) if line.accepted else (0, 120, 255), -1)
            if line.endpoints_xy_m.shape == (2, 2):
                endpoints = np.rint(self.xy_to_uv(line.endpoints_xy_m)).astype(int)
                cv2.line(canvas, tuple(endpoints[0]), tuple(endpoints[1]), (255, 100, 0), 3, cv2.LINE_AA)
        _put_lines(canvas, status, color=(255, 255, 255))
        return canvas


def _put_lines(
    image: np.ndarray,
    lines: Sequence[str],
    color: tuple[int, int, int] = (255, 255, 255),
    start_y: int = 28,
) -> None:
    overlay = image.copy()
    box_height = 12 + 27 * len(lines)
    cv2.rectangle(overlay, (8, 6), (min(image.shape[1] - 8, 810), box_height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.62, image, 0.38, 0.0, image)
    for index, line in enumerate(lines):
        cv2.putText(
            image,
            str(line),
            (18, start_y + index * 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.57,
            color,
            1,
            cv2.LINE_AA,
        )


class ObservationStore:
    def __init__(
        self,
        output_dir: Path,
        camera_info_path: Path,
        camera_info: RationalCameraInfo,
        args: argparse.Namespace,
    ) -> None:
        self.output_dir = output_dir.resolve()
        self.path = self.output_dir / "observations.json"
        self.screenshot_dir = self.output_dir / "screenshots"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.screenshot_dir.mkdir(parents=True, exist_ok=True)
        if self.path.exists():
            if not args.resume:
                raise FileExistsError(
                    f"{self.path} already exists; pass --resume to append without overwriting"
                )
            self.payload = json.loads(self.path.read_text(encoding="utf-8"))
            self._validate_existing()
            self._validate_resume_compatibility(
                camera_info_path, camera_info, args
            )
        else:
            self.payload = {
                "schema_version": SCHEMA_VERSION,
                "transform_convention": TRANSFORM_CONVENTION,
                "created_at_utc": datetime.now(timezone.utc).isoformat(),
                "camera_info_path": str(camera_info_path.resolve()),
                "camera_info_sha256": hashlib.sha256(
                    camera_info_path.read_bytes()
                ).hexdigest(),
                "camera_name": camera_info.camera_name,
                "image_size": list(camera_info.image_size),
                "distortion_model": "rational_polynomial",
                "rectification_alpha": 0.0,
                "topics": {"image": args.image_topic, "scan": args.scan_topic},
                "frames": {
                    "camera": args.camera_frame,
                    "lidar": args.lidar_frame,
                },
                "checkerboard": {
                    "inner_corners_cols": args.board_cols,
                    "inner_corners_rows": args.board_rows,
                    "square_size_m": args.square_size_m,
                },
                "quality_limits": {
                    "scan_median_count": SCAN_MEDIAN_COUNT,
                    "preferred_stamp_skew_s": PREFERRED_STAMP_SKEW_S,
                    "max_stamp_skew_s": MAX_STAMP_SKEW_S,
                    "max_pnp_reprojection_rms_px": MAX_REPROJECTION_RMS_PX,
                    "min_lidar_line_inliers": MIN_LINE_INLIERS,
                    "min_lidar_line_span_m": MIN_LINE_SPAN_M,
                    "max_lidar_line_rms_m": MAX_LINE_RMS_M,
                },
                "target_observation_count": args.target_views,
                "observations": [],
            }

    def _validate_existing(self) -> None:
        if not isinstance(self.payload, dict):
            raise ValueError("existing observations.json root must be an object")
        if self.payload.get("schema_version") != SCHEMA_VERSION:
            raise ValueError("existing observations.json has an unsupported schema_version")
        if self.payload.get("transform_convention") != TRANSFORM_CONVENTION:
            raise ValueError("existing observations.json transform convention differs")
        if not isinstance(self.payload.get("observations"), list):
            raise ValueError("existing observations.json must contain an observations list")

    def _validate_resume_compatibility(
        self,
        camera_info_path: Path,
        camera_info: RationalCameraInfo,
        args: argparse.Namespace,
    ) -> None:
        """Prevent silently mixing observations from different rigs/settings."""

        expected_path = str(camera_info_path.resolve())
        expected_hash = hashlib.sha256(camera_info_path.read_bytes()).hexdigest()
        if self.payload.get("camera_info_path") != expected_path:
            raise ValueError("--resume camera-info path differs from existing data")
        if self.payload.get("camera_info_sha256") != expected_hash:
            raise ValueError("--resume camera-info contents differ from existing data")
        if self.payload.get("image_size") != list(camera_info.image_size):
            raise ValueError("--resume camera calibration resolution differs")
        expected_board = {
            "inner_corners_cols": args.board_cols,
            "inner_corners_rows": args.board_rows,
            "square_size_m": args.square_size_m,
        }
        if self.payload.get("checkerboard") != expected_board:
            raise ValueError("--resume checkerboard settings differ from existing data")
        if self.payload.get("topics") != {
            "image": args.image_topic,
            "scan": args.scan_topic,
        }:
            raise ValueError("--resume ROS topics differ from existing data")
        if self.payload.get("frames") != {
            "camera": args.camera_frame,
            "lidar": args.lidar_frame,
        }:
            raise ValueError("--resume sensor frames differ from existing data")

    @property
    def count(self) -> int:
        return len(self.payload["observations"])

    def save(
        self,
        frozen: FrozenObservation,
        line: LineFit,
        lidar_screenshot: np.ndarray,
        rectified_matrix: np.ndarray,
        roi_px: tuple[tuple[int, int], tuple[int, int]],
    ) -> str:
        if not line.accepted:
            raise ValueError(f"LiDAR line is not accepted: {line.reason}")
        pose_id = f"pose_{self.count + 1:03d}"
        camera_name = f"{pose_id}_camera.png"
        lidar_name = f"{pose_id}_lidar.png"
        combined_name = f"{pose_id}_combined.png"
        camera_path = self.screenshot_dir / camera_name
        lidar_path = self.screenshot_dir / lidar_name
        combined_path = self.screenshot_dir / combined_name
        _write_png(camera_path, frozen.annotated_camera)
        _write_png(lidar_path, lidar_screenshot)
        _write_png(combined_path, _combine_images(frozen.annotated_camera, lidar_screenshot))

        inliers = line.inlier_points_xy_m
        observation = {
            "pose_id": pose_id,
            "saved_at_utc": datetime.now(timezone.utc).isoformat(),
            "camera_stamp_s": frozen.image_stamp_s,
            "scan_stamps_s": list(frozen.scan.stamps_s),
            "scan_reference_stamp_s": frozen.scan.reference_stamp_s,
            "stamp_skew_s": frozen.stamp_skew_s,
            "camera_frame_id": frozen.camera_frame_id,
            "lidar_frame_id": frozen.scan.frame_id,
            "camera_frame": frozen.camera_frame_id,
            "lidar_frame": frozen.scan.frame_id,
            "board_normal_camera": frozen.board.normal_camera.tolist(),
            "board_offset_camera_m": frozen.board.offset_camera_m,
            "board_plane_equation": "board_normal_camera dot X_camera + board_offset_camera_m = 0",
            "board_rvec_camera": frozen.board.rvec.tolist(),
            "board_tvec_camera_m": frozen.board.tvec_m.tolist(),
            "pnp_reprojection_rms_px": frozen.board.reprojection_rms_px,
            "checkerboard_corners_rectified_px": frozen.board.corners_px.tolist(),
            "rectified_camera_matrix": rectified_matrix.reshape(3, 3).tolist(),
            # Canonical solver input: LiDAR z=0 is implicit for this 2-D scanner.
            "lidar_points_xy_m": inliers.tolist(),
            "lidar_line": {
                "normal_lidar_xy": line.normal_xy.tolist(),
                "offset_lidar_m": line.offset_m,
                "direction_lidar_xy": line.direction_xy.tolist(),
                "centroid_lidar_xy_m": line.centroid_xy_m.tolist(),
                "endpoints_lidar_m": [
                    [float(point[0]), float(point[1]), 0.0]
                    for point in line.endpoints_xy_m
                ],
                "inlier_points_lidar_m": [
                    [float(point[0]), float(point[1]), 0.0] for point in inliers
                ],
                "selected_count": int(line.selected_points_xy_m.shape[0]),
                "inlier_count": int(inliers.shape[0]),
                "span_m": line.span_m,
                "orthogonal_rms_m": line.rms_m,
                "initial_mad_m": line.mad_m,
                "mad_threshold_m": line.threshold_m,
            },
            "roi_pixels": {
                "start": list(roi_px[0]),
                "end": list(roi_px[1]),
            },
            "screenshots": {
                "camera": str(Path("screenshots") / camera_name),
                "lidar": str(Path("screenshots") / lidar_name),
                "combined": str(Path("screenshots") / combined_name),
            },
        }
        self.payload["observations"].append(observation)
        self.payload["updated_at_utc"] = datetime.now(timezone.utc).isoformat()
        self.payload["observation_count"] = self.count
        temporary = self.path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self.payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, self.path)
        return pose_id


def _write_png(path: Path, image: np.ndarray) -> None:
    success, encoded = cv2.imencode(".png", image)
    if not success:
        raise OSError(f"failed to encode screenshot: {path}")
    encoded.tofile(path)


def _combine_images(camera: np.ndarray, lidar: np.ndarray) -> np.ndarray:
    height = min(camera.shape[0], lidar.shape[0])
    camera_scaled = cv2.resize(
        camera, (max(1, int(round(camera.shape[1] * height / camera.shape[0]))), height)
    )
    lidar_scaled = cv2.resize(
        lidar, (max(1, int(round(lidar.shape[1] * height / lidar.shape[0]))), height)
    )
    return np.hstack((camera_scaled, lidar_scaled))


class CaptureSession:
    def __init__(
        self,
        node: Any,
        camera_info: RationalCameraInfo,
        rectified_matrix: np.ndarray,
        args: argparse.Namespace,
        store: ObservationStore,
    ) -> None:
        self.node = node
        self.camera_info = camera_info
        self.rectified_matrix = rectified_matrix
        self.args = args
        self.store = store
        self.renderer = TopDownRenderer()
        self.frozen: FrozenObservation | None = None
        self.line: LineFit | None = None
        self.roi_start: tuple[int, int] | None = None
        self.roi_end: tuple[int, int] | None = None
        self.dragging = False
        self.message = "STEP 1: hold board still in camera + LiDAR beam; STEP 2: SPACE"

    def reset(self) -> None:
        self.frozen = None
        self.line = None
        self.roi_start = None
        self.roi_end = None
        self.dragging = False
        self.message = "Reset. STEP 1: hold board still; STEP 2: SPACE"

    def freeze(self) -> None:
        if self.node.latest_image is None:
            raise ValueError(f"no actual messages received on {self.args.image_topic}")
        if len(self.node.scans) < SCAN_MEDIAN_COUNT:
            raise ValueError(
                f"need {SCAN_MEDIAN_COUNT} actual messages on {self.args.scan_topic}; "
                f"received {len(self.node.scans)}"
            )
        image_sample, scan, skew = closest_synchronized_observation(
            tuple(self.node.images), tuple(self.node.scans)
        )
        if self.args.lidar_frame and scan.frame_id != self.args.lidar_frame:
            raise ValueError(
                f"LaserScan frame_id is '{scan.frame_id}', expected '{self.args.lidar_frame}'"
            )
        if image_sample.frame_id != self.args.camera_frame:
            raise ValueError(
                f"Image frame_id is '{image_sample.frame_id}', expected '{self.args.camera_frame}'"
            )
        if image_sample.stamp_s <= 0.0:
            raise ValueError("camera image header stamp is zero")
        if skew > MAX_STAMP_SKEW_S:
            raise ValueError(
                f"camera/LiDAR stamp skew {skew:.3f}s exceeds {MAX_STAMP_SKEW_S:.2f}s; "
                "hold the board still for one second, then press SPACE again"
            )
        rectified, matrix, _ = rectify_alpha_zero(image_sample.image, self.camera_info)
        if not np.allclose(matrix, self.rectified_matrix, rtol=0.0, atol=1.0e-9):
            raise ValueError("rectified camera matrix changed unexpectedly")
        board = estimate_board_pose(
            rectified,
            matrix,
            self.args.board_cols,
            self.args.board_rows,
            self.args.square_size_m,
        )
        annotated = annotate_board(
            rectified,
            board,
            matrix,
            self.args.board_cols,
            self.args.board_rows,
            self.args.square_size_m,
        )
        self.frozen = FrozenObservation(
            rectified_image=rectified,
            annotated_camera=annotated,
            image_stamp_s=image_sample.stamp_s,
            camera_frame_id=image_sample.frame_id,
            board=board,
            scan=scan,
            stamp_skew_s=skew,
        )
        self.line = None
        self.roi_start = None
        self.roi_end = None
        timing = (
            "GOOD" if skew <= PREFERRED_STAMP_SKEW_S else "OK for a stationary board"
        )
        self.message = (
            f"Frozen: PnP {board.reprojection_rms_px:.3f}px, "
            f"skew {skew:.3f}s ({timing}). STEP 3: drag board-line ROI."
        )

    def _finish_roi(self, end: tuple[int, int]) -> None:
        if self.frozen is None or self.roi_start is None:
            return
        self.roi_end = end
        selected = self.renderer.points_in_rectangle(
            self.frozen.scan.points_xy_m, self.roi_start, self.roi_end
        )
        self.line = fit_line_pca_mad(selected)
        result = "PASS" if self.line.accepted else "FAIL"
        action = "STEP 4: press h" if self.line.accepted else "select ROI again"
        self.message = (
            f"Line {result}: "
            f"{self.line.inlier_points_xy_m.shape[0]} inliers, "
            f"span {self.line.span_m:.3f}m, RMS {self.line.rms_m:.4f}m; "
            f"{action}"
        )

    def mouse(self, event: int, x: int, y: int, _flags: int, _param: Any) -> None:
        if self.frozen is None:
            return
        point = (x, y)
        if event == cv2.EVENT_RBUTTONDOWN:
            self.line = None
            self.roi_start = None
            self.roi_end = None
            self.dragging = False
            self.message = "ROI cleared. Click two opposite corners or drag a box."
        elif event == cv2.EVENT_LBUTTONDOWN:
            if self.roi_start is not None and not self.dragging and self.line is None:
                # Second simple click completes a two-click rectangle.
                self._finish_roi(point)
            else:
                self.dragging = True
                self.roi_start = point
                self.roi_end = point
                self.line = None
                self.message = (
                    "ROI first corner set (MAGENTA). Drag, or release and click "
                    "the opposite corner."
                )
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.roi_end = point
        elif event == cv2.EVENT_LBUTTONUP and self.dragging:
            assert self.roi_start is not None
            distance = math.hypot(x - self.roi_start[0], y - self.roi_start[1])
            self.dragging = False
            self.roi_end = point
            if distance >= 6.0:
                self._finish_roi(point)
            else:
                # Keep the anchor so a second click can finish the rectangle.
                self.roi_end = self.roi_start
                self.message = (
                    "First MAGENTA corner saved. Now click the opposite corner "
                    "around only the board line."
                )

    def lidar_status(self) -> list[str]:
        lines = [
            f"Saved {self.store.count}/{self.args.target_views} poses",
            self.message,
        ]
        if self.frozen is None:
            lines.append(
                f"LIVE: image={'yes' if self.node.latest_image is not None else 'NO'}, "
                f"scan buffer={len(self.node.scans)}; YELLOW points are actual ranges; MAGENTA is your ROI"
            )
            lines.append(
                f"Full 360 deg, display radius {self.renderer.metres:.1f}m; + zoom in, - zoom out"
            )
        elif self.line is not None:
            lines.append(
                f"Gate: inliers >=8, span >=0.15m, RMS <=0.015m => "
                f"{'PASS - press h' if self.line.accepted else 'FAIL - select ROI again'}"
            )
        return lines

    def camera_canvas(self) -> np.ndarray:
        if self.frozen is not None:
            return self.frozen.annotated_camera.copy()
        if self.node.latest_image is None:
            canvas = np.full((720, 1280, 3), 25, dtype=np.uint8)
            _put_lines(canvas, [f"Waiting for actual {self.args.image_topic} messages..."])
            return canvas
        try:
            rectified, _, _ = rectify_alpha_zero(
                self.node.latest_image.image, self.camera_info
            )
            _put_lines(
                rectified,
                [
                    "STEP 1: show the full board; LiDAR beam must hit the board",
                    "STEP 2: hold still for 1 second, then press SPACE",
                    f"Saved {self.store.count}/{self.args.target_views} poses",
                ],
                color=(255, 255, 255),
            )
            return rectified
        except Exception as exc:
            canvas = self.node.latest_image.image.copy()
            _put_lines(canvas, [f"IMAGE ERROR: {exc}"], color=(0, 0, 255))
            return canvas

    def lidar_canvas(self) -> np.ndarray:
        points = None
        if self.frozen is not None:
            points = self.frozen.scan.points_xy_m
        elif len(self.node.scans) >= SCAN_MEDIAN_COUNT:
            try:
                points = median_scan(
                    tuple(self.node.scans)[-SCAN_MEDIAN_COUNT:]
                ).points_xy_m
            except ValueError as exc:
                self.message = f"LIVE SCAN ERROR: {exc}"
        return self.renderer.draw(
            points, self.roi_start, self.roi_end, self.line, self.lidar_status()
        )

    def save(self) -> str:
        if self.frozen is None:
            raise ValueError("press SPACE to freeze a pose first")
        if self.line is None or not self.line.accepted:
            raise ValueError("drag a valid LiDAR board ROI before saving")
        if self.roi_start is None or self.roi_end is None:
            raise ValueError("LiDAR ROI is missing")
        screenshot = self.lidar_canvas()
        pose_id = self.store.save(
            self.frozen,
            self.line,
            screenshot,
            self.rectified_matrix,
            (self.roi_start, self.roi_end),
        )
        count = self.store.count
        self.reset()
        self.message = f"Saved {pose_id} ({count}/{self.args.target_views})"
        return pose_id


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Collect synchronized checkerboard planes and RPLIDAR C1 line returns "
            "for camera-to-2D-LiDAR extrinsic calibration."
        )
    )
    parser.add_argument(
        "--camera-info",
        type=Path,
        default=PROJECT_ROOT / "outputs/pi_camera3_wide_intrinsic/camera_info.yaml",
        help="Rational8 camera_info.yaml from intrinsic calibration",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "data/extrinsic",
    )
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--camera-frame", default="camera_optical_frame")
    parser.add_argument("--lidar-frame", default="laser")
    parser.add_argument("--board-cols", type=int, default=8)
    parser.add_argument("--board-rows", type=int, default=9)
    parser.add_argument("--square-size-m", type=float, default=0.070)
    parser.add_argument("--target-views", type=int, default=24)
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument(
        "--resume",
        action="store_true",
        help="append to an existing observations.json; never overwrites observations",
    )
    return parser


def validate_arguments(args: argparse.Namespace) -> None:
    if args.board_cols <= 1 or args.board_rows <= 1:
        raise ValueError("board-cols and board-rows must both exceed one")
    if args.square_size_m <= 0.0:
        raise ValueError("square-size-m must be positive")
    if args.target_views < 3:
        raise ValueError("target-views must be at least three")
    if args.startup_timeout <= 0.0:
        raise ValueError("startup-timeout must be positive")
    if not str(args.image_topic).startswith("/") or not str(args.scan_topic).startswith("/"):
        raise ValueError("image-topic and scan-topic must be absolute ROS topic names")


def _make_ros_node(args: argparse.Namespace) -> Any:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, LaserScan
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python packages are unavailable. Source /opt/ros/jazzy/setup.bash first."
        ) from exc

    rclpy.init(args=None)

    class LiveSensorNode(Node):
        def __init__(self) -> None:
            super().__init__("camera_lidar_extrinsic_observation_collector")
            self.latest_image: ImageSample | None = None
            self.images: deque[ImageSample] = deque(maxlen=SYNC_BUFFER_COUNT)
            self.scans: deque[ScanSample] = deque(maxlen=SYNC_BUFFER_COUNT)
            self.image_count = 0
            self.scan_count = 0
            self.image_error = ""
            self.create_subscription(
                Image, args.image_topic, self._image_callback, qos_profile_sensor_data
            )
            self.create_subscription(
                LaserScan, args.scan_topic, self._scan_callback, qos_profile_sensor_data
            )

        def _image_callback(self, message: Any) -> None:
            self.image_count += 1
            try:
                sample = ImageSample(
                    image=image_message_to_bgr(message),
                    stamp_s=ros_stamp_seconds(message.header.stamp),
                    frame_id=str(message.header.frame_id),
                )
                self.latest_image = sample
                self.images.append(sample)
                self.image_error = ""
            except Exception as exc:
                self.image_error = str(exc)
                self.get_logger().error(f"Rejected image: {exc}")

        def _scan_callback(self, message: Any) -> None:
            self.scan_count += 1
            self.scans.append(
                ScanSample(
                    ranges=np.asarray(message.ranges, dtype=np.float64),
                    angle_min=float(message.angle_min),
                    angle_increment=float(message.angle_increment),
                    range_min=float(message.range_min),
                    range_max=float(message.range_max),
                    stamp_s=ros_stamp_seconds(message.header.stamp),
                    frame_id=str(message.header.frame_id),
                )
            )

    return rclpy, LiveSensorNode()


def _wait_for_actual_topics(
    rclpy_module: Any, node: Any, args: argparse.Namespace
) -> None:
    started = time.monotonic()
    while rclpy_module.ok():
        rclpy_module.spin_once(node, timeout_sec=0.05)
        if node.latest_image is not None and len(node.scans) >= SCAN_MEDIAN_COUNT:
            return
        if time.monotonic() - started >= args.startup_timeout:
            image_publishers = node.count_publishers(args.image_topic)
            scan_publishers = node.count_publishers(args.scan_topic)
            details = (
                f"Timed out waiting for live sensors. {args.image_topic}: "
                f"publishers={image_publishers}, valid_messages="
                f"{1 if node.latest_image is not None else 0}; {args.scan_topic}: "
                f"publishers={scan_publishers}, messages={node.scan_count}."
            )
            if node.image_error:
                details += f" Last image error: {node.image_error}"
            raise RuntimeError(details)
        time.sleep(0.005)


def run(args: argparse.Namespace) -> int:
    validate_arguments(args)
    camera_info_path = args.camera_info.expanduser().resolve()
    camera_info = load_rational_camera_info(camera_info_path)
    width, height = camera_info.image_size
    dummy = np.zeros((height, width, 3), dtype=np.uint8)
    _, rectified_matrix, _ = rectify_alpha_zero(dummy, camera_info)
    store = ObservationStore(
        args.output_dir.expanduser(), camera_info_path, camera_info, args
    )
    rclpy_module, node = _make_ros_node(args)
    print(f"[extrinsic] intrinsics: {camera_info_path}")
    print(f"[extrinsic] output: {store.path}")
    print(f"[extrinsic] waiting for actual {args.image_topic} and {args.scan_topic} ...")
    try:
        _wait_for_actual_topics(rclpy_module, node, args)
        print(
            f"[extrinsic] live topics ready; latest {SCAN_MEDIAN_COUNT} scans available. "
            "SPACE freeze | drag LiDAR ROI | h save | r reset | q quit"
        )
        cv2.namedWindow(CAMERA_WINDOW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(LIDAR_WINDOW, cv2.WINDOW_NORMAL)
        session = CaptureSession(
            node, camera_info, rectified_matrix, args, store
        )
        cv2.setMouseCallback(LIDAR_WINDOW, session.mouse)
        while rclpy_module.ok():
            rclpy_module.spin_once(node, timeout_sec=0.005)
            cv2.imshow(CAMERA_WINDOW, session.camera_canvas())
            cv2.imshow(LIDAR_WINDOW, session.lidar_canvas())
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            try:
                if key == ord(" "):
                    if session.frozen is None:
                        session.freeze()
                    else:
                        session.message = "Already frozen; press r to reset or h after ROI"
                elif key == ord("r"):
                    session.reset()
                elif key in (ord("+"), ord("=")):
                    session.renderer.zoom(0.75)
                    session.message = f"LiDAR zoom: {session.renderer.metres:.1f} m range"
                elif key in (ord("-"), ord("_")):
                    session.renderer.zoom(1.35)
                    session.message = f"LiDAR zoom: {session.renderer.metres:.1f} m range"
                elif key == ord("h"):
                    pose_id = session.save()
                    print(f"[extrinsic] saved {pose_id}: {store.path}")
                    if store.count >= args.target_views:
                        print(f"[extrinsic] target {args.target_views} reached; solving next.")
                        break
            except Exception as exc:
                session.message = f"ERROR: {exc}"
                print(f"[extrinsic] {session.message}", file=sys.stderr)
        print(f"[extrinsic] collected {store.count}/{args.target_views} poses")
        print(f"[extrinsic] observations: {store.path}")
        print(f"[extrinsic] screenshots: {store.screenshot_dir}")
        return 0
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy_module.ok():
            rclpy_module.shutdown()


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
