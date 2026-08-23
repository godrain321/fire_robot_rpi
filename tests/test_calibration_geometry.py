"""Geometry, input validation, splitting, and robust-statistics tests."""

from pathlib import Path

import cv2
import numpy as np
import pytest

from tools.calibration.calibration_core import (
    CheckerboardConfig,
    DetectedView,
    compute_mad_statistics,
    create_object_points,
    detect_checkerboards,
    discover_images,
    pose_diverse_split,
)


def _view(name: str, descriptor: list[float]) -> DetectedView:
    corners = np.zeros((72, 1, 2), dtype=np.float32)
    return DetectedView(
        path=Path(name),
        width=1280,
        height=720,
        read_success=True,
        detection_success=True,
        corners=corners,
        corner_count=72,
        center=(640.0, 360.0),
        area_ratio=0.1,
        blur_score=100.0,
        pose_descriptor=np.asarray(descriptor, dtype=np.float64),
    )


def test_object_points_count_order_and_units() -> None:
    board = CheckerboardConfig(8, 9, 0.070)
    points = create_object_points(board)
    assert points.shape == (72, 3)
    assert points.dtype == np.float32
    np.testing.assert_allclose(points[0], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(points[1], [0.070, 0.0, 0.0])
    np.testing.assert_allclose(points[8], [0.0, 0.070, 0.0])
    np.testing.assert_allclose(points[-1], [0.49, 0.56, 0.0])
    assert np.all(points[:, 2] == 0.0)


def test_pose_diverse_split_is_reproducible() -> None:
    views = [
        _view(f"view_{index:02d}.png", [index / 20, (index % 4) / 4, 0.1 + index / 100, index / 10, 1.0, 1.0, index / 30])
        for index in range(20)
    ]
    first_train, first_validation = pose_diverse_split(views, 0.2, 42)
    second_train, second_validation = pose_diverse_split(views, 0.2, 42)
    assert [v.path for v in first_train] == [v.path for v in second_train]
    assert [v.path for v in first_validation] == [v.path for v in second_validation]
    assert len(first_validation) == 4


def test_mad_threshold() -> None:
    statistics = compute_mad_statistics(
        np.array([0.9, 1.0, 1.1, 1.0, 5.0]), mad_multiplier=3.0
    )
    assert statistics.median == pytest.approx(1.0)
    assert statistics.mad == pytest.approx(0.1)
    assert statistics.threshold == pytest.approx(1.0 + 3.0 * 1.4826 * 0.1)


def test_different_resolution_is_rejected_by_default(tmp_path: Path) -> None:
    image_a = np.zeros((240, 320, 3), dtype=np.uint8)
    image_b = np.zeros((480, 640, 3), dtype=np.uint8)
    path_a, path_b = tmp_path / "a.png", tmp_path / "b.png"
    assert cv2.imwrite(str(path_a), image_a)
    assert cv2.imwrite(str(path_b), image_b)
    views, image_size = detect_checkerboards(
        [path_a, path_b], CheckerboardConfig(8, 9, 0.070)
    )
    assert image_size == (320, 240)
    assert views[1].exclusion_reason == "resolution_mismatch"


def test_different_resolution_strict_mode_fails(tmp_path: Path) -> None:
    path_a, path_b = tmp_path / "a.png", tmp_path / "b.png"
    cv2.imwrite(str(path_a), np.zeros((240, 320, 3), dtype=np.uint8))
    cv2.imwrite(str(path_b), np.zeros((480, 640, 3), dtype=np.uint8))
    with pytest.raises(ValueError, match="resolution"):
        detect_checkerboards(
            [path_a, path_b],
            CheckerboardConfig(8, 9, 0.070),
            strict_resolution=True,
        )


def test_empty_image_glob_fails(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="no input images"):
        discover_images(str(tmp_path / "*.png"))
