"""Rational projection, stability, and synthetic calibration tests."""

import cv2
import numpy as np

from tools.calibration.calibration_core import (
    CalibrationConfig,
    CheckerboardConfig,
    DetectedView,
    calibrate_rational,
    create_object_points,
)
from tools.calibration.calibration_validation import check_rational_stability


def _rational_project(points: np.ndarray, K: np.ndarray, D: np.ndarray) -> np.ndarray:
    x, y = points[:, 0], points[:, 1]
    r2 = x * x + y * y
    k1, k2, p1, p2, k3, k4, k5, k6 = D
    radial = (1 + k1 * r2 + k2 * r2**2 + k3 * r2**3) / (
        1 + k4 * r2 + k5 * r2**2 + k6 * r2**3
    )
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.column_stack((K[0, 0] * xd + K[0, 2], K[1, 1] * yd + K[1, 2]))


def test_rational_formula_matches_cv_project_points() -> None:
    K = np.array([[800.0, 0, 640.0], [0, 805.0, 360.0], [0, 0, 1.0]])
    D = np.array([-0.25, 0.08, 0.001, -0.002, -0.01, 0.03, -0.02, 0.004])
    normalized = np.array([[-0.6, -0.3], [0.0, 0.0], [0.5, 0.4], [0.8, -0.45]])
    object_points = np.column_stack((normalized, np.ones(len(normalized))))
    projected, _ = cv2.projectPoints(
        object_points.astype(np.float64),
        np.zeros(3),
        np.zeros(3),
        K,
        D,
    )
    np.testing.assert_allclose(
        _rational_project(normalized, K, D), projected.reshape(-1, 2), atol=1e-8
    )


def test_rational_denominator_singularity_is_detected() -> None:
    K = np.array([[250.0, 0, 320.0], [0, 250.0, 240.0], [0, 0, 1.0]])
    # q(r)=1-r^2 crosses zero inside this image's normalized field.
    D = np.array([0, 0, 0, 0, 0, -1.0, 0, 0], dtype=np.float64)
    result = check_rational_stability(K, D, (640, 480))
    assert result["stable"] is False
    assert result["denominator_sign_change"] or result["denominator_near_zero"]


def test_synthetic_rational_calibration_recovers_intrinsics() -> None:
    rng = np.random.default_rng(7)
    board = CheckerboardConfig(8, 9, 0.04)
    obj = create_object_points(board).astype(np.float64)
    K_true = np.array([[820.0, 0, 640.0], [0, 815.0, 360.0], [0, 0, 1.0]])
    D_true = np.array([-0.12, 0.03, 0.0008, -0.0005, -0.008, 0.01, -0.005, 0.001])
    views = []
    for index in range(24):
        rvec = rng.normal(0, [0.20, 0.20, 0.10]).astype(np.float64)
        tvec = np.array([
            rng.uniform(-0.16, 0.12), rng.uniform(-0.14, 0.10), rng.uniform(0.8, 1.4)
        ])
        corners, _ = cv2.projectPoints(obj, rvec, tvec, K_true, D_true)
        corners += rng.normal(0, 0.05, corners.shape)
        views.append(
            DetectedView(
                path=f"synthetic_{index:02d}.png",
                width=1280,
                height=720,
                corners=corners.astype(np.float32),
            )
        )
    result = calibrate_rational(
        views, board, (1280, 720), CalibrationConfig(minimum_training_views=10)
    )
    assert result.rms < 0.15
    np.testing.assert_allclose(result.camera_matrix[0, 0], K_true[0, 0], rtol=0.04)
    np.testing.assert_allclose(result.camera_matrix[1, 1], K_true[1, 1], rtol=0.04)
    assert result.distortion_coefficients.shape == (8,)
