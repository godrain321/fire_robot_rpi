"""Unit tests for the guided Rational-dataset checkerboard detector."""

import cv2
from fire_robot_camera_calibration.guided_capture_node import GuidedCaptureNode
import numpy as np
import pytest


class DetectorOnlyNode:
    """Minimal receiver for calling the node's pure detector method."""

    pattern_size = (8, 9)
    board_cols = 8
    board_rows = 9


def valid_corners() -> np.ndarray:
    """Return 8x9 image points fully inside a 640x480 test image."""
    xs, ys = np.meshgrid(
        np.linspace(80.0, 560.0, 8, dtype=np.float32),
        np.linspace(60.0, 420.0, 9, dtype=np.float32),
    )
    return np.column_stack((xs.ravel(), ys.ravel())).reshape(-1, 1, 2)


def test_detector_matches_offline_sb_flags(monkeypatch):
    gray = np.zeros((480, 640), dtype=np.uint8)
    corners = valid_corners()
    observed = {}

    def fake_sb(image, pattern_size, *, flags):
        observed['image'] = image
        observed['pattern_size'] = pattern_size
        observed['flags'] = flags
        return True, corners

    monkeypatch.setattr(cv2, 'findChessboardCornersSB', fake_sb)
    result = GuidedCaptureNode._detect_corners(DetectorOnlyNode(), gray)

    expected_flags = (
        cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_EXHAUSTIVE
        | cv2.CALIB_CB_ACCURACY
    )
    assert observed['image'] is gray
    assert observed['pattern_size'] == (8, 9)
    assert observed['flags'] == expected_flags
    assert result is not None
    assert result.shape == (72, 1, 2)
    assert result.dtype == np.float32
    assert result.flags.c_contiguous


@pytest.mark.parametrize(
    'corners',
    [
        valid_corners()[:-1],
        np.full((72, 1, 2), np.nan, dtype=np.float32),
        np.full((72, 1, 2), 1000.0, dtype=np.float32),
    ],
    ids=['incomplete', 'non_finite', 'outside_image'],
)
def test_detector_rejects_views_offline_pipeline_would_reject(
    monkeypatch,
    corners,
):
    gray = np.zeros((480, 640), dtype=np.uint8)
    monkeypatch.setattr(
        cv2,
        'findChessboardCornersSB',
        lambda *_args, **_kwargs: (True, corners),
    )

    result = GuidedCaptureNode._detect_corners(DetectorOnlyNode(), gray)
    assert result is None
