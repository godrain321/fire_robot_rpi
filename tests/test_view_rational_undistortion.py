"""Tests for the Rational undistortion comparison."""

from argparse import Namespace
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from tools.calibration.view_rational_undistortion import (
    load_rational_camera_info,
    make_side_by_side,
    run,
    undistort_rational,
)


def _payload() -> dict:
    return {
        "image_width": 96,
        "image_height": 64,
        "camera_name": "pi_camera3_wide",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [80.0, 0.0, 47.5, 0.0, 81.0, 31.5, 0.0, 0.0, 1.0],
        },
        "distortion_model": "rational_polynomial",
        "distortion_coefficients": {
            "rows": 1,
            "cols": 8,
            "data": [0.1, -0.03, 0.001, -0.002, 0.005, 0.01, -0.004, 0.001],
        },
    }


def _write(path: Path, payload: dict | None = None) -> Path:
    path.write_text(
        yaml.safe_dump(payload or _payload(), sort_keys=False), encoding="utf-8"
    )
    return path


def _image() -> np.ndarray:
    image = np.zeros((64, 96, 3), dtype=np.uint8)
    image[:, ::8] = 255
    image[::8, :] = 255
    return image


def test_load_requires_rational_model_and_exact_eight_coefficients(
    tmp_path: Path,
) -> None:
    path = _write(tmp_path / "camera_info.yaml")
    assert load_rational_camera_info(path).distortion_coefficients.size == 8
    invalid = _payload()
    invalid["distortion_model"] = "plumb_bob"
    _write(path, invalid)
    with pytest.raises(ValueError, match="rational_polynomial"):
        load_rational_camera_info(path)


def test_undistortion_resolution_and_comparison(tmp_path: Path) -> None:
    info = load_rational_camera_info(_write(tmp_path / "camera_info.yaml"))
    original = _image()
    corrected, new_matrix, roi = undistort_rational(original, info, 0.0)
    comparison = make_side_by_side(original, corrected, 0.0)
    assert corrected.shape == original.shape
    assert new_matrix.shape == (3, 3)
    assert len(roi) == 4
    assert comparison.shape[1] == original.shape[1] * 2
    with pytest.raises(ValueError, match="resolution"):
        undistort_rational(original[:32], info, 0.0)


def test_headless_run_saves_both_files(tmp_path: Path) -> None:
    calibration = _write(tmp_path / "camera_info.yaml")
    source = tmp_path / "calib_007.png"
    assert cv2.imwrite(str(source), _image())
    (tmp_path / "accepted_views.txt").write_text(
        str(source) + "\n", encoding="utf-8"
    )
    paths = run(
        Namespace(
            calibration=calibration,
            image=None,
            accepted_views=None,
            output_dir=tmp_path / "comparison",
            alpha=0.0,
            no_display=True,
            display_max_width=1920,
        )
    )
    assert paths["original"] == source.resolve()
    assert paths["corrected"].name == "undistorted_calib_007_alpha_0p00.png"
    assert (
        paths["comparison"].name
        == "original_vs_undistorted_calib_007_alpha_0p00.png"
    )
    assert cv2.imread(str(paths["corrected"])).shape == (64, 96, 3)
    assert cv2.imread(str(paths["comparison"])).shape[1] == 192
