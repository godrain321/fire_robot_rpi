"""Pure-function tests for calibrated 2D LiDAR click ranging."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from tools.extrinsic.lidar_camera_distance import (
    ProjectedScan,
    load_lidar_camera_extrinsic,
    load_rational_camera_info,
    measurement_near_click,
    project_laser_scan,
    validate_pair,
)


BASE_ROTATION = np.array(
    [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
)


def _camera_yaml(path: Path) -> Path:
    payload = {
        "image_width": 100,
        "image_height": 80,
        "camera_name": "pi_camera3_wide",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [100.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0],
        },
        "distortion_model": "rational_polynomial",
        "distortion_coefficients": {
            "rows": 1,
            "cols": 8,
            "data": [0.0] * 8,
        },
    }
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return path


def _extrinsic_payload() -> dict:
    transform = np.eye(4)
    transform[:3, :3] = BASE_ROTATION
    return {
        "schema_version": 1,
        "transform_convention": (
            "p_camera = R_camera_lidar * p_lidar + t_camera_lidar_m"
        ),
        "frames": {"camera": "camera_optical_frame", "lidar": "laser"},
        "T_camera_lidar": transform.tolist(),
        "R_camera_lidar": BASE_ROTATION.tolist(),
        "t_camera_lidar_m": [0.0, 0.0, 0.0],
        "quality": {
            "passed": True,
            "pose_count": 24,
            "normal_rank": 3,
            "jacobian_rank": 6,
            "jacobian_condition_number": 12.0,
            "rmse_m": 0.004,
            "median_abs_residual_m": 0.003,
            "max_abs_residual_m": 0.012,
        },
    }


def _extrinsic_yaml(path: Path, payload: dict | None = None) -> Path:
    path.write_text(
        yaml.safe_dump(payload or _extrinsic_payload(), sort_keys=False),
        encoding="utf-8",
    )
    return path


def _stamp(seconds: float) -> SimpleNamespace:
    sec = int(seconds)
    return SimpleNamespace(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


def test_strict_intrinsic_and_extrinsic_loading(tmp_path: Path) -> None:
    camera = load_rational_camera_info(_camera_yaml(tmp_path / "camera.yaml"))
    extrinsic = load_lidar_camera_extrinsic(
        _extrinsic_yaml(tmp_path / "extrinsic.yaml")
    )
    assert camera.distortion_coefficients.size == 8
    assert extrinsic.lidar_frame == "laser"
    assert np.allclose(extrinsic.rotation, BASE_ROTATION)

    rejected = _extrinsic_payload()
    rejected["quality"]["passed"] = False
    with pytest.raises(ValueError, match="passed"):
        load_lidar_camera_extrinsic(
            _extrinsic_yaml(tmp_path / "rejected.yaml", rejected)
        )


def test_projection_and_click_report_three_distances(tmp_path: Path) -> None:
    extrinsic = load_lidar_camera_extrinsic(
        _extrinsic_yaml(tmp_path / "extrinsic.yaml")
    )
    scan = SimpleNamespace(
        ranges=[1.0],
        angle_min=0.0,
        angle_increment=0.01,
        range_min=0.05,
        range_max=12.0,
    )
    projected = project_laser_scan(
        scan,
        extrinsic,
        np.array([[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]]),
        (100, 80),
    )
    assert projected.pixels.shape == (1, 2)
    assert np.allclose(projected.pixels[0], (50.0, 40.0))
    measured = measurement_near_click(projected, (51.0, 40.0), 20.0)
    assert measured is not None
    assert measured.lidar_range_cm == pytest.approx(100.0)
    assert measured.camera_forward_z_cm == pytest.approx(100.0)
    assert measured.camera_euclidean_cm == pytest.approx(100.0)
    assert measurement_near_click(projected, (90.0, 70.0), 5.0) is None


def test_click_prefers_foreground_cluster_over_background() -> None:
    projected = ProjectedScan(
        pixels=np.array([[52.0, 40.0], [50.0, 40.0], [51.0, 41.0]]),
        lidar_ranges_m=np.array([1.0, 3.0, 1.04]),
        camera_points_m=np.array(
            [[0.02, 0.0, 1.0], [0.0, 0.0, 3.0], [0.01, 0.01, 1.04]]
        ),
    )
    measured = measurement_near_click(projected, (50.0, 40.0), 20.0)
    assert measured is not None
    assert measured.lidar_range_cm < 110.0
    assert measured.support_count == 2


def test_timing_gate_rejects_skew_and_stale_data() -> None:
    image = SimpleNamespace(
        width=100,
        height=80,
        header=SimpleNamespace(frame_id="camera_optical_frame", stamp=_stamp(10.0)),
    )
    scan = SimpleNamespace(
        header=SimpleNamespace(frame_id="laser", stamp=_stamp(10.05)),
        angle_increment=0.01,
        range_min=0.05,
        range_max=12.0,
    )
    assert validate_pair(
        image, scan, 10.1, (100, 80), "camera_optical_frame", "laser"
    ).valid
    scan.header.stamp = _stamp(10.2)
    assert not validate_pair(
        image, scan, 10.21, (100, 80), "camera_optical_frame", "laser"
    ).valid
    scan.header.stamp = _stamp(10.05)
    assert validate_pair(
        image, scan, 11.0, (100, 80), "camera_optical_frame", "laser"
    ).reason.startswith("STALE")
