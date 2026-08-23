"""Synthetic regression tests for 2-D lidar/camera extrinsic calibration."""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from tools.extrinsic.calibrate_lidar_camera_extrinsic import main
from tools.extrinsic.capture_extrinsic_observations import (
    CaptureSession,
    ImageSample,
    ScanSample,
    TopDownRenderer,
    closest_synchronized_observation,
    median_scan,
)
from tools.extrinsic.extrinsic_core import (
    DegenerateGeometryError,
    ExtrinsicCalibrationError,
    ObservationSet,
    PlaneObservation,
    SolverConfig,
    TRANSFORM_CONVENTION,
    calibrate_extrinsic,
    load_rational_camera_info,
)


def _truth() -> tuple[np.ndarray, np.ndarray]:
    rotation, _ = cv2.Rodrigues(np.array([0.17, -0.23, 0.09], dtype=np.float64))
    translation = np.array([0.075, -0.042, 0.128], dtype=np.float64)
    return rotation, translation


def _synthetic_observations(
    count: int = 32,
    *,
    degenerate_normals: bool = False,
) -> ObservationSet:
    rotation, translation = _truth()
    observations: list[PlaneObservation] = []
    for index in range(count):
        if degenerate_normals:
            normal = np.array([0.0, 0.0, 1.0])
        else:
            phase = 2.0 * np.pi * index / count
            normal = np.array(
                [
                    0.58 * np.sin(phase),
                    0.48 * np.cos(1.7 * phase + 0.15),
                    0.78 + 0.16 * np.sin(2.3 * phase),
                ]
            )
            normal /= np.linalg.norm(normal)
        center = np.array(
            [
                0.65 + 0.28 * np.sin(1.3 * index),
                -0.25 + 0.33 * np.cos(0.7 * index),
            ]
        )
        line_normal_lidar = rotation[:, :2].T @ normal
        direction = np.array([-line_normal_lidar[1], line_normal_lidar[0]])
        direction /= np.linalg.norm(direction)
        coordinates = np.linspace(-0.22, 0.22, 11)
        points = center + coordinates[:, None] * direction
        center_3d = np.array([center[0], center[1], 0.0])
        offset = -float(normal @ (rotation @ center_3d + translation))
        observations.append(
            PlaneObservation(
                pose_id=f"pose_{index:03d}",
                board_normal_camera=normal,
                board_offset_camera_m=offset,
                lidar_points_xy_m=points,
            )
        )
    return ObservationSet(tuple(observations))


def _write_camera_info(path: Path, coefficient_count: int = 8) -> None:
    payload = {
        "image_width": 1280,
        "image_height": 720,
        "camera_name": "pi_camera3_wide",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [820.0, 0.0, 640.0, 0.0, 818.0, 360.0, 0.0, 0.0, 1.0],
        },
        "distortion_model": "rational_polynomial",
        "distortion_coefficients": {
            "rows": 1,
            "cols": coefficient_count,
            "data": [0.01] * coefficient_count,
        },
    }
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _write_observations_json(path: Path, observations: ObservationSet) -> None:
    payload = {
        "schema_version": 1,
        "transform_convention": TRANSFORM_CONVENTION,
        "frames": {"camera": "camera_optical_frame", "lidar": "laser"},
        "observations": [
            {
                "pose_id": item.pose_id,
                "board_normal_camera": item.board_normal_camera.tolist(),
                "board_offset_camera_m": item.board_offset_camera_m,
                "lidar_points_xy_m": item.lidar_points_xy_m.tolist(),
            }
            for item in observations.observations
        ],
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_synthetic_transform_is_recovered() -> None:
    observations = _synthetic_observations()
    result = calibrate_extrinsic(observations, SolverConfig(min_views=20))
    expected_rotation, expected_translation = _truth()
    rotation_error, _ = cv2.Rodrigues(
        result.rotation_camera_lidar @ expected_rotation.T
    )

    assert np.linalg.norm(rotation_error) < 1.0e-7
    assert np.linalg.norm(result.translation_camera_lidar_m - expected_translation) < 1.0e-7
    assert result.rmse_m < 1.0e-9
    assert result.normal_rank == 3
    assert result.jacobian_rank == 6
    assert result.jacobian_condition_number < 1.0e8
    np.testing.assert_allclose(
        result.transform_camera_lidar[3], [0.0, 0.0, 0.0, 1.0], atol=0.0
    )


def test_rank_deficient_board_normals_are_rejected() -> None:
    with pytest.raises(DegenerateGeometryError, match="normals have rank < 3"):
        calibrate_extrinsic(
            _synthetic_observations(degenerate_normals=True),
            SolverConfig(min_views=20),
        )



def test_excessive_per_pose_rmse_is_rejected() -> None:
    original = _synthetic_observations()
    items = list(original.observations)
    first = items[0]
    items[0] = PlaneObservation(
        pose_id=first.pose_id,
        board_normal_camera=first.board_normal_camera,
        board_offset_camera_m=first.board_offset_camera_m + 0.010,
        lidar_points_xy_m=first.lidar_points_xy_m,
    )
    with pytest.raises(ExtrinsicCalibrationError, match="pose .* RMSE"):
        calibrate_extrinsic(
            ObservationSet(tuple(items)),
            SolverConfig(max_pose_rmse_m=1.0e-4),
        )


def test_minimum_view_requirement_cannot_be_lowered() -> None:
    with pytest.raises(ValueError, match="at least 20"):
        SolverConfig(min_views=19)



def test_camera_info_requires_exact_rational_eight_coefficients(tmp_path: Path) -> None:
    valid = tmp_path / "valid.yaml"
    _write_camera_info(valid)
    loaded = load_rational_camera_info(valid)
    assert loaded.distortion_coefficients.shape == (8,)

    invalid = tmp_path / "invalid.yaml"
    _write_camera_info(invalid, coefficient_count=5)
    with pytest.raises(ValueError, match="rows=1 and cols=8"):
        load_rational_camera_info(invalid)


def test_cli_writes_strict_yaml_json_and_report(tmp_path: Path) -> None:
    camera_info = tmp_path / "camera_info.yaml"
    observations_path = tmp_path / "observations.json"
    output_dir = tmp_path / "result"
    _write_camera_info(camera_info)
    _write_observations_json(observations_path, _synthetic_observations())

    return_code = main(
        [
            "--observations",
            str(observations_path),
            "--camera-info",
            str(camera_info),
            "--output-dir",
            str(output_dir),
        ]
    )
    assert return_code == 0
    result_yaml = output_dir / "lidar_camera_extrinsic.yaml"
    result_json = output_dir / "extrinsic_calibration_result.json"
    report = output_dir / "extrinsic_calibration_report.txt"
    assert result_yaml.is_file() and result_json.is_file() and report.is_file()

    payload = yaml.safe_load(result_yaml.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1
    assert payload["transform_convention"] == TRANSFORM_CONVENTION
    assert payload["frames"] == {"camera": "camera_optical_frame", "lidar": "laser"}
    assert np.asarray(payload["T_camera_lidar"]).shape == (4, 4)
    assert payload["quality"]["passed"] is True
    assert payload["quality"]["normal_rank"] == 3
    assert payload["quality"]["jacobian_rank"] == 6
    assert "per_pose_rmse_m" in json.loads(result_json.read_text(encoding="utf-8"))
    assert payload["quality"]["max_allowed_rmse_m"] == 0.020
    assert payload["quality"]["max_allowed_pose_rmse_m"] == 0.030
    assert "EXTRINSIC CALIBRATION REPORT" in report.read_text(encoding="utf-8")


def _scan_sample(stamp_s: float) -> ScanSample:
    return ScanSample(
        ranges=np.full(360, 1.0, dtype=np.float64),
        angle_min=-np.pi,
        angle_increment=2.0 * np.pi / 360.0,
        range_min=0.05,
        range_max=16.0,
        stamp_s=stamp_s,
        frame_id="laser",
    )


def test_buffered_sync_avoids_startup_latest_scan_skew() -> None:
    image = ImageSample(
        image=np.zeros((2, 2, 3), dtype=np.uint8),
        stamp_s=10.0,
        frame_id="camera_optical_frame",
    )
    scans = tuple(_scan_sample(9.8 + 0.1 * index) for index in range(11))

    selected_image, selected_scan, skew = closest_synchronized_observation(
        (image,), scans
    )

    assert selected_image is image
    assert skew < 1.0e-9
    assert selected_scan.reference_stamp_s == pytest.approx(10.0)
    assert selected_scan.reference_stamp_s != pytest.approx(scans[-1].stamp_s)


def test_five_scan_reference_is_middle_timestamp() -> None:
    scans = tuple(_scan_sample(20.0 + 0.1 * index) for index in range(5))
    assert median_scan(scans).reference_stamp_s == pytest.approx(20.2)


def test_lidar_renderer_keeps_front_and_rear_points_visible() -> None:
    renderer = TopDownRenderer(metres=4.0)
    pixels = renderer.xy_to_uv(np.array([[3.0, 0.0], [-3.0, 0.0]]))
    assert np.all(pixels[:, 0] >= 0.0)
    assert np.all(pixels[:, 0] < renderer.width)
    assert np.all(pixels[:, 1] >= 0.0)
    assert np.all(pixels[:, 1] < renderer.height)


def test_two_click_roi_selection_accepts_board_line() -> None:
    session = object.__new__(CaptureSession)
    session.renderer = TopDownRenderer(metres=4.0)
    points = np.column_stack(
        (np.linspace(0.8, 1.2, 16), np.zeros(16, dtype=np.float64))
    )
    session.frozen = SimpleNamespace(
        scan=SimpleNamespace(points_xy_m=points)
    )
    session.line = None
    session.roi_start = None
    session.roi_end = None
    session.dragging = False
    session.message = ""
    pixels = session.renderer.xy_to_uv(points)
    first = tuple(np.floor(np.min(pixels, axis=0) - 5.0).astype(int))
    second = tuple(np.ceil(np.max(pixels, axis=0) + 5.0).astype(int))

    session.mouse(cv2.EVENT_LBUTTONDOWN, *first, 0, None)
    session.mouse(cv2.EVENT_LBUTTONUP, *first, 0, None)
    assert session.line is None
    assert "opposite corner" in session.message

    session.mouse(cv2.EVENT_LBUTTONDOWN, *second, 0, None)
    assert session.line is not None
    assert session.line.accepted
    assert "PASS" in session.message
