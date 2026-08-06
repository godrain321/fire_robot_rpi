"""ROS CameraInfo serialization tests."""

from pathlib import Path

import numpy as np
import yaml

from tools.calibration.calibration_core import write_camera_info_yaml


def test_camera_info_fields_and_rational_coefficient_order(tmp_path: Path) -> None:
    K = np.array([[800.0, 0, 640.0], [0, 805.0, 360.0], [0, 0, 1.0]])
    D = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])
    output = tmp_path / "camera_info.yaml"
    write_camera_info_yaml(output, "pi_camera3_wide", (1280, 720), K, D)
    data = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert data["image_width"] == 1280
    assert data["image_height"] == 720
    assert data["camera_name"] == "pi_camera3_wide"
    assert data["distortion_model"] == "rational_polynomial"
    assert data["distortion_coefficients"]["rows"] == 1
    assert data["distortion_coefficients"]["cols"] == 8
    assert data["distortion_coefficients"]["data"] == D.tolist()
    assert data["rectification_matrix"]["data"] == np.eye(3).reshape(-1).tolist()
    expected_projection = np.zeros((3, 4))
    expected_projection[:, :3] = K
    assert data["projection_matrix"]["data"] == expected_projection.reshape(-1).tolist()
