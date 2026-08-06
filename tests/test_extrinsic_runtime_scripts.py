"""Static safety checks for the C1/camera extrinsic runtime wrappers."""

from pathlib import Path
import os
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "capture_lidar_camera_extrinsic.sh"
DISTANCE = ROOT / "run_lidar_camera_distance.sh"
HELPERS = ROOT / "tools/extrinsic/runtime_helpers.sh"


def test_runtime_scripts_are_executable_and_parse_as_bash() -> None:
    scripts = (CAPTURE, DISTANCE, HELPERS)
    for script in scripts:
        assert script.stat().st_mode & 0o111
    subprocess.run(
        ["bash", "-n", *(str(script) for script in scripts)],
        check=True,
        cwd=ROOT,
    )


def test_runtime_is_c1_specific_and_requires_live_sensor_topics() -> None:
    contents = "\n".join(
        path.read_text(encoding="utf-8") for path in (CAPTURE, DISTANCE, HELPERS)
    )
    assert "inno_bringup lidar_only.launch.py" in contents
    assert "RPLIDAR C1" in contents
    assert "sllidar_a1_launch.py" not in contents
    assert "115200" not in contents
    assert "ros2 topic echo" in contents
    assert "sensor_msgs/msg/Image" in contents
    assert "sensor_msgs/msg/LaserScan" in contents
    assert "frame_id is not" in contents
    assert "IMX708" in contents
    assert "rp1-cfe" in contents


def test_wrappers_keep_intrinsic_and_extrinsic_outputs_separate() -> None:
    capture = CAPTURE.read_text(encoding="utf-8")
    distance = DISTANCE.read_text(encoding="utf-8")
    assert "outputs/pi_camera3_wide_intrinsic/camera_info.yaml" in capture
    assert "data/extrinsic" in capture
    assert "outputs/pi_camera3_wide_extrinsic/lidar_camera_extrinsic.yaml" in distance
    assert "distance_screenshots" in distance


def test_help_is_available_without_connected_hardware() -> None:
    capture_help = subprocess.run(
        [str(CAPTURE), "--help"],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    ).stdout
    distance_help = subprocess.run(
        [str(DISTANCE), "--help"],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
    ).stdout
    assert "SPACE freezes" in capture_help
    assert "24+ diverse" in capture_help
    assert "h saves" in capture_help
    collector = (ROOT / "tools/extrinsic/capture_extrinsic_observations.py").read_text(
        encoding="utf-8"
    )
    assert 'key == ord("h")' in collector
    assert 'key == ord("s")' not in collector
    assert "Only objects intersecting the 2D LiDAR scan plane" in distance_help


def test_camera_ros_receives_a_named_copy_of_intrinsics(tmp_path: Path) -> None:
    source = tmp_path / "camera_info.yaml"
    payload = {
        "image_width": 1280,
        "image_height": 720,
        "camera_name": "pi_camera3_wide",
        "distortion_model": "rational_polynomial",
        "distortion_coefficients": {"rows": 1, "cols": 8, "data": [0.0] * 8},
    }
    source.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    environment = os.environ.copy()
    environment["EXTRINSIC_PROJECT_ROOT"] = str(tmp_path)
    completed = subprocess.run(
        [
            "bash",
            "-c",
            "set -euo pipefail; source \"$1\"; "
            "extrinsic_prepare_camera_info \"$2\" 1280 720",
            "bash",
            str(HELPERS),
            str(source),
        ],
        check=True,
        text=True,
        capture_output=True,
        env=environment,
    )
    derived = Path(completed.stdout.strip())
    assert derived.is_file()
    original_payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    derived_payload = yaml.safe_load(derived.read_text(encoding="utf-8"))
    assert original_payload["camera_name"] == "pi_camera3_wide"
    assert derived_payload["camera_name"] == (
        "imx708_wide__base_axi_pcie_120000_rp1_i2c_88000_imx708_1a_1280x720"
    )
    derived_payload["camera_name"] = original_payload["camera_name"]
    assert derived_payload == original_payload
