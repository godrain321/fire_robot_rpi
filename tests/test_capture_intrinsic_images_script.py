"""Safety and contract tests for the guided intrinsic-image capture wrapper."""

from pathlib import Path
import shlex
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "capture_intrinsic_images.sh"


def run_sourced(
    command: str,
    timeout: float | None = None,
) -> subprocess.CompletedProcess[str]:
    """Source the script as a function library and run one shell expression."""
    return subprocess.run(
        [
            "bash",
            "-c",
            f"source {shlex.quote(str(SCRIPT))}; {command}",
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def test_script_has_valid_bash_syntax():
    result = subprocess.run(
        ["bash", "-n", str(SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_script_does_not_invoke_any_calibrator():
    contents = SCRIPT.read_text(encoding="utf-8")
    assert "calibrate_fisheye" not in contents
    assert "calibrate_checkerboard_rational.py" not in contents
    assert "calibrate_camera.sh" not in contents
    assert "cam -l" not in contents
    assert "/usr/bin/cam" not in contents


def test_empty_destination_is_safe_for_new_capture(tmp_path):
    result = run_sourced(
        "validate_capture_directory "
        f"{shlex.quote(str(tmp_path))} false 80"
    )
    assert result.returncode == 0, result.stderr


def test_new_capture_refuses_to_mix_with_existing_files(tmp_path):
    (tmp_path / "calib_000.png").touch()
    result = run_sourced(
        "validate_capture_directory "
        f"{shlex.quote(str(tmp_path))} false 80"
    )
    assert result.returncode != 0
    assert "not empty" in result.stderr


def test_resume_accepts_only_contiguous_capture_sequence(tmp_path):
    (tmp_path / "calib_000.png").touch()
    (tmp_path / "calib_001.png").touch()
    (tmp_path / "capture_stats.csv").touch()
    result = run_sourced(
        "validate_capture_directory "
        f"{shlex.quote(str(tmp_path))} true 80"
    )
    assert result.returncode == 0, result.stderr


def test_resume_refuses_gap_that_guided_node_could_overwrite(tmp_path):
    (tmp_path / "calib_000.png").touch()
    (tmp_path / "calib_002.png").touch()
    result = run_sourced(
        "validate_capture_directory "
        f"{shlex.quote(str(tmp_path))} true 80"
    )
    assert result.returncode != 0
    assert "unbroken sequence" in result.stderr


def test_resume_refuses_unrelated_input_images(tmp_path):
    (tmp_path / "phone_photo.png").touch()
    result = run_sourced(
        "validate_capture_directory "
        f"{shlex.quote(str(tmp_path))} true 80"
    )
    assert result.returncode != 0


def test_imx708_detection_reads_i2c_name_files(tmp_path):
    sensor_dir = tmp_path / "10-001a"
    sensor_dir.mkdir()
    (sensor_dir / "name").write_text("imx708\n", encoding="utf-8")
    found = run_sourced(
        "i2c_tree_has_imx708 " + shlex.quote(str(tmp_path))
    )
    (sensor_dir / "name").write_text("another_sensor\n", encoding="utf-8")
    missing = run_sourced(
        "i2c_tree_has_imx708 " + shlex.quote(str(tmp_path))
    )
    assert found.returncode == 0
    assert missing.returncode != 0


def test_rp1_cfe_detection_uses_media_model_attribute():
    found = run_sourced(
        "media_attributes_have_rp1_cfe "
        + shlex.quote('ATTR{model}=="rp1-cfe"')
    )
    missing = run_sourced(
        "media_attributes_have_rp1_cfe "
        + shlex.quote('ATTR{model}=="pispbe"')
    )
    assert found.returncode == 0
    assert missing.returncode != 0


def test_runtime_requires_matching_libcamera_07_pair(tmp_path):
    lib_dir = tmp_path / "lib"
    lib_dir.mkdir()
    (lib_dir / "libcamera.so.0.7").touch()
    missing_base = run_sourced(
        "runtime_has_required_libcamera " + shlex.quote(str(lib_dir))
    )
    (lib_dir / "libcamera-base.so.0.7").touch()
    complete = run_sourced(
        "runtime_has_required_libcamera " + shlex.quote(str(lib_dir))
    )
    assert missing_base.returncode != 0
    assert complete.returncode == 0


def test_runtime_is_prepended_to_ld_library_path(tmp_path):
    lib_dir = tmp_path / "lib"
    lib_dir.mkdir()
    (lib_dir / "libcamera.so.0.7").touch()
    (lib_dir / "libcamera-base.so.0.7").touch()
    command = (
        f"export CAMERA_RUNTIME_ROOT={shlex.quote(str(tmp_path))}; "
        "export LD_LIBRARY_PATH=/system/lib; "
        "activate_camera_runtime >/dev/null; "
        f"[[ $LD_LIBRARY_PATH == {shlex.quote(str(lib_dir))}:/system/lib ]]"
    )
    result = run_sourced(command)
    assert result.returncode == 0, result.stderr


def test_frame_probe_requires_a_real_positive_image_width():
    received = run_sourced(
        "frame_probe_received_image "
        + shlex.quote("1280\n---")
        + " 1280"
    )
    timed_out = run_sourced("frame_probe_received_image '' 1280")
    warning_number = run_sourced(
        "frame_probe_received_image "
        + shlex.quote("warning: libcamera 0.7 timed out after 20 seconds")
        + " 1280"
    )
    wrong_width = run_sourced("frame_probe_received_image '640' 1280")
    assert received.returncode == 0
    assert timed_out.returncode != 0
    assert warning_number.returncode != 0
    assert wrong_width.returncode != 0


def test_cleanup_escalates_stubborn_process_group_without_hanging():
    command = r"""
setsid bash -c 'trap "" INT TERM; while :; do sleep 1; done' &
stubborn_pid=$!
for _ in {1..40}; do
  process_group_has_live_members "$stubborn_pid" && break
  sleep 0.05
done
process_group_has_live_members "$stubborn_pid" || exit 2
export CAPTURE_INT_GRACE_SECONDS=1
export CAPTURE_TERM_GRACE_SECONDS=1
export CAPTURE_KILL_GRACE_SECONDS=1
started=$SECONDS
terminate_launch_process_group "$stubborn_pid" "$stubborn_pid"
elapsed=$((SECONDS - started))
(( elapsed <= 5 ))
! process_group_has_live_members "$stubborn_pid"
"""
    result = run_sourced(command, timeout=8)
    assert result.returncode == 0, result.stderr
    assert "SIGTERM" in result.stderr
    assert "SIGKILL" in result.stderr
