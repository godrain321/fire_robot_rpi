"""Regression tests for safe same-command intrinsic capture restart."""

from pathlib import Path
import shlex
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "capture_intrinsic_images.sh"


def _validate(directory: Path, resume: bool = False) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "bash",
            "-c",
            (
                f"source {shlex.quote(str(SCRIPT))}; "
                "validate_capture_directory "
                f"{shlex.quote(str(directory))} "
                f"{'true' if resume else 'false'} 80"
            ),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


def test_header_only_stats_file_is_a_safe_fresh_restart(tmp_path: Path) -> None:
    (tmp_path / "capture_stats.csv").write_text("filename,unix_time\n", encoding="utf-8")
    result = _validate(tmp_path)
    assert result.returncode == 0, result.stderr
    assert "Safe restart" in result.stdout


def test_same_command_auto_resumes_its_contiguous_capture_set(tmp_path: Path) -> None:
    (tmp_path / "capture_stats.csv").write_text("filename,unix_time\n", encoding="utf-8")
    (tmp_path / "calib_000.png").touch()
    (tmp_path / "calib_001.png").touch()
    result = _validate(tmp_path)
    assert result.returncode == 0, result.stderr
    assert "next file is calib_002.png" in result.stdout


def test_same_command_still_rejects_gaps_and_unrelated_files(tmp_path: Path) -> None:
    (tmp_path / "capture_stats.csv").touch()
    (tmp_path / "calib_001.png").touch()
    gap = _validate(tmp_path)
    assert gap.returncode != 0
    assert "unbroken sequence" in gap.stderr

    (tmp_path / "calib_001.png").unlink()
    (tmp_path / "phone_photo.png").touch()
    unrelated = _validate(tmp_path)
    assert unrelated.returncode != 0
    assert "unrelated entry" in unrelated.stderr
