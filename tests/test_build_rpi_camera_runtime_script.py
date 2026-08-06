"""Static safety and reproducibility checks for the local camera runtime builder.

These tests intentionally never execute the builder: doing so would clone sources,
compile libcamera, and install its artifacts into the repository-local prefix.
"""

from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "build_rpi_camera_runtime.sh"

EXPECTED_REPOSITORY = "https://github.com/raspberrypi/libcamera.git"
EXPECTED_TAG = "v0.7.1+rpt20260609"
EXPECTED_COMMIT = "06c385619acb10bbfb33f52f3abeb8f8c095f42b"


def script_contents() -> str:
    return SCRIPT.read_text(encoding="utf-8")


def single_quoted_assignment(contents: str, variable: str) -> str:
    match = re.search(rf"^{re.escape(variable)}='([^']+)'$", contents, re.MULTILINE)
    assert match is not None, f"missing single-quoted assignment for {variable}"
    return match.group(1)


def test_script_has_valid_bash_syntax():
    result = subprocess.run(
        ["bash", "-n", str(SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_raspberry_pi_fork_tag_and_commit_are_pinned():
    contents = script_contents()

    assert single_quoted_assignment(contents, "libcamera_repository") == EXPECTED_REPOSITORY
    assert single_quoted_assignment(contents, "libcamera_tag") == EXPECTED_TAG
    assert single_quoted_assignment(contents, "libcamera_commit") == EXPECTED_COMMIT
    assert re.fullmatch(r"[0-9a-f]{40}", EXPECTED_COMMIT)

    assert '--branch "${libcamera_tag}"' in contents
    assert 'actual_commit="$(git -C "${source_dir}" rev-parse HEAD)"' in contents
    assert '[[ "${actual_commit}" == "${libcamera_commit}" ]]' in contents


def test_install_prefix_defaults_inside_repository():
    contents = script_contents()

    assert 'project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"' in contents
    assert 'runtime_root="${CAMERA_RUNTIME_ROOT:-${project_root}/.camera_runtime}"' in contents
    assert 'build_root="${CAMERA_BUILD_ROOT:-${project_root}/.camera_build}"' in contents
    assert '"--prefix=${runtime_root}"' in contents
    assert "'--libdir=lib'" in contents


def test_canonical_paths_cannot_escape_to_usr_or_replace_repository_root():
    contents = script_contents()

    assert 'canonical_project_root="$(realpath -m -- "${project_root}")"' in contents
    assert 'canonical_build_root="$(realpath -m -- "${build_root}")"' in contents
    assert 'canonical_runtime_root="$(realpath -m -- "${runtime_root}")"' in contents
    assert 'expected_build_root="${canonical_project_root}/.camera_build"' in contents
    assert 'expected_runtime_root="${canonical_project_root}/.camera_runtime"' in contents
    assert '"${expected_build_root}"|/tmp/*) ;;' in contents
    assert '[[ "${canonical_runtime_root}" == "${expected_runtime_root}" ]]' in contents
    assert 'Runtime root must be exactly' in contents

    project_prefix = str(ROOT.resolve()) + "/"
    assert not str(Path("/usr").resolve()).startswith(project_prefix)


def test_builder_refuses_root_and_a_dirty_pinned_checkout():
    contents = script_contents()

    assert "(( EUID != 0 ))" in contents
    assert "not with sudo or as root" in contents
    assert (
        'source_status="$(git -C "${source_dir}" status '
        '--porcelain --untracked-files=all)"'
    ) in contents
    assert '[[ -z "${source_status}" ]]' in contents
    assert "Pinned libcamera source is modified" in contents


def test_meson_build_enables_only_the_pi5_pisp_pipeline_and_ipa():
    contents = script_contents()

    pipeline_options = re.findall(r"-Dpipelines=([^'\"\s]+)", contents)
    ipa_options = re.findall(r"-Dipas=([^'\"\s]+)", contents)
    assert pipeline_options == ["rpi/pisp"]
    assert ipa_options == ["rpi/pisp"]
    assert "rpi/vc4" not in contents
    assert "'--wrap-mode=nodownload'" in contents
    assert 'ninja -C "${meson_build_dir}" clean' in contents


def test_build_is_locked_to_the_ros_libpisp_abi():
    contents = script_contents()

    assert 'PKG_CONFIG_PATH="${ros_prefix}/lib/pkgconfig"' in contents
    assert "pkg-config --modversion libpisp" in contents
    assert '[[ "${libpisp_version}" == \'1.3.0\' ]]' in contents


def test_builder_has_no_privileged_package_or_system_mutations():
    contents = script_contents()

    forbidden_command = re.compile(
        r"^\s*(?:sudo|apt|apt-get|dpkg|snap|rm|rmdir|purge)(?:\s|$)",
        re.MULTILINE,
    )
    assert forbidden_command.search(contents) is None
    assert "--prefix=/usr" not in contents
    assert "--prefix=/usr/local" not in contents
    assert "DESTDIR=/" not in contents


def test_built_library_validates_the_fixed_underscore_entity_name():
    contents = script_contents()

    assert "strings \"${runtime_library}\"" in contents
    assert "grep -F 'rp1-cfe-fe_image0'" in contents
    assert "rp1-cfe-fe-image0" not in contents


def test_completion_requires_the_full_camera_runtime_and_linkage_checks():
    contents = script_contents()

    required_paths = (
        "lib/libcamera.so.0.7.1",
        "lib/libcamera.so.0.7",
        "lib/libcamera-base.so.0.7.1",
        "lib/libcamera-base.so.0.7",
        "lib/libcamera/ipa/ipa_rpi_pisp.so",
        "lib/libcamera/ipa/ipa_rpi_pisp.so.sign",
        "share/libcamera/ipa/rpi/pisp/imx708_wide.json",
    )
    for path in required_paths:
        assert path in contents

    assert "Library soname: [libcamera.so.0.7]" in contents
    assert 'ldd "${runtime_library}"' in contents
    assert "Built libcamera has an unresolved runtime dependency" in contents


def test_build_manifest_records_inputs_tools_and_artifact_hashes():
    contents = script_contents()

    assert 'manifest="${runtime_root}/BUILD_INFO.txt"' in contents
    assert "libcamera_tag=%s" in contents
    assert "libcamera_commit=%s" in contents
    assert "libpisp_version=%s" in contents
    assert "meson_version=%s" in contents
    assert "compiler_version=%s" in contents
    assert 'sha256sum "${required_runtime_files[@]}"' in contents
