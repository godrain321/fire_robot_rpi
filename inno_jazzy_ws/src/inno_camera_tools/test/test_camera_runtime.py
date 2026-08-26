from pathlib import Path

from inno_camera_tools.camera_runtime import camera_runtime_environment


def _make_runtime(root: Path) -> Path:
    runtime = root / '.camera_runtime'
    for relative in (
        'lib/libcamera.so.0.7',
        'lib/libcamera/ipa/ipa_rpi_pisp.so',
        'libexec/libcamera/raspberrypi_ipa_proxy',
        'share/libcamera/ipa/rpi/pisp/imx708_wide.json',
    ):
        target = runtime / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.touch()
    return runtime


def test_camera_runtime_is_found_above_launch_file(tmp_path):
    runtime = _make_runtime(tmp_path)
    launch_file = tmp_path / 'inno_jazzy_ws/src/pkg/launch/camera.launch.py'
    launch_file.parent.mkdir(parents=True)
    launch_file.touch()

    result = camera_runtime_environment(
        launch_file, {'LD_LIBRARY_PATH': '/opt/ros/jazzy/lib'}
    )

    assert result['LD_LIBRARY_PATH'] == (
        f'{runtime}/lib:/opt/ros/jazzy/lib'
    )
    assert result['LIBCAMERA_IPA_MODULE_PATH'] == (
        f'{runtime}/lib/libcamera/ipa'
    )


def test_invalid_explicit_runtime_fails_closed(tmp_path):
    result = camera_runtime_environment(
        tmp_path / 'camera.launch.py',
        {'INNO_CAMERA_RUNTIME': str(tmp_path / 'missing')},
    )

    assert result == {}
