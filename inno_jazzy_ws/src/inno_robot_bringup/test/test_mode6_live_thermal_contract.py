"""Static contract checks for the stationary Mode 6 live thermal preview."""

import ast
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[2]
BRINGUP = SOURCE_ROOT / "inno_robot_bringup" / "launch"
MODE6 = BRINGUP / "mode6_thermal_preview.launch.py"
MODE7 = BRINGUP / "mode7_thermal_drive.launch.py"
MODE8 = BRINGUP / "mode8_evacuation_thermal.launch.py"
THERMAL_LAUNCH = SOURCE_ROOT / "inno_thermal" / "launch" / "thermal_sensor.launch.py"
RUN_MODE6 = SOURCE_ROOT.parents[1] / "run_mode6.sh"


def _include_arguments(path: Path, required_key: str) -> dict[str, str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        values = {}
        for key, value in zip(node.keys, node.values):
            if not (
                isinstance(key, ast.Constant)
                and isinstance(key.value, str)
                and isinstance(value, ast.Constant)
            ):
                continue
            values[key.value] = value.value
        if required_key in values:
            return values
    raise AssertionError(f"launch arguments containing {required_key!r} not found")


def test_mode6_enables_latest_frame_replacement_and_short_timeout():
    arguments = _include_arguments(MODE6, "replace_observations_each_frame")
    assert arguments["persistent_observations"] == "false"
    assert arguments["replace_observations_each_frame"] == "true"
    assert arguments["observation_timeout_sec"] == "0.75"
    assert arguments["thermal_data_timeout_sec"] == "0.75"
    assert arguments["use_latest_tf_fallback"] == "true"


def test_shared_thermal_defaults_preserve_navigation_modes():
    source = THERMAL_LAUNCH.read_text(encoding="utf-8")
    assert '"persistent_observations", default_value="true"' in source
    assert '"replace_observations_each_frame", default_value="false"' in source
    assert '"use_latest_tf_fallback", default_value="false"' in source
    assert 'LaunchConfiguration("replace_observations_each_frame")' in source
    assert 'LaunchConfiguration("use_latest_tf_fallback")' in source
    assert "replace_observations_each_frame" not in MODE7.read_text(encoding="utf-8")
    assert "replace_observations_each_frame" not in MODE8.read_text(encoding="utf-8")


def test_mode6_defaults_to_stationary_manual_localization():
    source = MODE6.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("use_serial", default_value="false")' in source
    assert 'DeclareLaunchArgument("set_initial_pose", default_value="false")' in source


def test_run_mode6_defaults_serial_off_and_allows_one_override():
    source = RUN_MODE6.read_text(encoding="utf-8")
    assert "use_serial='false'" in source
    assert 'use_serial:=*) use_serial=' in source
    assert source.count('"use_serial:=${use_serial}"') == 1
