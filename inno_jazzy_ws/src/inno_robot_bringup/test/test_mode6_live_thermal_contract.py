"""Static contract checks for the stationary Mode 6 live thermal preview."""

import ast
from pathlib import Path

import yaml


SOURCE_ROOT = Path(__file__).resolve().parents[2]
BRINGUP = SOURCE_ROOT / "inno_robot_bringup" / "launch"
MODE6 = BRINGUP / "mode6_thermal_preview.launch.py"
MODE7 = BRINGUP / "mode7_thermal_drive.launch.py"
MODE8 = BRINGUP / "mode8_evacuation_thermal.launch.py"
THERMAL_LAUNCH = SOURCE_ROOT / "inno_thermal" / "launch" / "thermal_sensor.launch.py"
THERMAL_SENSOR = (
    SOURCE_ROOT / "inno_thermal" / "inno_thermal" / "mlx90640_sensor_node.py"
)
RUN_MODE6 = SOURCE_ROOT.parents[1] / "run_mode6.sh"
MODE6_RVIZ = SOURCE_ROOT / "inno_robot_bringup" / "rviz" / "mode6_thermal.rviz"


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
    assert arguments["arc_use_latest_tf"] == "true"


def test_shared_thermal_defaults_preserve_navigation_modes():
    source = THERMAL_LAUNCH.read_text(encoding="utf-8")
    assert '"persistent_observations", default_value="true"' in source
    assert '"replace_observations_each_frame", default_value="false"' in source
    assert '"use_latest_tf_fallback", default_value="false"' in source
    assert '"arc_use_latest_tf", default_value="false"' in source
    sensor_source = THERMAL_SENSOR.read_text(encoding="utf-8")
    assert 'self.declare_parameter("arc_use_latest_tf", False)' in sensor_source
    assert "arc_stamp = type(stamp)()" in sensor_source
    assert 'LaunchConfiguration("replace_observations_each_frame")' in source
    assert 'LaunchConfiguration("use_latest_tf_fallback")' in source
    assert "replace_observations_each_frame" not in MODE7.read_text(encoding="utf-8")
    assert "replace_observations_each_frame" not in MODE8.read_text(encoding="utf-8")
    assert "arc_use_latest_tf" not in MODE7.read_text(encoding="utf-8")
    assert "arc_use_latest_tf" not in MODE8.read_text(encoding="utf-8")


def test_mode6_defaults_to_stationary_automatic_localization():
    source = MODE6.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("use_serial", default_value="false")' in source
    assert 'DeclareLaunchArgument("set_initial_pose", default_value="true")' in source
    assert 'DeclareLaunchArgument("initial_pose_x", default_value="4.817799091339111")' in source
    assert 'DeclareLaunchArgument("initial_pose_y", default_value="-9.854209899902344")' in source
    assert '"initial_pose_yaw", default_value="-0.4439678115329046"' in source


def test_mode6_rviz_hides_scan_but_preserves_thermal_displays():
    config = yaml.safe_load(MODE6_RVIZ.read_text(encoding="utf-8"))
    displays = {
        display["Name"]: display
        for display in config["Visualization Manager"]["Displays"]
    }

    assert displays["SLAM Map"]["Enabled"] is True
    assert displays["RPLIDAR Scan"]["Enabled"] is False
    assert displays["RPLIDAR Scan"]["Topic"]["Value"] == "/scan"
    assert displays["Thermal Arc Points"]["Enabled"] is True
    assert displays["Thermal Arc Points"]["Topic"]["Value"] == "/thermal/arc_points"
    assert displays["Thermal Image"]["Enabled"] is True
    assert displays["Thermal Image"]["Topic"]["Value"] == "/thermal/image"

    cost_grid = displays["Thermal Cost Grid"]
    assert cost_grid["Enabled"] is True
    assert cost_grid["Topic"]["Value"] == "/thermal_cost_grid"
    assert cost_grid["Color Scheme"] == "costmap"
    assert cost_grid["Alpha"] == 0.65
    assert cost_grid["Draw Behind"] is False


def test_mode6_rviz_starts_centered_near_the_initial_pose():
    config = yaml.safe_load(MODE6_RVIZ.read_text(encoding="utf-8"))
    view = config["Visualization Manager"]["Views"]["Current"]
    window = config["Window Geometry"]

    assert view["Class"] == "rviz_default_plugins/TopDownOrtho"
    assert view["X"] == 4.817799091339111
    assert view["Y"] == -9.854209899902344
    assert view["Scale"] == 130
    # TopDownOrtho Scale is pixels/metre. The 900 px window therefore gives
    # a roughly 6 m-tall usable canvas after RViz chrome is accounted for.
    assert 5.0 <= window["Height"] / view["Scale"] <= 8.0


def test_run_mode6_defaults_serial_off_and_allows_one_override():
    source = RUN_MODE6.read_text(encoding="utf-8")
    assert "use_serial='false'" in source
    assert 'use_serial:=*) use_serial=' in source
    assert source.count('"use_serial:=${use_serial}"') == 1
