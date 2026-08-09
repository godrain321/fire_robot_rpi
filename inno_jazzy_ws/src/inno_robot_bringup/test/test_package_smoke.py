from pathlib import Path


def test_bringup_package_imports():
    import inno_robot_bringup.tf_to_path_node  # noqa: F401
    import inno_robot_bringup.tf_heading_marker  # noqa: F401


def test_field_launch_defaults_to_operator_adjustable_point_two_speed():
    launch = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "field_waypoint_test.launch.py"
    ).read_text(encoding="utf-8")

    assert "DeclareLaunchArgument('drive_speed', default_value='0.20')" in launch
    assert "L('drive_speed'), value_type=float" in launch


def test_rviz_uses_visible_localized_path_and_mode_colours():
    config = (
        Path(__file__).resolve().parents[1] / "rviz" / "inno_slam.rviz"
    ).read_text(encoding="utf-8")

    assert "Name: LiDAR Localized Path (Green)" in config
    assert "Line Style: Billboards\n      Line Width: 0.15" in config
    assert (
        "Name: Dynamic LiDAR Obstacles "
        "(Mode 2 Cyan / Mode 3 Red)"
    ) in config
