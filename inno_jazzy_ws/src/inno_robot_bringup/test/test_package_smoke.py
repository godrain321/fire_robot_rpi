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
    assert 'de2033aed827f0119bb79ad8346f00fe-if00-port0' in launch
    assert '4a5b9018526eef11bff6e0c2c169b110-if00-port0' in launch
    assert "DeclareLaunchArgument('start_thermal_viewer', default_value='true')" in launch
    assert "thermal_dir + '/mlx90640.py'" in launch
    assert "condition=IfCondition(L('start_thermal_viewer'))" in launch
    assert "'trigger_topic': '/victim_detected'" in launch
    assert "'gpio_lines': [17, 27, 22, 23, 24]" in launch
    assert "'reset_on_false': True" in launch


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
