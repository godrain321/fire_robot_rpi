"""Static contracts for the existing Stage 9 integrated launch hierarchy."""

from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[2]
BRINGUP = SOURCE_ROOT / "inno_robot_bringup" / "launch"
AUTONAV = SOURCE_ROOT / "inno_autonav" / "launch" / "autonav_demo.launch.py"
THERMAL = SOURCE_ROOT / "inno_thermal" / "launch" / "thermal_sensor.launch.py"
THERMAL_CONFIG = SOURCE_ROOT / "inno_thermal" / "config" / "thermal_params.yaml"
HAZARD_CONFIG = SOURCE_ROOT / "inno_hazard" / "config" / "hazard_params.yaml"
HAZARD_SNAPSHOT = (
    SOURCE_ROOT / "inno_hazard" / "inno_hazard" / "hazard_snapshot.py"
)
RUN_MODE8 = SOURCE_ROOT.parents[1] / "run_mode8.sh"


def source(path):
    return path.read_text(encoding="utf-8")


def test_integrated_launch_reuses_thermal_and_field_launches_with_static_tf():
    text = source(BRINGUP / "evacuation_demo.launch.py")
    assert 'thermal + "/launch/thermal_sensor.launch.py"' in text
    assert '"enable_cost_layer": "true"' in text
    assert 'bringup + "/launch/field_waypoint_test.launch.py"' in text
    assert 'executable="static_transform_publisher"' in text
    assert '"--frame-id", "base_link"' in text
    assert '"--child-frame-id", "thermal_camera_link"' in text
    assert 'condition=IfCondition(L("use_thermal_sensor"))' in text


def test_field_launch_connects_localization_autonav_waypoint_and_optional_serial():
    text = source(BRINGUP / "field_waypoint_test.launch.py")
    assert "lidar_amcl_localization.launch.py" in text
    assert "autonav_demo.launch.py" in text
    assert "executable='waypoint_queue'" in text
    assert "executable='cmd_vel_mode_mux'" in text
    assert "executable='cmdvel_to_esp32_serial'" in text
    assert "condition=IfCondition(L('use_serial'))" in text
    # The nested serial bridge stays disabled; the field-level bridge is the
    # sole owner controlled by the top-level Motor-OFF/Motor-ON argument.
    assert "'use_serial': 'false'" in text


def test_integrated_arguments_reach_field_and_autonav_layers():
    outer = source(BRINGUP / "evacuation_demo.launch.py")
    field = source(BRINGUP / "field_waypoint_test.launch.py")
    compact_field = "".join(field.split())
    for argument in (
        "use_serial", "use_thermal_sensor", "event_replanning_enabled",
        "waypoint_planning_enabled",
    ):
        assert f'DeclareLaunchArgument("{argument}"' in outer
        assert f'"{argument}": L("{argument}")' in outer
    for argument in (
        "event_replanning_enabled", "waypoint_planning_enabled",
        "require_thermal_grid", "require_thermal_active",
    ):
        assert f"'{argument}':L('{argument}')" in compact_field


def test_thermal_launch_starts_sensor_and_cost_layer():
    text = source(THERMAL)
    assert 'executable="mlx90640_sensor_node"' in text
    assert 'executable="thermal_cost_layer"' in text
    assert 'DeclareLaunchArgument(\n                "enable_cost_layer"' in text
    assert "condition=IfCondition(enable_cost_layer)" in text


def test_mode8_separates_legacy_soft_scale_from_50c_hard_block():
    mode8 = source(BRINGUP / "mode8_evacuation_thermal.launch.py")
    evacuation = source(BRINGUP / "evacuation_demo.launch.py")
    field = source(BRINGUP / "field_waypoint_test.launch.py")
    autonav = source(AUTONAV)
    thermal = source(THERMAL)

    assert '"temperature_cost_scale_max_c": "60.0"' in mode8
    assert '"temperature_blocked_c": "50.0"' in mode8
    assert '"temperature_cost_scale_max_c", default_value="60.0"' in evacuation
    assert '"temperature_blocked_c", default_value="60.0"' in evacuation
    assert '"blocked_temperature_c": L("temperature_blocked_c")' in evacuation
    assert "'temperature_cost_scale_max_c': L(" in field
    assert "'temperature_blocked_c': L('temperature_blocked_c')" in field
    assert "'temperature_cost_scale_max_c': ParameterValue(" in autonav
    assert "'temperature_blocked_c': ParameterValue(" in autonav
    assert '"blocked_temperature_c": ParameterValue(' in thermal
    assert 'temperature_cost_scale_max_c: 60.0' in source(THERMAL_CONFIG)
    assert 'blocked_temperature_c: 60.0' in source(THERMAL_CONFIG)
    assert 'temperature_cost_scale_max_c: 60.0' in source(HAZARD_CONFIG)
    assert 'temperature_blocked_c: 60.0' in source(HAZARD_CONFIG)
    assert (
        '"temperature_blocked_c": float(belief.config.temperature_blocked_c)'
        in source(HAZARD_SNAPSHOT)
    )


def test_mode8_nested_rviz_disable_is_scoped_from_thermal_rviz():
    mode8 = source(BRINGUP / "mode8_evacuation_thermal.launch.py")
    assert "GroupAction" in mode8
    assert "mode5 = GroupAction(" in mode8
    assert "scoped=True" in mode8
    assert 'condition=IfCondition(L("use_rviz"))' in mode8


def test_mode8_disables_display_only_thermal_image_pipeline():
    mode8 = source(BRINGUP / "mode8_evacuation_thermal.launch.py")
    evacuation = source(BRINGUP / "evacuation_demo.launch.py")
    thermal = source(THERMAL)
    rviz = source(BRINGUP.parent / "rviz" / "mode8_evacuation_thermal.rviz")

    assert '"publish_thermal_image": "false"' in mode8
    assert '"publish_image": L("publish_thermal_image")' in evacuation
    assert 'DeclareLaunchArgument(\n                "publish_image"' in thermal
    assert 'LaunchConfiguration("publish_image"), value_type=bool' in thermal
    assert "Thermal Image" not in rviz
    assert "/thermal/image" not in rviz
    assert "Thermal Cost Grid" in rviz
    assert "/thermal_cost_grid" in rviz


def test_mode8_script_accepts_ros_launch_sigint_exit_code():
    script = source(RUN_MODE8)
    assert "launch_status != 254" in script


def test_mode8_script_raises_only_default_straight_speed():
    script = source(RUN_MODE8)
    assert "drive_speed='0.15'" in script
    assert '"drive_speed:=${drive_speed}"' in script
    assert "turn_speed:=0.64" in script


def test_autonav_has_exactly_one_planned_path_owner_per_selector_profile():
    text = source(AUTONAV)
    assert "'/astar_path' if '" in text
    assert "else '/planned_path'" in text
    assert "'path_output_topic': astar_path_output_topic" in text
    assert "executable='path_selector_node'" in text
    assert "condition=IfCondition(waypoint_planning_enabled)" in text
    # The waypoint planner only publishes /waypoint_path; PathSelector owns the
    # canonical path whenever that optional pipeline is enabled.
    planner = source(
        SOURCE_ROOT / "inno_autonav" / "inno_autonav" / "waypoint_planner_node.py"
    )
    assert '"waypoint_path_topic": "/waypoint_path"' in planner
    assert "create_publisher(\n            Path, str(value(\"waypoint_path_topic\"))" in planner
    assert "create_publisher(\n            Path, str(value(\"planned_path_topic\"))" not in planner


def test_hazard_mode_waypoint_planner_uses_astar_active_reachability_grid():
    text = source(AUTONAV)
    declaration = text.split(
        "'waypoint_planning_grid_topic'", 1
    )[1].split("DeclareLaunchArgument('astar_accept_goal_pose'", 1)[0]
    assert "'/planning_grid_active'" in declaration
    assert "hazard_belief_enabled" in declaration
    assert "'/planning_grid_hazard'" in declaration
    assert "hazard_co_enabled" in declaration


def test_autonav_runs_existing_follower_and_event_replan_nodes():
    text = source(AUTONAV)
    assert "executable='skid_path_follower'" in text
    assert "executable='replan_supervisor_node'" in text
    assert "condition=IfCondition(event_replanning_enabled)" in text
    assert "'periodic_replanning_enabled': ParameterValue(" in text


def test_no_new_stage9_or_full_system_launch_was_added():
    names = {path.name for path in BRINGUP.glob("*.launch.py")}
    assert "stage9.launch.py" not in names
    assert "full_system.launch.py" not in names
    assert "thermal_autonav.launch.py" not in names
