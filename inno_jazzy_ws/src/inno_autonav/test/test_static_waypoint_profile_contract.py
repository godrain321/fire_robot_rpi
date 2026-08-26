"""Source contracts for the sensor-free static waypoint field profile."""

from pathlib import Path

from inno_autonav.project_paths import project_path
from inno_autonav.waypoint_selection import load_waypoint_document


ROOT = Path(__file__).parents[4]
PROFILE = ROOT / "inno_jazzy_ws/src/inno_robot_bringup/launch/static_waypoint_drive.launch.py"
AUTONAV = Path(__file__).parents[1] / "launch/autonav_demo.launch.py"
GRID = Path(__file__).parents[1] / "inno_autonav/planning_grid_publisher.py"
DOC = ROOT / "docs/static_waypoint_drive_test.md"


def test_static_profile_disables_all_unavailable_hazard_inputs():
    text = PROFILE.read_text(encoding="utf-8")
    for setting in (
        '"use_dynamic_obstacles": "false"',
        '"require_thermal_grid": "false"',
        '"require_thermal_active": "false"',
        '"hazard_belief_enabled": "false"',
        '"event_replanning_enabled": "false"',
        '"exit_switching_enabled": "false"',
    ):
        assert setting in text


def test_static_profile_is_waypoint_first_and_direct_astar_goal_is_off():
    text = PROFILE.read_text(encoding="utf-8")
    assert '"waypoint_planning_enabled": "true"' in text
    assert '"waypoint_accept_direct_goal": "true"' in text
    assert '"astar_accept_goal_pose": "false"' in text
    assert "astar_accept_goal_pose" in AUTONAV.read_text(encoding="utf-8")


def test_static_grid_is_composited_into_planning_grid_without_sensor_layers():
    grid_text = GRID.read_text(encoding="utf-8")
    profile_text = PROFILE.read_text(encoding="utf-8")
    assert "'/planning_grid_static'" in grid_text
    assert '"require_thermal_grid": "false"' in profile_text
    assert '"use_dynamic_obstacles": "false"' in profile_text


def test_profile_uses_real_159_waypoint_document():
    document = load_waypoint_document(project_path(
        "docs", "full_map_waypoints_1m_numbered.yaml"
    ))
    assert document["spacing_m"] == 1.0
    assert document["frame_id"] == "map"
    assert len(document["poses"]) == 159


def test_bringup_document_starts_motor_off_and_uses_real_goal_command():
    text = DOC.read_text(encoding="utf-8")
    assert "use_serial:=false" in text
    assert "ros2 run inno_autonav go_to exit2" in text
    assert "ros2 topic echo /planned_path" in text
    assert "ros2 topic echo /cmd_vel" in text
