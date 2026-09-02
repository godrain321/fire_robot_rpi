"""Static contracts for the Mode 9 composition-only launch profile."""

import ast
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[2]
BRINGUP = SOURCE_ROOT / "inno_robot_bringup"
MODE8 = BRINGUP / "launch" / "mode8_evacuation_thermal.launch.py"
MODE9 = BRINGUP / "launch" / "mode9_full_evacuation.launch.py"
VOICE = SOURCE_ROOT / "inno_evacuation_voice" / "launch" / "periodic_evacuation_voice.launch.py"
RUN_MODE9 = SOURCE_ROOT.parents[1] / "run_mode9.sh"


def source(path):
    return path.read_text(encoding="utf-8")


def assigned_dict(path, name):
    tree = ast.parse(source(path))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not found in {path}")


def assigned_dict_expressions(path, name):
    tree = ast.parse(source(path))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name for target in node.targets
        ):
            return {
                ast.literal_eval(key): ast.unparse(value)
                for key, value in zip(node.value.keys, node.value.values)
            }
    raise AssertionError(f"{name} not found in {path}")


def test_mode9_includes_mode8_and_periodic_voice_exactly_once():
    text = source(MODE9)
    assert text.count('mode8_evacuation_thermal.launch.py') == 1
    assert text.count('periodic_evacuation_voice.launch.py') == 1
    assert text.count('IncludeLaunchDescription(') == 2


def test_mode9_forwards_mode8_arguments_with_inspection_profile_overrides():
    mode8 = {
        "use_rviz": "true",
        **assigned_dict(MODE8, "_FORWARD"),
    }
    mode9 = {
        **assigned_dict(MODE9, "_MODE8_ARGUMENTS"),
    }
    overrides = {
        "use_camera_mode4": "true",
        "yolo_only_during_mode4_observation": "true",
        "moving_survivor_enabled": "false",
        "moving_priority_enabled": "false",
        "stationary_combined_inspection_enabled": "true",
    }
    assert {name: mode9[name] for name in overrides} == overrides
    assert {
        name: value for name, value in mode9.items() if name not in overrides
    } == {
        name: value for name, value in mode8.items() if name not in overrides
    }
    assert assigned_dict_expressions(MODE9, "_MODE8_PATH_ARGUMENTS") == (
        assigned_dict_expressions(MODE8, "_FORWARD_PATHS")
    )
    text = source(MODE9)
    assert "name: L(name)" in text


def test_mode9_voice_contract_is_drive_mode_5_at_seven_seconds():
    voice = assigned_dict(MODE9, "_VOICE_ARGUMENTS")
    assert voice["voice_enabled"] == "true"
    assert voice["voice_interval_sec"] == "7.0"
    assert voice["voice_play_immediately"] == "true"
    text = source(MODE9)
    assert '"active_drive_mode": "5"' in text
    assert '"activation_mode": "drive_mode"' in text


def test_mode9_adds_no_drive_mode_or_duplicate_nodes_and_scopes_includes():
    text = source(MODE9)
    assert "/drive_mode=9" not in text
    assert '"active_drive_mode": "9"' not in text
    assert "Node(" not in text
    assert text.count("GroupAction(scoped=True") == 2
    assert "rviz2" not in text


def test_mode9_reuses_mode8_rviz_and_thermal_policy_unchanged():
    mode8 = source(MODE8)
    assert 'mode8_evacuation_thermal.rviz' in mode8
    assert '"temperature_blocked_c": "50.0"' in mode8
    assert '"event_replanning_enabled": "true"' in mode8
    assert '"exit_switching_enabled": "true"' in mode8
    assert '"waypoint_planning_enabled": "true"' in mode8


def test_mode9_enables_only_stationary_combined_camera_inspection():
    text = source(MODE9)
    assert '"stationary_combined_inspection_enabled": "true"' in text
    assert '"moving_priority_enabled": "false"' in text
    assert '"moving_survivor_enabled": "false"' in text
    assert '"yolo_only_during_mode4_observation": "true"' in text


def test_run_mode9_uses_wrapper_and_forwards_user_arguments():
    script = source(RUN_MODE9)
    assert "mode9_full_evacuation.launch.py" in script
    assert '"drive_speed:=${drive_speed}"' in script
    assert '"${launch_args[@]}"' in script
    assert "launch_status != 130" in script
    assert "launch_status != 254" in script
    assert RUN_MODE9.stat().st_mode & 0o111


def test_bringup_declares_voice_runtime_dependency():
    assert "<exec_depend>inno_evacuation_voice</exec_depend>" in source(
        BRINGUP / "package.xml"
    )


def test_existing_mode8_and_voice_sources_are_only_referenced():
    text = source(MODE9)
    assert MODE8.name in text
    assert VOICE.name in text
