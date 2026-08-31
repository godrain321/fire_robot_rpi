"""Static launch contract for Mode 7: existing exit-decision + thermal, no gas,
no person detection. Mirrors test_stage9_launch_integration.py's source-text style.
"""

import ast
from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parents[2]
BRINGUP = SOURCE_ROOT / "inno_robot_bringup" / "launch"
MODE7 = BRINGUP / "mode7_thermal_drive.launch.py"
MODE6 = BRINGUP / "mode6_thermal_preview.launch.py"
EVAC = BRINGUP / "evacuation_demo.launch.py"
AUTONAV = SOURCE_ROOT / "inno_autonav" / "launch" / "autonav_demo.launch.py"
COORDINATOR = SOURCE_ROOT / "inno_autonav" / "inno_autonav" / "mode7_mission_coordinator.py"
MANAGER = SOURCE_ROOT / "inno_autonav" / "inno_autonav" / "evacuation_manager_node.py"
RUN_MODE7 = SOURCE_ROOT.parents[1] / "run_mode7.sh"

_MODE7_SRC = MODE7.read_text(encoding="utf-8")


def _field_kwargs():
    """The launch_arguments dict Mode 7 forwards into field_waypoint_test.

    Constant string values are kept verbatim; ``L("x")`` pass-throughs are
    recorded as the marker ``"<launchconfig>"`` so the key presence can still
    be asserted.
    """
    tree = ast.parse(_MODE7_SRC)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        out = {}
        for k, v in zip(node.keys, node.values):
            if not (isinstance(k, ast.Constant) and isinstance(k.value, str)):
                continue
            if isinstance(v, ast.Constant):
                out[k.value] = v.value
            elif isinstance(v, ast.Call):
                out[k.value] = "<launchconfig>"
        if "hazard_belief_enabled" in out:
            return out
    raise AssertionError("field_waypoint_test launch_arguments not found")


FIELD = _field_kwargs()


# -- mission / exit decision ON (existing Mode 5 nodes) -------------------
def test_exit_evaluator_and_manager_enabled():
    assert FIELD["exit_evaluator_enabled"] == "true"
    assert FIELD["evacuation_manager_enabled"] == "true"
    assert FIELD["evacuation_activate_selected_route"] == "true"


def test_exit_switching_forwarded_for_replan_exhausted():
    assert FIELD["exit_switching_enabled"] == "<launchconfig>"
    assert 'DeclareLaunchArgument("exit_switching_enabled", default_value="true")' \
        in _MODE7_SRC


def test_thin_coordinator_calls_existing_plan_service():
    assert "/plan_evacuation" in _MODE7_SRC
    assert 'executable="mode7_mission_coordinator"' in _MODE7_SRC
    assert 'DeclareLaunchArgument("mode7_auto_start", default_value="false")' in _MODE7_SRC
    assert "create_publisher" not in _MODE7_SRC          # no home-grown goal pub
    assert "rclpy" not in _MODE7_SRC                     # no new node in this file


def test_coordinator_readiness_plan_and_cancel_contract():
    src = COORDINATOR.read_text(encoding="utf-8")
    for interface in (
        "/localization_ready", "/hazard/status", "/exit_evaluator/status",
        "/evacuation/status", "/plan_evacuation", "/mode7/start",
        "/mode7/stop", "/autonomy_cancel", "/follower_state",
    ):
        assert interface in src
    assert "lookup_pose_2d" in src
    assert 'Int32(data=5)' in src
    assert 'message.data == "GOAL_REACHED"' in src
    assert "future.cancel()" in src


def test_existing_manager_owns_evaluation_selection_plan_and_goal_activation():
    src = MANAGER.read_text(encoding="utf-8")
    assert '"exit_evaluation_service": "/evaluate_exits"' in src
    assert '"plan_service": "/plan_evacuation"' in src
    assert '"plan_topic": "/evacuation/plan"' in src
    assert '"selected_exit_topic": "/evacuation/selected_exit"' in src
    assert '"planner_goal_topic": "/goal_pose"' in src
    assert "build_evacuation_decision" in src


# -- person / victim detection OFF --------------------------------------
def test_person_detection_all_off():
    assert FIELD["use_mmwave"] == "false"
    assert FIELD["use_camera_mode4"] == "false"
    assert FIELD["use_mode3_audio"] == "false"
    assert FIELD["mode5_enabled"] == "false"             # orchestrator not started
    assert FIELD["person_inspection_enabled"] == "false"


def test_mode5_orchestrator_tree_not_included():
    assert "evacuation_demo.launch.py" not in _MODE7_SRC
    assert "field_waypoint_test.launch.py" in _MODE7_SRC


def test_shared_launch_rviz_disable_is_scoped_from_mode7_rviz_argument():
    assert "GroupAction" in _MODE7_SRC
    assert "field_bringup = GroupAction(" in _MODE7_SRC
    assert "scoped=True" in _MODE7_SRC
    assert 'condition=IfCondition(L("use_rviz"))' in _MODE7_SRC


def test_autonav_mission_commander_mutually_exclusive_with_evac_manager():
    autonav = AUTONAV.read_text(encoding="utf-8")
    assert "UnlessCondition(evacuation_manager_enabled)" in autonav
    assert "mission_commander" in autonav


# -- gas / CO OFF ---------------------------------------------------
def test_gas_layer_off():
    assert FIELD["hazard_co_enabled"] == "false"
    assert "use_gas_sensor" not in _MODE7_SRC            # field launch has no such arg
    assert "mq135" not in _MODE7_SRC.lower()


def test_dynamic_obstacles_remain_on_without_person_inspectors():
    assert FIELD["use_dynamic_obstacles"] == "<launchconfig>"
    autonav = AUTONAV.read_text(encoding="utf-8")
    assert "person_inspection_enabled" in autonav
    assert "executable='dynamic_obstacle_layer'" in autonav
    assert autonav.count("person_inspection_enabled, \"' == 'true'\"") == 2


# -- thermal ON ---------------------------------------------------
def test_thermal_hazard_on():
    assert FIELD["hazard_belief_enabled"] == "true"
    assert FIELD["hazard_thermal_enabled"] == "true"
    assert FIELD["use_thermal_sensor"] == "true"
    assert "enable_cost_layer" in _MODE7_SRC
    assert "thermal_sensor.launch.py" in _MODE7_SRC
    assert "thermal_camera_link" in _MODE7_SRC           # static TF kept


def test_planner_pipeline_forwarded_unchanged():
    assert FIELD["waypoint_planning_enabled"] == "<launchconfig>"
    assert FIELD["event_replanning_enabled"] == "<launchconfig>"
    assert FIELD["waypoint_accept_direct_goal"] == "false"
    assert FIELD["use_dynamic_obstacles"] == "<launchconfig>"
    assert FIELD["astar_accept_goal_pose"] == "true"


def test_required_navigation_nodes_are_present_in_shared_launch():
    autonav = AUTONAV.read_text(encoding="utf-8")
    for executable in (
        "hazard_belief_node", "exit_evaluator_node",
        "evacuation_manager_node", "replan_supervisor_node",
        "exit_switching_node", "waypoint_planner_node",
        "path_selector_node", "astar_replanner", "skid_path_follower",
    ):
        assert f"executable='{executable}'" in autonav


def test_run_script_parses_safety_arguments_without_duplicate_forwarding():
    src = RUN_MODE7.read_text(encoding="utf-8")
    assert 'use_serial:=*) use_serial=' in src
    assert 'mode7_auto_start:=*) mode7_auto_start=' in src
    assert src.count('"use_serial:=${use_serial}"') == 1
    assert src.count('"mode7_auto_start:=${mode7_auto_start}"') == 1


# -- Mode 6 / Mode 5 untouched ------------------------------------
def test_mode6_still_bench_only():
    src = MODE6.read_text(encoding="utf-8")
    assert "hazard_belief" not in src
    assert "exit_evaluator" not in src
    assert "plan_evacuation" not in src


def test_mode5_launch_still_defaults_thermal_off():
    src = EVAC.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("use_thermal_sensor", default_value="false")' in src
