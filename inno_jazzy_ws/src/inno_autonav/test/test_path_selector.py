"""Stage 8-8 PathSelector tests: pure-core relay behavior + source-scan proof of
final /planned_path ownership.
"""

from pathlib import Path as FsPath

from inno_autonav.path_selector import PathSelectorCore, PathSelectorMode


def test_waypoint_mode_relays_waypoint_path_only():
    core = PathSelectorCore(PathSelectorMode.WAYPOINT)
    out = core.on_waypoint_path("WP-1")
    assert out.publish is True and out.payload == "WP-1" and out.source == "waypoint"
    out = core.on_astar_path("ASTAR-1")
    assert out.publish is False and out.payload is None


def test_a_star_mode_relays_astar_path_only():
    core = PathSelectorCore(PathSelectorMode.A_STAR)
    out = core.on_astar_path("ASTAR-1")
    assert out.publish is True and out.payload == "ASTAR-1"
    out = core.on_waypoint_path("WP-1")
    assert out.publish is False and out.payload is None


def test_set_mode_switches_which_source_is_relayed():
    core = PathSelectorCore("WAYPOINT")
    core.on_waypoint_path("WP-1")
    core.set_mode("A_STAR")
    out = core.on_astar_path("ASTAR-1")
    assert out.publish is True and out.payload == "ASTAR-1"
    # A later, unrelated waypoint_path update must not leak through post-switch.
    out = core.on_waypoint_path("WP-2")
    assert out.publish is False


def test_switch_to_astar_releases_only_the_already_received_astar_path():
    core = PathSelectorCore("WAYPOINT")
    assert core.on_astar_path("ASTAR-CANDIDATE").publish is False
    out = core.set_mode("A_STAR")
    assert out.publish is True
    assert out.payload == "ASTAR-CANDIDATE"
    assert out.source == "astar"


def test_status_reports_mode_and_seen_sources():
    core = PathSelectorCore()
    assert core.status()["mode"] == "WAYPOINT"
    core.on_waypoint_path("WP-1")
    status = core.status()
    assert status["has_waypoint_path"] is True
    assert status["has_astar_path"] is False


# -- Stage 8-8 ownership: source-scan proof ------------------------------------

_SRC = FsPath(__file__).parents[1] / "inno_autonav"


def test_waypoint_planner_never_publishes_goal_pose_or_planned_path():
    text = (_SRC / "waypoint_planner_node.py").read_text(encoding="utf-8")
    assert "create_publisher(PoseStamped" not in text  # no /goal_pose publisher
    assert '"/planned_path"' not in text
    assert "'/planned_path'" not in text


def test_astar_replanner_path_output_is_a_parameter_not_a_hardcoded_publish_target():
    text = (_SRC / "astar_replanner.py").read_text(encoding="utf-8")
    # The only literal '/planned_path' left is the parameter *default value* --
    # the create_publisher call itself must read the parameter, not the literal.
    assert "self.create_publisher(\n            Path, str(self.get_parameter('path_output_topic')" in text
    assert "create_publisher(Path, '/planned_path'" not in text


def test_path_selector_node_is_the_default_planned_path_owner():
    text = (_SRC / "path_selector_node.py").read_text(encoding="utf-8")
    assert '"planned_path_topic": "/planned_path"' in text
    assert "create_publisher(\n            Path, str(value(\"planned_path_topic\"))" in text


def test_stage8_ownership_contract_remains_separated():
    evacuation = (_SRC / "evacuation_manager_node.py").read_text(encoding="utf-8")
    waypoint = (_SRC / "waypoint_planner_node.py").read_text(encoding="utf-8")
    astar = (_SRC / "astar_replanner.py").read_text(encoding="utf-8")
    supervisor = (_SRC / "replan_supervisor_node.py").read_text(encoding="utf-8")
    assert '"planner_goal_topic": "/goal_pose"' in evacuation
    assert "waypoint_path_publisher = self.create_publisher" in waypoint
    assert "Path, str(self.get_parameter('path_output_topic').value)" in astar
    assert '"hold_topic": "/replanning/hold"' in supervisor
    assert "create_publisher(Path" not in supervisor
    launch = (_SRC.parent / "launch/autonav_demo.launch.py").read_text(encoding="utf-8")
    assert "condition=UnlessCondition(evacuation_manager_enabled)" in launch
    assert "if not self.waypoint_planning_enabled:" in supervisor


def test_selector_contains_no_planner_or_path_mutation_algorithms():
    text = (_SRC / "path_selector.py").read_text(encoding="utf-8")
    for forbidden in ("weighted_astar", "WaypointGraphPlanner", "simplify_path", "path_cost"):
        assert forbidden not in text


def test_mode5_astar_candidate_is_the_direct_cell_fallback():
    astar = (_SRC / "astar_replanner.py").read_text(encoding="utf-8")
    config = (_SRC.parent / "config/autonav_params.yaml").read_text(
        encoding="utf-8"
    )
    assert "'direct_planning_modes': [3, 4, 5]" in astar
