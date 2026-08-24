"""Spec section 42 (A-J) + Stage 7 boundary tests for ReplanSupervisorCore."""

from pathlib import Path as FsPath
import json

import numpy as np

from inno_autonav.event_replanning import EventReplanningConfig
from inno_autonav.exit_evaluator import ExitHazardSnapshot, ExitStatus
from inno_autonav.replan_supervisor import (
    ActiveGoal,
    ReplanSupervisorCore,
    RetryConfig,
    parse_active_goal_payload,
)
from inno_hazard.hazard_belief import HazardGridGeometry


GOAL = ActiveGoal("EXIT1", (4.5, 0.5), 1)
SAFE_PATH_WORLD = [(0.5, 0.5), (1.5, 0.5), (2.5, 0.5), (3.5, 0.5), (4.5, 0.5)]


def snapshot(size=8, resolution=1.0, revision=1, **overrides):
    shape = size, size
    geometry = HazardGridGeometry(size, size, resolution)
    values = {
        "final_cost": np.ones(shape),
        "temperature_c": np.full(shape, np.nan),
        "co_ppm": np.full(shape, np.nan),
        "observed_mask": np.zeros(shape, dtype=bool),
        "temperature_observed_mask": np.zeros(shape, dtype=bool),
        "co_observed_mask": np.zeros(shape, dtype=bool),
        "fire_probability": np.zeros(shape),
        "static_obstacle_map": np.zeros(shape, dtype=bool),
        "dynamic_obstacle_map": np.zeros(shape, dtype=bool),
        "blocked_mask": np.zeros(shape, dtype=bool),
    }
    values.update(overrides)
    return ExitHazardSnapshot(
        geometry, values["final_cost"], values["temperature_c"], values["co_ppm"],
        values["observed_mask"], values["temperature_observed_mask"],
        values["co_observed_mask"], values["fire_probability"],
        values["static_obstacle_map"], values["dynamic_obstacle_map"],
        values["blocked_mask"], revision, 60.0, 1600.0, 1.0,
    )


def bootstrap(core, path_world=None, pose=(0.5, 0.5)):
    """Establish goal/path/snapshot/pose and consume the one-time baseline tick.

    ReplanSupervisorCore mirrors run_partial_costmap_evacuation.py's setup, which
    calls EventReplanningPolicy.mark_reevaluation_complete() once, before the first
    real per-tick evaluate() call (see the comment in replan_supervisor.py). The
    very first tick where goal+path+snapshot+pose are all known consumes that
    baseline and always reports PATH_VALID -- callers apply the scenario they
    actually want to test as a *separate*, subsequent on_hazard_snapshot() call.
    """
    core.on_active_goal(GOAL)
    core.on_planned_path(path_world or SAFE_PATH_WORLD)
    core.on_hazard_snapshot(snapshot(revision=0))
    baseline = core.tick(pose, 0.0)
    assert baseline.status["state"] == "PATH_VALID"
    return baseline


# -- A/B: revision changes or soft cost changes alone must not replan ----------

def test_a_revision_changed_but_path_safe_no_replan():
    core = ReplanSupervisorCore(EventReplanningConfig())
    bootstrap(core)
    out = core.on_hazard_snapshot(snapshot(revision=1))
    assert out.hold is False and out.publish_goal is None
    out2 = core.on_hazard_snapshot(snapshot(revision=2))
    assert out2.hold is False
    assert out2.publish_goal is None
    assert out2.status["state"] == "PATH_VALID"
    assert out2.status["last_validated_revision"] == 2


def test_b_soft_cost_increase_alone_does_not_replan():
    core = ReplanSupervisorCore(EventReplanningConfig())
    bootstrap(core)
    cost = np.ones((8, 8))
    cost[0, 2] = 8.0  # finite, just more expensive
    out = core.on_hazard_snapshot(snapshot(revision=1, final_cost=cost))
    assert out.hold is False
    assert out.publish_goal is None


# -- C: dynamic obstacle appears -> hold together with the same-goal request ---

def test_c_dynamic_obstacle_holds_and_requests_same_goal():
    core = ReplanSupervisorCore(EventReplanningConfig())
    bootstrap(core)
    dynamic = np.zeros((8, 8), dtype=bool)
    dynamic[0, 2] = True
    out = core.on_hazard_snapshot(snapshot(revision=1, dynamic_obstacle_map=dynamic))
    assert out.hold is True
    assert out.publish_goal == GOAL.approach_world
    assert out.status["state"] == "REPLAN_REQUESTED"
    assert out.status["attempt_count"] == 1
    assert out.status["active_exit_id"] == "EXIT1"


# -- D: obstacle behind the robot must not trigger a replan ---------------------

def test_d_obstacle_behind_robot_is_ignored():
    # Placed behind the robot's current position (cell (0,0)); dynamic_obstacle_map
    # is a condition policy.evaluate() actively checks, so this only stays "no
    # replan" if remaining_path_from_pose actually trims the path first.
    core = ReplanSupervisorCore(EventReplanningConfig())
    bootstrap(core, pose=(2.5, 0.5))  # robot already at grid cell (2,0)
    dynamic = np.zeros((8, 8), dtype=bool)
    dynamic[0, 0] = True
    out = core.on_hazard_snapshot(snapshot(revision=1, dynamic_obstacle_map=dynamic))
    assert out.hold is False
    assert out.publish_goal is None


# -- E: a sparse /planned_path must not hide a mid-segment obstacle -------------

def test_e_sparse_path_mid_cell_obstacle_is_caught():
    core = ReplanSupervisorCore(EventReplanningConfig())
    bootstrap(core)
    core.on_planned_path([(0.5, 0.5), (4.5, 0.5)])  # simplified/sparse version
    dynamic = np.zeros((8, 8), dtype=bool)
    dynamic[0, 2] = True  # not a waypoint in the sparse path above
    out = core.on_hazard_snapshot(snapshot(revision=1, dynamic_obstacle_map=dynamic))
    assert out.hold is True
    assert out.publish_goal == GOAL.approach_world


# -- F/G: revalidate the *new* path before releasing hold -----------------------

def test_f_new_plan_still_unsafe_keeps_hold():
    core = ReplanSupervisorCore(EventReplanningConfig())
    bootstrap(core)
    dynamic = np.zeros((8, 8), dtype=bool)
    dynamic[0, 2] = True
    out = core.on_hazard_snapshot(snapshot(revision=1, dynamic_obstacle_map=dynamic))
    assert out.hold is True and out.publish_goal is not None  # replan requested

    still_blocked = np.zeros((8, 8), dtype=bool)
    still_blocked[0, 3] = True
    core.on_hazard_snapshot(snapshot(revision=2, static_obstacle_map=still_blocked))
    core.on_planner_state("PLANNING")
    core.on_planned_path(SAFE_PATH_WORLD)  # astar_replanner's candidate result
    out = core.on_planner_state("PATH_READY")
    assert out.hold is True
    assert out.status["state"] == "REPLAN_FAILED"
    assert out.status["last_failure_reason"].startswith("NEW_PATH_UNSAFE")


def test_g_new_plan_safe_releases_hold():
    core = ReplanSupervisorCore(EventReplanningConfig())
    bootstrap(core)
    dynamic = np.zeros((8, 8), dtype=bool)
    dynamic[0, 2] = True
    out = core.on_hazard_snapshot(snapshot(revision=1, dynamic_obstacle_map=dynamic))
    assert out.hold is True

    core.on_hazard_snapshot(snapshot(revision=2))  # obstacle cleared
    core.on_planner_state("REPLANNING")
    core.on_planned_path(SAFE_PATH_WORLD)
    out = core.on_planner_state("PATH_READY")
    assert out.hold is False
    assert out.status["attempt_count"] == 0


# -- H: planner failure keeps the hold, does not resume on the old path --------

def test_h_planner_failure_keeps_hold():
    core = ReplanSupervisorCore(EventReplanningConfig(), RetryConfig(cooldown_seconds=10.0))
    bootstrap(core)
    dynamic = np.zeros((8, 8), dtype=bool)
    dynamic[0, 2] = True
    core.on_hazard_snapshot(snapshot(revision=1, dynamic_obstacle_map=dynamic))
    out = core.on_planner_state("NO_PATH")
    assert out.hold is True
    assert out.status["state"] == "REPLAN_FAILED"
    assert out.publish_goal is None  # still cooling down, no reckless immediate retry


def test_waypoint_to_astar_progress_refreshes_timeout_without_spending_retry():
    core = ReplanSupervisorCore(EventReplanningConfig(), RetryConfig(replan_timeout_s=1.0))
    bootstrap(core)
    dynamic = np.zeros((8, 8), dtype=bool)
    dynamic[0, 2] = True
    core.on_hazard_snapshot(snapshot(revision=1, dynamic_obstacle_map=dynamic))
    assert core.attempt_count == 1
    core.tick((0.5, 0.5), 0.8)
    out = core.on_replan_progress()
    assert out.hold is True
    assert out.status["state"] == "WAITING_FOR_NEW_PATH"
    assert core.attempt_count == 1
    assert core.request_started_at == 0.8


# -- I: retry cooldown/backoff up to max_replan_attempts, then exhausted -------

def test_i_retry_then_exhausted_never_switches_exit():
    retry = RetryConfig(max_replan_attempts=2, cooldown_seconds=0.1)
    core = ReplanSupervisorCore(EventReplanningConfig(), retry)
    bootstrap(core)
    dynamic = np.zeros((8, 8), dtype=bool)
    dynamic[0, 2] = True
    out = core.on_hazard_snapshot(snapshot(revision=1, dynamic_obstacle_map=dynamic))
    assert out.status["attempt_count"] == 1

    out = core.on_planner_state("NO_PATH")
    assert out.publish_goal is None  # cooldown gate

    out = core.tick((0.5, 0.5), 0.05)
    assert out.publish_goal is None  # still cooling down

    out = core.tick((0.5, 0.5), 0.2)
    assert out.publish_goal == GOAL.approach_world
    assert out.status["attempt_count"] == 2

    out = core.on_planner_state("NO_PATH")
    assert out.publish_goal is None

    out = core.tick((0.5, 0.5), 0.35)
    assert out.status["state"] == "REPLAN_EXHAUSTED"
    assert out.hold is True
    assert out.publish_goal is None
    assert out.status["active_exit_id"] == "EXIT1"

    # Terminal: further ticks change nothing without an external goal update.
    out = core.tick((0.5, 0.5), 100.0)
    assert out.status["state"] == "REPLAN_EXHAUSTED"
    assert out.hold is True


def test_i_goal_change_resets_out_of_exhaustion():
    retry = RetryConfig(max_replan_attempts=1, cooldown_seconds=0.0)
    core = ReplanSupervisorCore(EventReplanningConfig(), retry)
    bootstrap(core)
    dynamic = np.zeros((8, 8), dtype=bool)
    dynamic[0, 2] = True
    core.on_hazard_snapshot(snapshot(revision=1, dynamic_obstacle_map=dynamic))
    core.on_planner_state("NO_PATH")
    out = core.tick((0.5, 0.5), 1.0)
    assert out.status["state"] == "REPLAN_EXHAUSTED"

    new_goal = ActiveGoal("EXIT2", (9.5, 9.5), 5)
    out = core.on_active_goal(new_goal)
    assert out.status["state"] != "REPLAN_EXHAUSTED"
    assert out.hold is False
    assert out.status["active_exit_id"] == "EXIT2"


# -- current-exit-invalid: hold + EXIT_RESELECTION_REQUIRED, no auto-switch ----

def test_current_exit_blocked_holds_without_selecting_another_exit():
    core = ReplanSupervisorCore(
        EventReplanningConfig(), exit_status_lookup=lambda: {"EXIT1": ExitStatus.BLOCKED},
    )
    bootstrap(core)
    # EXIT_INVALID is not an "emergency" priority (only >= HAZARD_BLOCKED is), so
    # it is still subject to the minimum_replan_interval_s gate relative to the
    # baseline timestamp -- advance past it first, exactly like the simulation
    # needs real elapsed time to pass after its own mark_reevaluation_complete().
    core.tick((0.5, 0.5), 0.3)
    out = core.on_hazard_snapshot(snapshot(revision=1))
    assert out.hold is True
    assert out.publish_goal is None
    assert out.status["state"] == "EXIT_RESELECTION_REQUIRED"
    assert out.status["active_exit_id"] == "EXIT1"


# -- parse_active_goal_payload ---------------------------------------------------

def _plan_payload(**overrides):
    payload = {
        "success": True, "activated": True, "selected_exit_id": "EXIT1",
        "selected_approach_position_world": [4.5, 0.5], "hazard_revision": 3,
    }
    payload.update(overrides)
    return json.dumps(payload)


def test_parse_active_goal_payload_accepts_activated_plan():
    goal = parse_active_goal_payload(_plan_payload())
    assert goal == ActiveGoal("EXIT1", (4.5, 0.5), 3)


def test_parse_active_goal_payload_preserves_optional_inspection_yaw():
    goal = parse_active_goal_payload(
        _plan_payload(selected_approach_yaw_rad=1.25)
    )
    assert goal == ActiveGoal("EXIT1", (4.5, 0.5), 3, 1.25)


def test_parse_active_goal_payload_rejects_non_activated_plan():
    assert parse_active_goal_payload(_plan_payload(activated=False)) is None


def test_parse_active_goal_payload_rejects_malformed_json():
    assert parse_active_goal_payload("not json") is None


def test_parse_active_goal_payload_rejects_unsuccessful_plan():
    assert parse_active_goal_payload(_plan_payload(success=False)) is None


# -- Stage 7 boundary: static source scan + behavioral guarantee ---------------

CORE_SOURCE = (
    FsPath(__file__).parents[1] / "inno_autonav" / "replan_supervisor.py"
).read_text(encoding="utf-8")
NODE_SOURCE = (
    FsPath(__file__).parents[1] / "inno_autonav" / "replan_supervisor_node.py"
).read_text(encoding="utf-8")


def test_core_never_calls_plan_evacuation_or_publishes_path_or_cmd_vel():
    # Only reject an actual topic/service-name string literal (quoted), not the
    # substring appearing in prose inside this module's own docstrings/comments
    # that document the exclusion.
    for source in (CORE_SOURCE, NODE_SOURCE):
        assert "'/plan_evacuation'" not in source
        assert '"/plan_evacuation"' not in source
        assert "create_publisher(Path" not in source
        assert "create_publisher(Twist" not in source


def test_node_publishes_hold_before_goal_pose():
    start = NODE_SOURCE.index("def _publish(")
    body = NODE_SOURCE[start:]
    assert body.index("hold_publisher.publish") < body.index("goal_publisher.publish")
