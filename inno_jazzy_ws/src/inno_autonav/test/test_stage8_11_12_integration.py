"""ROS-independent acceptance tests for Stage 8-11/8-12 orchestration."""

import numpy as np

from inno_autonav.event_replanning import EventReplanningConfig
from inno_autonav.exit_evaluator import ExitHazardSnapshot
from inno_autonav.exit_switching import ExitSwitchingConfig
from inno_autonav.exit_switching_orchestrator import ExitSwitchingCore
from inno_autonav.path_selector import PathSelectorCore
from inno_autonav.replan_supervisor import ActiveGoal, ReplanSupervisorCore, RetryConfig
from inno_autonav.same_exit_replanning import SameExitReplanCoordinator
from inno_hazard.hazard_belief import HazardGridGeometry


EXIT1 = ActiveGoal("EXIT1", (4.5, 0.5), 1)
EXIT2 = ActiveGoal("EXIT2", (4.5, 4.5), 2)
OLD_PATH = [(0.5, 0.5), (1.5, 0.5), (2.5, 0.5), (3.5, 0.5), (4.5, 0.5)]
SAFE_DETOUR = [(0.5, 0.5), (0.5, 1.5), (4.5, 1.5), (4.5, 0.5)]


def snapshot(revision, blocked=()):
    shape = (6, 6)
    dynamic = np.zeros(shape, dtype=bool)
    for col, row in blocked:
        dynamic[row, col] = True
    false = np.zeros(shape, dtype=bool)
    return ExitHazardSnapshot(
        HazardGridGeometry(6, 6, 1.0), np.ones(shape), np.full(shape, np.nan),
        np.full(shape, np.nan), false, false, false, np.zeros(shape), false,
        dynamic, false, revision, 60.0, 1600.0, 1.0,
    )


def unsafe_supervisor(max_attempts=2):
    supervisor = ReplanSupervisorCore(
        EventReplanningConfig(),
        RetryConfig(max_replan_attempts=max_attempts, cooldown_seconds=0.0),
    )
    supervisor.on_active_goal(EXIT1)
    supervisor.on_planned_path(OLD_PATH)
    supervisor.on_hazard_snapshot(snapshot(0))
    supervisor.tick((0.5, 0.5), 0.0)
    out = supervisor.on_hazard_snapshot(snapshot(1, blocked=((2, 0),)))
    assert out.hold and out.publish_goal == EXIT1.approach_world
    return supervisor, out


def astar_activation(coordinator, request, stamp=101):
    astar = coordinator.on_waypoint_result({**request.payload, "success": False})
    coordinator.on_candidate_path(
        "A_STAR", stamp_ns=stamp, goal_world=EXIT1.approach_world, nonempty=True,
    )
    return coordinator.on_astar_result({
        **astar.payload, "success": True, "path_stamp_ns": stamp,
    })


def test_astar_success_relays_exact_payload_then_safe_validation_releases_hold():
    supervisor, out = unsafe_supervisor()
    coordinator = SameExitReplanCoordinator()
    coordinator.on_goal(EXIT1)
    request = coordinator.start(out.status["hazard_revision"], 1)
    selector = PathSelectorCore("WAYPOINT")
    candidate = object()
    assert not selector.on_astar_path(candidate).publish
    activation = astar_activation(coordinator, request)
    selected = selector.set_mode(activation["mode"])
    assert selected.publish and selected.payload is candidate
    supervisor.on_hazard_snapshot(snapshot(2))
    supervisor.on_planned_path(SAFE_DETOUR)
    result = supervisor.on_planner_state("PATH_READY")
    assert result.hold is False
    assert result.status["state"] in ("REPLAN_SUCCEEDED", "PATH_VALID")
    assert result.status["last_validated_revision"] == 2


def test_astar_path_returned_but_unsafe_keeps_hold_and_fails_attempt():
    supervisor, out = unsafe_supervisor()
    coordinator = SameExitReplanCoordinator(); coordinator.on_goal(EXIT1)
    request = coordinator.start(out.status["hazard_revision"], 1)
    assert astar_activation(coordinator, request)["mode"] == "A_STAR"
    supervisor.on_planned_path(OLD_PATH)
    result = supervisor.on_planner_state("PATH_READY")
    assert result.hold is True
    assert result.status["state"] in ("REPLAN_FAILED", "REPLAN_REQUESTED")
    assert result.status["last_failure_reason"].startswith("NEW_PATH_UNSAFE")


def test_all_waypoint_and_astar_attempts_fail_before_stage7_hard_switch():
    supervisor, out = unsafe_supervisor(max_attempts=2)
    coordinator = SameExitReplanCoordinator(); coordinator.on_goal(EXIT1)
    for attempt in (1, 2):
        request = coordinator.start(out.status["hazard_revision"], attempt)
        astar = coordinator.on_waypoint_result({**request.payload, "success": False})
        assert coordinator.on_astar_result({**astar.payload, "success": False}) == "A_STAR_FAILED"
        out = supervisor.on_planner_state("NO_PATH")
        if attempt == 1:
            out = supervisor.tick((0.5, 0.5), float(attempt))
            assert out.status["state"] == "REPLAN_REQUESTED"
    out = supervisor.tick((0.5, 0.5), 3.0)
    assert out.status["state"] == "REPLAN_EXHAUSTED"
    switcher = ExitSwitchingCore(
        ExitSwitchingConfig(), {"EXIT1": EXIT1.approach_world, "EXIT2": EXIT2.approach_world},
    )
    switcher.on_active_goal(EXIT1)
    switch = switcher.on_supervisor_status("REPLAN_EXHAUSTED").switch_request
    assert switch is not None and switch.excluded_exit_ids == ("EXIT1",)


def test_new_exit_invalidates_old_astar_and_starts_waypoint_first_again():
    coordinator = SameExitReplanCoordinator(); coordinator.on_goal(EXIT1)
    request = coordinator.start(1, 1)
    astar = coordinator.on_waypoint_result({**request.payload, "success": False})
    coordinator.on_candidate_path(
        "A_STAR", stamp_ns=7, goal_world=EXIT1.approach_world, nonempty=True,
    )
    coordinator.on_goal(EXIT2)
    assert coordinator.mode == "WAYPOINT"
    assert coordinator.on_astar_result({
        **astar.payload, "success": True, "path_stamp_ns": 7,
    }) is None
    new_request = coordinator.start(2, 1)
    assert new_request.topic == "waypoint"
    new_astar = coordinator.on_waypoint_result({**new_request.payload, "success": False})
    assert new_astar.topic == "astar" and new_astar.payload["exit_id"] == "EXIT2"
