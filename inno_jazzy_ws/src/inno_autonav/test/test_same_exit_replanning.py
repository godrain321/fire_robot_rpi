from inno_autonav.replan_supervisor import ActiveGoal
from inno_autonav.same_exit_replanning import SameExitReplanCoordinator, ReplanCommand


GOAL1 = ActiveGoal("EXIT1", (5.0, 1.0), 7)
GOAL2 = ActiveGoal("EXIT2", (9.0, 2.0), 8)


def result(command, success, stamp=100):
    return {**command.payload, "success": success, "path_stamp_ns": stamp}


def candidate(core, source, stamp=100, goal=GOAL1):
    return core.on_candidate_path(
        source, stamp_ns=stamp, goal_world=goal.approach_world, nonempty=True,
    )


def test_waypoint_success_does_not_request_astar_and_stays_waypoint():
    core = SameExitReplanCoordinator()
    core.on_goal(GOAL1)
    request = core.start(11, 1)
    assert request.topic == "waypoint"
    assert core.on_waypoint_result(result(request, True)) is None
    activation = candidate(core, "WAYPOINT")
    assert activation["mode"] == "WAYPOINT"
    assert core.mode == "WAYPOINT"


def test_waypoint_failure_then_astar_success_switches_only_after_success():
    core = SameExitReplanCoordinator()
    core.on_goal(GOAL1)
    waypoint = core.start(11, 1)
    astar = core.on_waypoint_result(result(waypoint, False))
    assert isinstance(astar, ReplanCommand) and astar.topic == "astar"
    assert core.mode == "WAYPOINT"
    assert core.on_astar_result(result(astar, True)) is None
    activation = candidate(core, "A_STAR")
    assert activation["mode"] == "A_STAR"
    assert core.mode == "A_STAR"


def test_both_fail_reports_attempt_failure_not_exhaustion():
    core = SameExitReplanCoordinator()
    core.on_goal(GOAL1)
    waypoint = core.start(11, 1)
    astar = core.on_waypoint_result(result(waypoint, False))
    assert core.on_astar_result(result(astar, False)) == "A_STAR_FAILED"
    assert core.phase == "FAILED"


def test_new_exit_resets_waypoint_and_rejects_old_waypoint_and_astar_results():
    core = SameExitReplanCoordinator()
    core.on_goal(GOAL1)
    waypoint = core.start(11, 1)
    astar = core.on_waypoint_result(result(waypoint, False))
    core.on_goal(GOAL2)
    assert core.mode == "WAYPOINT"
    assert core.on_astar_result(result(astar, True)) is None
    assert core.on_waypoint_result(result(waypoint, True)) is None


def test_newer_request_rejects_older_same_goal_result():
    core = SameExitReplanCoordinator()
    core.on_goal(GOAL1)
    old = core.start(11, 1)
    new = core.start(12, 2)
    assert core.on_waypoint_result(result(old, True)) is None
    assert core.on_waypoint_result(result(new, True)) is None
    assert candidate(core, "WAYPOINT")["request_id"] == new.payload["request_id"]


def test_astar_path_before_result_activates_only_after_matching_result():
    core = SameExitReplanCoordinator()
    core.on_goal(GOAL1)
    waypoint = core.start(11, 1)
    astar = core.on_waypoint_result(result(waypoint, False))
    assert candidate(core, "A_STAR", stamp=321) is None
    activation = core.on_astar_result(result(astar, True, stamp=321))
    assert activation["mode"] == "A_STAR"
    assert activation["path_stamp_ns"] == 321


def test_success_result_never_activates_empty_or_wrong_stamp_path():
    core = SameExitReplanCoordinator()
    core.on_goal(GOAL1)
    waypoint = core.start(11, 1)
    astar = core.on_waypoint_result(result(waypoint, False))
    assert core.on_astar_result(result(astar, True, stamp=500)) is None
    assert core.on_candidate_path(
        "A_STAR", stamp_ns=500, goal_world=GOAL1.approach_world, nonempty=False,
    ) is None
    assert candidate(core, "A_STAR", stamp=499) is None
    assert core.mode == "WAYPOINT"
