"""Hard A-D / Soft A-H tests (spec sections 50/51) plus Stage 7 boundary proofs
for ExitSwitchingCore.
"""

from pathlib import Path as FsPath

import numpy as np

from inno_autonav.exit_evaluator import ExitHazardSnapshot
from inno_autonav.exit_switching import ExitSwitchingConfig
from inno_autonav.exit_switching_orchestrator import (
    ExitSwitchingCore,
    PeekResult,
    SwitchAck,
)
from inno_autonav.replan_supervisor import ActiveGoal
from inno_hazard.hazard_belief import HazardGridGeometry


EXIT_POSITIONS = {
    "EXIT1": (-20.0, 0.5),   # due west of the robot -- opposite of eastward travel
    "EXIT2": (20.0, 0.5),    # current exit, due east
    "EXIT3": (15.0, 0.5),    # also east -- never opposite
}
GOAL_EXIT2 = ActiveGoal("EXIT2", (19.0, 0.5), 1)
DENSE_PATH_WORLD = [(0.5 + i, 0.5) for i in range(10)]


def snapshot(size=24, revision=1, final_cost=1.0, temperature_c=None, temperature_observed_mask=None):
    shape = (size, size)
    geometry = HazardGridGeometry(size, size, 1.0)
    false = np.zeros(shape, dtype=bool)
    cost = final_cost if isinstance(final_cost, np.ndarray) else np.full(shape, final_cost)
    temp = np.full(shape, np.nan) if temperature_c is None else temperature_c
    observed = false if temperature_observed_mask is None else temperature_observed_mask
    return ExitHazardSnapshot(
        geometry, cost, temp, np.full(shape, np.nan), observed, observed,
        false, np.zeros(shape), false, false, false, revision, 60.0, 1600.0, 1.0,
    )


def core(config=None):
    return ExitSwitchingCore(
        config or ExitSwitchingConfig(
            evaluation_window=6, additional_travel_before_switch_m=1.0,
        ),
        dict(EXIT_POSITIONS),
    )


# -- Hard A: current exit excluded, ordinary path failure -> risk_first False --

def test_hard_a_replan_exhausted_excludes_current_exit():
    c = core()
    c.on_active_goal(GOAL_EXIT2)
    out = c.on_supervisor_status("REPLAN_EXHAUSTED")
    assert out.switch_request is not None
    assert out.switch_request.excluded_exit_ids == ("EXIT2",)
    assert out.switch_request.candidate_exit_ids is None
    assert out.switch_request.risk_first is False
    assert out.switch_request.reason == "replan_exhausted"
    # Latched: repeating the same terminal state does not re-request.
    out2 = c.on_supervisor_status("REPLAN_EXHAUSTED")
    assert out2.switch_request is None


def test_hard_a_exit_reselection_required_uses_risk_first():
    c = core()
    c.on_active_goal(GOAL_EXIT2)
    out = c.on_supervisor_status("EXIT_RESELECTION_REQUIRED")
    assert out.switch_request.risk_first is True
    assert out.switch_request.excluded_exit_ids == ("EXIT2",)


# -- Hard B: no safe alternative -> explicit failure, no forced switch --------

def test_hard_b_no_safe_alternative_reports_failure_and_stays_latched():
    c = core()
    c.on_active_goal(GOAL_EXIT2)
    request = c.on_supervisor_status("REPLAN_EXHAUSTED").switch_request
    out = c.on_switch_result(SwitchAck(request.request_id, False, "NO_SAFE_ALTERNATIVE_EXIT", None))
    assert out.switch_request is None
    assert out.status["last_failure_reason"] == "NO_SAFE_ALTERNATIVE_EXIT"
    # Still in the same terminal state -- must not spam a second request.
    out2 = c.on_supervisor_status("REPLAN_EXHAUSTED")
    assert out2.switch_request is None


# -- Hard C/D: canonical propagation + exhaustion reset via goal change only --

def test_hard_c_and_d_successful_switch_propagates_via_goal_change():
    c = core()
    c.on_active_goal(GOAL_EXIT2)
    request = c.on_supervisor_status("REPLAN_EXHAUSTED").switch_request
    out = c.on_switch_result(SwitchAck(request.request_id, True, "ROUTE_ACTIVATED", "EXIT1"))
    assert out.switch_request is None
    # The core never mutates canonical state itself -- it only reacts once the
    # *separate* /evacuation/plan update reaches on_active_goal().
    out2 = c.on_active_goal(ActiveGoal("EXIT1", (-19.0, 0.5), 2))
    assert out2.status["active_exit_id"] == "EXIT1"
    assert out2.status["hard_switch_latched"] is False
    # Stage 6 reporting the new goal healthy again requests nothing further.
    out3 = c.on_supervisor_status("PATH_VALID")
    assert out3.switch_request is None


# -- Soft A/B: trend must be sustained AND travelled before anything happens --

def _drive_rising_trend(c, costs, temperature=41.0, pose=(0.5, 0.5)):
    c.tick(pose, 0.0)
    last = None
    for revision, cost in enumerate(costs, start=1):
        observed = np.ones((24, 24), dtype=bool)
        last = c.on_hazard_snapshot(snapshot(
            revision=revision, final_cost=cost,
            temperature_c=np.full((24, 24), temperature),
            temperature_observed_mask=observed,
        ))
    return last


def test_soft_a_no_trend_yet_means_no_peek():
    c = core()
    c.on_active_goal(GOAL_EXIT2)
    c.on_planned_path(DENSE_PATH_WORLD)
    out = _drive_rising_trend(c, [2.0, 2.0, 2.0])
    assert out.peek_request is None
    assert out.switch_request is None


def test_soft_b_trend_triggers_but_not_yet_travelled_1m():
    c = core()
    c.on_active_goal(GOAL_EXIT2)
    c.on_planned_path(DENSE_PATH_WORLD)
    out = _drive_rising_trend(c, [2.0, 2.2, 2.4, 2.6, 2.8, 3.0])
    assert out.peek_request is None  # armed, but travelled_distance_m is still 0
    assert out.status["delayed_switch_active"] is True


# -- Soft C: 1m travelled + opposite candidate + genuinely lower cost -> switch

def test_soft_c_ready_emits_opposite_only_peek_then_switches_if_better():
    c = core()
    c.on_active_goal(GOAL_EXIT2)
    c.on_planned_path(DENSE_PATH_WORLD)
    _drive_rising_trend(c, [2.0, 2.2, 2.4, 2.6, 2.8, 3.0])
    out = c.tick((1.6, 0.5), 10.0)  # 1.1 m of travel since arming
    assert out.peek_request is not None
    assert out.peek_request.candidate_exit_ids == ("EXIT1",)  # only the opposite exit
    assert out.peek_request.excluded_exit_ids == ("EXIT2",)
    out2 = c.on_peek_result(PeekResult(True, "EXIT1", 2.0, False))  # 2.0 < 3.0 baseline
    assert out2.switch_request is not None
    assert out2.switch_request.candidate_exit_ids == ("EXIT1",)


def test_soft_d_no_opposite_candidate_requests_any_safe_exit():
    east_only = {"EXIT2": (20.0, 0.5), "EXIT3": (15.0, 0.5)}
    c = ExitSwitchingCore(
        ExitSwitchingConfig(evaluation_window=6, additional_travel_before_switch_m=1.0),
        east_only,
    )
    c.on_active_goal(GOAL_EXIT2)
    c.on_planned_path(DENSE_PATH_WORLD)
    _drive_rising_trend(c, [2.0, 2.2, 2.4, 2.6, 2.8, 3.0])
    out = c.tick((1.6, 0.5), 10.0)
    assert out.peek_request is not None
    assert out.peek_request.candidate_exit_ids is None  # unrestricted -- no opposite exit exists


def test_soft_e_replacement_not_better_keeps_current_route():
    c = core()
    c.on_active_goal(GOAL_EXIT2)
    c.on_planned_path(DENSE_PATH_WORLD)
    _drive_rising_trend(c, [2.0, 2.2, 2.4, 2.6, 2.8, 3.0])
    c.tick((1.6, 0.5), 10.0)
    out = c.on_peek_result(PeekResult(True, "EXIT1", 5.0, False))  # 5.0 is worse than 3.0
    assert out.switch_request is None
    assert out.status["delayed_switch_active"] is False  # cleared, no forcing


def test_soft_e_peek_failure_also_keeps_current_route():
    c = core()
    c.on_active_goal(GOAL_EXIT2)
    c.on_planned_path(DENSE_PATH_WORLD)
    _drive_rising_trend(c, [2.0, 2.2, 2.4, 2.6, 2.8, 3.0])
    c.tick((1.6, 0.5), 10.0)
    out = c.on_peek_result(PeekResult(False, None, None, True))
    assert out.switch_request is None


def test_soft_f_usable_gate_is_a_documented_no_op():
    # Section 31's "skip soft switching on a confirmed-USABLE current exit" gate
    # has no live signal in ROS today (see exit_switching_orchestrator.py's
    # module docstring / Stage 7 report) -- ExitSwitchingCore's public interface
    # intentionally has no exit-status input at all, so soft switching always
    # proceeds exactly like the tests above regardless of any such status.
    import inspect
    assert "exit_status" not in inspect.signature(ExitSwitchingCore.on_active_goal).parameters


# -- Soft G/H: cooldown gates soft switches but never hard ones ---------------

def test_soft_g_cooldown_suppresses_a_new_soft_arm():
    c = ExitSwitchingCore(
        ExitSwitchingConfig(evaluation_window=6, switch_cooldown_sec=1000.0),
        dict(EXIT_POSITIONS),
    )
    c.on_active_goal(GOAL_EXIT2)
    c.on_planned_path(DENSE_PATH_WORLD)
    c.tick((0.5, 0.5), 0.0)
    request = c.on_supervisor_status("REPLAN_EXHAUSTED").switch_request
    c.on_switch_result(SwitchAck(request.request_id, True, "ROUTE_ACTIVATED", "EXIT1"))
    c.on_active_goal(ActiveGoal("EXIT1", (-19.0, 0.5), 2))  # last_switch_time recorded
    c.on_planned_path(DENSE_PATH_WORLD)
    out = _drive_rising_trend(c, [2.0, 2.2, 2.4, 2.6, 2.8, 3.0], pose=(0.5, 0.5))
    assert out.status["delayed_switch_active"] is False  # cooldown blocked arming
    assert out.peek_request is None


def test_soft_h_cooldown_does_not_block_a_hard_trigger():
    c = ExitSwitchingCore(
        ExitSwitchingConfig(evaluation_window=6, switch_cooldown_sec=1000.0),
        dict(EXIT_POSITIONS),
    )
    c.on_active_goal(GOAL_EXIT2)
    request = c.on_supervisor_status("REPLAN_EXHAUSTED").switch_request
    c.on_switch_result(SwitchAck(request.request_id, True, "ROUTE_ACTIVATED", "EXIT1"))
    c.on_active_goal(ActiveGoal("EXIT1", (-19.0, 0.5), 2))
    out = c.on_supervisor_status("REPLAN_EXHAUSTED")  # a fresh hard failure on EXIT1
    assert out.switch_request is not None
    assert out.switch_request.excluded_exit_ids == ("EXIT1",)


# -- Stage 7 boundary: source scan -----------------------------------------

_SOURCE_DIR = FsPath(__file__).parents[1] / "inno_autonav"


def test_no_stage7_file_publishes_path_cmd_vel_or_calls_plan_evacuation():
    for name in ("exit_switching.py", "exit_switching_orchestrator.py", "exit_switching_node.py"):
        text = (_SOURCE_DIR / name).read_text(encoding="utf-8")
        assert "create_publisher(Path" not in text
        assert "create_publisher(Twist" not in text
        assert "'/plan_evacuation'" not in text
        assert '"/plan_evacuation"' not in text
