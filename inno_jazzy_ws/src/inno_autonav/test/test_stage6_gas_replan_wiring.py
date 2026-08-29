"""Stage 6: gas-inclusive hazard cost reaches the EXISTING replan pipeline.

No new replanning system. These tests only prove that the gas fusion already
carried by /hazard/snapshot (final_cost inf / blocked mask / co layer) flows
through the unchanged ReplanSupervisorCore + EventReplanningPolicy +
SameExitReplanCoordinator:
  * gas off the remaining path              -> no PATH_INVALID, no replan
  * finite traversable gas cost on the path -> existing "soft cost" policy: no replan
  * gas-blocked cell on the remaining path  -> hold + same-goal replan request
  * new path still gas-blocked              -> hold kept (existing revalidation)
  * repeated identical gas snapshot         -> no duplicate replan
  * gas off                                 -> byte-identical to a no-gas snapshot
Plus the two-line wiring change contract (effective co_blocked_ppm in the
snapshot metadata; the supervisor node's defensive rebuild guard).
"""

from pathlib import Path as FsPath

import numpy as np
import pytest

from inno_autonav.event_replanning import EventReplanningConfig, ReplanReason
from inno_autonav.exit_evaluator import ExitHazardSnapshot
from inno_autonav.replan_supervisor import ActiveGoal, ReplanSupervisorCore
from inno_autonav.same_exit_replanning import ReplanCommand, SameExitReplanCoordinator
from inno_hazard.hazard_belief import (
    HazardBelief, HazardBeliefConfig, HazardGridGeometry,
)

_HAZARD_PKG = FsPath(__file__).resolve().parents[2] / "inno_hazard"

GOAL = ActiveGoal("EXIT1", (4.5, 0.5), 1)
SAFE_PATH = [(0.5, 0.5), (1.5, 0.5), (2.5, 0.5), (3.5, 0.5), (4.5, 0.5)]


def snapshot(size=8, revision=1, co_blocked_ppm=1600.0, **overrides):
    shape = (size, size)
    g = HazardGridGeometry(size, size, 1.0)
    v = {
        "final_cost": np.ones(shape), "temperature_c": np.full(shape, np.nan),
        "co_ppm": np.full(shape, np.nan),
        "observed_mask": np.zeros(shape, bool),
        "temperature_observed_mask": np.zeros(shape, bool),
        "co_observed_mask": np.zeros(shape, bool),
        "fire_probability": np.zeros(shape),
        "static_obstacle_map": np.zeros(shape, bool),
        "dynamic_obstacle_map": np.zeros(shape, bool),
        "blocked_mask": np.zeros(shape, bool),
    }
    v.update(overrides)
    return ExitHazardSnapshot(
        g, v["final_cost"], v["temperature_c"], v["co_ppm"], v["observed_mask"],
        v["temperature_observed_mask"], v["co_observed_mask"],
        v["fire_probability"], v["static_obstacle_map"], v["dynamic_obstacle_map"],
        v["blocked_mask"], revision, 60.0, co_blocked_ppm, 1.0,
    )


def _bootstrap(core, pose=(0.5, 0.5)):
    core.on_active_goal(GOAL)
    core.on_planned_path(SAFE_PATH)
    core.on_hazard_snapshot(snapshot(revision=0))
    assert core.tick(pose, 0.0).status["state"] == "PATH_VALID"


# -- A: gas hazard off the remaining path -> nothing happens ----------------
def test_gas_blocked_off_path_does_not_replan():
    core = ReplanSupervisorCore(EventReplanningConfig())
    _bootstrap(core)
    cost = np.ones((8, 8))
    cost[5, 5] = np.inf          # gas-blocked cell nowhere near y=0.5 path
    blocked = np.zeros((8, 8), bool)
    blocked[5, 5] = True
    out = core.on_hazard_snapshot(
        snapshot(revision=1, final_cost=cost, blocked_mask=blocked)
    )
    assert out.hold is False and out.publish_goal is None
    assert out.status["state"] == "PATH_VALID"


# -- B: finite traversable gas cost on the path -> existing policy: no replan
def test_finite_gas_cost_on_path_does_not_replan():
    core = ReplanSupervisorCore(EventReplanningConfig())
    _bootstrap(core)
    cost = np.ones((8, 8))
    cost[0, 2] = 6.0             # finite, just more expensive (gas soft cost)
    co = np.full((8, 8), np.nan)
    co[0, 2] = 1200.0           # observed gas, still below the 1600 block level
    seen = np.zeros((8, 8), bool)
    seen[0, 2] = True
    out = core.on_hazard_snapshot(snapshot(
        revision=1, final_cost=cost, co_ppm=co, co_observed_mask=seen,
    ))
    assert out.hold is False and out.publish_goal is None


# -- C: gas-blocked cell invades the remaining path -> PATH_INVALID + replan -
def test_gas_blocked_on_path_holds_and_requests_same_goal():
    core = ReplanSupervisorCore(EventReplanningConfig())
    _bootstrap(core)
    cost = np.ones((8, 8))
    cost[0, 2] = np.inf         # gas-blocked cell on the path (final_cost route)
    blocked = np.zeros((8, 8), bool)
    blocked[0, 2] = True
    out = core.on_hazard_snapshot(
        snapshot(revision=1, final_cost=cost, blocked_mask=blocked)
    )
    assert out.hold is True
    assert out.publish_goal == GOAL.approach_world
    assert out.status["state"] == "REPLAN_REQUESTED"
    assert out.status["attempt_count"] == 1
    assert out.status["last_replan_reason"] == ReplanReason.PATH_CELL_BLOCKED.value


# -- C2: co-layer block on the path uses PATH_CO_BLOCKED (effective threshold)
def test_gas_co_layer_block_on_path_uses_co_blocked_reason():
    core = ReplanSupervisorCore(EventReplanningConfig(co_block_ppm=3000.0))
    _bootstrap(core)
    co = np.full((8, 8), np.nan)
    co[0, 2] = 3100.0          # observed gas at/above the effective ADC block
    seen = np.zeros((8, 8), bool)
    seen[0, 2] = True
    out = core.on_hazard_snapshot(snapshot(
        revision=1, co_ppm=co, co_observed_mask=seen, co_blocked_ppm=3000.0,
    ))
    assert out.hold is True
    assert out.status["last_replan_reason"] == ReplanReason.PATH_CO_BLOCKED.value


# -- F: replacement path also gas-blocked -> hold kept -----------------------
def test_replacement_path_still_gas_blocked_keeps_hold():
    core = ReplanSupervisorCore(EventReplanningConfig())
    _bootstrap(core)
    cost = np.ones((8, 8))
    cost[0, 2] = np.inf
    out = core.on_hazard_snapshot(snapshot(revision=1, final_cost=cost))
    assert out.hold is True and out.publish_goal is not None

    still = np.ones((8, 8))
    still[0, 3] = np.inf        # the new path is still gas-blocked further along
    core.on_hazard_snapshot(snapshot(revision=2, final_cost=still))
    core.on_planner_state("PLANNING")
    core.on_planned_path(SAFE_PATH)
    out = core.on_planner_state("PATH_READY")
    assert out.hold is True
    assert out.status["state"] == "REPLAN_FAILED"
    assert out.status["last_failure_reason"].startswith("NEW_PATH_UNSAFE")


# -- G: repeated identical gas snapshot -> no runaway replan ----------------
def test_repeated_identical_gas_snapshot_does_not_stack_requests():
    core = ReplanSupervisorCore(EventReplanningConfig())
    _bootstrap(core)
    cost = np.ones((8, 8))
    cost[0, 2] = np.inf
    first = core.on_hazard_snapshot(snapshot(revision=1, final_cost=cost))
    assert first.status["attempt_count"] == 1
    # same revision, same blocked cell, sensor just re-published
    again = core.on_hazard_snapshot(snapshot(revision=1, final_cost=cost))
    assert again.status["attempt_count"] == 1        # not 2
    assert again.publish_goal is None                # no fresh request emitted


# -- H: gas OFF -> identical to a no-gas snapshot --------------------------
def test_gas_off_snapshot_matches_no_gas_baseline():
    core = ReplanSupervisorCore(EventReplanningConfig())
    _bootstrap(core)
    out = core.on_hazard_snapshot(snapshot(revision=1))   # no co obs, no inf
    assert out.hold is False and out.publish_goal is None
    assert out.status["state"] == "PATH_VALID"
    assert out.status["last_validated_revision"] == 1


# -- D/E: waypoint-first then A* fallback (existing coordinator, unchanged) --
def _result(cmd, success, stamp=100):
    return {**cmd.payload, "success": success, "path_stamp_ns": stamp}


def test_gas_replan_prefers_waypoint_then_falls_back_to_astar():
    coord = SameExitReplanCoordinator()
    coord.on_goal(GOAL)
    waypoint = coord.start(1, 1)
    assert waypoint.topic == "waypoint"                       # waypoint first

    astar = coord.on_waypoint_result(_result(waypoint, False))
    assert isinstance(astar, ReplanCommand) and astar.topic == "astar"  # fallback

    assert coord.on_astar_result(_result(astar, True)) is None
    activation = coord.on_candidate_path(
        "A_STAR", stamp_ns=100, goal_world=GOAL.approach_world, nonempty=True,
    )
    assert activation["mode"] == "A_STAR"


def test_gas_replan_uses_waypoint_path_when_waypoint_succeeds():
    coord = SameExitReplanCoordinator()
    coord.on_goal(GOAL)
    waypoint = coord.start(1, 1)
    assert coord.on_waypoint_result(_result(waypoint, True)) is None
    activation = coord.on_candidate_path(
        "WAYPOINT", stamp_ns=100, goal_world=GOAL.approach_world, nonempty=True,
    )
    assert activation["mode"] == "WAYPOINT"


# -- wiring change contract -------------------------------------------------
def _belief(**cfg):
    g = HazardGridGeometry(6, 6, 1.0)
    return HazardBelief(g, np.zeros((6, 6), bool), HazardBeliefConfig(**cfg))


def test_effective_gas_block_threshold_is_mode_aware():
    # The value hazard_snapshot.py now puts in the snapshot's co_blocked_ppm.
    assert _belief(co_blocked_ppm=1600.0).config.gas_blocked_threshold == 1600.0
    adc = _belief(gas_input_mode="adc", gas_safe_adc=1000.0, gas_blocked_adc=3000.0)
    assert adc.config.gas_blocked_threshold == 3000.0


def test_hazard_snapshot_metadata_uses_effective_threshold_source():
    src = (_HAZARD_PKG / "inno_hazard" / "hazard_snapshot.py").read_text()
    meta_block = src.split("metadata = {", 1)[1].split("}", 1)[0]
    assert '"co_blocked_ppm": float(belief.config.gas_blocked_threshold)' in meta_block


def test_hazard_snapshot_roundtrip_carries_effective_threshold():
    pytest.importorskip("std_msgs")
    from inno_hazard.hazard_snapshot import (
        decode_hazard_snapshot_message, hazard_snapshot_message,
    )
    adc = _belief(gas_input_mode="adc", gas_safe_adc=1000.0, gas_blocked_adc=3000.0)
    meta, _ = decode_hazard_snapshot_message(
        hazard_snapshot_message(adc, np.zeros((6, 6)), status="ACTIVE")
    )
    assert meta["co_blocked_ppm"] == 3000.0


def test_supervisor_node_rebuild_has_defensive_guard():
    src = (FsPath(__file__).resolve().parents[1]
           / "inno_autonav" / "replan_supervisor_node.py").read_text()
    body = src.split("_rebuild_core_if_thresholds_changed", 1)[1].split("def ", 1)[0]
    assert "try:" in body and "except ValueError" in body
    assert "EventReplanningConfig(" in body


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
