"""Spec section 41 parity tests for the ported EventReplanningPolicy (Tests 1-14)."""

import math

import numpy as np
import pytest

from inno_autonav.event_replanning import (
    EventReplanningConfig,
    EventReplanningPolicy,
    ReplanPriority,
    ReplanReason,
)


def policy(**overrides):
    return EventReplanningPolicy(EventReplanningConfig(**overrides))


def costmap(size=6, value=1.0):
    return np.full((size, size), value, dtype=float)


PATH = ((1, 1), (2, 1), (3, 1), (4, 1))


def test_1_safe_path_requires_no_replan():
    decision = policy().evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    assert decision.required is False
    assert decision.reason is ReplanReason.NONE


def test_2_dynamic_obstacle_on_path():
    dynamic = np.zeros((6, 6), dtype=bool)
    dynamic[1, 3] = True  # row=1, col=3 -> cell (3,1) on PATH
    decision = policy().evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        dynamic_obstacle_map=dynamic, robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    assert decision.required is True
    assert decision.reason is ReplanReason.DYNAMIC_OBSTACLE_ON_PATH
    assert decision.priority is ReplanPriority.PATH_BLOCKED
    assert decision.affected_cell_grid == (3, 1)


def test_3_blocked_cost_cell():
    grid = costmap()
    grid[1, 2] = math.inf  # cell (2,1)
    decision = policy().evaluate(
        current_path=PATH, current_costmap=grid, costmap_revision=1,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    assert decision.required is True
    assert decision.reason is ReplanReason.PATH_CELL_BLOCKED
    assert decision.affected_cell_grid == (2, 1)


def test_4_out_of_map():
    path = ((1, 1), (2, 1), (99, 99))
    decision = policy().evaluate(
        current_path=path, current_costmap=costmap(), costmap_revision=1,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    assert decision.required is True
    assert decision.reason is ReplanReason.PATH_INVALID
    assert decision.affected_cell_grid == (99, 99)


def test_5_thermal_hard_block():
    temperature = np.zeros((6, 6))
    temperature[1, 2] = 61.0
    observed = np.zeros((6, 6), dtype=bool)
    observed[1, 2] = True
    decision = policy().evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        temperature_map=temperature, temperature_observed_mask=observed,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    assert decision.required is True
    assert decision.reason is ReplanReason.PATH_TEMPERATURE_BLOCKED
    assert decision.priority is ReplanPriority.HAZARD_BLOCKED


def test_6_unobserved_high_placeholder_is_not_evidence():
    temperature = np.full((6, 6), 200.0)  # implausibly high, but unobserved
    observed = np.zeros((6, 6), dtype=bool)
    decision = policy().evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        temperature_map=temperature, temperature_observed_mask=observed,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    assert decision.required is False
    assert decision.reason is ReplanReason.NONE


def test_7_co_hard_block():
    co = np.zeros((6, 6))
    co[1, 3] = 1601.0
    observed = np.zeros((6, 6), dtype=bool)
    observed[1, 3] = True
    decision = policy().evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        co_map=co, co_observed_mask=observed,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    assert decision.required is True
    assert decision.reason is ReplanReason.PATH_CO_BLOCKED


def _temperature_fixture(value):
    temperature = np.zeros((6, 6))
    temperature[1, 2] = value
    observed = np.zeros((6, 6), dtype=bool)
    observed[1, 2] = True
    return temperature, observed


def test_8_hysteresis_latch_survives_readings_between_release_and_block():
    core = policy()
    temperature, observed = _temperature_fixture(61.0)
    latched = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        temperature_map=temperature, temperature_observed_mask=observed,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    assert latched.reason is ReplanReason.PATH_TEMPERATURE_BLOCKED
    core.mark_processed(
        latched, costmap_revision=1, elapsed_time=0.0,
        robot_pose=(1.0, 1.0), selected_exit_id="EXIT1",
    )
    for step, value in enumerate((59.0, 58.0, 56.0), start=1):
        temperature, observed = _temperature_fixture(value)
        decision = core.evaluate(
            current_path=PATH, current_costmap=costmap(), costmap_revision=1 + step,
            temperature_map=temperature, temperature_observed_mask=observed,
            robot_pose=(1.0, 1.0), elapsed_time=float(step),
        )
        assert decision.reason is ReplanReason.PATH_TEMPERATURE_BLOCKED, (
            f"latch released too early at {value} C (release threshold is 55.0 C)"
        )
        core.mark_processed(
            decision, costmap_revision=1 + step, elapsed_time=float(step),
            robot_pose=(1.0, 1.0), selected_exit_id="EXIT1",
        )


def test_9_hysteresis_release_after_enough_low_observations():
    core = policy()
    temperature, observed = _temperature_fixture(61.0)
    latched = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        temperature_map=temperature, temperature_observed_mask=observed,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    core.mark_processed(
        latched, costmap_revision=1, elapsed_time=0.0,
        robot_pose=(1.0, 1.0), selected_exit_id="EXIT1",
    )
    last = None
    for step in range(1, 4):  # release_confirmation_observations default = 3
        temperature, observed = _temperature_fixture(50.0)
        last = core.evaluate(
            current_path=PATH, current_costmap=costmap(), costmap_revision=1 + step,
            temperature_map=temperature, temperature_observed_mask=observed,
            robot_pose=(1.0, 1.0), elapsed_time=float(step),
        )
        if last.required:
            core.mark_processed(
                last, costmap_revision=1 + step, elapsed_time=float(step),
                robot_pose=(1.0, 1.0), selected_exit_id="EXIT1",
            )
    assert last.reason is not ReplanReason.PATH_TEMPERATURE_BLOCKED


def test_10_duplicate_suppression_same_reason_same_revision():
    core = policy()
    dynamic = np.zeros((6, 6), dtype=bool)
    dynamic[1, 3] = True
    first = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=5,
        dynamic_obstacle_map=dynamic, robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    assert first.required is True
    core.mark_processed(
        first, costmap_revision=5, elapsed_time=0.0,
        robot_pose=(1.0, 1.0), selected_exit_id="EXIT1",
    )
    second = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=5,
        dynamic_obstacle_map=dynamic, robot_pose=(1.0, 1.0), elapsed_time=0.05,
    )
    assert second.required is False


def test_11_new_revision_is_not_suppressed():
    core = policy()
    dynamic = np.zeros((6, 6), dtype=bool)
    dynamic[1, 3] = True
    first = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=5,
        dynamic_obstacle_map=dynamic, robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    core.mark_processed(
        first, costmap_revision=5, elapsed_time=0.0,
        robot_pose=(1.0, 1.0), selected_exit_id="EXIT1",
    )
    second = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=6,
        dynamic_obstacle_map=dynamic, robot_pose=(1.0, 1.0), elapsed_time=0.05,
    )
    assert second.required is True
    assert second.reason is ReplanReason.DYNAMIC_OBSTACLE_ON_PATH


def test_12_path_blocked_outranks_periodic():
    core = policy()
    baseline = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    assert baseline.required is False
    dynamic = np.zeros((6, 6), dtype=bool)
    dynamic[1, 3] = True
    decision = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=2,
        dynamic_obstacle_map=dynamic, robot_pose=(1.0, 1.0), elapsed_time=10.0,
    )
    assert decision.reason is ReplanReason.DYNAMIC_OBSTACLE_ON_PATH
    assert decision.priority is ReplanPriority.PATH_BLOCKED


def test_13_periodic_reevaluation():
    core = policy(periodic_interval_s=5.0)
    baseline = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        robot_pose=(1.0, 1.0), elapsed_time=0.0,
    )
    assert baseline.required is False
    decision = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        robot_pose=(1.0, 1.0), elapsed_time=5.5,
    )
    assert decision.required is True
    assert decision.reason is ReplanReason.PERIODIC_REEVALUATION
    assert decision.priority is ReplanPriority.PERIODIC


def test_14_distance_reevaluation():
    core = policy(periodic_interval_s=5.0, periodic_travel_distance_m=2.0)
    baseline = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        robot_pose=(0.0, 0.0), elapsed_time=0.0,
    )
    assert baseline.required is False
    decision = core.evaluate(
        current_path=PATH, current_costmap=costmap(), costmap_revision=1,
        robot_pose=(3.0, 0.0), elapsed_time=1.0,
    )
    assert decision.required is True
    assert decision.reason is ReplanReason.DISTANCE_REEVALUATION
