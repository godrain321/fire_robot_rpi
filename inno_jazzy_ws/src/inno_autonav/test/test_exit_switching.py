"""Tests for the ported exit_switching.py pure-core components."""

import math

import numpy as np

from inno_autonav.exit_switching import (
    DelayedCostSwitch,
    ExitSwitchingConfig,
    RouteTemperatureTrendMonitor,
    current_direction_world,
    evaluate_path_cost,
    is_opposite_direction,
)


PATH = ((0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0))


def test_field_default_danger_expected_temperature_is_36c():
    assert ExitSwitchingConfig().danger_expected_min_temperature_c == 36.0


def _temp_grid(size, value):
    return np.full((size, size), value)


def _observed(size):
    return np.ones((size, size), dtype=bool)


def _cost_grid(size, value):
    return np.full((size, size), value)


def test_1_below_temperature_threshold_never_trends():
    monitor = RouteTemperatureTrendMonitor(evaluation_window=6, minimum_temperature_c=40.0)
    for revision in range(1, 8):
        decision = monitor.record(
            PATH, _cost_grid(6, 2.0 + 0.2 * revision), _temp_grid(6, 20.0),
            _observed(6), revision=revision, evaluated_at=float(revision),
        )
        assert decision.switch_required is False


def test_2_six_sample_strict_rise_triggers():
    monitor = RouteTemperatureTrendMonitor(evaluation_window=6, minimum_temperature_c=40.0)
    decision = None
    for revision, cost in enumerate((2.0, 2.2, 2.4, 2.6, 2.8, 3.0), start=1):
        decision = monitor.record(
            PATH, _cost_grid(6, cost), _temp_grid(6, 41.0), _observed(6),
            revision=revision, evaluated_at=float(revision),
        )
    assert decision.switch_required is True
    assert decision.consecutive_increases == 5
    assert decision.baseline_average_cost == 2.0
    assert decision.current_average_cost == 3.0


def test_3_mid_sequence_decrease_prevents_trigger():
    monitor = RouteTemperatureTrendMonitor(evaluation_window=6, minimum_temperature_c=40.0)
    decision = None
    for revision, cost in enumerate((2.0, 2.2, 2.4, 2.2, 2.6, 2.8), start=1):
        decision = monitor.record(
            PATH, _cost_grid(6, cost), _temp_grid(6, 41.0), _observed(6),
            revision=revision, evaluated_at=float(revision),
        )
    assert decision.switch_required is False


def test_4_duplicate_revision_is_a_no_op():
    monitor = RouteTemperatureTrendMonitor(evaluation_window=6, minimum_temperature_c=40.0)
    monitor.record(
        PATH, _cost_grid(6, 2.0), _temp_grid(6, 41.0), _observed(6),
        revision=1, evaluated_at=0.0,
    )
    assert len(monitor.samples) == 1
    decision = monitor.record(
        PATH, _cost_grid(6, 99.0), _temp_grid(6, 99.0), _observed(6),
        revision=1, evaluated_at=1.0,
    )
    assert decision == type(decision)(False, 0, None, None, None)
    assert len(monitor.samples) == 1  # the duplicate-revision call recorded nothing


def test_unobserved_high_temperature_placeholder_is_not_evidence():
    monitor = RouteTemperatureTrendMonitor(evaluation_window=6, minimum_temperature_c=40.0)
    unobserved = np.zeros((6, 6), dtype=bool)
    decision = monitor.record(
        PATH, _cost_grid(6, 2.0), _temp_grid(6, 200.0), unobserved,
        revision=1, evaluated_at=0.0,
    )
    assert decision.switch_required is False
    assert len(monitor.samples) == 0


def test_5_delayed_switch_not_ready_under_required_distance():
    switch = DelayedCostSwitch(1.0)
    switch.arm("EXIT2", "sustained_route_cost_increase", 5.0)
    assert switch.ready(5.5) is False


def test_6_delayed_switch_ready_at_required_distance():
    switch = DelayedCostSwitch(1.0)
    switch.arm("EXIT2", "sustained_route_cost_increase", 5.0)
    assert switch.ready(6.0) is True


def test_7_current_direction_prefers_next_waypoint():
    direction = current_direction_world(
        (0.0, 0.0), next_waypoint_world=(1.0, 0.0),
        recent_positions_world=[(0.0, 0.0), (0.0, 1.0)], yaw_rad=math.pi,
    )
    assert direction == (1.0, 0.0)


def test_8_current_direction_falls_back_to_recent_motion():
    direction = current_direction_world(
        (0.0, 0.0), next_waypoint_world=None,
        recent_positions_world=[(0.0, 0.0), (0.0, 1.0)], yaw_rad=math.pi,
    )
    assert direction == (0.0, 1.0)


def test_9_current_direction_falls_back_to_yaw():
    direction = current_direction_world(
        (0.0, 0.0), next_waypoint_world=None, recent_positions_world=(),
        yaw_rad=math.pi / 2,
    )
    assert math.isclose(direction[0], 0.0, abs_tol=1e-9)
    assert math.isclose(direction[1], 1.0, abs_tol=1e-9)


def test_10_is_opposite_direction_matches_angle_semantics():
    direction = (1.0, 0.0)  # travelling east
    robot = (0.0, 0.0)
    assert is_opposite_direction(
        direction, robot, (-5.0, 0.0), minimum_difference_deg=90.0
    ) is True  # due west: 180 degrees away
    assert is_opposite_direction(
        direction, robot, (0.0, 5.0), minimum_difference_deg=90.0
    ) is True  # due north: exactly 90 degrees, inclusive boundary
    assert is_opposite_direction(
        direction, robot, (5.0, 0.0), minimum_difference_deg=90.0
    ) is False  # due east: same direction as travel


def test_11_evaluate_path_cost_includes_sparse_mid_segment_cell():
    cost = np.ones((6, 6))
    cost[0, 2] = 100.0  # not a waypoint in the sparse path below
    result = evaluate_path_cost(((0, 0), (4, 0)), cost)
    assert result is not None
    _, average, maximum = result
    assert maximum == 100.0
    assert average > 1.0  # would stay 1.0 if only endpoints were checked
