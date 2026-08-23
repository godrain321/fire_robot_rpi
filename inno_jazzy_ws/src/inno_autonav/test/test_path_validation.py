"""Tests for validate_remaining_path/remaining_path_from_pose (spec section 42 D/E/F
plus the individual rejection-reason cases from section 18/20-22)."""

import numpy as np

from inno_autonav.event_replanning import (
    PathRejectionReason,
    remaining_path_from_pose,
    validate_remaining_path,
)
from inno_hazard.hazard_belief import HazardGridGeometry
from inno_autonav.exit_evaluator import ExitHazardSnapshot


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


def test_safe_straight_path_is_safe():
    result = validate_remaining_path(((0, 0), (1, 0), (2, 0)), snapshot())
    assert result.safe is True
    assert result.rejection_reasons == ()


def test_sparse_waypoints_do_not_hide_a_mid_segment_obstacle():
    static = np.zeros((8, 8), dtype=bool)
    static[0, 2] = True  # row=0, col=2 -- not a waypoint, but on the segment
    result = validate_remaining_path(((0, 0), (5, 0)), snapshot(static_obstacle_map=static))
    assert result.safe is False
    assert PathRejectionReason.STATIC_OBSTACLE in result.rejection_reasons
    assert result.first_unsafe_cell == (2, 0)


def test_out_of_map_waypoint_is_rejected():
    result = validate_remaining_path(((0, 0), (99, 99)), snapshot())
    assert result.safe is False
    assert PathRejectionReason.OUT_OF_MAP in result.rejection_reasons


def test_dynamic_obstacle_on_segment_is_rejected():
    dynamic = np.zeros((8, 8), dtype=bool)
    dynamic[0, 3] = True
    result = validate_remaining_path(((0, 0), (5, 0)), snapshot(dynamic_obstacle_map=dynamic))
    assert result.safe is False
    assert PathRejectionReason.DYNAMIC_OBSTACLE in result.rejection_reasons


def test_temperature_hard_block_on_observed_cell_is_rejected():
    temperature = np.zeros((8, 8))
    temperature[0, 3] = 61.0
    observed = np.zeros((8, 8), dtype=bool)
    observed[0, 3] = True
    result = validate_remaining_path(
        ((0, 0), (5, 0)),
        snapshot(temperature_c=temperature, temperature_observed_mask=observed),
    )
    assert result.safe is False
    assert PathRejectionReason.TEMPERATURE_LIMIT_EXCEEDED in result.rejection_reasons


def test_unobserved_temperature_placeholder_is_not_rejected():
    temperature = np.full((8, 8), 200.0)
    result = validate_remaining_path(
        ((0, 0), (5, 0)), snapshot(temperature_c=temperature),
    )
    assert result.safe is True


def test_co_hard_block_on_observed_cell_is_rejected():
    co = np.zeros((8, 8))
    co[0, 3] = 1601.0
    observed = np.zeros((8, 8), dtype=bool)
    observed[0, 3] = True
    result = validate_remaining_path(
        ((0, 0), (5, 0)), snapshot(co_ppm=co, co_observed_mask=observed),
    )
    assert result.safe is False
    assert PathRejectionReason.CO_LIMIT_EXCEEDED in result.rejection_reasons


def test_invalid_cost_cell_is_rejected():
    cost = np.ones((8, 8))
    cost[0, 3] = np.inf
    result = validate_remaining_path(((0, 0), (5, 0)), snapshot(final_cost=cost))
    assert result.safe is False
    assert PathRejectionReason.INVALID_COST in result.rejection_reasons


def test_diagonal_corner_cutting_is_labelled():
    static = np.zeros((8, 8), dtype=bool)
    static[0, 1] = True  # side cell of the (0,0)->(1,1) diagonal crossing
    result = validate_remaining_path(((0, 0), (1, 1)), snapshot(static_obstacle_map=static))
    assert result.safe is False
    assert PathRejectionReason.CORNER_CUTTING in result.rejection_reasons


def test_remaining_path_from_pose_drops_cells_behind_robot():
    full_path = ((0, 0), (1, 0), (2, 0), (3, 0), (4, 0))
    remaining = remaining_path_from_pose(full_path, (2, 0))
    assert remaining == ((2, 0), (3, 0), (4, 0))


def test_obstacle_behind_robot_does_not_invalidate_remaining_path():
    static = np.zeros((8, 8), dtype=bool)
    static[0, 0] = True  # behind the robot
    full_path = ((0, 0), (1, 0), (2, 0), (3, 0), (4, 0))
    remaining = remaining_path_from_pose(full_path, (2, 0))
    result = validate_remaining_path(remaining, snapshot(static_obstacle_map=static))
    assert result.safe is True
