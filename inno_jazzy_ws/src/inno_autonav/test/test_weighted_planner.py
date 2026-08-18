import math

import numpy as np
import pytest

from inno_autonav.weighted_planner import (
    path_cost,
    thermal_readiness_state,
    traversal_multiplier,
    weighted_astar_search,
)


def test_zero_thermal_cost_matches_geometric_shortest_path():
    data = np.zeros((7, 9), dtype=np.int8)
    result = weighted_astar_search(data, (0, 3), (8, 3), allow_diagonal=False)
    assert result.path[0] == (0, 3)
    assert result.path[-1] == (8, 3)
    assert result.total_cost == 8.0


def test_weighted_astar_avoids_finite_hot_corridor():
    data = np.zeros((7, 9), dtype=np.int8)
    data[3, 2:7] = 90
    result = weighted_astar_search(
        data, (0, 3), (8, 3), allow_diagonal=False,
        thermal_cost_weight=8.0, thermal_cost_power=2.0,
    )
    assert result.path
    assert all(data[y, x] == 0 for x, y in result.path)
    assert len(result.path) > 9


def test_cost_100_is_lethal_and_diagonal_corner_cutting_is_rejected():
    data = np.zeros((3, 3), dtype=np.int8)
    data[0, 1] = 100
    data[1, 0] = 100
    result = weighted_astar_search(data, (0, 0), (2, 2))
    assert not result.path
    assert math.isinf(result.total_cost)


def test_thermal_multiplier_and_path_cost_use_power():
    data = np.zeros((1, 3), dtype=np.int8)
    data[0, 1] = 50
    expected = traversal_multiplier(50, 8.0, 2.0) + 1.0
    assert path_cost(
        [(0, 0), (1, 0), (2, 0)], data,
        thermal_cost_weight=8.0, thermal_cost_power=2.0,
    ) == expected
    assert traversal_multiplier(99, 8.0, 2.0) == 9.0


def test_factory_v5_evacuation_formula_with_co_fixed_to_zero():
    # thermal grid value 49.5 represents temperature ratio 0.5 before integer
    # OccupancyGrid quantization. This must match factory_v5's configured
    # 1 + 24 * ratio**1.5 + 8 * co_ratio**2 equation.
    result = traversal_multiplier(
        49.5, 24.0, 1.5,
        fixed_co_ppm=0.0,
        co_safe_ppm=0.0,
        co_blocked_ppm=1600.0,
        co_cost_weight=8.0,
        co_cost_power=2.0,
    )
    assert result == pytest.approx(1.0 + 24.0 * 0.5 ** 1.5)


def test_factory_v5_co_term_is_explicit_and_zero_at_zero_ppm():
    zero_co = traversal_multiplier(0.0, 24.0, 1.5, fixed_co_ppm=0.0)
    half_co = traversal_multiplier(0.0, 24.0, 1.5, fixed_co_ppm=800.0)
    assert zero_co == 1.0
    assert half_co == pytest.approx(1.0 + 8.0 * 0.5 ** 2.0)


def test_unknown_policy_is_preserved():
    data = np.zeros((3, 5), dtype=np.int8)
    data[:, 2] = -1
    assert not weighted_astar_search(
        data, (0, 1), (4, 1), unknown_is_occupied=True, allow_diagonal=False
    ).path
    assert weighted_astar_search(
        data, (0, 1), (4, 1), unknown_is_occupied=False, allow_diagonal=False
    ).path


def test_thermal_fail_safe_states_and_recovery():
    base = dict(
        require_grid=True,
        require_active=True,
        grid_available=False,
        geometry_matches=True,
        status="",
        age_sec=None,
        timeout_sec=1.0,
    )
    assert thermal_readiness_state(**base) == "WAITING_FOR_THERMAL_GRID"
    base.update(grid_available=True, geometry_matches=False)
    assert thermal_readiness_state(**base) == "THERMAL_GRID_MISMATCH"
    base.update(geometry_matches=True, status="ACTIVE", age_sec=1.1)
    assert thermal_readiness_state(**base) == "THERMAL_GRID_STALE"
    base.update(age_sec=0.1, status="WAITING_FOR_TF")
    assert thermal_readiness_state(**base) == "WAITING_FOR_THERMAL_ACTIVE"
    base.update(status="ACTIVE")
    assert thermal_readiness_state(**base) is None
