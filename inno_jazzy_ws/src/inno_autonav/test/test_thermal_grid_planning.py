import numpy as np
import pytest

from inno_autonav.safe_path_simplifier import expanded_path, simplify_path_safely
from inno_autonav.weighted_planner import combine_cost_grids, weighted_astar_search


def test_static_dynamic_and_thermal_grids_combine_without_mutation():
    static = np.zeros((5, 6), dtype=np.int8)
    static[0, 0] = -1
    static[1, 1] = 100
    dynamic = np.zeros_like(static)
    dynamic[2, 2] = 100
    thermal = np.zeros_like(static)
    thermal[3, 3] = 75
    thermal[4, 4] = 100
    result = combine_cost_grids(
        static, dynamic, thermal, unknown_is_occupied=True
    )
    assert result[0, 0] == -1
    assert result[1, 1] == 100
    assert result[2, 2] == 100
    assert result[3, 3] == 75
    assert result[4, 4] == 100
    assert static[3, 3] == 0


def test_thermal_geometry_mismatch_is_rejected():
    with pytest.raises(ValueError, match='thermal grid geometry'):
        combine_cost_grids(
            np.zeros((4, 4)), None, np.zeros((3, 4)),
            unknown_is_occupied=True,
        )


def test_weighted_plan_and_simplification_keep_away_from_heat():
    static = np.zeros((9, 12), dtype=np.int8)
    thermal = np.zeros_like(static)
    thermal[4, 3:9] = 85
    combined = combine_cost_grids(
        static, None, thermal, unknown_is_occupied=True
    )
    planned = weighted_astar_search(
        combined, (1, 4), (10, 4), allow_diagonal=True
    )
    assert planned.path
    assert all(thermal[y, x] == 0 for x, y in planned.path)
    simplified = simplify_path_safely(planned.path, combined)
    assert simplified.safe
    assert all(thermal[y, x] == 0 for x, y in expanded_path(simplified.path))


def test_thermal_lethal_band_produces_no_path():
    static = np.zeros((5, 7), dtype=np.int8)
    thermal = np.zeros_like(static)
    thermal[:, 3] = 100
    combined = combine_cost_grids(
        static, None, thermal, unknown_is_occupied=True
    )
    assert not weighted_astar_search(combined, (1, 2), (5, 2)).path
