"""Tests for the Stage 8-2 WaypointCostProjector pure core."""

import math

import numpy as np
import pytest

from inno_autonav.grid_utils import MapGrid
from inno_autonav.waypoint_cost_projector import (
    WaypointCostProjector,
    WaypointCostProjectorConfig,
)


def grid(size=20, resolution=1.0, origin_x=0.0, origin_y=0.0, value=1.0):
    data = np.full((size, size), value, dtype=np.int16)
    return MapGrid(size, size, resolution, origin_x, origin_y, 0.0, "map", data)


def with_cells(base_grid, values: dict):
    """Return a copy of base_grid.data with specific (col,row) cells overwritten."""
    data = base_grid.data.copy()
    for (col, row), value in values.items():
        data[row, col] = value
    return MapGrid(
        base_grid.width, base_grid.height, base_grid.resolution,
        base_grid.origin_x, base_grid.origin_y, base_grid.origin_yaw,
        base_grid.frame_id, data,
    )


def test_1_surrounding_maximum():
    base = grid(size=20, value=0)
    g = with_cells(base, {
        (4, 4): 1, (5, 4): 2, (6, 4): 1,
        (4, 5): 3, (5, 5): 0, (6, 5): 7,
        (4, 6): 2, (5, 6): 4, (6, 6): 1,
    })
    projector = WaypointCostProjector(
        {"W": (5.5, 5.5)}, WaypointCostProjectorConfig(waypoint_cost_radius_m=1.5),
    )
    assert projector.project_costs(g)["W"] == 7.0


def test_2_outside_radius_is_ignored():
    base = grid(size=30, value=0)
    g = with_cells(base, {(15, 5): 50})  # far from the waypoint below
    projector = WaypointCostProjector(
        {"W": (5.5, 5.5)}, WaypointCostProjectorConfig(waypoint_cost_radius_m=1.0),
    )
    assert projector.project_costs(g)["W"] == 0.0


def test_3_radius_boundary_is_circular_not_square():
    base = grid(size=20, value=0)
    projector = WaypointCostProjector(
        {"W": (10.5, 10.5)}, WaypointCostProjectorConfig(waypoint_cost_radius_m=2.0),
    )
    cells = projector.project_costs(base)  # force lookup build
    lookup = projector.cell_lookup["W"]
    center = (10, 10)
    assert (center[0] + 2, center[1]) in lookup  # exactly on the radius: included
    assert (center[0] + 2, center[1] + 2) not in lookup  # diagonal sqrt(8): excluded
    assert cells is not None


def test_4_nonzero_map_origin():
    base = grid(size=20, resolution=0.5, origin_x=100.0, origin_y=-50.0, value=0)
    # World point (101.25, -49.25) -> local (1.25, 0.75) -> grid cell (2, 1)
    g = with_cells(base, {(2, 1): 9})
    projector = WaypointCostProjector(
        {"W": (101.25, -49.25)},
        WaypointCostProjectorConfig(waypoint_cost_radius_m=0.3),
    )
    assert projector.project_costs(g)["W"] == 9.0


def test_5_fine_resolution():
    base = grid(size=40, resolution=0.2, value=0)
    # Waypoint at cell (10,10); a cell 0.8 m away (4 cells at 0.2 m) should be
    # included, one 1.0 m away should not.
    g = with_cells(base, {(14, 10): 5, (15, 10): 99})
    projector = WaypointCostProjector(
        {"W": (2.1, 2.1)}, WaypointCostProjectorConfig(waypoint_cost_radius_m=0.8),
    )
    assert projector.project_costs(g)["W"] == 5.0


def test_6_obstacle_in_radius_makes_waypoint_cost_infinite():
    base = grid(size=20, value=0)
    g = with_cells(base, {(6, 5): 100})  # lethal, matches cell_is_blocked convention
    projector = WaypointCostProjector(
        {"W": (5.5, 5.5)}, WaypointCostProjectorConfig(waypoint_cost_radius_m=1.5),
    )
    assert projector.project_costs(g)["W"] == math.inf


def test_6_unknown_cell_in_radius_is_treated_like_the_rest_of_the_planner():
    base = grid(size=20, value=0)
    g = with_cells(base, {(6, 5): -1})  # unknown
    projector = WaypointCostProjector(
        {"W": (5.5, 5.5)},
        WaypointCostProjectorConfig(waypoint_cost_radius_m=1.5, unknown_is_occupied=True),
    )
    assert projector.project_costs(g)["W"] == math.inf


def test_7_geometry_cache_is_not_rebuilt_when_unchanged():
    g = grid(size=20, value=0)
    projector = WaypointCostProjector({"W": (5.5, 5.5)})
    projector.project_costs(g)
    calls = []
    original = projector._rebuild_cell_lookup
    projector._rebuild_cell_lookup = lambda grid: (calls.append(1), original(grid))[-1]
    projector.project_costs(g)
    projector.project_costs(with_cells(g, {(5, 5): 3}))  # data changes, geometry doesn't
    assert calls == []


def test_8_geometry_change_rebuilds_cache():
    projector = WaypointCostProjector({"W": (5.5, 5.5)})
    g1 = grid(size=20, resolution=1.0, value=0)
    projector.project_costs(g1)
    calls = []
    original = projector._rebuild_cell_lookup
    projector._rebuild_cell_lookup = lambda grid: (calls.append(1), original(grid))[-1]
    g2 = grid(size=20, resolution=0.5, value=0)  # resolution changed
    projector.project_costs(g2)
    assert calls == [1]
    g3 = grid(size=20, resolution=0.5, origin_x=10.0, value=0)  # origin changed
    projector.project_costs(g3)
    assert calls == [1, 1]


def test_9_duplicate_revision_is_a_no_op():
    projector = WaypointCostProjector({"W": (5.5, 5.5)})
    g1 = with_cells(grid(size=20, value=0), {(5, 5): 3})
    first = projector.project_costs(g1, revision=15)
    assert first["W"] == 3.0
    g2 = with_cells(grid(size=20, value=0), {(5, 5): 99})  # would change the result
    second = projector.project_costs(g2, revision=15)  # same revision -> no-op
    assert second["W"] == 3.0
    third = projector.project_costs(g2, revision=16)
    assert third["W"] == 99.0


def test_10_multiple_waypoints_are_independent():
    base = grid(size=30, value=0)
    g = with_cells(base, {(5, 5): 3, (25, 25): 8})
    projector = WaypointCostProjector(
        {"A": (5.5, 5.5), "B": (25.5, 25.5)},
        WaypointCostProjectorConfig(waypoint_cost_radius_m=1.0),
    )
    result = projector.project_costs(g)
    assert result["A"] == 3.0
    assert result["B"] == 8.0


def test_config_rejects_non_positive_radius():
    with pytest.raises(ValueError):
        WaypointCostProjectorConfig(waypoint_cost_radius_m=0.0)


def test_empty_waypoints_rejected():
    with pytest.raises(ValueError):
        WaypointCostProjector({})
