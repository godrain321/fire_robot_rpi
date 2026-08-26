"""Stage 8-3/8-4 tests: waypoint->cell caching and revision-gated updates on top
of the Stage 8-2 WaypointCostProjector (test_waypoint_cost_projector.py covers
the base radius/max/obstacle behavior; this file covers the cache/revision
additions and the real 159-waypoint file).
"""

import math

import numpy as np

from inno_autonav.grid_utils import MapGrid, world_to_grid
from inno_autonav.project_paths import project_path
from inno_autonav.waypoint_cost_projector import (
    WaypointCostProjector,
    WaypointCostProjectorConfig,
)
from inno_autonav.waypoint_selection import (
    load_waypoint_document,
    named_waypoints_from_document,
)
from inno_autonav.weighted_planner import cell_is_blocked


def grid(size=20, resolution=1.0, origin_x=0.0, origin_y=0.0, value=1.0):
    data = np.full((size, size), value, dtype=np.int16)
    return MapGrid(size, size, resolution, origin_x, origin_y, 0.0, "map", data)


def with_cells(base_grid, values: dict):
    data = base_grid.data.copy()
    for (col, row), value in values.items():
        data[row, col] = value
    return MapGrid(
        base_grid.width, base_grid.height, base_grid.resolution,
        base_grid.origin_x, base_grid.origin_y, base_grid.origin_yaw,
        base_grid.frame_id, data,
    )


WAYPOINTS = {"A": (5.5, 5.5), "B": (12.5, 12.5)}


# -- Test 1: first build ------------------------------------------------------

def test_1_first_build_creates_cache_and_costs():
    projector = WaypointCostProjector(WAYPOINTS)
    status = projector.status()
    assert status["cache_initialized"] is False
    costs = projector.project_costs(grid(size=20, value=2))
    status = projector.status()
    assert status["cache_initialized"] is True
    assert status["cached_waypoint_count"] == 2
    assert status["cache_rebuild_count"] == 1
    assert status["projection_count"] == 1
    assert costs["A"] == 2.0 and costs["B"] == 2.0


# -- Test 2: cache reuse on a new costmap, same geometry ----------------------

def test_2_cache_reused_but_cost_reflects_new_grid():
    projector = WaypointCostProjector(WAYPOINTS)
    g1 = grid(size=20, value=1)
    projector.project_costs(g1)
    rebuilds_before = projector.status()["cache_rebuild_count"]
    g2 = with_cells(grid(size=20, value=1), {(5, 5): 9})
    costs = projector.project_costs(g2)
    assert projector.status()["cache_rebuild_count"] == rebuilds_before
    assert costs["A"] == 9.0


# -- Test 3/4/5: geometry changes rebuild -------------------------------------

def test_3_resolution_change_rebuilds():
    projector = WaypointCostProjector(WAYPOINTS)
    projector.project_costs(grid(size=20, resolution=1.0))
    projector.project_costs(grid(size=20, resolution=0.5))
    assert projector.status()["cache_rebuild_count"] == 2


def test_4_origin_change_rebuilds():
    projector = WaypointCostProjector(WAYPOINTS)
    projector.project_costs(grid(size=20))
    projector.project_costs(grid(size=20, origin_x=3.0))
    assert projector.status()["cache_rebuild_count"] == 2


def test_5_size_change_rebuilds():
    projector = WaypointCostProjector(WAYPOINTS)
    projector.project_costs(grid(size=20))
    projector.project_costs(grid(size=30))
    assert projector.status()["cache_rebuild_count"] == 2


# -- Test 6: waypoint set changes rebuild; unchanged reload does not ---------

def test_6_waypoint_change_rebuilds_but_identical_reload_does_not():
    projector = WaypointCostProjector(WAYPOINTS)
    projector.project_costs(grid(size=20))
    projector.set_waypoints(dict(WAYPOINTS))  # identical -- no invalidation
    assert projector.status()["geometry_key"] is not None
    projector.project_costs(grid(size=20))
    assert projector.status()["cache_rebuild_count"] == 1

    projector.set_waypoints({"A": (6.5, 5.5), "B": (12.5, 12.5)})  # A moved
    assert projector.status()["cache_initialized"] is False
    projector.project_costs(grid(size=20))
    assert projector.status()["cache_rebuild_count"] == 2


# -- Test 7/8: revision dedup --------------------------------------------------

def test_7_duplicate_revision_is_a_projection_no_op():
    projector = WaypointCostProjector(WAYPOINTS)
    g1 = with_cells(grid(size=20, value=0), {(5, 5): 3})
    projector.project_costs(g1, revision=100)
    count_after_first = projector.status()["projection_count"]
    g2 = with_cells(grid(size=20, value=0), {(5, 5): 99})  # would change the result
    costs = projector.project_costs(g2, revision=100)
    assert projector.status()["projection_count"] == count_after_first
    assert costs["A"] == 3.0  # stale on purpose: same revision -> no-op


def test_8_new_revision_recomputes():
    projector = WaypointCostProjector(WAYPOINTS)
    g1 = with_cells(grid(size=20, value=0), {(5, 5): 3})
    projector.project_costs(g1, revision=100)
    g2 = with_cells(grid(size=20, value=0), {(5, 5): 99})
    costs = projector.project_costs(g2, revision=101)
    assert costs["A"] == 99.0
    assert projector.status()["last_revision"] == 101


# -- Test 9: cost-only change without geometry change, no revision given -----

def test_9_cost_change_without_geometry_change_reuses_cache():
    projector = WaypointCostProjector(WAYPOINTS)
    g1 = with_cells(grid(size=20, value=0), {(5, 5): 3})
    projector.project_costs(g1)
    rebuilds = projector.status()["cache_rebuild_count"]
    g2 = with_cells(grid(size=20, value=0), {(5, 5): 80})
    costs = projector.project_costs(g2)
    assert projector.status()["cache_rebuild_count"] == rebuilds
    assert costs["A"] == 80.0


# -- Test 10: cached result matches an independent direct calculation --------

def _direct_max_cost(waypoints_world, source_grid, radius_m, unknown_is_occupied=True):
    """Independent (non-cached) reference implementation for parity checking."""
    radius_cells = int(math.ceil(radius_m / source_grid.resolution))
    results = {}
    for waypoint_id, (x, y) in waypoints_world.items():
        center_col, center_row = world_to_grid(x, y, source_grid)
        best = -math.inf
        blocked = False
        for row in range(center_row - radius_cells, center_row + radius_cells + 1):
            for col in range(center_col - radius_cells, center_col + radius_cells + 1):
                distance_m = math.hypot(col - center_col, row - center_row) * source_grid.resolution
                if distance_m > radius_m + 1e-12:
                    continue
                if cell_is_blocked(source_grid.data, (col, row), unknown_is_occupied, False):
                    blocked = True
                    break
                best = max(best, float(source_grid.data[row, col]))
            if blocked:
                break
        results[waypoint_id] = math.inf if blocked else best
    return results


def test_10_cached_result_matches_direct_calculation():
    rng = np.random.default_rng(42)
    waypoints = {f"W{i}": (float(rng.uniform(1, 18)), float(rng.uniform(1, 18))) for i in range(15)}
    data = rng.integers(0, 100, size=(20, 20)).astype(np.int16)
    source_grid = MapGrid(20, 20, 0.5, -3.0, 2.0, 0.0, "map", data)
    config = WaypointCostProjectorConfig(waypoint_cost_radius_m=0.8)
    projector = WaypointCostProjector(waypoints, config)
    cached = projector.project_costs(source_grid)
    direct = _direct_max_cost(waypoints, source_grid, config.waypoint_cost_radius_m)
    assert cached == direct


# -- Real waypoint file: load 159 points via the existing helpers only -------

def test_real_waypoint_file_loads_159_points_via_existing_helpers():
    path = project_path("docs", "full_map_waypoints_1m_numbered.yaml")
    document = load_waypoint_document(path)
    assert document.get("spacing_m") == 1.0
    assert document.get("frame_id") == "map"
    waypoints = named_waypoints_from_document(document, "map")
    assert len(waypoints) == 159

    waypoints_world = {item.name: (item.x, item.y) for item in waypoints}
    xs = [x for x, _ in waypoints_world.values()]
    ys = [y for _, y in waypoints_world.values()]
    margin = 2.0
    resolution = 0.2
    origin_x, origin_y = min(xs) - margin, min(ys) - margin
    width = int(math.ceil((max(xs) - origin_x + margin) / resolution))
    height = int(math.ceil((max(ys) - origin_y + margin) / resolution))
    data = np.zeros((height, width), dtype=np.int16)
    source_grid = MapGrid(width, height, resolution, origin_x, origin_y, 0.0, "map", data)

    projector = WaypointCostProjector(waypoints_world)
    costs = projector.project_costs(source_grid, revision=1)
    assert len(costs) == 159
    assert all(math.isfinite(value) for value in costs.values())  # all-free synthetic grid
