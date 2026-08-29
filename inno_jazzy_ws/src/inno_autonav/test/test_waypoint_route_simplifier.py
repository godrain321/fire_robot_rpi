"""Tests for Stage 8-6 waypoint route simplification (spec section 31 A-D)."""

import numpy as np

from inno_autonav.grid_utils import MapGrid
from inno_autonav.waypoint_route_simplifier import (
    WaypointRouteSimplifierConfig,
    blocked_waypoint_edges,
    nearest_reachable_waypoint,
    simplify_waypoint_route,
)


def clear_grid(size=10, resolution=1.0, value=0):
    data = np.full((size, size), value, dtype=np.int16)
    return MapGrid(size, size, resolution, 0.0, 0.0, 0.0, "map", data)


CONFIG = WaypointRouteSimplifierConfig()  # matches astar_replanner's own defaults


# -- A: a fully clear, collinear run collapses to its endpoints ---------------

def test_a_straight_collinear_run_collapses_to_endpoints():
    waypoints = {"W1": (0.5, 0.5), "W2": (1.5, 0.5), "W3": (2.5, 0.5), "W4": (3.5, 0.5)}
    result = simplify_waypoint_route(("W1", "W2", "W3", "W4"), waypoints, clear_grid(), CONFIG)
    assert result.success
    assert result.simplified_ids == ("W1", "W4")


# -- B: a genuine corner is preserved when the diagonal shortcut is blocked --

def test_b_corner_is_preserved_when_shortcut_is_blocked():
    waypoints = {"W1": (0.5, 0.5), "W2": (2.5, 0.5), "W3": (2.5, 2.5)}
    grid = clear_grid()
    grid.data[1, 1] = 100  # sits exactly on the W1->W3 diagonal supercover, not on W1-W2-W3
    result = simplify_waypoint_route(("W1", "W2", "W3"), waypoints, grid, CONFIG)
    assert result.success
    assert result.simplified_ids == ("W1", "W2", "W3")  # corner kept, shortcut rejected


# -- C: obstacle-on-shortcut rejection with a longer, non-adjacent candidate --

def test_c_obstacle_on_long_shortcut_keeps_an_intermediate_waypoint():
    waypoints = {
        "W1": (0.5, 0.5), "W2": (2.5, 0.5), "W3": (2.5, 2.5), "W4": (4.5, 2.5),
    }
    grid = clear_grid()
    grid.data[1, 1] = 100  # blocks the full W1->W4-style long diagonal shortcuts
    result = simplify_waypoint_route(("W1", "W2", "W3", "W4"), waypoints, grid, CONFIG)
    assert result.success
    assert "W2" in result.simplified_ids or "W3" in result.simplified_ids
    assert result.simplified_ids[0] == "W1"
    assert result.simplified_ids[-1] == "W4"


# -- D: a shortcut through a materially higher-risk cell is rejected ---------

def test_d_high_risk_shortcut_is_rejected():
    waypoints = {"W1": (0.5, 0.5), "W2": (2.5, 0.5), "W3": (2.5, 2.5)}
    grid = clear_grid()
    grid.data[1, 1] = 95  # not blocked (<100), but a big finite risk increase
    result = simplify_waypoint_route(("W1", "W2", "W3"), waypoints, grid, CONFIG)
    assert result.success
    assert result.simplified_ids == ("W1", "W2", "W3")


# -- error handling ------------------------------------------------------------

def test_empty_route_fails_cleanly():
    result = simplify_waypoint_route((), {}, clear_grid(), CONFIG)
    assert result.success is False


def test_unknown_waypoint_id_fails_cleanly():
    waypoints = {"W1": (0.5, 0.5)}
    result = simplify_waypoint_route(("W1", "GHOST"), waypoints, clear_grid(), CONFIG)
    assert result.success is False
    assert "GHOST" in result.detail


def test_two_safe_waypoints_with_blocked_edge_are_rejected():
    waypoints = {"W18": (0.5, 1.5), "W43": (3.5, 1.5)}
    grid = clear_grid()
    grid.data[1, 2] = 100
    result = simplify_waypoint_route(("W18", "W43"), waypoints, grid, CONFIG)
    assert not result.success
    assert result.detail == "blocked_edge:W18->W43"


def test_blocked_adjacent_edge_in_long_route_is_rejected():
    waypoints = {
        "W1": (0.5, 0.5), "W2": (1.5, 0.5), "W3": (2.5, 0.5),
    }
    grid = clear_grid()
    grid.data[0, 2] = 100
    result = simplify_waypoint_route(("W1", "W2", "W3"), waypoints, grid, CONFIG)
    assert not result.success
    assert result.detail == "blocked_edge:W2->W3"


def test_diagonal_corner_touching_blocked_cell_is_rejected():
    waypoints = {"A": (0.5, 0.5), "B": (1.5, 1.5)}
    grid = clear_grid()
    grid.data[0, 1] = 100
    result = simplify_waypoint_route(("A", "B"), waypoints, grid, CONFIG)
    assert not result.success
    assert result.detail == "blocked_edge:A->B"


def test_grid_blocked_edge_is_reported_for_graph_exclusion():
    waypoints = {"A": (0.5, 0.5), "B": (2.5, 0.5), "C": (0.5, 2.5)}
    grid = clear_grid()
    grid.data[0, 1] = 100
    edges = (("A", "B", 2.0), ("A", "C", 2.0))
    assert blocked_waypoint_edges(edges, waypoints, grid, True) == {
        frozenset(("A", "B")),
    }


def test_unknown_cell_blocks_edge_when_unknown_is_occupied():
    waypoints = {"A": (0.5, 0.5), "B": (2.5, 0.5)}
    grid = clear_grid()
    grid.data[0, 1] = -1
    edges = (("A", "B", 2.0),)
    assert blocked_waypoint_edges(edges, waypoints, grid, True) == {
        frozenset(("A", "B")),
    }


def test_nearest_reachable_waypoint_skips_blocked_connector():
    waypoints = {"nearest": (2.5, 0.5), "reachable": (0.5, 2.5)}
    grid = clear_grid(size=5)
    grid.data[0, 1] = 100
    costs = {waypoint_id: 0.0 for waypoint_id in waypoints}
    assert nearest_reachable_waypoint(
        (0.5, 0.5), waypoints, costs, grid, True,
    ) == "reachable"


def test_nearest_reachable_waypoint_fails_when_all_connectors_are_blocked():
    waypoints = {"east": (2.5, 0.5), "north": (0.5, 2.5)}
    grid = clear_grid(size=5)
    grid.data[0, 1] = 100
    grid.data[1, 0] = 100
    costs = {waypoint_id: 0.0 for waypoint_id in waypoints}
    assert nearest_reachable_waypoint(
        (0.5, 0.5), waypoints, costs, grid, True,
    ) is None
