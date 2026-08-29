"""Tests for the Stage 8-5 pure WaypointGraphPlanner (spec section 30)."""

import math

import numpy as np
import pytest

from inno_autonav.grid_utils import MapGrid
from inno_autonav.project_paths import project_path
from inno_autonav.waypoint_cost_projector import WaypointCostProjector, WaypointCostProjectorConfig
from inno_autonav.waypoint_graph_planner import (
    WaypointGraphPlanner,
    WaypointGraphPlannerConfig,
    nearest_safe_waypoint,
)
from inno_autonav.waypoint_selection import load_waypoint_document, named_waypoints_from_document


def grid_3x3():
    """A 3x3 waypoint grid at 1 m spacing: W{col}{row}, matching maps/*'s spacing."""
    return {
        f"W{col}{row}": (float(col), float(row))
        for row in range(3) for col in range(3)
    }


def flat_costs(waypoints, value=0.0):
    return {name: value for name in waypoints}


# -- Test 1: equal cost -> shortest (diagonal) route --------------------------

def test_1_equal_cost_takes_the_shortest_route():
    waypoints = grid_3x3()
    planner = WaypointGraphPlanner(waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.5))
    result = planner.plan(flat_costs(waypoints), "W00", "W22")
    assert result.success
    # Diagonal hops (W00->W11->W22) are strictly shorter than any cardinal-only route.
    assert result.waypoint_ids == ("W00", "W11", "W22")
    assert result.total_cost == pytest.approx(2.0 * math.hypot(1, 1))


# -- Test 2: high-cost waypoint is avoided ------------------------------------

def test_2_high_cost_waypoint_is_avoided():
    waypoints = grid_3x3()
    planner = WaypointGraphPlanner(waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.5))
    costs = flat_costs(waypoints)
    costs["W11"] = 50.0  # finite but very expensive -- must be routed around
    result = planner.plan(costs, "W00", "W22")
    assert result.success
    assert "W11" not in result.waypoint_ids


# -- Test 3: blocked waypoint excluded from traversal -------------------------

def test_3_blocked_waypoint_is_excluded():
    waypoints = grid_3x3()
    planner = WaypointGraphPlanner(waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.5))
    costs = flat_costs(waypoints)
    costs["W11"] = math.inf
    result = planner.plan(costs, "W00", "W22")
    assert result.success
    assert "W11" not in result.waypoint_ids


# -- Test 4: no route -----------------------------------------------------------

def test_4_no_route_when_disconnected():
    waypoints = {"A": (0.0, 0.0), "B": (100.0, 100.0)}  # far beyond any radius
    planner = WaypointGraphPlanner(waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.5))
    result = planner.plan(flat_costs(waypoints), "A", "B")
    assert result.success is False
    assert result.status == "NO_ROUTE"


def test_excluded_blocked_edge_uses_available_graph_detour():
    waypoints = {
        "A": (0.0, 0.0), "B": (1.0, 0.0),
        "C": (0.0, 1.0), "D": (1.0, 1.0),
    }
    planner = WaypointGraphPlanner(
        waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.05),
    )
    result = planner.plan(
        flat_costs(waypoints), "A", "B",
        excluded_edges={frozenset(("A", "B"))},
    )
    assert result.success
    assert result.waypoint_ids == ("A", "C", "D", "B")


# -- Test 5: start == goal -----------------------------------------------------

def test_5_start_equals_goal():
    waypoints = grid_3x3()
    planner = WaypointGraphPlanner(waypoints)
    result = planner.plan(flat_costs(waypoints), "W00", "W00")
    assert result == type(result)(True, ("W00",), 0.0, "ALREADY_AT_GOAL")


# -- Test 6: invalid start/goal ids --------------------------------------------

def test_6_invalid_start_and_goal():
    waypoints = grid_3x3()
    planner = WaypointGraphPlanner(waypoints)
    costs = flat_costs(waypoints)
    assert planner.plan(costs, "NOPE", "W00").status == "INVALID_START"
    assert planner.plan(costs, "W00", "NOPE").status == "INVALID_GOAL"


# -- Test 7: missing waypoint cost entries -------------------------------------

def test_7_missing_waypoint_cost():
    waypoints = grid_3x3()
    planner = WaypointGraphPlanner(waypoints)
    costs = flat_costs(waypoints)
    del costs["W22"]
    assert planner.plan(costs, "W00", "W22").status == "MISSING_WAYPOINT_COST"


# -- Test 8: connectivity correctness ------------------------------------------

def test_8_connectivity_matches_neighbor_radius():
    waypoints = grid_3x3()
    planner = WaypointGraphPlanner(waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.5))
    pairs = {frozenset((a, b)) for a, b, _ in planner.edges}
    assert frozenset(("W00", "W10")) in pairs  # cardinal, distance 1.0
    assert frozenset(("W00", "W11")) in pairs  # diagonal, distance 1.414
    assert frozenset(("W00", "W20")) not in pairs  # distance 2.0, beyond radius
    assert frozenset(("W00", "W22")) not in pairs  # distance 2.828, beyond radius


# -- Test 9: deterministic tie-break -------------------------------------------

def test_9_deterministic_result_across_repeated_calls():
    waypoints = grid_3x3()
    # Cardinal-only connectivity creates two equal-length routes around the center.
    planner = WaypointGraphPlanner(waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.05))
    costs = flat_costs(waypoints)
    first = planner.plan(costs, "W00", "W22")
    for _ in range(5):
        assert planner.plan(costs, "W00", "W22") == first


# -- Test 10/8: real 159-waypoint graph build and connectivity -----------------

def test_10_real_159_waypoint_graph_builds_and_connects():
    document = load_waypoint_document(project_path(
        "docs", "full_map_waypoints_1m_numbered.yaml"
    ))
    records = named_waypoints_from_document(document, "map")
    assert len(records) == 159
    waypoints = {item.name: (item.x, item.y) for item in records}
    planner = WaypointGraphPlanner(waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.5))
    assert len(planner.edges) > 0
    costs = flat_costs(waypoints)
    names = list(waypoints)
    result = planner.plan(costs, names[0], names[-1])
    assert result.success, f"real waypoint graph at neighbor_radius_m=1.5 is disconnected: {result.status}"


def test_nearest_safe_waypoint_skips_blocked_ones():
    waypoints = grid_3x3()
    costs = flat_costs(waypoints)
    costs["W11"] = math.inf  # closest to (1.1, 1.1) but blocked
    assert nearest_safe_waypoint((1.1, 1.1), waypoints, costs) != "W11"


# -- Test 11: WaypointCostProjector integration -- hot region forces a detour -

def test_11_projector_integration_hot_region_forces_detour():
    size = 9
    data = np.zeros((size, size), dtype=np.int16)
    data[3:6, 4] = 90  # a "hot" vertical wall at grid column 4, rows 3-5
    grid = MapGrid(size, size, 1.0, 0.0, 0.0, 0.0, "map", data)

    waypoints = {
        f"W{col}{row}": (col + 0.5, row + 0.5) for row in range(size) for col in range(size)
    }
    projector = WaypointCostProjector(waypoints, WaypointCostProjectorConfig(waypoint_cost_radius_m=0.5))
    costs = projector.project_costs(grid)
    planner = WaypointGraphPlanner(waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.5))

    result = planner.plan(costs, "W04", "W64")  # straight through the hot wall
    assert result.success
    # The hot column (col=4) waypoints directly inside the wall rows must be avoided.
    hot = {f"W4{row}" for row in range(3, 6)}
    assert not (hot & set(result.waypoint_ids))
