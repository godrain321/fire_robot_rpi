"""Stage 5: gas hazard cost reaches the real waypoint planner and A*.

No planner algorithm is touched. These tests exercise:
  * the pure overlay/merge helpers (gas_planning_grid),
  * the real inno_autonav WaypointCostProjector + WaypointGraphPlanner fed a
    gas-composed grid (finite -> detour, blocked -> excluded, off -> identical),
  * the source/launch wiring contract (A* reads /hazard/final_cost directly;
    the RViz vis topic is never used as planner input).
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

from inno_hazard.gas_planning_grid import gas_overlay_cells, merge_planning_cells

_HAZARD_PKG = Path(__file__).resolve().parents[1]
_AUTONAV = _HAZARD_PKG.parent / "inno_autonav"
_NODE_SRC = (_HAZARD_PKG / "inno_hazard" / "hazard_belief_node.py").read_text()
_MERGE_SRC = (_HAZARD_PKG / "inno_hazard" / "planning_grid_hazard_merge.py").read_text()
_LAUNCH_SRC = (_AUTONAV / "launch" / "autonav_demo.launch.py").read_text()
_ASTAR_SRC = (_AUTONAV / "inno_autonav" / "astar_replanner.py").read_text()

# Let the real planner tests run from a bare checkout too (colcon installs it).
if _AUTONAV.is_dir() and str(_AUTONAV) not in sys.path:
    sys.path.insert(0, str(_AUTONAV))


# --------------------------------------------------------------------------
# gas_overlay_cells: raw /planning_grid encoding of the gas belief
# --------------------------------------------------------------------------
def _overlay(values, observed, safe=1000.0, blocked=3000.0):
    return gas_overlay_cells(
        np.asarray(values, float), np.asarray(observed, bool), safe, blocked
    )


def test_overlay_below_safe_is_zero():
    cells = _overlay([[800.0]], [[True]])
    assert cells[0, 0] == 0


def test_overlay_middle_is_between_1_and_99():
    cells = _overlay([[2000.0]], [[True]])          # ratio 0.5 -> ~50
    assert 1 <= cells[0, 0] <= 99
    assert cells[0, 0] == round(99 * 0.5)


def test_overlay_at_or_above_blocked_is_100():
    cells = _overlay([[3000.0], [5000.0]], [[True], [True]])
    assert cells[0, 0] == 100 and cells[1, 0] == 100


def test_overlay_unobserved_is_zero():
    cells = _overlay([[9999.0]], [[False]])
    assert cells[0, 0] == 0


def test_overlay_rejects_bad_thresholds():
    with pytest.raises(ValueError):
        gas_overlay_cells(np.zeros((1, 1)), np.zeros((1, 1), bool), 3000.0, 1000.0)


# --------------------------------------------------------------------------
# merge_planning_cells: compose overlay onto the real /planning_grid encoding
# --------------------------------------------------------------------------
def test_merge_gas_off_is_byte_identical_to_base():
    base = np.array([[-1, 0, 25, 100], [0, 40, 0, 12]], dtype=np.int16)
    merged = merge_planning_cells(base, np.zeros_like(base), unknown_is_occupied=True)
    np.testing.assert_array_equal(merged, base)


def test_merge_finite_gas_raises_only_where_higher():
    base = np.array([[0, 40, 0]], dtype=np.int16)
    gas = np.array([[20, 10, 60]], dtype=np.int16)
    merged = merge_planning_cells(base, gas, unknown_is_occupied=True)
    np.testing.assert_array_equal(merged, [[20, 40, 60]])


def test_merge_blocked_gas_makes_cell_lethal():
    base = np.array([[0, 5]], dtype=np.int16)
    gas = np.array([[100, 0]], dtype=np.int16)
    merged = merge_planning_cells(base, gas, unknown_is_occupied=True)
    assert merged[0, 0] == 100


def test_merge_never_overwrites_unknown_or_static_lethal():
    base = np.array([[-1, 100, 3]], dtype=np.int16)
    gas = np.array([[99, 50, 0]], dtype=np.int16)
    merged = merge_planning_cells(base, gas, unknown_is_occupied=True)
    assert merged[0, 0] == -1 and merged[0, 1] == 100 and merged[0, 2] == 3


def test_merge_shape_mismatch_raises():
    with pytest.raises(ValueError):
        merge_planning_cells(np.zeros((2, 2), np.int16), np.zeros((2, 3), np.int16),
                             unknown_is_occupied=True)


# --------------------------------------------------------------------------
# The REAL waypoint planner consuming a gas-composed grid (no algo changes)
# --------------------------------------------------------------------------
def _waypoint_stack():
    pytest.importorskip("inno_autonav.waypoint_graph_planner")
    from inno_autonav.grid_utils import MapGrid
    from inno_autonav.waypoint_cost_projector import (
        WaypointCostProjector, WaypointCostProjectorConfig,
    )
    from inno_autonav.waypoint_graph_planner import (
        WaypointGraphPlanner, WaypointGraphPlannerConfig,
    )
    waypoints = {
        f"W{c}{r}": (float(c), float(r)) for r in range(3) for c in range(3)
    }
    projector = WaypointCostProjector(
        waypoints, WaypointCostProjectorConfig(waypoint_cost_radius_m=0.4),
    )
    planner = WaypointGraphPlanner(
        waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.5),
    )

    def map_grid(data):
        return MapGrid(5, 5, 1.0, 0.0, 0.0, 0.0, "map", np.asarray(data, np.int16))

    return waypoints, projector, planner, map_grid


def test_waypoint_plan_unchanged_when_gas_off():
    waypoints, projector, planner, map_grid = _waypoint_stack()
    base = np.zeros((5, 5), np.int16)
    off = merge_planning_cells(base, np.zeros_like(base), unknown_is_occupied=True)
    costs = projector.project_costs(map_grid(off))
    result = planner.plan(costs, "W00", "W22")
    assert result.success
    assert result.waypoint_ids == ("W00", "W11", "W22")   # shortest, as with a bare grid


def test_waypoint_detours_around_finite_gas_cost():
    waypoints, projector, planner, map_grid = _waypoint_stack()
    base = np.zeros((5, 5), np.int16)
    gas = np.zeros((5, 5), np.int16)
    gas[1, 1] = 80  # finite, expensive gas cost on the straight route's mid waypoint
    merged = merge_planning_cells(base, gas, unknown_is_occupied=True)
    costs = projector.project_costs(map_grid(merged))
    assert math.isfinite(costs["W11"]) and costs["W11"] == 80.0
    result = planner.plan(costs, "W00", "W22")
    assert result.success
    assert "W11" not in result.waypoint_ids           # routed around, still traversable


def test_waypoint_excludes_gas_blocked_cell():
    waypoints, projector, planner, map_grid = _waypoint_stack()
    base = np.zeros((5, 5), np.int16)
    gas = np.zeros((5, 5), np.int16)
    gas[1, 1] = 100  # gas at/above blocked threshold
    merged = merge_planning_cells(base, gas, unknown_is_occupied=True)
    costs = projector.project_costs(map_grid(merged))
    assert costs["W11"] == math.inf
    result = planner.plan(costs, "W00", "W22")
    assert result.success
    assert "W11" not in result.waypoint_ids


# --------------------------------------------------------------------------
# Wiring contract
# --------------------------------------------------------------------------
def test_hazard_node_publishes_gas_overlay_occupancygrid():
    assert 'OccupancyGrid, "/hazard/gas_cost_grid"' in _NODE_SRC
    assert "gas_overlay_cells(" in _NODE_SRC


def test_merge_node_composes_planning_grid_not_the_vis_topic():
    assert "/planning_grid" in _MERGE_SRC and "/hazard/gas_cost_grid" in _MERGE_SRC
    assert "/planning_grid_hazard" in _MERGE_SRC
    assert "merge_planning_cells(" in _MERGE_SRC
    assert "final_cost_grid_vis" not in _MERGE_SRC      # never the RViz topic


def test_launch_routes_waypoint_grid_and_gates_merge_on_gas():
    assert "waypoint_planning_grid_topic" in _LAUNCH_SRC
    assert "'/planning_grid_hazard' if '" in _LAUNCH_SRC
    assert "planning_grid_hazard_merge" in _LAUNCH_SRC
    assert "'planning_grid_topic': waypoint_planning_grid_topic" in _LAUNCH_SRC


def test_astar_still_reads_exact_hazard_final_cost_directly():
    # A* integration is pre-existing; Stage 5 must not have altered it.
    assert "Float32MultiArray, self.hazard_final_cost_topic" in _ASTAR_SRC
    assert "self.hazard_grid if self.hazard_belief_enabled" in _ASTAR_SRC
    assert "costs_are_traversal=self.hazard_belief_enabled" in _ASTAR_SRC


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
