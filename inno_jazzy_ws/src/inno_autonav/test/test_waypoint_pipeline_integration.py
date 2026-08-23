"""End-to-end pure-core integration for Stage 8-5~8-8's planning half of the
pipeline: WaypointCostProjector -> WaypointGraphPlanner -> simplify_waypoint_route.
No rclpy involved (that wiring is already covered by
test_waypoint_planner_node_contract.py) -- this is the "does the whole thing
actually produce a sane route" check, plus the real-159-waypoint dry-run with
timings requested in the Stage 8-5~8-8 report. All numbers here are
synthetic/dev-PC, never claimed as Raspberry Pi measurements.
"""

import time

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
from inno_autonav.waypoint_route_simplifier import (
    WaypointRouteSimplifierConfig,
    simplify_waypoint_route,
)
from inno_autonav.waypoint_selection import load_waypoint_document, named_waypoints_from_document


def test_synthetic_fire_forces_a_detour_through_the_full_pipeline():
    #   ●──●──●──●      row 0 (y=1)
    #   │  🔥  │        fire blocks the direct col=1..2 crossing
    #   ●──●──●──●      row 1 (y=0)
    waypoints = {
        f"W{col}{row}": (float(col), float(row))
        for row in range(2) for col in range(4)
    }
    size = 5
    grid = MapGrid(size, size, 1.0, 0.0, 0.0, 0.0, "map", np.zeros((size, size), dtype=np.int16))
    grid.data[0, 1] = 95  # between row0/row1 under columns 1-2: forces a detour
    grid.data[0, 2] = 95

    projector = WaypointCostProjector(waypoints, WaypointCostProjectorConfig(waypoint_cost_radius_m=0.5))
    costs = projector.project_costs(grid)
    planner = WaypointGraphPlanner(waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.5))
    result = planner.plan(costs, "W00", "W31")
    assert result.success
    # W10/W20 sit directly under the fire (cost 95, ~96x the base traversal
    # cost) -- Dijkstra must detour around them via the clear row instead.
    assert "W10" not in result.waypoint_ids
    assert "W20" not in result.waypoint_ids

    simplification = simplify_waypoint_route(result.waypoint_ids, waypoints, grid, WaypointRouteSimplifierConfig())
    assert simplification.success
    assert simplification.simplified_ids[0] == "W00"
    assert simplification.simplified_ids[-1] == "W31"


@pytest.mark.parametrize("resolution", [0.2])
def test_real_159_waypoint_dry_run_reports_pipeline_metrics(resolution):
    document = load_waypoint_document(project_path("maps", "waypoint_queue_latest.yaml"))
    records = named_waypoints_from_document(document, "map")
    assert document.get("spacing_m") == 1.0
    assert document.get("frame_id") == "map"
    assert len(records) == 159
    waypoints = {item.name: (item.x, item.y) for item in records}

    xs = [x for x, _ in waypoints.values()]
    ys = [y for _, y in waypoints.values()]
    margin = 2.0
    origin_x, origin_y = min(xs) - margin, min(ys) - margin
    width = int((max(xs) - origin_x + margin) / resolution) + 1
    height = int((max(ys) - origin_y + margin) / resolution) + 1
    grid = MapGrid(
        width, height, resolution, origin_x, origin_y, 0.0, "map",
        np.zeros((height, width), dtype=np.int16),
    )

    projector = WaypointCostProjector(waypoints, WaypointCostProjectorConfig(waypoint_cost_radius_m=0.8))
    names = list(waypoints)
    start_id, goal_id = names[0], names[-1]

    t0 = time.perf_counter()
    costs = projector.project_costs(grid, revision=1)
    t1 = time.perf_counter()
    projection_s = t1 - t0

    t0 = time.perf_counter()
    graph_planner = WaypointGraphPlanner(waypoints, WaypointGraphPlannerConfig(neighbor_radius_m=1.5))
    t1 = time.perf_counter()
    graph_build_s = t1 - t0

    t0 = time.perf_counter()
    plan_result = graph_planner.plan(costs, start_id, goal_id)
    t1 = time.perf_counter()
    plan_s = t1 - t0
    assert plan_result.success, f"synthetic/dev-PC dry-run: real 159-waypoint graph disconnected: {plan_result.status}"

    t0 = time.perf_counter()
    simplification = simplify_waypoint_route(
        plan_result.waypoint_ids, waypoints, grid, WaypointRouteSimplifierConfig(),
    )
    t1 = time.perf_counter()
    simplify_s = t1 - t0
    assert simplification.success

    total_s = projection_s + graph_build_s + plan_s + simplify_s
    print(
        "\n[synthetic/dev-PC dry-run] "
        f"waypoints=159 resolution={resolution}m grid={width}x{height} "
        f"neighbor_radius=1.5m\n"
        f"  raw route waypoints: {len(plan_result.waypoint_ids)}\n"
        f"  simplified route waypoints: {len(simplification.simplified_ids)}\n"
        f"  total graph cost: {plan_result.total_cost:.3f}\n"
        f"  expanded graph edges: {len(graph_planner.edges)}\n"
        f"  cost projection time: {projection_s * 1000:.3f} ms\n"
        f"  graph build time: {graph_build_s * 1000:.3f} ms\n"
        f"  graph plan time: {plan_s * 1000:.3f} ms\n"
        f"  simplification time: {simplify_s * 1000:.3f} ms\n"
        f"  total pipeline time: {total_s * 1000:.3f} ms"
    )
    assert len(simplification.simplified_ids) <= len(plan_result.waypoint_ids)
