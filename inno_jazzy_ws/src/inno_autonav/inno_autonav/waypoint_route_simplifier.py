"""Reduce a raw waypoint-graph route to its direction-change corners using the
existing, already-tested Stage 1 ``SafePathSimplifier`` machinery.

Requirement A/B split (see the Stage 8-5~8-8 report): ``waypoint_graph_planner.py``
never reads raw ``/planning_grid`` cells -- it only sees compressed waypoint costs.
This module is exactly the opposite by design: it legitimately *does* read the raw
grid, because collapsing ``W1-W2-W3`` into ``W1-W3`` is only safe if the straight
``W1->W3`` segment is itself collision/corner-cutting/risk-increase safe, which
requires the real cell data -- the same reasoning ``safe_path_simplifier.py``
already encodes. Nothing here reimplements that reasoning; it only converts a
waypoint-id route to grid cells, calls ``simplify_path_safely()`` unmodified, and
maps the result back to waypoint ids.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

from .grid_utils import MapGrid, world_to_grid
from .safe_path_simplifier import simplify_path_safely
from .weighted_planner import Cell


@dataclass(frozen=True)
class WaypointRouteSimplifierConfig:
    # Mirrors astar_replanner's own simplification parameters verbatim (same
    # defaults as autonav_params.yaml's astar_replanner: block) -- no new
    # safety convention invented.
    unknown_is_occupied: bool = True
    thermal_cost_weight: float = 24.0
    thermal_cost_power: float = 1.5
    fixed_co_ppm: float = 0.0
    co_safe_ppm: float = 0.0
    co_blocked_ppm: float = 1600.0
    co_cost_weight: float = 8.0
    co_cost_power: float = 2.0
    maximum_risk_ratio: float = 1.0
    risk_absolute_tolerance: float = 0.0


@dataclass(frozen=True)
class WaypointRouteSimplificationResult:
    success: bool
    original_ids: tuple[str, ...]
    simplified_ids: tuple[str, ...]
    detail: str | None = None


def simplify_waypoint_route(
    route_ids: Sequence[str], waypoints_world: Mapping[str, tuple[float, float]],
    grid: MapGrid, config: WaypointRouteSimplifierConfig | None = None,
) -> WaypointRouteSimplificationResult:
    config = config or WaypointRouteSimplifierConfig()
    ids = tuple(route_ids)
    if not ids:
        return WaypointRouteSimplificationResult(False, ids, (), "empty_route")
    for waypoint_id in ids:
        if waypoint_id not in waypoints_world:
            return WaypointRouteSimplificationResult(
                False, ids, (), f"unknown_waypoint:{waypoint_id}",
            )

    cells: list[Cell] = []
    cell_to_id: dict[Cell, str] = {}
    for waypoint_id in ids:
        x, y = waypoints_world[waypoint_id]
        cell = world_to_grid(x, y, grid)
        cells.append(cell)
        cell_to_id.setdefault(cell, waypoint_id)

    result = simplify_path_safely(
        cells, grid.data,
        unknown_is_occupied=config.unknown_is_occupied,
        thermal_cost_weight=config.thermal_cost_weight,
        thermal_cost_power=config.thermal_cost_power,
        fixed_co_ppm=config.fixed_co_ppm,
        co_safe_ppm=config.co_safe_ppm,
        co_blocked_ppm=config.co_blocked_ppm,
        co_cost_weight=config.co_cost_weight,
        co_cost_power=config.co_cost_power,
        maximum_risk_ratio=config.maximum_risk_ratio,
        risk_absolute_tolerance=config.risk_absolute_tolerance,
        costs_are_traversal=False,  # raw /planning_grid encoding, same as Stage 8-2
    )
    if not result.safe or not result.path:
        return WaypointRouteSimplificationResult(False, ids, (), "route_unsafe_or_empty")

    simplified_ids = tuple(cell_to_id[cell] for cell in result.path)
    return WaypointRouteSimplificationResult(True, ids, simplified_ids, None)
