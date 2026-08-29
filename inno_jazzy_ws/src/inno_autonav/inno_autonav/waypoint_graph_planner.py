"""Deterministic Dijkstra search over the existing ~1 m reference waypoints,
using only already-projected waypoint costs (Stage 8-2's WaypointCostProjector).

Deliberately does not read ``/planning_grid`` cells at all -- unlike
``reference_waypoint_graph.ReferenceWaypointGraphPlanner`` (a line-for-line port of
``fire_robot/simulator/factory_v5/planner/reference_waypoint_graph.py``), whose
``_edge()``/``_connectors()`` validate every candidate edge with a fresh cell-level
``weighted_astar_search``/``unweighted_a_star`` run against the raw costmap. That
file keeps serving Stage 1/2/4 exactly as before; this module is a separate,
simpler graph search that only ever sees the compressed per-waypoint cost dict.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Collection, Mapping, Sequence


@dataclass(frozen=True)
class WaypointGraphPlannerConfig:
    # Reuses the exact neighbor radius already proven for these same 159
    # waypoints in ReferenceWaypointGraphConfig.neighbor_radius_m (both the ROS
    # and simulation copies default to 1.5 m for ~1 m spacing).
    neighbor_radius_m: float = 1.5
    # Same "1.0 + risk" floor used throughout the project (HazardBeliefConfig
    # .base_cost, weighted_planner.traversal_multiplier) -- without a positive
    # floor, an edge between two completely clear waypoints (raw cost == 0)
    # would cost 0 regardless of length, which breaks shortest-path meaning.
    base_cost: float = 1.0

    def __post_init__(self) -> None:
        for name in ("neighbor_radius_m", "base_cost"):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class WaypointPlanResult:
    success: bool
    waypoint_ids: tuple[str, ...]
    total_cost: float
    status: str


class WaypointGraphPlanner:
    """Build the neighbor-radius edge set once; re-run Dijkstra per cost update."""

    def __init__(
        self, waypoints_world: Mapping[str, tuple[float, float]],
        config: WaypointGraphPlannerConfig | None = None,
    ) -> None:
        if not waypoints_world:
            raise ValueError("waypoints_world must not be empty")
        self.waypoints_world = dict(waypoints_world)
        self.config = config or WaypointGraphPlannerConfig()
        self._edges = self._build_candidate_edges()

    def _build_candidate_edges(self) -> tuple[tuple[str, str, float], ...]:
        limit = self.config.neighbor_radius_m + 1e-12
        ids = list(self.waypoints_world)
        edges = []
        for first in range(len(ids)):
            ax, ay = self.waypoints_world[ids[first]]
            for second in range(first + 1, len(ids)):
                bx, by = self.waypoints_world[ids[second]]
                distance = math.hypot(bx - ax, by - ay)
                if distance <= limit:
                    edges.append((ids[first], ids[second], distance))
        return tuple(edges)

    @property
    def edges(self) -> tuple[tuple[str, str, float], ...]:
        return self._edges

    def plan(
        self, waypoint_costs: Mapping[str, float], start_id: str, goal_id: str,
        *, excluded_edges: Collection[frozenset[str]] = (),
    ) -> WaypointPlanResult:
        if start_id not in self.waypoints_world:
            return WaypointPlanResult(False, (), math.inf, "INVALID_START")
        if goal_id not in self.waypoints_world:
            return WaypointPlanResult(False, (), math.inf, "INVALID_GOAL")
        if start_id not in waypoint_costs or goal_id not in waypoint_costs:
            return WaypointPlanResult(False, (), math.inf, "MISSING_WAYPOINT_COST")
        if start_id == goal_id:
            return WaypointPlanResult(True, (start_id,), 0.0, "ALREADY_AT_GOAL")
        if not math.isfinite(float(waypoint_costs[start_id])):
            return WaypointPlanResult(False, (), math.inf, "START_BLOCKED")
        if not math.isfinite(float(waypoint_costs[goal_id])):
            return WaypointPlanResult(False, (), math.inf, "GOAL_BLOCKED")

        excluded = set(excluded_edges)
        adjacency: dict[str, list[tuple[str, float]]] = {}
        for a_id, b_id, distance in self._edges:
            if frozenset((a_id, b_id)) in excluded:
                continue
            a_cost = waypoint_costs.get(a_id)
            b_cost = waypoint_costs.get(b_id)
            if (
                a_cost is None or b_cost is None
                or not math.isfinite(float(a_cost)) or not math.isfinite(float(b_cost))
            ):
                continue  # missing or blocked -- excluded from traversal entirely
            adjacency.setdefault(a_id, []).append(
                (b_id, distance * (self.config.base_cost + float(b_cost)))
            )
            adjacency.setdefault(b_id, []).append(
                (a_id, distance * (self.config.base_cost + float(a_cost)))
            )

        distances = {start_id: 0.0}
        parent: dict[str, str | None] = {start_id: None}
        visited: set[str] = set()
        frontier: list[tuple[float, str]] = [(0.0, start_id)]
        while frontier:
            current_cost, current_id = heapq.heappop(frontier)
            if current_id in visited:
                continue
            visited.add(current_id)
            if current_id == goal_id:
                break
            for neighbor_id, edge_cost in adjacency.get(current_id, ()):
                if neighbor_id in visited:
                    continue
                new_cost = current_cost + edge_cost
                if new_cost + 1e-12 < distances.get(neighbor_id, math.inf):
                    distances[neighbor_id] = new_cost
                    parent[neighbor_id] = current_id
                    heapq.heappush(frontier, (new_cost, neighbor_id))

        if goal_id not in visited:
            return WaypointPlanResult(False, (), math.inf, "NO_ROUTE")
        path = []
        node: str | None = goal_id
        while node is not None:
            path.append(node)
            node = parent[node]
        path.reverse()
        return WaypointPlanResult(True, tuple(path), distances[goal_id], "PATH_FOUND")


def nearest_safe_waypoint(
    position_world: tuple[float, float], waypoints_world: Mapping[str, tuple[float, float]],
    waypoint_costs: Mapping[str, float],
) -> str | None:
    """Nearest waypoint with a finite cost -- no grid A* is run to find it (spec section 17)."""
    px, py = position_world
    best_id = None
    best_distance = math.inf
    for waypoint_id in sorted(waypoints_world):  # deterministic tie-break by id
        cost = waypoint_costs.get(waypoint_id)
        if cost is None or not math.isfinite(float(cost)):
            continue
        x, y = waypoints_world[waypoint_id]
        distance = math.hypot(x - px, y - py)
        if distance < best_distance:
            best_distance = distance
            best_id = waypoint_id
    return best_id
