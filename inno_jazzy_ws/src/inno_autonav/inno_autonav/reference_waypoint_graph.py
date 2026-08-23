"""ROS-independent planning over map-frame reference waypoints."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Sequence

import numpy as np

from .weighted_planner import (
    Cell,
    WeightedPathResult,
    cell_is_blocked,
    traversal_multiplier,
    weighted_a_star_with_escape,
    weighted_astar_search,
)


@dataclass(frozen=True)
class ReferenceWaypoint:
    waypoint_id: str
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class PlanningGridGeometry:
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0
    frame_id: str = "map"

    def __post_init__(self) -> None:
        values = (
            self.resolution, self.origin_x, self.origin_y, self.origin_yaw,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("planning grid geometry must be finite")
        if self.resolution <= 0.0:
            raise ValueError("planning grid resolution must be positive")
        if not self.frame_id:
            raise ValueError("planning grid frame_id must not be empty")

    def world_to_grid(self, x: float, y: float) -> Cell:
        dx, dy = x - self.origin_x, y - self.origin_y
        cosine, sine = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        return (
            int(math.floor(local_x / self.resolution)),
            int(math.floor(local_y / self.resolution)),
        )


@dataclass(frozen=True)
class ReferenceWaypointGraphConfig:
    enabled: bool = True
    neighbor_radius_m: float = 1.5
    connector_search_radius_m: float = 3.0
    connector_candidate_count: int = 8
    fallback_to_cell_astar: bool = True
    waypoint_cost_radius_m: float = 0.10
    waypoint_risk_weight: float = 1.0

    def __post_init__(self) -> None:
        for name in ("enabled", "fallback_to_cell_astar"):
            if not isinstance(getattr(self, name), bool):
                raise TypeError(f"{name} must be bool")
        for name in (
            "neighbor_radius_m", "connector_search_radius_m",
            "waypoint_cost_radius_m",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not math.isfinite(float(value))
                or float(value) <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if (
            isinstance(self.waypoint_risk_weight, bool)
            or not math.isfinite(float(self.waypoint_risk_weight))
            or self.waypoint_risk_weight < 0.0
        ):
            raise ValueError(
                "waypoint_risk_weight must be finite and non-negative"
            )
        if (
            isinstance(self.connector_candidate_count, bool)
            or not isinstance(self.connector_candidate_count, int)
            or self.connector_candidate_count < 1
        ):
            raise ValueError(
                "connector_candidate_count must be a positive integer"
            )


def _append_path(output: list[Cell], path: Sequence[Cell]) -> None:
    for x, y in path:
        cell = int(x), int(y)
        if not output or output[-1] != cell:
            output.append(cell)


class ReferenceWaypointGraphPlanner:
    """Build distance candidates once and validate them on every costmap."""

    def __init__(
        self,
        waypoints: Sequence[ReferenceWaypoint],
        config: ReferenceWaypointGraphConfig | None = None,
    ) -> None:
        self.config = config or ReferenceWaypointGraphConfig()
        self.waypoints = tuple(waypoints)
        if self.config.enabled and len(self.waypoints) < 2:
            raise ValueError(
                "reference waypoint graph requires at least two nodes"
            )
        ids = [waypoint.waypoint_id.casefold() for waypoint in self.waypoints]
        if len(set(ids)) != len(ids):
            raise ValueError("reference waypoint IDs must be unique")
        self.candidate_edges = self._build_candidate_edges()

    def _build_candidate_edges(self):
        limit = self.config.neighbor_radius_m + 1e-12
        edges = []
        for first, waypoint_a in enumerate(self.waypoints):
            for second in range(first + 1, len(self.waypoints)):
                waypoint_b = self.waypoints[second]
                distance = math.hypot(
                    waypoint_b.x - waypoint_a.x,
                    waypoint_b.y - waypoint_a.y,
                )
                if distance <= limit:
                    edges.append((first, second, distance))
        return tuple(edges)

    @staticmethod
    def _cell_search(
        costs, start, goal, unknown_is_occupied, costs_are_traversal=False,
    ):
        return weighted_astar_search(
            costs, start, goal,
            unknown_is_occupied=unknown_is_occupied,
            allow_diagonal=True,
            thermal_cost_weight=0.0,
            thermal_cost_power=1.0,
            costs_are_traversal=costs_are_traversal,
            use_traversal_cost=False,
        )

    def _waypoint_cost(
        self, traversal_costs, waypoint_grid, geometry,
    ) -> float:
        radius = int(math.ceil(
            self.config.waypoint_cost_radius_m / geometry.resolution
        ))
        center_col, center_row = waypoint_grid
        values = []
        for row in range(center_row - radius, center_row + radius + 1):
            for col in range(center_col - radius, center_col + radius + 1):
                if (
                    math.hypot(col - center_col, row - center_row)
                    * geometry.resolution
                    > self.config.waypoint_cost_radius_m + 1e-12
                ):
                    continue
                if not (
                    0 <= row < traversal_costs.shape[0]
                    and 0 <= col < traversal_costs.shape[1]
                    and math.isfinite(float(traversal_costs[row, col]))
                    and float(traversal_costs[row, col]) > 0.0
                ):
                    return math.inf
                values.append(float(traversal_costs[row, col]))
        return max(values) if values else math.inf

    def _connectors(
        self, costs, waypoint_grids, endpoint, geometry,
        unknown_is_occupied, costs_are_traversal=False, *, reverse=False,
    ):
        candidates = []
        for index, waypoint in enumerate(self.waypoints):
            anchor = waypoint_grids[index]
            distance = math.hypot(
                anchor[0] - endpoint[0], anchor[1] - endpoint[1]
            ) * geometry.resolution
            if distance <= self.config.connector_search_radius_m + 1e-12:
                candidates.append((distance, waypoint.waypoint_id, index))
        output = {}
        for _, _, index in sorted(candidates)[
            :self.config.connector_candidate_count
        ]:
            anchor = waypoint_grids[index]
            result = self._cell_search(
                costs,
                anchor if reverse else endpoint,
                endpoint if reverse else anchor,
                unknown_is_occupied,
                costs_are_traversal,
            )
            if result.path:
                output[index] = result
        return output

    @staticmethod
    def _stage1(
        costs, start, goal, static_obstacles, planner_options, reason,
    ) -> WeightedPathResult:
        result = weighted_a_star_with_escape(
            costs, start, goal, static_obstacles, **planner_options
        )
        return WeightedPathResult(
            result.path, result.total_cost, result.expanded_nodes,
            result.escape_path, result.replan_start,
            f"{reason}; cell A* fallback: {result.reason}", (), False,
        )

    def _fallback(
        self, costs, start, goal, static_obstacles, planner_options, reason,
    ) -> WeightedPathResult:
        if not self.config.fallback_to_cell_astar:
            return WeightedPathResult((), math.inf, 0, reason=reason)
        return self._stage1(
            costs, start, goal, static_obstacles, planner_options, reason
        )

    def plan(
        self,
        cost_map: np.ndarray,
        start: Cell,
        goal: Cell,
        geometry: PlanningGridGeometry,
        static_obstacle_map: np.ndarray,
        **planner_options,
    ) -> WeightedPathResult:
        costs = np.asarray(cost_map, dtype=float)
        static = np.asarray(static_obstacle_map, dtype=bool)
        if costs.ndim != 2:
            raise ValueError("planning data must be a two-dimensional array")
        if static.shape != costs.shape:
            raise ValueError("static obstacle geometry differs from planning data")
        if geometry.frame_id != planner_options.pop(
            "waypoint_frame_id", geometry.frame_id
        ):
            return WeightedPathResult(
                (), math.inf, 0, reason="waypoint and planning grid frames differ"
            )
        start = int(start[0]), int(start[1])
        goal = int(goal[0]), int(goal[1])
        if start == goal:
            return WeightedPathResult(
                (start,), 0.0, 0, reason="ALREADY_AT_GOAL"
            )
        if not self.config.enabled:
            return self._stage1(
                costs, start, goal, static, planner_options,
                "reference waypoint graph disabled",
            )
        unknown_is_occupied = bool(
            planner_options.get("unknown_is_occupied", True)
        )
        costs_are_traversal = bool(
            planner_options.get("costs_are_traversal", False)
        )
        if cell_is_blocked(
            costs, start, unknown_is_occupied, costs_are_traversal
        ):
            return self._stage1(
                costs, start, goal, static, planner_options,
                "start requires escape before reference graph",
            )
        if cell_is_blocked(
            costs, goal, unknown_is_occupied, costs_are_traversal
        ):
            return self._fallback(
                costs, start, goal, static, planner_options,
                "goal is blocked for reference graph",
            )

        waypoint_grids = tuple(
            geometry.world_to_grid(waypoint.x, waypoint.y)
            for waypoint in self.waypoints
        )
        if len(set(waypoint_grids)) != len(waypoint_grids):
            return self._fallback(
                costs, start, goal, static, planner_options,
                "reference waypoints occupy duplicate map cells",
            )
        start_links = self._connectors(
            costs, waypoint_grids, start, geometry,
            unknown_is_occupied,
            costs_are_traversal,
        )
        if not start_links:
            return self._fallback(
                costs, start, goal, static, planner_options,
                "start reference connector unavailable",
            )
        goal_links = self._connectors(
            costs, waypoint_grids, goal, geometry,
            unknown_is_occupied, costs_are_traversal, reverse=True,
        )
        if not goal_links:
            return self._fallback(
                costs, start, goal, static, planner_options,
                "goal reference connector unavailable",
            )

        if costs_are_traversal:
            traversal_costs = costs.copy()
        else:
            traversal_costs = np.full(costs.shape, math.inf, dtype=float)
            for row, col in np.ndindex(costs.shape):
                if not cell_is_blocked(
                    costs, (col, row), unknown_is_occupied
                ):
                    traversal_costs[row, col] = traversal_multiplier(
                        float(costs[row, col]),
                        planner_options.get("thermal_cost_weight", 24.0),
                        planner_options.get("thermal_cost_power", 1.5),
                        planner_options.get("fixed_co_ppm", 0.0),
                        planner_options.get("co_safe_ppm", 0.0),
                        planner_options.get("co_blocked_ppm", 1600.0),
                        planner_options.get("co_cost_weight", 8.0),
                        planner_options.get("co_cost_power", 2.0),
                    )
        waypoint_costs = tuple(
            self._waypoint_cost(traversal_costs, cell, geometry)
            for cell in waypoint_grids
        )
        finite_costs = traversal_costs[np.isfinite(traversal_costs)]
        if finite_costs.size == 0:
            return self._fallback(
                costs, start, goal, static, planner_options,
                "costmap has no finite traversal cost",
            )
        baseline_cost = float(finite_costs.min())

        adjacency = {index: [] for index in range(len(self.waypoints))}
        edge_paths = {}
        edge_costs = {}
        for first, second, distance in self.candidate_edges:
            if not (
                math.isfinite(waypoint_costs[first])
                and math.isfinite(waypoint_costs[second])
            ):
                continue
            route = self._cell_search(
                costs, waypoint_grids[first], waypoint_grids[second],
                unknown_is_occupied,
                costs_are_traversal,
            )
            if not route.path:
                continue
            endpoint_risk = max(
                0.0,
                0.5 * (waypoint_costs[first] + waypoint_costs[second])
                - baseline_cost,
            )
            graph_cost = distance * (
                1.0 + self.config.waypoint_risk_weight * endpoint_risk
            )
            adjacency[first].append((second, graph_cost))
            adjacency[second].append((first, graph_cost))
            edge_paths[first, second] = route.path
            edge_paths[second, first] = tuple(reversed(route.path))
            edge_costs[first, second] = graph_cost
            edge_costs[second, first] = graph_cost

        frontier = []
        distance_so_far = {}
        parent = {}
        expanded = 0
        for anchor, connector in start_links.items():
            connector_cost = float(connector.total_cost)
            distance_so_far[anchor] = connector_cost
            parent[anchor] = None
            heapq.heappush(frontier, (connector_cost, anchor))
        best_goal = None
        best_total = math.inf
        while frontier:
            current_cost, current = heapq.heappop(frontier)
            if current_cost > distance_so_far[current] + 1e-12:
                continue
            expanded += 1
            if current in goal_links:
                total = current_cost + goal_links[current].total_cost
                if total < best_total:
                    best_total = total
                    best_goal = current
            if current_cost >= best_total:
                continue
            for neighbor, edge_cost in adjacency[current]:
                new_cost = current_cost + edge_cost
                if new_cost + 1e-12 < distance_so_far.get(
                    neighbor, math.inf
                ):
                    distance_so_far[neighbor] = new_cost
                    parent[neighbor] = current
                    heapq.heappush(frontier, (new_cost, neighbor))
        if best_goal is None:
            return self._fallback(
                costs, start, goal, static, planner_options,
                "no safe reference graph route",
            )

        anchors = []
        current = best_goal
        while current is not None:
            anchors.append(current)
            current = parent[current]
        anchors.reverse()
        path = []
        _append_path(path, start_links[anchors[0]].path)
        for first, second in zip(anchors, anchors[1:]):
            _append_path(path, edge_paths[first, second])
        _append_path(path, goal_links[anchors[-1]].path)
        total_cost = (
            start_links[anchors[0]].total_cost
            + sum(
                edge_costs[first, second]
                for first, second in zip(anchors, anchors[1:])
            )
            + goal_links[anchors[-1]].total_cost
        )
        return WeightedPathResult(
            tuple(path), float(total_cost), expanded,
            reason="reference waypoint graph path found",
            reference_waypoint_ids=tuple(
                self.waypoints[index].waypoint_id for index in anchors
            ),
            used_reference_graph=True,
        )
