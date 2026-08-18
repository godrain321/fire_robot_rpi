"""Hardware-independent weighted A* for occupancy and thermal costs."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Sequence

import numpy as np


Cell = tuple[int, int]


@dataclass(frozen=True)
class WeightedPathResult:
    path: tuple[Cell, ...]
    total_cost: float
    expanded_nodes: int


def thermal_readiness_state(
    *,
    require_grid: bool,
    require_active: bool,
    grid_available: bool,
    geometry_matches: bool,
    status: str,
    age_sec: float | None,
    timeout_sec: float,
) -> str | None:
    """Return a planner fail-safe state, or None when thermal input is usable."""
    if timeout_sec < 0.0 or not math.isfinite(timeout_sec):
        raise ValueError("thermal timeout must be finite and non-negative")
    if not require_grid and not require_active:
        return None
    if not geometry_matches:
        return "THERMAL_GRID_MISMATCH"
    if require_grid and not grid_available:
        return "WAITING_FOR_THERMAL_GRID"
    if require_grid and age_sec is not None and age_sec > timeout_sec:
        return "THERMAL_GRID_STALE"
    if require_active and status != "ACTIVE":
        return "WAITING_FOR_THERMAL_ACTIVE"
    return None


def combine_cost_grids(
    static_data: np.ndarray,
    dynamic_data: np.ndarray | None,
    thermal_data: np.ndarray | None,
    *,
    unknown_is_occupied: bool,
) -> np.ndarray:
    """Compose static occupancy, dynamic lethal cells and thermal cost 0..100."""
    static = np.asarray(static_data, dtype=np.int16)
    if static.ndim != 2:
        raise ValueError("static grid must be a two-dimensional array")
    combined = static.copy()
    if dynamic_data is not None:
        dynamic = np.asarray(dynamic_data, dtype=np.int16)
        if dynamic.shape != static.shape:
            raise ValueError("dynamic grid geometry differs from static grid")
        combined[dynamic >= 100] = 100
    if thermal_data is not None:
        thermal = np.asarray(thermal_data, dtype=np.int16)
        if thermal.shape != static.shape:
            raise ValueError("thermal grid geometry differs from static grid")
        if np.any((thermal < 0) | (thermal > 100)):
            raise ValueError("thermal grid values must be in [0, 100]")
        eligible = combined < 100
        if unknown_is_occupied:
            eligible &= combined >= 0
        combined[eligible] = np.maximum(combined[eligible], thermal[eligible])
    return combined.astype(np.int8)


def traversal_multiplier(
    cell_value: float,
    thermal_cost_weight: float,
    thermal_cost_power: float,
    fixed_co_ppm: float = 0.0,
    co_safe_ppm: float = 0.0,
    co_blocked_ppm: float = 1600.0,
    co_cost_weight: float = 8.0,
    co_cost_power: float = 2.0,
) -> float:
    """Apply the factory_v5 temperature/CO traversal-cost equation.

    ``cell_value`` is the linearly encoded temperature ratio (0..99) emitted
    by ``thermal_cost_layer``.  The real robot currently has no CO sensor in
    this pipeline, so ``fixed_co_ppm`` defaults to the explicit 0 ppm requested
    for parity testing.
    """
    if thermal_cost_weight < 0.0 or not math.isfinite(thermal_cost_weight):
        raise ValueError("thermal_cost_weight must be finite and non-negative")
    if thermal_cost_power <= 0.0 or not math.isfinite(thermal_cost_power):
        raise ValueError("thermal_cost_power must be finite and positive")
    co_values = (
        fixed_co_ppm, co_safe_ppm, co_blocked_ppm,
        co_cost_weight, co_cost_power,
    )
    if not all(math.isfinite(value) for value in co_values):
        raise ValueError("CO cost inputs must be finite")
    if fixed_co_ppm < 0.0:
        raise ValueError("fixed_co_ppm must be non-negative")
    if co_blocked_ppm <= co_safe_ppm:
        raise ValueError("co_blocked_ppm must exceed co_safe_ppm")
    if co_cost_weight < 0.0:
        raise ValueError("co_cost_weight must be non-negative")
    if co_cost_power <= 0.0:
        raise ValueError("co_cost_power must be positive")
    value = float(cell_value)
    if not math.isfinite(value):
        return math.inf
    if value >= 100.0 or fixed_co_ppm >= co_blocked_ppm:
        return math.inf
    temperature_normalized = min(99.0, max(0.0, value)) / 99.0
    co_normalized = min(1.0, max(
        0.0,
        (fixed_co_ppm - co_safe_ppm) / (co_blocked_ppm - co_safe_ppm),
    ))
    return (
        1.0
        + thermal_cost_weight * temperature_normalized ** thermal_cost_power
        + co_cost_weight * co_normalized ** co_cost_power
    )


def cell_is_blocked(
    data: np.ndarray, cell: Cell, unknown_is_occupied: bool = True
) -> bool:
    x, y = cell
    if not (0 <= y < data.shape[0] and 0 <= x < data.shape[1]):
        return True
    value = float(data[y, x])
    return (
        not math.isfinite(value)
        or value >= 100.0
        or (value < 0.0 and unknown_is_occupied)
    )


def step_cost(
    data: np.ndarray,
    start: Cell,
    end: Cell,
    thermal_cost_weight: float,
    thermal_cost_power: float,
    fixed_co_ppm: float = 0.0,
    co_safe_ppm: float = 0.0,
    co_blocked_ppm: float = 1600.0,
    co_cost_weight: float = 8.0,
    co_cost_power: float = 2.0,
) -> float:
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    return distance * traversal_multiplier(
        float(data[end[1], end[0]]), thermal_cost_weight, thermal_cost_power,
        fixed_co_ppm, co_safe_ppm, co_blocked_ppm,
        co_cost_weight, co_cost_power,
    )


def path_cost(
    path: Sequence[Cell],
    data: np.ndarray,
    thermal_cost_weight: float = 24.0,
    thermal_cost_power: float = 1.5,
    unknown_is_occupied: bool = True,
    fixed_co_ppm: float = 0.0,
    co_safe_ppm: float = 0.0,
    co_blocked_ppm: float = 1600.0,
    co_cost_weight: float = 8.0,
    co_cost_power: float = 2.0,
) -> float:
    cells = tuple((int(x), int(y)) for x, y in path)
    if not cells:
        return math.inf
    if any(cell_is_blocked(data, cell, unknown_is_occupied) for cell in cells):
        return math.inf
    return sum(
        step_cost(
            data, first, second, thermal_cost_weight, thermal_cost_power,
            fixed_co_ppm, co_safe_ppm, co_blocked_ppm,
            co_cost_weight, co_cost_power,
        )
        for first, second in zip(cells, cells[1:])
    )


def weighted_astar_search(
    data: np.ndarray,
    start: Cell,
    goal: Cell,
    *,
    unknown_is_occupied: bool = True,
    allow_diagonal: bool = True,
    thermal_cost_weight: float = 24.0,
    thermal_cost_power: float = 1.5,
    fixed_co_ppm: float = 0.0,
    co_safe_ppm: float = 0.0,
    co_blocked_ppm: float = 1600.0,
    co_cost_weight: float = 8.0,
    co_cost_power: float = 2.0,
) -> WeightedPathResult:
    """Run 8-connected A* without diagonal corner cutting."""
    costs = np.asarray(data, dtype=float)
    if costs.ndim != 2:
        raise ValueError("planning data must be a two-dimensional array")
    traversal_multiplier(
        0.0, thermal_cost_weight, thermal_cost_power,
        fixed_co_ppm, co_safe_ppm, co_blocked_ppm,
        co_cost_weight, co_cost_power,
    )
    start = int(start[0]), int(start[1])
    goal = int(goal[0]), int(goal[1])
    if cell_is_blocked(costs, start, unknown_is_occupied) or cell_is_blocked(
        costs, goal, unknown_is_occupied
    ):
        return WeightedPathResult((), math.inf, 0)

    straight = ((1, 0), (-1, 0), (0, 1), (0, -1))
    diagonal = ((1, 1), (1, -1), (-1, 1), (-1, -1))
    neighbors = straight + diagonal if allow_diagonal else straight
    frontier = [(math.hypot(goal[0] - start[0], goal[1] - start[1]), 0.0, start)]
    came_from: dict[Cell, Cell] = {}
    best_cost: dict[Cell, float] = {start: 0.0}
    closed: set[Cell] = set()

    while frontier:
        _, current_cost, current = heapq.heappop(frontier)
        if current in closed:
            continue
        closed.add(current)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return WeightedPathResult(
                tuple(reversed(path)), float(current_cost), len(closed)
            )
        for dx, dy in neighbors:
            candidate = current[0] + dx, current[1] + dy
            if cell_is_blocked(costs, candidate, unknown_is_occupied):
                continue
            if dx and dy and (
                cell_is_blocked(costs, (current[0] + dx, current[1]), unknown_is_occupied)
                or cell_is_blocked(costs, (current[0], current[1] + dy), unknown_is_occupied)
            ):
                continue
            candidate_cost = current_cost + step_cost(
                costs, current, candidate, thermal_cost_weight, thermal_cost_power,
                fixed_co_ppm, co_safe_ppm, co_blocked_ppm,
                co_cost_weight, co_cost_power,
            )
            if candidate_cost >= best_cost.get(candidate, math.inf) - 1e-12:
                continue
            best_cost[candidate] = candidate_cost
            came_from[candidate] = current
            heuristic = math.hypot(goal[0] - candidate[0], goal[1] - candidate[1])
            heapq.heappush(
                frontier, (candidate_cost + heuristic, candidate_cost, candidate)
            )
    return WeightedPathResult((), math.inf, len(closed))
