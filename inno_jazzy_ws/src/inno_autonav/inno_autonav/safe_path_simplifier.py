"""Conservative thermal-risk-aware simplification of grid paths."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .weighted_planner import (
    Cell,
    cell_is_blocked,
    path_cost,
    traversal_multiplier,
)


@dataclass(frozen=True)
class SimplifiedPathResult:
    path: tuple[Cell, ...]
    safe: bool
    raw_cost: float
    simplified_cost: float
    rejected_shortcuts: int


def _sign(value: int) -> int:
    return (value > 0) - (value < 0)


def supercover_line(start: Cell, end: Cell) -> tuple[Cell, ...]:
    """Return every grid cell touched by a centre-to-centre segment."""
    x, y = int(start[0]), int(start[1])
    end_x, end_y = int(end[0]), int(end[1])
    if (x, y) == (end_x, end_y):
        return ((x, y),)
    dx, dy = end_x - x, end_y - y
    step_x, step_y = _sign(dx), _sign(dy)
    delta_x = math.inf if dx == 0 else 1.0 / abs(dx)
    delta_y = math.inf if dy == 0 else 1.0 / abs(dy)
    maximum_x = math.inf if dx == 0 else 0.5 / abs(dx)
    maximum_y = math.inf if dy == 0 else 0.5 / abs(dy)
    output = [(x, y)]

    def append(cell: Cell) -> None:
        if not output or output[-1] != cell:
            output.append(cell)

    while (x, y) != (end_x, end_y):
        if abs(maximum_x - maximum_y) <= 1e-12:
            append((x + step_x, y))
            append((x, y + step_y))
            x += step_x
            y += step_y
            append((x, y))
            maximum_x += delta_x
            maximum_y += delta_y
        elif maximum_x < maximum_y:
            x += step_x
            append((x, y))
            maximum_x += delta_x
        else:
            y += step_y
            append((x, y))
            maximum_y += delta_y
    return tuple(output)


def expanded_path(path: Sequence[Cell]) -> tuple[Cell, ...]:
    points = tuple((int(x), int(y)) for x, y in path)
    if not points:
        return ()
    output = [points[0]]
    for start, end in zip(points, points[1:]):
        for cell in supercover_line(start, end)[1:]:
            if cell != output[-1]:
                output.append(cell)
    return tuple(output)


def extract_direction_change_points(path: Sequence[Cell]) -> tuple[Cell, ...]:
    """Keep the start, grid-direction changes, and goal like factory_v5."""
    points = []
    for x, y in path:
        point = int(x), int(y)
        if not points or point != points[-1]:
            points.append(point)
    if len(points) <= 2:
        return tuple(points)
    output = [points[0]]
    previous = (
        _sign(points[1][0] - points[0][0]),
        _sign(points[1][1] - points[0][1]),
    )
    for index in range(1, len(points) - 1):
        direction = (
            _sign(points[index + 1][0] - points[index][0]),
            _sign(points[index + 1][1] - points[index][1]),
        )
        if direction != previous:
            output.append(points[index])
        previous = direction
    output.append(points[-1])
    return tuple(output)


def segment_is_safe(
    start: Cell,
    end: Cell,
    data: np.ndarray,
    unknown_is_occupied: bool,
    costs_are_traversal: bool = False,
) -> bool:
    cells = supercover_line(start, end)
    if any(cell_is_blocked(
        data, cell, unknown_is_occupied, costs_are_traversal
    ) for cell in cells):
        return False
    # Exact corner crossings include side cells; requiring all of them free
    # conservatively prevents diagonal corner cutting.
    return True


def _segment_risk(
    start: Cell,
    end: Cell,
    data: np.ndarray,
    planner_parameters: tuple[float, ...],
    costs_are_traversal: bool,
) -> tuple[float, float, float]:
    cells = supercover_line(start, end)
    multipliers = (
        [float(data[y, x]) for x, y in cells]
        if costs_are_traversal else [
            traversal_multiplier(float(data[y, x]), *planner_parameters)
            for x, y in cells
        ]
    )
    average = float(np.mean(multipliers)) if multipliers else math.inf
    maximum = max(multipliers, default=math.inf)
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    return average * distance, average, maximum


def _path_risk(
    path: Sequence[Cell],
    data: np.ndarray,
    planner_parameters: tuple[float, ...],
    costs_are_traversal: bool,
) -> tuple[float, float, float]:
    if len(path) == 1:
        return _segment_risk(
            path[0], path[0], data, planner_parameters,
            costs_are_traversal,
        )
    segments = [
        _segment_risk(
            first, second, data, planner_parameters,
            costs_are_traversal,
        )
        for first, second in zip(path, path[1:])
    ]
    accumulated = sum(item[0] for item in segments)
    distance = sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(path, path[1:])
    )
    average = accumulated / distance if distance > 0.0 else segments[0][1]
    return accumulated, average, max(item[2] for item in segments)


def simplify_path_safely(
    raw_path: Sequence[Cell],
    data: np.ndarray,
    *,
    unknown_is_occupied: bool = True,
    thermal_cost_weight: float = 24.0,
    thermal_cost_power: float = 1.5,
    fixed_co_ppm: float = 0.0,
    co_safe_ppm: float = 0.0,
    co_blocked_ppm: float = 1600.0,
    co_cost_weight: float = 8.0,
    co_cost_power: float = 2.0,
    maximum_risk_ratio: float = 1.0,
    risk_absolute_tolerance: float = 0.0,
    costs_are_traversal: bool = False,
) -> SimplifiedPathResult:
    """Greedily shortcut only through collision-free, non-riskier segments."""
    source = tuple((int(x), int(y)) for x, y in raw_path)
    costs = np.asarray(data, dtype=float)
    if costs.ndim != 2:
        raise ValueError("planning data must be a two-dimensional array")
    if maximum_risk_ratio < 1.0 or not math.isfinite(maximum_risk_ratio):
        raise ValueError("maximum_risk_ratio must be finite and at least 1")
    if risk_absolute_tolerance < 0.0 or not math.isfinite(risk_absolute_tolerance):
        raise ValueError("risk_absolute_tolerance must be finite and non-negative")
    raw_cost = path_cost(
        source, costs, thermal_cost_weight, thermal_cost_power,
        unknown_is_occupied, fixed_co_ppm, co_safe_ppm, co_blocked_ppm,
        co_cost_weight, co_cost_power,
        costs_are_traversal,
    )
    if not source or not math.isfinite(raw_cost):
        return SimplifiedPathResult((), False, raw_cost, math.inf, 0)
    if any(
        not segment_is_safe(
            first, second, costs, unknown_is_occupied, costs_are_traversal,
        )
        for first, second in zip(source, source[1:])
    ):
        return SimplifiedPathResult((), False, raw_cost, math.inf, 0)
    if len(source) <= 2:
        return SimplifiedPathResult(source, True, raw_cost, raw_cost, 0)

    planner_parameters = (
        thermal_cost_weight, thermal_cost_power, fixed_co_ppm, co_safe_ppm,
        co_blocked_ppm, co_cost_weight, co_cost_power,
    )
    corners = extract_direction_change_points(source)
    corner_positions = []
    search_from = 0
    for corner in corners:
        position = source.index(corner, search_from)
        corner_positions.append(position)
        search_from = position + 1

    simplified = [corners[0]]
    anchor = 0
    rejected = 0
    while anchor < len(corners) - 1:
        selected = anchor + 1
        for candidate in range(len(corners) - 1, anchor, -1):
            if not segment_is_safe(
                corners[anchor], corners[candidate], costs, unknown_is_occupied
                , costs_are_traversal
            ):
                rejected += 1
                continue
            reference = source[
                corner_positions[anchor]:corner_positions[candidate] + 1
            ]
            original_risk = _path_risk(
                reference, costs, planner_parameters, costs_are_traversal
            )
            shortcut_risk = _segment_risk(
                corners[anchor], corners[candidate], costs,
                planner_parameters, costs_are_traversal,
            )
            if any(
                shortcut > original * maximum_risk_ratio
                + risk_absolute_tolerance + 1e-12
                for shortcut, original in zip(shortcut_risk, original_risk)
            ):
                rejected += 1
                continue
            selected = candidate
            break
        simplified.append(corners[selected])
        anchor = selected

    expanded = expanded_path(simplified)
    safe = all(
        not cell_is_blocked(
            costs, cell, unknown_is_occupied, costs_are_traversal
        ) for cell in expanded
    )
    simplified_cost = path_cost(
        expanded, costs, thermal_cost_weight, thermal_cost_power,
        unknown_is_occupied, fixed_co_ppm, co_safe_ppm, co_blocked_ppm,
        co_cost_weight, co_cost_power,
        costs_are_traversal,
    )
    return SimplifiedPathResult(
        tuple(simplified), bool(safe and math.isfinite(simplified_cost)),
        raw_cost, simplified_cost, rejected,
    )
