"""Conservative thermal-risk-aware simplification of grid paths."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .weighted_planner import Cell, cell_is_blocked, path_cost


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


def segment_is_safe(
    start: Cell,
    end: Cell,
    data: np.ndarray,
    unknown_is_occupied: bool,
) -> bool:
    cells = supercover_line(start, end)
    if any(cell_is_blocked(data, cell, unknown_is_occupied) for cell in cells):
        return False
    # Exact corner crossings include side cells; requiring all of them free
    # conservatively prevents diagonal corner cutting.
    return True


def _thermal_exposure(cells: Sequence[Cell], data: np.ndarray) -> float:
    if not cells:
        return math.inf
    return sum(max(0.0, min(99.0, float(data[y, x]))) for x, y in cells)


def simplify_path_safely(
    raw_path: Sequence[Cell],
    data: np.ndarray,
    *,
    unknown_is_occupied: bool = True,
    thermal_cost_weight: float = 8.0,
    thermal_cost_power: float = 2.0,
    maximum_risk_ratio: float = 1.0,
    risk_absolute_tolerance: float = 0.0,
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
        source, costs, thermal_cost_weight, thermal_cost_power, unknown_is_occupied
    )
    if not source or not math.isfinite(raw_cost):
        return SimplifiedPathResult((), False, raw_cost, math.inf, 0)
    if len(source) <= 2:
        return SimplifiedPathResult(source, True, raw_cost, raw_cost, 0)

    simplified = [source[0]]
    anchor = 0
    rejected = 0
    while anchor < len(source) - 1:
        selected = anchor + 1
        for candidate in range(len(source) - 1, anchor, -1):
            shortcut_cells = supercover_line(source[anchor], source[candidate])
            if not segment_is_safe(
                source[anchor], source[candidate], costs, unknown_is_occupied
            ):
                rejected += 1
                continue
            original_cells = source[anchor:candidate + 1]
            shortcut_risk = _thermal_exposure(shortcut_cells, costs)
            original_risk = _thermal_exposure(original_cells, costs)
            if shortcut_risk > (
                original_risk * maximum_risk_ratio + risk_absolute_tolerance + 1e-12
            ):
                rejected += 1
                continue
            selected = candidate
            break
        simplified.append(source[selected])
        anchor = selected

    expanded = expanded_path(simplified)
    safe = all(
        not cell_is_blocked(costs, cell, unknown_is_occupied) for cell in expanded
    )
    simplified_cost = path_cost(
        expanded, costs, thermal_cost_weight, thermal_cost_power, unknown_is_occupied
    )
    return SimplifiedPathResult(
        tuple(simplified), bool(safe and math.isfinite(simplified_cost)),
        raw_cost, simplified_cost, rejected,
    )
