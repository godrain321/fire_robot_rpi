"""Pure helpers for Stage 5: gas belief -> raw /planning_grid encoding.

ROS-independent so both ``hazard_belief_node`` (overlay producer) and
``planning_grid_hazard_merge`` (compositor) share one implementation and it
stays unit-testable without a ROS runtime. No new cost semantics here -- the
overlay reuses inno_thermal's ``(value - safe) / (blocked - safe)`` ratio
encoding and the merge reuses ``combine_cost_grids``' max rule.
"""

from __future__ import annotations

import numpy as np


def gas_overlay_cells(
    co_belief_map: np.ndarray,
    co_observed_mask: np.ndarray,
    safe_threshold: float,
    blocked_threshold: float,
) -> np.ndarray:
    """Return an int16 grid in the raw ``/planning_grid`` convention.

    ``0..99`` for observed cells (``(value - safe) / (blocked - safe)`` clipped
    to ``[0, 1]`` then scaled by 99), ``100`` where the belief is at/above the
    blocked threshold, ``0`` where a cell has no gas observation.
    """
    span = float(blocked_threshold) - float(safe_threshold)
    if not np.isfinite(span) or span <= 0.0:
        raise ValueError("blocked_threshold must exceed safe_threshold")
    values = np.asarray(co_belief_map, dtype=float)
    observed = np.asarray(co_observed_mask, dtype=bool)
    cells = np.zeros(values.shape, dtype=np.int16)
    if observed.any():
        ratio = np.clip((values[observed] - float(safe_threshold)) / span, 0.0, 1.0)
        cells[observed] = np.rint(99.0 * ratio).astype(np.int16)
        cells[observed & (values >= float(blocked_threshold))] = 100
    return cells


def merge_planning_cells(
    base_cells: np.ndarray,
    gas_cells: np.ndarray,
    *,
    unknown_is_occupied: bool,
) -> np.ndarray:
    """Max-merge a 0..99/100 gas overlay onto a raw ``/planning_grid``.

    Same rule as ``inno_autonav.weighted_planner.combine_cost_grids`` applies to
    the thermal layer: only known, non-lethal base cells are raised; a ``100``
    overlay cell carries straight through as lethal.
    """
    base = np.asarray(base_cells, dtype=np.int16)
    gas = np.asarray(gas_cells, dtype=np.int16)
    if base.shape != gas.shape:
        raise ValueError("gas overlay geometry differs from base grid")
    merged = base.copy()
    eligible = base < 100
    if unknown_is_occupied:
        eligible &= base >= 0
    merged[eligible] = np.maximum(merged[eligible], gas[eligible])
    return merged
