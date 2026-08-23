"""Compress a high-resolution planning grid into one representative cost per
existing ~1 m reference waypoint.

``waypoint_cost = max(raw /planning_grid cell value)`` over every cell within a
world-space Euclidean radius of the waypoint. This deliberately mirrors
``inno_autonav.weighted_planner.cell_is_blocked``'s existing raw-grid convention
(``-1`` = unknown, ``0..99`` = relative cost, ``>=100``/non-finite = lethal) --
that function is reused directly, not reimplemented, so a blocked/unknown cell
inside the radius makes the whole waypoint ``math.inf`` exactly like the rest of
the planner treats blocked cells.

Note on prior art: ``reference_waypoint_graph.ReferenceWaypointGraphPlanner``
already has a private ``_waypoint_cost()`` with the same radius/max formula, but
it operates on an already thermal/CO-*decoded* traversal-cost array built fresh
inside ``plan()`` on every call (no caching, no revision dedup, tightly coupled
to that class's A* internals). This module intentionally does not import or
modify it -- Stage 8-2 is a standalone, cacheable projector over the *raw*
``/planning_grid`` encoding instead, decoupled from thermal/CO decoding
entirely; see the Stage 8-2 report for the full comparison. The formula (radius
in metres, circular, conservative max, fail-fast on any blocked cell) is the
same proven convention either way -- only the input array's encoding differs.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np

from .grid_utils import MapGrid, world_to_grid
from .weighted_planner import Cell, cell_is_blocked


@dataclass(frozen=True)
class WaypointCostProjectorConfig:
    waypoint_cost_radius_m: float = 0.8
    unknown_is_occupied: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.unknown_is_occupied, bool):
            raise TypeError("unknown_is_occupied must be bool")
        value = self.waypoint_cost_radius_m
        if isinstance(value, bool) or not math.isfinite(float(value)) or float(value) <= 0.0:
            raise ValueError("waypoint_cost_radius_m must be finite and positive")


class WaypointCostProjector:
    """Cache waypoint->radius-cell lookups; recompute only what actually changed.

    Static geometry (map resolution/origin/size) is separated from dynamic cost
    (the grid's cell values): the radius-cell lookup is rebuilt only when the
    grid's geometry signature changes, never on every ``project_costs()`` call
    (Stage 8-3). ``project_costs()`` also skips recomputation entirely on a
    duplicate revision, or (when no revision is supplied) a byte-identical grid
    (Stage 8-4). ``set_waypoints()``/``status()`` were added alongside those two
    stages for waypoint-set-change invalidation and test/benchmark introspection
    -- everything else here is unchanged from Stage 8-2.
    """

    def __init__(
        self, waypoints_world: Mapping[str, tuple[float, float]],
        config: WaypointCostProjectorConfig | None = None,
    ) -> None:
        if not waypoints_world:
            raise ValueError("waypoints_world must not be empty")
        self.waypoints_world = dict(waypoints_world)
        self.config = config or WaypointCostProjectorConfig()
        self._waypoint_signature = self._waypoint_signature_of(self.waypoints_world)
        self._geometry_signature: tuple | None = None
        self._cell_lookup: dict[str, tuple[Cell, ...]] = {}
        self._last_revision: int | None = None
        self._last_data: np.ndarray | None = None
        self._last_costs: dict[str, float] = {}
        self._cache_rebuild_count = 0
        self._projection_count = 0

    @property
    def cell_lookup(self) -> Mapping[str, tuple[Cell, ...]]:
        """Read-only view of the cached waypoint -> radius-cell lookup (tests/introspection)."""
        return dict(self._cell_lookup)

    def status(self) -> dict:
        """Minimal introspection for tests/benchmarks (Stage 8-3/8-4 section 17).

        No new ROS topic is published for this -- callers (tests, or a future
        Stage 8-5 planner in the same process) just call this directly.
        """
        return {
            "cache_initialized": self._geometry_signature is not None,
            "cached_waypoint_count": len(self._cell_lookup),
            "geometry_key": self._geometry_signature,
            "last_revision": self._last_revision,
            "projection_count": self._projection_count,
            "cache_rebuild_count": self._cache_rebuild_count,
        }

    def set_waypoints(self, waypoints_world: Mapping[str, tuple[float, float]]) -> None:
        """Replace the tracked waypoint set if it actually changed.

        Compared by ``(name, x, y)`` signature, not just count (section 14) --
        a waypoint document reload with the same count but moved/renamed points
        must still invalidate the geometry cache; an identical reload must not.
        """
        if not waypoints_world:
            raise ValueError("waypoints_world must not be empty")
        signature = self._waypoint_signature_of(waypoints_world)
        if signature == self._waypoint_signature:
            return
        self.waypoints_world = dict(waypoints_world)
        self._waypoint_signature = signature
        self._geometry_signature = None  # force a rebuild on the next project_costs()

    @staticmethod
    def _waypoint_signature_of(waypoints_world: Mapping[str, tuple[float, float]]) -> tuple:
        return tuple(sorted(
            (name, float(x), float(y)) for name, (x, y) in waypoints_world.items()
        ))

    @staticmethod
    def _geometry_signature_of(grid: MapGrid) -> tuple:
        return (
            grid.width, grid.height, grid.resolution,
            grid.origin_x, grid.origin_y, grid.origin_yaw,
        )

    def _rebuild_cell_lookup(self, grid: MapGrid) -> None:
        radius_m = self.config.waypoint_cost_radius_m
        radius_cells = int(math.ceil(radius_m / grid.resolution))
        lookup: dict[str, tuple[Cell, ...]] = {}
        for waypoint_id, (x, y) in self.waypoints_world.items():
            center_col, center_row = world_to_grid(x, y, grid)
            cells = []
            for row in range(center_row - radius_cells, center_row + radius_cells + 1):
                for col in range(center_col - radius_cells, center_col + radius_cells + 1):
                    distance_m = math.hypot(col - center_col, row - center_row) * grid.resolution
                    if distance_m > radius_m + 1e-12:
                        continue
                    cells.append((col, row))
            lookup[waypoint_id] = tuple(cells)
        self._cell_lookup = lookup
        self._geometry_signature = self._geometry_signature_of(grid)
        self._cache_rebuild_count += 1
        # A geometry rebuild invalidates any cached cost/revision/data dedup --
        # the same revision number could now mean a different set of cells.
        self._last_revision = None
        self._last_data = None

    def project_costs(
        self, grid: MapGrid, *, revision: int | None = None,
    ) -> dict[str, float]:
        """Return ``{waypoint_id: max(cost) within radius}`` for the given grid.

        ``revision`` is optional: ``/planning_grid`` (unlike Stage 3's
        ``/hazard/snapshot``) carries no revision counter today, so passing
        ``None`` falls back to the exact same "did the array actually change"
        dedup ``astar_replanner`` already uses for its own grids
        (geometry-equality + ``np.array_equal`` on the data) instead of
        inventing a synthetic revision system.
        """
        signature = self._geometry_signature_of(grid)
        if signature != self._geometry_signature:
            self._rebuild_cell_lookup(grid)
        data = np.asarray(grid.data)
        if revision is not None:
            if revision == self._last_revision:
                return dict(self._last_costs)
        elif self._last_data is not None and np.array_equal(self._last_data, data):
            return dict(self._last_costs)
        costs = {
            waypoint_id: self._max_cost(data, cells)
            for waypoint_id, cells in self._cell_lookup.items()
        }
        self._last_costs = costs
        self._last_revision = revision
        self._last_data = data.copy()
        self._projection_count += 1
        return dict(costs)

    def _max_cost(self, data: np.ndarray, cells: Sequence[Cell]) -> float:
        if not cells:
            return math.inf
        best = -math.inf
        for cell in cells:
            if cell_is_blocked(
                data, cell, self.config.unknown_is_occupied, costs_are_traversal=False,
            ):
                return math.inf
            value = float(data[cell[1], cell[0]])
            if value > best:
                best = value
        return best
