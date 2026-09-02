"""Cost-trend and world-direction policy for deterministic exit switching.

Ports the subset of ``fire_robot/simulator/factory_v5/navigation/exit_switching.py``
that ``run_partial_costmap_evacuation.py`` actually wires into its main loop.
``RouteCostTrendMonitor`` (a generic threshold/ratio-based trend monitor also
defined upstream) is never imported by the simulation's own main loop -- only
``RouteTemperatureTrendMonitor`` is -- so it is intentionally not ported here; see
the Stage 7 report for the full parity analysis. ``ExitSwitchingConfig`` still
carries ``minimum_consecutive_increases``/``minimum_increase_ratio``/
``minimum_absolute_increase`` for structural/``from_mapping`` parity with the
upstream dataclass, but ``RouteTemperatureTrendMonitor`` -- exactly like upstream
-- never reads them: it hardcodes "all ``evaluation_window - 1`` consecutive
samples must be strictly increasing".
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .safe_path_simplifier import expanded_path


Cell = tuple[int, int]


@dataclass(frozen=True)
class ExitSwitchingConfig:
    enabled: bool = True
    evaluation_window: int = 5
    minimum_consecutive_increases: int = 3
    minimum_increase_ratio: float = 0.10
    minimum_absolute_increase: float = 0.25
    minimum_direction_difference_deg: float = 90.0
    switch_cooldown_sec: float = 10.0
    additional_travel_before_switch_m: float = 0.0
    danger_expected_min_temperature_c: float = 36.0
    danger_expected_confirmation_sec: float = 3.0
    danger_expected_max_observation_gap_sec: float = 1.0
    danger_expected_path_radius_m: float = 0.30

    def __post_init__(self) -> None:
        if not isinstance(self.enabled, bool):
            raise TypeError("enabled must be bool")
        for name in ("evaluation_window", "minimum_consecutive_increases"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be an integer of at least 1")
        if self.minimum_consecutive_increases >= self.evaluation_window:
            raise ValueError(
                "minimum_consecutive_increases must be smaller than evaluation_window"
            )
        for name in (
            "minimum_increase_ratio", "minimum_absolute_increase",
            "switch_cooldown_sec", "additional_travel_before_switch_m",
            "danger_expected_min_temperature_c",
            "danger_expected_confirmation_sec",
            "danger_expected_max_observation_gap_sec",
            "danger_expected_path_radius_m",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(float(value)) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        angle = self.minimum_direction_difference_deg
        if isinstance(angle, bool) or not 0 <= float(angle) <= 180:
            raise ValueError("minimum_direction_difference_deg must be in [0,180]")

    @classmethod
    def from_mapping(cls, values):
        values = dict(values or {})
        unknown = set(values) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f"unknown exit_switching settings: {sorted(unknown)}")
        return cls(**values)


@dataclass(frozen=True)
class CostTrendDecision:
    switch_required: bool
    consecutive_increases: int
    baseline_average_cost: float | None
    current_average_cost: float | None
    reason: str | None


@dataclass(frozen=True)
class RouteTemperatureSample:
    costmap_revision: int
    evaluated_at: float
    maximum_temperature_c: float
    average_route_cost: float


def evaluate_path_cost(
    path_grid: Sequence[Cell], cost_map: np.ndarray,
) -> tuple[float, float, float] | None:
    """Evaluate every cell touched by a waypoint path in ``(col,row)`` order.

    Reuses :func:`inno_autonav.safe_path_simplifier.expanded_path` (Stage 1's
    supercover) instead of a third Bresenham/supercover implementation -- this is
    the same construction as the simulation's own ``evaluate_path_cost``.
    """
    waypoints = tuple((int(col), int(row)) for col, row in path_grid)
    if not waypoints:
        return None
    cells = expanded_path(waypoints)
    costs = np.asarray(cost_map, dtype=float)
    values = []
    for col, row in cells:
        if row < 0 or col < 0 or row >= costs.shape[0] or col >= costs.shape[1]:
            return None
        value = float(costs[row, col])
        if not math.isfinite(value) or value < 0:
            return None
        values.append(value)
    return float(sum(values)), float(np.mean(values)), float(max(values))


class RouteTemperatureTrendMonitor:
    """Gate DANGER_EXPECTED on heat, then track forward-route cost rises.

    Ported from factory_v5's ``RouteTemperatureTrendMonitor``. One deliberate,
    strictly-defensive addition over upstream: an explicit
    ``temperature_observed_mask`` parameter. Upstream relies solely on
    ``math.isfinite(value)`` to skip unobserved cells, which is safe *only*
    because its belief map also uses NaN as the "unobserved" sentinel -- exactly
    like Stage 3's ``HazardBelief.temperature_belief_map``
    (``np.full(shape, np.nan)``, only set where actually observed). Both
    mechanisms are equivalent against the current sensor-belief representation;
    the explicit mask is kept so this stays correct even if that representation
    ever changes, matching the same explicit-mask style already used in
    ``event_replanning.validate_remaining_path``.
    """

    def __init__(
        self, evaluation_window: int = 6,
        minimum_temperature_c: float = 40.0,
    ):
        if isinstance(evaluation_window, bool) or evaluation_window < 2:
            raise ValueError("evaluation_window must be an integer of at least 2")
        self.evaluation_window = int(evaluation_window)
        if not math.isfinite(float(minimum_temperature_c)):
            raise ValueError("minimum_temperature_c must be finite")
        self.minimum_temperature_c = float(minimum_temperature_c)
        self._samples: deque[RouteTemperatureSample] = deque(maxlen=self.evaluation_window)
        self._last_revision: int | None = None

    @property
    def samples(self) -> tuple[RouteTemperatureSample, ...]:
        return tuple(self._samples)

    def reset(self) -> None:
        self._samples.clear()
        self._last_revision = None

    def record(
        self, path_grid: Sequence[Cell], cost_map: np.ndarray,
        temperature_map: np.ndarray, temperature_observed_mask: np.ndarray,
        *, revision: int, evaluated_at: float,
    ) -> CostTrendDecision:
        if self._last_revision == int(revision):
            return CostTrendDecision(False, 0, None, None, None)
        self._last_revision = int(revision)
        waypoints = tuple((int(col), int(row)) for col, row in path_grid)
        temperatures = np.asarray(temperature_map, dtype=float)
        observed = np.asarray(temperature_observed_mask, dtype=bool)
        values = []
        for col, row in expanded_path(waypoints):
            if (
                0 <= row < temperatures.shape[0] and 0 <= col < temperatures.shape[1]
                and observed[row, col]
            ):
                value = float(temperatures[row, col])
                if math.isfinite(value):
                    values.append(value)
        if not values:
            return CostTrendDecision(False, 0, None, None, None)
        maximum = max(values)
        if maximum < self.minimum_temperature_c:
            self._samples.clear()
            return CostTrendDecision(False, 0, None, None, None)
        evaluated = evaluate_path_cost(waypoints, cost_map)
        if evaluated is None:
            return CostTrendDecision(False, 0, None, None, "invalid_path_cost")
        _, average_cost, _ = evaluated
        self._samples.append(RouteTemperatureSample(
            int(revision), float(evaluated_at), maximum, average_cost,
        ))
        samples = tuple(self._samples)
        consecutive = 0
        for previous, current in reversed(tuple(zip(samples, samples[1:]))):
            if current.average_route_cost > previous.average_route_cost + 1e-12:
                consecutive += 1
            else:
                break
        required = (
            len(samples) == self.evaluation_window
            and consecutive == self.evaluation_window - 1
        )
        reason = None if not required else (
            f"route_temperature_at_least_{self.minimum_temperature_c:.1f}C;"
            "sustained_route_cost_increase:"
            + "->".join(f"{item.average_route_cost:.3f}" for item in samples)
        )
        return CostTrendDecision(
            required, consecutive, samples[0].average_route_cost,
            average_cost, reason,
        )


@dataclass
class DelayedCostSwitch:
    """Delay a soft cost-driven switch by actual robot travel distance."""

    required_distance_m: float
    exit_id: str | None = None
    reason: str | None = None
    start_travel_distance_m: float | None = None

    def __post_init__(self) -> None:
        value = float(self.required_distance_m)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("required delayed-switch distance must be non-negative")
        self.required_distance_m = value

    @property
    def active(self) -> bool:
        return self.exit_id is not None and self.start_travel_distance_m is not None

    def arm(self, exit_id: str, reason: str, travelled_distance_m: float) -> None:
        travelled_distance_m = float(travelled_distance_m)
        if not math.isfinite(travelled_distance_m) or travelled_distance_m < 0.0:
            raise ValueError("travelled distance must be finite and non-negative")
        self.exit_id = str(exit_id)
        self.reason = str(reason)
        self.start_travel_distance_m = travelled_distance_m

    def travelled_distance(self, travelled_distance_m: float) -> float:
        if not self.active:
            return 0.0
        return max(0.0, float(travelled_distance_m) - self.start_travel_distance_m)

    def ready(self, travelled_distance_m: float) -> bool:
        return self.active and self.travelled_distance(
            travelled_distance_m
        ) >= self.required_distance_m - 1e-12

    def clear(self) -> None:
        self.exit_id = None
        self.reason = None
        self.start_travel_distance_m = None


def current_direction_world(
    robot_position_world, next_waypoint_world=None, recent_positions_world=(),
    yaw_rad: float = 0.0,
) -> tuple[float, float]:
    """Return next-waypoint, recent-motion, then yaw direction, in that order."""
    x, y = map(float, robot_position_world)
    candidates = []
    if next_waypoint_world is not None:
        candidates.append((next_waypoint_world[0] - x, next_waypoint_world[1] - y))
    recent = tuple(recent_positions_world)
    if len(recent) >= 2:
        candidates.append((recent[-1][0] - recent[-2][0], recent[-1][1] - recent[-2][1]))
    candidates.append((math.cos(float(yaw_rad)), math.sin(float(yaw_rad))))
    for dx, dy in candidates:
        norm = math.hypot(dx, dy)
        if norm > 1e-9:
            return float(dx / norm), float(dy / norm)
    return 1.0, 0.0


def is_opposite_direction(
    direction_world, robot_position_world, target_position_world,
    *, minimum_difference_deg: float,
) -> bool:
    dx = float(target_position_world[0]) - float(robot_position_world[0])
    dy = float(target_position_world[1]) - float(robot_position_world[1])
    norm = math.hypot(dx, dy)
    if norm <= 1e-9:
        return False
    dot = (direction_world[0] * dx + direction_world[1] * dy) / norm
    threshold = math.cos(math.radians(float(minimum_difference_deg)))
    return dot <= threshold + 1e-12
