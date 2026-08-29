"""ROS-independent partial fire costmap matching factory_v5 semantics."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


Cell = tuple[int, int]


@dataclass(frozen=True)
class HazardGridGeometry:
    width: int
    height: int
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0
    frame_id: str = "map"

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid width and height must be positive")
        values = self.resolution, self.origin_x, self.origin_y, self.origin_yaw
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("grid geometry must be finite")
        if self.resolution <= 0.0 or not self.frame_id:
            raise ValueError("grid resolution/frame are invalid")

    def world_to_grid(self, x: float, y: float) -> Cell | None:
        dx, dy = float(x) - self.origin_x, float(y) - self.origin_y
        cosine, sine = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        col = math.floor((cosine * dx + sine * dy) / self.resolution)
        row = math.floor((-sine * dx + cosine * dy) / self.resolution)
        if 0 <= col < self.width and 0 <= row < self.height:
            return int(col), int(row)
        return None

    def grid_to_world(self, col: int, row: int) -> tuple[float, float]:
        local_x = (int(col) + 0.5) * self.resolution
        local_y = (int(row) + 0.5) * self.resolution
        cosine, sine = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )


@dataclass(frozen=True)
class HazardBeliefConfig:
    base_cost: float = 1.0
    temperature_safe_c: float = 40.0
    temperature_blocked_c: float = 60.0
    temperature_weight: float = 24.0
    temperature_power: float = 1.5
    co_enabled: bool = False
    # "legacy_ppm": gas cost uses co_safe_ppm/co_blocked_ppm (unchanged).
    # "adc": gas cost uses gas_safe_adc/gas_blocked_adc against the raw
    # /mq135/filtered_adc scalar (no ppm conversion). See gas_*_threshold.
    gas_input_mode: str = "legacy_ppm"
    co_safe_ppm: float = 100.0
    co_blocked_ppm: float = 1600.0
    gas_safe_adc: float = 0.0
    gas_blocked_adc: float = 4096.0
    co_weight: float = 8.0
    co_power: float = 2.0
    gas_update_radius_m: float = 0.0
    gas_gaussian_sigma_m: float = 0.5
    unknown_penalty: float = 0.0
    unobserved_temperature_penalty: float = 0.0
    unobserved_co_penalty: float = 0.0
    stale_enabled: bool = True
    stale_grace_period_s: float = 5.0
    stale_cost_per_second: float = 0.05
    stale_maximum_cost: float = 2.0
    stale_apply_to_temperature: bool = True
    stale_apply_to_co: bool = True

    @property
    def gas_safe_threshold(self) -> float:
        if self.gas_input_mode == "adc":
            return self.gas_safe_adc
        return self.co_safe_ppm

    @property
    def gas_blocked_threshold(self) -> float:
        if self.gas_input_mode == "adc":
            return self.gas_blocked_adc
        return self.co_blocked_ppm

    def __post_init__(self):
        if self.base_cost <= 0.0:
            raise ValueError("base_cost must be positive")
        if self.temperature_blocked_c <= self.temperature_safe_c:
            raise ValueError("temperature blocked threshold must exceed safe")
        if self.gas_input_mode not in ("legacy_ppm", "adc"):
            raise ValueError(
                "gas_input_mode must be 'legacy_ppm' or 'adc'"
            )
        if self.co_blocked_ppm <= self.co_safe_ppm:
            raise ValueError("CO blocked threshold must exceed safe")
        if self.gas_blocked_threshold <= self.gas_safe_threshold:
            raise ValueError(
                "gas blocked threshold must exceed safe threshold "
                f"({self.gas_input_mode} mode)"
            )
        nonnegative = (
            self.temperature_weight, self.co_weight,
            self.gas_update_radius_m, self.unknown_penalty,
            self.unobserved_temperature_penalty,
            self.unobserved_co_penalty, self.stale_grace_period_s,
            self.stale_cost_per_second, self.stale_maximum_cost,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in nonnegative):
            raise ValueError("hazard weights/radii must be finite and non-negative")
        if self.temperature_power <= 0.0 or self.co_power <= 0.0:
            raise ValueError("hazard powers must be positive")
        if self.gas_gaussian_sigma_m <= 0.0:
            raise ValueError("gas Gaussian sigma must be positive")


@dataclass(frozen=True)
class BeliefUpdate:
    changed_cells: frozenset[Cell]
    newly_blocked_cells: frozenset[Cell]


class HazardBelief:
    """Maintain only localized sensor observations; no Ground Truth input."""

    def __init__(self, geometry, static_obstacle_map, config=None):
        self.geometry = geometry
        self.config = config or HazardBeliefConfig()
        shape = geometry.height, geometry.width
        static = np.asarray(static_obstacle_map, dtype=bool)
        if static.shape != shape:
            raise ValueError(f"static map shape={static.shape}, expected={shape}")
        self.static_obstacle_map = static.copy()
        self.temperature_observed_mask = np.zeros(shape, dtype=bool)
        self.co_observed_mask = np.zeros(shape, dtype=bool)
        self.observed_mask = np.zeros(shape, dtype=bool)
        self.temperature_belief_map = np.full(shape, np.nan)
        self.co_belief_map = np.full(shape, np.nan)
        self.co_confidence_map = np.zeros(shape)
        self.temperature_cost_map = np.zeros(shape)
        self.co_cost_map = np.zeros(shape)
        self.unknown_cost_map = np.zeros(shape)
        self.estimated_fire_cost_map = np.zeros(shape)
        self.stale_observation_cost_map = np.zeros(shape)
        self.dynamic_obstacle_map = np.zeros(shape, dtype=bool)
        self.dynamic_inflated_obstacle_map = np.zeros(shape, dtype=bool)
        self.blocked_mask = static.copy()
        self.final_cost_map = np.full(shape, self.config.base_cost)
        self.last_observed_time_map = np.full(shape, np.nan)
        self.current_time = 0.0
        self.revision = 0
        self.recalculate()

    @property
    def shape(self):
        return self.final_cost_map.shape

    def _finish(self, changed, old_blocked):
        self.recalculate()
        if changed:
            self.revision += 1
        new_blocked = self.blocked_mask & ~old_blocked
        newly = {(int(col), int(row)) for row, col in np.argwhere(new_blocked)}
        return BeliefUpdate(frozenset(changed), frozenset(newly))

    def update_temperature_observations(self, observations, observation_time):
        """Replace observed cells with this scan's per-cell maximum Celsius."""
        now = float(observation_time)
        if not math.isfinite(now):
            raise ValueError("observation time must be finite")
        scan = {}
        for cell, temperature in observations:
            col, row = int(cell[0]), int(cell[1])
            value = float(temperature)
            if not math.isfinite(value) or not (
                0 <= col < self.geometry.width
                and 0 <= row < self.geometry.height
            ) or self.static_obstacle_map[row, col]:
                continue
            scan[col, row] = max(scan.get((col, row), -math.inf), value)
        old_blocked = self.blocked_mask.copy()
        changed = set()
        for (col, row), value in scan.items():
            if (
                not self.temperature_observed_mask[row, col]
                or not math.isclose(
                    self.temperature_belief_map[row, col], value,
                    rel_tol=1e-9, abs_tol=1e-12,
                )
                or self.last_observed_time_map[row, col] != now
            ):
                changed.add((col, row))
            self.temperature_belief_map[row, col] = value
            self.temperature_observed_mask[row, col] = True
            self.last_observed_time_map[row, col] = now
        return self._finish(changed, old_blocked)

    def update_co_observation(self, robot_x, robot_y, measured_ppm, time):
        if not self.config.co_enabled:
            return BeliefUpdate(frozenset(), frozenset())
        center = self.geometry.world_to_grid(robot_x, robot_y)
        value, now = float(measured_ppm), float(time)
        if center is None or not math.isfinite(value) or not math.isfinite(now):
            return BeliefUpdate(frozenset(), frozenset())
        old_blocked = self.blocked_mask.copy()
        radius = int(math.ceil(
            self.config.gas_update_radius_m / self.geometry.resolution
        ))
        changed = set()
        for row in range(center[1] - radius, center[1] + radius + 1):
            for col in range(center[0] - radius, center[0] + radius + 1):
                if not (0 <= col < self.geometry.width and 0 <= row < self.geometry.height):
                    continue
                distance = math.hypot(col - center[0], row - center[1]) * self.geometry.resolution
                if distance > self.config.gas_update_radius_m + 1e-9:
                    continue
                if self.static_obstacle_map[row, col]:
                    continue
                confidence = math.exp(-distance ** 2 / (
                    2.0 * self.config.gas_gaussian_sigma_m ** 2
                ))
                prior = self.co_belief_map[row, col]
                if not self.co_observed_mask[row, col] or not math.isclose(prior, value):
                    changed.add((col, row))
                self.co_belief_map[row, col] = value
                self.co_confidence_map[row, col] = confidence
                self.co_observed_mask[row, col] = True
                self.last_observed_time_map[row, col] = now
        return self._finish(changed, old_blocked)

    def update_dynamic_obstacles(self, obstacle_map, *, already_inflated=True, inflation_radius_m=0.0):
        raw = np.asarray(obstacle_map, dtype=bool)
        if raw.shape != self.shape:
            raise ValueError("dynamic obstacle geometry differs from belief")
        inflated = raw.copy()
        if not already_inflated:
            radius = int(math.ceil(inflation_radius_m / self.geometry.resolution))
            for row, col in np.argwhere(raw):
                for dy in range(-radius, radius + 1):
                    for dx in range(-radius, radius + 1):
                        if math.hypot(dx, dy) * self.geometry.resolution > inflation_radius_m + 1e-12:
                            continue
                        yy, xx = int(row + dy), int(col + dx)
                        if 0 <= yy < self.shape[0] and 0 <= xx < self.shape[1]:
                            inflated[yy, xx] = True
        differences = (raw != self.dynamic_obstacle_map) | (
            inflated != self.dynamic_inflated_obstacle_map
        )
        if not np.any(differences):
            return BeliefUpdate(frozenset(), frozenset())
        old_blocked = self.blocked_mask.copy()
        self.dynamic_obstacle_map = raw.copy()
        self.dynamic_inflated_obstacle_map = inflated
        changed = {(int(col), int(row)) for row, col in np.argwhere(differences)}
        return self._finish(changed, old_blocked)

    def update_estimated_fire_probability(self, probability_map, *, cost_weight, minimum_probability):
        values = np.asarray(probability_map, dtype=float)
        if values.shape != self.shape or not np.all(np.isfinite(values)):
            raise ValueError("invalid fire probability map")
        if np.any((values < 0.0) | (values > 1.0)):
            raise ValueError("fire probabilities must be in [0,1]")
        new_cost = np.where(values >= minimum_probability, values * cost_weight, 0.0)
        differences = ~np.isclose(new_cost, self.estimated_fire_cost_map)
        old_blocked = self.blocked_mask.copy()
        self.estimated_fire_cost_map = new_cost
        changed = {(int(col), int(row)) for row, col in np.argwhere(differences)}
        return self._finish(changed, old_blocked)

    def advance_time(self, time):
        now = float(time)
        if not math.isfinite(now) or now < self.current_time - 1e-12:
            raise ValueError("time must be finite and monotonic")
        old_cost = self.stale_observation_cost_map.copy()
        self.current_time = now
        self.recalculate()
        differences = ~np.isclose(old_cost, self.stale_observation_cost_map)
        changed = {(int(col), int(row)) for row, col in np.argwhere(differences)}
        if changed:
            self.revision += 1
        return BeliefUpdate(frozenset(changed), frozenset())

    def recalculate(self):
        cfg = self.config
        self.observed_mask = self.temperature_observed_mask | self.co_observed_mask
        temp = np.zeros(self.shape)
        temp[self.temperature_observed_mask] = np.clip(
            (self.temperature_belief_map[self.temperature_observed_mask] - cfg.temperature_safe_c)
            / (cfg.temperature_blocked_c - cfg.temperature_safe_c), 0.0, 1.0,
        )
        self.temperature_cost_map = cfg.temperature_weight * temp ** cfg.temperature_power
        gas_safe = cfg.gas_safe_threshold
        gas_span = cfg.gas_blocked_threshold - gas_safe
        co = np.zeros(self.shape)
        co[self.co_observed_mask] = np.clip(
            (self.co_belief_map[self.co_observed_mask] - gas_safe) / gas_span,
            0.0, 1.0,
        )
        self.co_cost_map = cfg.co_weight * co ** cfg.co_power
        neither = ~self.temperature_observed_mask & ~self.co_observed_mask
        self.unknown_cost_map = np.zeros(self.shape)
        self.unknown_cost_map[neither] = cfg.unknown_penalty
        partial = ~neither
        self.unknown_cost_map[partial & ~self.temperature_observed_mask] += cfg.unobserved_temperature_penalty
        self.unknown_cost_map[partial & ~self.co_observed_mask] += cfg.unobserved_co_penalty
        eligible = np.zeros(self.shape, dtype=bool)
        if cfg.stale_apply_to_temperature:
            eligible |= self.temperature_observed_mask
        if cfg.stale_apply_to_co:
            eligible |= self.co_observed_mask
        self.stale_observation_cost_map = np.zeros(self.shape)
        if cfg.stale_enabled:
            valid = eligible & np.isfinite(self.last_observed_time_map)
            age = np.zeros(self.shape)
            age[valid] = np.maximum(0.0, self.current_time - self.last_observed_time_map[valid])
            stale_age = np.maximum(0.0, age - cfg.stale_grace_period_s)
            self.stale_observation_cost_map[valid] = np.minimum(
                cfg.stale_maximum_cost, stale_age[valid] * cfg.stale_cost_per_second
            )
        temperature_blocked = self.temperature_observed_mask & (
            self.temperature_belief_map >= cfg.temperature_blocked_c
        )
        co_blocked = self.co_observed_mask & (
            self.co_belief_map >= cfg.gas_blocked_threshold
        )
        self.blocked_mask = (
            self.static_obstacle_map | self.dynamic_inflated_obstacle_map
            | temperature_blocked | co_blocked
        )
        self.final_cost_map = (
            cfg.base_cost + self.temperature_cost_map + self.co_cost_map
            + self.unknown_cost_map + self.estimated_fire_cost_map
            + self.stale_observation_cost_map
        )
        self.final_cost_map[self.blocked_mask] = math.inf
