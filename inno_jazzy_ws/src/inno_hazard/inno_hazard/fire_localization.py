"""Ground-Truth-free fire localization core ported from factory_v5."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np


class FireEstimateState(Enum):
    UNOBSERVED = "unobserved"
    WEAK_EVIDENCE = "weak_evidence"
    POSSIBLE_FIRE = "possible_fire"
    LIKELY_FIRE = "likely_fire"
    CONFIRMED_FIRE_REGION = "confirmed_fire_region"


@dataclass(frozen=True)
class FireLocalizationConfig:
    enabled: bool = False
    prior_probability: float = 0.05
    possible_probability_threshold: float = 0.40
    likely_probability_threshold: float = 0.65
    confirm_probability_threshold: float = 0.80
    release_probability_threshold: float = 0.70
    thermal_warning_threshold_c: float = 40.0
    thermal_strong_threshold_c: float = 60.0
    thermal_evidence_weight: float = 1.0
    co_rise_threshold_ppm: float = 5.0
    temperature_rise_threshold_c: float = 2.0
    minimum_robot_motion_m: float = 0.2
    co_gradient_evidence_weight: float = 0.4
    minimum_consecutive_co_rises: int = 2
    maximum_observation_interval_s: float = 2.0
    minimum_distinct_observation_poses: int = 2
    minimum_confirmation_observations: int = 3
    distinct_pose_distance_m: float = 0.5
    distinct_heading_deg: float = 15.0
    maximum_evidence_per_update: float = 1.0
    maximum_accumulated_evidence: float = 8.0
    evidence_decay_per_second: float = 0.001
    candidate_probability_threshold: float = 0.50
    maximum_confirmed_region_cells: int = 250
    estimated_fire_cost_weight: float = 50.0
    co_projection_distance_m: float = 4.0
    co_projection_half_angle_deg: float = 15.0

    def __post_init__(self):
        probabilities = (
            self.prior_probability, self.possible_probability_threshold,
            self.likely_probability_threshold,
            self.confirm_probability_threshold,
            self.release_probability_threshold,
            self.candidate_probability_threshold,
        )
        if any(not 0.0 <= value <= 1.0 for value in probabilities):
            raise ValueError("fire probabilities must be in [0,1]")
        if not (
            self.prior_probability < self.possible_probability_threshold
            < self.likely_probability_threshold
            < self.confirm_probability_threshold
        ):
            raise ValueError("fire probability thresholds are invalid")
        if self.thermal_strong_threshold_c <= self.thermal_warning_threshold_c:
            raise ValueError("thermal strong threshold must exceed warning")


@dataclass(frozen=True)
class ThermalRay:
    pixel_temperature: float
    ray_cells: tuple[tuple[int, int], ...]
    valid: bool = True


@dataclass(frozen=True)
class FireLocalizationResult:
    state: FireEstimateState
    highest_probability: float
    highest_probability_grid: tuple[int, int] | None
    confidence: float
    valid_observation_count: int
    distinct_pose_count: int
    confirmed: bool


class FireLocalizer:
    """Fuse bounded thermal-ray and motion-conditioned CO evidence."""

    def __init__(self, geometry, static_obstacle_map, config=None):
        self.geometry = geometry
        self.config = config or FireLocalizationConfig()
        shape = geometry.height, geometry.width
        static = np.asarray(static_obstacle_map, dtype=bool)
        if static.shape != shape:
            raise ValueError("static obstacle geometry differs from localizer")
        self.static_obstacle_map = static.copy()
        self.thermal_fire_evidence = np.zeros(shape)
        self.co_gradient_evidence = np.zeros(shape)
        self.temporal_consistency_evidence = np.zeros(shape)
        self.combined_fire_evidence = np.zeros(shape)
        self.fire_probability = np.full(shape, self.config.prior_probability)
        self.observation_count_map = np.zeros(shape, dtype=np.int32)
        self._thermal_count_map = np.zeros(shape, dtype=np.int32)
        self._co_count_map = np.zeros(shape, dtype=np.int32)
        self._pose_keys_by_cell = {}
        self._ids = set()
        self._last_time = None
        self._last_co = None
        self._co_rise_streak = 0
        self._confirmed = False
        self.latest_result = self.result()

    def _prepare(self, observation_id, now):
        if not observation_id or observation_id in self._ids:
            raise ValueError("observation ID is empty or duplicate")
        now = float(now)
        if not math.isfinite(now) or (
            self._last_time is not None and now < self._last_time - 1e-12
        ):
            raise ValueError("observation time must be finite and monotonic")
        dt = 0.0 if self._last_time is None else now - self._last_time
        factor = math.exp(-self.config.evidence_decay_per_second * dt)
        self.thermal_fire_evidence *= factor
        self.co_gradient_evidence *= factor
        self.temporal_consistency_evidence *= factor
        self._last_time = now
        self._ids.add(observation_id)
        return now

    def _pose_key(self, pose):
        return (
            round(pose[0] / self.config.distinct_pose_distance_m),
            round(pose[1] / self.config.distinct_pose_distance_m),
            round(math.degrees(pose[2]) / max(self.config.distinct_heading_deg, 1e-6)),
        )

    def _temperature_strength(self, value):
        if value < self.config.thermal_warning_threshold_c:
            return 0.0
        normalized = (value - self.config.thermal_warning_threshold_c) / (
            self.config.thermal_strong_threshold_c
            - self.config.thermal_warning_threshold_c
        )
        return self.config.thermal_evidence_weight * min(
            2.0, 0.25 + 0.75 * max(0.0, normalized)
        )

    def add_thermal_observation(self, observation_id, time, robot_pose, rays):
        self._prepare(observation_id, time)
        pose_key = self._pose_key(robot_pose)
        frame_delta = {}
        for ray in rays:
            value = float(ray.pixel_temperature)
            strength = self._temperature_strength(value) if ray.valid and math.isfinite(value) else 0.0
            cells = []
            for col, row in ray.ray_cells:
                cell = int(col), int(row)
                if not (0 <= cell[0] < self.geometry.width and 0 <= cell[1] < self.geometry.height):
                    continue
                if self.static_obstacle_map[cell[1], cell[0]]:
                    break
                if not cells or cells[-1] != cell:
                    cells.append(cell)
            if strength <= 0.0 or not cells:
                continue
            weights = np.linspace(0.5, 1.0, len(cells))
            weights /= weights.sum()
            for cell, weight in zip(cells, weights):
                frame_delta[cell] = frame_delta.get(cell, 0.0) + strength * weight
        for (col, row), value in frame_delta.items():
            delta = min(self.config.maximum_evidence_per_update, value)
            if self.observation_count_map[row, col]:
                self.temporal_consistency_evidence[row, col] += min(
                    0.25 * delta, self.config.maximum_evidence_per_update
                )
            self.thermal_fire_evidence[row, col] += delta
            self.observation_count_map[row, col] += 1
            self._thermal_count_map[row, col] += 1
            self._pose_keys_by_cell.setdefault((col, row), set()).add(pose_key)
        self._recalculate()
        return self.latest_result

    def add_co_observation(self, observation_id, time, robot_pose, co_ppm, local_temperature_c, *, thermal_direction_supported=False):
        now = self._prepare(observation_id, time)
        pose = tuple(map(float, robot_pose))
        co, temperature = float(co_ppm), float(local_temperature_c)
        if not math.isfinite(co) or not math.isfinite(temperature):
            raise ValueError("CO and temperature must be finite")
        previous = self._last_co
        if previous is not None:
            old_time, old_pose, old_co, old_temperature = previous
            dx, dy = pose[0] - old_pose[0], pose[1] - old_pose[1]
            distance = math.hypot(dx, dy)
            qualifies = (
                0.0 < now - old_time <= self.config.maximum_observation_interval_s
                and distance >= self.config.minimum_robot_motion_m
                and co - old_co >= self.config.co_rise_threshold_ppm
            )
            self._co_rise_streak = self._co_rise_streak + 1 if qualifies else 0
            if qualifies and self._co_rise_streak >= self.config.minimum_consecutive_co_rises:
                supported = (
                    temperature - old_temperature >= self.config.temperature_rise_threshold_c
                    or thermal_direction_supported
                )
                normalized = min(2.0, (co - old_co) / distance / self.config.co_rise_threshold_ppm)
                strength = min(
                    self.config.maximum_evidence_per_update,
                    self.config.co_gradient_evidence_weight * normalized
                    * (1.0 if supported else 0.35),
                )
                self._project(pose, (dx / distance, dy / distance), strength)
        self._last_co = now, pose, co, temperature
        self._recalculate()
        return self.latest_result

    def _project(self, pose, direction, strength):
        pose_key = self._pose_key(pose)
        base = math.atan2(direction[1], direction[0])
        visited = set()
        step = self.geometry.resolution * 0.5
        for offset in (-self.config.co_projection_half_angle_deg, 0.0, self.config.co_projection_half_angle_deg):
            theta = base + math.radians(offset)
            for distance in np.arange(step, self.config.co_projection_distance_m + step, step):
                cell = self.geometry.world_to_grid(
                    pose[0] + distance * math.cos(theta),
                    pose[1] + distance * math.sin(theta),
                )
                if cell is None:
                    break
                col, row = cell
                if self.static_obstacle_map[row, col]:
                    break
                if cell in visited:
                    continue
                attenuation = max(0.2, 1.0 - distance / self.config.co_projection_distance_m)
                self.co_gradient_evidence[row, col] += strength * attenuation
                self.observation_count_map[row, col] += 1
                self._co_count_map[row, col] += 1
                self._pose_keys_by_cell.setdefault(cell, set()).add(pose_key)
                visited.add(cell)

    def _recalculate(self):
        overlap = np.minimum(self.thermal_fire_evidence, self.co_gradient_evidence)
        self.combined_fire_evidence = np.clip(
            self.thermal_fire_evidence + self.co_gradient_evidence
            + self.temporal_consistency_evidence + 0.5 * overlap,
            0.0, self.config.maximum_accumulated_evidence,
        )
        prior = self.config.prior_probability
        logits = math.log(prior / (1.0 - prior)) + self.combined_fire_evidence
        self.fire_probability = np.clip(1.0 / (1.0 + np.exp(-logits)), 0.0, 1.0)
        self.latest_result = self.result()

    def result(self):
        if not np.any(self.observation_count_map):
            return FireLocalizationResult(
                FireEstimateState.UNOBSERVED, self.config.prior_probability,
                None, 0.0, 0, 0, False,
            )
        row, col = np.unravel_index(np.argmax(self.fire_probability), self.fire_probability.shape)
        highest = float(self.fire_probability[row, col])
        count = int(self.observation_count_map[row, col])
        poses = len(self._pose_keys_by_cell.get((int(col), int(row)), ()))
        supported = self._co_count_map[row, col] > 0 or self.temporal_consistency_evidence[row, col] > 0
        ready = (
            highest >= self.config.confirm_probability_threshold
            and count >= self.config.minimum_confirmation_observations
            and poses >= self.config.minimum_distinct_observation_poses
            and self._thermal_count_map[row, col] > 0 and supported
        )
        self._confirmed = (
            highest >= self.config.release_probability_threshold
            if self._confirmed else ready
        )
        if self._confirmed:
            state = FireEstimateState.CONFIRMED_FIRE_REGION
        elif highest >= self.config.likely_probability_threshold:
            state = FireEstimateState.LIKELY_FIRE
        elif highest >= self.config.possible_probability_threshold:
            state = FireEstimateState.POSSIBLE_FIRE
        else:
            state = FireEstimateState.WEAK_EVIDENCE
        return FireLocalizationResult(
            state, highest, (int(col), int(row)), highest,
            count, poses, self._confirmed,
        )
