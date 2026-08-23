"""Tuned, testable human-presence gate for C4001 speed-mode samples.

This is an experimental range/energy heuristic, not an AI classifier.  Raw
sensor values remain untouched; only the dedicated calibrated distance and
human-presence outputs use these settings.
"""

from dataclasses import dataclass
import math
from typing import Optional

from .c4001_protocol import C4001Measurement


@dataclass(frozen=True)
class HumanDetectionConfig:
    calibration_scale: float = 1.0
    calibration_offset_m: float = -0.1
    range_min_m: float = 0.6
    range_max_m: float = 6.0
    energy_threshold: int = 3000
    confirm_frames: int = 3
    clear_frames: int = 6

    def validate(self) -> None:
        if not math.isfinite(self.calibration_scale) or self.calibration_scale <= 0:
            raise ValueError('calibration_scale must be finite and positive')
        if not math.isfinite(self.calibration_offset_m):
            raise ValueError('calibration_offset_m must be finite')
        if (
            not math.isfinite(self.range_min_m)
            or not math.isfinite(self.range_max_m)
            or self.range_min_m < 0.0
            or self.range_min_m >= self.range_max_m
        ):
            raise ValueError('human range must satisfy 0 <= min < max')
        if isinstance(self.energy_threshold, bool) or self.energy_threshold < 0:
            raise ValueError('human_energy_threshold must be non-negative')
        if isinstance(self.confirm_frames, bool) or self.confirm_frames <= 0:
            raise ValueError('human_confirm_frames must be positive')
        if isinstance(self.clear_frames, bool) or self.clear_frames <= 0:
            raise ValueError('human_clear_frames must be positive')


@dataclass(frozen=True)
class HumanDetection:
    presence: bool
    calibrated_distance_m: Optional[float]
    sample_matches: bool


class HumanPresenceDetector:
    """Debounce calibrated range plus energy into a stable presence flag."""

    def __init__(self, config: HumanDetectionConfig) -> None:
        config.validate()
        self.config = config
        self.presence = False
        self._confirm_count = 0
        self._clear_count = 0

    def calibrate_distance(self, raw_distance_m: float) -> float:
        return (
            float(raw_distance_m) * self.config.calibration_scale
            + self.config.calibration_offset_m
        )

    def update(self, measurement: C4001Measurement) -> HumanDetection:
        calibrated: Optional[float] = None
        matches = False
        if (
            measurement.detected
            and measurement.distance_m is not None
            and measurement.energy is not None
            and math.isfinite(float(measurement.distance_m))
            and float(measurement.distance_m) > 0.0
        ):
            calibrated = self.calibrate_distance(measurement.distance_m)
            matches = bool(
                self.config.range_min_m
                <= calibrated
                <= self.config.range_max_m
                and measurement.energy >= self.config.energy_threshold
            )

        if matches:
            self._confirm_count += 1
            self._clear_count = 0
            if self._confirm_count >= self.config.confirm_frames:
                self.presence = True
        else:
            self._confirm_count = 0
            self._clear_count += 1
            if self._clear_count >= self.config.clear_frames:
                self.presence = False

        return HumanDetection(self.presence, calibrated, matches)

    def reset(self) -> HumanDetection:
        self.presence = False
        self._confirm_count = 0
        self._clear_count = 0
        return HumanDetection(False, None, False)
