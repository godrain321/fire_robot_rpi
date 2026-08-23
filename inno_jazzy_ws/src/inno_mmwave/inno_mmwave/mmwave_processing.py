"""Pure distance processing and human-candidate helpers for C4001 tuning.

This module never changes the raw sensor measurement.  A valid sample passes
through outlier rejection, calibration and the selected filter in that order.
It intentionally has no ROS, serial or Tk dependency so it can be tested on a
development PC without the radar attached.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
import statistics
from typing import Deque, Optional


class FilterType(str, Enum):
    NONE = "None"
    MEDIAN = "Median"
    MOVING_AVERAGE = "Moving Average"
    EMA = "EMA"


@dataclass(frozen=True)
class ProcessingSettings:
    scale: float = 1.0
    offset_m: float = -0.1
    filter_type: FilterType = FilterType.NONE
    filter_size: int = 5
    ema_alpha: float = 0.30
    outlier_threshold_m: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.scale) or self.scale <= 0.0:
            raise ValueError("Scale must be finite and greater than zero")
        if not math.isfinite(self.offset_m):
            raise ValueError("Offset must be finite")
        if not isinstance(self.filter_type, FilterType):
            raise ValueError("Invalid filter type")
        if isinstance(self.filter_size, bool) or self.filter_size <= 0:
            raise ValueError("Filter size must be greater than zero")
        if not math.isfinite(self.ema_alpha) or not 0.0 < self.ema_alpha <= 1.0:
            raise ValueError("EMA alpha must satisfy 0 < alpha <= 1")
        if (
            not math.isfinite(self.outlier_threshold_m)
            or self.outlier_threshold_m < 0.0
        ):
            raise ValueError("Outlier threshold must be zero or greater")


@dataclass(frozen=True)
class ProcessedRange:
    raw_range_m: Optional[float]
    calibrated_range_m: Optional[float]
    filtered_range_m: Optional[float]
    accepted: bool
    rejection_reason: str = ""


class DistanceProcessor:
    """Stateful, explicitly staged distance processing pipeline."""

    def __init__(self, settings: ProcessingSettings = ProcessingSettings()) -> None:
        self.settings = settings
        self._buffer: Deque[float] = deque(maxlen=settings.filter_size)
        self._last_accepted_raw_m: Optional[float] = None
        self._ema_value: Optional[float] = None

    @staticmethod
    def validate_raw_range(raw_range_m: object) -> Optional[float]:
        try:
            value = float(raw_range_m)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value) or value <= 0.0:
            return None
        return value

    def reject_outlier(self, raw_range_m: float) -> bool:
        threshold = self.settings.outlier_threshold_m
        return bool(
            threshold > 0.0
            and self._last_accepted_raw_m is not None
            and abs(raw_range_m - self._last_accepted_raw_m) > threshold
        )

    def calibrate_range(self, raw_range_m: float) -> float:
        return raw_range_m * self.settings.scale + self.settings.offset_m

    def filter_range(self, calibrated_range_m: float) -> float:
        filter_type = self.settings.filter_type
        if filter_type is FilterType.NONE:
            return calibrated_range_m
        if filter_type is FilterType.EMA:
            if self._ema_value is None:
                self._ema_value = calibrated_range_m
            else:
                alpha = self.settings.ema_alpha
                self._ema_value = (
                    alpha * calibrated_range_m + (1.0 - alpha) * self._ema_value
                )
            return self._ema_value

        self._buffer.append(calibrated_range_m)
        if filter_type is FilterType.MEDIAN:
            return float(statistics.median(self._buffer))
        return float(statistics.fmean(self._buffer))

    def process(self, raw_range_m: object, *, target_valid: bool) -> ProcessedRange:
        if not target_valid:
            return ProcessedRange(None, None, None, False, "NO TARGET")
        raw = self.validate_raw_range(raw_range_m)
        if raw is None:
            return ProcessedRange(None, None, None, False, "INVALID RANGE")
        if self.reject_outlier(raw):
            return ProcessedRange(raw, None, None, False, "OUTLIER")

        self._last_accepted_raw_m = raw
        calibrated = self.calibrate_range(raw)
        filtered = self.filter_range(calibrated)
        return ProcessedRange(raw, calibrated, filtered, True)

    def apply_settings(self, settings: ProcessingSettings) -> None:
        reset_filter = (
            settings.filter_type != self.settings.filter_type
            or settings.filter_size != self.settings.filter_size
            or settings.ema_alpha != self.settings.ema_alpha
            or settings.scale != self.settings.scale
            or settings.offset_m != self.settings.offset_m
        )
        self.settings = settings
        if reset_filter:
            self.clear_filter_buffer()

    def clear_filter_buffer(self) -> None:
        self._buffer = deque(maxlen=self.settings.filter_size)
        self._ema_value = None

    def reset(self) -> None:
        self.settings = ProcessingSettings()
        self._last_accepted_raw_m = None
        self.clear_filter_buffer()


@dataclass(frozen=True)
class HumanTuningSettings:
    range_min_m: float = 0.6
    range_max_m: float = 6.0
    energy_threshold: int = 3000
    confirm_frames: int = 3
    clear_frames: int = 6

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.range_min_m)
            or not math.isfinite(self.range_max_m)
            or self.range_min_m < 0.0
            or self.range_min_m >= self.range_max_m
        ):
            raise ValueError("Human range must satisfy 0 <= min < max")
        if isinstance(self.energy_threshold, bool) or self.energy_threshold < 0:
            raise ValueError("Energy threshold must be zero or greater")
        if isinstance(self.confirm_frames, bool) or self.confirm_frames <= 0:
            raise ValueError("Confirm frames must be greater than zero")
        if isinstance(self.clear_frames, bool) or self.clear_frames <= 0:
            raise ValueError("Clear frames must be greater than zero")


class HumanCandidateDetector:
    """Simple experiment heuristic; this is not a human-classification AI."""

    def __init__(
        self, settings: HumanTuningSettings = HumanTuningSettings()
    ) -> None:
        self.settings = settings
        self.is_candidate = False
        self._matching_frames = 0
        self._clear_frames = 0

    def update(
        self,
        *,
        target_count: int,
        filtered_range_m: Optional[float],
        energy: Optional[int],
    ) -> bool:
        matches = bool(
            target_count > 0
            and filtered_range_m is not None
            and self.settings.range_min_m
            <= filtered_range_m
            <= self.settings.range_max_m
            and energy is not None
            and energy >= self.settings.energy_threshold
        )
        if matches:
            self._matching_frames += 1
            self._clear_frames = 0
            if self._matching_frames >= self.settings.confirm_frames:
                self.is_candidate = True
        else:
            self._matching_frames = 0
            self._clear_frames += 1
            if self._clear_frames >= self.settings.clear_frames:
                self.is_candidate = False
        return self.is_candidate

    def apply_settings(self, settings: HumanTuningSettings) -> None:
        self.settings = settings
        self.reset()

    def reset(self) -> None:
        self.is_candidate = False
        self._matching_frames = 0
        self._clear_frames = 0
