"""Robust, frame-rate-independent filtering for a single C4001 target.

The C4001 may alternate between unrelated reflectors, so a plain moving
average can produce a believable but physically impossible distance halfway
between them.  This filter instead locks to one coherent range cluster,
rejects gated/Hampel outliers, and only changes target after a new cluster has
remained coherent for a configured amount of time.

``speed_mps`` combines robust range-rate with repeated plausible Doppler
activity from an already range-locked target. Isolated or implausibly large
Doppler reports cannot declare motion, while hand/arm movement can.

The module has no ROS or serial dependency and uses caller-supplied monotonic
timestamps, which makes it deterministic and straightforward to unit test.
"""

from collections import deque
from dataclasses import dataclass
import math
from statistics import median
from typing import Deque, Iterable, Optional, Sequence, Tuple


NO_TARGET = 'NO_TARGET'
ACQUIRING = 'ACQUIRING'
TRACKING = 'TRACKING'
HOLDING = 'HOLDING'


@dataclass(frozen=True)
class SignalFilterConfig:
    """Tuning parameters for one human-scale target track."""

    min_distance_m: float = 1.2
    max_distance_m: float = 12.0

    # Initial/replacement targets must form a compact range cluster.  Both a
    # hit count, time span, and dominant support are required, so scattered
    # room reflections cannot become a person merely by repeating briefly.
    acquire_radius_m: float = 0.30
    acquire_min_hits: int = 5
    acquire_min_support_ratio: float = 0.55
    acquire_confirm_sec: float = 0.50
    relock_confirm_sec: float = 2.00
    candidate_window_sec: float = 3.00

    # Keep a confirmed target through short missing/rejected reports.
    # Bridge short UART dropouts without showing a departed person for
    # several seconds. This is separate from the MOVING display hold.
    presence_hold_sec: float = 1.20

    # A frame must be physically reachable from the most recently accepted
    # range.  The gate widens only over a bounded time horizon.
    distance_gate_m: float = 0.25
    max_target_speed_mps: float = 2.2
    gate_horizon_sec: float = 0.35

    # A Hampel gate catches smaller single-frame jumps that still fit inside
    # the physical gate.  ``hampel_floor_m`` prevents normal quantisation from
    # making the threshold unrealistically narrow.
    hampel_window_sec: float = 1.00
    hampel_min_samples: int = 5
    hampel_sigma: float = 3.0
    hampel_floor_m: float = 0.08

    # Accepted ranges pass through a short time-window median and a
    # continuous-time EMA.  These values do not assume a particular FPS.
    median_window_sec: float = 0.45
    distance_ema_tau_sec: float = 0.45

    # Speed is a Theil-Sen range slope with time-based confirmation and
    # hysteresis.  It therefore remains zero for stationary range jitter even
    # if the sensor's raw Doppler field is noisy.
    velocity_window_sec: float = 1.50
    velocity_pair_min_dt_sec: float = 0.15
    velocity_min_span_sec: float = 0.60
    # SEN0610's specified velocity range begins at 0.1 m/s. Do not place the
    # software threshold above that lower bound: ordinary arm/hand motion can
    # leave the tracked body range almost unchanged while still producing a
    # small, repeated Doppler velocity.
    motion_start_mps: float = 0.10
    motion_stop_mps: float = 0.05
    motion_confirm_sec: float = 0.15
    motion_release_sec: float = 0.70

    # Plausible Doppler activity from an already range-locked target catches
    # hand/arm motion that barely changes the person's centre distance.
    doppler_window_sec: float = 0.70
    doppler_min_mps: float = 0.10
    doppler_max_mps: float = 2.20
    doppler_min_samples: int = 5
    doppler_min_active_ratio: float = 0.50
    doppler_min_span_sec: float = 0.30
    motion_activity_reference_mps: float = 1.00

    def validate(self) -> None:
        if not (0.0 <= self.min_distance_m < self.max_distance_m):
            raise ValueError('distance limits are invalid')
        if self.acquire_min_hits < 2:
            raise ValueError('acquire_min_hits must be at least two')
        if not 0.0 < self.acquire_min_support_ratio <= 1.0:
            raise ValueError('acquire_min_support_ratio must be in (0, 1]')
        positive = (
            self.acquire_radius_m,
            self.candidate_window_sec,
            self.presence_hold_sec,
            self.distance_gate_m,
            self.max_target_speed_mps,
            self.gate_horizon_sec,
            self.hampel_window_sec,
            self.hampel_sigma,
            self.hampel_floor_m,
            self.median_window_sec,
            self.velocity_window_sec,
            self.velocity_pair_min_dt_sec,
            self.velocity_min_span_sec,
            self.motion_start_mps,
            self.doppler_window_sec,
            self.doppler_min_mps,
            self.doppler_max_mps,
            self.doppler_min_span_sec,
            self.motion_activity_reference_mps,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError('filter windows and thresholds must be positive')
        non_negative = (
            self.acquire_confirm_sec,
            self.relock_confirm_sec,
            self.distance_ema_tau_sec,
            self.motion_stop_mps,
            self.motion_confirm_sec,
            self.motion_release_sec,
        )
        if any(value < 0.0 for value in non_negative):
            raise ValueError('filter durations must not be negative')
        if self.acquire_confirm_sec > self.candidate_window_sec:
            raise ValueError('acquire confirmation exceeds candidate window')
        if self.relock_confirm_sec > self.candidate_window_sec:
            raise ValueError('relock confirmation exceeds candidate window')
        if self.hampel_min_samples < 3:
            raise ValueError('hampel_min_samples must be at least three')
        if self.doppler_min_samples < 3:
            raise ValueError('doppler_min_samples must be at least three')
        if not 0.0 < self.doppler_min_active_ratio <= 1.0:
            raise ValueError('doppler_min_active_ratio must be in (0, 1]')
        if self.doppler_min_mps >= self.doppler_max_mps:
            raise ValueError('doppler_min_mps must be below doppler_max_mps')
        if self.motion_stop_mps >= self.motion_start_mps:
            raise ValueError('motion stop threshold must be below start threshold')
        if self.velocity_min_span_sec > self.velocity_window_sec:
            raise ValueError('velocity minimum span exceeds its window')


@dataclass(frozen=True)
class FilteredTarget:
    """One filtered output snapshot.

    ``distance_m`` and ``speed_mps`` are zero when no target is published.
    ``sample_accepted`` distinguishes a fresh tracked frame from a held value.
    ``reason`` is diagnostic only and is stable enough for tests/logging.
    """

    timestamp: float
    presence: bool
    distance_m: float
    speed_mps: float
    sample_accepted: bool
    tracking_state: str
    reason: str = ''
    activity_percent: float = 0.0


Candidate = Tuple[float, float]


def _finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(float(value))


def _prune(samples: Deque[Candidate], cutoff: float) -> None:
    while samples and samples[0][0] < cutoff:
        samples.popleft()


def _cluster_for_anchor(
    samples: Sequence[Candidate], anchor: float, radius_m: float
) -> Tuple[Candidate, ...]:
    members = tuple(item for item in samples if abs(item[1] - anchor) <= radius_m)
    if not members:
        return ()
    centre = float(median(item[1] for item in members))
    return tuple(item for item in members if abs(item[1] - centre) <= radius_m)


def _best_range_cluster(
    samples: Iterable[Candidate], radius_m: float
) -> Tuple[Candidate, ...]:
    """Return the densest compact 1-D cluster with deterministic tie breaks."""

    items = tuple(samples)
    best: Tuple[Candidate, ...] = ()
    best_key = (-1, float('-inf'), float('-inf'))
    for _, anchor in items:
        cluster = _cluster_for_anchor(items, anchor, radius_m)
        if not cluster:
            continue
        values = [item[1] for item in cluster]
        spread = max(values) - min(values)
        newest = max(item[0] for item in cluster)
        # Prefer: more supporting frames, tighter range, newer evidence.
        key = (len(cluster), -spread, newest)
        if key > best_key:
            best = cluster
            best_key = key
    return best


class SingleTargetSignalFilter:
    """Track and smooth one coherent range target at a time."""

    def __init__(self, config: Optional[SignalFilterConfig] = None) -> None:
        self.config = config or SignalFilterConfig()
        self.config.validate()
        self._last_update: Optional[float] = None
        self._candidates: Deque[Candidate] = deque()
        self._raw_ranges: Deque[Candidate] = deque()
        self._velocity_points: Deque[Candidate] = deque()
        self._doppler_points: Deque[Candidate] = deque()
        self._tracking = False
        self._filtered_distance: Optional[float] = None
        self._last_accepted_time: Optional[float] = None
        self._last_accepted_range: Optional[float] = None
        self._motion_active = False
        self._motion_evidence_since: Optional[float] = None
        self._quiet_since: Optional[float] = None
        self._filtered_speed = 0.0
        self._activity_percent = 0.0

    def reset(self) -> None:
        """Forget all candidates and the current target."""

        self._last_update = None
        self._candidates.clear()
        self._clear_track(clear_candidates=False)

    def update(
        self,
        *,
        timestamp: float,
        detected: bool,
        distance_m: Optional[float] = None,
        speed_mps: Optional[float] = None,
        energy: Optional[int] = None,
    ) -> FilteredTarget:
        """Consume one raw report and return the current filtered snapshot.

        ``timestamp`` must be monotonic. Distance owns target identity. Raw
        Doppler contributes only after that distance target is locked and its
        activity is plausible across several accepted frames.
        """

        del energy
        now = float(timestamp)
        if not math.isfinite(now):
            raise ValueError('timestamp must be finite')
        if self._last_update is not None and now < self._last_update:
            raise ValueError('timestamps must be monotonic')
        self._last_update = now

        valid = (
            bool(detected)
            and _finite(distance_m)
            and self.config.min_distance_m
            <= float(distance_m)
            <= self.config.max_distance_m
        )
        if not valid:
            keep_candidates = not bool(detected)
            if keep_candidates:
                _prune(
                    self._candidates,
                    now - self.config.candidate_window_sec,
                )
            return self._without_accepted_sample(
                now,
                reason='no_detection' if not detected else 'invalid_range',
                keep_candidates=keep_candidates,
            )

        distance = float(distance_m)
        if not self._tracking:
            if not self._candidate_speed_is_plausible(speed_mps):
                _prune(
                    self._candidates,
                    now - self.config.candidate_window_sec,
                )
                state = ACQUIRING if self._candidates else NO_TARGET
                return self._empty_snapshot(
                    now, state, 'candidate_speed_gate'
                )
            self._add_candidate(now, distance)
            cluster = self._confirmed_cluster(self.config.acquire_confirm_sec)
            if cluster:
                self._start_track(now, cluster)
                return self._snapshot(
                    now, accepted=True, state=TRACKING, reason='acquired'
                )
            return self._empty_snapshot(now, ACQUIRING, 'confirming_target')

        accepted, reason = self._matches_track(now, distance)
        if accepted:
            self._accept(now, distance, speed_mps)
            self._candidates.clear()
            return self._snapshot(now, accepted=True, state=TRACKING)

        # A rejected measurement is kept separately as possible evidence of a
        # real replacement target.  It can never drag the existing EMA toward
        # an in-between distance.
        if not self._candidate_speed_is_plausible(speed_mps):
            return self._without_accepted_sample(
                now, reason='candidate_speed_gate', keep_candidates=True
            )
        self._add_candidate(now, distance)
        assert self._last_accepted_time is not None
        old_track_age = now - self._last_accepted_time
        cluster = self._confirmed_cluster(self.config.relock_confirm_sec)
        if old_track_age >= self.config.relock_confirm_sec and cluster:
            self._start_track(now, cluster)
            return self._snapshot(
                now, accepted=True, state=TRACKING, reason='relocked'
            )
        return self._without_accepted_sample(
            now, reason=reason, keep_candidates=True
        )

    def advance(self, timestamp: float) -> FilteredTarget:
        """Advance dropout timers when no new UART report was received."""

        return self.update(timestamp=timestamp, detected=False)

    def _candidate_speed_is_plausible(
        self, raw_speed_mps: Optional[float]
    ) -> bool:
        # SPEED_MODE can report fast non-human reflections. They must not
        # establish or replace a person track, although an isolated bad speed
        # on an already locked range is harmless and ignored by Doppler logic.
        return (
            not _finite(raw_speed_mps)
            or abs(float(raw_speed_mps))
            <= self.config.max_target_speed_mps
        )

    def _add_candidate(self, now: float, distance: float) -> None:
        self._candidates.append((now, distance))
        _prune(
            self._candidates,
            now - self.config.candidate_window_sec,
        )

    def _confirmed_cluster(self, confirm_sec: float) -> Tuple[Candidate, ...]:
        cluster = _best_range_cluster(
            self._candidates, self.config.acquire_radius_m
        )
        if len(cluster) < self.config.acquire_min_hits:
            return ()
        support_ratio = len(cluster) / max(1, len(self._candidates))
        if support_ratio < self.config.acquire_min_support_ratio:
            return ()
        times = [item[0] for item in cluster]
        if max(times) - min(times) < confirm_sec:
            return ()
        return cluster

    def _start_track(self, now: float, cluster: Sequence[Candidate]) -> None:
        distance = float(median(item[1] for item in cluster))
        self._tracking = True
        self._filtered_distance = distance
        self._last_accepted_time = now
        self._last_accepted_range = distance
        self._raw_ranges.clear()
        self._raw_ranges.append((now, distance))
        self._velocity_points.clear()
        self._velocity_points.append((now, distance))
        self._doppler_points.clear()
        self._activity_percent = 0.0
        self._motion_active = False
        self._motion_evidence_since = None
        self._quiet_since = None
        self._filtered_speed = 0.0
        self._candidates.clear()

    def _matches_track(self, now: float, distance: float) -> Tuple[bool, str]:
        assert self._last_accepted_time is not None
        assert self._last_accepted_range is not None
        elapsed = max(0.0, now - self._last_accepted_time)
        gate = self.config.distance_gate_m + (
            self.config.max_target_speed_mps
            * min(elapsed, self.config.gate_horizon_sec)
        )
        if abs(distance - self._last_accepted_range) > gate:
            return False, 'range_gate'

        _prune(self._raw_ranges, now - self.config.hampel_window_sec)
        if len(self._raw_ranges) >= self.config.hampel_min_samples:
            values = [item[1] for item in self._raw_ranges]
            centre = float(median(values))
            mad = float(median(abs(value - centre) for value in values))
            robust_scale = max(
                self.config.hampel_floor_m,
                1.4826 * mad,
            )
            if abs(distance - centre) > self.config.hampel_sigma * robust_scale:
                return False, 'hampel_gate'
        return True, ''

    def _accept(
        self, now: float, distance: float, raw_speed_mps: Optional[float]
    ) -> None:
        assert self._last_accepted_time is not None
        assert self._filtered_distance is not None
        dt = max(0.0, now - self._last_accepted_time)

        self._raw_ranges.append((now, distance))
        _prune(self._raw_ranges, now - self.config.hampel_window_sec)
        recent = [
            value
            for stamp, value in self._raw_ranges
            if stamp >= now - self.config.median_window_sec
        ]
        median_range = float(median(recent))
        if self.config.distance_ema_tau_sec == 0.0:
            alpha = 1.0
        else:
            alpha = 1.0 - math.exp(
                -dt / self.config.distance_ema_tau_sec
            )
        self._filtered_distance += alpha * (
            median_range - self._filtered_distance
        )
        self._last_accepted_time = now
        self._last_accepted_range = distance

        self._velocity_points.append((now, self._filtered_distance))
        _prune(
            self._velocity_points,
            now - self.config.velocity_window_sec,
        )
        slope = self._robust_range_slope()
        doppler = self._robust_doppler_activity(now, raw_speed_mps)
        motion_value = slope if abs(slope) >= doppler else doppler
        self._update_motion_hysteresis(now, motion_value)

    def _robust_range_slope(self) -> float:
        points = tuple(self._velocity_points)
        if len(points) < 3:
            return 0.0
        if points[-1][0] - points[0][0] < self.config.velocity_min_span_sec:
            return 0.0
        slopes = []
        for index, (t0, d0) in enumerate(points[:-1]):
            for t1, d1 in points[index + 1:]:
                dt = t1 - t0
                if dt >= self.config.velocity_pair_min_dt_sec:
                    slopes.append((d1 - d0) / dt)
        if not slopes:
            return 0.0
        result = float(median(slopes))
        return max(
            -self.config.max_target_speed_mps,
            min(self.config.max_target_speed_mps, result),
        )

    def _robust_doppler_activity(
        self, now: float, raw_speed_mps: Optional[float]
    ) -> float:
        magnitude = 0.0
        if _finite(raw_speed_mps):
            candidate = abs(float(raw_speed_mps))
            if candidate <= self.config.doppler_max_mps:
                magnitude = candidate
        self._doppler_points.append((now, magnitude))
        _prune(
            self._doppler_points,
            now - self.config.doppler_window_sec,
        )
        points = tuple(self._doppler_points)
        if len(points) < self.config.doppler_min_samples:
            return 0.0
        if points[-1][0] - points[0][0] < self.config.doppler_min_span_sec:
            return 0.0
        active = [
            value
            for _, value in points
            if value >= self.config.doppler_min_mps
        ]
        if len(active) / len(points) < self.config.doppler_min_active_ratio:
            return 0.0
        return float(median(active))

    def _update_motion_hysteresis(
        self, now: float, motion_value: float
    ) -> None:
        magnitude = abs(motion_value)
        if not self._motion_active:
            self._quiet_since = None
            if magnitude >= self.config.motion_start_mps:
                if self._motion_evidence_since is None:
                    self._motion_evidence_since = now
                if (
                    now - self._motion_evidence_since
                    >= self.config.motion_confirm_sec
                ):
                    self._motion_active = True
            else:
                self._motion_evidence_since = None
        else:
            self._motion_evidence_since = None
            if magnitude <= self.config.motion_stop_mps:
                if self._quiet_since is None:
                    self._quiet_since = now
                if now - self._quiet_since >= self.config.motion_release_sec:
                    self._motion_active = False
                    self._quiet_since = None
            else:
                self._quiet_since = None
        self._filtered_speed = motion_value if self._motion_active else 0.0
        if magnitude >= self.config.doppler_min_mps:
            self._activity_percent = min(
                100.0,
                100.0 * magnitude / self.config.motion_activity_reference_mps,
            )
        else:
            self._activity_percent = 0.0

    def _without_accepted_sample(
        self, now: float, *, reason: str, keep_candidates: bool
    ) -> FilteredTarget:
        if not keep_candidates:
            self._candidates.clear()
        if self._tracking and self._last_accepted_time is not None:
            if now - self._last_accepted_time <= self.config.presence_hold_sec:
                # Never carry a questionable speed through a missing/outlier
                # frame.  The higher-level mobility classifier can apply its
                # own short motion hold to genuine preceding movement.
                return self._snapshot(
                    now,
                    accepted=False,
                    state=HOLDING,
                    reason=reason,
                    speed_override=0.0,
                    activity_override=0.0,
                )
        had_candidates = bool(self._candidates)
        self._clear_track(clear_candidates=not keep_candidates)
        if had_candidates and keep_candidates:
            return self._empty_snapshot(now, ACQUIRING, reason)
        return self._empty_snapshot(now, NO_TARGET, reason)

    def _clear_track(self, *, clear_candidates: bool) -> None:
        self._tracking = False
        self._filtered_distance = None
        self._last_accepted_time = None
        self._last_accepted_range = None
        self._raw_ranges.clear()
        self._velocity_points.clear()
        self._doppler_points.clear()
        self._activity_percent = 0.0
        self._motion_active = False
        self._motion_evidence_since = None
        self._quiet_since = None
        self._filtered_speed = 0.0
        if clear_candidates:
            self._candidates.clear()

    def _snapshot(
        self,
        now: float,
        *,
        accepted: bool,
        state: str,
        reason: str = '',
        speed_override: Optional[float] = None,
        activity_override: Optional[float] = None,
    ) -> FilteredTarget:
        assert self._filtered_distance is not None
        speed = self._filtered_speed if speed_override is None else speed_override
        activity = (
            self._activity_percent
            if activity_override is None else activity_override
        )
        return FilteredTarget(
            timestamp=now,
            presence=True,
            distance_m=self._filtered_distance,
            speed_mps=float(speed),
            sample_accepted=accepted,
            tracking_state=state,
            reason=reason,
            activity_percent=float(activity),
        )

    @staticmethod
    def _empty_snapshot(now: float, state: str, reason: str) -> FilteredTarget:
        return FilteredTarget(
            timestamp=now,
            presence=False,
            distance_m=0.0,
            speed_mps=0.0,
            sample_accepted=False,
            tracking_state=state,
            reason=reason,
        )
