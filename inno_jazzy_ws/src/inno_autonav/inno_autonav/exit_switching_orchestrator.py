"""ROS-independent Stage 7 orchestration: "should we abandon the current exit?".

``ExitSwitchingCore`` answers exactly one question: given Stage 6's current state
and the observed route trend, should a *different* exit be requested? It never
evaluates exits itself (no HTTP/service calls, no A*, no rclpy) -- it only ever
produces a :class:`PeekRequest` (ask the node to cheaply rank candidates without
committing to anything) or a :class:`SwitchRequest` (ask
``EvacuationManagerNode`` to actually select-and-activate a replacement). It never
publishes ``/goal_pose``/``/planned_path``/``/cmd_vel*`` and never calls
``/plan_evacuation`` -- see ``test_exit_switching_orchestrator.py`` for the
structural proof.

Two switch flavours, matching factory_v5's actual main-loop wiring:

- **Hard** (`on_supervisor_status`): Stage 6 reported ``REPLAN_EXHAUSTED`` or
  ``EXIT_RESELECTION_REQUIRED`` for the current exit. No cost comparison --
  go straight to a commit-style ``SwitchRequest`` excluding the current exit.
- **Soft** (`on_hazard_snapshot`/`on_planned_path`/`tick`): a sustained observed-
  temperature-gated route-cost rise (`RouteTemperatureTrendMonitor`), delayed by
  actual travelled distance (`DelayedCostSwitch`), only escalates to a
  ``PeekRequest`` -- the resulting candidate's cost is compared against the
  current route's own last recorded average cost (`on_peek_result`) and a
  ``SwitchRequest`` is only emitted if the replacement is genuinely better.
  Otherwise the current, still-safe route is kept with no forced switch.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import json
import math
from typing import Any, Mapping, Sequence

from .event_replanning import convert_path_to_cells, remaining_path_from_pose
from .exit_switching import (
    DelayedCostSwitch,
    ExitSwitchingConfig,
    RouteTemperatureTrendMonitor,
    current_direction_world,
    is_opposite_direction,
)
from .replan_supervisor import ActiveGoal
from .safe_path_simplifier import expanded_path

_TRANSIENT_SWITCH_FAILURE_PREFIXES = (
    "EXIT_EVALUATOR_NOT_READY:",
    "EVALUATION_SERVICE_UNAVAILABLE",
    "EVALUATION_SERVICE_FAILED",
    "EVALUATION_SERVICE_TIMEOUT",
)

_HARD_TRIGGER_STATES = frozenset({"REPLAN_EXHAUSTED", "EXIT_RESELECTION_REQUIRED"})


@dataclass(frozen=True)
class PeekRequest:
    """Ask the node to rank candidates via /evaluate_exits without activating."""

    reason: str
    current_exit_id: str
    excluded_exit_ids: tuple[str, ...]
    candidate_exit_ids: tuple[str, ...] | None
    risk_first: bool


@dataclass(frozen=True)
class PeekResult:
    success: bool
    selected_exit_id: str | None
    average_cost: float | None
    used_fallback: bool


@dataclass(frozen=True)
class SwitchRequest:
    """Ask EvacuationManagerNode to select-and-activate a replacement exit."""

    reason: str
    request_id: int
    current_exit_id: str
    excluded_exit_ids: tuple[str, ...]
    candidate_exit_ids: tuple[str, ...] | None
    risk_first: bool

    def to_payload(self) -> dict[str, Any]:
        return {
            "reason": self.reason,
            "request_id": self.request_id,
            "current_exit_id": self.current_exit_id,
            "excluded_exit_ids": list(self.excluded_exit_ids),
            "candidate_exit_ids": (
                None if self.candidate_exit_ids is None else list(self.candidate_exit_ids)
            ),
            "risk_first": self.risk_first,
            "direct_target_activation": self.reason.startswith(
                "waypoint_proximity:"
            ),
        }


@dataclass(frozen=True)
class SwitchAck:
    request_id: int | None
    success: bool
    status: str | None
    selected_exit_id: str | None


@dataclass(frozen=True)
class ForcedProximitySwitch:
    """One-shot demonstration switch near configured map positions."""

    source_exit_id: str
    target_exit_id: str
    trigger_positions_world: tuple[tuple[float, float], ...]
    radius_m: float = 1.0
    retry_interval_sec: float = 1.0

    def __post_init__(self) -> None:
        if not self.source_exit_id or not self.target_exit_id:
            raise ValueError("forced proximity exit ids must not be empty")
        if self.source_exit_id == self.target_exit_id:
            raise ValueError("forced proximity source and target exits must differ")
        if not math.isfinite(self.radius_m) or self.radius_m <= 0.0:
            raise ValueError("forced proximity radius_m must be finite and positive")
        if (
            not math.isfinite(self.retry_interval_sec)
            or self.retry_interval_sec <= 0.0
        ):
            raise ValueError(
                "forced proximity retry_interval_sec must be finite and positive"
            )
        if not self.trigger_positions_world:
            raise ValueError("forced proximity trigger positions must not be empty")
        for point in self.trigger_positions_world:
            if len(point) != 2 or not all(math.isfinite(float(v)) for v in point):
                raise ValueError("forced proximity trigger positions must be finite x,y pairs")


def parse_switch_result_payload(payload: str) -> SwitchAck | None:
    try:
        value = json.loads(payload)
    except (TypeError, ValueError):
        return None
    if not isinstance(value, dict):
        return None
    return SwitchAck(
        value.get("request_id"), bool(value.get("success", False)),
        value.get("status"), value.get("selected_exit_id"),
    )


@dataclass(frozen=True)
class ExitSwitchingOutput:
    peek_request: PeekRequest | None
    switch_request: SwitchRequest | None
    status: dict[str, Any] = field(default_factory=dict)


class ExitSwitchingCore:
    def __init__(
        self, config: ExitSwitchingConfig,
        exit_positions_world: Mapping[str, tuple[float, float]],
        forced_proximity_switch: ForcedProximitySwitch | None = None,
    ) -> None:
        self.config = config
        self.exit_positions_world = dict(exit_positions_world)
        self.forced_proximity_switch = forced_proximity_switch
        if forced_proximity_switch is not None:
            missing = {
                forced_proximity_switch.source_exit_id,
                forced_proximity_switch.target_exit_id,
            } - set(self.exit_positions_world)
            if missing:
                raise ValueError(
                    "forced proximity exits are absent from registry: "
                    + ",".join(sorted(missing))
                )
        self.enabled = True
        self.active_goal: ActiveGoal | None = None
        self.supervisor_state: str | None = None
        self.snapshot = None
        self.latest_path_world: tuple[tuple[float, float], ...] = ()
        self.robot_pose_world: tuple[float, float] | None = None
        self.robot_yaw = 0.0
        self.elapsed_time = 0.0
        self.travelled_distance_m = 0.0
        self._last_pose_world: tuple[float, float] | None = None
        self._recent_positions: deque = deque(maxlen=2)
        self.route_monitor = RouteTemperatureTrendMonitor(
            config.evaluation_window, config.danger_expected_min_temperature_c,
        )
        self.delayed_switch = DelayedCostSwitch(config.additional_travel_before_switch_m)
        self.hard_latch: tuple[str, str] | None = None
        self.pending_switch: SwitchRequest | None = None
        self.pending_peek: PeekRequest | None = None
        self._next_request_id = 0
        self.last_switch_time = -math.inf
        self.last_failure_reason: str | None = None
        self._pending_soft_current_average_cost: float | None = None
        self.danger_expected_exit_ids: set[str] = set()
        self._route_heat_started_at: float | None = None
        self._last_live_temperature_at: float | None = None
        self._route_heat_detected = False
        self._forced_proximity_triggered = False
        self._forced_retry_pending = False
        self._forced_retry_not_before = -math.inf

    def set_enabled(self, enabled: bool) -> ExitSwitchingOutput:
        self.enabled = bool(enabled)
        if not self.enabled:
            self._reset_route_heat_streak()
        return self._output()

    def current_output(self) -> ExitSwitchingOutput:
        """Return the current status without advancing any state (for initial publish)."""
        return self._output()

    def on_active_goal(self, goal: ActiveGoal | None) -> ExitSwitchingOutput:
        if goal != self.active_goal:
            self.active_goal = goal
            # New canonical exit -- the previous exit's trend/delay state means
            # nothing for a different route (spec sections 11/30).
            self.route_monitor = RouteTemperatureTrendMonitor(
                self.config.evaluation_window,
                self.config.danger_expected_min_temperature_c,
            )
            self.delayed_switch = DelayedCostSwitch(
                self.config.additional_travel_before_switch_m
            )
            self.hard_latch = None
            self.pending_switch = None
            self.pending_peek = None
            self.latest_path_world = ()
            self.last_failure_reason = None
            self._reset_route_heat_streak()
            if (
                self.forced_proximity_switch is None
                or goal is None
                or goal.exit_id != self.forced_proximity_switch.source_exit_id
            ):
                self._forced_retry_pending = False
                self._forced_retry_not_before = -math.inf
        return self._output()

    def on_supervisor_status(self, state: str) -> ExitSwitchingOutput:
        self.supervisor_state = state
        if not self.enabled or self.active_goal is None:
            return self._output()
        if state not in _HARD_TRIGGER_STATES:
            self.hard_latch = None
            return self._output()
        latch = (self.active_goal.exit_id, state)
        if self.hard_latch == latch or self.pending_switch is not None:
            return self._output()
        self.hard_latch = latch
        return self._output(switch_request=self._new_switch_request(
            reason=(
                "replan_exhausted" if state == "REPLAN_EXHAUSTED"
                else "exit_reselection_required"
            ),
            excluded_exit_ids=(self.active_goal.exit_id,), candidate_exit_ids=None,
            risk_first=(state == "EXIT_RESELECTION_REQUIRED"),
        ))

    def on_hazard_snapshot(self, snapshot) -> ExitSwitchingOutput:
        self.snapshot = snapshot
        return self._advance_soft()

    def on_planned_path(self, path_world: Sequence[tuple[float, float]]) -> ExitSwitchingOutput:
        self.latest_path_world = tuple((float(x), float(y)) for x, y in path_world)
        return self._output()

    def on_live_temperature_observations(
        self, observations: Sequence[tuple[int, int, float]],
        observed_at: float,
    ) -> ExitSwitchingOutput:
        """Confirm route heat from consecutive *live* thermal frames.

        The accumulated hazard belief intentionally retains old observations;
        it therefore cannot prove that a hot object is still visible. This
        input contains only cells localized from the latest sensor frame.
        """
        now = float(observed_at)
        if not math.isfinite(now):
            self._reset_route_heat_streak()
            return self._output()
        if (
            self._last_live_temperature_at is None
            or now < self._last_live_temperature_at
            or now - self._last_live_temperature_at
            > self.config.danger_expected_max_observation_gap_sec
        ):
            self._route_heat_started_at = None
        self._last_live_temperature_at = now

        hot_on_route = self._live_heat_intersects_remaining_route(observations)
        self._route_heat_detected = hot_on_route
        if not hot_on_route:
            self._route_heat_started_at = None
            return self._output()
        if self._route_heat_started_at is None:
            self._route_heat_started_at = now
            return self._output()
        duration = max(0.0, now - self._route_heat_started_at)
        if (
            duration + 1e-12
            < self.config.danger_expected_confirmation_sec
            or not self.enabled
            or self.active_goal is None
            or self.pending_switch is not None
            or self.active_goal.exit_id in self.danger_expected_exit_ids
        ):
            return self._output()

        dangerous_exit = self.active_goal.exit_id
        self.danger_expected_exit_ids.add(dangerous_exit)
        self.hard_latch = (dangerous_exit, "DANGER_EXPECTED")
        self.delayed_switch.clear()
        self.pending_peek = None
        reason = (
            f"route_temperature_at_least_"
            f"{self.config.danger_expected_min_temperature_c:.1f}C_for_"
            f"{self.config.danger_expected_confirmation_sec:.1f}s:"
            "DANGER_EXPECTED"
        )
        return self._output(switch_request=self._new_switch_request(
            reason=reason,
            excluded_exit_ids=tuple(sorted(self.danger_expected_exit_ids)),
            candidate_exit_ids=None,
            risk_first=True,
        ))

    def tick(
        self, pose_world: tuple[float, float] | None, elapsed_time: float,
        yaw_rad: float = 0.0,
    ) -> ExitSwitchingOutput:
        self.elapsed_time = float(elapsed_time)
        if pose_world is not None:
            pose = (float(pose_world[0]), float(pose_world[1]))
            if self._last_pose_world is not None:
                self.travelled_distance_m += math.dist(pose, self._last_pose_world)
            self._recent_positions.append(pose)
            self._last_pose_world = pose
            self.robot_pose_world = pose
        self.robot_yaw = float(yaw_rad)
        forced = self._advance_forced_proximity()
        if forced.switch_request is not None:
            return forced
        return self._advance_soft()

    def on_peek_result(self, result: PeekResult) -> ExitSwitchingOutput:
        self.pending_peek = None
        current_average = self._pending_soft_current_average_cost
        self._pending_soft_current_average_cost = None
        if (
            not result.success or result.average_cost is None
            or current_average is None
            or not (result.average_cost < current_average - 1e-12)
        ):
            # No candidate, or it isn't actually better (spec sections 26/40) --
            # the current route is still safe, so keep it. No forced switch.
            self.delayed_switch.clear()
            return self._output()
        reason = self.delayed_switch.reason or "sustained_route_cost_increase"
        self.delayed_switch.clear()
        return self._output(switch_request=self._new_switch_request(
            reason=reason, excluded_exit_ids=(self.active_goal.exit_id,),
            candidate_exit_ids=(result.selected_exit_id,), risk_first=False,
        ))

    def on_switch_result(self, ack: SwitchAck) -> ExitSwitchingOutput:
        if self.pending_switch is None or ack.request_id != self.pending_switch.request_id:
            return self._output()  # stale or unrelated ack
        completed_request = self.pending_switch
        self.pending_switch = None
        if ack.success:
            self.last_switch_time = self.elapsed_time
            self._forced_retry_pending = False
            self._forced_retry_not_before = -math.inf
            self.last_failure_reason = None
            # on_active_goal() picks up the new canonical /evacuation/plan
            # separately and performs the actual monitor reset.
        else:
            self.last_failure_reason = ack.status or "NO_SAFE_ALTERNATIVE_EXIT"
            if (
                completed_request.reason.startswith("waypoint_proximity:")
                and self._is_transient_switch_failure(self.last_failure_reason)
                and self.forced_proximity_switch is not None
            ):
                self._forced_retry_pending = True
                self._forced_retry_not_before = (
                    self.elapsed_time
                    + self.forced_proximity_switch.retry_interval_sec
                )
            # hard_latch (set before the request went out) is left in place so a
            # hard trigger does not re-request every tick for the same Stage 6
            # terminal state (spec section 14: hold stays owned by Stage 6, this
            # core never touches it, and never falls back to the old unsafe
            # route or picks an exit on its own).
        return self._output()

    # -- internals ---------------------------------------------------------

    def _reset_route_heat_streak(self) -> None:
        self._route_heat_started_at = None
        self._last_live_temperature_at = None
        self._route_heat_detected = False

    def _advance_forced_proximity(self) -> ExitSwitchingOutput:
        trigger = self.forced_proximity_switch
        if (
            trigger is None
            or not self.enabled
            or self.active_goal is None
            or self.robot_pose_world is None
            or self.active_goal.exit_id != trigger.source_exit_id
            or self.pending_switch is not None
        ):
            return self._output()
        if self._forced_proximity_triggered:
            if (
                self._forced_retry_pending
                and self.elapsed_time + 1e-12 >= self._forced_retry_not_before
            ):
                self._forced_retry_pending = False
                return self._output(
                    switch_request=self._new_forced_proximity_request(trigger)
                )
            return self._output()
        if trigger.source_exit_id in self.danger_expected_exit_ids:
            return self._output()
        if not any(
            math.dist(self.robot_pose_world, point) <= trigger.radius_m + 1e-12
            for point in trigger.trigger_positions_world
        ):
            return self._output()

        self.danger_expected_exit_ids.add(trigger.source_exit_id)
        self._forced_proximity_triggered = True
        self.hard_latch = (trigger.source_exit_id, "DANGER_EXPECTED")
        self.delayed_switch.clear()
        self.pending_peek = None
        self._reset_route_heat_streak()
        return self._output(
            switch_request=self._new_forced_proximity_request(trigger)
        )

    def _new_forced_proximity_request(
        self, trigger: ForcedProximitySwitch,
    ) -> SwitchRequest:
        return self._new_switch_request(
            reason="waypoint_proximity:DANGER_EXPECTED",
            excluded_exit_ids=tuple(sorted(self.danger_expected_exit_ids)),
            candidate_exit_ids=(trigger.target_exit_id,),
            risk_first=False,
        )

    @staticmethod
    def _is_transient_switch_failure(status: str) -> bool:
        return any(
            status.startswith(prefix)
            for prefix in _TRANSIENT_SWITCH_FAILURE_PREFIXES
        )

    def _live_heat_intersects_remaining_route(self, observations) -> bool:
        if (
            self.active_goal is None or self.snapshot is None
            or not self.latest_path_world or self.robot_pose_world is None
        ):
            return False
        cells = convert_path_to_cells(
            self.latest_path_world, self.snapshot.geometry
        )
        robot_cell = self.snapshot.geometry.world_to_grid(
            *self.robot_pose_world
        )
        if cells is None or robot_cell is None:
            return False
        remaining = remaining_path_from_pose(expanded_path(cells), robot_cell)
        route_cells = set(expanded_path(remaining))
        threshold = self.config.danger_expected_min_temperature_c
        radius_cells = int(math.ceil(
            self.config.danger_expected_path_radius_m
            / self.snapshot.geometry.resolution
        ))
        offsets = tuple(
            (dx, dy)
            for dy in range(-radius_cells, radius_cells + 1)
            for dx in range(-radius_cells, radius_cells + 1)
            if math.hypot(dx, dy) * self.snapshot.geometry.resolution
            <= self.config.danger_expected_path_radius_m + 1e-12
        )
        for col, row, temperature in observations:
            try:
                cell = (int(col), int(row))
                value = float(temperature)
            except (TypeError, ValueError):
                continue
            on_route = any(
                (cell[0] + dx, cell[1] + dy) in route_cells
                for dx, dy in offsets
            )
            if on_route and math.isfinite(value) and value >= threshold:
                return True
        return False

    def _advance_soft(self) -> ExitSwitchingOutput:
        if (
            not self.enabled or self.active_goal is None or self.snapshot is None
            or not self.latest_path_world or self.robot_pose_world is None
            or self.hard_latch is not None or self.pending_switch is not None
            or self.pending_peek is not None
        ):
            return self._output()
        cells = convert_path_to_cells(self.latest_path_world, self.snapshot.geometry)
        if cells is None:
            return self._output()  # Stage 6 owns out-of-map path invalidation
        robot_cell = self.snapshot.geometry.world_to_grid(*self.robot_pose_world)
        if robot_cell is None:
            return self._output()
        remaining = remaining_path_from_pose(expanded_path(cells), robot_cell)
        trend = self.route_monitor.record(
            remaining, self.snapshot.final_cost, self.snapshot.temperature_c,
            self.snapshot.temperature_observed_mask,
            revision=self.snapshot.revision, evaluated_at=self.elapsed_time,
        )
        current_average = trend.current_average_cost
        if current_average is None and self.route_monitor.samples:
            current_average = self.route_monitor.samples[-1].average_route_cost
        if self.delayed_switch.exit_id not in (None, self.active_goal.exit_id):
            self.delayed_switch.clear()
        cooling_down = (
            self.elapsed_time - self.last_switch_time < self.config.switch_cooldown_sec
        )
        if (
            self.config.enabled and trend.switch_required
            and not self.delayed_switch.active and not cooling_down
        ):
            self.delayed_switch.arm(
                self.active_goal.exit_id, trend.reason, self.travelled_distance_m,
            )
        if not (self.delayed_switch.ready(self.travelled_distance_m) and not cooling_down):
            return self._output()

        self._pending_soft_current_average_cost = current_average
        direction = current_direction_world(
            self.robot_pose_world, self._next_waypoint_world(),
            tuple(self._recent_positions), self.robot_yaw,
        )
        opposite = tuple(
            exit_id for exit_id, position in self.exit_positions_world.items()
            if exit_id != self.active_goal.exit_id
            and is_opposite_direction(
                direction, self.robot_pose_world, position,
                minimum_difference_deg=self.config.minimum_direction_difference_deg,
            )
        )
        request = PeekRequest(
            self.delayed_switch.reason or "sustained_route_cost_increase",
            self.active_goal.exit_id, (self.active_goal.exit_id,), opposite or None,
            False,
        )
        self.pending_peek = request
        return self._output(peek_request=request)

    def _next_waypoint_world(self) -> tuple[float, float] | None:
        path = self.latest_path_world
        if not path or self.robot_pose_world is None:
            return None
        best_index, best_distance = 0, math.inf
        for index, point in enumerate(path):
            distance = math.hypot(
                point[0] - self.robot_pose_world[0], point[1] - self.robot_pose_world[1],
            )
            if distance < best_distance:
                best_distance, best_index = distance, index
        return path[min(best_index + 1, len(path) - 1)]

    def _new_switch_request(
        self, *, reason: str, excluded_exit_ids: Sequence[str],
        candidate_exit_ids: Sequence[str] | None, risk_first: bool,
    ) -> SwitchRequest:
        self._next_request_id += 1
        request = SwitchRequest(
            reason, self._next_request_id, self.active_goal.exit_id,
            tuple(excluded_exit_ids),
            None if candidate_exit_ids is None else tuple(candidate_exit_ids),
            risk_first,
        )
        self.pending_switch = request
        return request

    def _output(
        self, *, peek_request: PeekRequest | None = None,
        switch_request: SwitchRequest | None = None,
    ) -> ExitSwitchingOutput:
        status = {
            "enabled": self.enabled,
            "active_exit_id": None if self.active_goal is None else self.active_goal.exit_id,
            "supervisor_state": self.supervisor_state,
            "hard_switch_latched": self.hard_latch is not None,
            "peek_pending": self.pending_peek is not None,
            "forced_switch_retry_pending": self._forced_retry_pending,
            "forced_switch_retry_not_before": self._forced_retry_not_before,
            "switch_pending": self.pending_switch is not None,
            "delayed_switch_active": self.delayed_switch.active,
            "travelled_distance_m": self.travelled_distance_m,
            "last_failure_reason": self.last_failure_reason,
            "last_switch_time": self.last_switch_time,
            "danger_expected_exit_ids": sorted(self.danger_expected_exit_ids),
            "route_heat_detected": self._route_heat_detected,
            "forced_proximity_triggered": self._forced_proximity_triggered,
            "route_heat_duration_sec": (
                0.0 if self._route_heat_started_at is None
                else max(0.0, self._last_live_temperature_at - self._route_heat_started_at)
            ),
        }
        return ExitSwitchingOutput(peek_request, switch_request, status)
