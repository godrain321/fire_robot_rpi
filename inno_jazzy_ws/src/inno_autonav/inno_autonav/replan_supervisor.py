"""ROS-independent Stage 6 orchestration: "is the active path still safe?".

``ReplanSupervisorCore`` answers exactly one question per update: does the currently
active remaining path still satisfy :mod:`inno_autonav.event_replanning`? If yes,
nothing happens. If no, it holds movement, re-requests a path to the *same* active
goal, revalidates whatever comes back, and only then releases the hold. It never
computes a path itself (no A*, no reference-waypoint-graph call) and never selects a
different exit — see the module-level ``FORBIDDEN`` note below, enforced by
``inno_autonav/test/test_replan_supervisor.py``.

FORBIDDEN in this module: calling ``/plan_evacuation``, publishing ``/planned_path``,
choosing an ``exit_id`` other than the one supplied via :meth:`ReplanSupervisorCore
.on_active_goal`, and reading FDS/Ground-Truth data. All are Stage 7+ responsibilities.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import json
import math
from typing import Any, Callable, Mapping, Sequence

from .event_replanning import (
    Cell,
    EventReplanningConfig,
    EventReplanningPolicy,
    PathValidationResult,
    ReplanDecision,
    ReplanPriority,
    ReplanReason,
    convert_path_to_cells,
    remaining_path_from_pose,
    validate_remaining_path,
)
from .exit_evaluator import ExitHazardSnapshot, ExitStatus
from .safe_path_simplifier import expanded_path


_EXIT_INVALID_REASONS = (
    ReplanReason.EXIT_BLOCKED,
    ReplanReason.EXIT_UNSAFE_FIRE,
    ReplanReason.EXIT_DANGER_EXPECTED,
)
_REEVALUATION_REASONS = (
    ReplanReason.PERIODIC_REEVALUATION,
    ReplanReason.DISTANCE_REEVALUATION,
)
_EXIT_STATUS_BLOCKING = (
    ExitStatus.BLOCKED, ExitStatus.DANGEROUS, ExitStatus.DANGER_EXPECTED,
)
# astar_replanner._plan() sets these two /planner_state values *before* it starts
# computing (astar_replanner.py:592-593); every other string it publishes is terminal
# (either 'PATH_READY' or a failure state).
_IN_PROGRESS_PLANNER_STATES = frozenset({"PLANNING", "REPLANNING"})


class SupervisorState(str, Enum):
    DISABLED = "DISABLED"
    WAITING_FOR_HAZARD = "WAITING_FOR_HAZARD"
    WAITING_FOR_PATH = "WAITING_FOR_PATH"
    READY = "READY"
    PATH_VALID = "PATH_VALID"
    HOLDING_FOR_REPLAN = "HOLDING_FOR_REPLAN"
    REPLAN_REQUESTED = "REPLAN_REQUESTED"
    WAITING_FOR_NEW_PATH = "WAITING_FOR_NEW_PATH"
    REPLAN_SUCCEEDED = "REPLAN_SUCCEEDED"
    REPLAN_FAILED = "REPLAN_FAILED"
    REPLAN_EXHAUSTED = "REPLAN_EXHAUSTED"
    EXIT_RESELECTION_REQUIRED = "EXIT_RESELECTION_REQUIRED"


@dataclass(frozen=True)
class RetryConfig:
    max_replan_attempts: int = 5
    cooldown_seconds: float = 0.5
    replan_timeout_s: float = 3.0

    def __post_init__(self) -> None:
        if isinstance(self.max_replan_attempts, bool) or self.max_replan_attempts < 1:
            raise ValueError("max_replan_attempts must be a positive integer")
        for name in ("cooldown_seconds", "replan_timeout_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True)
class ActiveGoal:
    exit_id: str
    approach_world: tuple[float, float]
    hazard_revision: int
    approach_yaw_rad: float | None = None


def parse_active_goal_payload(payload: str) -> ActiveGoal | None:
    """Extract the active goal from a Stage 5 ``/evacuation/plan`` JSON payload.

    Only an *activated* plan counts (spec §38): an evaluation batch that merely
    ranked exits, or one Stage 5 rejected/failed to activate, must not be treated as
    "our" goal. Returns ``None`` for anything else, including malformed JSON.
    """
    try:
        value = json.loads(payload)
    except (TypeError, ValueError):
        return None
    if not isinstance(value, dict) or not value.get("success") or not value.get("activated"):
        return None
    try:
        exit_id = str(value["selected_exit_id"])
        approach = value["selected_approach_position_world"]
        approach_world = (float(approach[0]), float(approach[1]))
        revision = int(value["hazard_revision"])
        raw_yaw = value.get("selected_approach_yaw_rad")
        approach_yaw = None if raw_yaw is None else float(raw_yaw)
    except (KeyError, TypeError, ValueError, IndexError):
        return None
    if (
        not exit_id
        or not all(math.isfinite(v) for v in approach_world)
        or approach_yaw is not None and not math.isfinite(approach_yaw)
    ):
        return None
    return ActiveGoal(exit_id, approach_world, revision, approach_yaw)


@dataclass(frozen=True)
class SupervisorOutput:
    hold: bool
    publish_goal: tuple[float, float] | None
    status: dict[str, Any] = field(default_factory=dict)


class ReplanSupervisorCore:
    def __init__(
        self, config: EventReplanningConfig, retry: RetryConfig | None = None,
        *, exit_status_lookup: Callable[[], Mapping[str, ExitStatus]] | None = None,
    ) -> None:
        self.config = config
        self.retry = retry or RetryConfig()
        # No live per-exit status registry exists yet in Stage 1-5 (ExitEvaluator is
        # on-demand only, via /evaluate_exits) -- see spec section 40. Rather than
        # fabricate a status, this stays unset unless a caller explicitly wires one in.
        self._exit_status_lookup = exit_status_lookup
        self.enabled = True
        self.policy = EventReplanningPolicy(config)
        self._baseline_established = False
        self.state = SupervisorState.WAITING_FOR_HAZARD
        self.active_goal: ActiveGoal | None = None
        self.snapshot: ExitHazardSnapshot | None = None
        self.latest_path_world: tuple[tuple[float, float], ...] = ()
        self.robot_pose_world: tuple[float, float] | None = None
        self.elapsed_time: float = 0.0
        self.hold: bool = False
        self.replan_in_flight: bool = False
        self.attempt_count: int = 0
        self.last_attempt_time: float = -math.inf
        self.request_started_at: float | None = None
        self.last_decision: ReplanDecision | None = None
        self.last_validated_revision: int | None = None
        self.last_failure_reason: str | None = None

    def set_enabled(self, enabled: bool) -> SupervisorOutput:
        self.enabled = bool(enabled)
        if not self.enabled:
            self.state = SupervisorState.DISABLED
            self.hold = False
        elif self.state is SupervisorState.DISABLED:
            self.state = (
                SupervisorState.WAITING_FOR_HAZARD if self.snapshot is None
                else SupervisorState.WAITING_FOR_PATH
            )
        return self._advance()

    def on_active_goal(self, goal: ActiveGoal | None) -> SupervisorOutput:
        if goal != self.active_goal:
            self.active_goal = goal
            self.policy = EventReplanningPolicy(self.config)
            self._baseline_established = False
            self.latest_path_world = ()
            self.hold = False
            self.replan_in_flight = False
            self.attempt_count = 0
            self.last_attempt_time = -math.inf
            self.request_started_at = None
            self.last_decision = None
            self.last_validated_revision = None
            self.last_failure_reason = None
            self.state = (
                SupervisorState.WAITING_FOR_PATH if self.enabled
                else SupervisorState.DISABLED
            )
        return self._advance()

    def on_hazard_snapshot(self, snapshot: ExitHazardSnapshot) -> SupervisorOutput:
        self.snapshot = snapshot
        return self._advance()

    def on_planned_path(
        self, path_world: Sequence[tuple[float, float]],
    ) -> SupervisorOutput:
        self.latest_path_world = tuple((float(x), float(y)) for x, y in path_world)
        return self._advance()

    def on_planner_state(self, text: str) -> SupervisorOutput:
        if self.enabled and self.replan_in_flight:
            if text in _IN_PROGRESS_PLANNER_STATES:
                self.state = SupervisorState.WAITING_FOR_NEW_PATH
            elif text == "PATH_READY":
                self._evaluate_replan_result()
            else:
                self._fail_replan(text)
        return self._advance()

    def on_replan_progress(self) -> SupervisorOutput:
        """Refresh the per-attempt timeout when waypoint repair advances to A*."""
        if self.enabled and self.replan_in_flight:
            self.request_started_at = self.elapsed_time
            self.state = SupervisorState.WAITING_FOR_NEW_PATH
        return self._output(None)

    def tick(self, robot_pose_world: tuple[float, float] | None, elapsed_time: float) -> SupervisorOutput:
        self.robot_pose_world = robot_pose_world
        self.elapsed_time = float(elapsed_time)
        return self._advance()

    def current_output(self) -> SupervisorOutput:
        """Return the current status without advancing any state (for initial publish)."""
        return self._output(None)

    # -- internals ---------------------------------------------------------

    def _evaluate_replan_result(self) -> None:
        assert self.snapshot is not None
        cells = convert_path_to_cells(self.latest_path_world, self.snapshot.geometry)
        if cells is None:
            self._fail_replan("NEW_PATH_OUTSIDE_MAP")
            return
        result = validate_remaining_path(cells, self.snapshot)
        if result.safe:
            self.replan_in_flight = False
            self.attempt_count = 0
            self.hold = False
            self.last_validated_revision = self.snapshot.revision
            self.state = SupervisorState.REPLAN_SUCCEEDED
        else:
            self._fail_replan("NEW_PATH_UNSAFE:" + ",".join(r.value for r in result.rejection_reasons))

    def _fail_replan(self, reason: str) -> None:
        self.replan_in_flight = False
        self.last_attempt_time = self.elapsed_time
        self.last_failure_reason = reason
        self.hold = True
        self.state = SupervisorState.REPLAN_FAILED

    def _start_replan(self) -> tuple[float, float] | None:
        assert self.active_goal is not None
        if self.attempt_count >= self.retry.max_replan_attempts:
            self.state = SupervisorState.REPLAN_EXHAUSTED
            self.hold = True
            return None
        self.attempt_count += 1
        self.replan_in_flight = True
        self.request_started_at = self.elapsed_time
        self.hold = True
        self.state = SupervisorState.REPLAN_REQUESTED
        return self.active_goal.approach_world

    def _current_exit_status(self) -> Mapping[str, ExitStatus]:
        if self._exit_status_lookup is None:
            return {}
        try:
            return dict(self._exit_status_lookup())
        except Exception:
            return {}

    def _advance(self) -> SupervisorOutput:
        if not self.enabled:
            self.state = SupervisorState.DISABLED
            self.hold = False
            return self._output(None)
        if self.snapshot is None:
            self.state = SupervisorState.WAITING_FOR_HAZARD
            return self._output(None)
        if self.active_goal is None:
            self.state = SupervisorState.WAITING_FOR_PATH
            self.hold = False
            return self._output(None)
        if not self.latest_path_world:
            self.state = SupervisorState.WAITING_FOR_PATH
            return self._output(None)

        if self.replan_in_flight:
            if (
                self.request_started_at is not None
                and self.elapsed_time - self.request_started_at > self.retry.replan_timeout_s
            ):
                self._fail_replan("REQUEST_TIMEOUT")
            else:
                return self._output(None)

        if self.state is SupervisorState.REPLAN_FAILED:
            if self.elapsed_time - self.last_attempt_time < self.retry.cooldown_seconds:
                return self._output(None)
            return self._output(self._start_replan())

        if self.state in (
            SupervisorState.REPLAN_EXHAUSTED, SupervisorState.EXIT_RESELECTION_REQUIRED,
        ):
            self.hold = True
            return self._output(None)

        if self.robot_pose_world is None:
            return self._output(None)

        if not self._baseline_established:
            # Mirrors run_partial_costmap_evacuation.py's setup, which calls
            # mark_reevaluation_complete() once before the first real evaluate()
            # tick. Without this, EventReplanningPolicy._periodic() would set
            # last_replan_time = now as a side effect of this very first call,
            # and a same-tick non-emergency decision (priority < HAZARD_BLOCKED,
            # e.g. EXIT_INVALID) would then spuriously self-suppress against that
            # freshly-set timestamp. Emergency-priority reasons (PATH_BLOCKED /
            # HAZARD_BLOCKED) are unaffected either way -- see _suppressed().
            self.policy.mark_reevaluation_complete(
                elapsed_time=self.elapsed_time, robot_pose=self.robot_pose_world,
                costmap_revision=self.snapshot.revision,
            )
            self._baseline_established = True
            self.state = SupervisorState.PATH_VALID
            self.last_validated_revision = self.snapshot.revision
            return self._output(None)

        cells = convert_path_to_cells(self.latest_path_world, self.snapshot.geometry)
        if cells is None:
            self.last_decision = ReplanDecision(
                True, True, True, ReplanReason.PATH_INVALID, ReplanPriority.PATH_BLOCKED,
                None, detail="planned path waypoint outside map",
            )
            self.policy.mark_processed(
                self.last_decision, costmap_revision=self.snapshot.revision,
                elapsed_time=self.elapsed_time, robot_pose=self.robot_pose_world,
                selected_exit_id=self.active_goal.exit_id,
            )
            return self._output(self._start_replan())

        robot_cell = self.snapshot.geometry.world_to_grid(*self.robot_pose_world)
        if robot_cell is None:
            return self._output(None)
        # A sparse simplified /planned_path must not be checked at its waypoints
        # only (spec section 17) -- expand to every supercover-touched cell first,
        # exactly like the simulation's call site does via _expanded_grid_segments
        # before calling EventReplanningPolicy.evaluate().
        remaining = remaining_path_from_pose(expanded_path(cells), robot_cell)

        decision = self.policy.evaluate(
            current_path=remaining, current_costmap=self.snapshot.final_cost,
            costmap_revision=self.snapshot.revision,
            dynamic_obstacle_map=self.snapshot.dynamic_obstacle_map,
            temperature_map=self.snapshot.temperature_c,
            co_map=self.snapshot.co_ppm,
            temperature_observed_mask=self.snapshot.temperature_observed_mask,
            co_observed_mask=self.snapshot.co_observed_mask,
            exit_statuses=self._current_exit_status(),
            current_exit_id=self.active_goal.exit_id,
            robot_pose=self.robot_pose_world, elapsed_time=self.elapsed_time,
        )
        self.last_decision = decision
        if not decision.required:
            self.state = SupervisorState.PATH_VALID
            self.hold = False
            self.last_validated_revision = self.snapshot.revision
            return self._output(None)

        if decision.reason in _EXIT_INVALID_REASONS:
            self.policy.mark_processed(
                decision, costmap_revision=self.snapshot.revision,
                elapsed_time=self.elapsed_time, robot_pose=self.robot_pose_world,
                selected_exit_id=self.active_goal.exit_id,
            )
            self.state = SupervisorState.EXIT_RESELECTION_REQUIRED
            self.hold = True
            return self._output(None)

        if decision.reason in _REEVALUATION_REASONS:
            validation = self._validate_remaining(remaining)
            exit_blocked = self._current_exit_status().get(self.active_goal.exit_id) in _EXIT_STATUS_BLOCKING
            if validation.safe and not exit_blocked:
                self.policy.mark_reevaluation_complete(
                    elapsed_time=self.elapsed_time, robot_pose=self.robot_pose_world,
                    costmap_revision=self.snapshot.revision,
                )
                self.state = SupervisorState.PATH_VALID
                self.hold = False
                self.last_validated_revision = self.snapshot.revision
                return self._output(None)
            # Reevaluation found the path (or exit) is actually unsafe -- escalate
            # exactly like a real invalidation instead of silently dropping it.

        self.policy.mark_processed(
            decision, costmap_revision=self.snapshot.revision,
            elapsed_time=self.elapsed_time, robot_pose=self.robot_pose_world,
            selected_exit_id=self.active_goal.exit_id,
        )
        return self._output(self._start_replan())

    def _validate_remaining(self, remaining: Sequence[Cell]) -> PathValidationResult:
        assert self.snapshot is not None
        return validate_remaining_path(remaining, self.snapshot)

    def _output(self, publish_goal: tuple[float, float] | None) -> SupervisorOutput:
        status = {
            "enabled": self.enabled,
            "state": self.state.value,
            "hazard_revision": None if self.snapshot is None else self.snapshot.revision,
            "last_validated_revision": self.last_validated_revision,
            "active_exit_id": None if self.active_goal is None else self.active_goal.exit_id,
            "last_replan_reason": (
                None if self.last_decision is None else self.last_decision.reason.value
            ),
            "affected_cell_grid": (
                None if self.last_decision is None or self.last_decision.affected_cell_grid is None
                else list(self.last_decision.affected_cell_grid)
            ),
            "attempt_count": self.attempt_count,
            "max_replan_attempts": self.retry.max_replan_attempts,
            "hold": self.hold,
            "replan_requested": publish_goal is not None,
            "last_failure_reason": self.last_failure_reason,
            "elapsed_time": self.elapsed_time,
        }
        return SupervisorOutput(self.hold, publish_goal, status)
