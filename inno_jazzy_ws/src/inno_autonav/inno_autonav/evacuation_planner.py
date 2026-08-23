"""Pure deterministic exit selection matching factory_v5 EvacuationPlanner."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Sequence

from .exit_evaluator import ExitEvaluation, ExitEvaluationBatch


class EvacuationFailureReason(Enum):
    NO_EXITS_REGISTERED = "no_exits_registered"
    NO_SAFE_EXIT = "no_safe_exit"
    INVALID_START_POSITION = "invalid_start_position"


@dataclass(frozen=True)
class ExitSelectionConfig:
    prefer_confirmed_usable_exit: bool = True
    fallback_to_shortest_reachable_exit: bool = True
    primary_key: str = "path_length_m"
    secondary_key: str = "accumulated_risk_cost"
    final_tie_breaker: str = "exit_id"
    float_tolerance: float = 1e-6

    def __post_init__(self):
        for name in (
            "prefer_confirmed_usable_exit",
            "fallback_to_shortest_reachable_exit",
        ):
            if not isinstance(getattr(self, name), bool):
                raise TypeError(f"{name} must be bool")
        actual = self.primary_key, self.secondary_key, self.final_tie_breaker
        expected = "path_length_m", "accumulated_risk_cost", "exit_id"
        if actual != expected:
            raise ValueError(f"unsupported exit selection order: {actual}")
        if (
            isinstance(self.float_tolerance, bool)
            or not isinstance(self.float_tolerance, (int, float))
            or not math.isfinite(float(self.float_tolerance))
            or self.float_tolerance <= 0.0
        ):
            raise ValueError("float_tolerance must be finite and positive")

    @classmethod
    def from_mapping(cls, values: dict[str, Any] | None):
        values = dict(values or {})
        unknown = set(values) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f"unknown exit selection settings: {sorted(unknown)}")
        return cls(**values)


@dataclass(frozen=True)
class EvacuationPlan:
    success: bool
    start_position_world: tuple[float, float]
    selected_exit_id: str | None
    selected_exit_position_world: tuple[float, float] | None
    selected_approach_position_world: tuple[float, float] | None
    path_world: tuple[tuple[float, float], ...]
    path_grid: tuple[tuple[int, int], ...]
    selected_evaluation: ExitEvaluation | None
    all_evaluations: tuple[ExitEvaluation, ...]
    failure_reason: EvacuationFailureReason | None
    selection_reason: str | None
    created_at: float
    hazard_revision: int

    def to_dict(self):
        return {
            "success": self.success,
            "start_position_world": list(self.start_position_world),
            "selected_exit_id": self.selected_exit_id,
            "selected_exit_position_world": (
                None if self.selected_exit_position_world is None
                else list(self.selected_exit_position_world)
            ),
            "selected_approach_position_world": (
                None if self.selected_approach_position_world is None
                else list(self.selected_approach_position_world)
            ),
            "path_world": [list(item) for item in self.path_world],
            "path_grid": [list(item) for item in self.path_grid],
            "selected_evaluation": (
                None if self.selected_evaluation is None
                else self.selected_evaluation.to_dict()
            ),
            "all_evaluations": [item.to_dict() for item in self.all_evaluations],
            "failure_reason": (
                None if self.failure_reason is None else self.failure_reason.value
            ),
            "selection_reason": self.selection_reason,
            "created_at": self.created_at,
            "hazard_revision": self.hazard_revision,
        }


class EvacuationPlanner:
    """Select from completed Stage 4 evaluations; never calculate a path."""

    def __init__(self, config: ExitSelectionConfig | None = None):
        self.config = config or ExitSelectionConfig()

    def plan(
        self, batch: ExitEvaluationBatch, *, risk_first: bool = False,
        excluded_exit_ids: Sequence[str] = (),
        candidate_exit_ids: Sequence[str] | None = None,
    ) -> EvacuationPlan:
        if not isinstance(risk_first, bool):
            raise TypeError("risk_first must be bool")
        start = tuple(map(float, batch.robot_position_world))
        if len(start) != 2 or not all(math.isfinite(value) for value in start):
            return self._failure(
                start, batch.evaluations,
                EvacuationFailureReason.INVALID_START_POSITION,
                batch.evaluated_at, batch.hazard_revision,
            )
        if not batch.evaluations:
            return self._failure(
                start, (), EvacuationFailureReason.NO_EXITS_REGISTERED,
                batch.evaluated_at, batch.hazard_revision,
            )
        excluded = {str(item) for item in excluded_exit_ids}
        included = (
            None if candidate_exit_ids is None
            else {str(item) for item in candidate_exit_ids}
        )
        evaluations = tuple(sorted(
            (item for item in batch.evaluations
             if item.exit_id not in excluded
             and (included is None or item.exit_id in included)),
            key=lambda item: item.exit_id,
        ))
        if not evaluations:
            return self._failure(
                start, (), EvacuationFailureReason.NO_EXITS_REGISTERED,
                batch.evaluated_at, batch.hazard_revision,
            )
        accepted = [item for item in evaluations if item.accepted]
        if not accepted:
            return self._failure(
                start, evaluations, EvacuationFailureReason.NO_SAFE_EXIT,
                batch.evaluated_at, batch.hazard_revision,
            )
        confirmed = [item for item in accepted if item.exit_status == "usable"]
        if self.config.prefer_confirmed_usable_exit and confirmed:
            candidates = confirmed
        elif (
            self.config.prefer_confirmed_usable_exit
            and not self.config.fallback_to_shortest_reachable_exit
        ):
            return self._failure(
                start, evaluations, EvacuationFailureReason.NO_SAFE_EXIT,
                batch.evaluated_at, batch.hazard_revision,
            )
        else:
            candidates = accepted

        tolerance = self.config.float_tolerance

        def bucket(value):
            number = float(value)
            if not math.isfinite(number):
                raise ValueError("accepted exit selection metrics must be finite")
            return int(round(number / tolerance))

        primary = "accumulated_risk_cost" if risk_first else self.config.primary_key
        secondary = "path_length_m" if risk_first else self.config.secondary_key
        candidates.sort(key=lambda item: (
            bucket(getattr(item, primary)),
            bucket(getattr(item, secondary)), item.exit_id,
        ))
        selected = candidates[0]
        reason = (
            "confirmed usable exits preferred; lowest accumulated path risk; "
            "ties resolved by path length then exit_id"
            if risk_first else
            "confirmed usable exits preferred; shortest cost-aware A* path; "
            "ties resolved by accumulated risk cost then exit_id"
        )
        return EvacuationPlan(
            True, start, selected.exit_id, selected.exit_position_world,
            selected.approach_position_world, selected.path_world,
            selected.path_grid, selected, evaluations, None, reason,
            float(batch.evaluated_at), int(batch.hazard_revision),
        )

    @staticmethod
    def _failure(start, evaluations, reason, created_at, revision):
        return EvacuationPlan(
            False, tuple(start), None, None, None, (), (), None,
            tuple(evaluations), reason, None, float(created_at), int(revision),
        )


def parse_evaluation_batch_json(payload: str, expected_frame="map"):
    import json

    try:
        value = json.loads(payload)
    except (TypeError, ValueError) as exc:
        raise ValueError("exit evaluation response is not valid JSON") from exc
    return ExitEvaluationBatch.from_dict(value, expected_frame)


def route_activation_decision(plan, *, activate, current_revision):
    """Return a manager status without monitoring or changing any goal."""
    if not plan.success:
        return "SELECTION_FAILED", False
    if not activate:
        return "SELECTED_NOT_ACTIVATED", False
    if current_revision is None:
        return "HAZARD_REVISION_NOT_READY", False
    if int(current_revision) != plan.hazard_revision:
        return "EVALUATION_STALE", False
    if plan.selected_approach_position_world is None:
        return "SELECTED_APPROACH_MISSING", False
    return "ROUTE_ACTIVATED", True


def build_evacuation_decision(
    payload, planner, *, expected_frame="map", risk_first=False,
    activate=False, current_revision=None,
):
    """Pure manager pipeline from a Stage 4 response to activation intent."""
    batch = parse_evaluation_batch_json(payload, expected_frame)
    plan = planner.plan(batch, risk_first=risk_first)
    status, activated = route_activation_decision(
        plan, activate=activate, current_revision=current_revision,
    )
    return plan, status, activated
