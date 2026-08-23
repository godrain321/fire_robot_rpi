"""ROS-independent identity/state gate for Stage 8-9/8-10 same-exit repair."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .replan_supervisor import ActiveGoal


@dataclass(frozen=True)
class ReplanCommand:
    topic: str
    payload: dict[str, Any]


class SameExitReplanCoordinator:
    """Orders waypoint first, then A*, and rejects results from stale goals/requests."""

    def __init__(self) -> None:
        self.goal: ActiveGoal | None = None
        self.mode = "WAYPOINT"
        self._sequence = 0
        self.active_request_id: str | None = None
        self.phase = "IDLE"
        self._candidate_stamps: dict[str, set[int]] = {}
        self._successful_result: dict[str, Any] | None = None

    def on_goal(self, goal: ActiveGoal | None) -> bool:
        changed = goal != self.goal
        if changed:
            self.goal = goal
            self._sequence += 1
            self.active_request_id = None
            self.phase = "IDLE"
            self.mode = "WAYPOINT"
            self._candidate_stamps.clear()
            self._successful_result = None
        return changed

    def start(self, hazard_revision: int | None, attempt: int) -> ReplanCommand | None:
        if self.goal is None:
            return None
        self._sequence += 1
        request_id = f"{self._sequence}:{self.goal.exit_id}:{attempt}"
        self.active_request_id = request_id
        self.phase = "WAYPOINT"
        self._candidate_stamps.clear()
        self._successful_result = None
        return ReplanCommand("waypoint", self._payload(request_id, hazard_revision))

    def on_waypoint_result(
        self, result: Mapping[str, Any],
    ) -> ReplanCommand | dict[str, Any] | None:
        if not self._matches(result, "WAYPOINT"):
            return None
        if bool(result.get("success")):
            self._successful_result = dict(result)
            return self._activation_if_ready("WAYPOINT")
        self.phase = "A_STAR"
        self._candidate_stamps.clear()
        self._successful_result = None
        return ReplanCommand("astar", self._payload(
            self.active_request_id, result.get("hazard_revision")
        ))

    def on_astar_result(self, result: Mapping[str, Any]) -> str | dict[str, Any] | None:
        if not self._matches(result, "A_STAR"):
            return None
        if bool(result.get("success")):
            self._successful_result = dict(result)
            return self._activation_if_ready("A_STAR")
        self.phase = "FAILED"
        return "A_STAR_FAILED"

    def on_candidate_path(
        self, source: str, *, stamp_ns: int, goal_world: tuple[float, float], nonempty: bool,
    ) -> dict[str, Any] | None:
        source = str(source).upper()
        if (
            self.goal is None or self.phase != source or not nonempty
            or tuple(goal_world) != self.goal.approach_world
        ):
            return None
        self._candidate_stamps.setdefault(source, set()).add(int(stamp_ns))
        return self._activation_if_ready(source)

    def _activation_if_ready(self, source: str) -> dict[str, Any] | None:
        result = self._successful_result
        if result is None:
            return None
        try:
            result_stamp = int(result["path_stamp_ns"])
        except (KeyError, TypeError, ValueError):
            return None
        if result_stamp not in self._candidate_stamps.get(source, set()):
            return None
        self.phase = "VALIDATING"
        self.mode = source
        return {
            "mode": source,
            "request_id": self.active_request_id,
            "path_stamp_ns": result_stamp,
            "exit_id": self.goal.exit_id,
            "goal_world": list(self.goal.approach_world),
        }

    def _payload(self, request_id: str | None, revision: Any) -> dict[str, Any]:
        assert self.goal is not None and request_id is not None
        return {
            "request_id": request_id,
            "exit_id": self.goal.exit_id,
            "goal_world": list(self.goal.approach_world),
            "goal_hazard_revision": self.goal.hazard_revision,
            "hazard_revision": revision,
        }

    def _matches(self, result: Mapping[str, Any], phase: str) -> bool:
        return bool(
            self.goal is not None
            and self.phase == phase
            and result.get("request_id") == self.active_request_id
            and result.get("exit_id") == self.goal.exit_id
            and tuple(result.get("goal_world", ())) == self.goal.approach_world
        )
