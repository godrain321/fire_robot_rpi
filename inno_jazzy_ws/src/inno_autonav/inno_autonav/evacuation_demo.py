"""Pure state helpers for the Mode 5 evacuation demo starter."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from typing import Iterable

from .evacuation_planner import (
    EvacuationPlanner,
    parse_evaluation_batch_json,
)


ACTIVE_HAZARD_STATES = frozenset({
    "ACTIVE", "ACTIVE_THERMAL_ONLY", "ACTIVE_STATIC_DYNAMIC_ONLY",
})


@dataclass(frozen=True)
class MovingCandidate:
    """One LiDAR track that moved enough to justify Mode 4 inspection."""

    track_id: int
    position: tuple[float, float]
    displacement_m: float
    observations: int


@dataclass
class _CandidateTrack:
    track_id: int
    history: list[tuple[float, float, float]] = field(default_factory=list)
    reported: bool = False


class MovingCandidateTracker:
    """Associate LiDAR clusters over time and report genuinely moving tracks.

    This is deliberately independent of ROS so the motion rule can be tested
    deterministically before it is allowed to pre-empt an evacuation route.
    """

    def __init__(
        self,
        *,
        association_radius_m: float = 0.75,
        minimum_displacement_m: float = 0.20,
        minimum_observations: int = 3,
        window_sec: float = 2.0,
        stale_timeout_sec: float = 1.0,
    ) -> None:
        values = (
            association_radius_m,
            minimum_displacement_m,
            window_sec,
            stale_timeout_sec,
        )
        if (
            any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in values)
            or isinstance(minimum_observations, bool)
            or int(minimum_observations) < 2
        ):
            raise ValueError("moving-candidate tracker parameters are invalid")
        self.association_radius_m = float(association_radius_m)
        self.minimum_displacement_m = float(minimum_displacement_m)
        self.minimum_observations = int(minimum_observations)
        self.window_sec = float(window_sec)
        self.stale_timeout_sec = float(stale_timeout_sec)
        self._tracks: dict[int, _CandidateTrack] = {}
        self._next_track_id = 1

    def reset(self) -> None:
        self._tracks.clear()
        self._next_track_id = 1

    def update(
        self, candidates: Iterable[tuple[float, float]], timestamp: float
    ) -> tuple[MovingCandidate, ...]:
        now = float(timestamp)
        if not math.isfinite(now):
            raise ValueError("timestamp must be finite")
        points = []
        for candidate in candidates:
            try:
                x, y = tuple(candidate)
                point = (float(x), float(y))
            except (TypeError, ValueError):
                continue
            if all(math.isfinite(value) for value in point):
                points.append(point)

        stale_before = now - self.stale_timeout_sec
        self._tracks = {
            track_id: track
            for track_id, track in self._tracks.items()
            if track.history and track.history[-1][0] >= stale_before
        }
        edges = []
        for track_id, track in self._tracks.items():
            _, old_x, old_y = track.history[-1]
            for candidate_index, (x, y) in enumerate(points):
                distance = math.hypot(x - old_x, y - old_y)
                if distance <= self.association_radius_m:
                    edges.append((distance, track_id, candidate_index))
        matched_tracks = set()
        matched_candidates = set()
        for _, track_id, candidate_index in sorted(edges):
            if track_id in matched_tracks or candidate_index in matched_candidates:
                continue
            x, y = points[candidate_index]
            self._tracks[track_id].history.append((now, x, y))
            matched_tracks.add(track_id)
            matched_candidates.add(candidate_index)
        for candidate_index, (x, y) in enumerate(points):
            if candidate_index in matched_candidates:
                continue
            track_id = self._next_track_id
            self._next_track_id += 1
            self._tracks[track_id] = _CandidateTrack(
                track_id=track_id, history=[(now, x, y)]
            )
            matched_tracks.add(track_id)

        detected = []
        window_start = now - self.window_sec
        for track_id in matched_tracks:
            track = self._tracks[track_id]
            track.history = [item for item in track.history if item[0] >= window_start]
            if track.reported or len(track.history) < self.minimum_observations:
                continue
            _, first_x, first_y = track.history[0]
            displacement = max(
                math.hypot(x - first_x, y - first_y)
                for _, x, y in track.history[1:]
            )
            if displacement < self.minimum_displacement_m:
                continue
            track.reported = True
            _, x, y = track.history[-1]
            detected.append(MovingCandidate(
                track_id=track.track_id,
                position=(x, y),
                displacement_m=displacement,
                observations=len(track.history),
            ))
        return tuple(detected)


def group_leg_candidates(
    candidates: Iterable[tuple[float, float]],
    maximum_pair_distance_m: float = 0.70,
) -> tuple[tuple[float, float], ...]:
    """Deterministically merge nearest small-cluster pairs into body centres."""
    radius = float(maximum_pair_distance_m)
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("maximum_pair_distance_m must be finite and positive")
    points = set()
    for candidate in candidates:
        try:
            point = tuple(float(value) for value in candidate)
        except (TypeError, ValueError):
            continue
        if len(point) == 2 and all(math.isfinite(value) for value in point):
            points.add(point)
    ordered = sorted(points)
    edges = sorted(
        (math.dist(first, second), first, second)
        for index, first in enumerate(ordered)
        for second in ordered[index + 1:]
        if math.dist(first, second) <= radius
    )
    used = set()
    output = []
    for _, first, second in edges:
        if first in used or second in used:
            continue
        used.update((first, second))
        output.append(((first[0] + second[0]) / 2.0, (first[1] + second[1]) / 2.0))
    output.extend(point for point in ordered if point not in used)
    return tuple(sorted(output))


def moving_priority_candidate(
    moving_candidates,
    confirmed_candidates,
    robot_position_world,
    inspected_positions=(),
    association_radius_m: float = 0.75,
    suppression_radius_m: float = 1.0,
):
    """Return the nearest red candidate associated with current motion evidence."""
    association = float(association_radius_m)
    if not math.isfinite(association) or association <= 0.0:
        raise ValueError("association_radius_m must be finite and positive")
    eligible_red = []
    for red in confirmed_candidates:
        selected = nearest_uninspected_candidate(
            [red], robot_position_world, inspected_positions, suppression_radius_m
        )
        if selected is not None:
            eligible_red.append(selected)
    matches = []
    robot = tuple(map(float, robot_position_world))
    for moving in moving_candidates:
        try:
            motion = tuple(map(float, moving))
        except (TypeError, ValueError):
            continue
        if len(motion) != 2 or not all(math.isfinite(value) for value in motion):
            continue
        for red in eligible_red:
            separation = math.dist(motion, red)
            if separation <= association:
                matches.append((math.dist(robot, red), separation, red))
    return None if not matches else min(matches)[2]


def startup_state(
    hazard_status: str,
    exit_evaluator_status: str,
    evacuation_manager_status: str,
    drive_mode_status: str,
    evaluation_service_ready: bool,
) -> str:
    """Return the first unmet Mode 5 prerequisite or ``SEARCH_EXITS``."""
    if not str(drive_mode_status).startswith("5:EVACUATION_DEMO"):
        return "SEARCH_EXITS:SELECTING_MODE_5"
    if hazard_status not in ACTIVE_HAZARD_STATES:
        return "SEARCH_EXITS:WAITING_FOR_HAZARD"
    if exit_evaluator_status != "READY":
        return "SEARCH_EXITS:WAITING_FOR_EXIT_EVALUATOR"
    if not evacuation_manager_status or evacuation_manager_status == "DISABLED":
        return "SEARCH_EXITS:WAITING_FOR_MANAGER"
    if not evaluation_service_ready:
        return "SEARCH_EXITS:WAITING_FOR_EVALUATION_SERVICE"
    return "SEARCH_EXITS"


@dataclass(frozen=True)
class ExplorationDecision:
    complete: bool
    success: bool
    status: str
    target_exit_id: str | None = None
    exit_position_world: tuple[float, float] | None = None
    approach_position_world: tuple[float, float] | None = None
    plan_payload: str | None = None


def build_next_exploration_decision(
    evaluation_payload: str,
    checked_exit_ids,
    *,
    expected_frame: str = "map",
) -> ExplorationDecision:
    """Select the nearest safe unchecked exit and build a canonical plan."""
    batch = parse_evaluation_batch_json(evaluation_payload, expected_frame)
    checked = {str(item) for item in checked_exit_ids}
    remaining = tuple(
        item.exit_id for item in batch.evaluations if item.exit_id not in checked
    )
    if not remaining:
        return ExplorationDecision(True, True, "EXPLORATION_COMPLETE")
    plan = EvacuationPlanner().plan(batch, candidate_exit_ids=remaining)
    if not plan.success:
        reason = (
            "NO_SAFE_UNCHECKED_EXIT"
            if plan.failure_reason is None
            else plan.failure_reason.value.upper()
        )
        return ExplorationDecision(False, False, reason)
    payload = plan.to_dict()
    payload["activated"] = True
    payload["manager_status"] = "EXPLORATION_ROUTE_ACTIVATED"
    serialized = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    return ExplorationDecision(
        False,
        True,
        "EXPLORATION_ROUTE_ACTIVATED",
        plan.selected_exit_id,
        plan.selected_exit_position_world,
        plan.selected_approach_position_world,
        serialized,
    )


def nearest_exit_obstacle_candidate(
    candidates,
    exit_position_world,
    approach_position_world,
    maximum_distance_m: float,
) -> tuple[float, float] | None:
    """Choose only a LiDAR candidate geometrically associated with the exit."""
    radius = float(maximum_distance_m)
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("maximum_distance_m must be finite and positive")
    anchors = [
        tuple(map(float, point))
        for point in (exit_position_world, approach_position_world)
        if point is not None
    ]
    if not anchors:
        return None
    matches = []
    for point in candidates:
        candidate = tuple(map(float, point))
        if len(candidate) != 2 or not all(math.isfinite(item) for item in candidate):
            continue
        distance = min(math.dist(candidate, anchor) for anchor in anchors)
        if distance <= radius:
            matches.append((distance, candidate))
    return None if not matches else min(matches)[1]


def nearest_uninspected_candidate(
    candidates,
    robot_position_world,
    inspected_positions=(),
    suppression_radius_m: float = 1.0,
) -> tuple[float, float] | None:
    """Choose the closest finite candidate that has not already been checked.

    The Mode 5 orchestrator uses this before it changes drive mode, so one
    target is locked for the whole approach/classification transaction.  A
    spatial suppression radius prevents a stationary red obstacle from
    immediately starting the same inspection again after Mode 5 resumes.
    """
    radius = float(suppression_radius_m)
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("suppression_radius_m must be finite and positive")
    try:
        robot = tuple(float(value) for value in robot_position_world)
    except (TypeError, ValueError):
        return None
    if len(robot) != 2 or not all(math.isfinite(value) for value in robot):
        return None
    inspected = []
    for value in inspected_positions:
        try:
            point = tuple(float(item) for item in value)
        except (TypeError, ValueError):
            continue
        if len(point) == 2 and all(math.isfinite(item) for item in point):
            inspected.append(point)
    eligible = []
    for value in candidates:
        try:
            point = tuple(float(item) for item in value)
        except (TypeError, ValueError):
            continue
        if len(point) != 2 or not all(math.isfinite(item) for item in point):
            continue
        if any(math.dist(point, old) <= radius for old in inspected):
            continue
        eligible.append((math.dist(robot, point), point))
    return None if not eligible else min(eligible)[1]


def parse_mode3_classification(value: str):
    """Return ``(kind, (x, y))`` for a completed Mode 3 result."""
    parts = str(value).strip().split(":", 1)
    if len(parts) != 2 or parts[0] not in {"PERSON", "DYNAMIC_OBSTACLE"}:
        return None
    try:
        coordinates = tuple(float(item) for item in parts[1].split(","))
    except ValueError:
        return None
    if len(coordinates) != 2 or not all(math.isfinite(item) for item in coordinates):
        return None
    return parts[0], coordinates


def parse_mode4_classification(
    value: str, inspection_target: tuple[float, float] | None = None
):
    """Return Mode 4's result, selecting the confirmed point nearest the target."""
    payload = str(value).strip()
    if payload == "NO_SURVIVOR":
        return "NO_SURVIVOR", None
    if not payload.startswith("SURVIVOR:"):
        return None
    points = []
    for raw in payload.split(":", 1)[1].split(";"):
        try:
            x, y, _votes = (float(item) for item in raw.split(","))
        except (TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))
    if not points:
        return None
    if inspection_target is None:
        return "SURVIVOR", points[0]
    target = tuple(map(float, inspection_target))
    return "SURVIVOR", min(points, key=lambda point: math.dist(point, target))


@dataclass(frozen=True)
class ActivationResult:
    activated: bool
    exit_id: str | None
    reason: str


def parse_activation_response(success: bool, message: str) -> ActivationResult:
    """Validate that exit selection produced and activated a real route."""
    if not success:
        return ActivationResult(False, None, str(message) or "PLAN_SERVICE_FAILED")
    try:
        payload = json.loads(message)
    except (TypeError, ValueError):
        return ActivationResult(False, None, "INVALID_PLAN_RESPONSE")
    if not isinstance(payload, dict):
        return ActivationResult(False, None, "INVALID_PLAN_RESPONSE")
    if payload.get("success") is not True:
        return ActivationResult(
            False, None, str(payload.get("status", "NO_REACHABLE_EXIT"))
        )
    exit_id = payload.get("selected_exit_id")
    if not isinstance(exit_id, str) or not exit_id.strip():
        return ActivationResult(False, None, "SELECTED_EXIT_MISSING")
    if payload.get("activated") is not True:
        return ActivationResult(False, exit_id, "SELECTED_ROUTE_NOT_ACTIVATED")
    return ActivationResult(True, exit_id, "SELECTED_ROUTE_ACTIVATED")
