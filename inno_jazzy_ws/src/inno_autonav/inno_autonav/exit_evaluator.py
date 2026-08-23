"""ROS-independent exit evaluation matching factory_v5 semantics."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import math
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np
import yaml

from inno_hazard.hazard_belief import HazardGridGeometry


Cell = tuple[int, int]
WorldPoint = tuple[float, float]


class ExitStatus(Enum):
    UNKNOWN = "unknown"
    USABLE = "usable"
    BLOCKED = "blocked"
    DANGEROUS = "dangerous"
    DANGER_EXPECTED = "danger_expected"


class ExitRejectionReason(Enum):
    EXIT_BLOCKED = "exit_blocked"
    EXIT_DANGEROUS = "exit_dangerous"
    EXIT_DANGER_EXPECTED = "exit_danger_expected"
    INVALID_EXIT_POSITION = "invalid_exit_position"
    NO_APPROACH_CELL = "no_approach_cell"
    NO_PATH = "no_path"
    STATIC_OBSTACLE = "static_obstacle"
    DYNAMIC_OBSTACLE = "dynamic_obstacle"
    TEMPERATURE_LIMIT_EXCEEDED = "temperature_limit_exceeded"
    CO_LIMIT_EXCEEDED = "co_limit_exceeded"
    PATH_RISK_COST_EXCEEDED = "path_risk_cost_exceeded"
    INVALID_COST = "invalid_cost"
    OUT_OF_MAP = "out_of_map"


@dataclass(frozen=True)
class ExitItem:
    exit_id: str
    position_world: WorldPoint
    approach_position_world: WorldPoint | None = None
    status: ExitStatus = ExitStatus.UNKNOWN

    def __post_init__(self) -> None:
        if not self.exit_id:
            raise ValueError("exit_id must not be empty")
        for point in (self.position_world, self.approach_position_world):
            if point is not None and (
                len(point) != 2
                or not all(math.isfinite(float(value)) for value in point)
            ):
                raise ValueError("exit positions must be finite (x, y) points")
        if not isinstance(self.status, ExitStatus):
            raise TypeError("status must be ExitStatus")


def load_exit_registry(path: str, expected_frame: str = "map") -> tuple[ExitItem, ...]:
    """Load category/name exits from the existing semantic-points source."""
    source = Path(path).expanduser().resolve(strict=False)
    if not source.is_file():
        raise ValueError(f"exit registry YAML does not exist: {source}")
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("exit registry YAML must be a mapping")
    default_frame = str(document.get("frame_id", expected_frame))
    raw = document.get("semantic_points", document.get("poses", {}))
    if not isinstance(raw, dict):
        raise ValueError("semantic_points/poses must be a mapping")
    exits = []
    for name, value in raw.items():
        if not isinstance(value, dict):
            continue
        is_exit = (
            str(value.get("category", "")).casefold() == "exit"
            or str(name).casefold().startswith("exit")
        )
        if not is_exit:
            continue
        frame = str(value.get("frame_id", default_frame))
        if frame.lstrip("/") != expected_frame.lstrip("/"):
            raise ValueError(
                f"exit {name} frame {frame!r} differs from {expected_frame!r}"
            )
        try:
            marker = float(value["x"]), float(value["y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"exit {name} marker is invalid") from exc
        approach_value = value.get("approach")
        approach = None
        if approach_value is not None:
            if not isinstance(approach_value, dict):
                raise ValueError(f"exit {name} approach must be a mapping")
            try:
                approach = float(approach_value["x"]), float(approach_value["y"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"exit {name} approach is invalid") from exc
        try:
            status = ExitStatus(str(value.get("status", "unknown")).casefold())
        except ValueError as exc:
            raise ValueError(f"exit {name} status is invalid") from exc
        exits.append(ExitItem(str(name).upper(), marker, approach, status))
    if not exits:
        raise ValueError("exit registry contains no exits")
    return tuple(exits)


@dataclass(frozen=True)
class ExitEvaluationConfig:
    exit_neighborhood_radius_m: float = 1.0
    approach_search_radius_m: float = 1.0
    reject_blocked_exit: bool = True
    reject_dangerous_exit: bool = True
    reject_path_over_threshold: bool = True
    reject_invalid_cost: bool = True
    usable_confirmation_distance_m: float = 3.0
    dangerous_accumulated_risk_cost: float | None = None
    dangerous_average_risk_cost: float | None = None
    dangerous_max_cell_risk_cost: float | None = None

    def __post_init__(self) -> None:
        for name in (
            "reject_blocked_exit", "reject_dangerous_exit",
            "reject_path_over_threshold", "reject_invalid_cost",
        ):
            if not isinstance(getattr(self, name), bool):
                raise TypeError(f"{name} must be bool")
        for name, allow_zero in (
            ("exit_neighborhood_radius_m", True),
            ("approach_search_radius_m", False),
            ("usable_confirmation_distance_m", False),
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or value < 0.0
                or (not allow_zero and value == 0.0)
            ):
                raise ValueError(f"{name} has an invalid distance")
        for name in (
            "dangerous_accumulated_risk_cost",
            "dangerous_average_risk_cost",
            "dangerous_max_cell_risk_cost",
        ):
            value = getattr(self, name)
            if value is not None and (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be positive or None")

    @classmethod
    def from_mapping(cls, values: dict[str, Any] | None):
        values = dict(values or {})
        forbidden = {"temperature_block_threshold_c", "co_block_threshold_ppm"}
        if forbidden & set(values):
            raise ValueError("fire thresholds belong to HazardBelief")
        unknown = set(values) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f"unknown exit settings: {sorted(unknown)}")
        return cls(**values)


@dataclass(frozen=True)
class ExitHazardSnapshot:
    geometry: HazardGridGeometry
    final_cost: np.ndarray
    temperature_c: np.ndarray
    co_ppm: np.ndarray
    observed_mask: np.ndarray
    temperature_observed_mask: np.ndarray
    co_observed_mask: np.ndarray
    fire_probability: np.ndarray
    static_obstacle_map: np.ndarray
    dynamic_obstacle_map: np.ndarray
    blocked_mask: np.ndarray
    revision: int
    temperature_blocked_c: float
    co_blocked_ppm: float
    base_cost: float

    def __post_init__(self) -> None:
        expected = self.geometry.height, self.geometry.width
        names = (
            "final_cost", "temperature_c", "co_ppm", "observed_mask",
            "temperature_observed_mask", "co_observed_mask",
            "fire_probability", "static_obstacle_map",
            "dynamic_obstacle_map", "blocked_mask",
        )
        for name in names:
            value = np.asarray(getattr(self, name))
            if value.shape != expected:
                raise ValueError(f"{name} shape must be {expected}")
            copy = value.astype(
                bool if name.endswith("mask") or name.endswith("map")
                and name in {
                    "static_obstacle_map", "dynamic_obstacle_map",
                    "blocked_mask",
                } else float,
                copy=True,
            )
            copy.setflags(write=False)
            object.__setattr__(self, name, copy)
        if self.revision < 0:
            raise ValueError("revision must be non-negative")
        if self.temperature_blocked_c < 0.0 or self.co_blocked_ppm < 0.0:
            raise ValueError("hazard thresholds must be non-negative")
        if not math.isfinite(self.base_cost) or self.base_cost <= 0.0:
            raise ValueError("base_cost must be finite and positive")


@dataclass(frozen=True)
class ExitEvaluation:
    exit_id: str
    exit_status: str
    exit_position_world: WorldPoint
    approach_position_world: WorldPoint | None
    approach_position_grid: Cell | None
    reachable: bool
    accepted: bool
    path_world: tuple[WorldPoint, ...]
    path_grid: tuple[Cell, ...]
    path_length_m: float | None
    accumulated_risk_cost: float | None
    max_path_temperature_c: float | None
    max_path_co_ppm: float | None
    exit_temperature_c: float | None
    exit_co_ppm: float | None
    unknown_ratio: float | None
    rejection_reasons: tuple[ExitRejectionReason, ...]
    evaluated_at: float
    reference_waypoint_ids: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["rejection_reasons"] = [item.value for item in self.rejection_reasons]
        return result

    @classmethod
    def from_dict(cls, value: dict[str, Any]):
        if not isinstance(value, dict):
            raise ValueError("exit evaluation must be a mapping")
        missing = set(cls.__dataclass_fields__) - set(value)
        if missing:
            raise ValueError(f"exit evaluation fields missing: {sorted(missing)}")

        def point(name, *, integer=False, optional=False):
            raw = value[name]
            if raw is None and optional:
                return None
            if not isinstance(raw, (list, tuple)) or len(raw) != 2:
                raise ValueError(f"{name} must be an (x, y) point")
            output = tuple((int if integer else float)(item) for item in raw)
            if not integer and not all(math.isfinite(item) for item in output):
                raise ValueError(f"{name} must be finite")
            return output

        def optional_number(name):
            raw = value[name]
            if raw is None:
                return None
            result = float(raw)
            if not math.isfinite(result):
                raise ValueError(f"{name} must be finite or null")
            return result

        exit_id = str(value["exit_id"]).strip()
        if not exit_id:
            raise ValueError("exit_id must not be empty")
        status = ExitStatus(str(value["exit_status"]))
        reasons = tuple(ExitRejectionReason(str(item)) for item in value["rejection_reasons"])
        path_world = tuple(
            tuple(map(float, item)) for item in value["path_world"]
        )
        path_grid = tuple(
            tuple(map(int, item)) for item in value["path_grid"]
        )
        if any(len(item) != 2 or not all(math.isfinite(v) for v in item)
               for item in path_world) or any(len(item) != 2 for item in path_grid):
            raise ValueError("evaluation path contains an invalid point")
        evaluated_at = float(value["evaluated_at"])
        if not math.isfinite(evaluated_at):
            raise ValueError("evaluated_at must be finite")
        if not isinstance(value["reachable"], bool) or not isinstance(
            value["accepted"], bool
        ):
            raise ValueError("reachable and accepted must be bool")
        result = cls(
            exit_id, status.value, point("exit_position_world"),
            point("approach_position_world", optional=True),
            point("approach_position_grid", integer=True, optional=True),
            bool(value["reachable"]), bool(value["accepted"]),
            path_world, path_grid, optional_number("path_length_m"),
            optional_number("accumulated_risk_cost"),
            optional_number("max_path_temperature_c"),
            optional_number("max_path_co_ppm"),
            optional_number("exit_temperature_c"),
            optional_number("exit_co_ppm"), optional_number("unknown_ratio"),
            reasons, evaluated_at,
            tuple(str(item) for item in value["reference_waypoint_ids"]),
        )
        if result.accepted and (
            not result.reachable
            or result.approach_position_world is None
            or result.path_length_m is None
            or result.accumulated_risk_cost is None
        ):
            raise ValueError("accepted evaluation lacks reachable path metrics")
        return result


@dataclass(frozen=True)
class ExitEvaluationBatch:
    hazard_revision: int
    frame_id: str
    robot_position_world: WorldPoint
    evaluations: tuple[ExitEvaluation, ...]
    evaluated_at: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "hazard_revision": self.hazard_revision,
            "frame_id": self.frame_id,
            "robot_position_world": list(self.robot_position_world),
            "evaluated_at": self.evaluated_at,
            "evaluations": [item.to_dict() for item in self.evaluations],
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any], expected_frame="map"):
        if not isinstance(value, dict):
            raise ValueError("exit evaluation batch must be a mapping")
        required = {
            "hazard_revision", "frame_id", "robot_position_world",
            "evaluated_at", "evaluations",
        }
        if required - set(value):
            raise ValueError("exit evaluation batch fields are incomplete")
        revision = int(value["hazard_revision"])
        if revision < 0:
            raise ValueError("hazard revision must be non-negative")
        frame = str(value["frame_id"])
        if frame.lstrip("/") != str(expected_frame).lstrip("/"):
            raise ValueError("evaluation batch frame differs from planner frame")
        robot = tuple(map(float, value["robot_position_world"]))
        if len(robot) != 2 or not all(math.isfinite(item) for item in robot):
            raise ValueError("robot_position_world must be finite")
        created = float(value["evaluated_at"])
        if not math.isfinite(created):
            raise ValueError("evaluated_at must be finite")
        evaluations = tuple(
            ExitEvaluation.from_dict(item) for item in value["evaluations"]
        )
        return cls(revision, frame, robot, evaluations, created)


def within_usable_confirmation_distance(robot, approach, config) -> bool:
    first, second = tuple(map(float, robot)), tuple(map(float, approach))
    if len(first) != 2 or len(second) != 2 or not all(
        math.isfinite(value) for value in (*first, *second)
    ):
        raise ValueError("confirmation positions must be finite (x, y) points")
    return math.dist(first, second) <= config.usable_confirmation_distance_m + 1e-12


def exit_evaluator_readiness(
    snapshot, static_geometry, hazard_status, map_frame="map",
) -> str:
    """Validate that evaluation can use a real, coherent Stage 3 snapshot."""
    if snapshot is None or static_geometry is None:
        return "HAZARD_NOT_READY"
    if snapshot.geometry != static_geometry:
        return "HAZARD_GEOMETRY_MISMATCH"
    if snapshot.geometry.frame_id.lstrip("/") != str(map_frame).lstrip("/"):
        return "HAZARD_FRAME_MISMATCH"
    if hazard_status not in {"ACTIVE", "ACTIVE_THERMAL_ONLY"}:
        return "HAZARD_NOT_READY:" + (hazard_status or "NO_STATUS")
    return "READY"


class ExitEvaluator:
    """Evaluate exits without selecting one or changing a navigation goal."""

    def __init__(
        self, config: ExitEvaluationConfig | None = None,
        *, path_planner: Callable | None = None,
    ) -> None:
        self.config = config or ExitEvaluationConfig()
        if path_planner is None:
            raise ValueError("a Stage 2 path planner must be supplied")
        self.path_planner = path_planner

    def evaluate_all(
        self, exits: Sequence[ExitItem], start_position_world: WorldPoint,
        *, snapshot: ExitHazardSnapshot, evaluated_at: float,
    ) -> ExitEvaluationBatch:
        # One immutable snapshot object is deliberately shared by the batch.
        evaluations = tuple(
            self.evaluate(
                item, start_position_world, snapshot=snapshot,
                evaluated_at=evaluated_at,
            )
            for item in exits
        )
        return ExitEvaluationBatch(
            snapshot.revision, snapshot.geometry.frame_id,
            tuple(map(float, start_position_world)),
            evaluations, float(evaluated_at),
        )

    def evaluate(self, exit_item, start_position_world, *, snapshot, evaluated_at):
        reasons = []
        if exit_item.status is ExitStatus.BLOCKED and self.config.reject_blocked_exit:
            reasons.append(ExitRejectionReason.EXIT_BLOCKED)
        if exit_item.status is ExitStatus.DANGEROUS and self.config.reject_dangerous_exit:
            reasons.append(ExitRejectionReason.EXIT_DANGEROUS)
        if exit_item.status is ExitStatus.DANGER_EXPECTED and self.config.reject_dangerous_exit:
            reasons.append(ExitRejectionReason.EXIT_DANGER_EXPECTED)
        if snapshot.geometry.world_to_grid(*exit_item.position_world) is None:
            reasons.append(ExitRejectionReason.OUT_OF_MAP)
        if reasons:
            return self._rejected(exit_item, reasons, evaluated_at)

        start = snapshot.geometry.world_to_grid(*start_position_world)
        if start is None:
            return self._rejected(exit_item, [ExitRejectionReason.OUT_OF_MAP], evaluated_at)
        start_failure = self._cell_failure(start, snapshot)
        if start_failure is not None:
            return self._rejected(exit_item, [start_failure], evaluated_at)

        if exit_item.approach_position_world is not None:
            registered = snapshot.geometry.world_to_grid(*exit_item.approach_position_world)
            if registered is None:
                return self._rejected(exit_item, [ExitRejectionReason.OUT_OF_MAP], evaluated_at)
            failure = self._cell_failure(registered, snapshot)
            if failure is not None:
                return self._rejected(
                    exit_item, [failure], evaluated_at,
                    tuple(exit_item.approach_position_world), registered,
                )
        approach = self._resolve_approach(exit_item, start, snapshot)
        if approach is None:
            return self._rejected(exit_item, [ExitRejectionReason.NO_APPROACH_CELL], evaluated_at)
        approach_world, approach_grid, result = approach
        if not result.path:
            return self._rejected(
                exit_item, [ExitRejectionReason.NO_PATH], evaluated_at,
                approach_world, approach_grid,
            )

        path = tuple(result.path)
        path_world = tuple(snapshot.geometry.grid_to_world(*cell) for cell in path)
        length = self._path_length(path, snapshot.geometry.resolution)
        risk = self._risk_cost(path, snapshot.final_cost, snapshot.base_cost,
                               snapshot.geometry.resolution)
        maximum = self._max_cell_risk(path, snapshot.final_cost, snapshot.base_cost)
        average = risk / length if length > 1e-12 else maximum
        path_temp = self._finite_observed_max(
            snapshot.temperature_c, snapshot.temperature_observed_mask, path
        )
        path_co = self._finite_observed_max(
            snapshot.co_ppm, snapshot.co_observed_mask, path
        )
        observed = np.asarray([
            snapshot.observed_mask[row, col] for col, row in path
        ], dtype=bool)
        unknown_ratio = float((~observed).sum() / len(path))
        exit_temp, exit_co = self._exit_neighborhood_values(exit_item, snapshot)

        if not math.isfinite(risk) and self.config.reject_invalid_cost:
            reasons.append(ExitRejectionReason.INVALID_COST)
        elif self._risk_threshold_exceeded(risk, average, maximum):
            reasons.append(ExitRejectionReason.PATH_RISK_COST_EXCEEDED)
        if self.config.reject_path_over_threshold:
            if any(value is not None and value >= snapshot.temperature_blocked_c
                   for value in (path_temp, exit_temp)):
                reasons.append(ExitRejectionReason.TEMPERATURE_LIMIT_EXCEEDED)
            if any(value is not None and value >= snapshot.co_blocked_ppm
                   for value in (path_co, exit_co)):
                reasons.append(ExitRejectionReason.CO_LIMIT_EXCEEDED)
        return ExitEvaluation(
            exit_item.exit_id, exit_item.status.value,
            tuple(exit_item.position_world), approach_world, approach_grid,
            True, not reasons, path_world, path, length, risk,
            path_temp, path_co, exit_temp, exit_co, unknown_ratio,
            tuple(dict.fromkeys(reasons)), float(evaluated_at),
            tuple(result.reference_waypoint_ids),
        )

    def _resolve_approach(self, item, start, snapshot):
        registered = item.approach_position_world is not None
        if registered:
            grid = snapshot.geometry.world_to_grid(*item.approach_position_world)
            candidates = [(tuple(item.approach_position_world), grid)]
        else:
            center = snapshot.geometry.world_to_grid(*item.position_world)
            radius = int(math.ceil(
                self.config.approach_search_radius_m / snapshot.geometry.resolution
            ))
            candidates = []
            for row in range(center[1] - radius, center[1] + radius + 1):
                for col in range(center[0] - radius, center[0] + radius + 1):
                    if not (0 <= col < snapshot.geometry.width and 0 <= row < snapshot.geometry.height):
                        continue
                    distance_m = math.hypot(col - center[0], row - center[1]) * snapshot.geometry.resolution
                    if 0.0 < distance_m <= self.config.approach_search_radius_m + 1e-12:
                        candidates.append((snapshot.geometry.grid_to_world(col, row), (col, row)))
            candidates.sort(key=lambda value: (
                math.dist(value[0], item.position_world), value[1][1], value[1][0]
            ))
        best = None
        for world, grid in candidates:
            if grid is None or self._cell_failure(grid, snapshot) is not None:
                continue
            result = self.path_planner(snapshot, start, grid)
            if registered:
                return world, grid, result
            if result.path and (best is None or result.total_cost < best[2].total_cost):
                best = world, grid, result
        return best

    @staticmethod
    def _cell_failure(cell, snapshot):
        col, row = cell
        if not (0 <= col < snapshot.geometry.width and 0 <= row < snapshot.geometry.height):
            return ExitRejectionReason.OUT_OF_MAP
        if snapshot.static_obstacle_map[row, col]:
            return ExitRejectionReason.STATIC_OBSTACLE
        if snapshot.dynamic_obstacle_map[row, col]:
            return ExitRejectionReason.DYNAMIC_OBSTACLE
        if (snapshot.temperature_observed_mask[row, col]
                and snapshot.temperature_c[row, col] >= snapshot.temperature_blocked_c):
            return ExitRejectionReason.TEMPERATURE_LIMIT_EXCEEDED
        if (snapshot.co_observed_mask[row, col]
                and snapshot.co_ppm[row, col] >= snapshot.co_blocked_ppm):
            return ExitRejectionReason.CO_LIMIT_EXCEEDED
        if snapshot.blocked_mask[row, col] or not np.isfinite(snapshot.final_cost[row, col]):
            return ExitRejectionReason.INVALID_COST
        return None

    @staticmethod
    def _path_length(path, resolution):
        return float(sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) * resolution
            for a, b in zip(path, path[1:])
        ))

    @staticmethod
    def _risk_cost(path, costs, base_cost, resolution):
        total = 0.0
        for first, second in zip(path, path[1:]):
            distance = math.dist(first, second) * resolution
            first_risk = max(0.0, float(costs[first[1], first[0]]) - base_cost)
            second_risk = max(0.0, float(costs[second[1], second[0]]) - base_cost)
            total += distance * 0.5 * (first_risk + second_risk)
        return float(total)

    @staticmethod
    def _max_cell_risk(path, costs, base_cost):
        return float(max((
            max(0.0, float(costs[row, col]) - base_cost)
            for col, row in path
        ), default=0.0))

    def _risk_threshold_exceeded(self, accumulated, average, maximum):
        return any(limit is not None and value >= limit for value, limit in (
            (accumulated, self.config.dangerous_accumulated_risk_cost),
            (average, self.config.dangerous_average_risk_cost),
            (maximum, self.config.dangerous_max_cell_risk_cost),
        ))

    @staticmethod
    def _finite_observed_max(layer, mask, cells):
        values = [float(layer[row, col]) for col, row in cells
                  if mask[row, col] and np.isfinite(layer[row, col])]
        return max(values) if values else None

    def _exit_neighborhood_values(self, item, snapshot):
        center = snapshot.geometry.world_to_grid(*item.position_world)
        radius = int(math.ceil(
            self.config.exit_neighborhood_radius_m / snapshot.geometry.resolution
        ))
        cells = []
        for row in range(center[1] - radius, center[1] + radius + 1):
            for col in range(center[0] - radius, center[0] + radius + 1):
                if not (0 <= col < snapshot.geometry.width and 0 <= row < snapshot.geometry.height):
                    continue
                world = snapshot.geometry.grid_to_world(col, row)
                if math.dist(world, item.position_world) <= self.config.exit_neighborhood_radius_m + 1e-12:
                    cells.append((col, row))
        return (
            self._finite_observed_max(snapshot.temperature_c, snapshot.temperature_observed_mask, cells),
            self._finite_observed_max(snapshot.co_ppm, snapshot.co_observed_mask, cells),
        )

    @staticmethod
    def _rejected(item, reasons, evaluated_at, approach_world=None, approach_grid=None):
        return ExitEvaluation(
            item.exit_id, item.status.value, tuple(item.position_world),
            approach_world, approach_grid, False, False, (), (), None, None,
            None, None, None, None, None, tuple(dict.fromkeys(reasons)),
            float(evaluated_at), (),
        )
