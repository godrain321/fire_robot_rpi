"""Robust 2-D lidar to monocular-camera extrinsic calibration core.

The fixed transform convention used by this module is::

    p_camera = R_camera_lidar @ p_lidar + t_camera_lidar_m

Each checkerboard observation supplies its plane in the camera optical frame
and the RPLIDAR C1 returns points in the lidar scan plane (z=0).  A correct
extrinsic transform therefore makes every selected lidar point lie on the
corresponding camera-frame checkerboard plane.

Only NumPy and OpenCV are required.  In particular, the nonlinear solver does
not depend on SciPy so it can run on the Raspberry Pi deployment image.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import cv2
import numpy as np
import yaml


TRANSFORM_CONVENTION = (
    "p_camera = R_camera_lidar * p_lidar + t_camera_lidar_m"
)
DISTORTION_ORDER = ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6")
DEFAULT_CAMERA_FRAME = "camera_optical_frame"
DEFAULT_LIDAR_FRAME = "laser"
MINIMUM_VIEWS = 20


class ExtrinsicCalibrationError(RuntimeError):
    """Base error for unusable data or an unsuccessful calibration."""


class DegenerateGeometryError(ExtrinsicCalibrationError):
    """Raised when the observations do not constrain all transform axes."""


@dataclass(frozen=True)
class RationalCameraInfo:
    """Strictly validated ROS CameraInfo using the eight-term Rational model."""

    source_path: Path
    sha256: str
    camera_name: str
    image_size: tuple[int, int]
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray


@dataclass(frozen=True)
class PlaneObservation:
    """One checkerboard plane paired with its 2-D lidar returns."""

    pose_id: str
    board_normal_camera: np.ndarray
    board_offset_camera_m: float
    lidar_points_xy_m: np.ndarray


@dataclass(frozen=True)
class ObservationSet:
    """Validated observations and their declared frame identifiers."""

    observations: tuple[PlaneObservation, ...]
    camera_frame: str = DEFAULT_CAMERA_FRAME
    lidar_frame: str = DEFAULT_LIDAR_FRAME
    source_path: Path | None = None


@dataclass(frozen=True)
class SolverConfig:
    """Numerical and quality thresholds for robust finite-difference LM."""

    min_views: int = 20
    huber_delta_m: float = 0.015
    max_iterations: int = 120
    finite_difference_step: float = 1.0e-6
    step_tolerance: float = 1.0e-10
    cost_tolerance: float = 1.0e-12
    initial_damping: float = 1.0e-3
    max_jacobian_condition_number: float = 1.0e8
    max_rmse_m: float = 0.020
    max_pose_rmse_m: float = 0.030
    rank_relative_tolerance: float = 1.0e-9

    def __post_init__(self) -> None:
        if self.min_views < MINIMUM_VIEWS:
            raise ValueError(f"min_views must be at least {MINIMUM_VIEWS}")
        if self.huber_delta_m <= 0 or not np.isfinite(self.huber_delta_m):
            raise ValueError("huber_delta_m must be positive and finite")
        if self.max_iterations <= 0:
            raise ValueError("max_iterations must be positive")
        if self.finite_difference_step <= 0:
            raise ValueError("finite_difference_step must be positive")
        if self.step_tolerance <= 0 or self.cost_tolerance <= 0:
            raise ValueError("solver tolerances must be positive")
        if self.initial_damping <= 0:
            raise ValueError("initial_damping must be positive")
        if self.max_jacobian_condition_number <= 1:
            raise ValueError("max_jacobian_condition_number must exceed 1")
        if self.max_rmse_m <= 0 or not np.isfinite(self.max_rmse_m):
            raise ValueError("max_rmse_m must be positive and finite")
        if self.max_pose_rmse_m <= 0 or not np.isfinite(self.max_pose_rmse_m):
            raise ValueError("max_pose_rmse_m must be positive and finite")
        if not 0 < self.rank_relative_tolerance < 1:
            raise ValueError("rank_relative_tolerance must be in (0, 1)")


@dataclass(frozen=True)
class ExtrinsicCalibrationResult:
    """Estimated lidar-to-camera transform and diagnostics."""

    rotation_camera_lidar: np.ndarray
    translation_camera_lidar_m: np.ndarray
    transform_camera_lidar: np.ndarray
    converged: bool
    iterations: int
    initial_cost: float
    final_cost: float
    raw_residuals_m: np.ndarray
    pose_rmse_m: tuple[float, ...]
    normal_rank: int
    normal_singular_values: np.ndarray
    jacobian_rank: int
    jacobian_singular_values: np.ndarray
    jacobian_condition_number: float

    @property
    def rmse_m(self) -> float:
        return float(np.sqrt(np.mean(np.square(self.raw_residuals_m))))

    @property
    def median_abs_residual_m(self) -> float:
        return float(np.median(np.abs(self.raw_residuals_m)))

    @property
    def max_abs_residual_m(self) -> float:
        return float(np.max(np.abs(self.raw_residuals_m)))


def _require_mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _matrix_field(
    payload: Mapping[str, Any], name: str, shape: tuple[int, int]
) -> np.ndarray:
    entry = _require_mapping(payload.get(name), name)
    if entry.get("rows") != shape[0] or entry.get("cols") != shape[1]:
        raise ValueError(
            f"{name} must declare rows={shape[0]} and cols={shape[1]}"
        )
    values = np.asarray(entry.get("data", []), dtype=np.float64)
    expected = shape[0] * shape[1]
    if values.size != expected or not np.all(np.isfinite(values)):
        raise ValueError(f"{name}.data must contain exactly {expected} finite values")
    return values.reshape(shape).copy()


def load_rational_camera_info(path: str | Path) -> RationalCameraInfo:
    """Load CameraInfo and reject anything except the fixed Rational-8 model."""

    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"camera_info.yaml does not exist: {source}")
    raw = source.read_bytes()
    payload = yaml.safe_load(raw.decode("utf-8"))
    root = _require_mapping(payload, "camera_info.yaml root")
    if root.get("distortion_model") != "rational_polynomial":
        raise ValueError("distortion_model must be exactly 'rational_polynomial'")

    width = root.get("image_width")
    height = root.get("image_height")
    if isinstance(width, bool) or isinstance(height, bool):
        raise ValueError("image_width and image_height must be positive integers")
    if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
        raise ValueError("image_width and image_height must be positive integers")

    camera_matrix = _matrix_field(root, "camera_matrix", (3, 3))
    distortion = _matrix_field(root, "distortion_coefficients", (1, 8)).reshape(8)
    if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
        raise ValueError("camera focal lengths fx and fy must be positive")
    if not np.allclose(camera_matrix[2], (0.0, 0.0, 1.0), atol=1.0e-9):
        raise ValueError("camera_matrix last row must be [0, 0, 1]")

    name = root.get("camera_name")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("camera_name must be a non-empty string")
    return RationalCameraInfo(
        source_path=source,
        sha256=hashlib.sha256(raw).hexdigest(),
        camera_name=name.strip(),
        image_size=(width, height),
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion,
    )


def _finite_vector(value: Any, length: int, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (length,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain exactly {length} finite values")
    return vector.copy()


def _observation_points(entry: Mapping[str, Any], pose_name: str) -> np.ndarray:
    points_value = entry.get("lidar_points_xy_m")
    allow_three_dimensions = False
    if points_value is None:
        lidar_line = entry.get("lidar_line")
        if isinstance(lidar_line, Mapping):
            points_value = lidar_line.get("inlier_points_lidar_m")
            allow_three_dimensions = True
    points = np.asarray(points_value, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] < 2:
        raise ValueError(f"{pose_name}.lidar_points_xy_m must contain at least 2 points")
    if points.shape[1] == 3 and allow_three_dimensions:
        if not np.allclose(points[:, 2], 0.0, atol=1.0e-6):
            raise ValueError(f"{pose_name} lidar points must lie in the z=0 scan plane")
        points = points[:, :2]
    if points.shape[1] != 2 or not np.all(np.isfinite(points)):
        raise ValueError(
            f"{pose_name}.lidar_points_xy_m must have shape (N, 2) and be finite"
        )
    if float(np.ptp(points[:, 0]) + np.ptp(points[:, 1])) <= 1.0e-8:
        raise ValueError(f"{pose_name} lidar points do not span a visible board segment")
    return points.copy()


def load_observations_json(
    path: str | Path,
    *,
    min_views: int = 20,
    camera_info: RationalCameraInfo | None = None,
) -> ObservationSet:
    """Load and normalize checkerboard/lidar observations from JSON.

    Canonical per-pose keys are ``board_normal_camera``,
    ``board_offset_camera_m`` and ``lidar_points_xy_m``.  The capture GUI's
    nested ``lidar_line.inlier_points_lidar_m`` is accepted as a compatibility
    fallback, but canonical writers should always emit ``lidar_points_xy_m``.
    """

    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"observations JSON does not exist: {source}")
    if min_views < MINIMUM_VIEWS:
        raise ValueError(f"min_views must be at least {MINIMUM_VIEWS}")
    root = _require_mapping(
        json.loads(source.read_text(encoding="utf-8")), "observations JSON root"
    )
    schema_version = root.get("schema_version", 1)
    if schema_version != 1:
        raise ValueError("observations schema_version must be 1")
    convention = root.get("transform_convention")
    if convention is not None and convention != TRANSFORM_CONVENTION:
        raise ValueError(
            f"transform_convention must be exactly: {TRANSFORM_CONVENTION}"
        )
    if camera_info is not None:
        declared_hash = root.get("camera_info_sha256")
        if declared_hash is not None and declared_hash != camera_info.sha256:
            raise ValueError(
                "observations were generated with a different camera_info.yaml"
            )
        declared_model = root.get("distortion_model")
        if declared_model is not None and declared_model != "rational_polynomial":
            raise ValueError("observations distortion_model must be rational_polynomial")

    frames = root.get("frames", {})
    if frames is None:
        frames = {}
    frames = _require_mapping(frames, "frames")
    top_camera_frame = root.get("camera_frame")
    top_lidar_frame = root.get("lidar_frame")
    nested_camera_frame = frames.get("camera")
    nested_lidar_frame = frames.get("lidar")
    if (
        top_camera_frame is not None
        and nested_camera_frame is not None
        and top_camera_frame != nested_camera_frame
    ):
        raise ValueError("camera_frame disagrees with frames.camera")
    if (
        top_lidar_frame is not None
        and nested_lidar_frame is not None
        and top_lidar_frame != nested_lidar_frame
    ):
        raise ValueError("lidar_frame disagrees with frames.lidar")
    camera_frame = (
        top_camera_frame
        if top_camera_frame is not None
        else nested_camera_frame
        if nested_camera_frame is not None
        else DEFAULT_CAMERA_FRAME
    )
    lidar_frame = (
        top_lidar_frame
        if top_lidar_frame is not None
        else nested_lidar_frame
        if nested_lidar_frame is not None
        else DEFAULT_LIDAR_FRAME
    )
    if not isinstance(camera_frame, str) or not camera_frame.strip():
        raise ValueError("camera_frame must be a non-empty string")
    if not isinstance(lidar_frame, str) or not lidar_frame.strip():
        raise ValueError("lidar_frame must be a non-empty string")
    camera_frame = camera_frame.strip()
    lidar_frame = lidar_frame.strip()
    if camera_frame == lidar_frame:
        raise ValueError("camera_frame and lidar_frame must be different")

    entries = root.get("observations")
    if not isinstance(entries, list):
        raise ValueError("observations must be a list")
    if len(entries) < min_views:
        raise ValueError(
            f"at least {min_views} observations are required; received {len(entries)}"
        )

    observations: list[PlaneObservation] = []
    pose_ids: set[str] = set()
    for index, raw_entry in enumerate(entries):
        entry = _require_mapping(raw_entry, f"observations[{index}]")
        if entry.get("camera_frame", camera_frame) != camera_frame:
            raise ValueError(
                f"observations[{index}].camera_frame disagrees with root frame"
            )
        if entry.get("lidar_frame", lidar_frame) != lidar_frame:
            raise ValueError(
                f"observations[{index}].lidar_frame disagrees with root frame"
            )
        pose_id_value = entry.get("pose_id", f"pose_{index:03d}")
        pose_id = str(pose_id_value).strip()
        if not pose_id or pose_id in pose_ids:
            raise ValueError(f"observation pose_id must be non-empty and unique: {pose_id!r}")
        pose_ids.add(pose_id)
        normal = _finite_vector(
            entry.get("board_normal_camera"),
            3,
            f"observations[{index}].board_normal_camera",
        )
        normal_norm = float(np.linalg.norm(normal))
        if normal_norm <= 1.0e-9:
            raise ValueError(f"observations[{index}] board normal cannot be zero")
        try:
            offset = float(entry.get("board_offset_camera_m"))
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"observations[{index}].board_offset_camera_m must be finite"
            ) from exc
        if not np.isfinite(offset):
            raise ValueError(
                f"observations[{index}].board_offset_camera_m must be finite"
            )
        # Preserve the same geometric plane while enforcing a unit normal.
        normal /= normal_norm
        offset /= normal_norm
        observations.append(
            PlaneObservation(
                pose_id=pose_id,
                board_normal_camera=normal,
                board_offset_camera_m=offset,
                lidar_points_xy_m=_observation_points(entry, f"observations[{index}]"),
            )
        )

    return ObservationSet(
        observations=tuple(observations),
        camera_frame=camera_frame,
        lidar_frame=lidar_frame,
        source_path=source,
    )


def _singular_rank(
    matrix: np.ndarray, relative_tolerance: float
) -> tuple[int, np.ndarray, float]:
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    if singular_values.size == 0 or singular_values[0] <= 0:
        return 0, singular_values, float("inf")
    threshold = singular_values[0] * relative_tolerance
    rank = int(np.count_nonzero(singular_values > threshold))
    condition = (
        float(singular_values[0] / singular_values[-1])
        if rank == min(matrix.shape) and singular_values[-1] > 0
        else float("inf")
    )
    return rank, singular_values, condition


def _normal_geometry(
    observations: Sequence[PlaneObservation], relative_tolerance: float
) -> tuple[int, np.ndarray]:
    normals = np.stack([item.board_normal_camera for item in observations])
    rank, singular_values, _ = _singular_rank(normals, relative_tolerance)
    if rank < 3:
        raise DegenerateGeometryError(
            "checkerboard normals have rank < 3; capture tilted boards spanning all axes"
        )
    return rank, singular_values


def _linear_initialization(
    observations: Sequence[PlaneObservation], relative_tolerance: float
) -> np.ndarray:
    rows: list[np.ndarray] = []
    targets: list[float] = []
    for item in observations:
        normal = item.board_normal_camera
        pose_weight = 1.0 / np.sqrt(item.lidar_points_xy_m.shape[0])
        for x_value, y_value in item.lidar_points_xy_m:
            rows.append(
                pose_weight
                * np.concatenate((x_value * normal, y_value * normal, normal))
            )
            targets.append(pose_weight * -item.board_offset_camera_m)
    design = np.stack(rows)
    target = np.asarray(targets, dtype=np.float64)
    design_rank, _, _ = _singular_rank(design, relative_tolerance)
    if design_rank < 9:
        raise DegenerateGeometryError(
            "linear initialization matrix has rank < 9; vary board distance, tilt, and position"
        )
    solution, _, _, _ = np.linalg.lstsq(design, target, rcond=None)
    first_column = solution[0:3]
    second_column = solution[3:6]
    approximate_rotation = np.column_stack(
        (first_column, second_column, np.cross(first_column, second_column))
    )
    u_matrix, _, vt_matrix = np.linalg.svd(approximate_rotation)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(u_matrix @ vt_matrix)
    rotation = u_matrix @ correction @ vt_matrix

    # Once R is on SO(3), re-solve translation with equal pose influence.
    translation_rows: list[np.ndarray] = []
    translation_targets: list[float] = []
    for item in observations:
        normal = item.board_normal_camera
        pose_weight = 1.0 / np.sqrt(item.lidar_points_xy_m.shape[0])
        for x_value, y_value in item.lidar_points_xy_m:
            point_lidar = np.array((x_value, y_value, 0.0))
            translation_rows.append(pose_weight * normal)
            translation_targets.append(
                pose_weight
                * (-item.board_offset_camera_m - normal @ rotation @ point_lidar)
            )
    translation, _, translation_rank, _ = np.linalg.lstsq(
        np.stack(translation_rows), np.asarray(translation_targets), rcond=None
    )
    if translation_rank < 3:
        raise DegenerateGeometryError("board normals do not constrain 3-D translation")
    rotation_vector, _ = cv2.Rodrigues(rotation)
    return np.concatenate((rotation_vector.reshape(3), translation.reshape(3)))


def _rotation_and_translation(parameters: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rotation, _ = cv2.Rodrigues(np.asarray(parameters[:3], dtype=np.float64))
    return rotation, np.asarray(parameters[3:6], dtype=np.float64)


def _raw_residuals(
    parameters: np.ndarray, observations: Sequence[PlaneObservation]
) -> np.ndarray:
    rotation, translation = _rotation_and_translation(parameters)
    residual_parts: list[np.ndarray] = []
    for item in observations:
        points_lidar = np.column_stack(
            (
                item.lidar_points_xy_m,
                np.zeros(item.lidar_points_xy_m.shape[0], dtype=np.float64),
            )
        )
        points_camera = points_lidar @ rotation.T + translation
        residual_parts.append(
            points_camera @ item.board_normal_camera + item.board_offset_camera_m
        )
    return np.concatenate(residual_parts)


def _pose_weights(observations: Sequence[PlaneObservation]) -> np.ndarray:
    return np.concatenate(
        [
            np.full(item.lidar_points_xy_m.shape[0], 1.0 / np.sqrt(item.lidar_points_xy_m.shape[0]))
            for item in observations
        ]
    )


def _huber_cost(
    residuals: np.ndarray,
    observations: Sequence[PlaneObservation],
    delta: float,
) -> float:
    absolute = np.abs(residuals)
    losses = np.where(
        absolute <= delta,
        0.5 * np.square(residuals),
        delta * (absolute - 0.5 * delta),
    )
    weights = np.square(_pose_weights(observations))
    return float(np.sum(weights * losses))


def _finite_difference_jacobian(
    parameters: np.ndarray,
    observations: Sequence[PlaneObservation],
    step: float,
) -> np.ndarray:
    row_count = sum(item.lidar_points_xy_m.shape[0] for item in observations)
    jacobian = np.empty((row_count, 6), dtype=np.float64)
    for column in range(6):
        delta = step * max(1.0, abs(float(parameters[column])))
        plus = parameters.copy()
        minus = parameters.copy()
        plus[column] += delta
        minus[column] -= delta
        jacobian[:, column] = (
            _raw_residuals(plus, observations) - _raw_residuals(minus, observations)
        ) / (2.0 * delta)
    return jacobian


def calibrate_extrinsic(
    observation_set: ObservationSet | Sequence[PlaneObservation],
    config: SolverConfig | None = None,
) -> ExtrinsicCalibrationResult:
    """Estimate ``T_camera_lidar`` with pose-balanced robust LM."""

    settings = config or SolverConfig()
    observations = (
        observation_set.observations
        if isinstance(observation_set, ObservationSet)
        else tuple(observation_set)
    )
    if len(observations) < settings.min_views:
        raise ValueError(
            f"at least {settings.min_views} observations are required; received {len(observations)}"
        )
    normal_rank, normal_singular_values = _normal_geometry(
        observations, settings.rank_relative_tolerance
    )
    parameters = _linear_initialization(observations, settings.rank_relative_tolerance)
    residuals = _raw_residuals(parameters, observations)
    initial_cost = _huber_cost(residuals, observations, settings.huber_delta_m)
    cost = initial_cost
    damping = settings.initial_damping
    pose_weight = _pose_weights(observations)
    converged = False
    iteration_count = 0

    for iteration in range(1, settings.max_iterations + 1):
        iteration_count = iteration
        residuals = _raw_residuals(parameters, observations)
        jacobian = _finite_difference_jacobian(
            parameters, observations, settings.finite_difference_step
        )
        absolute = np.abs(residuals)
        huber_weight = np.ones_like(residuals)
        outside = absolute > settings.huber_delta_m
        huber_weight[outside] = settings.huber_delta_m / absolute[outside]
        row_scale = pose_weight * np.sqrt(huber_weight)
        weighted_jacobian = jacobian * row_scale[:, None]
        weighted_residual = residuals * row_scale
        normal_matrix = weighted_jacobian.T @ weighted_jacobian
        gradient = weighted_jacobian.T @ weighted_residual
        diagonal = np.maximum(np.diag(normal_matrix), 1.0e-12)
        accepted = False
        previous_cost = cost

        for _ in range(12):
            try:
                step_vector = np.linalg.solve(
                    normal_matrix + damping * np.diag(diagonal), -gradient
                )
            except np.linalg.LinAlgError as exc:
                raise DegenerateGeometryError(
                    "LM normal matrix is singular; capture more varied board poses"
                ) from exc
            candidate = parameters + step_vector
            candidate_residuals = _raw_residuals(candidate, observations)
            candidate_cost = _huber_cost(
                candidate_residuals, observations, settings.huber_delta_m
            )
            if np.isfinite(candidate_cost) and candidate_cost < cost:
                parameters = candidate
                cost = candidate_cost
                damping = max(damping / 3.0, 1.0e-12)
                accepted = True
                break
            damping = min(damping * 10.0, 1.0e15)

        if not accepted:
            if float(np.linalg.norm(gradient, ord=np.inf)) < 1.0e-10:
                converged = True
                break
            raise ExtrinsicCalibrationError(
                "LM could not find a cost-reducing step; review point/plane associations"
            )
        if float(np.linalg.norm(step_vector)) <= settings.step_tolerance * (
            float(np.linalg.norm(parameters)) + settings.step_tolerance
        ):
            converged = True
            break
        if abs(previous_cost - cost) <= settings.cost_tolerance * max(1.0, previous_cost):
            converged = True
            break

    if not converged:
        raise ExtrinsicCalibrationError(
            f"LM did not converge within {settings.max_iterations} iterations"
        )

    final_residuals = _raw_residuals(parameters, observations)
    final_jacobian = _finite_difference_jacobian(
        parameters, observations, settings.finite_difference_step
    ) * pose_weight[:, None]
    jacobian_rank, jacobian_singular_values, jacobian_condition = _singular_rank(
        final_jacobian, settings.rank_relative_tolerance
    )
    if jacobian_rank < 6:
        raise DegenerateGeometryError(
            "final Jacobian has rank < 6; the full lidar-to-camera transform is unobservable"
        )
    if (
        not np.isfinite(jacobian_condition)
        or jacobian_condition > settings.max_jacobian_condition_number
    ):
        raise DegenerateGeometryError(
            "final Jacobian is ill-conditioned "
            f"({jacobian_condition:.3g} > {settings.max_jacobian_condition_number:.3g})"
        )

    rotation, translation = _rotation_and_translation(parameters)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    pose_rmse: list[float] = []
    cursor = 0
    for item in observations:
        count = item.lidar_points_xy_m.shape[0]
        values = final_residuals[cursor : cursor + count]
        pose_rmse.append(float(np.sqrt(np.mean(np.square(values)))))
        cursor += count

    final_rmse = float(np.sqrt(np.mean(np.square(final_residuals))))
    worst_pose_rmse = max(pose_rmse)
    if final_rmse > settings.max_rmse_m:
        raise ExtrinsicCalibrationError(
            f"final plane RMSE {final_rmse:.6f} m exceeds "
            f"{settings.max_rmse_m:.6f} m"
        )
    if worst_pose_rmse > settings.max_pose_rmse_m:
        worst_index = int(np.argmax(np.asarray(pose_rmse)))
        raise ExtrinsicCalibrationError(
            f"pose {observations[worst_index].pose_id!r} RMSE "
            f"{worst_pose_rmse:.6f} m exceeds {settings.max_pose_rmse_m:.6f} m"
        )
    return ExtrinsicCalibrationResult(
        rotation_camera_lidar=rotation,
        translation_camera_lidar_m=translation,
        transform_camera_lidar=transform,
        converged=converged,
        iterations=iteration_count,
        initial_cost=initial_cost,
        final_cost=cost,
        raw_residuals_m=final_residuals,
        pose_rmse_m=tuple(pose_rmse),
        normal_rank=normal_rank,
        normal_singular_values=normal_singular_values,
        jacobian_rank=jacobian_rank,
        jacobian_singular_values=jacobian_singular_values,
        jacobian_condition_number=jacobian_condition,
    )


def result_payload(
    result: ExtrinsicCalibrationResult,
    observation_set: ObservationSet,
    camera_info: RationalCameraInfo,
    config: SolverConfig,
) -> dict[str, Any]:
    """Build the stable YAML/JSON transform payload."""

    quality = {
        "passed": True,
        "pose_count": len(observation_set.observations),
        "lidar_point_count": int(result.raw_residuals_m.size),
        "normal_rank": result.normal_rank,
        "normal_singular_values": result.normal_singular_values.tolist(),
        "jacobian_rank": result.jacobian_rank,
        "jacobian_singular_values": result.jacobian_singular_values.tolist(),
        "jacobian_condition_number": result.jacobian_condition_number,
        "max_allowed_jacobian_condition_number": config.max_jacobian_condition_number,
        "rmse_m": result.rmse_m,
        "max_allowed_rmse_m": config.max_rmse_m,
        "max_allowed_pose_rmse_m": config.max_pose_rmse_m,
        "median_abs_residual_m": result.median_abs_residual_m,
        "max_abs_residual_m": result.max_abs_residual_m,
    }
    return {
        "schema_version": 1,
        "calibration_type": "2d_lidar_to_monocular_camera_extrinsic",
        "transform_convention": TRANSFORM_CONVENTION,
        "frames": {
            "camera": observation_set.camera_frame,
            "lidar": observation_set.lidar_frame,
        },
        "T_camera_lidar": result.transform_camera_lidar.tolist(),
        "R_camera_lidar": result.rotation_camera_lidar.tolist(),
        "t_camera_lidar_m": result.translation_camera_lidar_m.tolist(),
        "camera_intrinsics": {
            "camera_info_path": str(camera_info.source_path),
            "camera_info_sha256": camera_info.sha256,
            "camera_name": camera_info.camera_name,
            "image_width": camera_info.image_size[0],
            "image_height": camera_info.image_size[1],
            "distortion_model": "rational_polynomial",
            "distortion_coefficient_order": list(DISTORTION_ORDER),
            "camera_matrix": camera_info.camera_matrix.tolist(),
            "distortion_coefficients": camera_info.distortion_coefficients.tolist(),
        },
        "optimization": {
            "method": "pose-balanced Huber finite-difference Levenberg-Marquardt",
            "huber_delta_m": config.huber_delta_m,
            "minimum_views": config.min_views,
            "iterations": result.iterations,
            "converged": result.converged,
            "initial_cost": result.initial_cost,
            "final_cost": result.final_cost,
        },
        "quality": quality,
    }


__all__ = [
    "DEFAULT_CAMERA_FRAME",
    "DEFAULT_LIDAR_FRAME",
    "DISTORTION_ORDER",
    "TRANSFORM_CONVENTION",
    "DegenerateGeometryError",
    "ExtrinsicCalibrationError",
    "MINIMUM_VIEWS",
    "ExtrinsicCalibrationResult",
    "ObservationSet",
    "PlaneObservation",
    "RationalCameraInfo",
    "SolverConfig",
    "calibrate_extrinsic",
    "load_observations_json",
    "load_rational_camera_info",
    "result_payload",
]
