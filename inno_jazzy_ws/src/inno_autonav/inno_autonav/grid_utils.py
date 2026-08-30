"""Occupancy-grid loading, conversion, inflation, and path geometry helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np
from PIL import Image
import yaml


class GridError(RuntimeError):
    """Raised when map or grid data is invalid."""


@dataclass(frozen=True)
class MapGrid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    frame_id: str
    data: np.ndarray


def load_map_yaml(path: str | Path) -> Tuple[Path, Dict]:
    yaml_path = Path(path).expanduser().resolve(strict=False)
    if not yaml_path.is_file():
        raise GridError(f'map YAML 파일이 없습니다: {yaml_path}')
    try:
        metadata = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise GridError(f'map YAML을 읽을 수 없습니다 ({yaml_path}): {exc}') from exc
    if not isinstance(metadata, dict):
        raise GridError('map YAML 최상위 값은 mapping이어야 합니다.')
    required = (
        'image', 'resolution', 'origin', 'negate',
        'occupied_thresh', 'free_thresh',
    )
    missing = [key for key in required if key not in metadata]
    if missing:
        raise GridError(f'map YAML 필수 항목 누락: {", ".join(missing)}')
    return yaml_path, metadata


def load_pgm_as_occupancy(path: str | Path, frame_id: str = 'map') -> MapGrid:
    yaml_path, metadata = load_map_yaml(path)
    image_path = Path(str(metadata['image'])).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve(strict=False)
    if not image_path.is_file():
        raise GridError(f'PGM 이미지가 없습니다: {image_path}')

    try:
        resolution = float(metadata['resolution'])
        origin = metadata['origin']
        origin_x, origin_y = float(origin[0]), float(origin[1])
        origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0
        negate = int(metadata['negate'])
        occupied_thresh = float(metadata['occupied_thresh'])
        free_thresh = float(metadata['free_thresh'])
    except (TypeError, ValueError, IndexError) as exc:
        raise GridError('map YAML의 숫자 metadata가 올바르지 않습니다.') from exc
    if resolution <= 0.0 or negate not in (0, 1):
        raise GridError('resolution은 양수이고 negate는 0 또는 1이어야 합니다.')

    try:
        with Image.open(image_path) as source:
            pixels = np.asarray(source.convert('L'), dtype=np.uint8)
    except OSError as exc:
        raise GridError(f'PGM 이미지를 읽을 수 없습니다 ({image_path}): {exc}') from exc

    # PGM row 0 is the top; OccupancyGrid row 0 starts at map origin (bottom).
    pixels = np.flipud(pixels)
    if negate == 0:
        probability = (255.0 - pixels.astype(np.float32)) / 255.0
    else:
        probability = pixels.astype(np.float32) / 255.0
    occupancy = np.full(pixels.shape, -1, dtype=np.int8)
    occupancy[probability > occupied_thresh] = 100
    occupancy[probability < free_thresh] = 0
    height, width = occupancy.shape
    return MapGrid(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        frame_id=frame_id,
        data=occupancy,
    )


def world_to_grid(x: float, y: float, grid: MapGrid) -> Tuple[int, int]:
    # Current maps use origin yaw=0. Supporting a rotated origin here prevents silent errors.
    dx, dy = x - grid.origin_x, y - grid.origin_y
    cosine, sine = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    return int(math.floor(local_x / grid.resolution)), int(math.floor(local_y / grid.resolution))


def grid_to_world(grid_x: int, grid_y: int, grid: MapGrid) -> Tuple[float, float]:
    local_x = (grid_x + 0.5) * grid.resolution
    local_y = (grid_y + 0.5) * grid.resolution
    cosine, sine = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    return (
        grid.origin_x + cosine * local_x - sine * local_y,
        grid.origin_y + sine * local_x + cosine * local_y,
    )


def is_inside_grid(grid_x: int, grid_y: int, grid: MapGrid) -> bool:
    return 0 <= grid_x < grid.width and 0 <= grid_y < grid.height


def inflate_occupied_cells(data: np.ndarray, radius_cells: int) -> np.ndarray:
    source = np.asarray(data, dtype=np.int8)
    if source.ndim != 2:
        raise GridError('occupancy data는 2차원 배열이어야 합니다.')
    result = source.copy()
    if radius_cells <= 0:
        return result
    occupied = source >= 100
    inflated = occupied.copy()
    height, width = source.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            source_y0, source_y1 = max(0, -dy), min(height, height - dy)
            source_x0, source_x1 = max(0, -dx), min(width, width - dx)
            target_y0, target_y1 = source_y0 + dy, source_y1 + dy
            target_x0, target_x1 = source_x0 + dx, source_x1 + dx
            inflated[target_y0:target_y1, target_x0:target_x1] |= occupied[
                source_y0:source_y1, source_x0:source_x1
            ]
    result[inflated] = 100
    return result


def build_static_clearance_mask(
    static_data: np.ndarray,
    clearance_radius_m: float,
    resolution_m: float,
    *,
    unknown_is_occupied: bool = True,
) -> np.ndarray:
    """Return cells forbidden by the configured static-wall clearance."""
    static = np.asarray(static_data)
    if static.ndim != 2:
        raise ValueError('static occupancy data must be two-dimensional')
    if not math.isfinite(clearance_radius_m) or clearance_radius_m < 0.0:
        raise ValueError('static clearance radius must be finite and non-negative')
    if not math.isfinite(resolution_m) or resolution_m <= 0.0:
        raise ValueError('static grid resolution must be finite and positive')

    blocked = np.zeros(static.shape, dtype=np.int8)
    blocked[static >= 100] = 100
    if unknown_is_occupied:
        blocked[static < 0] = 100
    radius_cells = int(math.ceil(clearance_radius_m / resolution_m))
    return inflate_occupied_cells(blocked, radius_cells) >= 100


def apply_static_clearance_to_hazard_costs(
    hazard_costs: np.ndarray,
    static_clearance_mask: np.ndarray,
) -> np.ndarray:
    """Overlay static-wall clearance as impassable traversal costs."""
    costs = np.asarray(hazard_costs, dtype=float)
    blocked = np.asarray(static_clearance_mask, dtype=bool)
    if costs.ndim != 2:
        raise ValueError('hazard traversal costs must be two-dimensional')
    if blocked.shape != costs.shape:
        raise ValueError('static clearance geometry differs from hazard costs')
    result = costs.copy()
    result[blocked] = math.inf
    return result


def bresenham(start: Tuple[int, int], end: Tuple[int, int]) -> List[Tuple[int, int]]:
    x0, y0 = start
    x1, y1 = end
    points = []
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    step_x = 1 if x0 < x1 else -1
    step_y = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return points
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x0 += step_x
        if doubled <= dx:
            error += dx
            y0 += step_y


def path_cells_collision(
    cells: Iterable[Tuple[int, int]], data: np.ndarray, unknown_is_occupied: bool
) -> bool:
    height, width = data.shape
    for x, y in cells:
        if not (0 <= x < width and 0 <= y < height):
            return True
        value = int(data[y, x])
        if value >= 100 or (value < 0 and unknown_is_occupied):
            return True
    return False


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def yaw_from_quaternion(quaternion) -> float:
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
