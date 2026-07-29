"""Map/YAML loading, validation, coordinate conversion, and safe output helpers."""

from __future__ import annotations

import math
import os
from pathlib import Path
import tempfile
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

from PIL import Image
import yaml


MAP_METADATA_KEYS = (
    'resolution',
    'origin',
    'negate',
    'occupied_thresh',
    'free_thresh',
    'mode',
)


class MapToolsError(RuntimeError):
    """Raised for invalid inputs or failed map generation."""


def load_yaml(path: Path | str, label: str) -> Tuple[Path, Dict[str, Any]]:
    resolved = Path(path).expanduser().resolve(strict=False)
    if not resolved.is_file():
        raise MapToolsError(f'{label} 파일이 없습니다: {resolved}')
    try:
        with resolved.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise MapToolsError(f'{label} YAML을 읽을 수 없습니다 ({resolved}): {exc}') from exc
    if not isinstance(data, dict):
        raise MapToolsError(f'{label} YAML 최상위 값은 mapping이어야 합니다: {resolved}')
    return resolved, data


def load_map(map_yaml: Path | str) -> Tuple[Path, Dict[str, Any], Path, Image.Image]:
    yaml_path, metadata = load_yaml(map_yaml, 'map')
    missing = [key for key in ('image',) + MAP_METADATA_KEYS if key not in metadata]
    if missing:
        raise MapToolsError(f'map YAML 필수 항목이 없습니다: {", ".join(missing)}')

    try:
        resolution = float(metadata['resolution'])
        origin = metadata['origin']
        if resolution <= 0.0 or not math.isfinite(resolution):
            raise ValueError
        if not isinstance(origin, Sequence) or len(origin) < 2:
            raise ValueError
        origin_x = float(origin[0])
        origin_y = float(origin[1])
        if not math.isfinite(origin_x) or not math.isfinite(origin_y):
            raise ValueError
    except (TypeError, ValueError) as exc:
        raise MapToolsError('map resolution/origin 값이 올바르지 않습니다.') from exc

    image_path = Path(os.path.expandvars(os.path.expanduser(str(metadata['image']))))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve(strict=False)
    if not image_path.is_file():
        raise MapToolsError(f'map 이미지 파일이 없습니다: {image_path}')
    try:
        with Image.open(image_path) as source:
            image = source.convert('L').copy()
    except (OSError, ValueError) as exc:
        raise MapToolsError(f'map 이미지를 읽을 수 없습니다 ({image_path}): {exc}') from exc
    return yaml_path, metadata, image_path, image


def map_to_pixel(
    map_x: float,
    map_y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
    image_height: int,
) -> Tuple[int, int]:
    pixel_x = int((map_x - origin_x) / resolution)
    pixel_y = int(image_height - 1 - ((map_y - origin_y) / resolution))
    return pixel_x, pixel_y


def pixel_to_map(
    pixel_x: int,
    pixel_y: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
    image_height: int,
) -> Tuple[float, float]:
    map_x = origin_x + pixel_x * resolution
    map_y = origin_y + (image_height - 1 - pixel_y) * resolution
    return map_x, map_y


def load_zones(zones_yaml: Path | str) -> Tuple[Path, List[Dict[str, Any]]]:
    path, document = load_yaml(zones_yaml, 'no-go zones')
    zones = document.get('no_go_zones')
    if not isinstance(zones, list):
        raise MapToolsError('no_go_zones 항목은 list여야 합니다.')

    validated: List[Dict[str, Any]] = []
    used_names = set()
    for index, zone in enumerate(zones):
        owner = f'no_go_zones[{index}]'
        if not isinstance(zone, dict):
            raise MapToolsError(f'{owner}은 mapping이어야 합니다.')
        name = str(zone.get('name', '')).strip()
        if not name:
            raise MapToolsError(f'{owner}.name이 비어 있습니다.')
        if name in used_names:
            raise MapToolsError(f'중복된 no-go zone 이름입니다: {name}')
        used_names.add(name)
        if zone.get('type') != 'polygon':
            raise MapToolsError(f'{name}: type은 polygon이어야 합니다.')
        points = zone.get('points')
        if not isinstance(points, list) or len(points) < 3:
            raise MapToolsError(f'{name}: polygon에는 점이 3개 이상 필요합니다.')
        parsed_points = []
        for point_index, point in enumerate(points):
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise MapToolsError(f'{name}.points[{point_index}]는 [x, y]여야 합니다.')
            try:
                x, y = float(point[0]), float(point[1])
            except (TypeError, ValueError) as exc:
                raise MapToolsError(f'{name}.points[{point_index}] 값은 숫자여야 합니다.') from exc
            if not math.isfinite(x) or not math.isfinite(y):
                raise MapToolsError(f'{name}.points[{point_index}] 값은 유한해야 합니다.')
            parsed_points.append((x, y))
        validated.append({'name': name, 'type': 'polygon', 'points': parsed_points})
    return path, validated


def derived_map_metadata(source: Mapping[str, Any], image_name: str) -> Dict[str, Any]:
    result: Dict[str, Any] = {'image': image_name}
    for key in MAP_METADATA_KEYS:
        result[key] = source[key]
    return result


def atomic_save_yaml(path: Path, document: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', dir=path.parent,
            prefix=f'.{path.name}.', suffix='.tmp', delete=False,
        ) as stream:
            temporary = Path(stream.name)
            yaml.safe_dump(
                dict(document), stream, allow_unicode=True,
                default_flow_style=False, sort_keys=False,
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    except (OSError, yaml.YAMLError) as exc:
        raise MapToolsError(f'YAML 저장 실패 ({path}): {exc}') from exc
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def atomic_save_pgm(path: Path, image: Image.Image) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent, prefix=f'.{path.name}.', suffix='.pgm.tmp'
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        image.convert('L').save(temporary, format='PPM')
        os.replace(temporary, path)
    except OSError as exc:
        raise MapToolsError(f'PGM 저장 실패 ({path}): {exc}') from exc
    finally:
        temporary.unlink(missing_ok=True)
