"""Safe, atomic persistence for semantic map points."""

from __future__ import annotations

import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Dict, Mapping, Union

import yaml

from .geometry_utils import normalize_yaw


PathLike = Union[str, os.PathLike]
NAME_PATTERN = re.compile(r'^[A-Za-z0-9_-]+$')


class SemanticStoreError(RuntimeError):
    """Base error for semantic file operations."""


class InvalidSemanticFileError(SemanticStoreError):
    """Raised when a semantic YAML file cannot be parsed or validated."""


class DuplicateNameError(SemanticStoreError):
    """Raised when a name already exists and overwrite was not requested."""


class InvalidNameError(SemanticStoreError):
    """Raised when a semantic name contains unsupported characters."""


def default_document() -> Dict[str, Any]:
    """Return a new, empty version-1 semantic map document."""
    return {
        'version': 1,
        'site_id': 'test_map',
        'frame_id': 'map',
        'poses': {},
        'landmarks': {},
    }


def validate_name(name: str) -> str:
    if not isinstance(name, str) or not NAME_PATTERN.fullmatch(name):
        raise InvalidNameError(
            f'잘못된 이름 {name!r}: 영문, 숫자, 밑줄(_), 하이픈(-)만 사용할 수 있습니다.'
        )
    return name


def _finite_float(value: Any, field: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise SemanticStoreError(f'{field} 값은 숫자여야 합니다: {value!r}') from exc
    if not math.isfinite(number):
        raise SemanticStoreError(f'{field} 값은 유한한 숫자여야 합니다: {value!r}')
    return number


class SemanticStore:
    """Load and update one semantic YAML file without partial writes."""

    def __init__(self, path: PathLike):
        self.path = Path(path).expanduser().resolve(strict=False)

    def load(self, create_if_missing: bool = True) -> Dict[str, Any]:
        if not self.path.exists():
            if not create_if_missing:
                raise SemanticStoreError(f'semantic 파일이 없습니다: {self.path}')
            data = default_document()
            self.save(data)
            return data

        if not self.path.is_file():
            raise SemanticStoreError(f'semantic 경로가 일반 파일이 아닙니다: {self.path}')

        try:
            with self.path.open('r', encoding='utf-8') as stream:
                loaded = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as exc:
            raise InvalidSemanticFileError(
                f'semantic YAML을 읽을 수 없습니다 ({self.path}): {exc}'
            ) from exc

        if loaded is None:
            loaded = {}
        if not isinstance(loaded, dict):
            raise InvalidSemanticFileError('semantic YAML 최상위 값은 mapping이어야 합니다.')

        data = loaded
        defaults = default_document()
        for key, value in defaults.items():
            if key not in data:
                data[key] = value

        if not isinstance(data['poses'], dict):
            raise InvalidSemanticFileError('semantic YAML의 poses는 mapping이어야 합니다.')
        if not isinstance(data['landmarks'], dict):
            raise InvalidSemanticFileError('semantic YAML의 landmarks는 mapping이어야 합니다.')
        if not isinstance(data['frame_id'], str) or not data['frame_id'].strip():
            raise InvalidSemanticFileError('semantic YAML의 frame_id는 비어 있지 않은 문자열이어야 합니다.')
        return data

    def save(self, data: Mapping[str, Any]) -> None:
        if not isinstance(data, Mapping):
            raise SemanticStoreError('저장할 semantic 데이터는 mapping이어야 합니다.')
        parent = self.path.parent
        if not parent.exists():
            raise SemanticStoreError(f'semantic 파일의 상위 디렉터리가 없습니다: {parent}')
        if not parent.is_dir():
            raise SemanticStoreError(f'semantic 파일의 상위 경로가 디렉터리가 아닙니다: {parent}')

        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode='w',
                encoding='utf-8',
                dir=str(parent),
                prefix=f'.{self.path.name}.',
                suffix='.tmp',
                delete=False,
            ) as stream:
                temporary_path = Path(stream.name)
                yaml.safe_dump(
                    dict(data),
                    stream,
                    allow_unicode=True,
                    default_flow_style=False,
                    sort_keys=False,
                )
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_path, self.path)
            temporary_path = None
        except (OSError, yaml.YAMLError) as exc:
            raise SemanticStoreError(f'semantic YAML 저장에 실패했습니다 ({self.path}): {exc}') from exc
        finally:
            if temporary_path is not None:
                try:
                    temporary_path.unlink()
                except FileNotFoundError:
                    pass

    def _check_duplicate(
        self,
        data: Mapping[str, Any],
        name: str,
        section: str,
        overwrite: bool,
    ) -> None:
        other_section = 'landmarks' if section == 'poses' else 'poses'
        if name in data[other_section]:
            raise DuplicateNameError(
                f'{name!r}은(는) 이미 {other_section}에 있어 다른 종류로 저장할 수 없습니다.'
            )
        if name in data[section] and not overwrite:
            raise DuplicateNameError(
                f'{name!r}이(가) 이미 존재합니다. 덮어쓰려면 --overwrite를 사용하십시오.'
            )

    def add_pose(
        self,
        name: str,
        x: float,
        y: float,
        yaw: float,
        category: str = '',
        description: str = '',
        overwrite: bool = False,
    ) -> Dict[str, Any]:
        name = validate_name(name)
        data = self.load(create_if_missing=True)
        self._check_duplicate(data, name, 'poses', overwrite)
        data['poses'][name] = {
            'category': str(category),
            'description': str(description),
            'x': _finite_float(x, 'x'),
            'y': _finite_float(y, 'y'),
            'yaw': normalize_yaw(_finite_float(yaw, 'yaw')),
        }
        self.save(data)
        return data

    def add_landmark(
        self,
        name: str,
        x: float,
        y: float,
        category: str = '',
        description: str = '',
        overwrite: bool = False,
    ) -> Dict[str, Any]:
        name = validate_name(name)
        data = self.load(create_if_missing=True)
        self._check_duplicate(data, name, 'landmarks', overwrite)
        data['landmarks'][name] = {
            'category': str(category),
            'description': str(description),
            'x': _finite_float(x, 'x'),
            'y': _finite_float(y, 'y'),
        }
        self.save(data)
        return data
