"""Portable paths for data kept at the fire_robot_rpi repository root."""

from __future__ import annotations

import os
from pathlib import Path


def project_root() -> Path:
    """Return the checkout root, honoring FIRE_ROBOT_RPI_ROOT when provided."""
    override = os.environ.get('FIRE_ROBOT_RPI_ROOT', '').strip()
    if override:
        return Path(override).expanduser().resolve(strict=False)

    starts = (Path(__file__).resolve().parent, Path.cwd().resolve())
    for start in starts:
        for candidate in (start, *start.parents):
            if (candidate / 'maps').is_dir() and (
                candidate / 'inno_jazzy_ws' / 'src'
            ).is_dir():
                return candidate

    return (Path.home() / 'fire_robot_rpi').resolve(strict=False)


def project_path(*parts: str) -> str:
    return str(project_root().joinpath(*parts))
