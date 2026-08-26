"""Select the Raspberry Pi libcamera runtime bundled with this project."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping, Optional


_REQUIRED_PATHS = (
    Path('lib/libcamera.so.0.7'),
    Path('lib/libcamera/ipa/ipa_rpi_pisp.so'),
    Path('libexec/libcamera/raspberrypi_ipa_proxy'),
    Path('share/libcamera/ipa/rpi/pisp/imx708_wide.json'),
)


def _is_complete_runtime(path: Path) -> bool:
    return path.is_dir() and all((path / item).is_file()
                                 for item in _REQUIRED_PATHS)


def find_camera_runtime(
    launch_file: str | Path,
    environ: Optional[Mapping[str, str]] = None,
) -> Optional[Path]:
    """Return a complete runtime from an override or a project parent."""
    environment = os.environ if environ is None else environ
    override = environment.get('INNO_CAMERA_RUNTIME', '').strip()
    if override:
        candidate = Path(override).expanduser().resolve()
        return candidate if _is_complete_runtime(candidate) else None

    launch_path = Path(launch_file).resolve()
    for parent in (launch_path.parent, *launch_path.parents):
        candidate = parent / '.camera_runtime'
        if _is_complete_runtime(candidate):
            return candidate
    return None


def camera_runtime_environment(
    launch_file: str | Path,
    environ: Optional[Mapping[str, str]] = None,
) -> dict[str, str]:
    """Build process-local environment variables for Raspberry Pi libcamera."""
    environment = os.environ if environ is None else environ
    runtime = find_camera_runtime(launch_file, environment)
    if runtime is None:
        return {}

    existing_library_path = environment.get('LD_LIBRARY_PATH', '')
    library_path = str(runtime / 'lib')
    if existing_library_path:
        library_path += ':' + existing_library_path
    return {
        'LD_LIBRARY_PATH': library_path,
        'LIBCAMERA_IPA_MODULE_PATH': str(runtime / 'lib/libcamera/ipa'),
        'LIBCAMERA_IPA_PROXY_PATH': str(runtime / 'libexec/libcamera'),
        'LIBCAMERA_IPA_CONFIG_PATH': str(runtime / 'share/libcamera/ipa'),
    }
