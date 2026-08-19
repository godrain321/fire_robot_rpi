#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  printf '%s\n' \
    'ERROR: No graphical display detected.' \
    'Run this command on the Raspberry Pi desktop or with GUI forwarding.' >&2
  exit 1
fi

exec python3 "${project_root}/camera_preview.py" "$@"
