#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "${project_root}"
exec python3 tools/calibration/view_rational_undistortion.py "$@"
