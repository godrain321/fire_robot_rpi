#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

observations="${project_root}/data/extrinsic/observations.json"
camera_info="${project_root}/outputs/pi_camera3_wide_intrinsic/camera_info.yaml"
output_dir="${project_root}/outputs/pi_camera3_wide_extrinsic"

if (( $# > 0 )) && [[ "$1" != -* ]]; then
  observations="$1"
  shift
fi
if (( $# > 0 )) && [[ "$1" != -* ]]; then
  output_dir="$1"
  shift
fi

cd -- "${project_root}"
exec python3 tools/extrinsic/calibrate_lidar_camera_extrinsic.py \
  --observations "${observations}" \
  --camera-info "${camera_info}" \
  --output-dir "${output_dir}" \
  "$@"
