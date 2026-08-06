#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

images_glob='data/intrinsic/*.png'
output_dir='outputs/pi_camera3_wide_intrinsic'

if (( $# > 0 )); then
  images_glob="$1"
  shift
fi
if (( $# > 0 )); then
  output_dir="$1"
  shift
fi

cd -- "${project_root}"
exec python3 tools/calibration/calibrate_checkerboard_rational.py \
  --images "${images_glob}" \
  --config config/checkerboard_rational.yaml \
  --output-dir "${output_dir}" \
  "$@"
