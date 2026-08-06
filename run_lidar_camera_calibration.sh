#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
observations="${project_root}/data/extrinsic/observations.json"
camera_info="${project_root}/outputs/pi_camera3_wide_intrinsic/camera_info.yaml"
output_dir="${project_root}/outputs/pi_camera3_wide_extrinsic"

usage() {
  cat <<'EOF'
Collect 24 checkerboard/C1 poses and solve the lidar-to-camera transform.

Usage:
  ./run_lidar_camera_calibration.sh [options]

Options:
  --resume                 Continue a compatible observation set.
  --force                  Replace existing derived extrinsic result files.
  --solve-only             Skip live capture and solve existing observations.
  --lidar-port PATH        C1 serial device (default: /dev/ttyUSB0).
  --use-running-camera     Reuse an already running /camera/image_raw.
  --use-running-lidar      Reuse an already running /scan.
  -h, --help               Show this help.

During capture: YELLOW points are the live full-360 LiDAR scan; select with two clicks or drag (+/- zoom). Hold
the board still for one second, SPACE freezes a matched pair, drag only around
the board line, h saves, r resets, and q ends collection. Capture at
least 20; the guided target is 24 diverse board poses.
EOF
}

main() {
  local resume='false'
  local force='false'
  local solve_only='false'
  local lidar_port='/dev/ttyUSB0'
  local use_running_camera='false'
  local use_running_lidar='false'

  while (( $# > 0 )); do
    case "$1" in
      --resume) resume='true'; shift ;;
      --force) force='true'; shift ;;
      --solve-only) solve_only='true'; shift ;;
      --use-running-camera) use_running_camera='true'; shift ;;
      --use-running-lidar) use_running_lidar='true'; shift ;;
      --lidar-port)
        (( $# >= 2 )) || { printf 'ERROR: --lidar-port requires a value.\n' >&2; return 2; }
        lidar_port="$2"
        shift 2 ;;
      -h|--help) usage; return 0 ;;
      *) printf 'ERROR: unknown option: %s\n' "$1" >&2; return 2 ;;
    esac
  done

  cd -- "${project_root}"
  if [[ "${solve_only}" == 'false' ]]; then
    local -a capture_command=(
      "${project_root}/capture_lidar_camera_extrinsic.sh"
      --lidar-port "${lidar_port}"
      --camera-info "${camera_info}"
      --output-dir "${project_root}/data/extrinsic"
      --target-views 24
    )
    [[ "${resume}" == 'false' ]] || capture_command+=(--resume)
    [[ "${use_running_camera}" == 'false' ]] || capture_command+=(--use-running-camera)
    [[ "${use_running_lidar}" == 'false' ]] || capture_command+=(--use-running-lidar)
    "${capture_command[@]}"
  fi

  local -a solve_command=(
    python3 tools/extrinsic/calibrate_lidar_camera_extrinsic.py
    --observations "${observations}"
    --camera-info "${camera_info}"
    --output-dir "${output_dir}"
    --min-views 20
  )
  [[ "${force}" == 'false' ]] || solve_command+=(--force)
  "${solve_command[@]}"
  printf 'External calibration complete: %s\n' \
    "${output_dir}/lidar_camera_extrinsic.yaml"
}

main "$@"
