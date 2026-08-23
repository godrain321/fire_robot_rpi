#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
EXTRINSIC_PROJECT_ROOT="${project_root}"
# shellcheck source=tools/extrinsic/runtime_helpers.sh
source "${project_root}/tools/extrinsic/runtime_helpers.sh"

usage() {
  cat <<'EOF'
Collect paired checkerboard-plane and RPLIDAR C1 line observations.

Usage:
  ./capture_lidar_camera_extrinsic.sh [options]

Options:
  --resume                 Append to the existing, compatible observation set.
  --lidar-port PATH        C1 serial device (default: /dev/ttyUSB0).
  --camera ID              camera_ros selector (default: 0).
  --width PX               Camera width (default: 1280).
  --height PX              Camera height (default: 720).
  --target-views N         Guided observation target (default: 24).
  --startup-timeout SEC    Live image/scan timeout (default: 25).
  --camera-info PATH       Rational 8 CameraInfo YAML.
  --output-dir PATH        Observation directory (default: data/extrinsic).
  --use-running-camera     Do not start a second camera_ros process.
  --use-running-lidar      Do not start a second C1 driver process.
  -h, --help               Show this help.

GUI: YELLOW points are the live full-360 LiDAR scan; select with two clicks or drag (+/- zoom). Hold the board
still for one second, SPACE freezes a matched pair, drag only over the board line,
h saves, and r resets. Keep the sensor mount fixed and move only
the board through 24+ diverse positions and two-axis tilts.
EOF
}

main() {
  local camera='0'
  local width='1280'
  local height='720'
  local target_views='24'
  local startup_timeout='25'
  local camera_info="${project_root}/outputs/pi_camera3_wide_intrinsic/camera_info.yaml"
  local output_dir="${project_root}/data/extrinsic"
  local lidar_port='/dev/ttyUSB0'
  local lidar_frame='laser'
  local image_topic='/camera/image_raw'
  local scan_topic='/scan'
  local start_camera='true'
  local start_lidar='true'
  local resume='false'

  while (( $# > 0 )); do
    case "$1" in
      --resume) resume='true'; shift ;;
      --use-running-camera) start_camera='false'; shift ;;
      --use-running-lidar) start_lidar='false'; shift ;;
      --lidar-port|--camera|--width|--height|--target-views|--startup-timeout|--camera-info|--output-dir)
        (( $# >= 2 )) || extrinsic_die "$1 requires a value."
        case "$1" in
          --lidar-port) lidar_port="$2" ;;
          --camera) camera="$2" ;;
          --width) width="$2" ;;
          --height) height="$2" ;;
          --target-views) target_views="$2" ;;
          --startup-timeout) startup_timeout="$2" ;;
          --camera-info) camera_info="$2" ;;
          --output-dir) output_dir="$2" ;;
        esac
        shift 2 ;;
      -h|--help) usage; return 0 ;;
      *) extrinsic_die "Unknown option: $1 (use --help)." ;;
    esac
  done

  extrinsic_require_positive_integer '--width' "${width}"
  extrinsic_require_positive_integer '--height' "${height}"
  extrinsic_require_positive_integer '--target-views' "${target_views}"
  extrinsic_require_positive_integer '--startup-timeout' "${startup_timeout}"
  [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || \
    extrinsic_die 'A local/X11 GUI is required for LiDAR board-line selection.'
  [[ "${camera_info}" == /* ]] || camera_info="${project_root}/${camera_info}"
  [[ "${output_dir}" == /* ]] || output_dir="${project_root}/${output_dir}"
  [[ -f "${camera_info}" ]] || \
    extrinsic_die "Intrinsic result is missing: ${camera_info} (run ./calibrate_camera.sh first)."

  extrinsic_verify_camera_hardware
  [[ "${start_lidar}" == 'false' ]] || extrinsic_verify_lidar_port "${lidar_port}"
  extrinsic_source_ros
  extrinsic_activate_camera_runtime
  command -v setsid >/dev/null 2>&1 || extrinsic_die 'setsid is required.'

  trap extrinsic_stop_processes EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  extrinsic_start_sensor_pair "${start_camera}" "${start_lidar}" \
    "${camera}" "${width}" "${height}" "${camera_info}" \
    "${lidar_port}" "${lidar_frame}"
  extrinsic_probe_topics "${image_topic}" "${scan_topic}" \
    "${width}" "${lidar_frame}" "${startup_timeout}"

  local -a command=(
    python3 "${project_root}/tools/extrinsic/capture_extrinsic_observations.py"
    --camera-info "${camera_info}"
    --output-dir "${output_dir}"
    --image-topic "${image_topic}"
    --scan-topic "${scan_topic}"
    --lidar-frame "${lidar_frame}"
    --board-cols 8 --board-rows 9 --square-size-m 0.070
    --target-views "${target_views}"
    --startup-timeout "${startup_timeout}"
  )
  [[ "${resume}" == 'false' ]] || command+=(--resume)
  "${command[@]}"
}

main "$@"
