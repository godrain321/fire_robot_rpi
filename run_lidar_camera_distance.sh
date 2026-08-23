#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
EXTRINSIC_PROJECT_ROOT="${project_root}"
# shellcheck source=tools/extrinsic/runtime_helpers.sh
source "${project_root}/tools/extrinsic/runtime_helpers.sh"

usage() {
  cat <<'EOF'
Run the calibrated Raspberry Pi camera + RPLIDAR C1 click-distance viewer.

Usage:
  ./run_lidar_camera_distance.sh [options]

Options:
  --lidar-port PATH        C1 serial device (default: /dev/ttyUSB0).
  --camera ID              camera_ros selector (default: 0).
  --width PX               Camera width (default: 1280).
  --height PX              Camera height (default: 720).
  --startup-timeout SEC    Live image/scan timeout (default: 25).
  --camera-info PATH       Rational 8 CameraInfo YAML.
  --extrinsic PATH         T_camera_lidar result YAML.
  --use-running-camera     Do not start a second camera_ros process.
  --use-running-lidar      Do not start a second C1 driver process.
  -h, --help               Show this help.

Click a projected LiDAR point on an object to show LiDAR range, camera-forward
depth and camera Euclidean distance in cm. c saves a screenshot; q exits.
Only objects intersecting the 2D LiDAR scan plane can have a valid distance.
EOF
}

main() {
  local camera='0'
  local width='1280'
  local height='720'
  local startup_timeout='25'
  local camera_info="${project_root}/outputs/pi_camera3_wide_intrinsic/camera_info.yaml"
  local extrinsic="${project_root}/outputs/pi_camera3_wide_extrinsic/lidar_camera_extrinsic.yaml"
  local screenshot_dir="${project_root}/outputs/pi_camera3_wide_extrinsic/distance_screenshots"
  local lidar_port='/dev/ttyUSB0'
  local lidar_frame='laser'
  local image_topic='/camera/image_raw'
  local scan_topic='/scan'
  local start_camera='true'
  local start_lidar='true'

  while (( $# > 0 )); do
    case "$1" in
      --use-running-camera) start_camera='false'; shift ;;
      --use-running-lidar) start_lidar='false'; shift ;;
      --lidar-port|--camera|--width|--height|--startup-timeout|--camera-info|--extrinsic)
        (( $# >= 2 )) || extrinsic_die "$1 requires a value."
        case "$1" in
          --lidar-port) lidar_port="$2" ;;
          --camera) camera="$2" ;;
          --width) width="$2" ;;
          --height) height="$2" ;;
          --startup-timeout) startup_timeout="$2" ;;
          --camera-info) camera_info="$2" ;;
          --extrinsic) extrinsic="$2" ;;
        esac
        shift 2 ;;
      -h|--help) usage; return 0 ;;
      *) extrinsic_die "Unknown option: $1 (use --help)." ;;
    esac
  done

  extrinsic_require_positive_integer '--width' "${width}"
  extrinsic_require_positive_integer '--height' "${height}"
  extrinsic_require_positive_integer '--startup-timeout' "${startup_timeout}"
  [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || \
    extrinsic_die 'A local/X11 GUI is required for the distance viewer.'
  [[ "${camera_info}" == /* ]] || camera_info="${project_root}/${camera_info}"
  [[ "${extrinsic}" == /* ]] || extrinsic="${project_root}/${extrinsic}"
  [[ -f "${camera_info}" ]] || extrinsic_die "Intrinsic result is missing: ${camera_info}"
  [[ -f "${extrinsic}" ]] || extrinsic_die "Extrinsic result is missing: ${extrinsic}"

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

  python3 "${project_root}/tools/extrinsic/lidar_camera_distance.py" \
    --camera-info "${camera_info}" \
    --extrinsic "${extrinsic}" \
    --image-topic "${image_topic}" \
    --scan-topic "${scan_topic}" \
    --screenshot-dir "${screenshot_dir}"
}

main "$@"
