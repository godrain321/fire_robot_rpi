#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
distance='2.0'
camera='0'
width='1280'
height='720'
display_scale='0.85'
start_camera='true'
image_topic='/camera/image_raw'
camera_info_topic='/camera/camera_info'
output_dir="${project_root}/data/fov_check"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  printf '%s\n' \
    'Open Camera Module 3 with distance/FOV overlays.' \
    '' \
    'Usage: ./run_camera_fov_check.sh [options]' \
    '' \
    '  --distance METRES       Measured camera-to-person distance (default: 2.0)' \
    '  --camera ID             camera_ros selector (default: 0)' \
    '  --width PIXELS          Image width (default: 1280)' \
    '  --height PIXELS         Image height (default: 720)' \
    '  --display-scale NUMBER  Preview scaling (default: 0.85)' \
    '  --output-dir PATH       Snapshot directory (default: data/fov_check)' \
    '  --use-running-camera    Use an existing ROS image topic' \
    '  --image-topic TOPIC     Existing image topic' \
    '  --camera-info-topic T   Existing CameraInfo topic' \
    '  -h, --help              Show this help' \
    '' \
    'Preview keys: s/SPACE=save, +/-=adjust distance, q=quit.'
}

positive_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] && \
    awk -v value="$1" 'BEGIN { exit !(value > 0) }'
}

positive_integer() {
  [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

verify_camera_hardware() {
  local name_file sensor_name media attributes
  local found_sensor='false'
  local found_receiver='false'
  for name_file in /sys/bus/i2c/devices/*/name; do
    [[ -r "${name_file}" ]] || continue
    sensor_name=''
    IFS= read -r sensor_name < "${name_file}" || true
    if [[ "${sensor_name,,}" == *imx708* ]]; then
      found_sensor='true'
      break
    fi
  done
  [[ "${found_sensor}" == 'true' ]] || \
    die 'IMX708 Camera Module 3 is not connected. Power off the Pi and reseat the CSI cable.'

  command -v udevadm >/dev/null 2>&1 || die 'udevadm is required.'
  for media in /dev/media*; do
    [[ -e "${media}" ]] || continue
    attributes="$(
      udevadm info --attribute-walk --name="${media}" 2>/dev/null || true
    )"
    if grep -Eq 'ATTR\{model\}=="rp1-cfe"' <<<"${attributes}"; then
      found_receiver='true'
      printf 'Camera connection OK: IMX708 + %s (rp1-cfe)\n' "${media}"
      break
    fi
  done
  [[ "${found_receiver}" == 'true' ]] || \
    die 'IMX708 exists, but no Pi 5 rp1-cfe media device was found.'
}

source_ros() {
  local ros_setup="${ROS_SETUP_FILE:-/opt/ros/jazzy/setup.bash}"
  local camera_setup="${CAMERA_WS_SETUP_FILE:-${project_root}/camera_ws/install/local_setup.bash}"
  local robot_setup="${ROBOT_WS_SETUP_FILE:-${project_root}/inno_jazzy_ws/install/local_setup.bash}"
  [[ -r "${ros_setup}" ]] || die "ROS 2 Jazzy is missing: ${ros_setup}"
  [[ -r "${camera_setup}" ]] || die \
    "camera_ws is not built. Build it before running: ${camera_setup}"
  [[ -r "${robot_setup}" ]] || die \
    "inno_jazzy_ws is not built. Build inno_camera_tools: ${robot_setup}"
  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  # shellcheck disable=SC1090
  source "${camera_setup}"
  # shellcheck disable=SC1090
  source "${robot_setup}"
  set -u
  ros2 pkg prefix camera_ros >/dev/null 2>&1 || die 'camera_ros is unavailable.'
  ros2 pkg prefix inno_camera_tools >/dev/null 2>&1 || \
    die 'inno_camera_tools is unavailable in the sourced workspaces.'
}

activate_camera_runtime() {
  local runtime_root="${CAMERA_RUNTIME_ROOT:-${project_root}/.camera_runtime}"
  [[ -e "${runtime_root}/lib/libcamera.so.0.7" ]] || die \
    "Pi camera runtime is missing. Run: ${project_root}/build_rpi_camera_runtime.sh"
  export LD_LIBRARY_PATH="${runtime_root}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
}

while (( $# > 0 )); do
  case "$1" in
    --distance|--camera|--width|--height|--display-scale|--output-dir|\
    --image-topic|--camera-info-topic)
      (( $# >= 2 )) || die "$1 requires a value."
      case "$1" in
        --distance) distance="$2" ;;
        --camera) camera="$2" ;;
        --width) width="$2" ;;
        --height) height="$2" ;;
        --display-scale) display_scale="$2" ;;
        --output-dir) output_dir="$2" ;;
        --image-topic) image_topic="$2" ;;
        --camera-info-topic) camera_info_topic="$2" ;;
      esac
      shift 2
      ;;
    --use-running-camera)
      start_camera='false'
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown option: $1 (use --help)"
      ;;
  esac
done

positive_number "${distance}" || die '--distance must be positive.'
positive_number "${display_scale}" || die '--display-scale must be positive.'
positive_integer "${width}" || die '--width must be a positive integer.'
positive_integer "${height}" || die '--height must be a positive integer.'
if [[ "${start_camera}" == 'true' ]]; then
  verify_camera_hardware
fi
[[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || \
  die 'No graphical display detected. Run on the Pi desktop or enable X forwarding.'

source_ros
if [[ "${start_camera}" == 'true' ]]; then
  activate_camera_runtime
fi
mkdir -p -- "${output_dir}"

exec ros2 launch inno_camera_tools camera_fov_check.launch.py \
  "start_camera:=${start_camera}" \
  "camera:=${camera}" \
  "width:=${width}" \
  "height:=${height}" \
  "image_topic:=${image_topic}" \
  "camera_info_topic:=${camera_info_topic}" \
  "target_distance_m:=${distance}" \
  "display_scale:=${display_scale}" \
  "output_dir:=${output_dir}"
