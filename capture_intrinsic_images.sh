#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Capture raw checkerboard PNGs for the offline Rational Polynomial calibrator.

Usage:
  ./capture_intrinsic_images.sh [options]

Options:
  --resume                 Explicitly continue a verified sequence (default auto-resumes this tool's sequence).
  --use-running-camera     Subscribe to an already running /camera/image_raw.
  --camera ID              camera_ros camera selector (default: 0).
  --width PIXELS           Capture width (default: 1280).
  --height PIXELS          Capture height (default: 720).
  --max-images COUNT       Total target image count (default: 80).
  --board-cols COUNT       Checkerboard inner-corner columns (default: 8).
  --board-rows COUNT       Checkerboard inner-corner rows (default: 9).
  --blur-threshold VALUE   Auto-save sharpness threshold (default: 35.0).
  --startup-timeout SEC    Seconds to wait for a real ROS image (default: 20).
  --no-preview             Run headless; automatic capture remains enabled.
  --manual                 Disable auto-save; use the preview's s key.
  --output-dir PATH        Destination (default: data/intrinsic in this repo).
  -h, --help               Show this help.

The script only captures original, unrectified PNG frames. It never runs a
fisheye or intrinsic calibration calculation. IMX708/rp1-cfe hardware and one
real ROS image are mandatory gates. Press q in the preview to stop.
EOF
}

require_positive_integer() {
  local label="$1"
  local value="$2"
  [[ "$value" =~ ^[1-9][0-9]*$ ]] || \
    die "${label} must be a positive integer (got '${value}')."
}

process_group_has_live_members() {
  local pgid="$1"
  ps -eo pgid=,stat= | awk -v target="${pgid}" '
    $1 == target && $2 !~ /^Z/ { found = 1 }
    END { exit(found ? 0 : 1) }
  '
}

wait_for_process_group_exit() {
  local pgid="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  while process_group_has_live_members "${pgid}"; do
    if (( SECONDS >= deadline )); then
      return 1
    fi
    sleep 0.1
  done
  return 0
}

terminate_launch_process_group() {
  local launch_pid="$1"
  local launch_pgid="$2"
  local int_grace="${CAPTURE_INT_GRACE_SECONDS:-5}"
  local term_grace="${CAPTURE_TERM_GRACE_SECONDS:-3}"
  local kill_grace="${CAPTURE_KILL_GRACE_SECONDS:-2}"

  if ! process_group_has_live_members "${launch_pgid}"; then
    wait "${launch_pid}" 2>/dev/null || true
    return 0
  fi

  printf 'Stopping camera launch process group %s with SIGINT...\n' \
    "${launch_pgid}" >&2
  kill -INT -- "-${launch_pgid}" 2>/dev/null || true
  if wait_for_process_group_exit "${launch_pgid}" "${int_grace}"; then
    wait "${launch_pid}" 2>/dev/null || true
    return 0
  fi

  printf 'Camera launch did not stop in %ss; sending SIGTERM...\n' \
    "${int_grace}" >&2
  kill -TERM -- "-${launch_pgid}" 2>/dev/null || true
  if wait_for_process_group_exit "${launch_pgid}" "${term_grace}"; then
    wait "${launch_pid}" 2>/dev/null || true
    return 0
  fi

  printf 'Camera launch did not stop in %ss; sending SIGKILL...\n' \
    "${term_grace}" >&2
  kill -KILL -- "-${launch_pgid}" 2>/dev/null || true
  if wait_for_process_group_exit "${launch_pgid}" "${kill_grace}"; then
    wait "${launch_pid}" 2>/dev/null || true
    return 0
  fi

  printf 'WARNING: process group %s survived SIGKILL; cleanup is giving up without blocking.\n' \
    "${launch_pgid}" >&2
  return 1
}

i2c_tree_has_imx708() {
  local i2c_root="$1"
  local name_file sensor_name
  local -a name_files=("${i2c_root}"/*/name)

  for name_file in "${name_files[@]}"; do
    [[ -r "${name_file}" ]] || continue
    sensor_name=''
    IFS= read -r sensor_name < "${name_file}" || true
    if [[ "${sensor_name,,}" == *imx708* ]]; then
      return 0
    fi
  done
  return 1
}

media_attributes_have_rp1_cfe() {
  local attributes="${1:-}"
  grep -Eq 'ATTR\{model\}=="rp1-cfe"' <<<"${attributes}"
}

verify_camera_hardware() {
  local i2c_root="${CAMERA_I2C_SYSFS_ROOT:-/sys/bus/i2c/devices}"
  local dev_root="${CAMERA_DEV_ROOT:-/dev}"
  local udevadm_bin="${UDEVADM_BIN:-udevadm}"
  local media attributes
  local matched_media=''
  local -a media_devices=("${dev_root}"/media*)

  if ! i2c_tree_has_imx708 "${i2c_root}"; then
    die "IMX708 is absent under ${i2c_root}/*/name. Power off and reseat the CSI cable."
  fi

  command -v "${udevadm_bin}" >/dev/null 2>&1 || \
    die "'${udevadm_bin}' is required to identify /dev/media* devices."
  for media in "${media_devices[@]}"; do
    [[ -e "${media}" ]] || continue
    attributes=''
    if attributes="$(
      "${udevadm_bin}" info --attribute-walk --name="${media}" 2>/dev/null
    )" && media_attributes_have_rp1_cfe "${attributes}"; then
      matched_media="${media}"
      break
    fi
  done
  if [[ -z "${matched_media}" ]]; then
    die "No /dev/media* device backed by rp1-cfe was found; the Pi 5 CSI receiver is unavailable."
  fi

  printf 'Hardware check passed: IMX708 is bound and %s is rp1-cfe.\n' \
    "${matched_media}"
}

runtime_has_required_libcamera() {
  local lib_dir="$1"
  [[ -e "${lib_dir}/libcamera.so.0.7" && \
    -e "${lib_dir}/libcamera-base.so.0.7" ]]
}

activate_camera_runtime() {
  local runtime_root="${CAMERA_RUNTIME_ROOT:-${project_root}/.camera_runtime}"
  local runtime_lib="${runtime_root}/lib"

  if ! runtime_has_required_libcamera "${runtime_lib}"; then
    printf '%s\n' \
      "ERROR: compatible Pi 5 libcamera libraries are missing under ${runtime_lib}." \
      'Ubuntu 24.04 system libcamera 0.2 cannot drive this rp1-cfe/pisp stack.' \
      'Build the repository-local Raspberry Pi libcamera runtime with:' \
      '  ./build_rpi_camera_runtime.sh' >&2
    exit 1
  fi

  export LD_LIBRARY_PATH="${runtime_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  printf 'Camera runtime enabled: %s is first in LD_LIBRARY_PATH.\n' \
    "${runtime_lib}"
}

frame_probe_received_image() {
  local probe_output="${1:-}"
  local expected_width="$2"
  grep -Eq "^${expected_width}[[:space:]]*$" <<<"${probe_output}"
}

validate_capture_directory() {
  local output_dir="$1"
  local resume="$2"
  local max_images="$3"
  local -a png_files=()
  local -a unexpected=()
  local file basename expected
  local stats_file="${output_dir}/capture_stats.csv"
  local index=0

  mkdir -p -- "${output_dir}"

  while IFS= read -r -d "" file; do
    png_files+=("${file}")
  done < <(
    find "${output_dir}" -maxdepth 1 \( -type f -o -type l \) \
      -name "*.png" -print0 | sort -z
  )

  while IFS= read -r -d "" file; do
    unexpected+=("${file}")
  done < <(
    find "${output_dir}" -mindepth 1 -maxdepth 1 \
      ! -name "capture_stats.csv" ! -name "calib_*.png" -print0
  )
  if (( ${#unexpected[@]} > 0 )); then
    if [[ "${resume}" == "true" ]]; then
      die "--resume found an unexpected entry: ${unexpected[0]}"
    fi
    die "${output_dir} contains an unrelated entry: ${unexpected[0]}. Move it aside before capture."
  fi

  for file in "${png_files[@]}"; do
    basename="$(basename -- "${file}")"
    printf -v expected "calib_%03d.png" "${index}"
    [[ "${basename}" == "${expected}" ]] || \
      die "capture requires an unbroken sequence; expected ${expected}, found ${basename}."
    ((index += 1))
  done

  (( index < max_images )) || \
    die "${output_dir} already contains ${index}/${max_images} target images."

  if (( index == 0 )); then
    if [[ -e "${stats_file}" ]]; then
      printf "Safe restart: no PNG exists; reusing %s.\n" "${stats_file}"
    fi
    return 0
  fi

  if [[ "${resume}" != "true" && ! -f "${stats_file}" ]]; then
    die "${output_dir} is not empty: it has a calib_NNN.png sequence but no capture_stats.csv; pass --resume only if this set is intentional."
  fi
  printf "Auto-resume: verified %d contiguous images; next file is calib_%03d.png.\n" \
    "${index}" "${index}"
}

source_ros_environment() {
  local ros_setup="${ROS_SETUP_FILE:-/opt/ros/jazzy/setup.bash}"
  local workspace_setup="${CAMERA_WS_SETUP_FILE:-${project_root}/camera_ws/install/setup.bash}"

  [[ -r "${ros_setup}" ]] || die "ROS setup file is missing: ${ros_setup}"
  [[ -r "${workspace_setup}" ]] || \
    die "Camera workspace is not built: ${workspace_setup}"

  # ROS/colcon setup scripts legitimately inspect optional unset variables.
  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  # shellcheck disable=SC1090
  source "${workspace_setup}"
  set -u

  command -v ros2 >/dev/null 2>&1 || die 'ros2 is unavailable after sourcing ROS.'
  ros2 pkg prefix camera_ros >/dev/null 2>&1 || \
    die 'camera_ros is not installed in the sourced environment.'
  ros2 pkg prefix fire_robot_camera_calibration >/dev/null 2>&1 || \
    die 'fire_robot_camera_calibration is not built in camera_ws.'
}

main() {
  local output_dir="${project_root}/data/intrinsic"
  local camera='0'
  local width='1280'
  local height='720'
  local max_images='80'
  local board_cols='8'
  local board_rows='9'
  local blur_threshold='35.0'
  local startup_timeout='20'
  local preview='true'
  local auto_save='true'
  local resume='false'
  local start_camera='true'
  local image_topic='/camera/image_raw'
  local launch_pid=''
  local launch_pgid=''
  local frame_probe_output=''
  local frame_probe_status=0
  local launch_status=0

  while (( $# > 0 )); do
    case "$1" in
      --resume)
        resume='true'
        shift
        ;;
      --use-running-camera)
        start_camera='false'
        shift
        ;;
      --camera|--width|--height|--max-images|--board-cols|--board-rows|\
      --blur-threshold|--startup-timeout|--output-dir)
        (( $# >= 2 )) || die "$1 requires a value."
        case "$1" in
          --camera) camera="$2" ;;
          --width) width="$2" ;;
          --height) height="$2" ;;
          --max-images) max_images="$2" ;;
          --board-cols) board_cols="$2" ;;
          --board-rows) board_rows="$2" ;;
          --blur-threshold) blur_threshold="$2" ;;
          --startup-timeout) startup_timeout="$2" ;;
          --output-dir) output_dir="$2" ;;
        esac
        shift 2
        ;;
      --no-preview)
        preview='false'
        shift
        ;;
      --manual)
        auto_save='false'
        shift
        ;;
      -h|--help)
        usage
        return 0
        ;;
      *)
        die "Unknown option: $1 (use --help)."
        ;;
    esac
  done

  require_positive_integer '--width' "${width}"
  require_positive_integer '--height' "${height}"
  require_positive_integer '--max-images' "${max_images}"
  require_positive_integer '--board-cols' "${board_cols}"
  require_positive_integer '--board-rows' "${board_rows}"
  require_positive_integer '--startup-timeout' "${startup_timeout}"
  if [[ "${preview}" == 'false' && "${auto_save}" == 'false' ]]; then
    die '--manual cannot be combined with --no-preview.'
  fi
  if [[ "${preview}" == 'true' && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    die 'No graphical display was detected. Use --no-preview or enable X/Wayland forwarding.'
  fi

  if [[ "${output_dir}" != /* ]]; then
    output_dir="${project_root}/${output_dir}"
  fi
  validate_capture_directory "${output_dir}" "${resume}" "${max_images}"
  verify_camera_hardware
  source_ros_environment
  activate_camera_runtime
  command -v setsid >/dev/null 2>&1 || \
    die "'setsid' is required for bounded camera process cleanup."
  command -v ps >/dev/null 2>&1 || die "'ps' is required for camera cleanup."
  command -v awk >/dev/null 2>&1 || die "'awk' is required for camera cleanup."

  printf 'Saving original %sx%s PNG frames to %s\n' \
    "${width}" "${height}" "${output_dir}"
  printf '%s\n' 'Capture guidance: cover all corners/edges; vary distance, tilt, and roll.'

  cleanup_launch() {
    if [[ -n "${launch_pid}" && -n "${launch_pgid}" ]]; then
      terminate_launch_process_group \
        "${launch_pid}" "${launch_pgid}" || true
      launch_pid=''
      launch_pgid=''
    fi
  }
  trap cleanup_launch EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  setsid ros2 launch fire_robot_camera_calibration intrinsic_capture.launch.py \
    start_camera:="${start_camera}" \
    camera:="${camera}" \
    width:="${width}" \
    height:="${height}" \
    image_topic:="${image_topic}" \
    transport:=raw \
    board_cols:="${board_cols}" \
    board_rows:="${board_rows}" \
    max_images:="${max_images}" \
    preview:="${preview}" \
    auto_save:="${auto_save}" \
    blur_threshold:="${blur_threshold}" \
    output_dir:="${output_dir}" &
  launch_pid=$!

  launch_pgid="${launch_pid}"
  set +e
  frame_probe_output="$(
    ros2 topic echo "${image_topic}" sensor_msgs/msg/Image \
      --field width --once --timeout "${startup_timeout}" --no-daemon 2>&1
  )"
  frame_probe_status=$?
  set -e
  if (( frame_probe_status != 0 )) || \
    ! frame_probe_received_image "${frame_probe_output}" "${width}"; then
    printf '%s\n' "${frame_probe_output}" >&2
    die "No ${width}px-wide image arrived on ${image_topic} within ${startup_timeout}s."
  fi
  printf 'Streaming check passed: received an image on %s (%s).\n' \
    "${image_topic}" "${frame_probe_output//$'\n'/ }"

  set +e
  wait "${launch_pid}"
  launch_status=$?
  set -e
  launch_pid=''
  trap - EXIT INT TERM
  launch_pgid=''
  return "${launch_status}"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
