#!/usr/bin/env bash

# Shared, source-only runtime helpers for the C1 + IMX708 workflows.

declare -ag EXTRINSIC_PROCESS_PIDS=()
EXTRINSIC_LAST_PID=''

extrinsic_die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

extrinsic_require_positive_integer() {
  local label="$1"
  local value="$2"
  [[ "${value}" =~ ^[1-9][0-9]*$ ]] || \
    extrinsic_die "${label} must be a positive integer (got '${value}')."
}

extrinsic_source_ros() {
  local ros_setup="${ROS_SETUP_FILE:-/opt/ros/jazzy/setup.bash}"
  local lidar_setup="${LIDAR_WS_SETUP_FILE:-${EXTRINSIC_PROJECT_ROOT}/inno_jazzy_ws/install/setup.bash}"
  local camera_setup="${CAMERA_WS_SETUP_FILE:-${EXTRINSIC_PROJECT_ROOT}/camera_ws/install/local_setup.bash}"

  [[ -r "${ros_setup}" ]] || extrinsic_die "ROS setup is missing: ${ros_setup}"
  [[ -r "${lidar_setup}" ]] || \
    extrinsic_die "RPLIDAR workspace is not built: ${lidar_setup}"

  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  # shellcheck disable=SC1090
  source "${lidar_setup}"
  if [[ -r "${camera_setup}" ]]; then
    # shellcheck disable=SC1090
    source "${camera_setup}"
  fi
  set -u

  command -v ros2 >/dev/null 2>&1 || \
    extrinsic_die 'ros2 is unavailable after sourcing ROS 2 Jazzy.'
  ros2 pkg prefix camera_ros >/dev/null 2>&1 || \
    extrinsic_die 'camera_ros is not installed.'
  ros2 pkg prefix inno_bringup >/dev/null 2>&1 || \
    extrinsic_die 'inno_bringup is not available in the sourced workspace.'
  ros2 pkg prefix sllidar_ros2 >/dev/null 2>&1 || \
    extrinsic_die 'sllidar_ros2 is not available in the sourced workspace.'
}

extrinsic_activate_camera_runtime() {
  local runtime_root="${CAMERA_RUNTIME_ROOT:-${EXTRINSIC_PROJECT_ROOT}/.camera_runtime}"
  local runtime_lib="${runtime_root}/lib"

  [[ -e "${runtime_lib}/libcamera.so.0.7" ]] || \
    extrinsic_die "Compatible Pi libcamera is missing under ${runtime_lib}; run ./build_rpi_camera_runtime.sh first."
  [[ -e "${runtime_lib}/libcamera-base.so.0.7" ]] || \
    extrinsic_die "Compatible Pi libcamera-base is missing under ${runtime_lib}."
  export LD_LIBRARY_PATH="${runtime_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
}

extrinsic_verify_camera_hardware() {
  local i2c_root="${CAMERA_I2C_SYSFS_ROOT:-/sys/bus/i2c/devices}"
  local dev_root="${CAMERA_DEV_ROOT:-/dev}"
  local name_file sensor_name media attributes
  local found_sensor='false'
  local found_receiver='false'
  local -a names=("${i2c_root}"/*/name)
  local -a media_devices=("${dev_root}"/media*)

  for name_file in "${names[@]}"; do
    [[ -r "${name_file}" ]] || continue
    sensor_name=''
    IFS= read -r sensor_name < "${name_file}" || true
    if [[ "${sensor_name,,}" == *imx708* ]]; then
      found_sensor='true'
      break
    fi
  done
  [[ "${found_sensor}" == 'true' ]] || \
    extrinsic_die 'IMX708 is not bound. Power off and check the CSI cable.'

  command -v udevadm >/dev/null 2>&1 || \
    extrinsic_die 'udevadm is required for the rp1-cfe check.'
  for media in "${media_devices[@]}"; do
    [[ -e "${media}" ]] || continue
    attributes="$(udevadm info --attribute-walk --name="${media}" 2>/dev/null || true)"
    if grep -Eq 'ATTR\{model\}=="rp1-cfe"' <<<"${attributes}"; then
      found_receiver='true'
      break
    fi
  done
  [[ "${found_receiver}" == 'true' ]] || \
    extrinsic_die 'No rp1-cfe /dev/media device was found.'
  printf 'Camera hardware check passed: IMX708 + rp1-cfe.\n'
}

extrinsic_verify_lidar_port() {
  local port="$1"
  [[ -e "${port}" ]] || \
    extrinsic_die "RPLIDAR serial device is absent: ${port} (connect the C1 first)."
  [[ -c "${port}" ]] || extrinsic_die "RPLIDAR path is not a character device: ${port}"
  [[ -r "${port}" && -w "${port}" ]] || \
    extrinsic_die "RPLIDAR device is not readable/writable: ${port} (check dialout membership)."
  printf 'LiDAR serial check passed: %s\n' "${port}"
}

extrinsic_group_has_members() {
  local pgid="$1"
  ps -eo pgid=,stat= | awk -v target="${pgid}" \
    '$1 == target && $2 !~ /^Z/ { found=1 } END { exit(found ? 0 : 1) }'
}

extrinsic_wait_group_exit() {
  local pgid="$1"
  local timeout="$2"
  local deadline=$((SECONDS + timeout))
  while extrinsic_group_has_members "${pgid}"; do
    (( SECONDS < deadline )) || return 1
    sleep 0.1
  done
}

extrinsic_start_group() {
  setsid "$@" &
  EXTRINSIC_LAST_PID=$!
  EXTRINSIC_PROCESS_PIDS+=("${EXTRINSIC_LAST_PID}")
}

extrinsic_stop_processes() {
  local index pid signal grace signal_index
  local -a signals=(INT TERM KILL)
  local -a graces=(5 3 2)

  for ((index=${#EXTRINSIC_PROCESS_PIDS[@]}-1; index>=0; index--)); do
    pid="${EXTRINSIC_PROCESS_PIDS[index]}"
    extrinsic_group_has_members "${pid}" || { wait "${pid}" 2>/dev/null || true; continue; }
    for signal_index in 0 1 2; do
      signal="${signals[signal_index]}"
      grace="${graces[signal_index]}"
      kill -"${signal}" -- "-${pid}" 2>/dev/null || true
      if extrinsic_wait_group_exit "${pid}" "${grace}"; then
        break
      fi
    done
    wait "${pid}" 2>/dev/null || true
  done
  EXTRINSIC_PROCESS_PIDS=()
}

extrinsic_prepare_camera_info() {
  local source="$1"
  local width="$2"
  local height="$3"
  local camera_name="imx708_wide__base_axi_pcie_120000_rp1_i2c_88000_imx708_1a_${width}x${height}"
  local target_dir="${EXTRINSIC_PROJECT_ROOT}/.camera_runtime/camera_info"
  local target="${target_dir}/${camera_name}.yaml"
  local temporary="${target}.tmp"

  [[ -r "${source}" ]] || extrinsic_die "CameraInfo YAML is not readable: ${source}"
  mkdir -p -- "${target_dir}"
  sed -E "s|^camera_name:.*$|camera_name: ${camera_name}|" "${source}" > "${temporary}"
  grep -Fqx "camera_name: ${camera_name}" "${temporary}" || {
    rm -f -- "${temporary}"
    extrinsic_die "CameraInfo YAML has no replaceable camera_name: ${source}"
  }
  mv -f -- "${temporary}" "${target}"
  printf "%s\n" "${target}"
}

extrinsic_start_sensor_pair() {
  local start_camera="$1"
  local start_lidar="$2"
  local camera="$3"
  local width="$4"
  local height="$5"
  local camera_info="$6"
  local lidar_port="$7"
  local lidar_frame="$8"
  local camera_info_for_node

  if [[ "${start_camera}" == "true" ]]; then
    camera_info_for_node="$(extrinsic_prepare_camera_info "${camera_info}" "${width}" "${height}")"
    extrinsic_start_group ros2 run camera_ros camera_node --ros-args \
      -p camera:="${camera}" \
      -p format:=XRGB8888 \
      -p width:="${width}" \
      -p height:="${height}" \
      -p frame_id:=camera_optical_frame \
      -p camera_info_url:="file://${camera_info_for_node}"
    printf "Started camera_ros (process group %s).\n" "${EXTRINSIC_LAST_PID}"
  fi
  if [[ "${start_lidar}" == "true" ]]; then
    extrinsic_start_group ros2 launch inno_bringup lidar_only.launch.py \
      serial_port:="${lidar_port}" frame_id:="${lidar_frame}"
    printf "Started RPLIDAR C1 driver (process group %s, 460800 baud).\n" \
      "${EXTRINSIC_LAST_PID}"
  fi
}

extrinsic_probe_topics() {
  local image_topic="$1"
  local scan_topic="$2"
  local expected_width="$3"
  local lidar_frame="$4"
  local timeout="$5"
  local image_probe scan_probe

  set +e
  image_probe="$(ros2 topic echo "${image_topic}" sensor_msgs/msg/Image \
    --field width --once --timeout "${timeout}" --no-daemon 2>&1)"
  local image_status=$?
  set -e
  if (( image_status != 0 )) || \
    ! grep -Eq "^${expected_width}[[:space:]]*$" <<<"${image_probe}"; then
    extrinsic_die "No real ${expected_width}px camera frame arrived on ${image_topic} within ${timeout}s."
  fi

  set +e
  scan_probe="$(ros2 topic echo "${scan_topic}" sensor_msgs/msg/LaserScan \
    --once --timeout "${timeout}" --no-daemon 2>&1)"
  local scan_status=$?
  set -e
  (( scan_status == 0 )) || \
    extrinsic_die "No LaserScan arrived on ${scan_topic} within ${timeout}s."
  grep -Eq "frame_id:[[:space:]]*['\"]?${lidar_frame}['\"]?" <<<"${scan_probe}" || \
    extrinsic_die "LaserScan frame_id is not '${lidar_frame}'."
  grep -Eq '^ranges:' <<<"${scan_probe}" || \
    extrinsic_die "LaserScan on ${scan_topic} did not contain ranges."
  printf 'Live topic gate passed: %s + %s (frame %s).\n' \
    "${image_topic}" "${scan_topic}" "${lidar_frame}"
}
