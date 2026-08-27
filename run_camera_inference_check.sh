#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="${ROS_SETUP_FILE:-/opt/ros/jazzy/setup.bash}"
camera_setup="${CAMERA_WS_SETUP_FILE:-${project_root}/camera_ws/install/local_setup.bash}"
robot_setup="${ROBOT_WS_SETUP_FILE:-${project_root}/inno_jazzy_ws/install/local_setup.bash}"
runtime_root="${CAMERA_RUNTIME_ROOT:-${project_root}/.camera_runtime}"
model_path="${project_root}/models/yolov8n_best_opencv_640.onnx"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

[[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || \
  die '그래픽 화면이 없습니다. 라즈베리파이 데스크톱에서 실행하세요.'
[[ -r "${ros_setup}" ]] || die "ROS 2 Jazzy를 찾을 수 없습니다: ${ros_setup}"
[[ -r "${camera_setup}" ]] || die "카메라 workspace가 빌드되지 않았습니다: ${camera_setup}"
[[ -r "${robot_setup}" ]] || die "로봇 workspace가 빌드되지 않았습니다: ${robot_setup}"
[[ -f "${model_path}" ]] || die "YOLO 모델을 찾을 수 없습니다: ${model_path}"
[[ -e "${runtime_root}/lib/libcamera.so.0.7" ]] || \
  die "카메라 런타임이 없습니다. ${project_root}/build_rpi_camera_runtime.sh 를 먼저 실행하세요."

camera_found='false'
for name_file in /sys/bus/i2c/devices/*/name; do
  [[ -r "${name_file}" ]] || continue
  if grep -qi 'imx708' "${name_file}"; then
    camera_found='true'
    break
  fi
done
[[ "${camera_found}" == 'true' ]] || \
  die 'IMX708 Camera Module 3 연결을 찾지 못했습니다.'

set +u
# shellcheck disable=SC1090
source "${ros_setup}"
# shellcheck disable=SC1090
source "${camera_setup}"
# shellcheck disable=SC1090
source "${robot_setup}"
set -u

export LD_LIBRARY_PATH="${runtime_root}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

exec ros2 launch inno_camera_tools camera_inference_check.launch.py \
  "model_path:=${model_path}" \
  "$@"
