#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="${ROS_SETUP_FILE:-/opt/ros/jazzy/setup.bash}"
camera_setup="${CAMERA_WS_SETUP_FILE:-${project_root}/camera_ws/install/local_setup.bash}"
robot_setup="${ROBOT_WS_SETUP_FILE:-${project_root}/inno_jazzy_ws/install/local_setup.bash}"
runtime_root="${CAMERA_RUNTIME_ROOT:-${project_root}/.camera_runtime}"
model_path="${project_root}/models/yolov8n_best_opencv_640.onnx"
model_path_explicit='false'
launch_args=()

for argument in "$@"; do
  case "${argument}" in
    model_path:=*)
      model_path="${argument#model_path:=}"
      model_path_explicit='true'
      ;;
    *) launch_args+=("${argument}") ;;
  esac
done
if [[ "${model_path_explicit}" == 'false' && ! -f "${model_path}" ]]; then
  legacy_model="${HOME}/fire_robot_rpi/models/yolov8n_best_opencv_640.onnx"
  if [[ -f "${legacy_model}" ]]; then
    model_path="${legacy_model}"
  fi
fi

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

[[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || \
  die '그래픽 화면이 없습니다. 라즈베리파이 데스크톱에서 실행하세요.'
[[ -r "${ros_setup}" ]] || die "ROS 2 Jazzy를 찾을 수 없습니다: ${ros_setup}"
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
if [[ -r "${camera_setup}" ]]; then
  # An optional overlay may provide camera_ros, but the standard Jazzy
  # installation is sufficient when this workspace has not been built.
  # shellcheck disable=SC1090
  source "${camera_setup}"
fi
# shellcheck disable=SC1090
source "${robot_setup}"
set -u

ros2 pkg prefix camera_ros >/dev/null 2>&1 || \
  die 'camera_ros 패키지를 찾을 수 없습니다.'
ros2 pkg prefix inno_camera_tools >/dev/null 2>&1 || \
  die 'inno_camera_tools 패키지를 찾을 수 없습니다.'

export LD_LIBRARY_PATH="${runtime_root}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export INNO_CAMERA_RUNTIME="${runtime_root}"

printf '[카메라 테스트] YOLO 모델: %s\n' "${model_path}"

exec ros2 launch inno_camera_tools camera_inference_check.launch.py \
  "model_path:=${model_path}" \
  "${launch_args[@]}"
