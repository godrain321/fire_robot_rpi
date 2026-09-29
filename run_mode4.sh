#!/usr/bin/env bash
# Mode 4: existing Mode 3, then Mode 6 thermal preview + YOLO in RViz.
set -euo pipefail

robot_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="${robot_root}/inno_jazzy_ws"
esp32_port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_de2033aed827f0119bb79ad8346f00fe-if00-port0'
lidar_port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_4a5b9018526eef11bff6e0c2c169b110-if00-port0'
mode3_use_serial='true'

for argument in "$@"; do
  case "${argument}" in
    esp32_port:=*) esp32_port="${argument#esp32_port:=}" ;;
    lidar_port:=*) lidar_port="${argument#lidar_port:=}" ;;
    mode3_use_serial:=*) mode3_use_serial="${argument#mode3_use_serial:=}" ;;
    *)
      printf '[오류] 알 수 없는 Mode 4 인자: %s\n' "${argument}" >&2
      exit 2
      ;;
  esac
done
if [[ "${mode3_use_serial}" != 'true' && "${mode3_use_serial}" != 'false' ]]; then
  printf '[오류] mode3_use_serial은 true 또는 false여야 합니다: %s\n' \
    "${mode3_use_serial}" >&2
  exit 2
fi
if ! command -v setsid >/dev/null 2>&1; then
  printf '[오류] child process 정리에 필요한 setsid를 찾지 못했습니다.\n' >&2
  exit 1
fi
for required_script in run_mode6.sh run_camera_inference_check.sh; do
  if [[ ! -x "${robot_root}/${required_script}" ]]; then
    printf '[오류] 실행 파일을 찾을 수 없습니다: %s\n' \
      "${robot_root}/${required_script}" >&2
    exit 1
  fi
done

set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -f "${workspace}/install/setup.bash" ]]; then
  printf '[오류] ROS workspace가 빌드되지 않았습니다. 먼저 colcon build를 실행하세요.\n' >&2
  exit 1
fi
source "${workspace}/install/setup.bash"
set -u

mode3_launch_pid=''
mode3_gate_pid=''
mode6_pid=''
yolo_pid=''

stop_process_group() {
  local process_pid="$1"
  [[ "${process_pid}" =~ ^[0-9]+$ ]] || return 0
  kill -INT -- "-${process_pid}" 2>/dev/null || true
  for _attempt in {1..30}; do
    if ! kill -0 -- "-${process_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if kill -0 -- "-${process_pid}" 2>/dev/null; then
    kill -TERM -- "-${process_pid}" 2>/dev/null || true
  fi
  for _attempt in {1..20}; do
    if ! kill -0 -- "-${process_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if kill -0 -- "-${process_pid}" 2>/dev/null; then
    kill -KILL -- "-${process_pid}" 2>/dev/null || true
  fi
  wait "${process_pid}" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_process_group "${yolo_pid}"
  stop_process_group "${mode6_pid}"
  stop_process_group "${mode3_gate_pid}"
  stop_process_group "${mode3_launch_pid}"
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

printf '[MODE 4] 1단계: 기존 Mode 3 시작\n'
setsid ros2 launch inno_robot_bringup mode4_mode3_stage.launch.py \
  "esp32_port:=${esp32_port}" \
  "use_serial:=${mode3_use_serial}" &
mode3_launch_pid=$!

setsid ros2 run inno_robot_bringup mode4_mode3_gate &
mode3_gate_pid=$!
first_stage_pid=''
set +e
wait -n -p first_stage_pid "${mode3_launch_pid}" "${mode3_gate_pid}"
first_stage_status=$?
set -e
if [[ "${first_stage_pid}" == "${mode3_gate_pid}" ]]; then
  mode3_status=${first_stage_status}
  mode3_gate_pid=''
  stop_process_group "${mode3_launch_pid}"
  mode3_launch_pid=''
else
  printf '[오류] Mode 3 stage process가 완료 신호 전에 종료되었습니다.\n' >&2
  mode3_status=1
  mode3_launch_pid=''
  stop_process_group "${mode3_gate_pid}"
  mode3_gate_pid=''
fi
if (( mode3_status != 0 )); then
  printf '[오류] Mode 3 실패 또는 비정상 종료: 후단 기능을 시작하지 않습니다.\n' >&2
  exit "${mode3_status}"
fi

printf '[MODE 4] Mode 3 완료 확인\n'
printf '[MODE 4] 2단계: Mode 6 thermal preview와 RViz YOLO 화면 동시 시작\n'
setsid "${robot_root}/run_mode6.sh" \
  "lidar_port:=${lidar_port}" \
  use_serial:=false \
  set_initial_pose:=true \
  "rviz_config:=${workspace}/install/inno_robot_bringup/share/inno_robot_bringup/rviz/mode4_thermal_camera.rviz" &
mode6_pid=$!
setsid "${robot_root}/run_camera_inference_check.sh" use_image_view:=false &
yolo_pid=$!

completed_pid=''
set +e
wait -n -p completed_pid "${mode6_pid}" "${yolo_pid}"
combined_status=$?
set -e
if [[ "${completed_pid}" == "${mode6_pid}" ]]; then
  printf '[MODE 4] Mode 6 process 종료; YOLO process도 정리합니다.\n'
else
  printf '[MODE 4] YOLO process 종료; Mode 6 process도 정리합니다.\n'
fi
exit "${combined_status}"
