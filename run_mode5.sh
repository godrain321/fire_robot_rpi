#!/usr/bin/env bash
set -euo pipefail

drive_speed='0.06'
use_camera_mode4='true'
launch_args=()
for argument in "$@"; do
  case "${argument}" in
    drive_speed:=*) drive_speed="${argument#drive_speed:=}" ;;
    use_camera_mode4:=*)
      use_camera_mode4="${argument#use_camera_mode4:=}"
      ;;
    *) launch_args+=("${argument}") ;;
  esac
done
if ! [[ "${drive_speed}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || \
    ! awk -v value="${drive_speed}" 'BEGIN { exit !(value > 0) }'; then
  printf '[오류] drive_speed는 0보다 큰 숫자여야 합니다: %s\n' "${drive_speed}" >&2
  exit 2
fi
if [[ "${use_camera_mode4}" != 'true' && "${use_camera_mode4}" != 'false' ]]; then
  printf '[오류] use_camera_mode4는 true 또는 false여야 합니다: %s\n' \
    "${use_camera_mode4}" >&2
  exit 2
fi

cd /home/seeno04/fire_robot_rpi/inno_jazzy_ws
set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
  printf '[오류] ROS workspace가 빌드되지 않았습니다. 먼저 colcon build를 실행하세요.\n'
  exit 1
fi
source install/setup.bash
set -u

set +e
stdbuf -oL -eL ros2 launch inno_robot_bringup evacuation_demo.launch.py \
  esp32_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_de2033aed827f0119bb79ad8346f00fe-if00-port0 \
  lidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_4a5b9018526eef11bff6e0c2c169b110-if00-port0 \
  mmwave_port:=/dev/ttyAMA0 \
  use_serial:=true \
  use_lidar:=true \
  use_mmwave:=true \
  "use_camera_mode4:=${use_camera_mode4}" \
  use_thermal_sensor:=false \
  use_rviz:=true \
  "drive_speed:=${drive_speed}" \
  turn_speed:=0.35 \
  event_replanning_enabled:=true \
  exit_switching_enabled:=true \
  waypoint_planning_enabled:=true \
  evacuation_demo_auto_start:=false \
  "${launch_args[@]}" 2>&1 | sed -u -n 's/^.*\[ROBOT\] //p'
launch_status=${PIPESTATUS[0]}
set -e

if (( launch_status != 0 && launch_status != 130 )); then
  printf '[오류] 모드 5 실행이 비정상 종료되었습니다. 상세 내용은 ~/.ros/log에 저장했습니다.\n'
fi
exit "${launch_status}"
