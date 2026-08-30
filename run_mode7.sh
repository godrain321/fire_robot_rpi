#!/usr/bin/env bash
# Mode 7: autonomous driving on the thermal-camera hazard cost only (gas layer OFF).
# Waypoint + A* + Stage 6 replanning, hazard belief thermal-only, keyboard override.
set -euo pipefail

# Locate the repo from this script's own path, so it works from any checkout.
robot_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="${robot_root}/inno_jazzy_ws"
esp32_port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_de2033aed827f0119bb79ad8346f00fe-if00-port0'
lidar_port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_4a5b9018526eef11bff6e0c2c169b110-if00-port0'
drive_speed='0.06'
use_serial='true'
mode7_auto_start='false'
launch_args=()
for argument in "$@"; do
  case "${argument}" in
    esp32_port:=*) esp32_port="${argument#esp32_port:=}" ;;
    lidar_port:=*) lidar_port="${argument#lidar_port:=}" ;;
    drive_speed:=*) drive_speed="${argument#drive_speed:=}" ;;
    use_serial:=*) use_serial="${argument#use_serial:=}" ;;
    mode7_auto_start:=*) mode7_auto_start="${argument#mode7_auto_start:=}" ;;
    *) launch_args+=("${argument}") ;;
  esac
done
if ! [[ "${drive_speed}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || \
    ! awk -v value="${drive_speed}" 'BEGIN { exit !(value > 0) }'; then
  printf '[오류] drive_speed는 0보다 큰 숫자여야 합니다: %s\n' "${drive_speed}" >&2
  exit 2
fi
for boolean_name in use_serial mode7_auto_start; do
  boolean_value="${!boolean_name}"
  if [[ "${boolean_value}" != 'true' && "${boolean_value}" != 'false' ]]; then
    printf '[오류] %s은 true 또는 false여야 합니다: %s\n' \
      "${boolean_name}" "${boolean_value}" >&2
    exit 2
  fi
done

cd "${workspace}"
set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
  printf '[오류] ROS workspace가 빌드되지 않았습니다. 먼저 colcon build를 실행하세요.\n'
  exit 1
fi
source install/setup.bash
set -u

set +e
stdbuf -oL -eL ros2 launch inno_robot_bringup mode7_thermal_drive.launch.py \
  "esp32_port:=${esp32_port}" \
  "lidar_port:=${lidar_port}" \
  "drive_speed:=${drive_speed}" \
  "use_serial:=${use_serial}" \
  "mode7_auto_start:=${mode7_auto_start}" \
  use_rviz:=true \
  "${launch_args[@]}"
launch_status=${PIPESTATUS[0]}
set -e

if (( launch_status != 0 && launch_status != 130 )); then
  printf '[오류] 모드 7 실행이 비정상 종료되었습니다. 상세 내용은 ~/.ros/log에 저장했습니다.\n'
fi
exit "${launch_status}"
