#!/usr/bin/env bash
# Mode 8: full Mode 5 evacuation mission + MLX90640 thermal costmap.
# Same as run_mode5.sh but launches mode8_evacuation_thermal.launch.py, which
# runs Mode 5 with use_thermal_sensor:=true and a thermal-aware RViz.
set -euo pipefail

# Locate the repo from this script's own path, so it works from any checkout.
robot_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="${robot_root}/inno_jazzy_ws"
esp32_port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_de2033aed827f0119bb79ad8346f00fe-if00-port0'
lidar_port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_4a5b9018526eef11bff6e0c2c169b110-if00-port0'
# Raise straight-line velocity only. The angular speed passed below remains
# unchanged, and the path follower still sets linear.x=0 while rotating.
drive_speed='0.15'
# Mode 8 uses LiDAR/mmWave/thermal sensing by default. Keep the camera and
# YOLO person detector stopped unless the operator explicitly opts in.
use_camera_mode4='false'
launch_args=()
for argument in "$@"; do
  case "${argument}" in
    esp32_port:=*) esp32_port="${argument#esp32_port:=}" ;;
    lidar_port:=*) lidar_port="${argument#lidar_port:=}" ;;
    drive_speed:=*) drive_speed="${argument#drive_speed:=}" ;;
    use_camera_mode4:=*) use_camera_mode4="${argument#use_camera_mode4:=}" ;;
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
stdbuf -oL -eL ros2 launch inno_robot_bringup mode8_evacuation_thermal.launch.py \
  "esp32_port:=${esp32_port}" \
  "lidar_port:=${lidar_port}" \
  mmwave_port:=/dev/ttyAMA0 \
  use_serial:=true \
  use_lidar:=true \
  use_mmwave:=true \
  "use_camera_mode4:=${use_camera_mode4}" \
  use_rviz:=true \
  "drive_speed:=${drive_speed}" \
  turn_speed:=0.64 \
  event_replanning_enabled:=true \
  exit_switching_enabled:=true \
  waypoint_planning_enabled:=true \
  evacuation_demo_auto_start:=false \
  "${launch_args[@]}" 2>&1 | sed -u -n 's/^.*\[ROBOT\] //p'
launch_status=${PIPESTATUS[0]}
set -e

# ros2 launch can report Python SIGINT (-2) as shell status 254, especially
# when a second Ctrl+C arrives while child nodes are shutting down.
if (( launch_status != 0 && launch_status != 130 && launch_status != 254 )); then
  printf '[오류] 모드 8 실행이 비정상 종료되었습니다. 상세 내용은 ~/.ros/log에 저장했습니다.\n'
fi
exit "${launch_status}"
