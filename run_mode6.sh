#!/usr/bin/env bash
# Mode 6: thermal-camera-only bench test.
# LiDAR/AMCL + MLX90640 live thermal overlay + RViz. Motor serial is off by default.
# No hazard belief / gas / planner / replanning.
set -euo pipefail

# Locate the repo from this script's own path, so it works from any checkout.
robot_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="${robot_root}/inno_jazzy_ws"
esp32_port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_de2033aed827f0119bb79ad8346f00fe-if00-port0'
lidar_port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_4a5b9018526eef11bff6e0c2c169b110-if00-port0'
use_serial='false'
launch_args=()
for argument in "$@"; do
  case "${argument}" in
    esp32_port:=*) esp32_port="${argument#esp32_port:=}" ;;
    lidar_port:=*) lidar_port="${argument#lidar_port:=}" ;;
    use_serial:=*) use_serial="${argument#use_serial:=}" ;;
    *) launch_args+=("${argument}") ;;
  esac
done
if [[ "${use_serial}" != 'true' && "${use_serial}" != 'false' ]]; then
  printf '[오류] use_serial은 true 또는 false여야 합니다: %s\n' "${use_serial}" >&2
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
stdbuf -oL -eL ros2 launch inno_robot_bringup mode6_thermal_preview.launch.py \
  "esp32_port:=${esp32_port}" \
  "lidar_port:=${lidar_port}" \
  "use_serial:=${use_serial}" \
  use_rviz:=true \
  "${launch_args[@]}"
launch_status=${PIPESTATUS[0]}
set -e

if (( launch_status != 0 && launch_status != 130 )); then
  printf '[오류] 모드 6 실행이 비정상 종료되었습니다. 상세 내용은 ~/.ros/log에 저장했습니다.\n'
fi
exit "${launch_status}"
