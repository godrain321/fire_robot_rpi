#!/usr/bin/env bash
set -euo pipefail

robot_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="${robot_root}/inno_jazzy_ws"
cd "${workspace}"
set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
  printf '[오류] ROS workspace를 먼저 colcon build로 빌드하세요.\n' >&2
  exit 1
fi
source install/setup.bash
set -u

exec ros2 launch inno_robot_bringup mode10_ultrasonic_avoidance.launch.py \
  esp32_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_de2033aed827f0119bb79ad8346f00fe-if00-port0 \
  lidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_4a5b9018526eef11bff6e0c2c169b110-if00-port0 \
  "$@"
