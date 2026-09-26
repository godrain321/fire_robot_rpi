#!/usr/bin/env bash
# Final operator profile: 1=manual, 2=integrated evacuation, 3=greeting spin.
set -euo pipefail

robot_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="${robot_root}/inno_jazzy_ws"
esp32_port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_de2033aed827f0119bb79ad8346f00fe-if00-port0'
lidar_port='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_4a5b9018526eef11bff6e0c2c169b110-if00-port0'
mmwave_port='/dev/ttyAMA0'
drive_speed='0.06'
launch_args=()
for argument in "$@"; do
  case "${argument}" in
    esp32_port:=*) esp32_port="${argument#esp32_port:=}" ;;
    lidar_port:=*) lidar_port="${argument#lidar_port:=}" ;;
    mmwave_port:=*) mmwave_port="${argument#mmwave_port:=}" ;;
    drive_speed:=*) drive_speed="${argument#drive_speed:=}" ;;
    *) launch_args+=("${argument}") ;;
  esac
done
if ! [[ "${drive_speed}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] || \
    ! awk -v value="${drive_speed}" 'BEGIN { exit !(value > 0) }'; then
  printf '[오류] drive_speed는 0보다 큰 숫자여야 합니다: %s\n' "${drive_speed}" >&2
  exit 2
fi

cd "${workspace}"
set +u
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  ros_setup='/opt/ros/jazzy/setup.bash'
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  ros_setup='/opt/ros/humble/setup.bash'
else
  printf '[오류] ROS 2 Jazzy 또는 Humble 설치를 찾지 못했습니다.\n' >&2
  exit 1
fi
source "${ros_setup}"
if [[ ! -f install/setup.bash ]]; then
  printf '[오류] ROS workspace가 빌드되지 않았습니다. 먼저 colcon build를 실행하세요.\n' >&2
  exit 1
fi
source install/setup.bash
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

printf '============================================================\n'
printf 'Fire Robot: 1=키보드 / 2=통합 화재대피 / 3=안내 회전\n'
printf 'Mode 2 중 P: LiDAR 위치추정 blackout → Encoder+IMU 전환\n'
printf 'c 또는 s: 현재 자동 동작 취소 후 Mode 1 복귀\n'
printf '============================================================\n'

set +e
ros2 launch inno_robot_bringup mode2_integrated_fire_evacuation.launch.py \
  "esp32_port:=${esp32_port}" \
  "lidar_port:=${lidar_port}" \
  "mmwave_port:=${mmwave_port}" \
  map_yaml:="${robot_root}/maps/inno_map_raw.yaml" \
  planning_map_yaml:="${robot_root}/maps/inno_map_nav.yaml" \
  waypoint_file:="${robot_root}/maps/waypoint_queue_latest.yaml" \
  yolo_model_path:="${robot_root}/models/yolov8n_best_opencv_640.onnx" \
  use_serial:=true \
  use_lidar:=true \
  use_mmwave:=true \
  use_camera_mode4:=true \
  use_gas_sensor:=true \
  use_rviz:=true \
  "drive_speed:=${drive_speed}" \
  turn_speed:=0.35 \
  event_replanning_enabled:=true \
  exit_switching_enabled:=true \
  waypoint_planning_enabled:=true \
  evacuation_demo_auto_start:=false \
  "${launch_args[@]}"
launch_status=$?
set -e

if (( launch_status != 0 && launch_status != 130 && launch_status != 254 )); then
  printf '[오류] 통합 실행이 비정상 종료되었습니다. 상세 내용은 ~/.ros/log에 저장했습니다.\n' >&2
fi
exit "${launch_status}"
