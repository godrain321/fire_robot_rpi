#!/usr/bin/env bash
set -euo pipefail

robot_root="/home/seeno04/fire_robot_rpi"
workspace="${robot_root}/inno_jazzy_ws"
drive_speed='0.06'
launch_args=()
for argument in "$@"; do
  case "${argument}" in
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
source /opt/ros/jazzy/setup.bash

if [[ ! -f install/setup.bash ]]; then
  printf '[오류] ROS workspace가 빌드되지 않았습니다. 먼저 colcon build를 실행하세요.\n'
  exit 1
fi
source install/setup.bash
set -u

# All ROS internals remain in ~/.ros/log.  The terminal receives only lines
# explicitly emitted by the Korean operator console.
set +e
stdbuf -oL -eL ros2 launch inno_robot_bringup field_waypoint_test.launch.py \
  esp32_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_de2033aed827f0119bb79ad8346f00fe-if00-port0 \
  lidar_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_4a5b9018526eef11bff6e0c2c169b110-if00-port0 \
  mmwave_port:=/dev/ttyAMA0 \
  map_yaml:="${robot_root}/maps/inno_map_raw.yaml" \
  planning_map_yaml:="${robot_root}/maps/inno_map_nav.yaml" \
  waypoint_file:="${robot_root}/maps/waypoint_queue_latest.yaml" \
  use_serial:=true \
  use_lidar:=true \
  use_mmwave:=true \
  use_rviz:=true \
  set_initial_pose:=false \
  auto_localization:=true \
  use_dynamic_obstacles:=true \
  require_thermal_grid:=false \
  require_thermal_active:=false \
  waypoint_planning_enabled:=false \
  waypoint_accept_direct_goal:=false \
  astar_accept_goal_pose:=true \
  mode3_standoff_distance_m:=2.0 \
  mode3_publish_canonical_plan:=false \
  mode4_standoff_distance_m:=2.0 \
  mode4_publish_canonical_plan:=false \
  event_replanning_enabled:=false \
  exit_switching_enabled:=false \
  use_camera_mode4:=true \
  yolo_model_path:="${robot_root}/models/yolov8n_best_opencv_640.onnx" \
  "drive_speed:=${drive_speed}" \
  turn_speed:=0.35 \
  start_thermal_viewer:=false \
  "${launch_args[@]}" 2>&1 | sed -u -n 's/^.*\[ROBOT\] //p'
launch_status=${PIPESTATUS[0]}
set -e

if (( launch_status != 0 && launch_status != 130 )); then
  printf '[오류] 통합 실행이 비정상 종료되었습니다. 상세 내용은 ~/.ros/log에 저장했습니다.\n'
fi
exit "${launch_status}"
