#!/usr/bin/env bash
set -euo pipefail

robot_root="/home/seeno04/fire_robot_rpi"
workspace="${robot_root}/inno_jazzy_ws"

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
  esp32_port:=/dev/ttyUSB0 \
  lidar_port:=/dev/ttyUSB1 \
  mmwave_port:=/dev/ttyAMA0 \
  map_yaml:="${robot_root}/maps/inno_map_raw.yaml" \
  planning_map_yaml:="${robot_root}/maps/inno_map_nav.yaml" \
  waypoint_file:="${robot_root}/maps/waypoint_queue_latest.yaml" \
  use_camera_mode4:=true \
  yolo_model_path:="${robot_root}/models/yolov8n_best.onnx" \
  drive_speed:=0.06 \
  turn_speed:=0.35 \
  start_thermal_viewer:=false \
  "$@" 2>&1 | sed -u -n 's/^.*\[ROBOT\] //p'
launch_status=${PIPESTATUS[0]}
set -e

if (( launch_status != 0 && launch_status != 130 )); then
  printf '[오류] 통합 실행이 비정상 종료되었습니다. 상세 내용은 ~/.ros/log에 저장했습니다.\n'
fi
exit "${launch_status}"
