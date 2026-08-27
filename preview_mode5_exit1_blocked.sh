#!/usr/bin/env bash
set -euo pipefail

cd /home/seeno04/fire_robot_rpi/inno_jazzy_ws
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

exec ros2 launch inno_robot_bringup mode5_route_preview.launch.py
