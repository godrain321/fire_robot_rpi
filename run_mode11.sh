#!/usr/bin/env bash
set -euo pipefail

robot_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export FIRE_ROBOT_RPI_ROOT="${robot_root}"
workspace="${robot_root}/inno_jazzy_ws"
cd "${workspace}"
set +u
source /opt/ros/humble/setup.bash
if [[ ! -f install_humble/setup.bash ]]; then
  printf '[오류] Humble workspace를 install_humble로 먼저 빌드하세요.\n' >&2
  exit 1
fi
source install_humble/setup.bash
set -u

exec ros2 launch inno_robot_bringup mode11_blackout.launch.py "$@"
