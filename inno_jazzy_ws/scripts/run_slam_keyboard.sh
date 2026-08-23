#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="${INNO_WS:-$(cd -- "$script_dir/.." && pwd)}"
set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "$workspace/install/setup.bash"
set -u
start=auto; port=/dev/ttyUSB0; rviz=true; extra=()
while (($#)); do case "$1" in
 --start-lidar) start=true;; --no-lidar) start=false;; --serial-port) shift; port="${1:?missing port}";;
 --no-rviz) rviz=false;; --help) echo "Usage: $0 [--start-lidar|--no-lidar] [--serial-port DEVICE] [--no-rviz] [launch_arg:=value ...]"; exit 0;;
 *) extra+=("$1");; esac; shift; done
if [[ $start == auto ]]; then
 if timeout 2 ros2 topic info /scan 2>/dev/null | grep -q 'Publisher count: [1-9]'; then start=false
 else [[ -e $port ]] || { echo "오류: /scan이 없고 $port 장치도 없습니다." >&2; exit 1; }; start=true; fi
fi
echo "[INNO] start_lidar=$start serial_port=$port use_rviz=$rviz"
exec ros2 run inno_robot_bringup slam_keyboard_runner -- "start_lidar:=$start" "serial_port:=$port" "use_rviz:=$rviz" "${extra[@]}"
