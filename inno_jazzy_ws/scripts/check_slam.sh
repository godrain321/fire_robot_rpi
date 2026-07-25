#!/usr/bin/env bash
set -o pipefail
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "${INNO_WS:-$HOME/inno_jazzy_ws}/install/setup.bash"
set -u
echo '=== Topics ==='
for t in /scan /map /odom_rf2o /rf2o_path; do
 echo "[$t]"; ros2 topic info "$t" -v 2>/dev/null || echo MISSING
 timeout 6 ros2 topic hz "$t" --window 10 2>/dev/null | tail -n 3 || true
done
echo '=== TF ==='
for pair in 'map odom' 'odom base_link' 'base_link laser'; do read -r parent child <<<"$pair"; set -- "$parent" "$child"; echo "[$1 -> $2]"; timeout 3 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 | tail -n 8; done
echo '=== Map ==='
timeout 5 ros2 topic echo /map --once --field info 2>/dev/null || true
echo '=== TF broadcasters (inspect duplicates) ==='
ros2 topic info /tf -v 2>/dev/null || true; ros2 topic info /tf_static -v 2>/dev/null || true
echo "Maps: ${INNO_WS:-$HOME/inno_jazzy_ws}/maps"; ls -l "${INNO_WS:-$HOME/inno_jazzy_ws}/maps" 2>/dev/null || true
