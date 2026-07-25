#!/usr/bin/env bash
set -euo pipefail
set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "${INNO_WS:-$HOME/inno_jazzy_ws}/install/setup.bash"
set -u
maps="${INNO_WS:-$HOME/inno_jazzy_ws}/maps"; mkdir -p "$maps"
name="${1:-inno_map_$(date +%Y%m%d_%H%M%S)}"; base="$maps/$name"
timeout 6 ros2 topic echo /map --once >/dev/null || { echo "오류: /map 메시지가 없습니다." >&2; exit 1; }
ros2 run nav2_map_server map_saver_cli -t /map -f "$base"
[[ -f $base.yaml && -f $base.pgm ]] || { echo "오류: 저장 결과 파일이 없습니다." >&2; exit 1; }
ln -sfn "$(basename "$base.yaml")" "$maps/latest_map.yaml"; ln -sfn "$(basename "$base.pgm")" "$maps/latest_map.pgm"
echo "저장 성공: $base.yaml"; echo "저장 성공: $base.pgm"
