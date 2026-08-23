#!/usr/bin/env bash
set -e
set +u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${FIRE_ROBOT_RPI_ROOT:-$SCRIPT_DIR}"
WORKSPACE_DIR="${INNO_WS:-$PROJECT_ROOT/inno_jazzy_ws}"
ROS_SETUP="$WORKSPACE_DIR/install/setup.bash"
ROS_BASE_SETUP="/opt/ros/jazzy/setup.bash"
SERIAL_PORT="${1:-/dev/ttyUSB0}"
LINEAR_SPEED="${2:-0.08}"
ANGULAR_SPEED="${3:-0.35}"
MAP_YAML="${4:-$PROJECT_ROOT/maps/inno_map_raw.yaml}"

if [ ! -f "$ROS_BASE_SETUP" ]; then
  echo "ROS 2 Jazzy setup not found: $ROS_BASE_SETUP"
  exit 1
fi

if [ ! -f "$ROS_SETUP" ]; then
  echo "Workspace setup not found: $ROS_SETUP"
  echo "Build the workspace first with: colcon build --symlink-install"
  exit 1
fi

source "$ROS_BASE_SETUP"
source "$ROS_SETUP"

echo "Starting keyboard drive + RViz"
echo "Serial port: $SERIAL_PORT"
echo "Linear speed: $LINEAR_SPEED"
echo "Angular speed: $ANGULAR_SPEED"
echo ""
echo "Next steps:"
echo "  1) Press 1 in the keyboard-drive terminal"
echo "  2) Drive with W/A/X/D and stop with S"
echo "  3) In RViz, add Path topic: /wheel_path"
echo "Map: $MAP_YAML"
echo ""

# Start the keyboard drive node in the background.
ros2 launch inno_drive_bridge drive_keyboard_demo.launch.py \
  serial_port:="$SERIAL_PORT" \
  linear_speed:="$LINEAR_SPEED" \
  angular_speed:="$ANGULAR_SPEED" &

DRIVE_PID=$!

# Start map server and RViz in the background.
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$MAP_YAML" &
MAP_SERVER_PID=$!

sleep 2
ros2 lifecycle set /map_server configure >/dev/null 2>&1 || true
ros2 lifecycle set /map_server activate >/dev/null 2>&1 || true

rviz2 -d "$PROJECT_ROOT/keyboard_rviz.rviz" &
RVIZ_PID=$!

cleanup() {
  kill "$DRIVE_PID" "$RVIZ_PID" "$MAP_SERVER_PID" 2>/dev/null || true
  wait "$DRIVE_PID" "$RVIZ_PID" "$MAP_SERVER_PID" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

wait "$DRIVE_PID"
