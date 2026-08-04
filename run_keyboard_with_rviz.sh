#!/usr/bin/env bash
set -e
set +u

WORKSPACE_DIR="$HOME/fire_robot_rpi/inno_jazzy_ws"
ROS_SETUP="$WORKSPACE_DIR/install/setup.bash"
ROS_BASE_SETUP="/opt/ros/jazzy/setup.bash"
SERIAL_PORT="${1:-/dev/ttyUSB0}"
LINEAR_SPEED="${2:-0.08}"
ANGULAR_SPEED="${3:-0.35}"

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
echo "  1) ESP32 mode: printf '1\\n' > $SERIAL_PORT"
echo "  2) Drive with W/A/S/D"
echo "  3) In RViz, add Path topic: /wheel_path"
echo "  4) Later, use path_waypoint_recorder to save waypoints"
echo ""

# Start the keyboard drive node in the background.
ros2 launch inno_drive_bridge drive_keyboard_demo.launch.py \
  serial_port:="$SERIAL_PORT" \
  linear_speed:="$LINEAR_SPEED" \
  angular_speed:="$ANGULAR_SPEED" &

DRIVE_PID=$!

# Start map server and RViz in the background.
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$HOME/fire_robot_rpi/maps/inno_map_nav.yaml &
MAP_SERVER_PID=$!

sleep 2
ros2 lifecycle set /map_server configure >/dev/null 2>&1 || true
ros2 lifecycle set /map_server activate >/dev/null 2>&1 || true

rviz2 -d "$HOME/fire_robot_rpi/keyboard_rviz.rviz" &
RVIZ_PID=$!

cleanup() {
  kill "$DRIVE_PID" "$RVIZ_PID" "$MAP_SERVER_PID" 2>/dev/null || true
  wait "$DRIVE_PID" "$RVIZ_PID" "$MAP_SERVER_PID" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

wait "$DRIVE_PID"
