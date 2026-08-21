#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
timestamp="$(date '+%Y%m%d_%H%M%S')"
output="${project_root}/bags/fire_robot_${timestamp}"
check_seconds='2.0'
check_only='false'
declare -a extra_topics=()

declare -a topics=(
  /tf
  /tf_static
  /scan
  /map
  /odom_rf2o
  /amcl_pose
  /lidar_path
  /wheel_odom
  /wheel_path
  /cmd_vel
  /cmd_vel_keyboard
  /cmd_vel_auto
  /motor/left_steps_per_sec
  /motor/right_steps_per_sec
  /wheel_ticks
  /esp32/status
  /drive_mode
  /drive_mode_status
  /initialpose
  /goal_pose
  /waypoint_click
  /waypoint_queue
  /waypoint_poses
  /waypoint_markers
  /waypoint_queue_status
  /waypoint_queue_command
  /obstacle_inspection_command
  /planned_path
  /planner_state
  /follower_state
  /planning_grid
  /dynamic_obstacle_grid
  /dynamic_obstacle_markers
  /dynamic_obstacle_candidates
  /dynamic_obstacle_person
  /dynamic_obstacle_observations
  /dynamic_obstacle_markers_display
  /dynamic_obstacle_detected
  /mode3_status
  /mode3_classification
  /mode4_status
  /mode4_classification
  /autonomy_cancel
  /camera/image_raw
  /camera/camera_info
  /camera/person_detections
  /camera/person_detector_status
  /camera/person_detection_image
  /victim_markers
  /victim_detected
  /victim_fusion_status
  /mmwave/raw/presence
  /mmwave/raw/distance_m
  /mmwave/raw/speed_mps
  /mmwave/raw/energy_raw
  /mmwave/filtered_presence
  /mmwave/filtered_distance_m
  /mmwave/filtered_speed_mps
  /mmwave/motion_activity
  /mmwave/filter_state
  /mmwave/presence
  /mmwave/distance_m
  /mmwave/speed_mps
  /mmwave/energy_raw
  /mmwave/sensor_state
  /mmwave/mobility_state
  /mmwave/still_duration_sec
)

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  printf '%s\n' \
    'Record the complete fire-robot driving/sensor rosbag.' \
    '' \
    'Usage: ./record_robot_bag.sh [options]' \
    '' \
    '  --output PATH         Bag directory (default: bags/fire_robot_TIMESTAMP)' \
    '  --check-seconds SEC   Preflight sample window (default: 2.0)' \
    '  --extra-topic TOPIC   Add another topic; may be repeated' \
    '  --check-only          Print topic status without starting rosbag' \
    '  -h, --help            Show this help' \
    '' \
    'Missing publishers or samples are warnings only. Every listed topic remains' \
    'in the rosbag request so it can be recorded if it appears later.'
}

positive_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] && \
    awk -v value="$1" 'BEGIN { exit !(value > 0) }'
}

source_ros() {
  local ros_setup="${ROS_SETUP_FILE:-/opt/ros/jazzy/setup.bash}"
  local robot_setup="${ROBOT_WS_SETUP_FILE:-${project_root}/inno_jazzy_ws/install/local_setup.bash}"
  [[ -r "${ros_setup}" ]] || die "ROS 2 Jazzy is missing: ${ros_setup}"
  [[ -r "${robot_setup}" ]] || die \
    "inno_jazzy_ws is not built: ${robot_setup}"
  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  # shellcheck disable=SC1090
  source "${robot_setup}"
  set -u
  command -v ros2 >/dev/null 2>&1 || die 'ros2 is unavailable.'
  ros2 pkg prefix inno_robot_bringup >/dev/null 2>&1 || \
    die 'inno_robot_bringup is unavailable in the sourced workspace.'
}

while (( $# > 0 )); do
  case "$1" in
    --output|--check-seconds|--extra-topic)
      (( $# >= 2 )) || die "$1 requires a value."
      case "$1" in
        --output) output="$2" ;;
        --check-seconds) check_seconds="$2" ;;
        --extra-topic) extra_topics+=("$2") ;;
      esac
      shift 2
      ;;
    --check-only)
      check_only='true'
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown option: $1 (use --help)"
      ;;
  esac
done

positive_number "${check_seconds}" || die '--check-seconds must be positive.'
if [[ "${output}" != /* ]]; then
  output="${project_root}/${output}"
fi
for topic in "${extra_topics[@]}"; do
  [[ "${topic}" == /* ]] || die "Topic must start with '/': ${topic}"
  topics+=("${topic}")
done

source_ros
status_file="${output}.topics.txt"
mkdir -p -- "$(dirname -- "${output}")"
if [[ "${check_only}" != 'true' ]]; then
  [[ ! -e "${output}" ]] || die "Bag output already exists: ${output}"
  [[ ! -e "${status_file}" ]] || die "Status file already exists: ${status_file}"
fi

printf 'Rosbag output: %s\n' "${output}"
printf 'Checking %d topics; missing data will not stop recording.\n' "${#topics[@]}"
if [[ "${check_only}" == 'true' ]]; then
  ros2 run inno_robot_bringup bag_topic_preflight \
    --wait "${check_seconds}" "${topics[@]}"
  exit 0
fi

ros2 run inno_robot_bringup bag_topic_preflight \
  --wait "${check_seconds}" "${topics[@]}" | tee "${status_file}"

printf '\nStarting rosbag. Stop safely with Ctrl-C.\n'
printf 'Topic status sidecar: %s\n' "${status_file}"
exec ros2 bag record -o "${output}" "${topics[@]}"
