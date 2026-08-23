# Static-map waypoint drive test (no thermal/CO)

This profile is for the first Raspberry Pi 5 field test. It runs saved-map AMCL,
the 159-waypoint planner, PathSelector, follower, and optionally the ESP32 bridge.
Thermal, CO, hazard belief, dynamic obstacles, event replanning, exit evaluation,
and exit switching are disabled.

## Build

```bash
cd ~/Robot_project/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  inno_hazard inno_drive_bridge inno_robot_bringup inno_autonav
source install/setup.bash
```

## Test 1: motor OFF

Terminal 1 — localization and static waypoint navigation:

```bash
cd ~/Robot_project/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch inno_robot_bringup static_waypoint_drive.launch.py \
  use_serial:=false \
  lidar_port:=/dev/ttyUSB1 \
  use_rviz:=true
```

In RViz, set the initial pose with **2D Pose Estimate**. Do not send a goal until
`map -> odom -> base_link` is updating.

Terminal 2 — graph/topic preflight:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Robot_project/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
ros2 topic info /planned_path --verbose
ros2 topic info /goal_pose --verbose
ros2 topic echo /planning_grid --once
ros2 run tf2_ros tf2_echo map base_link
```

For this profile `/planned_path` must have one publisher, `path_selector_node`.
`/goal_pose` is published by `mission_commander`, never by the waypoint planner
or selector.

Terminal 3 — verify output before enabling any motor:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Robot_project/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
ros2 topic echo /planned_path
```

In another shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Robot_project/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
ros2 topic echo /cmd_vel
```

Terminal 4 — send an existing semantic destination:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Robot_project/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
ros2 run inno_autonav go_to exit2
```

Expected sequence:

```text
/goal_pose -> /waypoint_path -> /planned_path -> /cmd_vel
```

The waypoint planner logs `goal_to_path_ms=...`; this is the development hook
for goal-to-waypoint-path latency. With `use_serial:=false`, `/cmd_vel` is visible
but no ESP32 serial bridge is running and the motors do not move.

## Test 2: motor ON

Only after Test 1 has produced a correct path and velocity commands, stop the
launch and restart in a short, clear test lane:

```bash
cd ~/Robot_project/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch inno_robot_bringup static_waypoint_drive.launch.py \
  use_serial:=true \
  esp32_port:=/dev/ttyUSB0 \
  lidar_port:=/dev/ttyUSB1 \
  max_linear_speed:=0.06 \
  max_angular_speed:=0.45 \
  use_rviz:=true
```

Set the initial pose again, confirm TF, then run:

```bash
ros2 run inno_autonav go_to exit2
```

Keep an emergency-stop method within reach. Confirm that the TF pose updates,
the lookahead target progresses, `/follower_state` changes normally, `/cmd_vel`
is bounded, and the robot publishes zero velocity at `GOAL_REACHED`.

## RViz/topic checklist

- `/map`
- `/scan`
- `/planning_grid_static`
- `/planning_grid`
- `/waypoint_path`
- `/planned_path`
- TF: `map -> odom -> base_link -> laser`
- `/cmd_vel`
- `/follower_state`
- `/astar_path` only for a later synthetic fallback test

`planning_grid_publisher` loads the static navigation-map YAML and publishes
`/planning_grid_static`. `astar_replanner` remains present only as the existing
static/dynamic grid compositor and fallback planner; with thermal requirements,
dynamic obstacles, and hazard belief disabled, it republishes the static result
as `/planning_grid`. Direct `/goal_pose` planning in A* is disabled in this profile,
so normal goals are handled by the waypoint planner first.
