# factory_v3 simulation waypoints

`export_simulation_waypoints.py` converts the simulation's exported, metre-unit
`factory_v3_world_xy_m` trajectory into the PoseArray-style YAML consumed by
`inno_autonav/waypoint_queue.py`.  It performs no ROS publication and never
sends a motor command.

## Coordinate alignment

`factory_v3_to_map.yaml` contains the measured rigid transform fitted from the
EXIT1, EXIT2, EXIT3 and INIT pairs in
`fire_robot/simulator/factory_v3/config/semantic_points.yaml`:

```text
x_map = cos(60.2998258245°) x_sim - sin(60.2998258245°) y_sim + 13.00189199
y_map = sin(60.2998258245°) x_sim + cos(60.2998258245°) y_sim - 29.41813371
```

The largest residual at those four control points is below 0.000001 m.  The
ROS occupancy map is `maps/inno_map_nav.yaml`: 0.05 m/cell, origin
`[-7.521, -17.712, 0]`, image size 543 x 453.  PGM row zero is flipped to the
bottom before occupancy checks, matching `nav_msgs/OccupancyGrid`.

## Convert and validate

From the `fire_robot_rpi` repository root:

```bash
python3 pathplaning/export_simulation_waypoints.py \
  --input ../fire_robot/simulator/factory_v3/output/final_actual_robot_path.yaml \
  --map-yaml maps/inno_map_nav.yaml \
  --transform pathplaning/factory_v3_to_map.yaml \
  --output pathplaning/simulation_waypoint_queue.yaml \
  --dry-run
```

Remove `--dry-run` to write the file.  Existing outputs are protected unless
`--overwrite` is explicitly supplied.  Unknown map cells are rejected by
default.  The exporter checks each waypoint and every simplified segment
against the selected occupancy map.  Dynamic obstacles and fire costs are not
present in the intermediate trajectory, so the real ROS A* node must still use
its latest `/planning_grid` before every waypoint is driven.

## ROS field launch (manual only)

The existing launch already exposes `waypoint_file`; its default remains empty.
To load the generated queue deliberately:

```bash
ros2 launch inno_robot_bringup field_waypoint_test.launch.py \
  map_yaml:=$(pwd)/maps/inno_map_nav.yaml \
  waypoint_file:=$(pwd)/pathplaning/simulation_waypoint_queue.yaml
```

Loading only restores and displays the queue.  Sending `GO` on
`/waypoint_queue_command` starts actual motion and is intentionally not done by
the converter or its tests.  Before field use, verify localization, map
version, emergency-stop distance, the full queue in RViz, and a lifted-wheel or
low-speed test.
