# inno_thermal

ROS 2 Jazzy package for reading the repository's MLX90640 native driver and
publishing raw temperatures plus a 15 cm direction arc. It intentionally does
not publish TF/map data or modify navigation and costmaps.

## Sensor and array convention

The native API writes 768 `float` values. The legacy demo indexes them as
`raw[32 * row + column]`, so this package reshapes the buffer in C order to
`(24, 32)`. No display colors, resizing, Pillow operations, or blur are used.

`flip_horizontal`, `flip_vertical`, and `rotate_180` are applied to the raw
24x32 array before column maxima are calculated. Raspberry Pi hardware test 2
confirmed that the native order is horizontally mirrored on the installed
camera, so `flip_horizontal` now defaults to `true`. Physical left therefore
maps to column 0 and ROS `+y`; `flip_vertical` and `rotate_180` remain `false`.

The sensor node rejects a complete frame if any of its 768 samples is NaN or
infinite and retries on the next timer tick. The pure `compute_column_max()`
helper is more reusable: it ignores isolated NaNs with `nanmax`, but raises
`ValueError` when an entire column is NaN.

## Raspberry Pi I2C setup

Enable I2C in the Raspberry Pi firmware/configuration, reboot if required, and
verify the device and permissions:

```bash
ls -l /dev/i2c-1
groups
sudo usermod -aG i2c "$USER"   # log out and back in after this command
sudo apt install i2c-tools
i2cdetect -y 1
```

The table printed by `i2cdetect` should contain `33` (hexadecimal address
`0x33`). Do not run the legacy demo and this ROS node at the same time.

## Build the native driver

The existing core C++ sources remain in
`mlx90640/demo codes/mlx90640/python/lib/`; they are not duplicated. From the
repository root:

```bash
./inno_jazzy_ws/src/inno_thermal/scripts/build_native_driver.sh
```

This creates the ignored file
`inno_jazzy_ws/src/inno_thermal/native/libmlx90640.so`. Build it before
`colcon build` so it is installed into the package share directory. An explicit
absolute library can instead be selected with the ROS parameter
`native_library_path` or environment variable
`INNO_THERMAL_MLX90640_LIBRARY`. The node never relies on the launch working
directory.

The package contains only a small status bridge around the existing driver.
Unlike the legacy `void Get_temp_val()` API, it propagates initialization and
I2C frame-read failures to Python and closes the driver's file descriptor on
shutdown. The MLX90640 API and Linux I2C implementation remain single-sourced
in the original driver directory.

The supplied wrapper accepts integer refresh rates. Supported values are
`1, 2, 4, 8, 16, 32, 64` Hz; other values fail during node startup with a clear
error.

## Build and run

```bash
cd inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select inno_thermal
source install/setup.bash
ros2 launch inno_thermal thermal_sensor.launch.py
```

Direct execution is also available:

```bash
ros2 run inno_thermal mlx90640_sensor_node
```

Published topics:

- `/thermal/image` (`sensor_msgs/msg/Image`, `32FC1`, 24x32 Celsius values)
- `/thermal/column_max` (`std_msgs/msg/Float32MultiArray`, 32 temperatures)
- `/thermal/arc_points` (`sensor_msgs/msg/PointCloud2`, fields `x,y,z,intensity`)

All stamped messages use `thermal_camera_link`. Image and point cloud use the
same measurement timestamp. Arc coordinates use ROS camera mounting geometry
`x` forward, `y` left, `z` up, and have a default radius of 0.15 m.

## Thermal cost layer

`thermal_cost_layer` consumes the 32-point temperature arc and projects it into
a separate map-frame OccupancyGrid. It never edits `/planning_grid_static` and
does not merge its result into `/planning_grid`.

Inputs:

- `/planning_grid_static` (`nav_msgs/msg/OccupancyGrid`, transient-local)
- `/thermal/arc_points` (`sensor_msgs/msg/PointCloud2`)

Outputs and service:

- `/thermal_cost_grid` (`nav_msgs/msg/OccupancyGrid`, transient-local)
- `/thermal_cost_status` (`std_msgs/msg/String`, transient-local)
- `/clear_thermal_costs` (`std_srvs/srv/Trigger`)

The thermal grid copies the static grid's frame, resolution, width, height,
origin position, and origin quaternion exactly. Static occupancy values are not
copied: every unobserved thermal cell is zero and `-1` is never used. This exact
geometry match allows later consumers to combine corresponding cell indices
without resampling, while keeping the static map immutable.

For temperature `T`, safe threshold `Ts`, and blocked threshold `Tb`, the grid
stores the linear normalized temperature ratio:

```text
ratio = clamp((T - Ts) / (Tb - Ts), 0, 1)
```

- `T <= Ts`: cost 0
- `Ts < T < Tb`: `max(1, min(99, round(99 * ratio)))`
- `T >= Tb`: cost 100

Defaults are 20 °C, 60 °C, and encoding power 1. The nonlinear factory_v5
temperature exponent is applied once by `inno_autonav`, avoiding the old
double-exponent behavior. When multiple arc points enter one map cell, only
that frame's maximum cost is used. This is needed because the 32
points on a 15 cm arc can quantize into fewer map cells.

Each nonzero cell stores its last observation in ROS clock time. A newly
observed value replaces the previous value, including a safe reading clearing a
hot cell. Unobserved cells remain for `observation_timeout_sec` (default 2 s)
and then return to zero. With timeout exactly `0.0`, old cells are cleared on
the next thermal frame or publish timer cycle. Values are not permanent maxima.

Sensor-stream liveness is checked separately from cell expiry. If no
`/thermal/arc_points` message arrives for `thermal_data_timeout_sec` (default
1.0 s), status changes from `ACTIVE` to `THERMAL_DATA_STALE`. Costs still obey
their independent 2.0 s observation timeout. A later valid frame automatically
restores `ACTIVE`. All timeout calculations use the ROS clock, including
`use_sim_time`; a backward clock jump also enters the stale fail-safe state.

`inflation_radius_m` defaults to `0.0`, which writes only the exact observed
cells. A positive radius applies bounded Euclidean inflation with decreasing
cost away from the observation; the centre retains its original value.

Clear all current observations immediately with:

```bash
ros2 service call /clear_thermal_costs std_srvs/srv/Trigger "{}"
```

The node obtains one timestamped TF per PointCloud2 frame and applies its full
translation and quaternion rotation to all 32 points. The effective target is
always the actual `/planning_grid_static` frame. `target_frame` is a consistency
check and causes a warning if it differs. No identity transform is invented. A
missing `map -> ... -> thermal_camera_link` chain produces
`WAITING_FOR_TF`; this is a normal waiting state, the node stays alive, publishes
an all-zero grid once static geometry is known, and continues expiring old
observations. If source and target frame strings are already equal, no TF lookup
is needed.

The measured camera mounting transform must be published elsewhere. Command
format for a temporary measured static transform:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x <camera_x_m> \
  --y <camera_y_m> \
  --z <camera_z_m> \
  --roll <roll_rad> \
  --pitch <pitch_rad> \
  --yaw <yaw_rad> \
  --frame-id base_link \
  --child-frame-id thermal_camera_link
```

Run the cost node alone or select it from the combined launch:

```bash
ros2 run inno_thermal thermal_cost_layer
ros2 launch inno_thermal thermal_sensor.launch.py enable_cost_layer:=true
ros2 launch inno_thermal thermal_sensor.launch.py enable_cost_layer:=false
```

## RViz

Set `Fixed Frame` to `thermal_camera_link`, add a PointCloud2 display for
`/thermal/arc_points`, and select `Intensity` as the Color Transformer. There
are 32 points, with each intensity equal to that image column's maximum raw
temperature.

For map-frame cost inspection, use `map` as Fixed Frame and add:

- Map `/planning_grid_static` with reduced Alpha
- Map `/thermal_cost_grid` above it
- PointCloud2 `/thermal/arc_points`, Color Transformer `Intensity`
- TF display

Check that left/right heat affects the corresponding robot-relative side, that
the arc rotates and translates with the robot in the map, stale arcs disappear
after two seconds, and temperatures at or above 60 °C produce cost 100. The
static map must remain unchanged.

## Tests

Geometry tests need no camera or I2C device:

```bash
cd inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select inno_thermal
colcon test-result --verbose
```

## Deliberately not implemented

- physical vertical camera orientation calibration
- `base_link -> thermal_camera_link` TF
- merging `/thermal_cost_grid` into `/planning_grid`
- a Nav2 costmap plugin
- A* path integration (the current thermal grid is not consumed by planning)
- real driving integration

The next integration step should update `inno_autonav`'s `astar_replanner` to
subscribe to `/thermal_cost_grid`, validate that its geometry matches
`/planning_grid_static`, and combine thermal values with the planner's working
grid under an explicitly chosen cost/blocking policy. That integration is not
part of this package or this change.
