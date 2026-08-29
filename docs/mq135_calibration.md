# MQ-135 ADC threshold calibration (Stage 7)

The MQ-135 → ESP32 → `/mq135/filtered_adc` chain (Stage 2) and the gas cost /
planner / replanning pipeline (Stages 3–6) are already in place. What is **not**
done is choosing the two numbers that decide when gas matters:

| parameter | meaning |
|---|---|
| `gas_safe_adc` | filtered ADC at/below this ⇒ gas cost = 0 |
| `gas_blocked_adc` | filtered ADC at/above this ⇒ cell is lethal (planner + replanner avoid it) |

Between them the cost rises smoothly (see "How the ADC becomes cost" below).

> **The `/mq135/filtered_adc` value is a raw ESP32 ADC reading, NOT ppm.**
> It reacts to many gases and drifts with the specific sensor and its
> installation. There is no correct default. The placeholders shipped in
> `inno_hazard/config/hazard_params.yaml` (`gas_safe_adc: 0.0`,
> `gas_blocked_adc: 4096.0`, `gas_input_mode: legacy_ppm`) are deliberately inert
> — do not treat them as real hazard thresholds. You must measure your own
> sensor and pick the values from that data plus real experiments in a
> known-safe test setup.

## How the ADC becomes cost (do not change this in Stage 7)

`inno_hazard/inno_hazard/hazard_belief.py`, `HazardBelief.recalculate()`:

```
safe    = gas_safe_adc      (gas_input_mode == "adc")
blocked = gas_blocked_adc
ratio   = clip((filtered_adc - safe) / (blocked - safe), 0, 1)
co_cost = co_weight * ratio ** co_power          # co_weight=8.0, co_power=2.0
final_cost_map[cell] += co_cost
if filtered_adc >= blocked:  cell is added to blocked_mask, final_cost = inf
```

- `filtered_adc <= safe` → `ratio = 0` → gas cost `0` (free).
- `safe < filtered_adc < blocked` → finite extra cost; the planner may detour if
  a cheaper route exists, but the cell stays traversable.
- `filtered_adc >= blocked` → `inf` / blocked → A\* and the waypoint graph refuse
  it, and Stage 6 event-replanning invalidates a current path that crosses it.

`co_weight` / `co_power` decide *how hard* the planner avoids finite gas cost.
Stage 7 does **not** tune them — calibrating the thresholds and tuning the
avoidance strength are separate problems. `gas_update_radius_m` stays `0.0`
(current robot cell only); no spatial spread model in Stage 7.

Validation already lives in `HazardBeliefConfig.__post_init__`
(`gas_blocked threshold must exceed safe threshold`), so `gas_blocked_adc <=
gas_safe_adc` (e.g. `safe=2000 blocked=1500`, or `safe == blocked`) makes the
hazard node fail to start with a clear error instead of dividing by zero.

---

## Raspberry Pi procedure

### 1. Connect the MQ-135 and start the stack

```bash
cd ~/Robot_project/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# gas sensor on, thresholds left at the inert placeholders for now
./run_mode5.sh use_gas_sensor:=true
#   (or, ESP32 bridge only, without the full Mode 5 stack:)
# ros2 run inno_drive_bridge cmdvel_to_esp32_serial --ros-args -p serial_port:=/dev/ttyUSB0
```

### 2. Confirm the sensor is publishing

```bash
ros2 topic hz /mq135/filtered_adc          # steady rate
ros2 topic echo /mq135/raw_adc             # data: <int>
ros2 topic echo /mq135/filtered_adc        # data: <float>
```

### 3. Let the sensor warm up and settle

MQ-135 heater output drifts for a while after power-on. Watch
`ros2 topic echo /mq135/filtered_adc` until the value stops trending. Only start
recording after it is stable.

### 4. Record each environment to its own CSV

Record the value column straight to a file, one number per line. Do this once per
environment you want to characterise — for example a "normal indoor air" baseline
and any known-safe elevated-reading conditions you have prepared. **This document
does not tell you how to create a gas exposure; use whatever safe test setup you
already have.**

```bash
# environment A – baseline indoor air (run ~60 s, then Ctrl-C)
ros2 topic echo /mq135/filtered_adc --field data --csv > ~/mq135_A_baseline.csv

# environment B / C – your other prepared conditions
ros2 topic echo /mq135/filtered_adc --field data --csv > ~/mq135_B.csv
ros2 topic echo /mq135/filtered_adc --field data --csv > ~/mq135_C.csv
```

Prefer a rosbag if you want the raw stream kept:

```bash
ros2 bag record -o ~/mq135_bag /mq135/raw_adc /mq135/filtered_adc
# later, turn the bag into a CSV for analysis:
ros2 bag play ~/mq135_bag &
ros2 topic echo /mq135/filtered_adc --field data --csv > ~/mq135_from_bag.csv
```

### 5. Look at the statistics

Offline, on any machine with Python + numpy (no ROS needed):

```bash
cd ~/Robot_project/fire_robot_rpi
python3 tools/analyze_mq135_calibration.py ~/mq135_A_baseline.csv
python3 tools/analyze_mq135_calibration.py ~/mq135_A_baseline.csv ~/mq135_B.csv ~/mq135_C.csv
python3 tools/analyze_mq135_calibration.py --json ~/mq135_A_baseline.csv     # machine-readable
```

You get `count / mean / median / min / max / std` and percentiles per file (and a
combined block for multiple files). **The tool does not choose thresholds.**

### 6. Choose the thresholds (operator decision)

Using the printed statistics *and* your judgement about your sensor and test
conditions:

- `gas_safe_adc` — a value comfortably above the noise ceiling of your clean-air
  baseline (e.g. above its max / p95), so ordinary air never adds cost.
- `gas_blocked_adc` — a value only reached under a condition you consider
  genuinely unsafe, and strictly greater than `gas_safe_adc`.

Statistics like `mean + k·std` are only a starting point to eyeball — the final
numbers are yours, from repeated measurements, not from this tool.

### 7. Apply the thresholds

```bash
./run_mode5.sh \
  use_gas_sensor:=true \
  gas_input_mode:=adc \
  gas_safe_adc:=<your S> \
  gas_blocked_adc:=<your B>
```

The args forward `run_mode5.sh` → `evacuation_demo.launch.py` →
`field_waypoint_test.launch.py` → `autonav_demo.launch.py` →
`hazard_belief_node`. Verify they landed:

```bash
ros2 param get /hazard_belief_node gas_input_mode      # adc
ros2 param get /hazard_belief_node gas_safe_adc        # your S
ros2 param get /hazard_belief_node gas_blocked_adc     # your B
```

### 8. Confirm the cost pipeline end to end

```bash
ros2 topic echo /mq135/raw_adc --once            # raw ESP32 ADC
ros2 topic echo /mq135/filtered_adc --once       # filtered ESP32 ADC
ros2 topic echo /hazard/co_grid --once           # stored gas belief value per cell (nan = unobserved)
ros2 topic echo /hazard/gas_cost_grid --once     # 0..99 gas ratio overlay, 100 = blocked (planner encoding)
ros2 topic echo /planning_grid_hazard --once     # waypoint planner grid = /planning_grid + gas overlay
ros2 topic echo /hazard/final_cost --once        # exact fused traversal cost for A* (inf at blocked)
ros2 topic echo /planned_path --once             # PathSelector's final path
```

### 9. Confirm Stage 6 automatic replanning

With the robot driving `/planned_path`, expose the sensor (in your safe setup) so
a cell on the path crosses `gas_blocked_adc`:

```bash
ros2 topic echo /replanning/status               # last_replan_reason=path_cell_blocked, state=REPLAN_REQUESTED
ros2 topic echo /replanning/hold                 # data: true while it re-plans (follower stops)
ros2 topic echo /waypoint_path --once            # detour candidate
ros2 topic echo /astar_path --once               # fallback candidate (only if waypoint fails)
ros2 topic echo /planned_path --once             # updated final path
```

Exposing the sensor *away from* the path must leave `/replanning/status` at
`PATH_VALID` with no hold.

## RViz

No `.rviz` changes are needed. In RViz add these by topic:

- `Map` ← `/planning_grid_hazard` (Color Scheme: costmap) — waypoint planner input incl. gas
- `Map` ← `/hazard/final_cost_grid_vis` (Color Scheme: costmap) — normalised view of the fused cost (visual only, never a planner input)
- `Map` ← `/hazard/gas_cost_grid` — gas overlay alone
- `Path` ← `/waypoint_path`, `/astar_path`, `/planned_path`

## Still needs real measurement

- `gas_safe_adc`, `gas_blocked_adc` for the actual installed sensor (steps 4–7).
- Whether `co_weight` / `co_power` need tuning once thresholds are real (separate follow-up).
- Sensor warm-up time and any periodic re-zeroing for your unit.
