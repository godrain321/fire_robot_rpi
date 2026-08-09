# Field session 2026-08-07 15:04:42 KST

- Localization map: `inno_map_raw.yaml` / `inno_map_raw.pgm`
- Planning map: `inno_map_nav.yaml` / `inno_map_nav.pgm`
- Current localized pose at capture time: `current_amcl_pose.yaml`
- Observed green RViz trail: `lidar_path.yaml`

The original RViz 2D Pose Estimate was published on volatile topic
`/initialpose`, so ROS did not retain that click for a later subscriber.
The first pose in `lidar_path.yaml` is the recoverable start pose of this run.

`lidar_path` publishes only after the robot moves at least 0.02 m. A waiting
subscriber was started after this session directory was created, so the file
is populated on the next qualifying movement.
