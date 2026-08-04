Waypoint driving with UART + ESP32

Hardware values used now

- Wheel diameter: 0.08 m (80 mm)
- Encoder: AS5048A, 14-bit, 16384 counts/rev
- Encoder gear ratio: 1.0
- Track width: 0.18 m (adjust if your measured wheel-center distance differs)
- TB6600 microstep: 1/8 (default in this firmware)

How to run

1. Upload firmware to ESP32
   - File: firmware/esp32_tb6600_bridge/esp32_tb6600_bridge.ino
2. Connect ESP32 over UART to the Raspberry Pi / PC
   - Example port: /dev/ttyUSB0
3. Build the ROS package

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select inno_semantic_nav
source install/setup.bash
```

4. Run the waypoint script

```bash
ros2 run inno_semantic_nav go_dense_waypoints P1 P3 --spacing 0.5 --serial /dev/ttyUSB0 --speed 0.2
```

Notes

- `P1` and `P3` can be semantic names from `maps/semantic_points.yaml` or raw coordinates like `1.0,2.0,0.0`.
- The current script uses a simple rotate-then-drive open-loop pattern. It is suitable for initial testing.
- If the robot turns too much or too little, adjust `TRACK_WIDTH_M` and the speed values.
