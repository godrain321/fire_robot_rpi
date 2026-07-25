# 빠른 시작

```bash
cd ~/inno_jazzy_ws
./scripts/run_slam_keyboard.sh
```

주행하면서 지도 생성 후 `s`, 종료는 `q`를 누릅니다. 지도는 `~/inno_jazzy_ws/maps/`에 저장됩니다.

# inno_robot_bringup

Raspberry Pi Camera Module 3, RPLIDAR C1, RF2O를 함께 실행합니다.

```bash
ros2 launch inno_robot_bringup sensors.launch.py
```

LiDAR 포트가 다르면:

```bash
ros2 launch inno_robot_bringup sensors.launch.py lidar_port:=/dev/ttyUSB1
```

현재 외부 캘리브레이션 값이 없으므로 다음 임시 장착 위치를 기본 TF로
사용합니다. `base_link`는 지면의 로봇 중심이며 +X는 전방, +Y는 좌측,
+Z는 위쪽입니다.

- `base_link -> laser_frame`: `(x=0.00, y=0.00, z=0.30 m)`
- `base_link -> camera_link`: `(x=0.10, y=0.00, z=0.20 m)`
- 라이다는 로봇 중앙에서 지면 위 30 cm
- 카메라는 라이다보다 전방 10 cm, 아래 10 cm
- 두 센서의 roll, pitch, yaw는 임시로 0 rad

RF2O는 정확한 `base_link -> laser_frame` 변환이 있어야 로봇 중심 기준의
오도메트리를 올바르게 계산합니다. 최종 마운트 치수를 알기 전의 결과는 시험용으로만
사용하세요.

마운트가 확정되면 미터와 라디안 단위의 launch 인자로 기본값을 덮어쓸 수
있습니다.

```bash
ros2 launch inno_robot_bringup sensors.launch.py \
  laser_x:=0.0 laser_z:=0.30 laser_yaw:=0.0 \
  camera_x:=0.10 camera_z:=0.20 camera_pitch:=0.0
```

카메라 위치/회전은 `camera_link` 기준으로 입력합니다. 영상 메시지용
`camera_optical_frame` 변환은 ROS 광학 좌표 규약에 맞게 자동 발행됩니다.
