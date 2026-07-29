# inno_bringup

Raspberry Pi 5의 RPLIDAR C1과 `base_link -> laser` 고정 TF를 실행하는 ROS 2 Jazzy 패키지다.

## 빌드

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select inno_bringup
source install/setup.bash
```

## LiDAR와 TF 실행

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0

ros2 launch inno_bringup lidar_with_tf.launch.py
```

LiDAR만 실행하려면 `ros2 launch inno_bringup lidar_only.launch.py`를 사용한다. 기본 USB 장치는 `/dev/ttyUSB0`이며, 필요하면 `serial_port:=/dev/ttyUSB1`처럼 지정한다.

다른 터미널에서 다음을 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0

ros2 topic hz /scan
ros2 topic echo /scan --once | head -40
ros2 run tf2_ros tf2_echo base_link laser
```

기본 장착값은 `config/lidar_mount.yaml`의 `(x, y, z)=(0, 0, 0.35) m`, RPY `(0, 0, 0)`이다. 실제 장착 후 `laser +x`가 차체 후방이면 `lidar_yaw`를 `3.1416`으로 변경하고 다시 빌드한다.

## rosbag 기록

```bash
mkdir -p ~/bags

ros2 bag record /scan /tf /tf_static \
  -o ~/bags/rpi_lidar_mount_test_$(date +%Y%m%d_%H%M%S)
```

동일한 토픽을 launch로 기록할 수도 있다.

```bash
ros2 launch inno_bringup record_lidar.launch.py
```

기록 종료는 `Ctrl+C`를 사용한다. `~/bags`는 프로젝트 바깥 경로이며, 프로젝트 안에 bag을 둘 경우에도 `.gitignore` 규칙으로 제외된다.

## 노트북 RViz 원격 확인

Ubuntu 22.04 / ROS 2 Humble 노트북에서:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0

rviz2
```

RViz 설정:

- Fixed Frame: `base_link`
- Add -> TF
- Add -> LaserScan
- Topic: `/scan`
- Reliability Policy: `Best Effort`

차체 전방 물체가 `base_link +x`에 나타나는지 확인하고, 아주 저속으로 이동시키며 scan 평면의 흔들림과 기울어짐을 점검한다.
