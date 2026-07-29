# fire_robot_rpi

산업현장 화재 대피 유도 로봇의 Raspberry Pi 5 소프트웨어 저장소다.

현재 RPLIDAR C1 bringup 패키지는 `inno_jazzy_ws/src/inno_bringup`에 있다. 빌드 및 LiDAR/TF 실행, rosbag 기록, 노트북 RViz 원격 확인 방법은 [inno_bringup README](inno_jazzy_ws/src/inno_bringup/README.md)를 참고한다.

## 빠른 실행

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0

ros2 launch inno_bringup lidar_with_tf.launch.py
```

## Git

ROS 2 빌드 산출물(`build/`, `install/`, `log/`)과 rosbag(`bags/`, `*.db3`, `*.mcap`, `*.sqlite3`)은 루트 `.gitignore`에서 제외한다.

```bash
cd ~/fire_robot_rpi
git add .gitignore README.md inno_jazzy_ws/src/inno_bringup
git commit -m "Add Raspberry Pi lidar bringup configuration"
```
