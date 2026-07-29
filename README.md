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

## Waypoint 용 최신 SLAM 지도

가장 최근에 생성한 SLAM 지도는 다음 두 파일이다.

- `inno_jazzy_ws/maps/fire_demo_20260729_182054.yaml`: 지도 해상도, 원점, 이미지 경로 등의 메타데이터
- `inno_jazzy_ws/maps/fire_demo_20260729_182054.pgm`: SLAM으로 생성한 occupancy grid 지도 이미지

이 두 파일을 한 쌍으로 유지하며, 향후 로컬 노트북에서 AMCL 위치 추정과 A1/A2 웨이포인트를 지정할 때 사용할 기준 지도다.

## Git

ROS 2 빌드 산출물(`build/`, `install/`, `log/`)과 rosbag(`bags/`, `*.db3`, `*.mcap`, `*.sqlite3`)은 루트 `.gitignore`에서 제외한다.

```bash
cd ~/fire_robot_rpi
git add .gitignore README.md inno_jazzy_ws/src/inno_bringup
git commit -m "Add Raspberry Pi lidar bringup configuration"
```
