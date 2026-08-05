# INNO Jazzy SLAM

```bash
cd ~/inno_jazzy_ws
./scripts/run_slam_keyboard.sh
```

주행하면서 지도 생성 후 `s`, 종료는 `q`를 누릅니다.

지도 저장 위치: `~/inno_jazzy_ws/maps/`


## encoder&keyboard 실행 명령어
# 1. ROS 패키지 다시 빌드

```
cd ~/inno_jazzy_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select inno_drive_bridge

source install/setup.bash
```

# 2. launch 실행

```
ros2 launch inno_drive_bridge drive_keyboard_demo.launch.py \
  serial_port:=/dev/ttyUSB0
```

정상적으로 영점 명령을 보냈다면 다음 로그가 표시

```
Encoder angles and distances reset at launch start
```

초기 엔코더 출력예시

```
각도: 왼쪽 0.00°, 오른쪽 0.00° | 거리: 왼쪽 0.00 mm, 오른쪽 0.00 mm
```

센서의 미세한 지터 때문에 정지 상태에서도 `0.02°` 정도가 나오거나, 아주 작은 역방향 변화가 `359.98°`로 표시될 수 있음. `0°` 경계에서 발생하는 정상적인 현상
