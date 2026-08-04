# 오늘 현장 테스트: 키보드 경로 → RViz goal → LiDAR waypoint 주행

## 어제 실패의 직접 원인

1. `go_stepwise_waypoints.py`와 `go_dense_waypoints.py`는 ESP32에 속도 명령을 한 번만
   보내고 긴 시간 기다렸다. ESP32 watchdog은 0.5초 뒤 모터를 정지시키므로 로봇은
   약 0.5초분만 이동했다. 이 직접 UART waypoint 방식은 오늘 사용하지 않는다.
2. 어제 펌웨어는 motor pulse 생성 loop에 SPI 엔코더 읽기와 많은 serial 출력을
   추가했고, 잘못된 단위의 `ENC_ABS` delta도 출력했다. 또한 펌웨어 내부 waypoint와
   ROS waypoint라는 두 제어기가 공존했다. 오늘 펌웨어는 모터 bridge 한 역할로
   단순화했다.
3. `/wheel_path`는 실제 엔코더가 아닌 발생 STEP을 적분한 결과다. 오늘 확인할 실제
   궤적은 LiDAR 위치추정 TF로 만든 `/lidar_path`이다.

## 오늘 사용하는 저장 지도

오늘 지도는 `$FIRE_ROBOT_RPI_ROOT/maps/inno_map_raw.yaml`과 그 YAML이 가리키는
`inno_map_raw.pgm`이다. PGM 해시를 확인한 결과 7월 29일
`fire_demo_20260729_182054.pgm`과 동일하다. 이 지도에는 slam_toolbox posegraph가 없으므로
저장 PGM/YAML을 직접 지원하는 `map_server + AMCL`로 전역 위치를 잡고, RF2O LiDAR
odometry로 `odom → base_link`를 만든다. 불안정한 wheel encoder는 사용하지 않는다.

## 0. 빌드와 ESP32 업로드

Arduino IDE에서 다음 파일을 ESP32에 업로드한다.

`firmware/esp32_tb6600_bridge/esp32_tb6600_bridge.ino`

TB6600 DIP가 실제로 1/8인지 확인한 후:

```bash
export FIRE_ROBOT_RPI_ROOT="${FIRE_ROBOT_RPI_ROOT:-$HOME/fire_robot_rpi}"
cd "$FIRE_ROBOT_RPI_ROOT/inno_jazzy_ws"
source /opt/ros/jazzy/setup.bash
sudo apt-get install ros-jazzy-nav2-amcl
colcon build --symlink-install --packages-select \
  inno_drive_bridge inno_robot_bringup inno_autonav
source install/setup.bash
```

## 1. 먼저 바퀴를 띄우고 통합 실행

ESP32와 LiDAR 포트는 `ls -l /dev/serial/by-id/`로 구분한다. 예:

```bash
ros2 launch inno_robot_bringup field_waypoint_test.launch.py \
  esp32_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1 \
  map_yaml:="$FIRE_ROBOT_RPI_ROOT/maps/inno_map_raw.yaml" \
  planning_map_yaml:="$FIRE_ROBOT_RPI_ROOT/maps/inno_map_nav.yaml" \
  waypoint_file:="$FIRE_ROBOT_RPI_ROOT/maps/waypoint_queue_latest.yaml" \
  manual_linear_speed:=0.08 manual_angular_speed:=0.35 \
  auto_linear_speed:=0.06 auto_angular_speed:=0.45
```

RViz에서 Fixed Frame=`map`, Path topic=`/lidar_path`, planned Path=`/planned_path`를
선택한다. 처음에는 RViz의 **2D Pose Estimate**로 지도상 실제 시작 위치와
방향을 지정한다. TF가 안정됐는지 확인한다.

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /scan
ros2 topic echo /drive_mode_status
```

## 2. 모드 1: 키보드와 경로 확인

launch를 실행한 터미널에서 `1`, 그 다음 `w/a/s/d/x`를 사용한다. `/lidar_path`가
실제 LiDAR 보정 위치를 map 위에 그려야 한다. `/wheel_path`는 비교용일 뿐이다.
급회전이 거칠면 먼저 `drive_params.yaml`의 `angular_speed`를 낮춘다. 펌웨어의
`MAX_STEP_ACCEL`을 낮추면 시작/반전 ramp가 더 부드러워진다.

## 3. 모드 2: path 위에 여러 waypoint를 찍고 순차 주행

키보드 터미널에서 `s`로 정지한 뒤 `2`를 누른다. 초록색 `/lidar_path`는 지도 위에
그대로 남는다. RViz **2D Goal Pose**로 그 path 위의 waypoint를 원하는 순서대로 여러
개 찍는다. 클릭한 waypoint들은 노란색 `/waypoint_queue` 선으로 누적 표시되며 아직
로봇은 출발하지 않는다. 각 클릭은 즉시 `maps/waypoint_queue_latest.yaml`에 원자적으로
저장되며 launch를 다시 실행해도 자동 복원된다. 잘못 찍었으면 `c`로 전체 queue를
지우고 다시 찍는다. `c`는 화면뿐 아니라 저장 파일의 queue도 비운다.

모든 waypoint를 찍은 다음 키보드 터미널에서 `g`를 누르면 첫 waypoint부터 차례대로
`/goal_pose` → A* `/planned_path` → LiDAR TF feedback follower → `/cmd_vel_auto`로
주행한다. 각 waypoint가 `GOAL_REACHED`가 되어야 다음 waypoint가 전달된다. follower는
10Hz로 명령을 계속 보내므로 0.5초 watchdog에 걸리지 않는다.

속도는 펌웨어를 다시 굽지 않고 launch 명령의 `manual_linear_speed`,
`manual_angular_speed`, `auto_linear_speed`, `auto_angular_speed`로 조절한다. 먼저 바퀴를
띄운 상태에서 낮은 값으로 확인한 다음 현장 바닥에 맞춰 조금씩 올린다.

실행 중에도 다른 터미널에서 즉시 변경할 수 있다.

```bash
ros2 param set /keyboard_cmdvel_demo linear_speed 0.06
ros2 param set /keyboard_cmdvel_demo angular_speed 0.30
ros2 param set /skid_path_follower max_linear_speed 0.04
ros2 param set /skid_path_follower max_angular_speed 0.35
```

모든 속도 값은 0보다 커야 하며, 너무 큰 값은 펌웨어의 1600 step/s 제한에서 잘리거나
탈조를 유발할 수 있으므로 작은 값부터 올린다.

```bash
ros2 topic echo /planner_state
ros2 topic echo /follower_state
ros2 topic hz /cmd_vel
ros2 topic echo /esp32/status
ros2 topic echo /waypoint_queue_status
```

`WAITING_FOR_TF`이면 절대 모터 배율 문제가 아니라 localization/TF 문제다.
`NO_PATH`이면 목표가 장애물/unknown/inflation 안인지 확인한다. 앞 0.28m 이내에
장애물이 있으면 `EMERGENCY_STOP`이 정상이다.

## 현장에서 얻어야 할 보정값

1. 줄자로 좌우 구동륜 중심 간격(`wheel_separation`)을 mm 단위로 측정한다.
2. 바닥에서 직진 2.0m를 3회 운전해 실제 거리와 `/lidar_path` 거리를 기록한다.
3. 제자리 360도 회전을 좌/우 각 3회 하고 실제 종료 각도와 TF yaw를 기록한다.
4. 직진 중 좌/우 쏠림, 탈조가 시작되는 최소/최대 step/s, 안전 정지거리를 기록한다.
5. AS5048A 고정 후 바퀴를 손으로 정확히 10회 돌려 raw count 방향과 누적 회전수를
   좌우 각각 기록한다. 그 전에는 항법 입력으로 사용하지 않는다.

가장 먼저 조정할 값은 `drive_params.yaml`의 `wheel_separation`, `wheel_radius`,
`linear_speed`, `angular_speed`와 펌웨어의 `MAX_STEP_ACCEL`이다. LiDAR-only 모드에서
wheel 값은 localization 완료 조건에는 쓰이지 않지만 모터 속도 변환에는 사용된다.
