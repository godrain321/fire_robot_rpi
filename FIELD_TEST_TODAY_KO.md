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
  inno_camera_tools inno_drive_bridge inno_robot_bringup inno_autonav
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
  drive_speed:=0.06 turn_speed:=0.35
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

## 3. 모드 2: 저장된 waypoint 이름을 골라 한 점씩 주행

`maps/waypoint_queue_latest.yaml`에 `w1`, `w2`처럼 이름이 지정된 waypoint가 있어야
한다. 키보드 터미널에서 `2`를 누른 뒤, 프롬프트에 목적지를 두 개 이상 입력하고
Enter를 누른다.

```text
MODE 2 waypoints (example w1,w5,w6) > w1,w5,w6
```

Enter 즉시 현재 로봇 위치에서 `w1`로 출발한다. `w1`에 도착하여 다음 상태가
표시된 뒤에만 Space를 누른다.

```text
[MODE 2] MODE2_REACHED:w1:SPACE_FOR:w5
```

그러면 `w5`로 출발하고, `w5` 도착 후 Space를 다시 누르면 `w6`로 출발한다.
즉 입력 순서를 그대로 따르므로 `w1,w5`를 입력한 경우 Space 다음 목적지는 `w5`다.
주행 중 Space는 `MODE2_BUSY`로 거절되어 목적지가 건너뛰어지지 않는다. 존재하지
않는 이름이나 waypoint 한 개뿐인 입력도 출발 전에 거절된다.

모드 2는 A* 및 LiDAR 동적 장애물 회피 경로를 사용한다. 각 점에 도착하면 기존
경로를 취소하고 정지한 상태로 Space를 기다린다. `c`, `s`, `1`을 누르면 진행 중인
자율주행 경로를 취소한다. `s`와 `1`은 모드 1로 전환한다.

## 4. 모드 3: LiDAR 장애물에 접근해 mmWave 사람 판별

LiDAR가 새 물체를 큰 빨간 점으로 표시한 상태에서 `3`을 누른 뒤 Space를 누른다.
숫자 입력만으로는 출발하지 않는다. 로봇은 가장 가까운
동적장애물을 선택해 대상과 1.5m 떨어진 지점까지 A*로 이동하고 대상을 정면으로
바라본다. 도착 후 2초간 정지하고 5초 동안 C4001 micro-motion presence를 확인한다.

- presence 확인: 통합 로그 `사람 감지!`, 해당 점이 빨간색에서 파란색으로 변경
- ONLINE 상태에서 presence 없음: 통합 로그 `동적장애물!`, 빨간색 유지
- 센서 OFFLINE 또는 샘플 부족: `MODE3_SENSOR_UNAVAILABLE`, 빨간색 유지
- `s`, `c`, `1`: 검사 취소 및 모드 1 전환

```bash
ros2 topic echo /mode3_status
ros2 topic echo /mode3_classification
ros2 topic echo /dynamic_obstacle_candidates
ros2 topic echo /mmwave/sensor_state
```

C4001의 stationary-person micro-motion presence를 사용하는 기능이며 의료용 생체신호
판독은 아니다. 첫 현장 시험은 사람 대신 안전한 반사체와 보조 인원을 두고 바퀴를
띄운 상태부터 확인한다.

## 5. 모드 4: Camera Module 3 + LiDAR 요구조자 판별

현재 임시 시험용 사람 모델은 `~/fire_robot_rpi/models/yolov8n_best.onnx`에 들어
있다. 메타데이터상 class 0 하나가 `person`이다. ROS를 실행하는 Python 환경에서
다음 ONNX Runtime import가 성공해야 한다.

```bash
python3 -c "import onnxruntime; print('YOLO ONNX runtime OK')"
```

통합 launch에 카메라와 모델을 활성화한다.

```bash
ros2 launch inno_robot_bringup field_waypoint_test.launch.py \
  esp32_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1 \
  use_camera_mode4:=true \
  yolo_model_path:="$FIRE_ROBOT_RPI_ROOT/models/yolov8n_best.onnx" \
  map_yaml:="$FIRE_ROBOT_RPI_ROOT/maps/inno_map_raw.yaml" \
  planning_map_yaml:="$FIRE_ROBOT_RPI_ROOT/maps/inno_map_nav.yaml" \
  waypoint_file:="$FIRE_ROBOT_RPI_ROOT/maps/waypoint_queue_latest.yaml" \
  drive_speed:=0.06 turn_speed:=0.35
```

RViz에 빨간 점이 생기면 통합 터미널에서 `4`, Space를 차례로 누른다. 로봇은 가장
가까운 빨간 점 1.5m 앞에서 정면 정렬하고 2초 정지한 뒤 5초 동안 YOLO 결과를
모은다. 카메라 추론은 이 관찰 구간에만 실행된다.

사람과 일반 장애물이 한 화면에 함께 있고 빨간 점 두 개가 인접한 경우에도 YOLO
바운딩박스 중심의 좌우 방향과 LiDAR 점의 방위각을 대응한다. 한 바운딩박스는 LiDAR
점 하나에만 연결되므로 실제 사람 방향의 점만 파란색으로 바뀐다. 파란 점은 계속
유지되고 나머지 점은 빨간색으로 남는다.

```bash
ros2 topic echo /mode4_status
ros2 topic echo /mode4_classification
ros2 topic echo /camera/person_detector_status
ros2 topic echo /camera/person_detections
ros2 topic hz /camera/image_raw
```

`MODEL_NOT_FOUND`, `ULTRALYTICS_NOT_INSTALLED`, 카메라 프레임 부족 상태에서는 사람
아님으로 결정하지 않고 빨간 점을 유지한다. 화각 측정·내부 보정 뒤
`autonav_params.yaml`의 `fallback_horizontal_fov_deg`, `camera_yaw_offset_deg`,
`maximum_bearing_error_deg`를 현장값에 맞춘다.

속도는 펌웨어를 다시 굽지 않고 launch 명령의 `drive_speed`, `turn_speed`로 조절한다.
먼저 바퀴를 띄운 상태에서 낮은 값으로 확인한 다음 현장 바닥에 맞춰 조금씩 올린다.

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
