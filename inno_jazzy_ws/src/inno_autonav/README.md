# inno_autonav

`inno_autonav`는 ROS 2 Jazzy에서 planning용 occupancy map, LiDAR 동적장애물,
8-connected A*, 스키드 조향 path follower를 연결해 `/cmd_vel`을 생성한다. ESP32는
지도나 경로를 처리하지 않으며 기존 `inno_drive_bridge/cmdvel_to_esp32_serial`이
`/cmd_vel`을 좌우 step/s 명령으로 변환한다.

이 저장소의 실제 ROS workspace는 `~/fire_robot_rpi/inno_jazzy_ws`다. 아래 명령은
이 경로를 기준으로 한다.

## 데이터 흐름

```text
inno_map_nav.yaml ─▶ /planning_grid_static ─┐
                                             ├─▶ /planning_grid ─▶ A* ─▶ /planned_path
/scan + map TF ─▶ /dynamic_obstacle_grid ───┘                         │
                                                                        ▼
/mission_text ─▶ semantic goal ─▶ /goal_pose                  skid_path_follower
                                                                        │
                                                                        ▼
                                                                    /cmd_vel
                                                                        │
                                                cmdvel_to_esp32_serial ─┘
                                                                        ▼
                                                         ESP32 좌/우 step speed
```

Localization은 이 패키지 밖에서 실행한다. 필수 TF는 다음과 같다.

```text
map ─▶ odom ─▶ base_link ─▶ laser
```

- `map → odom`: AMCL
- `odom → base_link`: 실제 encoder wheel odom/EKF 또는 선택한 odometry 한 개
- `base_link → laser`: static TF, 현재 z=0.35m, yaw=0

## 노드

- `planning_grid_publisher`: `inno_map_nav.yaml`을 `/planning_grid_static`으로 발행
- `dynamic_obstacle_layer`: static-free 공간의 새 scan endpoint를 확인·팽창하여 persistent obstacle로 저장
- `astar_replanner`: static/dynamic grid를 합치고 현재 TF pose에서 goal까지 A* 수행
- `skid_path_follower`: 큰 heading error에서는 제자리 회전, 작으면 저속 전진 보정
- `mission_commander`: 문자열 mission을 semantic `/goal_pose`로 변환
- `go_to`: `/mission_text` CLI publisher

## semantic 이름

`config/semantic_points.yaml`에는 실제 지도에서 측정한 `exit1`, `exit2`, `exit3`,
`init` 좌표가 있다. 이름은 대소문자를 구분하지 않으며 `E1`, `e1`, `EXIT1`은
모두 `exit1`로 정규화된다.

지원 mission:

```text
go exit2
go exit1 exit2
exit1에서 exit2로가
exit1 to exit2
```

source label은 요청 기록에만 사용한다. 실제 시작점은 항상 현재 `map → base_link`
TF가 우선이다. TF가 없을 때 source 좌표를 쓰는 기능은 디버그 전용이며 기본값은
`use_source_if_no_tf: false`다.

## 설치

```bash
sudo apt-get update
sudo apt-get install -y python3-yaml python3-numpy python3-pil python3-serial
sudo apt-get install -y ros-jazzy-nav2-map-server ros-jazzy-nav2-amcl
```

현재 장치에는 `nav2_map_server`는 있지만 `nav2_amcl`은 별도 설치가 필요할 수 있다.

## 빌드

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

모든 터미널에서 DDS 설정도 동일하게 적용한다.

```bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
```

## 실행 순서

### 터미널 1: LiDAR와 static TF

먼저 `ls -l /dev/serial/by-id/`로 LiDAR 포트를 확인한다. 예를 들어 LiDAR가
`/dev/ttyUSB0`인 경우:

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
ros2 launch inno_bringup lidar_with_tf.launch.py serial_port:=/dev/ttyUSB0
```

### 터미널 2: localization용 원본 map_server

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/gosunwoo/fire_robot_rpi/maps/inno_map_raw.yaml
```

다른 터미널에서 lifecycle을 전환한다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

AMCL/localization은 원본 `inno_map_raw.yaml` 기반 `/map`을 사용한다. planning map은
이 패키지가 파일에서 직접 읽어 별도 `/planning_grid_static`으로 발행한다.

### 터미널 3: wheel odom/AMCL

사용자 AMCL YAML이 준비된 경우:

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
ros2 run nav2_amcl amcl --ros-args --params-file <사용자_AMCL_yaml_절대경로>
```

AMCL도 lifecycle node이므로 구성에 따라 다음 전환이 필요하다.

```bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

RViz `2D Pose Estimate`로 초기 pose를 지정한다. 실행 전 반드시 확인한다.

```bash
ros2 run tf2_ros tf2_echo map base_link
```

`map → base_link`가 안정적으로 출력되지 않으면 자율주행 goal을 보내지 않는다.

### 터미널 4: 자율주행 dry-run

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
ros2 launch inno_autonav autonav_demo.launch.py use_serial:=false
```

이 상태에서는 `/cmd_vel`까지만 생성하며 ESP32 serial을 열지 않는다.

### RViz 확인

```text
Fixed Frame: map
Map: /map
Map: /planning_grid_static
Map: /dynamic_obstacle_grid
Map: /planning_grid
LaserScan: /scan, Reliability=Best Effort
Path: /planned_path
MarkerArray: /dynamic_obstacle_markers
TF 표시
```

planning 관련 Map display의 Alpha를 낮추거나 하나씩 켜서 비교한다.

### 목표 발행

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0

ros2 run inno_autonav go_to exit2
ros2 run inno_autonav go_to exit1 exit2
```

또는:

```bash
ros2 topic pub --once /mission_text std_msgs/msg/String "{data: 'exit1에서 exit2로가'}"
```

상태 확인:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /planner_state
ros2 topic echo /follower_state
ros2 topic echo /mission_state
ros2 topic echo /planned_path
```

## 모터 연결

먼저 바퀴를 공중에 띄우고 ESP32 포트를 확인한다. LiDAR와 ESP32의 tty 번호가
바뀔 수 있으므로 CP2102N serial ID를 기준으로 구분한다.

```bash
ros2 launch inno_autonav autonav_demo.launch.py use_serial:=true serial_port:=/dev/ttyUSB1
```

이 launch는 기존 executable을 그대로 사용한다.

```text
inno_drive_bridge/cmdvel_to_esp32_serial
/cmd_vel → M,<seq>,<left_sps>,<right_sps>
```

wheel tick TF까지 이 launch에서 시험하려면 다음을 추가할 수 있다.

```bash
ros2 launch inno_autonav autonav_demo.launch.py use_serial:=true serial_port:=/dev/ttyUSB1 use_wheel_odom_tf:=true
```

이 옵션은 `step_count_to_odom`을 `odom → base_link`, `publish_tf=true`로 실행한다.
실제 encoder가 아니라 ESP32 step count만 쓰는 상태에서는 open-loop 추정이므로
미끄러짐이 큰 4륜 skid 로봇의 정확한 localization 용도로 신뢰하면 안 된다.

## 동적장애물 데모

1. `exit1 → exit2` goal을 보낸다.
2. 로봇이 기존 `/planned_path`로 이동한다.
3. 원래 free 공간의 경로 중간에 박스나 설비 모형을 놓는다.
4. LiDAR endpoint가 static-free cell에서 3회 확인된다.
5. `/dynamic_obstacle_grid`와 `/dynamic_obstacle_markers`에 표시된다.
6. `/planning_grid`가 갱신되고 A*가 재계획한다.
7. follower가 새 경로를 따라 `/cmd_vel`을 변경한다.
8. serial 사용 시 ESP32가 좌우 모터를 구동한다.

기본값은 붕괴 설비 시나리오에 맞춘 persistent obstacle이다. 센서에서 사라져도
자동 삭제되지 않는다. 명시적으로 지우려면:

```bash
ros2 service call /clear_dynamic_obstacles std_srvs/srv/Trigger "{}"
```

`persistent_obstacles: false`로 바꾸면 `obstacle_timeout_sec` 이후 제거된다.

## 성공 기준

- `/dynamic_obstacle_markers`에 새 장애물이 나타난다.
- `/planning_grid`에서 해당 영역이 occupied 100으로 표시된다.
- `/planner_state`가 재계획 상태를 거쳐 `PATH_READY`가 된다.
- `/planned_path`가 장애물을 피해 바뀐다.
- `/cmd_vel`이 회전/전진 명령으로 바뀐다.
- `use_serial:=true`에서 ESP32 ACK가 유지되고 실제 skid 구동이 된다.

## 안전 및 충돌 방지

- `keyboard_cmdvel_demo`와 `skid_path_follower`를 동시에 실행하지 않는다. 둘 다
  `/cmd_vel`을 발행하면 수동·자율 명령이 충돌한다.
- `drive_keyboard_demo.launch.py` 전체와 autonav `use_serial:=true`를 동시에 실행하지
  않는다. serial port 중복 open과 `/cmd_vel` 충돌이 발생한다.
- 실제 주행 전에 `use_serial:=false`로 경로와 `/cmd_vel`을 검증한다.
- 모터 연결 첫 시험은 바퀴를 공중에 띄우고 ESP32 ACK와 STOP을 확인한다.
- emergency stop 거리 `0.28m`를 너무 작게 설정하지 않는다. 실제 제동거리와 LiDAR
  높이 0.35m에서 보이지 않는 낮은 장애물을 고려한다.
- `odom → base_link` TF 발행자는 딱 하나여야 한다. wheel odom, RF2O, EKF가 동시에
  같은 TF를 발행하면 localization이 불안정해진다.
- `map → odom`은 AMCL 하나만 발행한다.
- `base_link → laser`는 static TF 또는 URDF 중 하나만 발행한다.
- source label은 현재 위치를 대체하지 않는다. 실제 TF pose가 항상 우선이다.
- 이 custom A* follower는 저속 데모용이다. 사람 주변 실제 운용에는 별도의 safety
  PLC/비상정지, 충돌검증, footprint 검증과 충분한 현장 시험이 필요하다.

## 개별 실행 및 테스트

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run inno_autonav go_to --help
ros2 run inno_autonav planning_grid_publisher --ros-args --params-file src/inno_autonav/config/autonav_params.yaml
ros2 run inno_autonav dynamic_obstacle_layer --ros-args --params-file src/inno_autonav/config/autonav_params.yaml
ros2 run inno_autonav astar_replanner --ros-args --params-file src/inno_autonav/config/autonav_params.yaml
ros2 run inno_autonav skid_path_follower --ros-args --params-file src/inno_autonav/config/autonav_params.yaml
ros2 launch inno_autonav autonav_demo.launch.py use_serial:=false
```

테스트:

```bash
colcon test --packages-select inno_autonav
colcon test-result --test-result-base build/inno_autonav --verbose
```
