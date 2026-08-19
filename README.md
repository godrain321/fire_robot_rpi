# Fire Robot RPi

Raspberry Pi 5와 ROS 2 Jazzy를 사용하는 산업현장 화재 대피·요구조자 탐색 로봇
소프트웨어다. RPLIDAR C1 지도 위치추정과 동적장애물 회피, ESP32 모터 제어,
C4001 mmWave 사람 판별, Camera Module 3 YOLO 요구조자 판별을 하나의 통합 launch로
실행한다.

## 현재 주행 모드

| 모드 | 입력 | 기능 |
|---|---|---|
| 1 | `1` | `w/x/a/d/s` 키보드 수동주행 |
| 2 | `2` | 이름으로 고른 waypoint를 순서대로 주행 |
| 3 | `3` → `Space` | 가장 가까운 LiDAR 동적장애물의 1.5m 앞에서 mmWave 사람 판별 |
| 4 | `4` → `Space` | 같은 위치에서 Camera Module 3 YOLO와 LiDAR를 결합해 요구조자 판별 |

RViz의 큰 빨간 점은 아직 분류하지 않은 동적장애물이고, 큰 파란 점은 모드 3 또는
4에서 사람·요구조자로 확인한 위치다. 파란 점은 실행 중 timeout 없이 유지되며
`/clear_dynamic_obstacles` 서비스 호출 또는 노드 재시작 시 초기화된다.

## 하드웨어와 기본 환경

- Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy
- Raspberry Pi Camera Module 3 Wide (IMX708)
- RPLIDAR C1
- C4001 mmWave presence sensor
- ESP32 + TB6600 4륜 skid-steer 구동부
- 선택 장치: MLX90640 열화상 카메라

현재 개발 PC처럼 센서가 연결되지 않은 환경에서 카메라·LiDAR·mmWave 미검출이
나오는 것은 정상이다. 실제 포트와 TF, 모델 정확도, 제동거리는 Raspberry Pi 5에서
별도로 검증해야 한다.

## 저장소 구성

| 경로 | 내용 |
|---|---|
| `inno_jazzy_ws/src/inno_robot_bringup` | 통합 launch, AMCL, RViz 설정 |
| `inno_jazzy_ws/src/inno_autonav` | A*, 동적장애물, 모드 3·4 검사, path follower |
| `inno_jazzy_ws/src/inno_drive_bridge` | 모드 선택, 키보드, ESP32 직렬 모터 bridge |
| `inno_jazzy_ws/src/inno_mmwave` | C4001 수신·필터링·상태 출력 |
| `inno_jazzy_ws/src/inno_camera_tools` | Camera Module 3 ROS 실행·YOLO 추론 |
| `maps` | 주행 지도, 약 1m 간격 waypoint YAML, no-go 설정 |
| `models` | 모드 4 YOLO weight 배치 안내 |
| `firmware/esp32_tb6600_bridge` | ESP32 모터 bridge 펌웨어 |
| `record_robot_bag.sh` | 센서·주행·분류 토픽 통합 rosbag 기록 |

## 1. 최초 설치와 빌드

Ubuntu 24.04에서 ROS 2 Jazzy와 기본 의존성을 처음 설치할 때 사용한다.

```bash
cd ~/fire_robot_rpi
sudo bash ./install_ubuntu2404_dependencies.sh
```

Camera Module 3를 사용할 Raspberry Pi에서는 호환 카메라 런타임도 한 번 빌드한다.

```bash
cd ~/fire_robot_rpi
./build_rpi_camera_runtime.sh
```

ROS workspace를 빌드한다.

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to inno_robot_bringup
source install/setup.bash
```

새 터미널을 열 때마다 `/opt/ros/jazzy`와 workspace의 `install/setup.bash`를 다시
source해야 한다.

## 2. 통합 현장 실행

먼저 장치별 고정 포트를 확인한다.

```bash
ls -l /dev/serial/by-id/
```

처음에는 바퀴를 바닥에서 띄우고 저속으로 실행한다. MLX90640을 사용하지 않으면
`start_thermal_viewer:=false`를 유지한다.

```bash
export FIRE_ROBOT_RPI_ROOT="$HOME/fire_robot_rpi"

cd "$FIRE_ROBOT_RPI_ROOT/inno_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch inno_robot_bringup field_waypoint_test.launch.py \
  esp32_port:=/dev/ttyUSB0 \
  lidar_port:=/dev/ttyUSB1 \
  mmwave_port:=/dev/ttyAMA0 \
  map_yaml:="$FIRE_ROBOT_RPI_ROOT/maps/inno_map_raw.yaml" \
  planning_map_yaml:="$FIRE_ROBOT_RPI_ROOT/maps/inno_map_nav.yaml" \
  waypoint_file:="$FIRE_ROBOT_RPI_ROOT/maps/waypoint_queue_latest.yaml" \
  drive_speed:=0.06 \
  turn_speed:=0.35 \
  start_thermal_viewer:=false
```

RViz가 열리면 다음 순서로 준비한다.

1. Fixed Frame이 `map`인지 확인한다.
2. **2D Pose Estimate**로 지도상의 실제 로봇 위치와 방향을 지정한다.
3. `/scan`, `/lidar_path`, `/planned_path`와 빨간 동적장애물 marker를 확인한다.
4. 통합 launch를 실행한 터미널에 포커스를 두고 모드 키를 입력한다.

기본 상태 확인:

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /drive_mode_status
ros2 topic echo /follower_state
```

## 3. 모드별 사용법

### 모드 1: 키보드 수동주행

`1`을 누른 뒤 사용한다.

| 키 | 동작 |
|---|---|
| `w` | 전진 |
| `x` | 후진 |
| `a` | 제자리 좌회전 |
| `d` | 제자리 우회전 |
| `s` | 정지 |
| `q` | 정지 후 키보드 노드 종료 |

### 모드 2: 선택 waypoint 단계주행

`maps/waypoint_queue_latest.yaml`에 저장된 `w1`, `w2` 등의 이름을 사용한다. `2`를
누른 뒤 waypoint를 두 개 이상 입력하고 Enter를 누르면 첫 지점으로 출발한다.

```text
MODE 2 waypoints (example w1,w5,w6) > w1,w5,w6
```

`w1`에 도착해 `MODE2_REACHED:w1:SPACE_FOR:w5`가 표시되면 Space를 눌러 `w5`로
출발한다. 입력한 순서만 사용하며 주행 중 Space를 눌러도 다음 목적지를 건너뛰지
않는다. 모드 2의 A* 경로는 LiDAR 동적장애물을 피해 재계획된다.

- `c`: 현재 모드 2 mission 취소
- `s` 또는 `1`: 자율주행 취소, 정지, 모드 1 복귀
- 상태 토픽: `/waypoint_queue_status`

약 1m 간격 waypoint 번호와 지도 시각화는
[docs/full_map_waypoints_1m_numbered.png](docs/full_map_waypoints_1m_numbered.png),
[maps/waypoint_queue_latest.yaml](maps/waypoint_queue_latest.yaml)에서 확인할 수 있다.

### 모드 3: LiDAR + mmWave 사람 판별

LiDAR가 새 물체를 큰 빨간 점으로 표시하면 `3`을 누른 뒤 Space를 누른다. 숫자만
입력했을 때는 출발하지 않는다.

1. 현재 로봇과 가장 가까운 미분류 빨간 점을 선택한다.
2. A*로 대상의 1.5m 앞까지 이동하고 대상을 정면으로 바라본다.
3. 2초 정지 후 5초 동안 C4001의 새로운 presence·거리 샘플을 확인한다.
4. presence가 충분하면 `사람 감지!`를 출력하고 해당 점만 파란색으로 바꾼다.
5. 센서가 ONLINE이지만 presence가 없으면 `동적장애물!`을 출력하고 빨간색을
   유지한다.

센서가 OFFLINE이거나 샘플이 부족하면 사람 아님으로 오판하지 않고 판정을 보류한다.
C4001 결과는 미세 움직임 기반 presence이며 의료용 생체신호 측정이 아니다.

```bash
ros2 topic echo /mode3_status
ros2 topic echo /mode3_classification
ros2 topic echo /mmwave/sensor_state
ros2 topic echo /mmwave/filtered_presence
```

### 모드 4: Camera Module 3 YOLO + LiDAR 요구조자 판별

저장소에는 임시 사람 검출 시험용 ONNX 모델이 다음 기본 경로에 포함되어 있다.

```text
~/fire_robot_rpi/models/yolov8n_best.onnx
```

모델 메타데이터상 YOLOv8n detect, 입력 640×640, class `{0: person}` 구성이다. 이
프로젝트에서 직접 학습한 모델은 아니므로 실제 배포 전 출처·재배포 권한과 현장
정확도를 다시 확인하고 자체 데이터 모델로 교체한다.

ONNX 모델은 Ultralytics 없이 ONNX Runtime CPU backend로 실행한다. ROS 노드를
실행하는 Python 환경에서 다음 import가 성공해야 한다.

```bash
python3 -c "import onnxruntime; print('YOLO ONNX runtime OK')"
```

통합 launch에 카메라와 모델을 활성화한다. 나머지 포트·지도 인자는 앞의 통합 실행
명령과 동일하게 함께 넘길 수 있다.

```bash
ros2 launch inno_robot_bringup field_waypoint_test.launch.py \
  use_camera_mode4:=true \
  yolo_model_path:="$HOME/fire_robot_rpi/models/yolov8n_best.onnx" \
  start_thermal_viewer:=false
```

빨간 점이 보이면 `4`를 누른 뒤 Space를 누른다. 로봇은 가장 가까운 빨간 점의
1.5m 앞에서 정면 정렬한 다음 관찰 구간에만 YOLO 추론을 수행한다.

사람과 일반 장애물이 한 카메라 프레임에 동시에 있고 LiDAR 빨간 점 두 개가 가까운
경우, 사람 바운딩박스의 수평 위치와 각 LiDAR 후보의 로봇 기준 방위각을 비교한다.
한 바운딩박스를 한 LiDAR 후보에만 연결하므로 사람 방향의 점만 파란색으로 바뀌고
다른 점은 빨간색으로 남는다.

- 요구조자 확인: `요구조자 감지!`, `/mode4_classification = SURVIVOR:...`
- 사람 미검출: `요구조자 미감지!`, 빨간색 유지
- 모델·카메라·추론 프레임 없음: 판정 보류, 빨간색 유지
- `c`, `s`, `1`: 검사 취소 및 모드 1 복귀

두 물체가 LiDAR 단계에서 이미 하나의 cluster로 합쳐져 빨간 점이 하나만 만들어지면
카메라만으로 지도 점을 둘로 나눌 수 없다. 현장에서 이런 경우
`inno_jazzy_ws/src/inno_autonav/config/autonav_params.yaml`의
`cluster_radius_m`을 낮춰 다시 검증한다.
내부 파라미터가 없을 때 사용하는 `fallback_horizontal_fov_deg`와 카메라 장착
`camera_yaw_offset_deg`도 실제 화각·장착각에 맞춰야 한다.

```bash
ros2 topic echo /mode4_status
ros2 topic echo /mode4_classification
ros2 topic echo /camera/person_detector_status
ros2 topic echo /camera/person_detections
```

## 4. Camera Module 3 사진 촬영과 보정

ROS 없이 카메라 화면을 열고 학습용 사진을 저장하려면 다음만 실행한다.

```bash
sudo apt install -y python3-picamera2 python3-opencv
cd ~/fire_robot_rpi
./run_camera.sh
```

- `s`: `data/camera_capture/`에 JPEG 한 장 저장
- `q` 또는 `Esc`: 종료
- 해상도 변경 예: `./run_camera.sh --width 1920 --height 1080`

YOLO 데이터 수집 해상도와 실제 추론 해상도·카메라 sensor mode는 일치시키는 것이
좋다. 내부·외부 보정 작업은 다음 순서로 실행한다.

```bash
./capture_intrinsic_images.sh
./calibrate_camera.sh
./view_calibration_result.sh
./run_lidar_camera_calibration.sh
./run_lidar_camera_distance.sh
```

세부 조건은 [체커보드 Rational 캘리브레이션 가이드](docs/checkerboard_rational_calibration.md)와
[RPLIDAR C1–카메라 외부 보정 가이드](docs/lidar_camera_extrinsic_and_distance.md)를
참고한다.

## 5. 전체 주행 rosbag 기록

통합 launch 실행 후 새 터미널에서 실행한다.

```bash
cd ~/fire_robot_rpi
./record_robot_bag.sh
```

다음 항목을 한 번에 기록한다.

- 좌·우 모터 목표 STEP/s와 `/cmd_vel` 계열
- LiDAR `/scan`, TF, AMCL, 지도, 실제·계획 경로
- waypoint 명령·상태와 동적장애물 후보·marker
- mmWave raw·filtered 값과 모드 3 판정
- 카메라 영상·YOLO 바운딩박스와 모드 4 판정

시작 전 각 토픽을 `수신`, `대기`, `없음`으로 표시한다. 일부 센서가 없더라도 기록을
중단하지 않으며, 나중에 나타난 토픽도 저장한다. 결과는 `bags/fire_robot_날짜_시간/`,
점검 결과는 같은 이름의 `.topics.txt`에 생성된다. 카메라 원본 영상은 용량이 크므로
저장 공간을 먼저 확인한다.

토픽 상태만 검사하려면:

```bash
./record_robot_bag.sh --check-only
```

## 6. 테스트

센서가 없는 PC에서도 핵심 파싱, 경로, 모드 전환, mmWave 판정, 카메라–LiDAR
일대일 결합을 테스트할 수 있다.

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon test --packages-select \
  inno_camera_tools inno_drive_bridge inno_mmwave inno_autonav \
  inno_robot_bringup
colcon test-result --verbose
```

현재 변경 기준으로 관련 7개 패키지 빌드와 147개 테스트가 통과했다. 개발 PC에서는
ROS 2 Humble 호환 빌드·토픽 통합 테스트까지 수행했으며, Raspberry Pi 5의 Jazzy와
실제 센서·모터 검증은 현장에서 진행해야 한다.

## 안전 주의사항

- 첫 모터 시험은 반드시 바퀴를 바닥에서 띄우고 `drive_speed:=0.06` 이하에서 한다.
- `s` 또는 `1`로 자율주행을 취소할 수 있는지 먼저 확인한다.
- `/cmd_vel` publisher와 `odom → base_link`, `map → odom` TF publisher는 각각 하나만
  활성화한다.
- 2D LiDAR가 보지 못하는 낮은 물체와 실제 제동거리를 별도로 고려한다.
- 사람 주변 실제 운용에는 소프트웨어 외의 비상정지와 안전 감시자가 필요하다.
- YOLO·mmWave 판정은 구조 보조 정보이며 사람의 안전이나 신원을 단독으로 보장하지
  않는다.

현장 점검 순서는 [FIELD_TEST_TODAY_KO.md](FIELD_TEST_TODAY_KO.md)에 더 자세히
정리되어 있다.
