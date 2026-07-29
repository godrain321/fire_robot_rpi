# Fire Robot Camera Calibration

Ubuntu 24.04 + ROS 2 Jazzy에서 Raspberry Pi 카메라의 내부 파라미터와
2D LiDAR↔카메라 외부 변환을 구하기 위한 패키지다. 국민대 차량에서 사용한
어안 보정 및 LaserScan 오버레이 흐름을 `camera_ros`와 표준 ROS 토픽에 맞게
이식했다.

## 좌표계와 토픽 기본값

- 카메라 원본: `/camera/image_raw`
- 카메라 보정 정보: `/camera/camera_info`
- 보정 영상: `/camera/image_rect`
- 보정 영상 정보: `/camera/camera_info_rect`
- LiDAR: `/scan`, 프레임 `laser_frame`
- 카메라 optical 프레임: `camera_optical_frame` (`x` 오른쪽, `y` 아래, `z` 앞)

다른 드라이버를 사용하면 launch 인자로 토픽과 프레임을 바꾸면 된다.

## Jazzy 설치와 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ~/fire_robot_rpi/camera_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select fire_robot_camera_calibration
source install/setup.bash
```

`camera_ros`가 rosdep으로 설치되지 않는 환경에서는 먼저 다음을 설치한다.

```bash
sudo apt update
sudo apt install ros-jazzy-camera-ros ros-jazzy-camera-calibration \
  ros-jazzy-cv-bridge python3-opencv python3-numpy python3-yaml
```

새 Raspberry Pi 카메라 모듈이 Ubuntu 기본 `libcamera`에서 보이지 않으면
`camera_ros` 공식 안내에 따라 Raspberry Pi fork의 `libcamera`를 소스 빌드해야
한다. 먼저 `ros2 run camera_ros camera_node`에서 카메라가 잡히는지 확인한다.

GUI는 로컬 데스크톱 또는 X11 전달이 필요하다. SSH라면 `ssh -X`로 접속하고
`echo $DISPLAY`가 비어 있지 않은지 확인한다.

## 1. 내부 캘리브레이션 — ROS GUI 방식

결과 디렉터리를 만든 뒤 실제 체커보드의 **내부 코너 수**와 한 칸 길이(m)를
지정한다. 아래 기본값은 기존 국민대 보드(8x9, 0.07 m)에 맞춰져 있다.

```bash
mkdir -p ~/.ros/camera_info
ros2 launch fire_robot_camera_calibration intrinsic_calibration.launch.py \
  width:=1280 height:=720 board_size:=8x9 square_size:=0.07
```

어안/초광각 렌즈라면 다음 인자를 추가하고, 열린 GUI 위쪽의 `Camera type`
슬라이더를 반드시 `1: fisheye`로 바꾼다.

```bash
ros2 launch fire_robot_camera_calibration intrinsic_calibration.launch.py \
  width:=1280 height:=720 board_size:=8x9 square_size:=0.07 \
  use_fisheye_flags:=true
```

보드를 영상의 중앙뿐 아니라 네 가장자리, 가까이/멀리, 좌우·상하로 기울여
채운다. `CALIBRATE` 후 직선이 곧게 보이는지 확인하고 `COMMIT`하면 기본적으로
`~/.ros/camera_info/camera.yaml`에 기록된다. 캘리브레이션 때와 실제 운용 때
카메라 해상도와 sensor mode가 같아야 한다.

카메라는 이미 실행 중이라면 다음처럼 중복 실행을 막는다.

```bash
ros2 launch fire_robot_camera_calibration intrinsic_calibration.launch.py \
  start_camera:=false image_topic:=/my_camera/image_raw \
  camera_service_namespace:=/my_camera
```

## 1-대안. 기존 국민대식 어안 이미지 수집 + 오프라인 계산

```bash
ros2 launch fire_robot_camera_calibration intrinsic_capture.launch.py \
  width:=1280 height:=720 board_cols:=8 board_rows:=9 max_images:=80

ros2 run fire_robot_camera_calibration calibrate_fisheye -- \
  --image-dir ~/fire_robot_calibration/intrinsic_images \
  --output-yaml ~/.ros/camera_info/camera.yaml \
  --board-cols 8 --board-rows 9 --square-size 0.07
```

수집 창에서 `s`는 현재 프레임 강제 저장, `q`는 종료다. RMS 오차뿐 아니라
실제 보정 영상의 직선성과 가장자리 상태도 확인해야 한다.

## 2. LiDAR↔카메라 외부 캘리브레이션

먼저 내부 캘리브레이션 결과 파일이 존재해야 한다. LiDAR가 별도 워크스페이스에
있다면 그 워크스페이스도 source한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/camera_ws/install/setup.bash
source ~/fire_robot_rpi/inno_ws/install/setup.bash  # sllidar가 여기 있을 때

ros2 launch fire_robot_camera_calibration extrinsic_calibration.launch.py \
  camera_info_path:=$HOME/.ros/camera_info/camera.yaml \
  start_lidar:=true lidar_launch_file:=sllidar_a1_launch.py
```

LiDAR가 이미 `/scan`을 발행한다면 `start_lidar:=false`가 기본값이므로 그 인자를
생략한다. 실제 카메라와 LiDAR 사이 위치를 자로 재서 초기 YAML의
`camera_position_in_laser_frame_m`에 반영하면 튜닝이 빨라진다.

외부 캘리브레이션 키:

- `w/s`: 카메라 x(앞/뒤), `a/d`: y(왼쪽/오른쪽), `r/f`: z(위/아래)
- `i/k`: pitch, `j/l`: yaw, `u/o`: roll
- `p`: `~/fire_robot_calibration/lidar_camera_extrinsic.yaml`에 저장
- `q`: 종료

벽 모서리, 박스, 수직 판처럼 카메라 영상과 LiDAR 스캔에서 동시에 식별되는
구조물을 서로 다른 거리에서 맞춘다. 이 도구는 기존과 같은 **수동 2D
LaserScan 오버레이 방식**이다. 평면 스캔 하나만으로 6DoF를 자동·유일하게
구할 수 없으므로 실측 초기값과 여러 거리의 검증이 중요하다.

결과를 TF로 발행해 다시 확인한다.

```bash
ros2 launch fire_robot_camera_calibration extrinsic_validation.launch.py \
  camera_info_path:=$HOME/.ros/camera_info/camera.yaml \
  extrinsic_path:=$HOME/fire_robot_calibration/lidar_camera_extrinsic.yaml
```

검증 창에서 `c`는 스크린샷 저장, `q`는 종료다.

## 자주 바꾸는 인자

```bash
ros2 launch fire_robot_camera_calibration extrinsic_calibration.launch.py --show-args
```

USB 카메라나 다른 ROS 카메라를 사용할 때는 `start_camera:=false`와 함께
`raw_image_topic`, `input_transport`, `camera_frame`을 바꾼다. 압축 영상이면
토픽을 `/.../image_raw/compressed`로 지정하고 `input_transport:=compressed`를
사용한다.

## 결과 파일 주의

기존 국민대 카메라의 내부 행렬/왜곡 계수와 LiDAR 외부 변환은 새 Raspberry Pi
카메라 장착 상태에 유효하지 않다. 센서, 렌즈, 해상도, 초점, 장착 위치가 하나라도
바뀌면 다시 캘리브레이션한다.
