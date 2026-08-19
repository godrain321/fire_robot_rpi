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

## Camera Module 3 바로 켜기

이 실행물은 ROS, `colcon`, 화각 계산 없이 Raspberry Pi Camera Module 3
IMX708의 원본 영상만 연다. Raspberry Pi 5에 CSI 케이블을 연결한 뒤 Picamera2와
OpenCV가 없다면 처음 한 번만 설치한다.

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv
```

이후에는 다음 한 줄로 실행한다.

```bash
cd ~/fire_robot_rpi
./run_camera.sh
```

영상 창에서 `s`를 누를 때마다 JPEG 사진이 한 장씩
`data/camera_capture/`에 저장된다. 여러 장이 필요하면 `s`를 여러 번 누른다.
`q` 또는 `Esc`로 종료한다. 기본 해상도는 1280x720이며, 필요하면
`./run_camera.sh --width 1920 --height 1080`처럼 바꿀 수 있다.

## Camera Module 3 Wide Rational 8계수 캘리브레이션

현재 배포 기준은 **원본 카메라 영상으로 계산하는 OpenCV Rational Polynomial
8계수 내부 캘리브레이션**이다. Raspberry Pi 5에서 사용할 호환 libcamera 런타임은
최초 한 번만 저장소 로컬에 빌드한다.

```bash
cd ~/fire_robot_rpi
./build_rpi_camera_runtime.sh
```

그다음 체커보드를 여러 위치·거리·기울기로 촬영한다. 캡처 스크립트는 실행 전에
IMX708 센서와 `rp1-cfe` 장치를 확인하고, 실제 ROS 원본 프레임 한 장이 수신되어야
촬영을 계속한다. 사진은 기본적으로 `data/intrinsic/calib_NNN.png`에 저장된다.

```bash
cd ~/fire_robot_rpi
./capture_intrinsic_images.sh
```

촬영을 중단했다가 같은 연속 번호로 이어가려면 `--resume`, 디스플레이가 없는
SSH/헤드리스 환경에서 자동 촬영하려면 `--no-preview`를 사용한다.

```bash
./capture_intrinsic_images.sh --resume
./capture_intrinsic_images.sh --resume --no-preview
```

사진 수집이 끝나면 다음 한 줄로 고정된 Rational 모델만 계산한다.

```bash
cd ~/fire_robot_rpi
./calibrate_camera.sh
```

계산이 끝난 뒤 원본과 Rational 8계수 보정 영상을 한 창에서 나란히 보고, 두 영상을
PNG로도 저장하려면 다음을 실행한다.

```bash
cd ~/fire_robot_rpi
./view_calibration_result.sh
```

GUI가 없는 SSH 환경에서는 `./view_calibration_result.sh --no-display`를 사용한다.
이때도 `outputs/pi_camera3_wide_intrinsic/comparison`에 동시 비교 PNG가 생성된다.
보정계수 원본은 `outputs/pi_camera3_wide_intrinsic/camera_info.yaml`이다.

내부 보정 완료 후 RPLIDAR C1을 연결하고, 고정된 최종 마운트에서 별도 외부 보정
관측을 수집한다. 기존 내부 결과는 읽기만 하며 수정하지 않는다.

```bash
cd ~/fire_robot_rpi
./run_lidar_camera_calibration.sh
```

외부 보정 결과는 `outputs/pi_camera3_wide_extrinsic/lidar_camera_extrinsic.yaml`에
저장된다. 완료 후 카메라와 C1을 동시에 실행해 영상의 LiDAR 투영점 근처를 클릭하면
cm 단위 거리를 확인할 수 있다.

```bash
cd ~/fire_robot_rpi
./run_lidar_camera_distance.sh
```

2D LiDAR이므로 실제 스캔 높이를 가로지르는 물체에만 거리값이 있다. 클릭 근처에
LiDAR 지원점이 없으면 값을 추측하지 않고 `NO_LIDAR_SUPPORT`로 표시한다. 보드 자세
수집법, 품질 조건과 전체 결과 경로는
[RPLIDAR C1–카메라 외부 보정 및 거리 측정 가이드](docs/lidar_camera_extrinsic_and_distance.md)를
참고한다.

캡처 해상도와 실제 배포 시 카메라 해상도·sensor mode는 반드시 같아야 한다.
사진 수집 조건, 고정 모델, 검증 절차와 결과 파일은
[체커보드 Rational 캘리브레이션 가이드](docs/checkerboard_rational_calibration.md)를
참고한다. ROS GUI와 fisheye 캘리브레이터는 과거 호환용이며 이 배포 경로가 아니다.

## Waypoint 용 최신 SLAM 지도

가장 최근에 생성한 SLAM 지도는 다음 두 파일이다.

- `inno_jazzy_ws/maps/fire_demo_20260729_182054.yaml`: 지도 해상도, 원점, 이미지 경로 등의 메타데이터
- `inno_jazzy_ws/maps/fire_demo_20260729_182054.pgm`: SLAM으로 생성한 occupancy grid 지도 이미지

이 두 파일을 한 쌍으로 유지하며, 향후 로컬 노트북에서 AMCL 위치 추정과 A1/A2 웨이포인트를 지정할 때 사용할 기준 지도다.

## 모드 4: 이름으로 선택한 waypoint를 Space로 한 점씩 주행

통합 주행 launch를 실행하고 AMCL 초기 위치를 지정한 다음, launch를 실행한 터미널에
포커스를 두고 `4`를 누른다. 나타나는 프롬프트에 waypoint를 두 개 이상 입력하고
Enter를 누른다.

```text
MODE 4 waypoints (example w1,w5,w6) > w1,w5,w6
```

로봇은 현재 위치에서 첫 목적지 `w1`로 즉시 출발한다. `w1` 도착 후 터미널에
`MODE4_REACHED:w1:SPACE_FOR:w5`가 표시되면 Space를 눌러 `w5`로 출발한다. 다시
도착한 뒤 Space를 누르면 `w6`로 이동한다. 입력하지 않은 waypoint로는 이동하지
않으므로 `w1,w5`만 입력했다면 두 번째 목적지는 `w5`다.

- 쉼표와 공백을 섞어 입력할 수 있으며 대소문자를 구분하지 않는다.
- 존재하지 않는 이름, `w0`, waypoint 한 개뿐인 입력은 출발 전에 거절한다.
- 주행 중 Space를 눌러도 다음 목적지를 건너뛰지 않고 `MODE4_BUSY`를 표시한다.
- 모드 4도 기존 A*, LiDAR/mmWave 동적 장애물 회피와 `/cmd_vel_auto`를 그대로 쓴다.
- 상태는 `ros2 topic echo /waypoint_queue_status`에서도 확인할 수 있다.

관련 빌드 명령:

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  inno_drive_bridge inno_autonav inno_robot_bringup
source install/setup.bash
```

## 전체 주행 rosbag 기록

위 workspace 빌드를 마치고 통합 bringup을 먼저 실행한 뒤, 별도 터미널에서 다음
도구를 실행한다.

```bash
cd ~/fire_robot_rpi
./record_robot_bag.sh
```

좌·우 모터 목표 STEP/s, LiDAR `/scan`, TF, 지도, 실제·계획 경로, waypoint/A*
상태, 동적장애물과 mmWave raw/filtered 값을 한 번에 기록한다. 시작 전 각 토픽을
`수신`, `대기`, `없음`으로 표시하지만 일부 센서가 없더라도 rosbag은 그대로
시작하며, 실행 뒤에 나타난 토픽도 기록 대상으로 유지한다. 결과는
`bags/fire_robot_날짜_시간/`, 점검 결과는 같은 이름의 `.topics.txt`에 저장한다.
안전한 종료는 `Ctrl-C`다.

## Git

ROS 2 빌드 산출물(`build/`, `install/`, `log/`)과 rosbag(`bags/`, `*.db3`, `*.mcap`, `*.sqlite3`)은 루트 `.gitignore`에서 제외한다.

```bash
cd ~/fire_robot_rpi
git add .gitignore README.md inno_jazzy_ws/src/inno_bringup
git commit -m "Add Raspberry Pi lidar bringup configuration"
```
