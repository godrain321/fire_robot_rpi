# Fire Robot RPi

Raspberry Pi 5와 ROS 2 Jazzy 또는 Humble 기반 화재 대피 안내 로봇이다. 최종 운영 화면에서 선택하는
모드는 세 개뿐이다.

| 운영 모드 | 입력 | 기능 |
|---|---|---|
| **Mode 1** | `1` | 기존 `w/x/a/d/s` 키보드 수동주행 |
| **Mode 2** | `2` | 기존 Mode 2~11 기능을 결합한 최종 통합 화재대피 주행 |
| **Mode 3** | `3` | 안내 음성을 재생하면서 빠르게 제자리 1회전 |

기존 개별 Mode 2~11 코드는 통합 기능의 내부 구성 요소와 단독 시험 프로필로 유지한다.
운영자는 `./run_modes.sh` 실행 후 `1`, `2`, `3`만 사용한다.

## Mode 2에 통합된 기능

| 기존 기능 | Mode 2에서의 역할 |
|---|---|
| 기존 Mode 2 | 저장 waypoint 그래프, cost-aware A*, 경로 단순화, path follower |
| 기존 Mode 3 | LiDAR 동적장애물 접근 후 C4001 mmWave 사람 가능성 판별 |
| 기존 Mode 4 | Camera Module 3 YOLO 사람 박스와 LiDAR 후보를 결합한 요구조자 확인 |
| 기존 Mode 5 | 출구 평가, 순차 탐색, 출구 차단 처리, 요구조자 동행 대피 상태머신 |
| 기존 Mode 6 | MLX90640 열화상 입력과 RViz thermal cost grid |
| 기존 Mode 7 | 열 위험도를 반영한 출구 선택, 경로 생성, 상황 변화 재계획 |
| 기존 Mode 8 | 전체 대피 상태머신과 열화상 hazard costmap 결합 |
| 기존 Mode 9 | USB 스피커 대피 안내 즉시 재생 및 7초 주기 반복 |
| 기존 Mode 10 | HC-SR04 50cm 미만 상태가 1초 지속되면 LiDAR 여유 방향으로 회피 |
| 기존 Mode 11 | 정상 RF2O 위치추정과 `P` 입력 후 Encoder+IMU fallback 위치추정 |

Mode 2의 공개 상태는 `/operator_mode=2`다. 내부에서는 기존에 검증한 상태머신을 재사용하기
위해 `/drive_mode`가 대피 주행 `5`, mmWave 검사 `3`, 카메라 검사 `4` 사이에서 자동으로
바뀐다. 이 내부 번호는 사용자가 다시 입력하는 모드가 아니다.

## 하드웨어

- Raspberry Pi 5, Ubuntu 24.04 + ROS 2 Jazzy 또는 Ubuntu 22.04 + ROS 2 Humble
- ESP32-WROOM-32E + TB6600 구동부
- RPLIDAR C1
- C4001 mmWave presence sensor
- Raspberry Pi Camera Module 3 Wide + YOLO ONNX 모델
- MLX90640 열화상 카메라
- MQ-135 가스 센서
- HC-SR04 전방 초음파 센서
- BNO055 IMU
- 좌·우 AS5048A 엔코더 2개
- USB 오디오 장치와 스피커

센서 배선은 다음 문서를 따른다.

- [HC-SR04 배선과 시험](docs/mode10_hcsr04_wiring_and_test.md)
- [BNO055·AS5048A 배선](docs/mode11_bno055_as5048a_wiring.md)

## 최초 설치와 빌드

```bash
cd ~/fire_robot_rpi
sudo bash ./install_ubuntu2404_dependencies.sh
./build_rpi_camera_runtime.sh

cd ~/fire_robot_rpi/inno_jazzy_ws
# Ubuntu 22.04: humble, Ubuntu 24.04: jazzy
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to inno_robot_bringup
source install/setup.bash
```

새 소스나 launch를 수정했으면 `colcon build`를 다시 실행해야 한다.

장치 포트는 실행 전에 확인한다.

```bash
ls -l /dev/serial/by-id/
```

## 최종 통합 실행 명령

```bash
cd ~/fire_robot_rpi
./run_modes.sh
```

장치 경로가 기본값과 다르면 다음처럼 덮어쓴다.

```bash
./run_modes.sh \
  esp32_port:=/dev/ttyUSB0 \
  lidar_port:=/dev/ttyUSB1 \
  mmwave_port:=/dev/ttyAMA0
```

실행하면 LiDAR, RF2O, AMCL, Encoder+IMU selector, ESP32 serial bridge, mmWave,
카메라/YOLO, 열화상, gas hazard, A*, path follower, ultrasonic safety, 음성, RViz를
한 번에 시작한다. 로봇은 **Mode 1의 정지 상태**에서 대기하므로 숫자 `2`를 누르기 전에는
통합 자율주행을 시작하지 않는다.

## 숫자 `2`를 누른 뒤 순서

1. 현재 키보드 속도와 남아 있는 자율주행 goal을 0과 cancel로 초기화한다.
2. 공개 운영 상태를 `/operator_mode=2`로 바꾸고 기존 통합 대피 상태머신을 시작한다.
3. 초음파 안전 필터가 활성화된다. 이후 모든 통합 자율주행 속도는 이 필터를 통과한 뒤
   `/cmd_vel`과 ESP32 모터 명령으로 전달된다.
4. USB 스피커가 대피 안내를 즉시 한 번 재생하고 Mode 2가 유지되는 동안 7초마다 반복한다.
5. 지도, `map → odom → base_link`, LiDAR, thermal/gas hazard, waypoint와 센서 준비 상태를
   확인한다. 필수 입력이 없으면 모터를 출발시키지 않고 대기 또는 오류 상태를 출력한다.
6. 저장된 출구 후보 각각에 대해 정적 지도, 동적장애물, 열화상 cost, 가스 hazard를
   합성해 현재 위치에서 이동 가능한 안전 경로와 비용을 평가한다.
7. 아직 확인하지 않은 출구 중 안전하고 비용이 가장 낮은 출구를 선택한다.
8. 현재 위치에서 waypoint 그래프와 cost-aware A*로 경로를 만들고, 경로를 단순화한 뒤
   RViz의 `/planned_path`와 waypoint marker에 표시한다.
9. path follower가 `/cmd_vel_auto`를 만들고 mode mux와 초음파 안전 필터를 거쳐 ESP32에
   좌·우 모터 STEP/s 명령을 전송한다.
10. HC-SR04가 50cm 미만을 1초 이상 연속 검출하면 먼저 정지하고
    `전방장애물주의!`를 출력한다. LiDAR 좌·우 여유 공간을 비교해 안전한 쪽으로 회전하며,
    양쪽이 모두 막혔거나 20cm 이하면 정지 상태를 유지한다.
11. 경로 주변의 LiDAR 동적장애물은 빨간 marker로 표시한다. 출구 접근 중 정지 후보가
    확인되면 2m 앞에 정지하고 C4001 mmWave 검사 흐름을 자동 실행한다.
12. mmWave presence가 사람 조건을 만족하면 marker를 파란색으로 바꾸고 요구조자 동행
    대피로 전환한다. 사람이 아니면 출구 차단 가능성을 기록하고 다른 출구를 재평가한다.
13. 이동하는 LiDAR 사람 후보가 만들어지면 카메라 방향으로 접근하고 Camera Module 3의
    YOLO 결과와 후보 위치를 결합해 요구조자를 확인한다.
14. 확인된 요구조자를 안내할 때 LiDAR track을 계속 확인한다. 로봇과 요구조자 거리가
    멀어지거나 track이 끊기면 정지하고, 다시 가까워지면 출구 주행을 재개한다.
15. 열화상 또는 가스 cost가 바뀌어 현재 경로가 위험해지면 A*를 다시 실행한다. 현재
    출구보다 다른 출구가 안전해지면 진행 중 경로를 취소하고 새 출구로 전환한다.
16. 로봇과 동행 요구조자가 안전 출구에 도착하면 완료 상태와 도착 안내를 출력한다.

### Mode 2에서 `P`: LiDAR 위치추정 blackout

Mode 2 시작 직후 위치추정은 다음 흐름이다.

```text
/scan → RF2O /odom_rf2o → Mode 11 selector → /odom_selected
                                     └──────→ odom → base_link TF
```

`P`를 누르면 다음 조건을 먼저 확인한다.

- 최신 RF2O pose 존재
- 좌·우 `/wheel_encoder_ticks`가 최소 두 번 이상 수신됨
- `/imu/data_raw.angular_velocity.z`가 유효하고 최신 상태
- BNO055 gyro 보정 단계가 충분함
- AMCL의 map 정렬값 존재

모든 조건이 정상일 때 마지막 RF2O `(x,y,yaw)`를 Encoder+IMU 적분기의 시작값으로 그대로
복사한 후 BLACKOUT으로 전환한다. 이후 RF2O와 AMCL에는 새 scan을 전달하지 않으며,
`/odom_selected`와 `odom → base_link`는 좌우 엔코더 이동량과 IMU gyro z로 계속 갱신한다.
원시 `/scan`은 장애물 감지와 Mode 10 안전 회피용으로 계속 사용할 수 있지만 위치추정에는
사용하지 않는다. 필요한 센서가 없거나 오래됐으면 BLACKOUT을 거부하고 RF2O를 유지한다.

RViz에서는 `/lidar_path`가 전환 전후 같은 pose에서 계속 이어져야 한다.

## Mode 1: 키보드 수동주행

`1`을 누르면 진행 중인 통합 임무나 Mode 3을 취소하고 0 속도를 발행한 뒤 수동 입력으로
돌아온다.

| 키 | 동작 |
|---|---|
| `w` | 전진 |
| `x` | 후진 |
| `a` | 제자리 좌회전 |
| `d` | 제자리 우회전 |
| `s` | 정지. 자동 모드 중에는 취소 후 Mode 1 복귀 |
| `c` | 현재 자동 모드 취소 후 Mode 1 복귀 |
| `q` | 정지 후 키보드 노드 종료 |

Mode 1의 모터 경로는 다음과 같다.

```text
keyboard → /cmd_vel_keyboard → mode mux
→ ultrasonic filter의 Mode 1 bypass → /cmd_vel
→ cmdvel_to_esp32_serial → ESP32 → TB6600
```

초음파 필터는 Mode 1에서 명령을 변경하지 않으므로 기존 키보드 주행 방식이 유지된다.

## Mode 3: 안내 음성 + 제자리 1회전

숫자 `3`을 누르면 다음 동작이 동시에 시작된다.

- 진행 중인 Mode 2 임무와 모터 명령 취소
- USB 스피커로 다음 문구 1회 재생
- `/cmd_vel_auto.angular.z=1.0 rad/s`로 약 6.28초 동안 제자리 회전
- 한 바퀴 시간이 끝나면 0 속도를 발행해 자동 정지

> 안녕하세요 저는 화재대피안내로봇입니다 안전한출구로 안내해드리겠습니다

기본 음원은 패키지의 `inno_robot_bringup/audio/mode3_intro.wav`다.
`~/fire_robot_audio/mode3_intro.wav`가 있으면 그 파일을 우선 사용한다. 음성과 회전은 같은
Mode 3 요청 callback에서 시작하므로 동시에 시작된다. 완료 후 Mode 3의 정지 상태로
남으며 `3`을 다시 누르면 한 번 더 실행하고, `1`을 누르면 키보드 주행으로 돌아간다.

회전 속도는 필요할 때 실행 인자로 조절할 수 있다.

```bash
./run_modes.sh mode3_spin_speed_radps:=0.8
```

바퀴를 공중에 띄운 상태에서 방향과 1회전 시간을 먼저 확인한 뒤 지상 시험한다.

## RViz와 상태 확인

RViz Fixed Frame은 `map`이다. 다음 항목을 확인한다.

- 저장 지도와 `map → odom → base_link → laser`
- 로봇 현재 위치와 방향
- `/scan`
- `/planned_path`
- `/lidar_path`
- 동적장애물 빨간 marker
- 확인된 사람/요구조자 파란 marker
- thermal cost grid와 hazard 상태

별도 터미널에서 확인할 주요 토픽은 다음과 같다.

```bash
# 설치된 배포판에 맞춰 humble 또는 jazzy 사용
source /opt/ros/humble/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash

ros2 topic echo /operator_mode
ros2 topic echo /drive_mode_status
ros2 topic echo /evacuation_demo/status
ros2 topic echo /mode10/status
ros2 topic echo /mode11/state
ros2 topic echo /mode3_demo/status
ros2 topic echo /wheel_encoder_ticks
ros2 topic echo /imu/data_raw --field angular_velocity.z
ros2 topic echo /cmd_vel
```

## 안전 시험 순서

1. TB6600 모터 전원을 끄고 모든 센서 토픽이 정상인지 확인한다.
2. ESP32에서 `ENC_PHYS`, `IMU`, `US` 패킷이 출력되는지 확인한다.
3. 바퀴를 공중에 띄우고 Mode 1의 `w/x/a/d/s`와 정지 기능을 확인한다.
4. Mode 3을 실행해 음성과 회전 방향, 약 1회전 후 정지를 확인한다.
5. Mode 2를 실행해 RViz 경로 생성과 모터 명령을 확인한다.
6. 초음파 센서 앞을 50cm 미만으로 1초 이상 막아 정지와 회피 방향을 확인한다.
7. Mode 2에서 `P`를 눌러 RF2O pose와 fallback pose가 끊기지 않는지 확인한다.
8. 가장 낮은 속도로 실제 바닥 주행을 시작한다.

Mode 10과 Mode 11을 분리해 진단할 때는 기존 `run_mode10.sh`, `run_mode11.sh`,
`run_mode11_odom.sh`를 사용할 수 있다. 이 스크립트들은 최종 운영 모드 번호가 아니라
센서와 fallback을 따로 검증하기 위한 개발 도구다.

## 저장소 구성

| 경로 | 내용 |
|---|---|
| `inno_jazzy_ws/src/inno_robot_bringup` | 최종 통합 launch, 위치추정 selector, Mode 3 |
| `inno_jazzy_ws/src/inno_autonav` | 출구 평가, A*, 동적장애물, 사람 검사, path follower |
| `inno_jazzy_ws/src/inno_drive_bridge` | 키보드, mode mux, 초음파 안전 필터, ESP32 serial bridge |
| `inno_jazzy_ws/src/inno_mmwave` | C4001 수신과 사람 가능성 상태 |
| `inno_jazzy_ws/src/inno_camera_tools` | Camera Module 3와 YOLO 추론 |
| `inno_jazzy_ws/src/inno_thermal` | MLX90640와 thermal cost layer |
| `inno_jazzy_ws/src/inno_evacuation_voice` | Mode 2 주기 대피 음성 |
| `maps` | 지도, waypoint, no-go 설정 |
| `models` | YOLO ONNX 모델 |
| `firmware/esp32_tb6600_bridge` | ESP32 모터·초음파·엔코더·IMU 펌웨어 |
