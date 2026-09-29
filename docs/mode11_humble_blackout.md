# Mode 11: Humble LiDAR blackout 위치추정 시험

## 현재 저장소에서 확인한 입력

Mode 1의 `w/x/a/d/s/q`와 `/cmd_vel_keyboard → cmd_vel_mode_mux → /cmd_vel →
ESP32`를 그대로 사용한다. `P`만 Mode 11 launch에서 활성화되는 추가 키다.

RF2O는 `/scan`으로 `/odom_rf2o` (`nav_msgs/Odometry`)를 만들고, 일반 launch에서는
`odom → base_link` TF도 발행한다. Mode 11에서는 RF2O의 `publish_tf`를 끄고
선택 노드가 TF를 단독 발행한다. AMCL은 저장 지도와 scan을 사용하며
`map → odom` 보정은 Mode 11 선택 노드가 단독 발행한다.

ESP32 펌웨어가 좌우 AS5048A를 `ENC_PHYS`, BNO055 gyro z를 `IMU` 패킷으로
전송한다. 기존 serial bridge는 이를 `/wheel_encoder_ticks`
(`std_msgs/Int64MultiArray [left,right]`)와 `/imu/data_raw` (`sensor_msgs/Imu`)로
발행한다. `/wheel_ticks`는 계속 가상 모터 STEP이므로 실제 fallback 기본 입력으로
쓰지 않는다. 상세 배선과 펌웨어 절차는
[Mode 11 센서 배선 문서](mode11_bno055_as5048a_wiring.md)를 따른다.
`Int64MultiArray`에는 timestamp가 없어 엔코더 신선도는 마지막 수신 시각으로 확인한다.
IMU는 유효한 gyro z, header timestamp, 0.25초 이하 연속 샘플이 필요하다. BNO055 gyro
보정 단계가 2 미만이면 bridge가 각속도를 미제공 상태로 표시해 P 전환을 막는다.

## 상태와 TF

```text
원본 /scan ──────────────────────────────── RViz (계속 표시)
       └─ Mode 11 scan gate (NORMAL만 발행) ─ /mode11/scan
                                               ├─ RF2O → /odom_rf2o
                                               └─ AMCL → /amcl_pose
IMU angular_velocity.z + 좌우 count ────────── fallback 적분기
RF2O / fallback ── 선택 노드 ── /odom_selected ── odom → base_link
AMCL 보정 / BLACKOUT 시 마지막 보정 고정 ─────── map → odom
선택 TF ── tf_to_path ── /mode11/path (map frame)
base_link ── 개략 차체/전방 화살표 ── /mode11/robot_body
```

`NORMAL`에서는 RF2O pose가 선택된다. `P`를 누르면 RF2O·IMU·엔코더와
AMCL 보정의 최신성·유효성을 확인하고, 마지막 RF2O `(x,y,yaw)`와 현재
count·IMU timestamp를 fallback 원점으로 사용한다. 첫 BLACKOUT pose는 마지막
RF2O pose와 동일하다. 이후 scan gate가 닫혀 RF2O와 AMCL은 새 scan을 받지
않고, 선택 노드는 뒤늦게 도착한 RF2O/AMCL 결과도 무시한다. 원본 `/scan`은
진단용으로 계속 수신한다.

fallback 전진거리는 좌우 AS5048A count 변화에 바퀴 반경 0.04m와
16384 count/회전을 적용한 평균이다. 좌우 장착 방향은 launch의
`left_encoder_sign`, `right_encoder_sign`으로 보정한다.
yaw는 IMU `angular_velocity.z`를 timestamp 간격으로 적분한다. x/y에는
전진거리와 그 구간 중간 yaw만 사용한다. IMU 가속도 이중적분은 하지 않는다.
센서 값이 끊기면 pose 적분을 중단하고 마지막 pose를 유지한다. 키보드 `s`와
ESP32 직렬 watchdog은 기존대로 작동한다.

## Ubuntu 22.04 / ROS 2 Humble 실행

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/humble/setup.bash
colcon --log-base log_humble build --build-base build_humble \
  --install-base install_humble --packages-up-to inno_robot_bringup

cd ~/fire_robot_rpi
./run_mode11.sh

# 저장 지도 없이 odom 기준 궤적만 확인
./run_mode11_odom.sh
```

기본 센서 토픽은 `/imu/data_raw`, `/wheel_encoder_ticks`이며 AS5048A 해상도는
16384 count/회전이다. `esp32_port:=...`, `lidar_port:=...`, `map_yaml:=...`,
`left_encoder_sign:=...`, `right_encoder_sign:=...`, `imu_yaw_sign:=...`를 실제
방향에 맞춘다. 바퀴 유효 반경은 `wheel_radius_m:=...`로 실측 보정한다. 센서가
준비되지 않으면 P는 누락되거나 오래된 입력 이름을 출력하며 거부된다.

`./run_mode11.sh`의 RViz Fixed Frame은 `map`이다. Mode 1의 자동 초기 위치추정도
기본 실행하며, `/mode11/scan`만 입력으로 사용하므로 BLACKOUT 뒤에는 새 scan을
받지 않는다. 시작 위치를 아는 경우
`set_initial_pose:=true initial_pose_x:=... initial_pose_y:=...
initial_pose_yaw:=...`를 전달할 수 있다. 자동 초기 위치추정이 수렴하지 않으면
RViz의 **2D Pose Estimate**로 AMCL 초기 자세를 지정한다. `/amcl_pose`가 아직
없으면 P 전환은 거부된다. 자동 초기화를 끄려면 `auto_localization:=false`를 쓴다.
RViz의 차체 표시는 실제 CAD/URDF가 없는 현재 저장소를 위한 개략 표식이다.

시험 순서: 바퀴를 띄우고 센서 토픽 확인 → Mode 11 실행 → RViz에서 지도,
`base_link`, `/mode11/path` 확인 → `w/x/a/d/s`로 주행 → `/odom_rf2o`와
`/odom_selected` 일치 확인 → `P` → 전환 배너와 동일한 fallback 시작 pose
확인 → scan이 계속 표시되지만 `/mode11/scan`과 `/odom_rf2o`는 새 값이
없고 `/odom_selected`·Path는 IMU/엔코더로 계속 움직이는지 확인.


## 지도 없는 odom Path

`./run_mode11_odom.sh`는 map_server, AMCL, 자동 초기 위치추정을 실행하지 않는다.
RViz Fixed Frame과 `/mode11/path`는 `odom`을 사용한다. LiDAR RF2O로 시작한 뒤 P 이후
Encoder+BNO055로 계속 이어지는 상대 궤적을 저장 지도 없이 바로 검사할 수 있다.
