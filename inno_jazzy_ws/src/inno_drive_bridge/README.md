# inno_drive_bridge

ROS 2 `/cmd_vel`을 좌우 TB6600용 step/s 명령으로 변환해 ESP32에 보내고, ESP32의 누적 step/encoder count로 임시 wheel odometry를 계산하는 패키지다.

> **안전:** 최초 테스트는 TB6600 모터 전원을 분리하거나 바퀴를 공중에 띄운 상태에서 한다. 저속 설정과 비상 전원 차단 수단을 준비한 뒤 지상 테스트를 시작한다.

## ESP32 USB와 권한 확인

ESP32를 연결한 뒤 장치 후보를 확인한다.

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
dmesg --follow
```

일반적으로 USB-UART 보드는 `/dev/ttyUSB0`, ESP32의 USB CDC/JTAG는 `/dev/ttyACM0`으로 나타난다. 사용자를 `dialout` 그룹에 추가한 뒤 로그아웃/로그인하거나 재부팅해야 권한이 적용된다.

```bash
sudo usermod -aG dialout "$USER"
groups
```

Arduino Serial Monitor, `screen`, `minicom`처럼 같은 포트를 점유하는 프로그램은 먼저 종료한다.

## 빌드

이 저장소의 실제 ROS 2 워크스페이스는 `inno_jazzy_ws`다.

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select inno_drive_bridge
source install/setup.bash
```

Python serial 모듈이 없다면 다음 패키지가 필요하다.

```bash
sudo apt install python3-serial
```

## 실행과 키 조작

모터 전원을 분리하거나 바퀴를 띄운 상태에서 실행한다.

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch inno_drive_bridge drive_keyboard_demo.launch.py \
  serial_port:=/dev/ttyUSB0 \
  linear_speed:=0.08 \
  angular_speed:=0.35
```

TB6600과 ESP32 펌웨어는 `1/8 microstep` 기준이며, ROS 설정도 `microsteps: 8`이다. 속도를 낮춰 시험하려면 launch 인자만 바꾼다.

```bash
ros2 launch inno_drive_bridge drive_keyboard_demo.launch.py \
  serial_port:=/dev/ttyUSB0 linear_speed:=0.04 angular_speed:=0.20
```


ESP32가 `/dev/ttyACM0`이라면 launch argument만 바꾼다. 키 입력은 launch를 실행한 대화형 터미널에 포커스를 둔 상태에서 사용한다.

| 키 | 동작 | `/cmd_vel` |
|---|---|---|
| `w` | 전진 | `linear.x > 0` |
| `x` | 후진 | `linear.x < 0` |
| `a` | 제자리 좌회전 | `angular.z > 0` |
| `d` | 제자리 우회전 | `angular.z < 0` |
| `s` | 정지 | 모두 0 |
| `q` | 정지 발행 후 키보드 노드 종료 | 모두 0 |

통합 waypoint launch에서는 `1`이 수동주행, `2`가 RViz waypoint 주행, `4`가 이름
기반 단계주행이다. `4`를 누른 뒤 `w1,w5,w6`처럼 두 개 이상의 waypoint를 입력하고
Enter를 누르면 첫 점으로 즉시 출발한다. 첫 점 도착 후 Space를 누를 때마다 입력한
목록의 다음 점으로 이동한다. 모드 4는 `/cmd_vel_auto` 채널을 선택한다.

키보드 노드는 선택한 명령을 10 Hz로 계속 발행한다. serial bridge는 `/cmd_vel`이 기본 0.5초 동안 끊기면 `STOP`을 전송하며, ESP32에도 독립적인 500 ms watchdog이 있어야 한다.

키 입력이 launch 환경에서 TTY를 받지 못하는 경우 세 노드를 분리해 실행할 수 있다. serial bridge와 odometry를 먼저 실행한 다음, 별도 대화형 터미널에서 `ros2 run inno_drive_bridge keyboard_cmdvel_demo --ros-args --params-file .../drive_params.yaml`을 실행한다.

## 토픽 확인

별도 터미널마다 Jazzy와 workspace를 source한 뒤 확인한다.

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /motor/left_steps_per_sec
ros2 topic echo /motor/right_steps_per_sec
ros2 topic echo /esp32/status
ros2 topic echo /wheel_ticks
ros2 topic echo /wheel_odom
ros2 topic echo /wheel_path
```

`/motor/left_steps_per_sec`와 `/motor/right_steps_per_sec`에는 속도 제한과 방향 보정을
적용한 뒤 ESP32 UART로 실제 전송한 좌·우 목표 STEP/s가 각각 기록됩니다. 정지 또는
watchdog timeout에서는 둘 다 `0`을 발행합니다.

연결 상태와 주기를 함께 확인하려면 다음 명령도 유용하다.

```bash
ros2 topic hz /wheel_ticks
ros2 topic hz /wheel_odom
ros2 node info /cmdvel_to_esp32_serial
```

## Serial protocol

Pi에서 ESP32로 ASCII 한 줄 단위로 보낸다.

```text
M,<seq>,<left_sps>,<right_sps>
STOP,<seq>
PING,<seq>
ZERO,<seq>
```

ESP32에서 Pi로 받는 형식은 다음과 같다.

```text
ACK,<seq>
STAT,<millis>,<state>,<left_sps>,<right_sps>
ENC,<millis>,<left_count>,<right_count>
ERR,<message>
```

ESP32의 `ENC` count는 전진 시 증가하고 후진 시 감소하는 누적 signed count여야 한다. 실제 encoder가 없으면 발생시킨 signed step count를 같은 형식으로 보낸다. 현재 물리적 방향 보정은 ESP32 펌웨어의 `INVERT_LEFT_DIR=true`에서 처리하므로 ROS의 `left_sign`, `right_sign`은 모두 `1`로 유지한다. 펌웨어와 ROS에서 같은 방향을 중복 반전하지 않는다.

## RViz 확인

임시 wheel odometry만 확인할 때:

1. Fixed Frame을 `wheel_odom`으로 설정한다.
2. Add -> Odometry, Topic `/wheel_odom`을 선택한다.
3. Add -> Path, Topic `/wheel_path`를 선택한다.

`publish_tf` 기본값은 `false`이므로 `wheel_odom -> base_link` TF를 만들지 않는다. 따라서 기본 설정에서는 `/scan`과 wheel path를 한 좌표계에 겹쳐 볼 수 없다. 다른 odometry/EKF가 TF를 발행하지 않을 때만 임시로 `publish_tf: true`로 바꾸면 Fixed Frame `wheel_odom`에서 `/scan`도 함께 볼 수 있다.

SLAM 또는 AMCL 통합 후에는 해당 시스템이 `map/odom -> base_link` TF를 담당하게 하고, Fixed Frame을 `map`으로 설정한다. 그러면 `/map`, `/scan`, `/wheel_odom`, `/wheel_path`를 추가할 수 있다. 중복 TF publisher는 동시에 켜지 않는다.

## 초기 검증 순서

1. 모터 전원을 끄고 `w/a/d/x/s` 입력에 맞춰 `/cmd_vel`과 ESP32 `ACK/STAT`을 확인한다.
2. ESP32가 `ENC`를 보내는지, `/wheel_ticks`가 signed 누적값인지 확인한다.
3. 바퀴를 띄우고 좌우 회전 방향을 확인한다.
4. 방향이 틀리면 ESP32의 `INVERT_LEFT_DIR`/`INVERT_RIGHT_DIR`만 수정하고 ROS sign은 `1`로 유지한다.
5. `s`, `q`, `/cmd_vel` 중단 및 USB 연결 이상에서 정지하는지 검증한다.
6. 그 후에만 최저 속도로 지상 테스트한다.
