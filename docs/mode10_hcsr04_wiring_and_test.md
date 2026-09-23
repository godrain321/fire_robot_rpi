# 모드 10: HC-SR04 배선과 초음파 회피 시험

## ESP32에 연결

기존 ESP32 DevKit V4가 모터 제어 USB 직렬 통신을 이미 담당하므로, 같은 ESP32에
HC-SR04를 연결한다. Raspberry Pi GPIO에 별도 센서를 달 필요가 없다.

| HC-SR04 | 연결 |
|---|---|
| VCC | ESP32 보드의 `5V`/`VIN` 핀 (USB 5V 공급 확인) |
| GND | ESP32 `GND` |
| TRIG | ESP32 `GPIO32` |
| ECHO | **전압 분배기 경유** ESP32 `GPIO33` |

ECHO는 5V 출력이므로 ESP32 입력에 직접 연결하지 않는다. 다음처럼 1kΩ과
2kΩ 저항을 직렬로 연결해 약 3.33V로 낮춘다. 센서와 ESP32의 GND는 공통이다.

```text
HC-SR04 ECHO ── 1kΩ ──┬── ESP32 GPIO33
                       │
                      2kΩ
                       │
ESP32 GND ─────────────┴── HC-SR04 GND

ESP32 GPIO32 ────────────── HC-SR04 TRIG
ESP32 5V/VIN ────────────── HC-SR04 VCC
```

기존 펌웨어의 모터 핀(12~14, 25~27)과 AS5048A SPI 핀(17~19, 23)은
그대로 둔다. 실제 보드에서 GPIO32/33이 다른 장치에 이미 사용 중이면 핀을
중복 연결하지 말고 펌웨어의 `US_TRIG`, `US_ECHO` 정의를 빈 핀에 맞춘다.
초음파 센서는 로봇 **전면 중앙**에 수평으로 고정한다. 반사파가 바퀴나 프레임에
맞지 않도록 센서 앞을 비운다.

## 코드 경로와 판정

1. ESP32 펌웨어가 100ms 간격으로 측정하고
   `US,<millis>,<distance_cm>,<valid>`를 기존 USB 직렬로 보낸다.
2. Pi의 직렬 브리지가 `/ultrasonic/front/range` (`sensor_msgs/Range`)로
   발행한다. 측정 실패는 `NaN`으로 표시하고 모터를 정지한다.
3. 모드 10은 초음파 거리(cm)를 실행 터미널에 표시한다. **50cm 미만**이면
   즉시 전진을 멈추고 `전방장애물주의!`를 출력한다.
4. 유효한 측정값이 1초 연속 50cm 미만이면 신선한 LiDAR `/scan`의 좌우
   공간을 비교해 빈 쪽으로 제자리 저속 회전 명령을 발행한다.
5. 전방이 초음파·LiDAR 모두 70cm 이상으로 0.3초 유지되면 저속 전진한다.
   측정이 끊기거나 회전 쪽 공간이 좁으면 정지한다. 20cm 이내 또는 8초
   동안 회피 실패 시 정지 상태가 유지되며 `/mode10/start`로 재시작한다.

모터 경로는 `/cmd_vel` → `cmdvel_to_esp32_serial` → ESP32 `M` 명령이다.
실제 좌·우 모터 명령은 `/motor/left_steps_per_sec`,
`/motor/right_steps_per_sec`에서 확인한다. 이 프로필은 단독 실행해야 한다.
한 개의 전방 초음파로 옆·뒤 공간을 확인할 수 없으므로 회전 방향에는 LiDAR가
필수다. 목적지까지 지도 기반 우회 경로를 만드는 모드는 아니며, 장애물 앞에서
멈추고 빈 방향으로 돌아 전진하는 근거리 회피 시험 모드다.

## 빌드와 실행

1. 수정한 `firmware/esp32_tb6600_bridge/esp32_tb6600_bridge.ino`를 ESP32에
   업로드한다. 업로드 중 Pi의 ROS 직렬 노드는 종료한다.
2. ROS 패키지를 빌드한다.

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select inno_drive_bridge inno_robot_bringup
```

3. **모터 전원을 끄거나 바퀴를 띄운 상태**에서 처음 실행하고 거리 로그와
   모터 명령 부호를 확인한다. 센서·LiDAR가 준비되면 모드 10은 자동으로
   0.06m/s 전진하므로 시작 전에 로봇 주변을 비운다.

```bash
cd ~/fire_robot_rpi
./run_mode10.sh
```

실제 USB 장치 경로가 다르면 인자로 덮어쓴다.

```bash
./run_mode10.sh esp32_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```

자동 전진 없이 거리만 확인하려면 `auto_start:=false`를 붙이고,
준비 후 아래 서비스를 호출한다. 정지는 Ctrl+C 또는 `/mode10/stop`이다.

```bash
./run_mode10.sh auto_start:=false
ros2 topic echo /ultrasonic/front/range
ros2 topic echo /mode10/status
ros2 topic echo /motor/left_steps_per_sec
ros2 topic echo /motor/right_steps_per_sec
ros2 service call /mode10/start std_srvs/srv/Trigger '{}'
ros2 service call /mode10/stop std_srvs/srv/Trigger '{}'
```

초기 시험은 80cm 이상 장애물 없음 → 50cm 미만 1초 미만 → 50cm 미만
1초 이상 순서로 확인한다. 회전 시 실제 바퀴가 서로 반대로 움직이는지 확인한다.
설정의 `left_sign: -1` 때문에 ROS 모터 토픽 숫자 부호만으로 회전 방향을
판단하지 않는다. 전진 시 전방이 정말 비어 있는지 확인한 다음 바퀴를 지면에
내려 저속으로 시험한다.
