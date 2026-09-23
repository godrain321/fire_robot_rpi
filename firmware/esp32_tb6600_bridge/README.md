# ESP32 TB6600 bridge firmware

대상 보드는 ESP32 DevKit V4의 ESP32-WROOM-32E 모듈이다. 기존 가상 STEP count와 함께 좌우 AS5048A 실제 회전 count 및 BNO055 gyro z를 USB serial로 전송한다.

## 핀 배치

| 기능 | ESP32 GPIO |
|---|---:|
| Left STEP/PUL | 25 |
| Left DIR | 26 |
| Left ENA | 27 |
| Right STEP/PUL | 14 |
| Right DIR | 12 |
| Right ENA | 13 |
| HC-SR04 TRIG | 32 |
| HC-SR04 ECHO (1kΩ/2kΩ 전압 분배 후) | 33 |
| AS5048A 공용 CLK / MISO / MOSI | 18 / 19 / 23 |
| AS5048A LEFT nCS / RIGHT nCS | 17 / 16 |
| AS5048A VDD5V | ESP32 5V/VIN |
| BNO055 ATX(SDA) / LRX(SCL) | 21 / 22 |
| BNO055 VCC / I2C 선택 핀 | ESP32 3V3 / GND |

첨부된 AS5048A 보드의 SPI 헤더는 `GND, nCS, CLK, MOSI, MISO, VDD5V, GND, GND`이고,
CJMCU-055 BNO055는 `ATX=SDA`, `LRX=SCL`로 사용하며 `I2C` 핀을 GND에 연결한다.
HC-SR04는 5V로 공급하고 ECHO의 5V 신호는 ESP32 GPIO33에 직접 넣지 않는다.
모드 10 배선과 시험 순서는 [문서](../../docs/mode10_hcsr04_wiring_and_test.md)를 따른다. Mode 11 센서 배선과 시험은 [문서](../../docs/mode11_bno055_as5048a_wiring.md)를 따른다.

GPIO12는 ESP32 strapping pin이다. TB6600 연결 상태 때문에 업로드 또는 부팅이 불안정하면 Right DIR 배선을 GPIO32 또는 GPIO33으로 옮기고 펌웨어의 `R_DIR`도 변경한다.

ESP32 GPIO는 3.3V 신호다. 사용하는 TB6600 모듈의 PUL/DIR/ENA 입력이 3.3V 신호를 확실히 인식하는지 데이터시트나 실제 입력 회로를 확인한다. 모터 전원과 USB 5V를 직접 연결하지 않는다. TB6600 신호 연결 방식에 맞는 공통 기준과 절연 입력 배선을 사용한다.

## encoder 없는 동작

`AccelStepper::runSpeed()`가 STEP pulse를 발생시킬 때 `currentPosition()`도 함께 증가하거나 감소한다. 펌웨어는 이 값을 100ms마다 다음 형식으로 보낸다.

```text
ENC,<millis>,<left_virtual_count>,<right_virtual_count>
```

이 값은 명령한 pulse의 누적값이지 실제 바퀴 회전 측정값은 아니다. 탈조, 미끄러짐, 바퀴 걸림은 검출하지 못하므로 SLAM/내비게이션의 최종 odometry로 사용하지 않는다.

## Arduino IDE 설정

1. Espressif ESP32 board package를 설치한다.
2. 보드는 `DOIT ESP32 DEVKIT V1` 또는 설치된 패키지에서 DevKit V4/WROOM-32E와 호환되는 항목을 선택한다.
3. Library Manager에서 `AccelStepper`를 설치한다.
4. Upload Speed는 우선 `115200` 또는 `460800`을 사용한다.
5. 해당 serial port를 선택하고 `esp32_tb6600_bridge.ino`를 업로드한다.

## 모터 연결 전 serial 시험

반드시 TB6600 모터 전원을 끈 상태에서 먼저 시험한다.

```bash
python3 ~/esp32_serial_check.py /dev/ttyUSB0 PING,1
python3 ~/esp32_serial_check.py /dev/ttyUSB0 ZERO,2
```

picocom에서는 CR/LF 어느 쪽으로 보내도 펌웨어가 명령을 분리한다.

```bash
picocom --baud 115200 /dev/ttyUSB0
```

수동 명령 예시(좌/우 step/s):

```text
PING,1
M,2,80,80
STOP,3
M,4,-80,80
STOP,5
ZERO,6
```

## 안전 시험 순서

1. 모터 전원을 끄고 `PING`, `ZERO`, `M,...`의 ACK/STAT/ENC만 확인한다.
2. 바퀴를 공중에 띄우고 TB6600 모터 전원을 켠다.
3. 낮은 `M` 명령 직후 `STOP`을 보내 한쪽씩 방향을 확인한다.
4. 반대 방향이면 우선 `INVERT_LEFT_DIR` 또는 `INVERT_RIGHT_DIR`을 수정한다.
5. 명령을 보내지 않았을 때 500ms 이내 `ERR,COMMAND_TIMEOUT_STOP`과 정지를 확인한다.
6. 모든 정지 시험이 성공한 뒤에만 최저 속도로 지상 시험한다.

`M` 명령이 갱신되지 않으면 500ms 후 자동 정지한다. ROS bridge가 10Hz 이상으로
계속 명령을 보내므로 정상 운전 중에는 watchdog이 갱신된다. 펌웨어의
`MAX_STEP_ACCEL`이 급격한 STEP 주파수 변화를 ramp 처리한다. 너무 둔하면 값을 조금
올리고, 꺾을 때 충격이 남으면 내린다.

## 엔코더와 IMU 텔레메트리

`ENC`는 ESP32가 **발생시킨 step 수**라 탈조와 미끄러짐을 알 수 없다. Mode 11은 이를
사용하지 않고 좌우 AS5048A에서 읽은 `ENC_PHYS`를 `/wheel_encoder_ticks`로 발행한다.
BNO055의 gyro z는 `IMU` 패킷으로 보내 `/imu/data_raw`가 된다. 두 AS5048A 중 하나라도
준비되지 않거나 BNO055 gyro 보정 단계가 2 미만이면 Mode 11 BLACKOUT 전환은 거부된다.

## 키보드 주행과 모드의 위치

키보드는 Raspberry Pi의 `keyboard_cmdvel_demo`가 읽는다. `w/x/a/d/s` 입력은
`/cmd_vel_keyboard` → `cmd_vel_mode_mux` → `/cmd_vel` → `cmdvel_to_esp32_serial` 순서로
전달되고, serial bridge가 `M,<seq>,<left_sps>,<right_sps>`를 보낸다. 이 펌웨어는 `M`
명령을 받아 TB6600용 좌우 STEP/DIR 펄스를 실제로 출력하므로 Mode 11에서도 키보드
주행이 포함된다. 명령이 500ms 동안 갱신되지 않으면 펌웨어가 모터를 정지한다.

펌웨어 안에 고정 거리 waypoint나 `1/2/11` 모드는 없다. 모드 선택과 `P` BLACKOUT은
ROS가 처리하고 ESP32는 센서 수집과 안전한 모터 출력만 담당한다.
