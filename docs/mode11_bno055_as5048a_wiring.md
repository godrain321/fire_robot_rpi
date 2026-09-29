# Mode 11 BNO055 + 좌우 AS5048A 배선과 지도 없는 Path 시험

## 결론

센서는 Raspberry Pi GPIO가 아니라 **모터를 제어하는 ESP32 DevKitC V4**에 연결한다.
ESP32가 좌우 엔코더를 100Hz, BNO055 gyro z를 50Hz로 읽고 기존 USB serial 한 가닥으로
Raspberry Pi에 보낸다. Raspberry Pi ROS bridge가 다음 토픽으로 변환한다.

- `/wheel_encoder_ticks`: `std_msgs/Int64MultiArray [left,right]`
- `/imu/data_raw`: `sensor_msgs/Imu`, `angular_velocity.z`
- `/imu/calibration`: BNO055 system/gyro 보정 단계

이 구성은 모터 STEP 생성과 실제 바퀴 회전을 같은 MCU가 읽고, Pi에 별도 I2C/SPI 배선을
길게 끌지 않아도 된다. 새 패킷과 두 번째 엔코더 CS, BNO055 I2C 초기화가 펌웨어에
추가되므로 **ESP32 펌웨어를 한 번 새로 업로드해야 한다.**

첨부 사진으로 IMU는 `VCC/GND/ATX/LRX/I2C/INT/RES/BOOT` 핀을 가진
**CJMCU-055 BNO055 모듈**, 엔코더는 `GND/nCS/CLK/MOSI/MISO/VDD5V/GND/GND`
핀을 가진 **AS5048A SPI 모듈**로 확인했다.
차동구동 거리 측정에는 동일 모듈과 직경방향 자석이 좌우에 각각 하나씩, 총 2세트가
필요하다. 엔코더 한 개만으로는 좌우 평균 이동량을 측정할 수 없어 Mode 11 전환이
완성되지 않는다.

## 전원 끈 상태에서 배선

ESP32 보드마다 헤더의 물리 순서가 다를 수 있으므로 최종 기준은 보드에 인쇄된
`3V3`, `GND`, `IOxx` 실크다. 아래 `J2/J3` 번호는 공식 38핀 ESP32-DevKitC V4 기준이다.
센서 세 개의 GND와 ESP32 GND를 반드시 공통으로 묶는다.

### 첨부 사진의 CJMCU-055 BNO055 1개: I2C

| 사진의 IMU 핀 | ESP32 연결 | 공식 DevKitC V4 헤더 | 설명 |
|---|---|---|---|
| `VCC` | `3V3` | J2-1 | 3.3V 공급 |
| `GND` | `GND` | J3-1 또는 J3-7 | 공통 접지 |
| `ATX` (`S_TX`처럼 보이는 핀) | `GPIO21` (`IO21`) | J3-6 | I2C SDA. UART 모드에서는 TX 핀 |
| `LRX` (`L_RX`) | `GPIO22` (`IO22`) | J3-3 | I2C SCL. UART 모드에서는 RX 핀 |
| `I2C` | `GND` | J3-1 또는 J3-7 | **반드시 LOW로 연결**해 I2C 모드와 주소 `0x28` 선택 |
| `INT` | 연결하지 않음 | - | 현재 펌웨어에서 사용하지 않음 |
| `RES` | 연결하지 않음 | - | reset 입력, 현재 사용하지 않음 |
| `BOOT` | 연결하지 않음 | - | bootloader 입력, 현재 사용하지 않음 |
| `S0`, `S1` 납땜 패드 | 출고 상태 유지 | - | 사진 오른쪽의 설정 패드 |

BNO055 보드의 Z축 화살표가 위쪽을 향하도록 차체에 단단히 고정한다. 차체를 위에서
봤을 때 반시계 회전에서 `/imu/data_raw.angular_velocity.z`가 양수가 되어야 한다.
반대면 재배선 없이 실행 인자 `imu_yaw_sign:=-1`을 사용한다. 진동이 큰 모터나 TB6600
위에 직접 고정하지 말고 차체 중심 부근에 스페이서와 단단한 브래킷으로 고정한다.

### 첨부 사진의 좌우 AS5048A 2개: 공용 SPI + 개별 nCS

사진에 보이는 SPI 헤더는 왼쪽부터
`GND, nCS, CLK, MOSI, MISO, VDD5V, GND, GND` 순서다. 사진과 같은 방향에서
읽었을 때의 순서이며, 실제 납땜 전에는 각 패드 옆 실크를 다시 확인한다.

| 사진의 엔코더 핀 | 왼쪽 모듈 | 오른쪽 모듈 | 공식 DevKitC V4 헤더 | 설명 |
|---|---|---|---|---|
| `VDD5V` | ESP32 `5V/VIN` | ESP32 `5V/VIN` | J2-19 (`5V`) | 사진의 5V 전원 입력 |
| `GND` 3개 | ESP32 `GND` | ESP32 `GND` | J3-1 또는 J3-7 | 최소 1개, 가능하면 3개 모두 공통 GND에 연결 |
| `CLK` | GPIO18에 같이 연결 | GPIO18에 같이 연결 | J3-9 | 공용 SPI clock |
| `MOSI` | GPIO23에 같이 연결 | GPIO23에 같이 연결 | J3-2 | 공용 ESP32→센서 데이터 |
| `MISO` | GPIO19에 같이 연결 | GPIO19에 같이 연결 | J3-8 | 공용 센서→ESP32 데이터 |
| `nCS` | **GPIO17** | **GPIO16** | J3-11 / J3-12 | LOW active, 좌우 개별 연결 |

AS5048A는 SPI 선 3개를 공유하고 nCS만 좌우를 분리한다. 첨부 모듈은 전원 패드가
`VDD5V`로 표시되어 있으므로 ESP32 DevKit의 USB 5V/VIN을 사용한다. AS5048A의 SPI
I/O는 내부 3.3V 영역을 사용하므로 ESP32의 3.3V SPI GPIO에 연결한다. 각 모듈 전원
핀 가까이에 0.1uF 세라믹 바이패스 커패시터를 추가하고 SPI 선은 모터 전원선과
떨어뜨려 짧게 배선한다. 외부 5V 전원을 쓸 경우에도 GND는 ESP32와 공통으로 묶는다.

각 바퀴 또는 감속기 출력축에 **직경방향 자화 자석**을 고정하고 AS5048A 칩 중심과
회전축 중심을 맞춘다. 자석과 센서가 서로 닿지 않게 하고 모듈 판매자가 지정한 간격을
유지한다. 축 편심이나 브래킷 흔들림은 정지 상태에서도 count 점프를 만든다.

### 전체 신호 연결 요약

```text
BNO055 (사진의 CJMCU-055)       ESP32 DevKitC V4
VCC      ───────────────────── 3V3
GND      ───────────────────── GND
ATX/SDA  ───────────────────── GPIO21
LRX/SCL  ───────────────────── GPIO22
I2C      ───────────────────── GND

AS5048A LEFT            ESP32                AS5048A RIGHT
VDD5V ───────────────── 5V/VIN ───────────── VDD5V
GND ────────────────── GND ───────────────── GND
CLK ───────────────── GPIO18 ─────────────── CLK
MISO ──────────────── GPIO19 ─────────────── MISO
MOSI ──────────────── GPIO23 ─────────────── MOSI
nCS ───────────────── GPIO17        GPIO16 ─ nCS

ESP32 USB ───────────────────────── Raspberry Pi 5 (`/dev/ttyUSB0` 등)
```

기존 핀은 그대로 유지된다: 좌 모터 GPIO25/26/27, 우 모터 GPIO14/12/13,
HC-SR04 GPIO32/33. GPIO16, 17, 18, 19, 21, 22, 23은 위 센서에 할당된다.
BNO055 3.3V나 AS5048A 5V/VIN을 TB6600 모터 전원 단자에 직접 연결하지 않는다.

## 펌웨어 업로드

수정된 스케치는 `firmware/esp32_tb6600_bridge/esp32_tb6600_bridge.ino`다. Arduino IDE에서
Espressif ESP32 보드 패키지와 `AccelStepper` 라이브러리를 설치하고 DevKit V4/WROOM-32E
호환 보드, 실제 ESP32 포트를 선택해 업로드한다. `Wire`와 `SPI`는 ESP32 보드 패키지에
포함된다. 업로드 중에는 ROS serial bridge와 Serial Monitor를 종료해 포트 중복 점유를
피한다.

업로드 뒤 115200 baud에서 다음 줄이 나와야 한다.

```text
ENC_PHYS,<ms>,<left_count>,<right_count>,<left_raw>,<right_raw>
IMU,<ms>,<gyro_z_rad_s>,<system_cal>,<gyro_cal>
```

`ERR,ENCODER_NOT_READY`이면 전원, 공통 GND, SPI, 좌우 CS, 자석 중심을 확인한다.
`ERR,IMU_NOT_READY`이면 ATX/LRX 뒤바뀜, `I2C`-GND 연결, 주소 `0x28`, 3.3V를 확인한다.
BNO055를 움직이지 않고 잠시 두어 gyro 보정이 2 이상이 되어야 Mode 11이 IMU를
유효하게 취급한다.

## ROS 빌드와 센서 확인

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/humble/setup.bash
colcon --log-base log_humble build --build-base build_humble \
  --install-base install_humble --packages-up-to inno_robot_bringup
source install_humble/setup.bash
```

펌웨어 업로드 후 ESP32 USB를 연결하고 Mode 11을 실행하면 serial bridge가 센서를 함께
발행한다. 별도 터미널에서 다음을 확인한다.

```bash
ros2 topic hz /wheel_encoder_ticks
ros2 topic echo /wheel_encoder_ticks
ros2 topic hz /imu/data_raw
ros2 topic echo /imu/data_raw --field angular_velocity.z
ros2 topic echo /imu/calibration
```

바퀴를 공중에 띄우고 `w`로 전진할 때 좌우 count의 보정 후 방향이 같아야 한다.
한쪽만 반대면 아래 실행 인자 중 해당 쪽을 `-1`로 준다.

```text
left_encoder_sign:=-1
right_encoder_sign:=-1
```

## 저장 지도 없이 RViz Path 확인

```bash
cd ~/fire_robot_rpi
./run_mode11_odom.sh esp32_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```

RViz Fixed Frame은 `odom`이고 저장 지도와 AMCL은 실행하지 않는다. 시작 직후에는 LiDAR
RF2O가 `/odom_selected`와 `/mode11/path`를 만든다. 키는 Mode 1과 동일하게
`w/x/a/d/s`, `q`이며 `P`가 BLACKOUT 전환이다.

시험 순서:

1. 모터 전원을 끄고 센서 토픽과 보정 상태를 확인한다.
2. 바퀴를 공중에 띄우고 좌우 count 방향과 IMU 회전 부호를 확인한다.
3. `./run_mode11_odom.sh`를 실행하고 RViz에서 녹색 `/mode11/path`를 확인한다.
4. 최저 속도로 `w/x/a/d/s` 주행하고 RF2O 궤적을 확인한다.
5. `P`를 누르고 BLACKOUT 배너와 fallback 시작 pose를 확인한다.
6. 계속 주행하면서 `/odom_selected`와 녹색 Path가 Encoder+BNO055로 이어지는지 확인한다.
7. 실제 이동거리와 RViz 거리가 다르면 `wheel_radius_m`를 실측 유효 반경으로 보정한다.

지도 좌표에서 기존 위치추정까지 함께 확인할 때는 `./run_mode11.sh`를 사용한다.
