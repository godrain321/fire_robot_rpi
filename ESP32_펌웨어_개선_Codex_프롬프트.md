# ESP32 TB6600 펌웨어 개선 Codex 작업 프롬프트

나는 Raspberry Pi 5 + ROS2 Jazzy + ESP32 DevKit V4(ESP32-WROOM-32E) + TB6600 2개 + 좌우 스테퍼 모터로 차동구동 로봇을 만들고 있다.

## 프로젝트 위치

```text
프로젝트: ~/fire_robot_rpi
ROS2 workspace: ~/fire_robot_rpi/inno_jazzy_ws
ESP32 펌웨어: ~/fire_robot_rpi/firmware/esp32_tb6600_bridge/esp32_tb6600_bridge.ino
ROS2 패키지: ~/fire_robot_rpi/inno_jazzy_ws/src/inno_drive_bridge
```

## 현재까지 성공한 내용

1. Raspberry Pi와 ESP32 USB Serial 통신 성공
2. `/dev/ttyUSB0`, 115200 baud 사용
3. `PING,1` 전송 시 `ACK,1` 수신 성공
4. `STAT`, `ENC` 메시지 수신 성공
5. ROS2 keyboard drive launch 성공
6. `w/x/a/d/s` 입력으로 실제 모터가 회전하는 것 확인
7. 실제 encoder는 없으며 `AccelStepper::currentPosition()`을 가상 encoder count로 사용
8. `STOP` 정상 동작 확인
9. 500ms command watchdog 정지 확인

## 현재 실제 방향 문제

차체를 공중에 띄우고 시험한 결과가 다음과 같다.

```text
현재:
w → 좌회전
x → 우회전
a → 전진
d → 후진
s → 정지

원하는 동작:
w → 전진
x → 후진
a → 제자리 좌회전
d → 제자리 우회전
s → 정지
```

이 결과는 키맵 문제가 아니라 왼쪽 모터의 물리적 방향 기준이 반대인 상태로 판단한다. 우선 다음 방향 보정을 적용해라.

```cpp
const bool INVERT_LEFT_DIR = true;
const bool INVERT_RIGHT_DIR = false;
```

단, 실제 관찰이 `a=후진`, `d=전진`이었다고 사용자가 정정하면 오른쪽 모터를 반전해야 한다.

```cpp
const bool INVERT_LEFT_DIR = false;
const bool INVERT_RIGHT_DIR = true;
```

`K` 명령의 키맵이나 ROS2 differential drive 계산식을 뒤집어서 임시 보정하지 마라. 물리적 모터 방향은 `INVERT_LEFT_DIR`, `INVERT_RIGHT_DIR`에서만 보정한다.

논리적 모터 명령은 반드시 다음을 유지한다.

```text
W: left 양수, right 양수 → 전진
X: left 음수, right 음수 → 후진
A: left 음수, right 양수 → 제자리 좌회전
D: left 양수, right 음수 → 제자리 우회전
S: 양쪽 즉시 정지
```

가상 encoder count도 로봇의 논리 방향을 따라야 한다.

```text
전진: left 증가, right 증가
후진: left 감소, right 감소
좌회전: left 감소, right 증가
우회전: left 증가, right 감소
```

ROS의 `left_sign`, `right_sign`은 모두 `1`로 유지하고 펌웨어와 ROS 양쪽에서 중복 반전하지 마라.

## 현재 핀 배치

```cpp
#define L_STEP 25
#define L_DIR  26
#define L_EN   27

#define R_STEP 14
#define R_DIR  12
#define R_EN   13
```

GPIO12는 ESP32 strapping pin이지만 배선이 완료되어 있으므로 이번 작업에서는 핀을 임의로 변경하지 마라. 관련 주의사항만 README에 유지해라.

## 유지할 Serial protocol

Pi에서 ESP32로 보내는 형식:

```text
M,<seq>,<left_sps>,<right_sps>
STOP,<seq>
PING,<seq>
ZERO,<seq>
K,<seq>,<W|X|A|D|S>
```

ESP32에서 Pi로 보내는 형식:

```text
ACK,<seq>
STAT,<millis>,<state>,<left_sps>,<right_sps>
ENC,<millis>,<left_count>,<right_count>
ERR,<message>
```

기존 Raspberry Pi serial protocol 및 ROS topic 구조를 변경하지 마라.

## 추가 개선 목표

현재 다음 문제도 있다.

1. 모터 속도가 너무 느리다.
2. 출발, 정지 및 방향 전환 시 떨림이 심하다.
3. 현재 `setSpeed()` 후 `runSpeed()`를 사용해 목표 속도가 즉시 변경된다.
4. 급격한 속도 변화와 즉시 방향 반전이 떨림과 탈조를 유발할 가능성이 있다.
5. TB6600 DIP를 `1/8 microstep`으로 변경할 예정이다.
6. 실제 encoder는 연결하지 않는다.

## 요청 작업

### 1. 기존 구조 보존

기존 펌웨어를 먼저 전부 읽고 현재 정상 동작하는 protocol과 안전 기능을 보존해라.

### 2. 방향 보정

우선 다음 설정을 적용해라.

```cpp
const bool INVERT_LEFT_DIR = true;
const bool INVERT_RIGHT_DIR = false;
```

README에 수정 전 관찰과 수정 후 기대 결과를 기록해라.

```text
수정 전:
w → 좌회전
x → 우회전
a → 전진
d → 후진

수정 후 기대:
w → 전진
x → 후진
a → 좌회전
d → 우회전
```

### 3. 속도와 가속도 설정

속도 관련 설정을 펌웨어 상단 한 곳에 모아라.

```cpp
const float MAX_STEP_SPEED = 1600.0F;
const float DEFAULT_KEY_SPS = 400.0F;
const float MAX_ACCEL_SPS2 = 500.0F;
const float SPEED_UPDATE_PERIOD_SEC = 0.005F;
const float ZERO_SPEED_EPSILON = 0.5F;
const unsigned long COMMAND_TIMEOUT_MS = 500;
```

값을 한 곳에서 수정하여 최고 속도, 수동 키 속도, 가속도를 쉽게 조절할 수 있어야 한다.

### 4. 목표 속도와 현재 속도 분리

다음 상태를 분리해라.

```cpp
targetLeftSps
targetRightSps
currentLeftSps
currentRightSps
```

`M` 또는 `K` 명령을 받으면 target speed만 변경한다. `loop()`에서 current speed가 target speed로 서서히 접근해야 한다.

### 5. Acceleration/slew-rate 제한

다음 방식의 non-blocking ramp를 구현해라.

```text
maxDelta = MAX_ACCEL_SPS2 × dt

current < target → 최대 maxDelta만큼 증가
current > target → 최대 maxDelta만큼 감소
```

`delay()`를 사용하지 말고 `millis()` 또는 `micros()` 기반으로 구현한다.

### 6. 안전한 방향 반전

다음과 같은 반전 명령에서 DIR을 즉시 바꾸면 안 된다.

```text
+400 step/s
→ 감속
→ 0
→ DIR 변경
→ -400 step/s까지 가속
```

모든 방향 반전은 반드시 속도 0을 통과하게 해라.

### 7. STOP과 watchdog 우선 처리

`STOP`과 watchdog 정지는 ramp 없이 즉시 적용해야 한다.

```cpp
targetLeftSps = 0;
targetRightSps = 0;
currentLeftSps = 0;
currentRightSps = 0;
leftMotor.setSpeed(0);
rightMotor.setSpeed(0);
```

다음 안전 기능을 유지해라.

- 500ms command timeout
- watchdog 즉시 정지
- 부팅 시 STOP
- 속도 hard clamp
- CR, LF, CRLF 명령 종료 처리
- serial line overflow 처리
- 잘못된 명령에서는 모터 상태 변경 금지
- blocking motor 함수 사용 금지
- `delay()` 사용 금지

### 8. 실제 encoder 없는 count

실제 encoder 없이 `AccelStepper::currentPosition()`을 발생한 STEP pulse count로 사용한다.

```text
ENC,<millis>,<left_logical_count>,<right_logical_count>
```

`INVERT_LEFT_DIR` 또는 `INVERT_RIGHT_DIR`이 true여도 전송되는 count는 물리적 DIR 신호가 아니라 로봇 논리 방향을 따라야 한다.

### 9. ZERO 처리

`setCurrentPosition(0)`이 AccelStepper 내부 speed를 0으로 만드는 부작용을 처리해라. `ZERO` 이후 기존 current ramp speed를 다시 `setSpeed()`하여 의도치 않게 모터가 멈추지 않게 해라.

### 10. STAT 상태

`STAT`에는 target이 아니라 ramp가 적용된 실제 current speed를 출력해라.

```text
STAT,<millis>,<state>,<current_left_sps>,<current_right_sps>
```

state는 최소 다음을 구분한다.

```text
READY: 부팅 완료
STOP: 목표와 현재 속도가 모두 0
RAMP: 현재 속도가 목표 속도로 이동 중
RUN: 현재 속도가 목표 속도에 도달
FAILSAFE: watchdog으로 정지
```

### 11. M 명령 숫자 검증

`String.toFloat()`만 사용하면 잘못된 문자열이 0으로 처리될 수 있으므로 `strtof()` 등을 사용해 전체 문자열이 정상 숫자인지 검증해라.

잘못된 예:

```text
M,10,abc,100
M,11,100xyz,100
M,12,,100
```

잘못된 숫자에서는 모터 상태를 변경하지 말고 다음을 발행한다.

```text
ERR,BAD_M_VALUE
```

seq가 비어 있는 명령도 오류 처리한다.

### 12. K 명령

K 명령은 다음 target speed를 설정한다.

```text
W: left=+DEFAULT_KEY_SPS, right=+DEFAULT_KEY_SPS
X: left=-DEFAULT_KEY_SPS, right=-DEFAULT_KEY_SPS
A: left=-DEFAULT_KEY_SPS, right=+DEFAULT_KEY_SPS
D: left=+DEFAULT_KEY_SPS, right=-DEFAULT_KEY_SPS
S: 양쪽 즉시 정지
```

### 13. TB6600 microstep 변경

TB6600 DIP를 `1/8 microstep`으로 변경할 예정이다.

```text
200 full steps/revolution × 8 microsteps
= 1600 pulses/revolution
```

다음 파일의 두 노드 설정을 모두 `microsteps: 8`로 변경해라.

```text
~/fire_robot_rpi/inno_jazzy_ws/src/inno_drive_bridge/config/drive_params.yaml
```

```yaml
cmdvel_to_esp32_serial:
  ros__parameters:
    microsteps: 8

step_count_to_odom:
  ros__parameters:
    microsteps: 8
```

Python 노드 내부의 기본값도 모두 `microsteps: 8`로 맞춰라.

다음 기존 설정은 유지한다.

```yaml
serial_port: /dev/ttyUSB0
baudrate: 115200
wheel_radius: 0.04
wheel_separation: 0.30
motor_full_steps_per_rev: 200
gear_ratio: 1.0
left_sign: 1
right_sign: 1
cmd_timeout_sec: 0.5
```

### 14. 수정 대상 파일

다음 파일을 업데이트해라.

```text
~/fire_robot_rpi/firmware/esp32_tb6600_bridge/esp32_tb6600_bridge.ino
~/fire_robot_rpi/firmware/esp32_tb6600_bridge/README.md
~/fire_robot_rpi/inno_jazzy_ws/src/inno_drive_bridge/config/drive_params.yaml
~/fire_robot_rpi/inno_jazzy_ws/src/inno_drive_bridge/inno_drive_bridge/cmdvel_to_esp32_serial.py
~/fire_robot_rpi/inno_jazzy_ws/src/inno_drive_bridge/inno_drive_bridge/step_count_to_odom.py
```

필요하면 ESP32 펌웨어용 `TESTING.md` 또는 `CHANGELOG.md`를 추가해도 된다.

### 15. 떨림 원인 문서화

README에 다음을 구분해서 작성해라.

코드로 완화 가능한 항목:

- 출발 가속 ramp
- 감속 ramp
- 방향 반전 시 속도 0 통과
- 시작 속도 조절
- microstep 증가

코드만으로 해결할 수 없는 항목:

- 잘못된 모터 코일 쌍
- TB6600 current DIP 설정 오류
- 모터 정격전류 불일치
- 한 상 단선
- 전원 전압 강하
- 기계적 공진
- 바퀴나 축 걸림
- 느슨한 단자

### 16. 안전 시험 절차 문서화

README에 다음 시험 순서를 작성해라.

1. 바퀴를 공중에 띄움
2. 모터 배터리 OFF 상태에서 펌웨어 업로드
3. `PING → ACK` 확인
4. `STOP` 확인
5. 모터 배터리 ON
6. `s` 먼저 입력
7. `w` 직후 `s`
8. `x` 직후 `s`
9. `a` 직후 `s`
10. `d` 직후 `s`
11. 물리적 방향 확인
12. ENC count 부호 확인
13. 500ms watchdog 확인
14. 이상 진동 시 모터 배터리 즉시 OFF

## 검증 제한

ESP32에 자동 업로드하거나 모터를 실행하지 마라.

가능하면 다음만 수행해라.

- 코드 작성 및 정적 검사
- `arduino-cli`와 필요한 core/library가 이미 설치돼 있으면 compile만 수행
- 도구나 library가 없으면 임의 설치하지 말고 컴파일하지 못한 이유 보고
- ROS2 Python 구문 검사
- `inno_drive_bridge` colcon build
- `git diff --check`
- `git status`

`build/`, `install/`, `log/`, `bags/` 및 Arduino 생성 산출물은 Git에 포함하지 마라.

## 완료 보고 내용

작업 완료 후 다음을 보고해라.

- 수정 파일 목록
- 적용한 모터 방향 반전 설정
- 수정 전/후 키 동작
- acceleration ramp 구현 방식
- `MAX_STEP_SPEED`
- `DEFAULT_KEY_SPS`
- `MAX_ACCEL_SPS2`
- 방향 반전 시 0 통과 처리
- STOP 및 watchdog 처리
- microstep 변경 내용
- ENC logical count 처리
- ESP32 compile 또는 미실행 사유
- ROS2 빌드 결과
- ESP32에 자동 업로드하지 않았다는 점
- 실제 모터를 실행하지 않았다는 점

## 최우선 조건

- 기존에 성공한 USB Serial protocol을 깨뜨리지 마라.
- Pi가 보내는 `M` 명령 형식을 변경하지 마라.
- `ACK`, `STAT`, `ENC`, `ERR` 형식을 변경하지 마라.
- 실제 encoder가 없어도 모터가 동작해야 한다.
- STOP과 watchdog은 acceleration ramp보다 항상 우선해야 한다.
- 키맵이나 differential drive 공식을 뒤집어 방향 문제를 숨기지 마라.
- 물리적 모터 방향은 펌웨어의 `INVERT_LEFT_DIR`, `INVERT_RIGHT_DIR`로 보정해라.
- 펌웨어와 ROS에서 같은 방향을 중복 반전하지 마라.
