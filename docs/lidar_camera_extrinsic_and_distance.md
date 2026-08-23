# RPLIDAR C1–Camera Module 3 Wide 외부 캘리브레이션과 거리 측정

이 경로는 기존 Rational 8계수 내부 캘리브레이션 파일을 수정하지 않는다. 내부 결과
`outputs/pi_camera3_wide_intrinsic/camera_info.yaml`을 입력으로 읽고, 새 데이터와
결과는 각각 `data/extrinsic`, `outputs/pi_camera3_wide_extrinsic`에 분리한다.

저장하는 변환 방향은 다음 하나로 고정한다.

```text
p_camera_optical = R_camera_lidar * p_lidar + t_camera_lidar
```

LiDAR 프레임은 현재 RPLIDAR C1 bringup과 같은 `laser`, 카메라는
`camera_optical_frame`이다. C1은 저장소의 `sllidar_c1_launch.py` 경로를 사용하므로
A1용 115200 baud launch를 사용하지 않는다.

## 준비 조건

- 내부 캘리브레이션을 실제 배포 해상도·초점 상태로 완료한다.
- 카메라와 LiDAR를 최종 마운트에 단단히 고정한 뒤 두 센서를 다시 움직이지 않는다.
- RPLIDAR C1을 연결해 `/dev/ttyUSB0`이 생기고 현재 사용자가 읽고 쓸 수 있어야 한다.
- 로컬 데스크톱 또는 X11 전달 GUI가 필요하다.
- 체커보드는 내부 보정과 같은 8×9 내부 코너, 한 칸 0.070 m를 사용한다.
- 보드 무늬가 인쇄된 종이는 평평하고 단단한 판에 부착한다. LiDAR는 무늬가 아니라
  판 표면을 측정하므로 레이저 스캔 높이가 실제 판을 지나야 한다.

## 1. 외부 보정 관측 수집

LiDAR를 연결한 다음 저장소 루트에서 실행한다.

```bash
cd /home/seeno04/fire_robot_rpi
./run_lidar_camera_calibration.sh
```

스크립트는 계산을 시작하기 전에 다음을 모두 필수 검사한다.

- IMX708 센서와 Pi 5 `rp1-cfe`
- 저장소 로컬 호환 libcamera
- RPLIDAR C1 직렬 장치
- 실제 `/camera/image_raw` 한 프레임
- 실제 `/scan` 한 메시지와 `frame_id=laser`
- 내부 YAML의 `rational_polynomial` 8계수와 촬영 해상도

각 자세에서 카메라 창의 체커보드 검출이 정상인지 확인한다. `SPACE`로 영상과 스캔을
고정한 뒤 LiDAR 평면도 창에서 체커보드 판에 해당하는 직선 점들만 사각형으로 드래그해
선택하고 `h`를 누른다. `r`은 선택 초기화, `q`는 종료다. 중단한 동일 세트를 이어갈
때만 다음을 사용한다.

```bash
./run_lidar_camera_calibration.sh --resume
```

최소 20자세가 계산 조건이며 24~30자세를 권장한다. 보드만 움직여 다음을 조합한다.

- 영상 왼쪽·중앙·오른쪽
- 가까운·중간·먼 거리
- 좌우 방향 회전
- 보드 윗부분을 센서 쪽과 반대쪽으로 기울이는 상하 방향 회전

보드를 계속 수직으로 둔 채 평행 이동만 하면 6자유도 일부를 계산할 수 없다. 주변 벽이나
상자가 LiDAR 선택 영역에 섞이지 않게 하고, 선분 적합이 실패하면 배경을 정리해 다시
선택한다.

관측 결과:

```text
data/extrinsic/observations.json
data/extrinsic/screenshots/
```

## 2. 외부 변환 계산

24번째 유효 자세를 저장하면 같은 명령이 자동으로 계산한다. 기존 관측만 다시 계산할 때는 다음을 실행한다.

```bash
cd /home/seeno04/fire_robot_rpi
./run_lidar_camera_calibration.sh --solve-only
```

카메라가 구한 자세별 보드 평면과 LiDAR 보드 선분의 point-to-plane 오차를 여러 자세에
걸쳐 최적화한다. 한 자세의 점 수가 많다는 이유로 그 자세가 결과를 지배하지 않도록
자세별 가중치를 같게 하고 robust loss를 사용한다. 보드 법선 다양성, 6자유도 Jacobian
rank와 조건수, 자세별 오차를 검사하며 퇴화한 데이터는 결과를 정상으로 승인하지 않는다.

주요 결과:

```text
outputs/pi_camera3_wide_extrinsic/lidar_camera_extrinsic.yaml
outputs/pi_camera3_wide_extrinsic/extrinsic_calibration_result.json
outputs/pi_camera3_wide_extrinsic/extrinsic_calibration_report.txt
```

배포 변환은 `lidar_camera_extrinsic.yaml`의 `T_camera_lidar`다. 입력 내부 파라미터의
경로와 해시, 전체·자세별 오차와 관측성 품질 지표도 함께 기록된다. 마운트 위치가
변했거나 카메라 해상도·초점이 바뀌면 내부 및 외부 보정을 다시 수행한다.

## 3. 카메라에서 클릭한 물체 거리 확인

외부 보정 완료 후 카메라와 C1을 동시에 켜는 다음 한 줄을 실행한다.

```bash
cd /home/seeno04/fire_robot_rpi
./run_lidar_camera_distance.sh
```

영상 위에 보정된 LiDAR 점이 표시된다. 물체 위의 투영점 가까이를 클릭하면 다음 값을
cm로 구분해 보여 준다.

- `lidar_range_cm`: LiDAR 원점에서 물체까지의 평면 방사거리
- `camera_forward_z_cm`: 카메라 광축 앞 방향 깊이
- `camera_euclidean_cm`: 카메라 원점에서의 3차원 직선거리

`c`는 현재 화면을 아래 경로에 저장하고 `q`는 종료한다.

```text
outputs/pi_camera3_wide_extrinsic/distance_screenshots/
```

영상과 스캔의 시간 차가 허용값을 넘거나 데이터가 오래되면 결과를 `INVALID`로
표시한다. 클릭 근처에 실제 LiDAR 투영점이 없으면 `NO_LIDAR_SUPPORT`이며 단안 영상만으로
거리를 임의 생성하지 않는다.

## 2D LiDAR의 거리 측정 한계

RPLIDAR C1은 한 높이의 2D 평면만 측정한다. 따라서 카메라에 보이는 모든 물체의
거리를 알 수 있는 것은 아니다. 물체가 실제 LiDAR 스캔 평면을 가로지르고 해당 반사점이
영상의 물체 위치로 정확히 투영될 때만 거리값이 유효하다. 스캔 높이보다 위나 아래에만
있는 물체, 투명·반사 표면, 빠르게 움직이는 물체에는 값이 없거나 부정확할 수 있다.

기본 직렬 장치가 다르면 두 실행 명령 모두 예를 들어 다음처럼 지정한다.

```bash
./run_lidar_camera_calibration.sh --lidar-port /dev/ttyUSB1
./run_lidar_camera_distance.sh --lidar-port /dev/ttyUSB1
```
