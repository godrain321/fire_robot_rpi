# inno_camera_tools

Raspberry Pi Camera Module 3용 `camera_ros` 실행 패키지입니다.

```bash
ros2 launch inno_camera_tools camera_module_3.launch.py
```

기본 영상은 1280x720 RGB888이며 `/camera/image_raw`과
`/camera/camera_info`를 발행합니다. 내부 캘리브레이션을 완료한 뒤
`rectify:=true`를 사용하세요.

체커보드 내부 캘리브레이션 예시:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args \
  -r image:=/camera/image_raw \
  -r camera:=/camera
```

`--size`, `--square`는 실제 체커보드의 내부 코너 수와 한 칸 길이(m)로
바꿔야 합니다.

## 화각과 사람 촬영 거리 확인

저장소 루트의 실행 도구는 시작 전에 IMX708과 Pi 5 `rp1-cfe` 연결을 확인합니다.
줄자로 카메라와 사람 사이 거리를 잰 뒤 같은 값을 넘겨 실행합니다.

```bash
cd ~/fire_robot_rpi
./run_camera_fov_check.sh --distance 2.0
```

영상에는 3x3 구도선, 캘리브레이션 기준 수평/수직 화각, 해당 거리 평면에서 보이는
대략적인 폭/높이, 1.7m 사람의 예상 픽셀 높이가 표시됩니다. `+`/`-`로 표시 거리를
0.25m씩 조절하고 `s` 또는 `Space`로 원본과 주석 이미지를
`data/fov_check/`에 함께 저장합니다. 종료 키는 `q`입니다.

카메라가 이미 `/camera/image_raw`을 발행 중이면 중복 실행하지 않습니다.

```bash
./run_camera_fov_check.sh --use-running-camera --distance 2.0
```
