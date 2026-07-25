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
