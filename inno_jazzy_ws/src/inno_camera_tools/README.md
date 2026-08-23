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

## ROS 없이 영상만 확인하고 사진 저장

단순 미리보기와 사진 촬영에는 위 ROS launch를 빌드할 필요가 없습니다.

```bash
cd ~/fire_robot_rpi
./run_camera.sh
```

`s`를 누를 때마다 `data/camera_capture/`에 사진 한 장이 저장되고, `q` 또는
`Esc`로 종료합니다.

## MODE 4 YOLO 추론

학습된 사람 검출 weight를 받은 뒤 Camera Module 3 ROS 영상에서 추론하는 노드는
`camera_person_detector`다. `.onnx` 모델은 ONNX Runtime CPU backend로 직접
실행하며, `.pt` 모델을 사용할 때만 Ultralytics가 필요하다. 모델이나 runtime이
없으면 노드는 오류 상태만 발행하고 사람으로 오판하지 않는다.

```bash
ros2 run inno_camera_tools camera_person_detector --ros-args \
  -p model_path:="$HOME/fire_robot_rpi/models/yolov8n_best.onnx"
```

출력 토픽은 `/camera/person_detections`, `/camera/person_detector_status`,
`/camera/person_detection_image`다. 기본 설정에서는 MODE 4가 1.5m 검사 위치에 도착해
`MODE4_CAMERA_YOLO_OBSERVING` 상태가 된 동안에만 실제 추론한다. 바운딩박스 JSON은
`image_width`, `image_height`, `x_min`, `y_min`, `x_max`, `y_max`, `confidence`를
포함하며 mode4 inspector가 LiDAR 점과 방향 기준으로 결합한다.
