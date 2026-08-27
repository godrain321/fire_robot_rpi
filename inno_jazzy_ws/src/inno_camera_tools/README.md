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
`camera_person_detector`다. `.onnx` 모델은 ONNX Runtime CPU backend를 먼저
사용하고, 설치되어 있지 않으면 OpenCV DNN CPU backend로 자동 전환한다. OpenCV
fallback에는 고정 입력 모델 `yolov8n_best_opencv_640.onnx`를 사용한다. `.pt`
모델을 사용할 때만 Ultralytics가 필요하다. 모델을 열 수 없으면 노드는 오류 상태만
발행하고 사람으로 오판하지 않는다.

```bash
ros2 run inno_camera_tools camera_person_detector --ros-args \
  -p model_path:="$HOME/fire_robot_rpi/models/yolov8n_best_opencv_640.onnx"
```

출력 토픽은 `/camera/person_detections`, `/camera/person_detector_status`,
`/camera/person_detection_image`다. 기본 설정에서는 MODE 4가 1.5m 검사 위치에 도착해
`MODE4_CAMERA_YOLO_OBSERVING` 상태가 된 동안에만 실제 추론한다. 바운딩박스 JSON은
`image_width`, `image_height`, `x_min`, `y_min`, `x_max`, `y_max`, `confidence`를
포함한다. 통합 MODE 4는 로봇이 선택한 빨간 LiDAR 점을 향해 정지한 뒤 판별한다.
관찰 중 사람이 2프레임 이상 검출되면 카메라 픽셀 각도와 LiDAR 각도를 다시
비교하지 않고 현재 검사 중인 빨간 점 하나를 요구조자로 확정한다. 시연 환경에서는
사람과 다른 동적 장애물을 한 카메라 프레임에 동시에 두지 않는 것을 전제로 한다.

카메라와 모델만 독립적으로 확인할 때는 아래 명령을 사용한다. 사람을 검출하면
미리보기 창에 파란색 바운딩박스와 confidence가 표시된다.

```bash
cd ~/fire_robot_rpi
./run_camera_inference_check.sh
```

필요하면 `confidence_threshold:=0.35`, `inference_rate_hz:=5.0` 같은 ROS launch
인자를 명령 끝에 붙일 수 있다. 창과 카메라는 실행한 터미널에서 `Ctrl+C`로 종료한다.
