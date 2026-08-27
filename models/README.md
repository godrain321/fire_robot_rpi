# MODE 4 YOLO weights

## 현재 임시 시험 모델

```text
models/yolov8n_best_opencv_640.onnx  # Pi 5 기본값(OpenCV CPU, 고정 640)
models/yolov8n_best.onnx             # ONNX Runtime용 동적 입력 모델
```

라즈베리파이 5 기본 설치에는 `onnxruntime` Python 패키지가 없으므로 통합 launch는
정적 입력 OpenCV 모델을 사용한다. 검출 노드는 ONNX Runtime을 먼저 시도하고, 사용할
수 없으면 별도 설치 없이 ROS에 포함된 OpenCV DNN CPU backend로 자동 전환한다.

ONNX 메타데이터와 무결성 확인 결과:

- task: `detect`
- architecture: `YOLOv8n`
- input: dynamic batch, `3 x height x width` (기본 `640 x 640`)
- dynamic model output: `batch x 5 x anchors`
- static OpenCV model output: raw DFL head `batch x 65 x anchors`
- classes: `{0: person}`
- opset: `16`
- SHA-256: `9d24faab26bebe6f25de708e235f02ec8733848cb0025ff67e60e4685a37e739`
- embedded license metadata: [`AGPL-3.0`](https://www.gnu.org/licenses/agpl-3.0.html)

이 프로젝트에서 직접 학습한 모델이 아니고 원 출처는 확인되지 않았다. 임시 사람
검출·모드 4 연동 시험에만 사용하며, 외부 배포 전 모델 출처와 재배포 권한을 반드시
확인한다. 실제 운영 전에는 Camera Module 3 현장 데이터로 학습한 검증 모델로
교체한다.

다른 모델을 사용할 때는 통합 launch에 `yolo_model_path:=절대경로`를 넘긴다.
사람 class ID 기본값은 `0`이며 다른 class 구성이면 `camera_person_detector`의
`person_class_ids` 파라미터도 함께 변경한다.
