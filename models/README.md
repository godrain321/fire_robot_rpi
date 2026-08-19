# MODE 4 YOLO weights

Camera Module 3 데이터로 학습한 최종 사람 검출 weight를 기본적으로 다음 이름으로
둔다.

```text
models/mode4_person.pt
```

모델 파일(`*.pt`, `*.onnx`)은 크기가 크므로 `.gitignore` 대상이다. 다른 경로 또는
이름을 사용할 때는 통합 launch에 `yolo_model_path:=절대경로`를 넘긴다. 사람 class
ID의 기본값은 `0`이며 다른 데이터셋 구성이면 `camera_person_detector`의
`person_class_ids` 파라미터를 바꿔야 한다.
