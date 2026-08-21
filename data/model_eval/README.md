# Temporary Mode 4 model evaluation

이 폴더는 `models/yolov8n_best.onnx`를 모드 4와 같은 native ONNX Runtime
backend로 실행한 공개사진 육안 점검 결과다. 입력 크기는 `640 x 640`, class는
`person(0)`, confidence threshold는 `0.50`이다.

## 결과 요약

| 묶음 | 목적 | 결과 |
|---|---|---|
| `yolov8n_best_robot_view_2026-08-19/` | 지면에 가까운 저시점·원거리 군중 확인 | 6장, 사람 박스 총 33개 |
| `.../near_1p5m/` | 로봇이 약 1.5m 앞에서 정지한 화면 크기 근사 | 6장 중 3장 검출, 3장 미검출 |

근거리 검출 장면의 confidence는 `0.87~0.90`이었지만 뒷모습, 강한 원근 왜곡과
극단적인 로우앵글에서는 사람을 놓쳤다. 인터넷 사진에는 실제 촬영 거리가 기록되어
있지 않아 `near_1p5m`는 화면 속 사람 크기를 기준으로 한 근사 분류다.

이 결과는 정답 bounding box가 있는 test set의 precision, recall 또는 mAP가 아니다.
실제 성능 평가는 Camera Module 3 Wide를 로봇의 실제 장착 높이·각도로 고정한 뒤
직접 촬영하고 라벨링한 데이터셋으로 다시 수행해야 한다.

## 파일 구성

- `originals/`: 공개 원본 사진
- `annotated/`: 현재 모델이 출력한 사람 bounding box
- `results.json`: 이미지별 검출 좌표, confidence와 개발 PC 추론 시간
- `*_contact_sheet.jpg`: GitHub에서 빠르게 비교하기 위한 모음 이미지
- `SOURCES.md`: 사진별 원본 페이지와 촬영자

개발 PC 추론 시간은 Raspberry Pi 5의 실제 처리속도로 해석하면 안 된다. 사진의
사용 조건과 촬영자 정보는 각 묶음의 `SOURCES.md`를 함께 확인한다.
