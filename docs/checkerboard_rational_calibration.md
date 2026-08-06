# Raspberry Pi Camera Module 3 Wide 내부 캘리브레이션

이 문서의 현재 배포 경로는 Raspberry Pi Camera Module 3 Wide(IMX708)에서
체커보드 원본 PNG를 수집한 뒤, 폴더에 저장된 사진으로 내부 파라미터를 계산하는
과정이다. 최종 모델은 **OpenCV Rational Polynomial 하나뿐**이며 왜곡계수 8개를
고정된 순서로 저장한다. 캡처 단계와 계산 단계는 분리되어 있으므로 캘리브레이터가
카메라를 임의로 제어하거나 다른 모델을 자동 선택하지 않는다.

저장소 루트의 `build_rpi_camera_runtime.sh`, `capture_intrinsic_images.sh`,
`calibrate_camera.sh`를 순서대로 사용한다. 기존 ROS GUI 및 fisheye 보정 도구는
레거시 호환용이며 **이 문서에서 요청한 Rational 8계수 파이프라인이 아니다**.

## 전체 실행 순서

### 1. Raspberry Pi 5 카메라 런타임 빌드(최초 한 번)

Ubuntu 24.04 기본 libcamera 대신 이 카메라 스택과 호환되는 저장소 로컬 런타임을
최초 한 번 빌드한다.

```bash
cd ~/fire_robot_rpi
./build_rpi_camera_runtime.sh
```

빌드 결과는 저장소의 `.camera_runtime` 아래에 놓이며 캡처 스크립트가 자동으로
우선 사용한다. 이미 정상적으로 빌드되어 있다면 매 촬영마다 다시 실행할 필요가 없다.

### 2. 원본 체커보드 사진 캡처

```bash
cd ~/fire_robot_rpi
./capture_intrinsic_images.sh
```

기본 설정은 1280×720 원본 PNG를 최대 80장까지
`data/intrinsic/calib_000.png`, `calib_001.png`, ... 순서로 저장한다. 이 스크립트는
촬영 전에 sysfs에서 IMX708이 실제로 바인딩되었는지, `/dev/media*` 중 Pi 5 CSI
수신기인 `rp1-cfe`가 있는지를 **필수 조건**으로 검사한다. 이어서 ROS
`/camera/image_raw`에서 지정한 폭의 실제 `sensor_msgs/Image` 한 장이 제한 시간 안에
도착해야만 스트리밍 확인을 통과한다. 따라서 장치 이름만 존재하고 실영상이 나오지
않는 상태에서는 사진 수집으로 넘어가지 않는다.

미리보기 창에서 `q`는 종료이며, 기본 자동 저장은 선명도와 자세 차이를 확인해
적합한 사진을 저장한다. 중단한 촬영을 이어갈 때는 다음처럼 실행한다.

```bash
./capture_intrinsic_images.sh --resume
```

`--resume`은 기존 디렉터리의 파일이 끊김 없는 `calib_NNN.png` 순서인지 검증하고
다음 번호부터 이어 쓴다. 기본 실행은 비어 있지 않은 출력 디렉터리를 덮어쓰지 않는다.

GUI가 없는 SSH/헤드리스 환경에서는 자동 저장을 유지한 채 미리보기만 끈다.

```bash
./capture_intrinsic_images.sh --resume --no-preview
```

새 촬영을 헤드리스로 시작한다면 `--resume` 없이 `--no-preview`만 사용하면 된다.
`--manual`은 미리보기의 `s` 키가 필요하므로 `--no-preview`와 함께 사용할 수 없다.
다른 촬영 해상도를 쓸 때는 예를 들어 `--width 2304 --height 1296`처럼 지정한다.
단, **캘리브레이션 촬영 해상도와 sensor mode는 실제 배포 시 카메라 설정과 반드시
같아야 한다.** 해상도가 달라지면 특히 `fx`, `fy`, `cx`, `cy`를 그대로 사용할 수 없다.

### 3. Rational 8계수 계산

촬영을 끝낸 뒤 저장소 루트에서 다음 한 줄을 실행한다.

```bash
cd ~/fire_robot_rpi
./calibrate_camera.sh
```

이 래퍼는 기본적으로 `data/intrinsic/*.png`만 입력으로 사용하고 결과를
`outputs/pi_camera3_wide_intrinsic`에 기록한다. 활성 모델은
`cv2.CALIB_RATIONAL_MODEL`이며 왜곡계수 순서는
`[k1, k2, p1, p2, k3, k4, k5, k6]`으로 고정된다. fisheye 모델을 계산·비교하거나
자동 선택하지 않는다.

## 시작 전 확인

기본 보드는 **8열 × 9행 내부 코너**, 한 칸의 실제 변 길이는 **0.070 m**다. 여기서 8×9는 검은색·흰색 사각형의 개수가 아니라 서로 맞닿는 격자선의 내부 교차점 개수다. 보드가 가로 8개, 세로 9개의 내부 코너라면 사각형 수는 가로 9개, 세로 10개다. 인쇄물의 한 칸을 자로 재서 `square_size_m`와 일치하는지 반드시 확인해야 한다.

OpenCV의 `pattern_size` 순서는 `(columns, rows)`이므로 설정도 열을 먼저 기록한다. 보드 규격이 다르면 [checkerboard_rational.yaml](../config/checkerboard_rational.yaml)의 세 값만 실제 규격에 맞게 바꾼다.

```yaml
board:
  inner_corners_cols: 8
  inner_corners_rows: 9
  square_size_m: 0.070
```

기본 캘리브레이션 설정은 다음과 같다. YAML 값을 바꾸면 CLI에서 별도로 재정의하지 않은 항목에 적용된다.

```yaml
calibration:
  validation_ratio: 0.20
  max_iterations: 200
  epsilon: 1.0e-12
  mad_multiplier: 3.0
  max_rejection_ratio: 0.15
  minimum_training_views: 20
  cv_folds: 5
  strict_resolution: false
  remove_duplicates: false
  duplicate_distance_threshold: 0.06
  coverage_grid_cols: 8
  coverage_grid_rows: 6
  sample_undistort_count: 5
```

`strict_resolution=false`이면 기준 해상도와 다른 사진을 제외하고 사유를 기록하며, `true`이면 즉시 오류로 끝낸다. `remove_duplicates=false`이면 가까운 pose descriptor는 경고와 보고서에만 남기고 자동 제외하지 않는다. `true`일 때만 `duplicate_distance_threshold`보다 가까운 중복 자세를 제거 대상으로 삼는다. coverage 기본 격자는 8열×6행이며 `sample_undistort_count`는 육안 확인용 왜곡 보정 샘플의 최대 개수다.

사진은 같은 해상도로 촬영하고 다음 자세를 고르게 포함하는 것이 좋다.

- 보드가 영상 중앙뿐 아니라 네 모서리와 각 변 가까이에 있는 사진
- 보드가 작게 보이는 사진과 크게 보이는 사진
- 수평·수직 방향 회전 및 앞뒤 기울기가 서로 다른 사진
- 초점이 맞고 모든 내부 코너가 영상 안에 들어온 사진

연속 촬영한 거의 같은 자세만 많이 넣으면 이미지 수는 늘어도 파라미터를 구속하는 정보는 거의 늘지 않는다. 특히 넓은 화각의 주변부 왜곡을 안정적으로 추정하려면 네 모서리에서 코너가 충분히 관측되어야 한다.

## 캘리브레이터 직접 실행

래퍼의 입력 glob이나 출력 위치를 세밀하게 바꿔야 할 때만 저장소 루트에서 Python
CLI를 직접 실행한다.

```bash
python3 tools/calibration/calibrate_checkerboard_rational.py \
  --images "data/intrinsic/*.png" \
  --config config/checkerboard_rational.yaml \
  --output-dir outputs/pi_camera3_wide_intrinsic
```

주요 CLI 인수는 다음과 같다.

- `--images`: 입력 이미지 glob. 셸이 먼저 확장하지 않도록 따옴표로 감싼다.
- `--config`: YAML 설정 파일.
- `--output-dir`: 결과 디렉터리. 이미 존재하면 `--force` 없이는 덮어쓰지 않는다.
- `--board-cols`, `--board-rows`, `--square-size-m`: 보드 설정의 CLI 재정의.
- `--validation-ratio`, `--cv-folds`, `--seed`: 검증 비율, 교차검증 수, 재현성 seed.
- `--camera-name`: `camera_info.yaml`에 기록할 카메라 이름.
- `--strict-resolution`: 기준 해상도와 다른 파일을 제외하지 않고 즉시 오류로 처리한다.
- `--remove-duplicates`: pose descriptor가 매우 가까운 중복 자세 제거를 활성화한다. 기본값은 경고만 기록하는 것이다.
- `--log-level`: 로그 상세도(`DEBUG`, `INFO`, `WARNING`, `ERROR`)를 지정한다.
- `--force`: 기존 결과 디렉터리에 새 결과를 쓸 때 명시한다.

설정 우선순위는 `CLI > YAML > 코드 기본값`이다. PNG 이외의 형식도 glob만 바꾸면 된다.

```bash
python3 tools/calibration/calibrate_checkerboard_rational.py \
  --images "data/intrinsic/*.jpg" \
  --config config/checkerboard_rational.yaml \
  --output-dir outputs/pi_camera3_wide_intrinsic \
  --board-cols 8 --board-rows 9 --square-size-m 0.070 \
  --validation-ratio 0.20 --cv-folds 5 --seed 42 \
  --camera-name pi_camera3_wide --log-level INFO
```

이 CLI는 저장된 이미지에만 접근한다. 카메라 연결과 실제 프레임 수신 여부는 앞의
`capture_intrinsic_images.sh`가 필수 게이트로 확인한다. 입력 glob에 정상 이미지가
하나도 없으면 보정값을 임의로 만들지 않고 오류로 종료한다.

## 계산 모델

### Zhang 평면 캘리브레이션

체커보드의 모든 3D 점은 보드 좌표계의 `Z=0` 평면에 둔다. 내부 코너 `(column, row)`의 좌표는 다음과 같다.

```text
X = column × square_size_m
Y = row    × square_size_m
Z = 0
```

서로 다른 자세로 본 평면과 영상 사이의 호모그래피들이 카메라 내부 행렬의 제약을 제공한다. 그 초기값과 각 사진의 보드 자세를 함께 비선형 최적화하여 전체 코너의 재투영 오차를 줄인다. 위치, 크기, 회전, 원근 기울기가 다양한 사진이 필요한 이유가 이 제약의 독립성과 영상 전역의 왜곡 관측을 확보하기 위해서다.

### 내부 행렬 K

```text
K = [ fx  0  cx ]
    [  0 fy  cy ]
    [  0  0   1 ]
```

`fx`, `fy`는 픽셀 단위 초점거리이고 `cx`, `cy`는 주점의 픽셀 좌표다. 이 값은 캘리브레이션을 수행한 이미지 해상도에 종속된다.

### Rational Polynomial 8계수

왜곡계수는 항상 다음 순서다.

```text
D = [k1, k2, p1, p2, k3, k4, k5, k6]
```

카메라 좌표 `(Xc, Yc, Zc)`를 정규화하면 다음과 같다.

```text
x = Xc / Zc
y = Yc / Zc
r² = x² + y²
```

방사 왜곡 비율은 분자와 분모를 갖는다.

```text
       1 + k1 r² + k2 r⁴ + k3 r⁶
L(r) = ---------------------------
       1 + k4 r² + k5 r⁴ + k6 r⁶
```

접선 성분까지 적용한 정규화 좌표와 픽셀 좌표는 다음과 같다.

```text
xd = x L(r) + 2 p1 x y + p2 (r² + 2 x²)
yd = y L(r) + p1 (r² + 2 y²) + 2 p2 x y

u = fx xd + cx
v = fy yd + cy
```

`k1`~`k6`는 방사 성분, `p1`, `p2`는 접선 성분이다. 높은 차수의 계수는 데이터가 부족하거나 자세 분포가 치우치면 관측점 사이에서 불안정한 곡선을 만들 수 있다. 따라서 작은 학습 RMS만으로 결과를 승인하지 않고, 교차검증의 파라미터 변동과 Rational 분모 및 방사 매핑의 수치 안정성도 함께 검사한다.

## 처리 흐름

### 1. 입력 및 코너 검출

파일명을 정렬해 처리하므로 같은 입력과 seed에서 순서가 재현된다. 첫 정상 이미지의 해상도를 기준으로 삼고, 기본 동작에서는 다른 해상도 이미지를 제외해 사유를 남긴다. strict 해상도 검사를 켠 경우에는 즉시 오류로 종료한다. 손상 파일, 크기가 다른 파일, 검출 실패 파일도 결과 메타데이터에 기록한다.

코너 검출에는 `cv2.findChessboardCornersSB()`를 사용한다. 큰 시야각, 기울어진 보드, 명암 변화가 있는 영상에서도 전체 체커보드 구조를 이용해 안정적이고 정밀한 코너를 직접 반환하기 때문이다. `found=True`, 코너 수가 정확히 `columns × rows`, 모든 좌표가 유한값, 모든 좌표가 영상 범위 안이라는 조건을 모두 만족한 사진만 정상 검출로 사용한다.

accepted/rejected 오버레이에는 코너와 순서, 보드 외곽, 중심, 영상 점유 면적이 표시된다. blur score는 사람이 품질을 판단할 참고값이며 그 값 하나만으로 사진을 자동 제외하지 않는다.

### 2. 자세 분포와 데이터 분할

각 사진에서 정규화 중심, 보드 점유 면적, 상단 변 각도, 변 길이 비율, 원근 변화 지표를 계산한다. 이 descriptor로 거의 같은 연속 자세를 경고하고, 위치·크기·회전·기울기가 고르게 포함되도록 학습/검증 집합을 나눈다. 동일한 seed는 동일한 분할을 만든다.

coverage map은 영상 격자별 코너 관측 횟수를 보여 준다. 중앙 집중이나 네 모서리 관측 부족 경고가 있으면 자세를 보충해 다시 촬영하는 것이 좋다.

### 3. 최적화와 MAD 기반 사진 단위 제거

초기 내부 행렬은 평면 대응점으로 계산하고 왜곡계수 8개는 0에서 시작한다. OpenCV 최적화는 내부 행렬 초기값을 사용하며 Rational 8계수를 함께 구한다.

활성화되는 기본 플래그는 `cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_RATIONAL_MODEL`뿐이다. Thin Prism, Tilted, zero-tangent, 고정 주점, 고정 종횡비 모델/제약은 활성화하지 않으며 다른 왜곡 모델을 계산·비교·자동 선택하지 않는다.

초기 보정 뒤 각 사진의 per-view RMS로 다음 robust threshold를 계산한다.

```text
median_error = median(per_view_errors)
MAD = median(abs(per_view_errors - median_error))
robust_sigma = 1.4826 × MAD
threshold = median_error + mad_multiplier × robust_sigma
```

임계값을 넘는 사진 중 오차가 가장 큰 한 장만 후보로 삼고, 최대 제거 비율·최소 학습 사진 수·coverage 손실을 확인한 뒤 제거한다. 그다음 다시 최적화하며 최대 3회 반복한다. 영상 주변부를 담았다는 이유만으로 제거하지 않으며 모든 결정과 제거 전 오차를 `rejected_views.csv`에 남긴다.

per-view error는 특정 사진 하나의 자세를 다시 투영했을 때의 오차다. 코너 오검출, 흔들림, 인쇄면 휨 같은 사진별 문제를 찾는 데 유용하다.

### 4. 검증과 교차검증

전체 RMS는 최적화에 사용된 사진에 대한 적합도다. 검증 RMS는 학습에 쓰지 않은 사진에서 `K`, `D`를 고정하고 보드 자세만 구한 뒤 측정한 재투영 오차다. 후자가 새로운 자세에 대한 일반화 품질을 더 직접적으로 보여 준다.

검증에서는 평균, 중앙값, 표준편차, p90/p95/p99, 최대값과 사진별·코너별 오차를 저장한다. 중앙과 주변부, 네 모서리 영역 RMS도 분리한다. 주변부는 정규화 반경이 크고 고차 방사 계수의 영향이 강하므로 전체 평균에 묻힐 수 있는 가장자리 문제를 따로 확인해야 한다.

pose-diverse 교차검증은 fold마다 학습 사진과 검증 사진을 바꾸어 `fx`, `fy`, `cx`, `cy` 및 8개 왜곡계수의 평균·표준편차·변동률을 구한다. 변동이 크면 사진 수보다 자세 분포와 주변부 coverage를 먼저 개선한다.

### 5. Rational 수치 안정성

영상 유효 범위에서 분모를 검사한다.

```text
q(r) = 1 + k4 r² + k5 r⁴ + k6 r⁶
rd(r) = r L(r)
```

분모가 0에 가까워지는지, 부호가 바뀌는지, NaN/Inf 또는 발산이 생기는지와 수치 미분 `d rd / d r`이 0 이하가 되는지를 검사한다. 문제가 있으면 결과를 정상 성공으로 표시하지 않고 quality status와 `rational_stability.json`에 남긴다.

### 6. 최종 재학습

개발 분할 검증과 교차검증, 안정성 확인이 끝나면 최종 제외된 사진을 뺀 모든 정상 사진을 합쳐 같은 설정으로 다시 학습한다. 배포용 `K`, `D`는 이 final all-valid-images 결과를 사용하며, 개발 분할 결과와 검증 지표는 별도로 보존한다.

## 주요 결과 확인

- `calibration_result.json`: 해상도, 보드 규격, 최종 `K`, `D`, RMS, 표준편차, 입력 목록, 검증·교차검증·안정성 결과와 quality status.
- `calibration_report.md`: 사람이 읽을 수 있는 전체 요약과 경고.
- `camera_info.yaml`: ROS 2 CameraInfo 형식의 배포용 내부 파라미터.
- `detections/accepted`, `detections/rejected`: 검출 판정 오버레이.
- `coverage_heatmap.png`, `pose_descriptors.csv`: 영상 coverage와 자세 분포.
- `validation_*`: 보지 않은 사진의 재투영 통계, 히스토그램, heatmap, 오버레이.
- `cross_validation_results.csv`, `parameter_stability.json`: fold별 오차와 파라미터 안정성.
- `rational_stability.json`, `rational_radial_curve.png`: Rational 분모와 방사 매핑 검사.
- `undistorted_samples/`: `alpha=0`, `alpha=1`로 만든 육안 확인용 보정 영상.
- `comparison/undistorted_<원본명>_alpha_0p00.png`: 선택한 원본에 배포용 `K`, `D`를 적용한 보정 영상.
- `comparison/original_vs_undistorted_<원본명>_alpha_0p00.png`: 원본과 보정 영상을 좌우로 붙인 동시 비교 영상.

결과를 승인하기 전에 `quality_status`, 검증 p95/최대 오차, edge/four-corner RMS, coverage 모서리 경고, 파라미터 fold 변동 및 Rational 안정성 경고를 함께 확인한다.

### 원본과 보정 결과를 동시에 보기

캘리브레이션이 끝난 뒤 다음 명령을 실행한다.

```bash
cd ~/fire_robot_rpi
./view_calibration_result.sh
```

이 도구는 `accepted_views.txt`의 첫 번째로 존재하는 원본을 선택하고,
`camera_info.yaml`이 `rational_polynomial`인지와 왜곡계수가 정확히 8개인지 검증한 뒤
그 파일의 `K`, `D=[k1,k2,p1,p2,k3,k4,k5,k6]`를 적용한다. 원본 해상도가
캘리브레이션 해상도와 다르면 이미지를 resize하지 않고 오류로 끝낸다. GUI에서는
`ORIGINAL | UNDISTORTED`를 한 창에 표시하고, 개별 보정 PNG와 좌우 비교 PNG도 항상
`outputs/pi_camera3_wide_intrinsic/comparison`에 저장한다. 실행할 때 선택된 원본,
보정 영상, 비교 영상 및 보정계수 파일의 절대 경로가 터미널에 출력된다.

특정 원본을 비교하려면 다음처럼 지정한다.

```bash
./view_calibration_result.sh --image data/intrinsic/calib_012.png
```

SSH/헤드리스 환경에서는 GUI만 끄고 파일 생성을 유지한다.

```bash
./view_calibration_result.sh --no-display
```

기본 `alpha=0`은 검은 경계가 최소가 되도록 유효 픽셀 중심으로 보정한다. 원래 화각을
최대한 남긴 결과도 확인하려면 `--alpha 1`을 사용한다.

## ROS 2 `camera_info.yaml`

왜곡 모델 이름은 반드시 다음 값이다.

```yaml
distortion_model: rational_polynomial
```

`distortion_coefficients.data` 순서는 반드시 `[k1, k2, p1, p2, k3, k4, k5, k6]`이다. `camera_matrix`에는 원본 영상 좌표용 `K`, `distortion_coefficients`에는 원본 영상의 `D`가 들어간다. 단안 `rectification_matrix`는 3×3 identity이고, 기본 `projection_matrix`는 원본 `K`를 3×4로 확장한 형태다.

왜곡 보정 샘플을 만들 때 `cv2.getOptimalNewCameraMatrix()`가 계산한 새 행렬은 rectified 영상 좌표용이다. 원본 영상 점을 투영할 때는 원본 `K`, `D`를 사용하고, rectified 영상에 투영할 때는 그 영상 생성에 사용한 새 행렬과 좌표 정의를 사용해야 한다. 두 행렬을 혼동하거나 `camera_info.yaml`의 원본 `K`, `D`를 새 행렬로 덮어쓰면 좌표계가 맞지 않는다.

왜곡 보정 샘플에서 `alpha=0`은 유효 픽셀 중심으로 주변부 crop을 허용하고, `alpha=1`은 원래 화각을 최대한 남기는 대신 검은 영역이 생길 수 있다. 이 결과는 시각 검사 용도이며 배포용 원본 내부 파라미터를 바꾸지 않는다.

## 문제 해결

- **카메라 런타임 없음**: 저장소 루트에서 `./build_rpi_camera_runtime.sh`를 최초 한 번 실행한다.
- **IMX708 또는 `rp1-cfe` 검사 실패**: 전원을 끈 뒤 CSI 케이블 방향·체결 상태를 확인하고 다시 부팅한다. 장치 검사를 우회해 캡처하지 않는다.
- **ROS 실제 프레임 검사 실패**: 중복 카메라 프로세스를 종료하고 `camera_ros`, 카메라 런타임 및 요청 해상도를 확인한다. `/dev/video*` 존재만으로 정상 스트리밍으로 간주하지 않는다.
- **출력 폴더가 비어 있지 않음**: 기존 촬영을 이어갈 목적이고 파일 번호가 연속이면 `--resume`을 사용한다. 서로 다른 촬영 세트는 디렉터리를 섞지 않는다.
- **헤드리스 환경에서 미리보기 오류**: 자동 촬영은 `--no-preview`로 실행한다. 수동 `s` 키 촬영과 함께 사용할 수 없다.
- **입력 이미지가 없음**: `--images` glob의 기준 디렉터리와 확장자를 확인한다. 따옴표를 유지한다.
- **유효 사진 수 부족**: 전체 내부 코너가 선명하게 보이는 서로 다른 자세의 사진을 추가한다. 기본 설정의 학습 최소 수는 20장이다.
- **해상도 불일치**: 모든 입력을 같은 카메라 모드와 해상도로 다시 준비한다. 이미지를 임의 resize하면 코너의 측정 특성도 바뀔 수 있다.
- **네 모서리 coverage 부족**: 보드를 작게 만들어 중앙에 두기보다, 전체 내부 코너를 유지하면서 보드를 각 모서리 가까이 옮긴 사진을 추가한다.
- **검증 오차나 fold 변동이 큼**: blur, 보드 평탄도, 한 칸 실측값, 중복 자세와 위치·크기·기울기 분포를 확인한다.
- **Rational 안정성 실패**: 해당 결과를 배포하지 말고 네 모서리와 다양한 거리·기울기의 데이터를 보강해 처음부터 다시 실행한다.
