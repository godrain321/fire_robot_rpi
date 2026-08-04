# inno_map_tools

원본 SLAM 지도를 수정하지 않고 RViz 클릭 좌표를 기록하며, 사람이 지정한 polygon에서 no-go mask와 planning용 occupancy map을 생성하는 ROS 2 Jazzy 패키지다.

## 파일 역할

```text
~/fire_robot_rpi/maps/
├── inno_map_raw.yaml       # AMCL/localization용 원본 메타데이터
├── inno_map_raw.pgm        # 절대 수정하지 않는 원본 지도
├── clicked_points_debug.yaml
├── no_go_zones.yaml        # 사람이 작성하는 polygon
├── no_go_mask.yaml         # 자동 생성
├── no_go_mask.pgm          # 흰색=일반, 검정=no-go
├── inno_map_nav.yaml       # 자동 생성
└── inno_map_nav.pgm        # 원본+no-go planning map
```

`inno_map_raw.*`는 localization용이고 `inno_map_nav.*`는 가상 장애물을 합성한 planning용이다. 생성 도구는 원본 파일을 절대 수정하지 않으며 출력 경로가 원본과 같으면 오류로 중단한다.

## 좌표 변환

RViz `Publish Point`는 `map` frame의 미터 좌표다. PGM row 0은 이미지 위쪽이므로 y축을 반전한다.

```text
pixel_x = int((map_x - origin_x) / resolution)
pixel_y = int(image_height - 1 - ((map_y - origin_y) / resolution))

map_x = origin_x + pixel_x * resolution
map_y = origin_y + (image_height - 1 - pixel_y) * resolution
```

## 의존성 및 빌드

```bash
sudo apt-get install -y python3-yaml python3-pil

cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select inno_map_tools
source install/setup.bash
```

## 사용 흐름

### 1. 원본 지도 복사

```bash
mkdir -p ~/fire_robot_rpi/maps
cp <기존맵>.yaml ~/fire_robot_rpi/maps/inno_map_raw.yaml
cp <기존맵>.pgm  ~/fire_robot_rpi/maps/inno_map_raw.pgm
```

현재 프로젝트에서는 `inno_jazzy_ws/maps/fire_demo_20260729_182054.pgm`을 `maps/inno_map_raw.pgm`으로 복사해 사용한다.

### 2. 원본 YAML image 수정

`inno_map_raw.yaml`의 이미지 경로를 같은 디렉터리의 파일명으로 지정한다.

```yaml
image: inno_map_raw.pgm
```

### 3. map_server로 원본 지도 실행

터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$FIRE_ROBOT_RPI_ROOT/maps/inno_map_raw.yaml"
```

터미널 2:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

### 4. RViz 설정

```text
Fixed Frame: map
Add → Map
Topic: /map
Durability Policy: Transient Local
```

### 5. clicked point recorder 실행

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
ros2 launch inno_map_tools clicked_point_recorder.launch.py
```

### 6. no-go polygon 꼭짓점 클릭

RViz의 `Publish Point`로 polygon 외곽점을 시계 또는 반시계 방향으로 순서대로 클릭한다. 노드는 `/clicked_point`의 x, y, z, timestamp를 다음 파일에 append한다.

```text
~/fire_robot_rpi/maps/clicked_points_debug.yaml
```

마지막 점을 첫 점과 다시 동일하게 찍을 필요는 없다. recorder는 polygon 구분을 자동 저장하지 않으므로 polygon 하나를 다 찍고 좌표를 옮긴 뒤 다음 polygon을 찍는 방식을 권장한다.

### 7. no_go_zones.yaml 작성

`clicked_points_debug.yaml`의 x, y만 복사한다. z와 timestamp는 polygon에 넣지 않는다. 이름은 고유해야 하며 점이 최소 3개 필요하다.

```yaml
no_go_zones:
  - name: demo_block_1
    type: polygon
    points:
      - [0.50, 0.20]
      - [1.30, 0.20]
      - [1.30, 0.90]
      - [0.50, 0.90]
```

### 8. mask와 planning map 생성

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash
ros2 run inno_map_tools build_no_go_mask --map-yaml "$FIRE_ROBOT_RPI_ROOT/maps/inno_map_raw.yaml" --zones-yaml "$FIRE_ROBOT_RPI_ROOT/maps/no_go_zones.yaml" --out-dir "$FIRE_ROBOT_RPI_ROOT/maps"
```

생성 규칙:

- `no_go_mask.pgm`: 일반 영역 255, polygon 내부 0
- `inno_map_nav.pgm`: 원본을 복사하고 mask가 0인 픽셀만 0으로 변경
- 출력 YAML의 resolution, origin, negate, threshold, mode는 원본에서 복사
- 지도 밖 꼭짓점은 경고하고 이미지와 겹치는 범위만 처리

### 9. planning map 실행

기존 `/map_server`를 종료한 뒤 실행한다.

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$FIRE_ROBOT_RPI_ROOT/maps/inno_map_nav.yaml"
```

다른 터미널:

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

### 10. RViz에서 확인

RViz `Map` display의 `/map`에서 no-go polygon이 검정 occupied 영역으로 채워졌는지 원본과 비교한다.

## 주의사항

- AMCL/localization에는 원본 `inno_map_raw.yaml`을 사용하는 것이 안전하다.
- planning/A*에는 `inno_map_nav.yaml` 또는 `no_go_mask`를 사용한다.
- 원본 map을 직접 칠하거나 덮어쓰지 않는다.
- no-go 영역은 LiDAR에 보이지 않는 가상 장애물이라 localization map에 넣으면 AMCL이 혼동할 수 있다.
- 원본과 planning map을 분리하고 실제 Nav2 통합에서는 localization map 토픽과 global costmap 또는 keepout filter mask를 분리한다.
- `/map` 하나를 planning map으로 교체하면 AMCL도 그 지도를 받을 수 있으므로 토픽과 namespace를 명확히 설계해야 한다.
- `clicked_points_debug.yaml`은 좌표 기록일 뿐 그 자체로 장애물이 아니다.

## 명령 확인

```bash
ros2 run inno_map_tools build_no_go_mask --help
ros2 launch inno_map_tools clicked_point_recorder.launch.py --show-args
```
