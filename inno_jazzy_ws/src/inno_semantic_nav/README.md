# inno_semantic_nav

## 목적

`inno_semantic_nav`는 Ubuntu 24.04와 ROS 2 Jazzy에서 이름 있는 이동 pose와
표시용 landmark를 관리한다. RViz 기본 도구로 `E1`, `E2`, `MACHINE_01` 같은
지점을 찍어 YAML에 저장하고, Marker로 재표시하며, 이름을 `PoseStamped` 또는
Nav2 `NavigateToPose` 목표로 변환한다.

이 Jazzy 패키지는 시스템 Python 3.12, `rclpy`, Jazzy의
`nav2_msgs/action/NavigateToPose`, `nav2_map_server`, RViz Goal/Publish Point 도구를
기준으로 배치했다. Conda나 별도 Python 가상환경 대신 Ubuntu 시스템 Python을
사용한다.

현재 지도와 waypoint는 `inno_jazzy_ws/maps/latest_map.yaml` 및
`inno_jazzy_ws/maps/semantic_points.yaml`에 있다. 다른 SLAM 지도는 launch의
`map` 인자만 바꿔 사용할 수 있다.

## 지원 환경

- Ubuntu 24.04 LTS (amd64 또는 Raspberry Pi 5 arm64)
- ROS 2 Jazzy
- Python 3.12 시스템 인터프리터
- Nav2, RViz2, `python3-yaml`

## 패키지 구조

```text
inno_semantic_nav/
├── inno_semantic_nav/
│   ├── capture_named_pose.py   # RViz 2D Goal Pose를 이름 있는 pose로 저장
│   ├── capture_landmark.py     # RViz Publish Point를 표시용 landmark로 저장
│   ├── semantic_store.py       # YAML 검증 및 원자적 저장
│   ├── semantic_marker_node.py # YAML을 MarkerArray로 계속 표시
│   ├── go_named_pose.py        # 이름을 PoseStamped/Nav2 목표로 변환
│   └── geometry_utils.py       # yaw와 quaternion 변환
├── launch/
│   └── semantic_map_editor.launch.py
├── rviz/
│   └── semantic_map_editor.rviz
├── config/
│   └── semantic_points.example.yaml
└── test/
```

## 작동 방식

웨이포인트 저장 흐름:

```text
RViz 2D Goal Pose
       │  /goal_pose (geometry_msgs/PoseStamped)
       ▼
capture_named_pose
       │  x, y, quaternion → yaw
       ▼
SemanticStore ──원자적 저장──▶ semantic_points.yaml
                                      │ 파일 변경 감시
                                      ▼
                              semantic_marker_node
                                      │ /semantic_markers
                                      ▼
                                    RViz
```

이름으로 이동하는 흐름:

```text
ros2 run ... go E1
       │ 이름으로 YAML 조회
       ├────────▶ /named_goal_pose (RViz 확인용)
       └────────▶ /navigate_to_pose (Nav2 action, 실제 이동)
```

`capture_named_pose`는 RViz가 발행한 첫 `/goal_pose`만 저장하고 종료한다.
`semantic_marker_node`는 기본 0.5초 주기로 YAML 변경을 감지한다. `go --dry-run`은
좌표와 방향을 확인하되 Nav2 action을 보내지 않으며, `--dry-run`이 없으면 Jazzy
Nav2의 `NavigateToPose` 서버로 목표를 전송한다.

## PGM에 라벨을 그리지 않는 이유

PGM 픽셀은 `nav2_map_server`가 free/occupied/unknown으로 해석하는 내비게이션 원본이다. 여기에 `E1` 같은 글자를 그리면 글자 픽셀이 장애물 또는 미지 영역으로 바뀌어 경로 계획 결과를 훼손한다. 따라서 PGM은 변경하지 않고, 이름·분류·설명·map-frame 미터 좌표는 별도의 `semantic_points.yaml`에 저장한다. RViz 라벨은 `/semantic_markers` 오버레이로만 표시한다.

## semantic_points.yaml

```yaml
version: 1
site_id: test_map
frame_id: map

poses:
  E1:
    category: exit
    description: main_exit
    x: 1.2
    y: -0.4
    yaw: 1.57

landmarks:
  MACHINE_01:
    category: machine
    description: fixed_factory_machine
    x: 0.5
    y: 0.8
```

`poses`는 이동 가능한 `x`, `y`, `yaw`를 가지며 `go`에서 사용할 수 있다. `landmarks`는 위치 표시 전용이라 `go` 목표가 될 수 없다. 이름에는 영문, 숫자, `_`, `-`만 허용된다. 파일이 없으면 `version`, `site_id`, `frame_id`, 빈 `poses`, 빈 `landmarks`를 자동 생성한다. 저장은 같은 디렉터리의 임시 파일에 쓴 뒤 `os.replace()`로 교체하며, 알 수 없는 최상위 키는 보존한다.

예제 전체는 설치 전에는 `config/semantic_points.example.yaml`, 설치 후에는 `share/inno_semantic_nav/config/semantic_points.example.yaml`에 있다.

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd ~/fire_robot_rpi/inno_jazzy_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select inno_semantic_nav
source install/setup.bash
```

## 지도 편집 모드

터미널 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash

ros2 launch inno_semantic_nav semantic_map_editor.launch.py \
  map:=~/fire_robot_rpi/inno_jazzy_ws/maps/latest_map.yaml \
  semantic_file:=~/fire_robot_rpi/inno_jazzy_ws/maps/semantic_points.yaml
```

이 launch는 `map_server`, map_server용 lifecycle manager, `semantic_marker_node`, RViz를 실행한다. RViz Fixed Frame은 `map`이고 다음 display와 기본 도구가 미리 설정돼 있다.

- Map: `/map`
- MarkerArray: `/semantic_markers`
- Pose: `/named_goal_pose`
- 2D Goal Pose 출력: `/goal_pose`
- Publish Point 출력: `/clicked_point`

GUI 없이 map server와 marker만 점검하려면 `start_rviz:=false`를 추가한다. 시뮬레이션 clock을 사용할 때만 `use_sim_time:=true`를 준다. 다른 RViz 설정은 `rviz_config:=/absolute/path/editor.rviz`로 지정할 수 있다.

## E1과 E2 저장

터미널 2에서 E1 capture를 먼저 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash

ros2 run inno_semantic_nav capture_named_pose E1 \
  --semantic-file ~/fire_robot_rpi/inno_jazzy_ws/maps/semantic_points.yaml \
  --category exit \
  --description main_exit
```

그 후 RViz 상단의 **2D Goal Pose**를 누르고 지도에서 위치를 누른 채 드래그하여 방향을 정한다. 첫 `/goal_pose`를 받으면 `x`, `y`, quaternion에서 계산한 정규화 yaw를 저장하고 종료한다.

E2도 같은 방식으로 저장한다.

```bash
ros2 run inno_semantic_nav capture_named_pose E2 \
  --semantic-file ~/fire_robot_rpi/inno_jazzy_ws/maps/semantic_points.yaml \
  --category exit \
  --description emergency_exit
```

기본 대기시간은 120초다. 예를 들어 `--timeout 30`으로 바꿀 수 있다. 같은 이름은 기본적으로 거부하며 의도적으로 다시 찍을 때만 `--overwrite`를 붙인다. Ctrl+C로 끝내면 수신 전 파일을 수정하지 않는다.

## 공장 기계 landmark 저장

```bash
ros2 run inno_semantic_nav capture_landmark MACHINE_01 \
  --semantic-file ~/fire_robot_rpi/inno_jazzy_ws/maps/semantic_points.yaml \
  --category machine \
  --description fixed_factory_machine
```

명령이 기다리는 동안 RViz의 **Publish Point**를 선택하고 기계 위치를 한 번 클릭한다. landmark에는 방향이 없고 NavigateToPose 목표로 사용되지 않는다. 중복, `--timeout`, `--overwrite`, Ctrl+C 정책은 pose capture와 같다.

## Marker 확인

저장 후 최대 약 0.5초 안에 marker node가 YAML 변경을 감지한다. pose는 위치 원통, yaw 화살표, `E1 [exit]` 형식의 텍스트로 보인다. landmark는 위치 큐브와 텍스트만 보인다. 이름 삭제·변경 때는 `DELETEALL` 뒤 전체 marker를 다시 발행해 RViz 잔상을 없앤다. YAML 편집 중 문법이 잠시 깨지면 마지막 정상 marker를 유지하고 오류를 출력하며, 다음 파일 변경 때 다시 읽는다.

## go E1 dry-run

터미널 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_robot_rpi/inno_jazzy_ws/install/setup.bash

ros2 run inno_semantic_nav go E1 \
  --semantic-file ~/fire_robot_rpi/inno_jazzy_ws/maps/semantic_points.yaml \
  --dry-run
```

터미널에 frame, timestamp, 위치, 정규화 quaternion, yaw가 출력된다. 동시에 `/named_goal_pose`에 `PoseStamped`를 발행하므로 실행 중인 RViz의 **Named Goal Pose** display에서 화살표를 확인할 수 있다. dry-run은 Nav2, AMCL, 모터가 없어도 된다.

## 실제 Nav2로 go E1

localization, TF, planner/controller를 포함한 Nav2가 이미 실행 중일 때:

```bash
ros2 run inno_semantic_nav go E1 \
  --semantic-file ~/fire_robot_rpi/inno_jazzy_ws/maps/semantic_points.yaml
```

기본 액션은 `/navigate_to_pose`, 서버 대기는 5초다. 필요하면
`--action-name /robot1/navigate_to_pose`와 `--server-timeout 10`을 사용한다.
이 도구는 Jazzy의 `rclpy.action.ActionClient`로 목표를 보내고 NavigateToPose
feedback과 Jazzy 결과의 `error_code`, `error_msg`를 출력한다. Ctrl+C를 누르면
수락된 goal의 취소를 시도한다.

## 새 지도 교체

실제 시연 공간마다 다음 세트를 한 디렉터리에서 함께 관리하는 것을 권장한다.

- 새 `map.pgm`
- 그 이미지를 가리키는 `map.yaml`
- 그 지도 위에서 다시 클릭해 만든 `semantic_points.yaml`

```bash
ros2 launch inno_semantic_nav semantic_map_editor.launch.py \
  map:=/path/to/factory_4/map.yaml \
  semantic_file:=/path/to/factory_4/semantic_points.yaml
```

코드에는 `inno_map`이나 E1 좌표가 들어 있지 않다. 지도마다 resolution과 origin이 다르므로 이전 지도의 semantic 좌표를 복사하지 말고 새 지도에서 다시 찍는다. 저장값은 이미지 픽셀이 아니라 `map` frame의 미터 좌표다.

## 자주 발생하는 오류

- **map YAML/이미지가 없다**: launch 오류에 출력된 절대경로를 확인한다. map YAML의 상대 `image` 경로는 map YAML 디렉터리 기준이다.
- **semantic 상위 디렉터리가 없다**: 디렉터리 경로 오타를 고친다. 파일 자체는 없어도 자동 생성된다.
- **RViz 클릭을 capture가 못 받는다**: capture 명령을 먼저 실행한 후 올바른 기본 도구를 누르고, Fixed Frame과 메시지 frame이 `map`인지 확인한다.
- **이름이 이미 존재한다**: 다른 이름을 쓰거나 의도한 경우에만 `--overwrite`를 사용한다.
- **landmark는 이동할 수 없다는 오류**: 해당 이름을 `poses` 아래에 별도의 방향과 함께 저장해야 한다.
- **NavigateToPose 서버가 없다**: 전체 Nav2가 아직 없다면 `--dry-run`을 사용한다.
- **Marker가 갱신되지 않는다**: marker node 로그의 YAML 문법 오류를 고치고 파일을 다시 저장한다. 마지막 정상 marker는 의도적으로 유지된다.
- **`Package not found`**: Jazzy와 `inno_jazzy_ws/install/setup.bash`를 현재 터미널에서 다시 source한다.

## 테스트

```bash
source /opt/ros/jazzy/setup.bash
cd ~/fire_robot_rpi/inno_jazzy_ws
colcon test --packages-select inno_semantic_nav
colcon test-result --verbose
```

## Ubuntu 24.04 / Jazzy 실행 전 확인

```bash
lsb_release -rs                 # 24.04
echo "$ROS_DISTRO"              # jazzy
python3 --version               # Python 3.12.x
ros2 pkg executables inno_semantic_nav
```

실제 이동에는 waypoint 도구 외에도 `map -> odom -> base_link` TF, AMCL 또는 다른
localization, Nav2 planner/controller, 올바른 footprint와 costmap 설정이 필요하다.
지도와 좌표만 검증할 때는 `go ... --dry-run`을 사용한다.
