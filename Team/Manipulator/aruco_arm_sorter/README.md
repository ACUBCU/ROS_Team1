# OpenMANIPULATOR-X ArUco 자동 분류

이 ROS 2 패키지는 ROBOTIS OpenMANIPULATOR-X의 공식 모델을 Gazebo Harmonic에 불러오고, `link5` 카메라 영상에서 ArUco ID 0·1을 자동 인식해 두 분류 동작을 실행합니다.

- 공식 `open_manipulator_description`의 Xacro·메시·관절 제한 사용
- `gz_ros2_control/GazeboSimSystem` 사용
- `arm_controller`와 `gripper_controller` Action 사용
- MoveIt 2 미사용
- `link5` 아래 방향 640×480 카메라
- OpenCV `DICT_4X4_50` 검출
- `/arm/status`가 `READY`일 때 같은 ID가 5프레임 연속 검출되면 한 번 발행
- Gazebo `DetachableJoint`로 박스 운반 재현

## 1. 반드시 사용할 저장소 위치

```text
ROS_Team1/Team/Manipulator/aruco_arm_sorter
```

이 패키지를 다른 ROS 작업공간으로 복사하거나 심볼릭 링크로 연결하지 않습니다. 같은 이름의 구버전 패키지가 다른 작업공간에 남아 있어도 괜찮지만, 실행 시 현재 저장소의 설치본이 선택되어야 합니다.

`Team/Manipulator/workspace.sh`는 현재 파일 위치에서 저장소 루트를 계산하므로 Linux 사용자명이나 저장소의 절대경로를 코드에 넣지 않습니다.

## 2. 빌드·점검·실행

팀 저장소 루트에서 실행합니다.

```bash
./Team/Manipulator/workspace.sh build
./Team/Manipulator/workspace.sh doctor
./Team/Manipulator/workspace.sh run
```

각 명령의 역할은 다음과 같습니다.

| 명령 | 역할 |
|---|---|
| `build` | 이 저장소의 `aruco_arm_sorter`만 지정해 기존 생성물을 정리하고 다시 빌드 |
| `doctor` | 활성 prefix, 세 실행 파일, 프로젝트 식별 정보, ROS·Gazebo·OpenCV 의존성 검사 |
| `run` | `doctor` 통과 후 Gazebo와 두 자동 노드 실행 |
| `status` | 현재 선택된 패키지와 계산된 저장소 경로 표시 |

공식 OpenMANIPULATOR 패키지가 별도 작업공간에 있으면 도우미가 사용 가능한 `install/setup.bash`를 찾습니다. 자동 탐색에 실패할 때만 경로를 지정합니다.

```bash
ARUCO_DEPENDENCY_SETUP="/의존성/워크스페이스/install/setup.bash" \
  ./Team/Manipulator/workspace.sh build
```

## 3. 기존 경로 문제를 막는 방법

ROS 2는 같은 이름의 패키지가 여러 overlay에 있으면 `setup.bash`를 적용한 순서에 따라 하나를 선택합니다. 그래서 소스에는 `aruco_detector`가 있어도 다른 작업공간의 오래된 설치본이 활성화되면 다음 현상이 생길 수 있습니다.

```text
ros2 run aruco_arm_sorter aruco_detector
No executable found
```

이번 수정본은 다음을 함께 검사합니다.

1. 패키지 prefix가 현재 저장소의 `install/aruco_arm_sorter`인지
2. `arm_sequence_controller`, `aruco_detector`, `preflight`가 모두 설치됐는지
3. 설치본에 `ACUBCU/ROS_Team1` 구조를 나타내는 `config/project.yaml`이 있는지
4. 동명 패키지가 다른 `AMENT_PREFIX_PATH`에 남아 있는지
5. 런치가 기대 prefix와 다르면 Gazebo 시작 전에 중단하는지

따라서 수동으로 여러 `source` 명령의 순서를 맞추는 대신 `workspace.sh run`을 사용합니다.

## 4. 자동 실행 흐름

```text
Gazebo + OpenMANIPULATOR-X
→ link5 카메라 /gripper_camera/image_raw
→ aruco_detector
→ /detected_marker_id
→ arm_sequence_controller
→ 집기·운반·놓기
→ 관찰 자세 복귀
```

런치 후 로봇팔은 관찰 자세로 이동합니다. 카메라가 지원 ID를 5프레임 연속 확인하면 별도의 수동 토픽 발행 없이 동작을 시작합니다.

새 터미널에서 상태만 확인할 때도 먼저 현재 저장소 환경을 확인합니다.

```bash
./Team/Manipulator/workspace.sh status
```

런치 터미널은 유지하고, ROS 환경이 적용된 다른 터미널에서 다음을 확인할 수 있습니다.

```bash
ros2 node list | grep -E 'aruco_detector|arm_sequence_controller'
ros2 topic hz /gripper_camera/image_raw
ros2 topic echo /gripper_camera/camera_info --once
ros2 topic echo /arm/status
```

검출 표시 영상은 `/aruco/detection_image`로 발행됩니다.

## 5. 수동 동작 시험

카메라를 제외한 제어 흐름만 확인할 때 사용합니다. 이미 처리한 ID는 중복 실행하지 않습니다.

```bash
ros2 topic pub --once /detected_marker_id std_msgs/msg/Int32 "{data: 0}"
ros2 topic pub --once /detected_marker_id std_msgs/msg/Int32 "{data: 1}"
```

## 6. 동작 수정 위치

저장소 루트 기준 파일입니다.

```text
Team/Manipulator/aruco_arm_sorter/config/motions.yaml
```

각 자세는 공식 관절 이름 `joint1`~`joint4`를 사용합니다. `observation`과 각 sequence의 마지막 자세는 같아야 다음 마커를 다시 볼 수 있습니다.

현재 구현은 카메라로 마커 ID를 구분하지만 박스 좌표는 `sorting_world.sdf`와 `motions.yaml`에 고정되어 있습니다. 임의 위치의 박스를 찾아가는 기능에는 카메라 보정, 마커 자세 추정과 TF 좌표 변환이 추가로 필요합니다.

## 7. 실제 하드웨어와의 관계

이 런치는 `use_sim=true`인 Gazebo 전용입니다. 실제 OpenMANIPULATOR-X는 ROBOTIS bringup의 Dynamixel 하드웨어를 사용합니다. Action 이름은 재사용할 수 있지만 실제 장치에서는 별도 bringup과 저속·소범위 안전 검증이 필요합니다.

## 8. 확인 기준

- [ROBOTIS OpenMANIPULATOR Jazzy 소스](https://github.com/ROBOTIS-GIT/open_manipulator/tree/jazzy)
- [OpenMANIPULATOR-X 공식 문서](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/)
- [Jazzy `gz_ros2_control`](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
- [Jazzy `joint_trajectory_controller`](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
- [Gazebo `DetachableJoint`](https://gazebosim.org/api/sim/8/detachablejoints.html)
- [Gazebo Sensors와 ROS 2 브리지](https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/)
- [OpenCV ArUco 검출](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
