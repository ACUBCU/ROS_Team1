# OpenMANIPULATOR-X ArUco 분류 프로젝트

이 패키지는 **ROBOTIS OpenMANIPULATOR-X**의 공식 ROS 2 모델을 Gazebo Harmonic에 불러오고, `link5` 카메라 영상에서 ArUco ID 0·1을 자동 인식해 분류 동작을 실행합니다.

- 공식 `open_manipulator_description`의 STL 메시·URDF·관절 제한 사용
- `gz_ros2_control/GazeboSimSystem` 사용
- `joint1`~`joint4`: `joint_trajectory_controller`
- 그리퍼: `gripper_controller`
- MoveIt 2 미사용
- `link5`에 아래를 향하는 640×480 Gazebo 카메라 장착
- OpenCV `DICT_4X4_50`으로 ArUco ID 0·1 자동 검출
- `/arm/status`가 `READY`일 때 5프레임 연속 검출된 ID만 한 번 발행
- `/detected_marker_id`의 `0`, `1`에 따라 YAML 동작 A/B 실행
- 박스 운반은 Gazebo `DetachableJoint`로 안정적으로 재현

## 1. 이전 프로젝트와 달라진 점

| 항목 | 잘못된 이전 버전 | 이 수정본 |
| --- | --- | --- |
| Gazebo 외형 | 자체 제작 3축 막대 모델 | 공식 OpenMANIPULATOR-X STL 모델 |
| 팔 관절 | `base_yaw`, `shoulder`, `elbow` | `joint1`, `joint2`, `joint3`, `joint4` |
| 제어 | Gazebo `JointPositionController` 토픽 | ROS 2 `JointTrajectoryController` Action |
| 그리퍼 | 자체 제작 좌우 prismatic joint | 공식 OpenMANIPULATOR-X 그리퍼 |
| MoveIt 2 | 사용 안 함 | 사용 안 함 |
| 마커 입력 | 수동 토픽 발행 | `link5` 카메라와 OpenCV로 자동 발행 |

## 2. 필요한 공식 패키지 확인

새 터미널에서 먼저 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix open_manipulator_description
ros2 pkg prefix open_manipulator_bringup
```

두 경로가 출력되면 이미 수업에서 설치한 공식 패키지를 사용할 수 있습니다. 해당 패키지가 별도 작업공간에 있다면 이 프로젝트를 빌드하기 전에 그 작업공간의 `install/setup.bash`도 source해야 합니다.

패키지를 찾지 못하면 공식 OpenMANIPULATOR Jazzy 소스를 별도의 의존성 작업공간에 설치해야 합니다. 이 의존성 작업공간은 `ROS_Team1` 프로젝트 자체와 구분합니다.

```bash
mkdir -p ~/Ros/open_manipulator_ws/src
cd ~/Ros/open_manipulator_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git

cd ~/Ros/open_manipulator_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

위 경로는 공식 의존성의 예시일 뿐입니다. 이미 `~/Ros/ws` 등에서 공식 OpenMANIPULATOR 패키지를 정상적으로 빌드했다면 새로 복제하지 않고 그 작업공간의 `install/setup.bash`를 사용합니다.

## 3. `ROS_Team1`에서 직접 빌드

팀 저장소에서 사용할 패키지는 다음 폴더입니다.

```text
ROS_Team1/Team/Manipulator/aruco_arm_sorter
```

패키지를 `~/Ros/move_arm` 같은 다른 작업공간으로 복사하거나 심볼릭 링크로 연결하지 않습니다. 팀 저장소 루트 `~/ROS_Team1`에서 다음 순서로 직접 빌드합니다.

```bash
cd ~/ROS_Team1
source /opt/ros/jazzy/setup.bash

# 공식 OpenMANIPULATOR가 ~/Ros/ws에 설치된 경우에만 source
source ~/Ros/ws/install/setup.bash

# 팀 저장소 안의 현재 패키지를 직접 의존성 검사·빌드
rosdep install \
  --from-paths Team/Manipulator/aruco_arm_sorter \
  --ignore-src -r -y
rm -rf build/aruco_arm_sorter install/aruco_arm_sorter
colcon build \
  --symlink-install \
  --base-paths Team/Manipulator/aruco_arm_sorter \
  --packages-select aruco_arm_sorter

# 다른 작업공간보다 ROS_Team1을 마지막에 source
source ~/ROS_Team1/install/setup.bash
```

`rm` 대상은 자동 생성된 해당 패키지의 빌드 결과뿐이며 `src` 원본은 삭제하지 않습니다.

빌드 직후 현재 터미널이 실제로 `ROS_Team1` 설치본을 선택했는지 확인합니다.

```bash
ros2 pkg prefix aruco_arm_sorter
```

정상 결과:

```text
/home/사용자이름/ROS_Team1/install/aruco_arm_sorter
```

`/home/사용자이름/Ros/move_arm/install/...`처럼 다른 작업공간이 나오면 실행하지 않습니다. 새 터미널을 열고 `/opt/ros/jazzy`, 공식 의존성 작업공간, `~/ROS_Team1/install` 순서로 다시 source합니다.

## 4. 사전 점검과 실행

터미널 1:

```bash
cd ~/ROS_Team1
source /opt/ros/jazzy/setup.bash
# 필요한 경우 공식 OpenMANIPULATOR 의존성 작업공간을 먼저 source
source ~/Ros/ws/install/setup.bash
# 프로젝트 설치본은 반드시 마지막에 source
source ~/ROS_Team1/install/setup.bash

ros2 pkg prefix aruco_arm_sorter

ARUCO_ARM_SORTER_EXPECTED_PREFIX="$HOME/ROS_Team1/install/aruco_arm_sorter" \
  ros2 run aruco_arm_sorter preflight
ros2 launch aruco_arm_sorter aruco_arm_sorter.launch.py
```

`~/Ros/ws`를 사용하지 않는 환경에서는 그 source 줄만 생략합니다. `preflight`는 현재 활성 패키지 위치가 `ROS_Team1`인지, 설치된 world에 Gazebo Sensors 시스템이 있는지, 영상·CameraInfo 브리지가 설정됐는지를 함께 검사합니다.

정상 실행 확인:

```bash
ros2 control list_controllers
ros2 action list -t
```

예상 controller:

```text
arm_controller           .../JointTrajectoryController  active
gripper_controller       .../GripperActionController    active
joint_state_broadcaster  .../JointStateBroadcaster      active
```

예상 Action:

```text
/arm_controller/follow_joint_trajectory [control_msgs/action/FollowJointTrajectory]
/gripper_controller/gripper_cmd [control_msgs/action/GripperCommand]
```

## 5. 카메라와 자동 인식 확인

launch가 시작되면 로봇팔은 자동으로 마커 관찰 자세로 이동합니다. 카메라 원점의 초기 계산값은 약 `(x=0.246, y=0.000, z=0.330) m`이고, 두 박스의 중앙을 내려다봅니다.

새 터미널에서 영상과 카메라 정보를 확인합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Ros/ws/install/setup.bash  # 필요한 경우에만
source ~/ROS_Team1/install/setup.bash

ros2 topic hz /gripper_camera/image_raw
ros2 topic echo /gripper_camera/camera_info --once
ros2 topic echo /detected_marker_id
```

검출 결과가 그려진 영상은 다음 토픽으로 발행됩니다.

```text
/aruco/detection_image
```

`rqt_image_view`에서 이 토픽을 선택하면 검출 ID, 연속 확인 횟수와 이미 발행한 ID를 볼 수 있습니다. ID가 5프레임 연속 확인되면 `/detected_marker_id`로 한 번만 전달되고 기존 집기 동작이 시작됩니다.

전체 처리 순서:

```text
Gazebo link5 카메라
-> /gripper_camera/image_raw
-> aruco_detector
-> /detected_marker_id
-> arm_sequence_controller
-> 집기/운반
-> 마커 관찰 자세 복귀
```

## 6. 수동 마커 ID 시험

카메라 검출과 별개로 기존 제어 동작만 확인할 때 사용할 수 있습니다. 이미 자동 처리된 ID는 제어 노드에서 중복 실행하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Ros/ws/install/setup.bash  # 필요한 경우에만
source ~/ROS_Team1/install/setup.bash

ros2 topic pub --once /detected_marker_id std_msgs/msg/Int32 "{data: 0}"
```

ID 1:

```bash
ros2 topic pub --once /detected_marker_id std_msgs/msg/Int32 "{data: 1}"
```

상태 확인:

```bash
ros2 topic echo /arm/status
```

## 7. 동작과 관찰 자세 수정

수정 파일:

```text
~/ROS_Team1/Team/Manipulator/aruco_arm_sorter/config/motions.yaml
```

각 자세는 공식 관절 이름을 사용합니다.

```yaml
positions:
  joint1: 0.45
  joint2: 0.70
  joint3: 0.60
  joint4: -1.30
gripper: 0.019
duration: 1.5
```

`observation`은 카메라가 두 마커를 보는 자세입니다. 각 분류 sequence의 마지막 자세도 같은 값이어야 다음 마커를 인식할 수 있습니다.

기본 자세는 현재 월드의 박스 위치에 맞춰 계산했습니다. 실제 과제 배치를 바꾸면 Gazebo에서 카메라 영상과 로봇의 충돌 여부를 확인한 뒤 관찰 자세, 동작 YAML과 박스 좌표를 함께 수정해야 합니다.

## 8. 현재 인식 범위와 실제 하드웨어의 관계

현재 구현은 카메라로 **ID를 구분하는 1단계**입니다. 박스 좌표는 `sorting_world.sdf`와 `motions.yaml`에 고정되어 있으므로, 박스를 임의의 위치로 옮겼을 때 카메라에서 계산한 위치로 찾아가는 기능은 아직 포함하지 않습니다. 그 기능에는 카메라 보정, 마커 자세 추정과 TF 좌표 변환이 추가로 필요합니다.

이 launch는 `use_sim=true`이므로 Gazebo용 `GazeboSimSystem`을 사용합니다. 실제 OpenMANIPULATOR-X는 ROBOTIS bringup에서 `DynamixelHardware`를 사용합니다. 상위 인터페이스는 둘 다 다음 Action이므로 동작 YAML과 제어 노드 구조를 재사용할 수 있습니다.

```text
/arm_controller/follow_joint_trajectory
/gripper_controller/gripper_cmd
```

다만 이 프로젝트의 자동 실행 launch는 안전상 Gazebo 전용입니다. 실제 장치에 적용할 때는 ROBOTIS의 실기기 bringup을 별도로 실행하고, 충돌하지 않는 낮은 속도의 작은 동작부터 검증해야 합니다.

## 9. 근거

- ROBOTIS 공식 Jazzy 패키지: <https://github.com/ROBOTIS-GIT/open_manipulator/tree/jazzy>
- OpenMANIPULATOR-X 공식 문서: <https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/>
- Jazzy `gz_ros2_control`: <https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html>
- Jazzy `joint_trajectory_controller`: <https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>
- Gazebo `DetachableJoint`: <https://gazebosim.org/api/sim/8/detachablejoints.html>
- Gazebo Sensors와 ROS 2 브리지: <https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/>
- OpenCV ArUco 검출: <https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html>
