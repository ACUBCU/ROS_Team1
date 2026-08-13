# OpenMANIPULATOR-X ArUco 분류 프로젝트

이 수정본은 자체 제작 막대형 로봇팔을 제거하고, 사용 중인 **ROBOTIS OpenMANIPULATOR-X**의 공식 ROS 2 모델을 Gazebo Harmonic에 불러옵니다.

- 공식 `open_manipulator_description`의 STL 메시·URDF·관절 제한 사용
- `gz_ros2_control/GazeboSimSystem` 사용
- `joint1`~`joint4`: `joint_trajectory_controller`
- 그리퍼: `gripper_controller`
- MoveIt 2 미사용
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

## 2. 필요한 공식 패키지 확인

새 터미널에서 먼저 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix open_manipulator_description
ros2 pkg prefix open_manipulator_bringup
```

두 경로가 출력되면 이미 수업에서 설치한 공식 패키지를 사용할 수 있습니다. 해당 패키지가 별도 작업공간에 있다면 이 프로젝트를 빌드하기 전에 그 작업공간의 `install/setup.bash`도 source해야 합니다.

패키지를 찾지 못하는 경우, `~/Ros/project`를 작업공간으로 사용할 때 다음처럼 공식 Jazzy 브랜치를 추가할 수 있습니다.

```bash
cd ~/Ros/project/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git

cd ~/Ros/project
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 3. 기존 잘못된 패키지 교체

압축을 풀면 다음 폴더가 있습니다.

```text
aruco_openmanipulator_sorter_project/src/aruco_arm_sorter
```

기존 `~/Ros/project/src/aruco_arm_sorter` 대신 이 폴더 전체를 사용합니다. 소스 원본을 교체한 뒤 이전 빌드 결과를 지우고 다시 빌드합니다.

```bash
cd ~/Ros/project
source /opt/ros/jazzy/setup.bash

# 공식 OpenMANIPULATOR가 다른 작업공간에 있다면 그 setup.bash도 여기서 source

rm -rf build/aruco_arm_sorter install/aruco_arm_sorter
colcon build --symlink-install --packages-select aruco_arm_sorter
source install/setup.bash
```

`rm` 대상은 자동 생성된 해당 패키지의 빌드 결과뿐이며 `src` 원본은 삭제하지 않습니다.

## 4. 사전 점검과 실행

터미널 1:

```bash
cd ~/Ros/project
source /opt/ros/jazzy/setup.bash
# 필요한 경우 공식 OpenMANIPULATOR 작업공간도 source
source install/setup.bash

ros2 run aruco_arm_sorter preflight
ros2 launch aruco_arm_sorter aruco_arm_sorter.launch.py
```

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

## 5. 마커 ID 시험

Gazebo와 launch를 실행한 상태에서 터미널 2를 엽니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/Ros/project/install/setup.bash

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

## 6. 동작 수정

수정 파일:

```text
~/Ros/project/src/aruco_arm_sorter/config/motions.yaml
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

기본 자세는 현재 월드의 박스 위치에 맞춰 계산했습니다. 실제 과제 배치를 바꾸면 Gazebo에서 안전하게 자세를 확인한 뒤 YAML 값과 박스 좌표를 함께 수정해야 합니다.

## 7. 실제 하드웨어와의 관계

이 launch는 `use_sim=true`이므로 Gazebo용 `GazeboSimSystem`을 사용합니다. 실제 OpenMANIPULATOR-X는 ROBOTIS bringup에서 `DynamixelHardware`를 사용합니다. 상위 인터페이스는 둘 다 다음 Action이므로 동작 YAML과 제어 노드 구조를 재사용할 수 있습니다.

```text
/arm_controller/follow_joint_trajectory
/gripper_controller/gripper_cmd
```

다만 이 프로젝트의 자동 실행 launch는 안전상 Gazebo 전용입니다. 실제 장치에 적용할 때는 ROBOTIS의 실기기 bringup을 별도로 실행하고, 충돌하지 않는 낮은 속도의 작은 동작부터 검증해야 합니다.

## 8. 근거

- ROBOTIS 공식 Jazzy 패키지: <https://github.com/ROBOTIS-GIT/open_manipulator/tree/jazzy>
- OpenMANIPULATOR-X 공식 문서: <https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/>
- Jazzy `gz_ros2_control`: <https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html>
- Jazzy `joint_trajectory_controller`: <https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>
- Gazebo `DetachableJoint`: <https://gazebosim.org/api/sim/8/detachablejoints.html>
