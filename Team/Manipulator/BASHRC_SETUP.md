# ROS_Team1용 `.bashrc` 적용

`shell/bashrc_ROS_Team1`은 사용자가 제공한 Ubuntu 기본 `.bashrc`를 정리하고, 이 저장소를 `~/ROS_Team1`에서 사용하는 기준으로 수정한 전체 파일입니다.

## 변경 이유

- `~/Ros/move_arm/install/setup.bash`를 터미널마다 자동으로 불러오지 않습니다. 이 위치의 구버전 `aruco_arm_sorter`가 새 설치본보다 먼저 선택되는 문제를 막습니다.
- `~/ROS_Team1/install/setup.bash`가 존재하면 가장 마지막 overlay로 불러옵니다.
- 이미 열린 터미널에 남아 있는 `move_arm`의 `AMENT_PREFIX_PATH`, `PYTHONPATH`, `LD_LIBRARY_PATH`, 실행 경로와 Gazebo 경로도 먼저 제거합니다.
- 공식 OpenMANIPULATOR 의존성용 `~/Ros/open_manipulator_ws`는 전역으로 source하지 않고 `workspace.sh`가 필요할 때만 사용합니다.
- Gazebo 모델 경로는 런치 파일이 설치된 package share를 기준으로 설정하므로 `.bashrc`에 소스 폴더 경로를 중복 추가하지 않습니다.
- `cb`, `team1doctor`, `team1run`, `team1status`가 모두 `workspace.sh`를 사용합니다.

## 안전하게 적용

저장소가 다음 위치에 있는지 먼저 확인합니다.

```bash
test -f ~/ROS_Team1/Team/Manipulator/workspace.sh && echo OK
```

현재 `.bashrc`를 백업하고 수정본을 복사합니다.

```bash
cp ~/.bashrc ~/.bashrc.before_ros_team1
cp ~/ROS_Team1/Team/Manipulator/shell/bashrc_ROS_Team1 ~/.bashrc
source ~/.bashrc
```

수정본은 기존 터미널에 남은 구 workspace 경로를 먼저 제거하므로 위 `source` 명령을 바로 사용할 수 있습니다. 그래도 확인 결과에 예전 경로가 보이면 해당 터미널을 닫고 새 터미널을 엽니다.

복구가 필요하면 다음을 실행합니다.

```bash
cp ~/.bashrc.before_ros_team1 ~/.bashrc
source ~/.bashrc
```

## 적용 확인

프로젝트를 한 번 빌드한 뒤 새 터미널에서 확인합니다.

```bash
team1status
ros2 pkg prefix aruco_arm_sorter
ros2 pkg executables aruco_arm_sorter
```

정상 prefix는 다음과 같습니다.

```text
/home/<사용자명>/ROS_Team1/install/aruco_arm_sorter
```

실행 파일 목록에는 아래 세 항목이 모두 있어야 합니다.

```text
aruco_arm_sorter arm_sequence_controller
aruco_arm_sorter aruco_detector
aruco_arm_sorter preflight
```

빌드·점검·실행은 다음처럼 단축할 수 있습니다.

```bash
cb
team1doctor
team1run
```

`~/Ros/open_manipulator_ws`가 실제 의존성 작업공간이 아니라면 `.bashrc`의 `ARUCO_DEPENDENCY_SETUP` 자동 지정 부분을 지우고, 실제 setup 경로를 직접 지정합니다.

```bash
export ARUCO_DEPENDENCY_SETUP="/실제/의존성/워크스페이스/install/setup.bash"
```
