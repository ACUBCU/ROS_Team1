# 이서현_project — 새 세션/다른 프롬프트가 읽어야 할 컨텍스트

이 폴더는 OpenManipulator-X를 Gazebo 시뮬레이션에서 조작하는 학생(이서현) 개인 프로젝트입니다. ROS2 패키지 이름은 `redball_sort_project`(한글 폴더명과 다름 — 아래 참고).

**사람 추적(YOLO26) 프로젝트는 여기 없습니다.** 완전히 별개 프로젝트로 취급해서 `ros_ws/src/이서현_projectYOLO`(ROS2 패키지명 `person_follow_project`)라는 별도 패키지에 있습니다. 이 폴더(`이서현_project`)는 빨간 공 추적 + 아루코 박스 자리 이동 프로젝트만 다룹니다.

## 문서 구조 (먼저 읽을 것)

- [HISTORY.md](HISTORY.md) — 기술적으로 자세한 설명, 겪은 버그와 해결 과정 전부 기록됨
- [OVERVIEW.md](OVERVIEW.md) — 같은 내용을 쉽게 풀어쓴 버전
- [RUN_GUIDE.md](RUN_GUIDE.md) — 실행 명령어만
- [CUSTOMIZE_GUIDE.md](CUSTOMIZE_GUIDE.md) — 파일별로 뭘 어떻게 고칠 수 있는지

**새 세션에서 이 프로젝트를 이어서 작업한다면, 위 문서들 특히 HISTORY.md의 "오늘 찾은 버그" 섹션들을 먼저 읽고 시작하세요. 이미 겪고 고친 문제를 반복하지 않기 위함입니다.**

## 워크스페이스 2개를 같이 씀 (헷갈리기 쉬움)

- `~/kongju_manipulator_2026/ros_ws/` — 이 git 저장소. 우리 코드(`이서현_project`), 수업용 예제 패키지들(`tf2_basic`, `camera_opencv` 등)
- `~/open_manipulator_ws/` — **이 git 저장소 밖**, ROBOTIS가 제공한 별도 워크스페이스. 실제 로봇 팔 description/bringup 패키지가 여기 있고, 그리퍼 카메라 센서·bridge 설정도 여기 있음(`open_manipulator_bringup`, `open_manipulator_description`).

두 워크스페이스를 둘 다 `source install/setup.bash` 해야 전체가 동작합니다.

## Gazebo는 항상 딱 1개만 떠 있어야 함 — 여러 번 겪은 사고

**AI(나)와 사용자가 각자 다른 터미널에서 동시에 Gazebo를 켜면, 컨트롤러 충돌·물리엔진 크래시·world 파일 수정이 반영 안 되는 것처럼 보이는 문제가 생깁니다.** world 파일은 Gazebo가 시작할 때 딱 한 번만 읽기 때문에, 이미 떠 있는 인스턴스는 파일을 고쳐도 재시작 전까지 반영이 안 됩니다 — "고쳤는데 왜 그대로냐"는 문제의 90%는 사실 오래된 인스턴스가 여전히 떠 있는 것입니다.

**작업 시작 전 항상 확인**:
```bash
ps aux | grep -iE "gz sim|parameter_bridge|robot_state_publisher|move_group" | grep -v grep
```
뭔가 떠 있으면 새로 켜기 전에 먼저 정리하세요.

**정리할 때 주의 (실제로 겪은 것)**: `pkill -9 -f "gz sim"` 같은 패턴 매칭 kill이 **다른 터미널(특히 VSCode의 별도 세션/사용자가 직접 연 터미널)에서 띄운 프로세스에는 안 먹힐 때가 있습니다** (조용히 실패함 — 에러도 안 남). `pkill` 실행 후에는 반드시 `ps aux | grep ...`로 진짜 다 죽었는지 재확인하고, 남아있으면 정확한 PID로 `kill -9 <pid>`를 다시 하세요. 이걸 안 하고 "정리했다"고 믿은 채 새로 띄웠다가 **Gazebo가 3개 동시에 뜬 적**이 있습니다.

**사용자가 직접 터미널에서 Gazebo를 켜려고 할 때**: 먼저 내가 띄워둔 게 있는지 확인해서 알려주고, 중복 실행하지 않도록 조율하세요 (죽여야 하면 누구 걸 죽일지 먼저 물어보기 — 단, 상황이 급하고 사용자가 이미 "정리해"라고 명확히 요청했다면 지체 없이 정리).

## 절대 하지 말 것 / 반드시 지킬 것 (실제로 겪은 사고 기반)

1. **관절 각도를 감으로 보내지 말 것.** 한 번 감으로 큰 값을 보냈다가 팔이 바닥 쪽으로 향한 적이 있습니다. 새 자세가 필요하면 반드시:
   - 작은 값부터 시작해서 점진적으로 늘리고,
   - `tf2_ros.Buffer`/`TransformListener`로 `world → end_effector_link`(또는 `gripper_left_link`/`gripper_right_link`)의 **실측 좌표**를 매번 확인하고,
   - 높이(z)가 안전선 아래로 내려가면 즉시 중단.
2. **`FollowJointTrajectory` 액션이 SUCCEEDED를 반환해도 실제로 목표에 도착했다고 믿지 말 것.** 낮은 위치 게인(P-only 제어 추정) 때문에 무게가 많이 실리는 자세(특히 joint2가 크게 움직이는 경우)에서 정상상태 오차가 남습니다. `box_sort_project.py`의 `_move_and_wait()`가 이미 "목표-실제값 확인 후 오차만큼 보정해서 최대 3번 재시도"하는 방식으로 이 문제를 처리합니다 — 새 이동 로직을 짤 때도 이 패턴을 재사용하세요.
3. **joint2는 약 0.85rad 근처가 이 아암의 실질적 한계로 보입니다** (그 이상 보내면 재시도해도 도달 못 함 — 자기충돌 또는 토크 한계로 추정). 이보다 큰 값을 목표로 잡지 마세요.
4. **`goal.trajectory.header.stamp`에 `self.get_clock().now()`(wall-clock)를 채우지 말 것.** Gazebo 컨트롤러는 `use_sim_time`을 쓰기 때문에 시간이 안 맞아서 결과가 영원히 안 옵니다. 기본값(0, "즉시 시작")을 그대로 두세요.
5. **`/joint_states`는 `[gripper_left_joint, gripper_right_joint, joint1, joint2, joint3, joint4]` 순서입니다.** `msg.position`을 그대로 쓰지 말고 반드시 `msg.name`으로 매칭해서 순서를 다시 맞추세요.
6. **world 파일(`<include><uri>...`)에 `package://` 스킴을 쓰지 말 것.** `gz sim`이 world를 직접 읽을 때는 `package://`를 이해 못 해서 Gazebo가 아예 안 켜집니다. `model://이름` + `GZ_SIM_RESOURCE_PATH`에 해당 경로 등록 방식만 쓰세요 (`open_manipulator_x_gazebo.launch.py` 참고).
7. **`LIBGL_ALWAYS_SOFTWARE=1` 같은 소프트웨어 렌더링 강제 옵션을 켜지 말 것.** `ZINK: failed to choose pdev` 에러 로그는 무시해도 되는 경고이고, Mesa가 자동으로 GPU(D3D12)로 전환합니다. 강제로 소프트웨어 렌더링을 켜면 극심한 렉만 생깁니다.
8. **실제로 GUI가 남아있는지, 중복 프로세스가 없는지 항상 확인.** `gz sim`, `ros2 launch`, `parameter_bridge`, `robot_state_publisher` 프로세스가 이전 실행에서 안 죽고 남아있으면 컨트롤러가 충돌해서 물리엔진이 죽거나(`Aborted`) 응답이 안 옵니다. 새로 띄우기 전에 `ps aux | grep -iE "gz sim|parameter_bridge|controller_manager"`로 확인하세요.

## 그리퍼 실제 집는 지점 (end_effector_link ≠ 손가락 중점)

`end_effector_link`은 URDF상 `link5`에서 `(0.126, 0, 0)` 떨어진 곳이고, 그리퍼 손가락 조인트 원점은 `(0.0817, ±0.021, 0)`입니다. 즉 **`end_effector_link`은 손가락이 실제로 오므라드는 지점보다 약 4.4cm 더 앞에 있습니다.** 물체를 집을 위치를 계산할 땐 `end_effector_link`이 아니라 `gripper_left_link`/`gripper_right_link` TF의 중점을 기준으로 삼아야 합니다.

## `pip install`로 ML 패키지(ultralytics 등)를 깔 때 — 실제로 ROS 빌드를 깨뜨린 사고 (2026-08-13)

YOLO26 사람 추적 프로젝트(`이서현_projectYOLO`, 이 폴더 밖의 별도 패키지)를 위해 `pip3 install ultralytics --user --break-system-packages`를 실행했더니, 의존성으로 딸려온 최신 `numpy`(2.5.2)와 `setuptools`(84.0.0)가 `--user` site-packages에 깔리면서(우선순위가 시스템 site-packages보다 높음) **이 사용자 계정의 모든 python3 실행에 영향**을 줬습니다:
- `setuptools>=80` 때문에 `colcon build`가 `error: option --uninstall not recognized`로 전부 실패함(`colcon-core`는 `setuptools<80` 요구).
- `numpy 2.x` 때문에 `cv_bridge`(시스템 ROS 패키지, numpy 1.x ABI로 컴파일됨)가 import 시점에 크래시함 — `redball_tracking_project.py`/`box_sort_project.py` 등 **기존에 잘 되던 노드까지 전부 같이 깨짐**.

**해결**: `pip3 install --user --break-system-packages "setuptools<80,>=30.3.0"`, `pip3 install --user --break-system-packages "numpy<2"`로 다시 맞춤. `torch`/`ultralytics`는 `numpy<2`에서도 정상 동작 확인함.

**교훈**: 이 워크스페이스는 시스템 python3(`/opt/ros/jazzy`가 설치된 그 python3)를 `ros2 run`이 그대로 쓰고 있어서, **`pip install --user`로 뭘 깔든 이 시스템 전체(ROS 빌드 포함)에 영향을 줍니다.** 앞으로 새 파이썬 패키지를 설치할 때는 설치 직후 반드시 `colcon build`와 `python3 -c "from cv_bridge import CvBridge"`(또는 실제 사용하는 노드 import)로 안 깨졌는지 확인하세요. 완전히 격리하려면 `.venv`(이 repo 루트에 이미 있음, `include-system-site-packages=true`)를 쓰는 방법도 있지만, `include-system-site-packages=true`라서 이것도 100% 격리는 아님 — 진짜 격리가 필요하면 `--system-site-packages` 없이 새 venv를 만들어야 함.

## 그 외 겪은 것

- **ROS 메시지의 정수 필드에 numpy 정수(`np.int32` 등)를 그대로 넣으면 죽음** (`PyLong_Check` assert 실패). `cv2.aruco`의 marker id처럼 numpy 배열에서 나온 값은 꼭 `int()`로 변환.

## 그리퍼로 실제 물체 집기 — 실제로 겪은 문제 4가지 (box_sort_project.py, 2026-08-10 해결)

**"팔이 목표에 도착했다"와 "그리퍼가 로그상 성공했다"는 절대로 "박스를 실제로 옮겼다"의 증거가 아닙니다.** 아래 4가지가 전부 "로그는 성공인데 실제로는 박스가 안 옮겨지거나 튕겨나가는" 형태의 조용한 실패였고, `gz topic -e -t /world/<world>/pose/info -n 1 | grep -A5 '<model_name>'`으로 **박스의 실제 월드 좌표를 이동 전/후 비교**해야만 발견할 수 있었습니다.

1. **`position_controllers/GripperActionController`의 기본값(`allow_stalling: false`)은 "쥐는 용도"가 아님.** 물체에 막혀서 목표까지 못 닫으면 기본값은 이걸 실패(ABORTED)로 처리합니다. `open_manipulator_ws`(git 저장소 밖)의 `open_manipulator_bringup/config/open_manipulator_x/hardware_controller_manager.yaml`에서 `gripper_controller`에 `allow_stalling: true` (+ `stall_timeout: 1.0`, 기본값)를 추가해야 "막히면 = 잡은 것"으로 정상 처리됨. **`stall_timeout`을 기본값(1.0초)보다 짧게 주지 마세요** — 0.5초로 줄였다가 그리퍼가 움직이기 시작하자마자 "멈췄다"고 오판해서 목표의 절반도 못 닫고 성공 처리된 적이 있음.
2. **집은 채로 큰 회전(joint1)을 하기 전에 반드시 먼저 들어올려야 함.** 박스가 바닥에 닿은 채로 joint1을 돌리면 바닥 마찰이 그리퍼 그립보다 세서 8도만 돌려도 즉시 빠져버림(실측 확인). `LIFT_DELTA`(joint2를 줄여서 들어올림)만큼 먼저 들어올린 뒤 회전 → 도착 후 다시 내리는 3단계로 나눠야 함.
3. **첫 접근(홈 자세 → 집을 자리)도 낮게 스쳐 지나가면 안 됨.** 관절-공간 스플라인 보간은 경로 중간에 팔이 목표보다 더 낮게 처지면서 박스를 미리 건드릴 수 있고, 그 상태에서 그리퍼를 닫으면 물리엔진이 겹친 충돌을 한 프레임에 풀어내며 박스가 수십 cm 튕겨나감(실측: 36cm). 집기 전에도 "들어올린 높이"를 먼저 거쳐서 위에서 아래로 내려가듯 접근해야 함.
4. **시스템 부하가 높으면 액션이 실제로는 성공했는데 결과 콜백만 늦게 와서 타임아웃으로 잘못 실패 처리될 수 있음.** `MOVE_TIMEOUT_SEC`을 너무 빡빡하게 잡지 말 것(8초→15초→자리가 4개로 늘면서 35초로 늘림). 타임아웃 로그가 떠도 `/joint_states`로 실제 도달 여부를 확인해보면 이미 도착해 있는 경우가 있었음.

이 4가지를 모두 고친 뒤, 실제 `run_command()`로 1↔2번 자리 왕복 픽앤플레이스를 **박스 좌표 확인까지 포함해서 2번 연속 성공** 확인함.

## 자리를 2개→4개로 늘리면서 새로 겪은 문제 3가지 (2026-08-10)

5. **자리 이동 속도를 거리와 상관없이 고정 시간으로 두면 안 됨.** 2자리일 때 "45도를 4초"로 고정해뒀던 걸 자리 4개(최대 180도 회전)에도 그대로 썼더니, 오히려 각속도가 더 빨라져서 박스를 쥔 채로 빠르게 돌다가 그립이 못 버티고 빠짐(실측: 목표 자리와 전혀 다른 곳에 떨어짐). `ANGULAR_SPEED_LIMIT_RAD_S`로 각속도 상한을 정해두고, 이동 시간을 `max(3.0, 거리/속도상한)`으로 항상 거리에 비례하게 계산하도록 고침 (`_send_trajectory_once` 참고).
6. **`gz_ros2_control`의 기본 `position_proportional_gain`(0.1)은 그리퍼가 회전 중 박스를 붙잡고 버티기엔 너무 약함.** 팔은 정확히 목표 자리(joint1 값 확인함)에 도착했는데 박스만 회전 초반에 떨어져서 원래 자리 근처에 남아있는 조용한 실패였음 — "그리퍼 닫기 성공" 로그도 정상으로 떴어서 처음엔 원인을 몰랐음. `open_manipulator_ws`(git 저장소 밖)의 `open_manipulator_description/ros2_control/open_manipulator_x_position.ros2_control.xacro`에서 `gripper_left_joint`에 `<param name="position_proportional_gain">20.0</param>`를 추가해서 해결. (5번 속도 문제만으로는 부족했음 - 속도를 늦춰도 여전히 살짝 밀렸고, 그리퍼 힘을 세게 준 뒤에야 목표 지점 2cm 이내로 정확히 도착함.)
7. **자리를 로봇의 기본(홈, 모든 관절=0) 자세와 겹치는 각도에 두면 안 됨.** 처음엔 자리를 0/90/180/270도(로봇 정면 포함)로 뒀는데, 로봇이 쉴 때(명령 사이 대기 중) 팔이 항상 0도 방향으로 뻗어있어서 그 방향 자리의 박스를 천장 카메라가 계속 못 봄(실측: 해당 자리 마커가 프레임 하나도 안 잡힘). 자리를 45/135/-135/-45도로 45도씩 어긋나게 옮겨서 로봇 휴식 자세와 안 겹치게 해결.

**천장 카메라 시야각/해상도도 같이 조정함**: 처음엔 그리퍼 카메라와 같은 60도를 썼는데 640x480 화면비 때문에 세로 시야각이 좁아서 자리 하나가 화면 밖으로 걸렸음 → 80도로 넓힘 → 그러자 마커가 너무 작게(20px) 나와서 인식 실패 → 해상도를 1280x960으로 2배 올려서 해결(자세한 내용은 `box_sort_project.py`의 `OVERHEAD_CAMERA_HFOV_RAD` 주석 참고).

**박스 마커를 6면 전부에 붙임** (`models/aruco_cube`, `models/aruco_cube_2`): 원래 윗면에만 있었는데, 혹시 박스가 넘어지거나 옆으로 눕는 경우에도 천장 카메라가 어느 면이 위로 오든 인식할 수 있도록 6면 모두에 같은 마커를 붙임.

**COMMAND 입력 방식이 "출발자리 도착자리"에서 "마커id 도착자리"로 바뀜.** 천장 카메라가 각 마커의 현재 자리를 실시간으로 알고 있으므로(`self.slot_occupancy`), 출발 자리를 사람이 알려줄 필요가 없어짐 (`find_slot_of_marker()` 참고). 도착 자리가 이미 다른 박스로 점유되어 있으면 이동 자체를 거부함.

## 현재 상태 (2026-08-10 갱신)

- `redball_tracking_project.py`: 빨간 공 카메라 추적 + MANUAL/AUTO 모드. 팔 이동 실제 동작 확인됨.
- `box_sort_project.py`: 아루코 박스 자리 이동. **자리 4개, 박스 2개**로 검증됨. COMMAND는 "마커id 도착자리" 형식(예: `0 3`) — 출발 자리는 천장 카메라가 실시간으로 찾음. 도착 자리가 이미 점유되어 있으면 거부함. **집기(grip)+회전+점유 확인까지 전부 실제 좌표로 검증됨**(위 두 섹션 참고).
- `web_control.py`: 웹 페이지(브라우저)에서 TEACH 조작 + COMMAND 박스 이동 + 카메라(천장/손목) 실시간 화면까지 전부 할 수 있음. `ros2 run redball_sort_project web_control` 실행 후 `http://localhost:8080` 접속. `BoxSortProject`를 그대로 재사용하고, 로봇과의 통신은 전용 스레드 하나에서만 처리(큐로 명령 전달)해서 Flask와 rclpy가 서로 안 막히게 함. 카메라 스트리밍(`/video/overhead`, `/video/gripper`)은 MJPEG 방식이고, 스트리밍 중에도 다른 버튼이 먹히려면 `app.run(threaded=True)`가 반드시 필요함(기본값 False면 스트리밍 연결 하나가 서버 전체를 막음).
- **다음 단계**: 4자리+박스 2개 픽앤플레이스가 안정적으로 검증되었으니, 자리를 더 늘리거나(8개 등) 박스를 더 추가할 수 있음. 자리를 늘릴 때는 반드시 로봇 홈 자세(joint1=0) 각도를 피해서 배치할 것(7번 문제 참고), 그리고 SLOT_RADIUS_M 원 위에 놓을 것(천장 카메라 점유 판단 전제).
- COMMAND 모드의 `run_command()`는 테스트 스크립트로 직접 호출해서 박스 좌표까지 검증함. 웹 페이지의 TEACH/COMMAND 버튼도 curl로 각 엔드포인트(`/jog`, `/move`, `/status`)를 호출해서 실제로 로봇이 움직이는 것까지 확인함.
- `open_manipulator_bringup/worlds/empty_world.sdf` 기본 world에는 이제 **`aruco_cube_1`(마커 id=0)이 1번 자리(0.2001, 0.2001), `aruco_cube_2`(마커 id=1)가 3번 자리(-0.2001, -0.2001)에 기본으로 있습니다.** red_ball은 기본 world에서 뺐습니다 (1단계 프로젝트가 끝나서). 필요하면 `models/red_ball`은 그대로 남아있으니 include만 다시 추가하면 됨.
- **아루코 박스 모델 소유권**: `aruco_cube` 모델은 원래 `tf2_basic` 패키지 것이었는데, `이서현_project/models/aruco_cube/`로 복사해서 이제 이 프로젝트 안에서 자체적으로 관리합니다 (`tf2_basic` 원본은 다른 수업 예제가 쓸 수도 있어 그대로 둠). `open_manipulator_x_gazebo.launch.py`의 `GZ_SIM_RESOURCE_PATH`에서 `redball_sort_project`(이서현_project) 경로를 `tf2_basic`보다 먼저 등록해서, 이름이 같아도 이서현_project 사본이 먼저 찾아지도록 해뒀습니다.
- **오버헤드(고정) 카메라 추가함** (2026-08-10): 작업공간 전체를 위에서 내려다보는 두 번째 카메라. `/overhead_camera/image_raw`, `/overhead_camera/camera_info` 토픽. 로봇 위 0.8m, 수직 아래를 봄. **주의**: 처음엔 손목 카메라처럼 로봇 URDF에 fixed joint로 끼워 넣었는데, 완전히 안 움직이는 카메라라 그런지 화면이 첫 프레임에서 멈춰있는 것처럼 보이는 문제가 있었음 — 나중에 알고보니 실제로는 정상 작동 중이었고 단지 로봇 기본 자세(all-zero)가 위에서 보면 원래 저렇게 구부러져 보이는 것뿐이었음(오해였음, 큰 각도로 움직여보고서야 확인됨). 그래도 **최종적으로는 로봇 URDF가 아니라 `empty_world.sdf`에 독립된 `<static>true</static>` 모델로 넣는 방식을 채택**함 — 환경 고정 센서는 로봇 기술서(URDF)보다 world 파일에 넣는 게 더 일반적인 방법이라 이쪽으로 정리함. 그리퍼 카메라(`gripper_camera`)는 그대로 URDF 방식 유지.
