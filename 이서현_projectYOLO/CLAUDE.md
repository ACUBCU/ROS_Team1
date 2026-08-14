# 이서현_projectYOLO — 새 세션/다른 프롬프트가 읽어야 할 컨텍스트

**이 폴더는 `이서현_project`(빨간 공/아루코 박스 프로젝트)와 완전히 별개의 프로젝트입니다.** YOLO26으로 사람을 인식해서 그리퍼 카메라로 다가가는 데모입니다. ROS2 패키지 이름은 `person_follow_project`(한글 폴더명과 다름).

## 구조

- `person_follow_project/person_follow_project.py` — 메인 노드. `redball_tracking_project.py`(이서현_project)와 같은 뼈대(MANUAL/AUTO 모드, ActionClient 기반)를 재사용하되, 색상 인식 대신 YOLO26으로 "person"을 찾음.
- `person_follow_project/web_control_person.py` — 위 노드를 웹 브라우저(`http://localhost:8080`)에서 보고 조작하는 Flask 서버.
- `models/person_photo/` — Gazebo에서 실제 사람 대신 쓰는, 저작권 없는 사람 실루엣을 텍스처로 붙인 고정 평면 모델.

## 워크스페이스 2개를 같이 씀 (이서현_project와 동일한 구조)

- `~/kongju_manipulator_2026/ros_ws/` — 이 git 저장소.
- `~/open_manipulator_ws/` — **이 git 저장소 밖**, ROBOTIS가 제공한 별도 워크스페이스. `open_manipulator_x_gazebo.launch.py`의 `GZ_SIM_RESOURCE_PATH`에 이 패키지(`person_follow_project`)의 `models/` 공유 디렉토리를 등록해뒀음(이걸 등록 안 하면 Gazebo가 `model://person_photo`를 못 찾음).

두 워크스페이스를 둘 다 `source install/setup.bash` 해야 동작합니다. Gazebo 관련 안전수칙(인스턴스 1개만, 좀비 프로세스 확인 등)은 `이서현_project/CLAUDE.md`에 이미 자세히 기록되어 있고 이 프로젝트에도 그대로 적용됩니다.

## `pip install ultralytics`가 ROS 빌드를 깨뜨린 사고 (2026-08-13) — 다시 겪을 수 있음

`pip3 install ultralytics --user --break-system-packages`로 torch까지 설치했더니, 딸려온 최신 `numpy`(2.x)와 `setuptools`(80+)가 `--user` site-packages에 깔리면서 **이 계정의 모든 python3 실행**(`colcon build`, `cv_bridge` 등)에 영향을 줬습니다. `setuptools<80`, `numpy<2`로 맞춰서 복구함 — `torch`/`ultralytics`는 `numpy<2`에서도 정상 동작 확인됨. 앞으로 이 프로젝트에 새 파이썬 패키지를 추가로 설치할 때는 설치 직후 `colcon build`와 `python3 -c "from cv_bridge import CvBridge"`로 안 깨졌는지 꼭 재확인하세요.

## 사람 사진은 평면 소품 — "잡기" 대신 "인사"

`models/person_photo`는 그냥 텍스처 붙인 평면이라 실제로 쥘 것이 없습니다. 충분히 가까워지면 그리퍼를 CLOSE→OPEN 한 번만 보내는 "인사" 동작으로 대신합니다 (`greet_state` 상태머신, `person_follow_project.py` 참고).

## 탐색(사람 못 찾으면 천천히 회전)

AUTO 모드에서 사람이 `NO_DETECTION_SEARCH_AFTER`(추론 프레임 기준) 넘게 안 보이면 joint1을 `SEARCH_STEP`씩 천천히 좌우로 왕복시키며 찾습니다(`_search_if_needed()`). joint1의 물리적 한계(-pi~+pi)에 딱 붙지 않도록 `SEARCH_MIN`/`SEARCH_MAX`(±3.0)로 여유를 둠 — `이서현_project`의 box_sort_project.py가 180도 대신 3.13을 쓴 것과 같은 이유.

## 실물 로봇 연동 (미래)

아직 실물이 없어서 시뮬레이션까지만 되어 있습니다. `person_follow_project.py`는 ROS2 표준 액션/토픽(`FollowJointTrajectory`, `GripperCommand`, `/joint_states`, `/gripper_camera/image_raw`)만 쓰고 Gazebo 전용 API를 직접 호출하지 않으므로, 실물이 생기면 launch 구성만 바꿔서(이서현_project/RUN_GUIDE.md의 "2-E" 절 원칙과 동일) 코드 수정 없이 재사용을 시도해볼 수 있습니다. 다만 그리퍼 타이밍/joint 한계 등 튜닝값은 Gazebo 물리엔진 기준이라 실물에서는 다시 검증이 필요합니다.
