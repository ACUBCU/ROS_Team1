# 실행 방법 — 이서현_projectYOLO (person_follow_project)

그리퍼(손목) 카메라로 "사람"을 찾아서 다가가는 프로젝트입니다. Gazebo에서는 진짜 사람이 걸어다닐 수 없으므로, 저작권 없는 사람 실루엣 그림을 텍스처로 붙인 고정 평면(`models/person_photo`)을 world에 세워두고 그걸 YOLO26으로 "person"으로 인식하게 합니다.

## 0. 최초 1회 — 준비물 + 빌드

```bash
pip3 install ultralytics --user --break-system-packages
```
torch까지 같이 설치됩니다(용량 큼, 인터넷 필요). **설치 직후 반드시 아래로 확인하세요** (numpy/setuptools 버전이 올라가면서 `colcon build`나 `cv_bridge`가 깨진 적이 있었습니다 — CLAUDE.md 참고):
```bash
python3 -c "from cv_bridge import CvBridge; import torch; print('OK')"
```
`ImportError`가 뜨면 `pip3 install --user --break-system-packages "numpy<2" "setuptools<80,>=30.3.0"`로 다시 맞추세요.

```bash
cd ~/kongju_manipulator_2026/ros_ws
colcon build --symlink-install --packages-select person_follow_project
source install/setup.bash
```

처음 노드를 실행하면 YOLO26 가중치(`yolo26n.pt`, 약 5MB)를 인터넷에서 자동으로 내려받습니다(딱 한 번만).

## 1. (터미널 1) Gazebo 시뮬레이터 + 로봇 팔 띄우기

```bash
cd ~/open_manipulator_ws
source install/setup.bash
ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
```

world 파일이 이 프로젝트용으로 바뀌어 있으므로(사람 실루엣 평면 포함), **Gazebo가 이미 다른 이유로 떠 있었다면 반드시 완전히 재시작**하세요(world는 시작 시 한 번만 읽습니다).

## 2-A. 키보드로 실행

```bash
cd ~/kongju_manipulator_2026/ros_ws && source install/setup.bash
ros2 run person_follow_project person_follow_project
```

터미널 창에 포커스를 준 상태에서 키보드를 눌러야 합니다.

| 키 | 동작 |
|---|---|
| `q`/`a`/`w`/`s`/`e`/`d`/`r`/`f` | 관절 수동 조작 |
| `z`/`x` | 그리퍼 열기/닫기 |
| `m` | MANUAL ↔ AUTO 모드 전환 |
| `Ctrl+C` | 종료 |

**사용 순서**: MANUAL 키로 팔을 움직여서 `img` 디버그 창에 사람 실루엣이 보이고 YOLO가 박스로 "person"이라고 표시하는지 먼저 확인 → `m`으로 AUTO 전환 → 로봇이 중심을 맞추며 다가가고, 충분히 가까워지면 그리퍼를 한 번 닫았다 열어서 "인사"합니다. **사람이 화면에서 한동안 안 보이면 joint1이 천천히 좌우로 왕복하며 찾습니다** — 다시 보이면 즉시 탐색을 멈추고 추적으로 돌아갑니다.

## 2-B. 웹 페이지에서 조작하기 (권장 — 보기 편함)

```bash
cd ~/kongju_manipulator_2026/ros_ws && source install/setup.bash
ros2 run person_follow_project web_control_person
```

터미널에 뜨는 주소(`http://localhost:8080`)를 브라우저에서 열면 됩니다. 같은 와이파이의 다른 기기에서 접속하려면 `http://<이 컴퓨터의 IP>:8080`.

화면 구성:
- **카메라**: 그리퍼 카메라 화면(YOLO가 인식한 박스가 그려진 채로 나옴).
- **지금 상태**: 모드(MANUAL/AUTO), 관절 각도, 인사 상태, 탐색 중 여부가 1초마다 자동 갱신.
- **조작**: MANUAL↔AUTO 전환 버튼 + jog 버튼(joint1~4, 그리퍼) — 키보드 버전과 완전히 같은 동작.

## 3. 주의 / 한계

- "거리"는 실측이 아니라 바운딩박스 높이(픽셀)로 어림짐작한 값입니다.
- YOLO 추론은 색상 기반 인식보다 훨씬 무거워서, 카메라 30프레임 중 매 3프레임에 1번만 추론합니다(`INFER_EVERY_N_FRAMES`, `person_follow_project.py`). CPU 환경에서 반응이 느리게 느껴지면 이 값을 늘리세요.
- 웹 페이지와 키보드 실행은 동시에 띄우지 마세요(둘 다 로봇에 명령을 보내면 꼬입니다) — `이서현_project`의 `box_sort_project`/`web_control` 관계와 동일한 주의사항입니다.
