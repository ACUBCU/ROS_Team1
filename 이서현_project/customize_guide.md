# 파일별 수정 가이드 — 이서현_project (redball_sort_project)

이 문서는 `이서현_project` 폴더 안의 **파일 하나하나가 무엇을 위한 파일이고, 무엇을 어떻게 바꿀 수 있는지** 아주 자세히 정리한 문서입니다. 실행 명령어는 [run_guide.md](run_guide.md)를 보세요.

전체 파일 목록:
```
이서현_project/
├── package.xml                                  ROS2 패키지 정보표
├── setup.py                                      빌드/설치 설정, 실행 파일 등록
├── setup.cfg                                     빌드 보조 설정 (거의 안 건드림)
├── resource/redball_sort_project                 ROS2 패키지 존재 표시용 빈 파일 (건드리지 않음)
├── redball_sort_project/
│   ├── __init__.py                               (빈 파일, 건드리지 않음)
│   ├── redball_tracking_project.py               1단계: 빨간 공 추적/잡기
│   └── box_sort_project.py                       2단계: 아루코 박스 자리 이동
├── models/red_ball/
│   ├── model.sdf                                 빨간 공의 3D 모양·물리 속성
│   └── model.config                              공 모델 이름표 (거의 안 건드림)
├── config/box_slots.yaml                         가르친 자리 24곳의 저장 파일
└── test/*.py                                      ROS2 표준 코드 스타일 검사 (안 건드림)
```

---

## 1. `redball_sort_project/redball_tracking_project.py` — 빨간 공 추적/잡기

파일 위쪽 70~81번째 줄 근처에 있는 **튜닝 상수**들을 고치면 동작이 바뀝니다.

| 이름 | 의미 | 바꾸면 생기는 일 |
|---|---|---|
| `X_GAIN` (기본 0.0008) | 화면 x축 오차 1픽셀당 joint1을 얼마나 돌릴지 | 값을 키우면 팔이 더 재빠르게 좌우로 반응 (너무 크면 흔들림/과잉반응) |
| `Y_GAIN` (기본 0.0008) | 화면 y축 오차 1픽셀당 joint2~4를 얼마나 움직일지 | 위와 동일한 원리, 상하 반응 속도 |
| `MAX_STEP` (기본 0.02) | 한 프레임(카메라 콜백 1번)에 관절이 움직일 수 있는 최대 라디안 | 키우면 더 빨리 쫓아가지만 뚝뚝 끊기듯 움직일 수 있음 |
| `MIN_BALL_AREA` (기본 20) | 이 값보다 작은 빨간 덩어리는 노이즈로 무시 | 키우면 작게 보이는(멀리 있는) 공은 무시하게 됨 |
| `GRAB_AREA` (기본 8000) | 공 면적이 이 값보다 커지면 "충분히 가까움"으로 판단해 그리퍼로 잡음 | 키우면 더 가까이 와야 잡고, 줄이면 멀리서도 잡으려 시도함 |
| `STEP` (기본 0.05) | MANUAL 모드에서 키 한 번에 관절이 움직이는 양(라디안) | 키우면 한 번 누를 때 더 크게 움직임 |
| `GRIPPER_OPEN` (0.019) / `GRIPPER_CLOSE` (-0.01) | 그리퍼가 "열림"/"닫힘"으로 취급하는 위치 값 | 로봇 그리퍼 최대 개폐 범위를 벗어나지 않게 조심해서 조정 |

**빨간색 인식 범위 바꾸기** (150번째 줄 근처):
```python
lower1 = np.array([0, 40, 40], dtype=np.uint8)
upper1 = np.array([10, 255, 255], dtype=np.uint8)
lower2 = np.array([170, 40, 40], dtype=np.uint8)
upper2 = np.array([180, 255, 255], dtype=np.uint8)
```
이건 HSV 색공간에서 "빨간색"으로 인정할 범위입니다. 첫 번째 숫자(Hue, 0~180)가 색상, 두 번째(Saturation)가 채도(선명함), 세 번째(Value)가 밝기입니다. 예를 들어 조명이 어두워서 공이 잘 안 잡히면 `40`으로 되어 있는 Saturation/Value 최소값을 `20`처럼 낮춰보세요. 다른 색(예: 파란색)을 잡고 싶으면 Hue 범위를 바꾸면 됩니다 (파란색은 대략 100~130).

**키 배치 바꾸기** (84번째 줄부터 시작하는 `MANUAL_KEY_BINDINGS` 딕셔너리): 예를 들어 joint1을 `q`/`a` 대신 다른 키로 바꾸고 싶으면 이 딕셔너리의 키 이름만 바꾸면 됩니다.

---

## 2. `redball_sort_project/box_sort_project.py` — 아루코 박스 자리 이동

파일 위쪽 72~77번째 줄 근처 **튜닝 상수**:

| 이름 | 의미 | 바꾸면 생기는 일 |
|---|---|---|
| `STEP` (0.05) | TEACH 모드에서 키 한 번에 움직이는 관절 각도(라디안) | 세밀하게 자리를 맞추고 싶으면 더 작게(예: 0.02) |
| `GRIPPER_OPEN`/`GRIPPER_CLOSE` | 그리퍼 열림/닫힘 값 | redball_tracking_project.py와 동일 |
| `TOTAL_SLOTS` (24) | 전체 자리 개수 | 자리를 더 늘리거나 줄이려면 이 값을 바꿈 (예: 방향을 4개로 줄이고 거리 3단계면 12) |
| `MOVE_TIMEOUT_SEC` (8.0) | COMMAND 실행 중 팔이 자리에 도착할 때까지 최대로 기다리는 시간(초) | 팔이 느리거나 먼 거리를 움직인다면 늘려야 함 |
| `SETTLE_SEC` (0.5) | 팔이 도착한 뒤 그리퍼를 움직이기 전에 흔들림이 가라앉기를 기다리는 시간(초) | 그리퍼가 너무 빨리 닫혀서 헛집는다면 늘려보세요 |

81~91번째 줄 근처 **아루코 인식 상수**:

| 이름 | 의미 | 바꾸면 생기는 일 |
|---|---|---|
| `MARKER_SIZE_M` (0.04) | 마커의 실제 한 변 크기(m) | 다른 크기의 마커를 쓰면 이 값도 같이 바꿔야 거리 계산이 맞음 |
| `CAMERA_HFOV_RAD` (1.0472) | 카메라 수평 시야각(라디안) | `open_manipulator_x.gazebo.xacro`의 `horizontal_fov`를 바꾸면 여기도 같이 바꿔야 거리 추정이 맞음 |
| `IMAGE_WIDTH`/`IMAGE_HEIGHT` (640/480) | 카메라 해상도 | 카메라 센서 설정(xacro)의 `width`/`height`를 바꾸면 같이 바꿔야 함 |

**자리 번호 매기는 규칙 바꾸기**: 지금은 코드가 아니라 **사람이 정하는 규칙**입니다 (파일 맨 위 주석 참고: 번호 = (방향번호-1)*3 + 거리단계). 실제로 몇 번이 어느 방향/거리인지는 여러분이 TEACH할 때 정하는 것이라, 코드를 안 고쳐도 원하는 대로 배치를 바꿀 수 있습니다. 다만 자리 개수 자체(24개)를 바꾸려면 `TOTAL_SLOTS`를 수정하세요.

**COMMAND 실행 순서 바꾸기** (`run_command` 메서드, 265번째 줄 근처): 지금은 "출발자리로 이동 → 집기 → 도착자리로 이동 → 놓기" 4단계입니다. 예를 들어 옮긴 뒤에 다시 홈 자세로 돌아오게 하고 싶으면, 이 메서드 마지막에 `self._move_and_wait([0.0, 0.0, 0.0, 0.0])` 같은 코드를 추가하면 됩니다.

**아루코 인식 로직 바꾸기** (`image_callback` 메서드, 183번째 줄 근처): 지금은 감지·거리 표시만 하고 자리 이동 로직에는 안 씁니다. 예를 들어 "출발 자리에 실제로 박스가 있는지 COMMAND 실행 전에 확인" 같은 걸 추가하고 싶으면, 이 메서드에서 마지막으로 감지된 마커 정보를 `self.last_marker_id`, `self.last_marker_distance` 같은 속성에 저장해두고 `run_command`에서 확인하는 식으로 확장할 수 있습니다.

**키 배치**: `redball_tracking_project.py`와 마찬가지로 `MANUAL_KEY_BINDINGS` 딕셔너리(98번째 줄부터)에서 관절 조작 키를, `handle_key` 메서드에서 `n`/`b`/`g`/`z`/`x` 같은 특수 키를 바꿀 수 있습니다.

---

## 3. `config/box_slots.yaml` — 가르친 자리 저장 파일

TEACH 모드에서 `g`를 누를 때마다 프로그램이 **자동으로** 이 파일을 고칩니다. **지금은 24개 자리가 이미 다 채워져 있습니다** (TF로 실제 위치를 확인하면서 자동으로 채운 값들 — 방법은 [project_summary.md](project_summary.md)의 "24개 자리를 자동으로 가르친 방법" 참고). 마음에 안 드는 자리가 있으면 TEACH 모드로 그 번호만 다시 가르쳐서 덮어쓰면 됩니다. 아래처럼 **직접 열어서 손으로 고치거나 추가할 수도** 있습니다.

```yaml
slots:
  1:
    joints: [0.1, -0.5, 0.3, 0.2]   # joint1, joint2, joint3, joint4 순서 (라디안)
    gripper: 0.019                  # 그리퍼 값 (0.019=열림 근처, -0.01=닫힘 근처)
  7:
    joints: [0.9, -0.2, 0.1, 0.4]
    gripper: 0.019
```

**주의할 점**:
- 번호는 정수(1~24)로, `joints`는 반드시 숫자 4개짜리 목록으로 적어야 합니다. 하나라도 빠지면 프로그램 실행 시 오류가 납니다.
- 손으로 각도를 직접 추측해서 적지 마세요 — 저희가 겪었듯이 관절 각도를 감으로 정하면 팔이 바닥이나 자기 자신에 부딪힐 수 있습니다. **반드시 TEACH 모드로 실제 팔을 움직여서 눈으로 확인한 뒤 `g`로 저장하는 방식**을 쓰세요. 이 파일을 손으로 고치는 건 "이미 확인된 값을 옮겨 적거나 살짝 미세조정"하는 용도로만 쓰는 걸 권장합니다.
- 특정 자리를 지우고 싶으면 그 번호의 블록(`7:` 부터 그 아래 `joints`, `gripper` 줄까지)을 통째로 삭제하면 됩니다.

---

## 4. `models/red_ball/model.sdf` — 빨간 공의 3D 모양

| 무엇을 바꾸나 | 어디를 고치나 |
|---|---|
| 공 크기 | `<radius>0.02</radius>` (두 군데: `ball_collision`, `ball_visual` — **둘 다 같이 바꿔야** 겉모습과 실제 충돌 판정이 일치함) |
| 공 무게 | `<mass>0.05</mass>` (단위: kg). 무게를 바꾸면 `<inertia>` 값도 다시 계산해야 정확함 (공식: 속이 찬 구는 `2/5 * 질량 * 반지름^2`) |
| 색깔 | `<ambient>1 0 0 1</ambient>`, `<diffuse>1 0 0 1</diffuse>` — R G B A(투명도) 순서, 0~1 값. 예를 들어 파란색은 `0 0 1 1` |
| 미끄러짐 정도 | `<mu>15</mu>`, `<mu2>15</mu2>` — 마찰 계수, 높을수록 안 미끄러짐 |

고친 뒤에는 `redball_sort_project` 패키지를 다시 빌드해야 Gazebo가 새 모양을 인식합니다 (`colcon build --symlink-install --packages-select redball_sort_project`).

## 5. `models/red_ball/model.config` — 공 모델 이름표

`<name>`(Gazebo 안에서 보이는 이름), `<description>`(설명 글) 정도만 있고, 동작에는 영향 없는 순수 정보성 파일입니다. 자유롭게 고쳐도 안전합니다.

---

## 6. `setup.py` — 새 파이썬 파일을 실행 가능하게 등록하기

**새 노드 파일을 하나 더 추가하고 싶다면** (예: `my_new_node.py`를 `redball_sort_project/` 폴더에 새로 만들었다면), `entry_points`의 `console_scripts` 목록에 아래처럼 한 줄 추가해야 `ros2 run`으로 실행할 수 있습니다.

```python
"entry_points={
    "console_scripts": [
        "redball_tracking_project = redball_sort_project.redball_tracking_project:main",
        "box_sort_project = redball_sort_project.box_sort_project:main",
        "my_new_node = redball_sort_project.my_new_node:main",   # <- 이렇게 추가
    ],
},
```
형식은 `"실행할때_부를_이름 = 폴더이름.파일이름:main함수이름"`입니다. 추가한 뒤에는 반드시 다시 빌드해야 합니다.

**새로운 종류의 데이터 폴더**(예: `sounds/` 폴더에 음성 파일 추가)를 설치에 포함시키고 싶다면, `data_files` 리스트에 `+ package_files("sounds")`처럼 한 줄 추가하면 됩니다 (지금 `models`, `config`가 이렇게 되어 있는 것과 같은 방식).

## 7. `package.xml` — 패키지 정보

`<description>`, `<maintainer>` 같은 항목은 동작에는 영향 없는 정보성 태그라 자유롭게 고쳐도 됩니다. `<name>redball_sort_project</name>`은 **이 패키지의 진짜 이름**이라, 함부로 바꾸면 `setup.py`의 `package_name`, 폴더 이름(`redball_sort_project/`), 다른 곳에서 이 패키지를 부르는 모든 곳(`ros2 run redball_sort_project ...`, world 파일의 `package://redball_sort_project/...`)을 전부 같이 고쳐야 해서 왠만하면 건드리지 않는 걸 추천합니다.

## 8. `setup.cfg`, `resource/redball_sort_project`, `redball_sort_project/__init__.py`, `test/*.py`

이 파일들은 ROS2 파이썬 패키지가 정상적으로 인식되기 위한 **표준 골격 파일**들이라, 특별한 이유가 없으면 건드리지 않는 걸 권장합니다. (`test/*.py`는 코드 스타일 자동 검사용이라, 어차피 우리 코드가 스타일 규칙을 어기면 검사만 실패할 뿐 실행에는 영향 없습니다.)
