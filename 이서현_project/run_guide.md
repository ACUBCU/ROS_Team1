# 실행 방법 — 이서현_project (redball_sort_project)

이 문서는 **명령어와 절차만** 다룹니다. 코드 설명은 [project_summary.md](project_summary.md) / [project_easy.md](project_easy.md)를, 파일별 수정법은 [customize_guide.md](customize_guide.md)를 보세요.

이 패키지에는 실행 파일이 **2개** 있습니다.

| 실행 파일 | 하는 일 |
|---|---|
| `redball_tracking_project` | 빨간 공을 카메라로 쫓아가서 잡기 (1단계 프로젝트) |
| `box_sort_project` | 아루코 박스를 정해진 자리끼리 옮기기 (2단계 프로젝트, 현재 진행 중) |

---

## 0. 최초 1회 — 빌드

```bash
cd ~/kongju_manipulator_2026/ros_ws
colcon build --symlink-install --packages-select redball_sort_project
source install/setup.bash
```

코드(`.py`)만 고쳤다면 다시 빌드할 필요 없습니다 (`--symlink-install`이 바로가기로 연결해줌). 단 아래는 **다시 빌드해야** 반영됩니다:
- `setup.py` 수정 (실행 파일 이름 추가/삭제 등)
- `config/`, `models/` 안에 **새 파일을 추가**한 경우 (기존 파일 내용 수정은 다시 빌드 안 해도 됨 — 심볼릭 링크라 바로 반영됨)

---

## 1. (터미널 1) Gazebo 시뮬레이터 + 로봇 팔 띄우기

```bash
cd ~/open_manipulator_ws
source install/setup.bash
ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
```

- 콘솔에 `joint_state_broadcaster`, `arm_controller`, `gripper_controller` 관련 로그가 다 뜰 때까지(몇 초) 기다리세요.
- `MESA: error: ZINK: failed to choose pdev` / `glx: failed to create drisw screen` 로그는 **무시해도 됩니다** (자동으로 GPU로 전환됨). `LIBGL_ALWAYS_SOFTWARE` 같은 옵션은 절대 켜지 마세요 — 렉이 심해집니다.

---

## 2-A. 빨간 공 추적 (1단계 프로젝트) 실행

```bash
cd ~/kongju_manipulator_2026/ros_ws
source install/setup.bash
ros2 run redball_sort_project redball_tracking_project
```

- 실행한 **터미널 창에 마우스로 클릭해서 포커스를 준 상태**에서 키보드를 눌러야 합니다.
- 시작하면 항상 **MANUAL(수동) 모드**입니다.

| 키 | 동작 |
|---|---|
| `q`/`a` | joint1 +/- |
| `w`/`s` | joint2 +/- |
| `e`/`d` | joint3 +/- |
| `r`/`f` | joint4 +/- |
| `z` | 그리퍼 열기 |
| `x` | 그리퍼 닫기 |
| `m` | MANUAL ↔ AUTO 모드 전환 |
| `Ctrl+C` | 종료 |

**사용 순서**: MANUAL 키로 팔을 움직여서 `img` 창(OpenCV 창)에 빨간 공이 보이게 만든 다음 → `m`으로 AUTO 전환 → 로봇이 알아서 공을 쫓아가고 가까워지면 그리퍼로 잡음.

---

## 2-B. 아루코 박스 자리 이동 (2단계 프로젝트) 실행

```bash
cd ~/kongju_manipulator_2026/ros_ws
source install/setup.bash
ros2 run redball_sort_project box_sort_project
```

역시 **터미널 창에 포커스를 준 상태**에서 키보드를 눌러야 합니다. OpenCV 창이 **2개** 뜹니다.
- `camera`: 그리퍼(손목) 카메라. 아루코 마커를 인식하면 테두리와 대략적인 거리(m)가 표시됩니다. 눈으로 확인하는 용도일 뿐, 자리 이동 동작에는 영향 없습니다.
- `overhead`: 천장 고정 카메라. 작업공간 전체를 위에서 내려다보며, 마커가 보이면 "어느 자리에 있는지"까지 계산해서 표시합니다. **이 결과는 실제로 동작에 씁니다** — 이미 박스가 있는 자리로 옮기려고 하면 COMMAND가 거부됩니다.

지금은 **1~4번 네 자리**가 준비되어 있고, **박스도 2개**(`aruco_cube`=마커 id 0, `aruco_cube_2`=마커 id 1) 기본으로 놓여 있어서 TEACH 없이 바로 COMMAND부터 써봐도 됩니다.

### 2-B-1. 자리 가르치기 (TEACH)

| 키 | 동작 |
|---|---|
| `q`/`a`/`w`/`s`/`e`/`d`/`r`/`f` | 관절 수동 조작 (redball_tracking_project와 동일) |
| `z`/`x` | 그리퍼 열기/닫기 |
| `n` | "지금 가르칠 자리 번호" 커서를 다음 번호로 (1→2→3→4→1) |
| `b` | 커서를 이전 번호로 |
| `g` | **지금 팔 자세를 현재 커서 번호 자리로 저장** |

**가르치는 순서 예시** (1번 자리부터):
1. 프로그램을 켜면 커서가 1번에 맞춰져 있습니다 (터미널에 "지금: 1" 안내가 뜸).
2. `q/a/w/s/e/d/r/f`로 팔을 원하는 1번 위치(방향 1, 거리 1=가까움)로 조금씩 움직입니다.
3. 원하는 자세가 되면 `g`를 눌러 저장합니다. 콘솔에 "1번 자리 저장 완료: [...]" 로그가 뜨면 성공입니다.
4. `n`을 눌러 커서를 2번으로 바꾸고, 다시 팔을 움직여서 2번 자리 자세를 만든 뒤 `g`.
5. 지금은 4자리(필요하면 더 늘릴 수 있음)까지만 반복합니다.
6. `g`를 누를 때마다 [config/box_slots.yaml](config/box_slots.yaml)에 바로 저장되므로, 중간에 프로그램을 껐다 켜도 이미 가르친 자리는 남아있습니다.

**팁**: `n`/`b`는 팔을 움직이지 않고 "번호표"만 바꾸는 키입니다. 실수로 엉뚱한 자리에 `g`를 눌렀다면, 그 번호로 다시 `n`/`b`로 이동해서 올바른 자세를 만들고 `g`를 다시 누르면 덮어써집니다.

### 2-B-2. 박스 옮기기 (COMMAND)

자리를 2개 이상 가르쳤다면, 실제로 옮기는 명령을 내릴 수 있습니다. **출발 자리는 사람이 몰라도 됩니다** — 천장 카메라가 어느 마커가 어느 자리에 있는지 실시간으로 알고 있습니다.

1. `c` 키를 누르면 화면에 `[COMMAND 모드]`라는 안내와 함께 저장된 자리 번호 목록 + 지금 점유 현황이 뜨고, `>` 프롬프트가 나타납니다.
2. `마커id 도착자리` 형식으로 숫자 두 개를 입력하고 Enter를 누릅니다. 예: `0 3` (마커 id=0인 박스를 3번 자리로 옮김)
3. 로봇이 천장 카메라로 그 마커가 지금 어느 자리에 있는지 스스로 찾은 뒤, 자동으로 아래 순서를 수행합니다: 그리퍼 열기 → 출발 자리로 이동 → 그리퍼 닫기(집기) → 도착 자리로 이동 → 그리퍼 열기(놓기)
4. 완료되면 다시 TEACH 모드(키보드 관절 조작)로 돌아갑니다. 계속 `c`로 다른 명령을 내릴 수 있습니다.
5. 아무것도 입력하지 않고 Enter만 누르면 취소됩니다.

**주의**:
- 아직 안 가르친 도착 번호를 입력하면 "OO번 자리는 아직 안 가르쳤습니다" 로그만 뜨고 아무 일도 안 일어납니다 (안전 장치).
- 천장 카메라가 그 마커id를 못 찾으면 "마커 id=... 박스를 천장 카메라에서 못 찾았습니다" 로그가 뜨고 취소됩니다 (박스가 카메라 시야 밖에 있거나, 로봇 팔에 가려져 있을 수 있음).
- **도착 자리에 이미 다른 박스가 있으면** ("OO번 자리에는 이미 마커 id=... 박스가 있어서 이동할 수 없습니다" 로그) 마찬가지로 아무 일도 안 일어납니다 — 천장 카메라가 실시간으로 확인한 결과입니다.

---

## 2-C. 웹 페이지에서 조작하기 (키보드 대신)

터미널 키보드 조작이 번거롭다면, 같은 기능을 웹 페이지 버튼으로 쓸 수 있습니다. box_sort_project와 별도 프로그램이라 **1단계(Gazebo)는 그대로 켜둔 채로 대신 이 프로그램을 실행**하세요 (box_sort_project와 web_control을 동시에 띄우진 마세요 — 둘 다 로봇에게 동시에 명령을 보내면 꼬입니다).

```bash
cd ~/kongju_manipulator_2026/ros_ws
source install/setup.bash
ros2 run redball_sort_project web_control
```

터미널에 뜨는 주소(`http://localhost:8080`)를 브라우저에서 열면 됩니다. 같은 와이파이의 폰에서 접속하려면 `http://<이 컴퓨터의 IP>:8080`으로 접속하세요 (실행하면 터미널에 IP까지 같이 출력됩니다).

화면 구성:
- **카메라**: 천장 카메라(자리 점유 확인용) + 손목 카메라 화면을 실시간으로 볼 수 있습니다.
- **지금 상태**: 관절 각도 / TEACH 커서 / 자리 점유 현황이 1초마다 자동으로 갱신됩니다.
- **수동 조작(TEACH)**: joint1~4 +/- 버튼, 그리퍼 열기/닫기, 자리 커서 이전/다음, "지금 자세를 자리로 저장" 버튼 — 키보드 `q/a/w/s/e/d/r/f/z/x/n/b/g`와 완전히 같은 동작입니다.
- **박스 이동(COMMAND)**: 마커id와 도착 자리를 드롭다운에서 고르고 "이동" 버튼.

**주의**: 개발용 서버라 나 혼자 쓰는 용도로만 적합합니다. 인터넷에 공개하거나 여러 명이 동시에 쓰는 용도로는 안 맞습니다.

---

## 3. (선택) 상태 확인용 명령

```bash
source ~/kongju_manipulator_2026/ros_ws/install/setup.bash
ros2 topic list                              # /gripper_camera/image_raw, /overhead_camera/image_raw, /joint_states 등이 보여야 함
ros2 topic hz /gripper_camera/image_raw      # 손목 카메라가 실제로 흐르는지 확인
ros2 topic hz /overhead_camera/image_raw     # 작업공간을 위에서 내려다보는 고정 카메라
```

---

## 자주 만나는 문제

- **키를 눌러도 반응이 없음**: 터미널 창을 마우스로 한 번 클릭해서 포커스를 준 다음 눌러보세요.
- **`Action 서버를 찾지 못했습니다` 로그만 뜸**: 1단계(Gazebo)가 아직 다 안 켜졌거나 실행이 안 된 상태입니다. 1단계 콘솔이 안정될 때까지 기다린 후 다시 시도하세요.
- **팔이 자리에 도착하지 못하고 "중단합니다" 로그가 뜸**: `MOVE_TIMEOUT_SEC`(기본 35초) 안에 도착 못한 경우입니다. 컴퓨터 메모리/CPU 여유가 없으면(오래 켜둔 시뮬레이터가 많을 때) 이 타임아웃이 특히 잘 남 — Gazebo를 껐다 켜보세요. 그 외에는 목표 자세가 실제로 도달 가능한 값인지, 액션 서버가 살아있는지 확인하세요.
- **COMMAND 모드에서 글자가 이상하게 입력됨**: 터미널이 raw 모드에서 line 모드로 바뀌는 과정이라, 가끔 첫 글자가 씹힐 수 있습니다. 그럴 땐 Enter만 눌러서 취소하고 `c`를 다시 눌러보세요.
