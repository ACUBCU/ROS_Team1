# 이서현_project 기술 정리 — 빨간 공 추적 + 아루코 박스 자리 이동

이 패키지에는 실행 파일이 2개 있습니다: **`redball_tracking_project.py`**(1단계, 빨간 공을 카메라로 쫓아가서 잡기)와 **`box_sort_project.py`**(2단계, 아루코 박스를 정해진 자리끼리 옮기기). 이 문서는 둘 다 기술적으로 자세히 다룹니다. 쉽게 풀어쓴 버전은 [OVERVIEW.md](OVERVIEW.md), 실행 명령어만 보려면 [RUN_GUIDE.md](RUN_GUIDE.md), 파일별 수정법은 [CUSTOMIZE_GUIDE.md](CUSTOMIZE_GUIDE.md)를 보세요.

**중요 — 오늘 발견하고 고친 심각한 버그 4개**: 실제로 팔을 움직여서 검증하기 전까지는 팔이 사실상 전혀 제대로 안 움직이고 있었습니다. 자세한 내용은 맨 아래 ["오늘 찾은 버그 4개 (검증 과정)"](#오늘-찾은-버그-4개-검증-과정) 섹션을 꼭 읽어보세요 — 지금은 실제로 Gazebo에서 집기→이동→놓기까지 전부 확인된 상태입니다.

## 전체 파일 지도 — 이번에 무엇을 어디에 만들었나

이 프로젝트는 서로 다른 **두 개의 워크스페이스**에 걸쳐 있습니다. 헷갈리기 쉬우니 먼저 전체 그림부터 정리합니다.

```
~/kongju_manipulator_2026/                    ← 지금 이 git 저장소 (수업용 연습 공간)
└── ros_ws/src/
    ├── 이서현_project/                        ★이 프로젝트 전용 ROS2 패키지 (내부 패키지명: redball_sort_project)
    │   ├── redball_sort_project/
    │   │   ├── redball_tracking_project.py     1단계: 빨간 공을 따라가는 로봇 팔 코드
    │   │   └── box_sort_project.py             2단계: 아루코 박스 자리 이동 코드
    │   ├── models/red_ball/                    빨간 공의 3D 모양(모델) 정의
    │   │   ├── model.sdf                        (공의 크기, 색, 물리 속성)
    │   │   └── model.config                     (모델 이름표)
    │   ├── config/box_slots.yaml               가르친 자리 24곳의 저장 파일 (box_sort_project용)
    │   ├── HISTORY.md                  지금 보고 있는 이 설명 파일
    │   ├── OVERVIEW.md                     쉽게 풀어쓴 버전
    │   ├── RUN_GUIDE.md                        실행 명령어만 정리
    │   ├── CUSTOMIZE_GUIDE.md                  파일별 수정 방법
    │   ├── package.xml, setup.py, setup.cfg     ROS2 패키지 골격 (패키지 이름: redball_sort_project)
    │   └── resource/, test/                     ROS2 패키지 표준 구성 파일
    ├── camera_opencv/
    │   └── camera_opencv/find_redball.py       (원본, 힌트 주석만 있던 파일 — 안 건드림, 그대로 있음)
    └── tf2_basic/
        └── world/empty_world.sdf               (수정: 위 red_ball 모델을 불러오는 <include>)

~/open_manipulator_ws/                        ← 실제 로봇 팔(그리퍼 카메라 포함)을 Gazebo에 띄우는 별도 워크스페이스
                                                  (이 git 저장소 바깥, ROBOTIS에서 제공한 패키지)
└── src/open_manipulator/open_manipulator_bringup/
    └── worlds/empty_world.sdf                  (수정: 위의 red_ball 모델을 불러오는 <include> 추가)
```

**핵심만 요약**: 이 프로젝트와 관련된 파일은 전부 `이서현_project/` 폴더(ROS2 패키지명 `redball_sort_project`) 안에 모아뒀습니다. 빨간 공의 "생김새"(`models/red_ball/`)와 로봇 팔을 움직이는 실제 로직(`redball_sort_project/redball_tracking_project.py`)이 모두 이 패키지 안에 있습니다. 두 월드 파일(연습용 1개 + 실제 로봇용 1개)은 `package://redball_sort_project/models/red_ball`로 이 패키지의 모델을 가져다 씁니다.

**폴더 이름과 ROS2 패키지 이름이 다른 이유**: ROS2/파이썬 패키지 이름은 영어 소문자만 안전하게 지원돼서(한글 이름은 빌드 도구에 따라 깨질 수 있음), 바깥 폴더 이름은 원하신 대로 `이서현_project`로 하고, 내부적으로 colcon이 인식하는 실제 패키지 이름만 `redball_sort_project`로 지었습니다. `package.xml`의 `<name>` 태그가 진짜 이름이고, 폴더 이름은 참고용일 뿐이라 이렇게 나눠도 문제없이 빌드됩니다.

## 빠른 시작 (Quick Start)

자세한 설명 없이 명령어만 빨리 보고 싶을 때 여기만 보면 됩니다. (더 자세한 설명·문제 해결은 아래 "실행 방법 (처음부터 끝까지)" 참고)

**0) 최초 1회 — 빌드**
```bash
cd ~/kongju_manipulator_2026/ros_ws
colcon build --symlink-install --packages-select redball_sort_project
source install/setup.bash
```

**1) 터미널 1 — Gazebo + 로봇 팔 띄우기**
```bash
cd ~/open_manipulator_ws
source install/setup.bash
ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
```
콘솔에 컨트롤러 관련 로그가 다 뜰 때까지(몇 초) 기다리기.

**2) 터미널 2 — 빨간 공 추적 노드 실행**
```bash
cd ~/kongju_manipulator_2026/ros_ws
source install/setup.bash
ros2 run redball_sort_project redball_tracking_project
```
OpenCV 창(`img`, `mask`)이 뜨고 로그가 찍히기 시작하면 정상. **시작하면 항상 MANUAL(수동) 모드**이니, 아래 "MANUAL/AUTO 모드 사용법"을 봐서 먼저 카메라가 공을 보도록 팔을 움직인 다음 `m`으로 AUTO를 켜세요.

## MANUAL / AUTO 모드 사용법

이 노드를 실행한 **터미널 2 창에 마우스로 클릭해서 포커스를 준 상태**에서 키보드를 누르면 됩니다 (다른 창에 포커스가 있으면 키 입력이 안 먹습니다).

| 키 | 동작 |
|---|---|
| `q` / `a` | joint1 +/- (좌우 회전) |
| `w` / `s` | joint2 +/- |
| `e` / `d` | joint3 +/- |
| `r` / `f` | joint4 +/- |
| `z` | 그리퍼 열기 |
| `x` | 그리퍼 닫기 |
| `m` | **MANUAL ↔ AUTO 모드 전환** |
| `Ctrl+C` | 종료 |

- **MANUAL(수동) 모드** (시작 시 기본값): 위 키로 사람이 직접 관절 하나씩 움직입니다. `keyboard_manipulator.py`(tf2_basic 패키지)와 조작법이 동일합니다.
- **AUTO(자동) 모드**: `m`을 눌러 전환하면, 카메라가 빨간 공을 보고 알아서 관절을 움직여 따라갑니다. MANUAL 키(q/a/w/s/e/d/r/f)는 이 모드에서는 무시됩니다 (충돌 방지). `z`/`x`(그리퍼)와 `m`(모드 전환)만 계속 작동합니다.
- **왜 시작이 MANUAL이냐면**: 로봇 팔의 기본 자세에서는 그리퍼 카메라가 가까운 바닥을 못 봅니다(아래 "카메라가 아무것도 안 보일 때" 참고). AUTO로 바로 시작하면 카메라가 아무것도 못 본 채로 대기만 하거나, 잘못된 자세 추정으로 위험하게 움직일 수 있어서, 사람이 먼저 카메라를 공 쪽으로 향하게 하고 나서 AUTO로 넘어가도록 설계했습니다.
- 두 모드 모두 화면에 빨간 공이 보이기만 하면 `공 중심 좌표`, `공 면적` 로그는 항상 찍힙니다 (팔을 실제로 움직이는 것만 AUTO에서만 일어남).

## box_sort_project.py — 아루코 박스 자리 이동 (2단계 프로젝트)

빨간 공 프로젝트 다음으로 만든, 매니퓰레이터 주변 8방향 × 거리 3단계 = **24개 자리**끼리 아루코 박스를 옮기는 프로젝트입니다. 색으로 구분하는 대신 위치(자리 번호)로 구분합니다.

**두 모드**:
- **TEACH**: `n`/`b`로 "지금 가르칠 자리 번호" 커서를 움직이고, 원하는 자세로 팔을 옮긴 뒤 `g`로 저장. `config/box_slots.yaml`에 즉시 기록됩니다.
- **COMMAND**: `c`로 진입해서 `"출발자리 도착자리"`(예: `1 7`)를 입력하면, 그리퍼 열기 → 출발자리 이동 → 집기 → 도착자리 이동 → 놓기를 자동 수행합니다.

**자리 저장 방식**: `get_package_share_directory('redball_sort_project')`로 install 경로를 찾는데, `--symlink-install`로 빌드했다면 그 경로가 `src/config/box_slots.yaml`을 직접 가리키는 심볼릭 링크라서, 프로그램이 저장한 내용이 그대로 src 쪽 원본 파일에 남습니다 (직접 테스트해서 확인함 — 표준 로드→저장 왕복 테스트로 3번 자리를 저장해봤고 파일에 정확히 반영되는 것까지 확인).

키보드 처리 방식(`select`/`termios`/`tty` 조합, raw 모드는 루프 시작 전 딱 한 번만 켜기)은 `redball_tracking_project.py`와 동일한 원리를 씁니다 — 이유는 그 파일 개발 중 실제로 겪었던 "매 틱마다 raw/일반 모드를 껐다 켰다 하면 키 입력을 거의 못 읽는" 버그를 여기서는 처음부터 피한 것입니다.

### 카메라 / 아루코 인식

`/gripper_camera/image_raw`를 구독해서 매 프레임 `cv2.aruco.detectMarkers`로 마커(`aruco_cube` 모델이 쓰는 `DICT_4X4_50`, id=0)를 찾고, 찾으면 `camera`라는 OpenCV 창에 테두리·좌표축·거리(m)를 그려서 보여줍니다. 지금은 자리 이동 로직에는 이 인식 결과를 쓰지 않고(자리는 여전히 번호로만 구분) 눈으로 확인하는 용도입니다.

- **거리 추정 방식**: 카메라를 실제로 캘리브레이션한 게 아니라, SDF에 적힌 시야각(`horizontal_fov=1.0472rad`=60°)과 이미지 크기(640x480)로부터 핀홀 카메라 모델 공식(`fx = (가로/2) / tan(시야각/2)`)으로 초점거리를 역산한 **근사 카메라 행렬**을 씁니다. `cv2.aruco.estimatePoseSingleMarkers`로 마커까지의 변환벡터(tvec)를 구하고 그 크기(norm)를 거리로 표시합니다. 정밀 계측값이 아니라 참고용 근사치입니다.
- **중요한 발견 — 마커는 박스 "윗면"에 있음**: `aruco_cube` 모델의 마커(`aruco_top_visual`)는 큐브 윗면에 위쪽을 보고 붙어 있습니다. 그래서 팔이 기본 자세처럼 카메라가 거의 수평으로 보는 상태에서는 마커가 옆에서 거의 안 보이는 각도라 인식이 안 됩니다. 실제로 테스트박스를 놓고 확인해보니, 홈 자세(수평 시야)에서는 마커가 감지되지 않았고, **그리퍼가 위에서 아래로 접근하는 자세(예: taught 자리 중 mid/far처럼 아래를 보는 자세)에서만 마커 전체가 화면에 들어오고 정상 인식됐습니다** (`id=0, 거리≈0.156m` 확인). 즉 실제 박스 집기 동작 중 그리퍼가 위에서 내려다보는 순간에 자연스럽게 인식되는 구조라, 지금 자리 이동 방식과도 맞물립니다.
- 마커가 카메라에 너무 가까우면 화면 밖으로 잘려서 인식이 안 될 수 있습니다 (실제로 겪음 — 박스와 그리퍼가 거의 겹치는 위치에서는 실패, 조금 떨어뜨리니 성공).

### 24개 자리를 자동으로 가르친 방법 (TF 기반 안전 탐색)

원래 설계는 사람이 TEACH 모드로 24곳을 전부 손으로 가르치는 것이었지만, 실제로는 제가 스크립트로 대신 채웠습니다. **다만 예전에 겪었던 "감으로 관절값을 보냈다가 팔이 바닥으로 향한 사고"를 반복하지 않기 위해, 아래 절차로 안전을 확보했습니다:**

1. 후보 관절값(joint2~4)으로 팔을 움직인 뒤, `tf2_ros.Buffer`/`TransformListener`로 `world → end_effector_link`의 **실제 3D 좌표**를 매번 조회함 (계산이 아니라 시뮬레이터의 실측값).
2. 높이(z)가 안전선 아래로 내려가면 즉시 그 자세를 버리고 이전 값을 유지.
3. joint1(허리 회전)은 축 자체가 회전만 시킬 뿐 반지름(r)·높이(z)에는 영향을 주지 않는다는 점(순수 Z축 회전)을 이용해서, "거리 3단계"에 대한 안전한 (joint2,joint3,joint4) 조합을 딱 3번만 탐색하면 됨 — 8방향은 joint1 값만 바꿔서 복사하면 되므로 24번이 아니라 3번의 실측 탐색으로 끝남.
4. 여러 각도(0°, 45°, 180°, -135°)에서 실제로 r/z가 동일하게 유지되는지 재검증한 뒤, 24개 전체를 생성해서 저장.
5. 실제 저장된 자리 2개(2번 mid ↔ 20번 near, 서로 반대 방향)로 `run_command`를 돌려서 전체 시퀀스(그리퍼 열기→이동→집기→이동→놓기)가 8.3초 만에 정상 완료되는 것까지 확인.

**실측 결과** (joint1=0 기준, 다른 방향은 joint1만 다름):

| 거리 단계 | joints (j2,j3,j4) | 실측 반지름 r | 실측 높이 z |
|---|---|---|---|
| 1 (가까움) | `0.30, 0.70, -1.25` | ≈0.25~0.26m | ≈0.10~0.11m |
| 2 (중간) | `0.85, 0.05, -0.85` | ≈0.32~0.33m | ≈0.03~0.04m |
| 3 (멈) | `1.10, -0.20, -0.70` | ≈0.34~0.35m | ≈0.03m |

**알아둘 점 (물리적 한계, 버그 아님)**: "가까움" 단계는 이 팔의 구조상 바닥 근처까지 못 내려갑니다. joint4를 더 접어서 낮추려고 시도하면 오히려 다시 높아지는 것을 실측으로 확인했습니다 (팔꿈치가 접히는 궤적이 최저점을 지나 다시 올라가는 구간에 들어감). 즉, 가까운 거리에서는 상자를 바닥이 아니라 살짝 위쪽에서 집는다고 생각하면 됩니다.

이 결과는 이미 `config/box_slots.yaml`에 24개 다 채워진 상태로 저장되어 있어서, 지금 바로 `c` 키로 COMMAND 모드에 들어가 `1 7` 같은 명령을 시험해볼 수 있습니다. TEACH 모드(`n`/`b`/`g`)는 그대로 남아있으니, 특정 자리가 마음에 안 들면 언제든 손으로 다시 가르쳐서 덮어쓸 수 있습니다.

## 원본 파일과의 관계

[find_redball.py](../camera_opencv/camera_opencv/find_redball.py)(camera_opencv 패키지)에는 빨간 공을 화면에서 찾아 네모 박스를 그려주는 부분(`image_callback`)까지만 있고, 그 아래에 "이제 이 좌표로 관절을 움직이세요", "면적으로 거리를 추측해서 로그를 찍으세요" 같은 힌트 주석만 남아있었습니다. [redball_tracking_project.py](redball_sort_project/redball_tracking_project.py)는 그 힌트들을 실제로 동작하는 코드로 채운 새 파일입니다. 원본 파일은 건드리지 않았습니다.

## 비유로 이해하기

사람이 눈앞의 공을 볼 때를 떠올려보세요.
- 공이 화면 왼쪽에 있으면 고개를 왼쪽으로 돌린다 → **joint1**을 돌림
- 공이 화면 아래쪽에 있으면 팔을 아래로 숙인다 → **joint2~4**를 움직임
- 공이 눈에 크게 보이면(=가까이 있으면) 손을 뻗어 잡는다 → **그리퍼**를 닫음

이 노드는 카메라 이미지가 들어올 때마다(`image_callback`) 이 세 가지 판단을 반복합니다.

## 동작 순서 (이미지 한 장마다 반복)

1. 이미지에서 빨간색만 골라내서(HSV 마스크) 공의 중심 좌표(x, y)와 크기(area)를 찾음
2. 공 중심 좌표, 면적(거리 지표) 로그 출력 — **여기까지는 MANUAL/AUTO 상관없이 항상 실행**
3. (AUTO 모드일 때만) 화면 중심과 공 중심의 차이(오차)를 계산
4. (AUTO 모드일 때만) 오차만큼 `joint1`(좌우), `joint2~4`(상하)를 아주 조금씩 움직임 — 한 프레임에 너무 많이 움직이지 않도록 `MAX_STEP`으로 제한
5. (AUTO 모드일 때만) 면적이 `GRAB_AREA`보다 커지면(충분히 가까워지면) 그리퍼를 닫아 공을 잡음 (한 번만)

## 원본 대비 고친 부분

- **빨간색 검출 범위 확장**: HSV에서 빨간색은 색상값(Hue) 0 근처와 180 근처 양 끝에 걸쳐 있는데, 원본은 0~10 구간만 봤습니다. 170~180 구간을 추가해서 더 잘 잡히도록 했습니다.
- **`cv2.boundingRect` 반환 순서**: 원본은 `x, y, h, w = cv2.boundingRect(...)`로 순서가 바뀌어 있었는데(실제 반환 순서는 `x, y, w, h`), 새 파일에서는 올바른 순서로 받습니다.
- **목표 중복 전송 방지**: 카메라 콜백은 초당 수십 번 호출될 수 있어서, 이전에 보낸 팔 움직임 목표(action goal)의 응답이 오기 전까지는 새 목표를 보내지 않도록(`arm_goal_in_progress`) 막았습니다. 안 그러면 액션 서버에 목표가 계속 쌓여서 팔이 뚝뚝 끊기듯 움직입니다.

## 한계 (알고 써야 하는 부분)

- `joint2~4`에 전부 같은 값을 더하는 방식은 실제 로봇의 역기구학(inverse kinematics) 계산이 아니라, 초심자용으로 단순화한 방식입니다. 실제 팔로 테스트했을 때 방향이 이상하면 `redball_tracking_project.py`의 `Y_GAIN` 부호를 바꿔가며 확인해보세요.
- `X_GAIN`, `Y_GAIN`, `MAX_STEP`, `GRAB_AREA` 값은 실제 카메라/팔 반응을 보면서 조정이 필요한 튜닝값입니다. 임의로 정한 초기값일 뿐입니다.

## Gazebo 공 모델 자세히 보기 — `models/red_ball/`

### `model.sdf` (공의 생김새 정의)

```xml
<model name='red_ball'>
  <static>false</static>              <!-- false = 중력의 영향을 받아 굴러다닐 수 있음 (true면 벽처럼 고정) -->
  <link name='ball_link'>
    <inertial>                        <!-- 질량/관성: 물리엔진이 "이 물체가 얼마나 무겁고, 얼마나 돌리기 힘든지" 계산할 때 씀 -->
      <mass>0.05</mass>               <!-- 50g. 탁구공 정도의 무게 -->
      <inertia>...</inertia>          <!-- 속이 꽉 찬 구(solid sphere) 공식 2/5 * m * r^2 으로 계산한 값 -->
    </inertial>
    <collision name='ball_collision'> <!-- "충돌 판정"용 모양. 그리퍼가 닿았는지, 바닥에 부딪혔는지 계산할 때 이 모양을 씀 -->
      <geometry><sphere><radius>0.02</radius></sphere></geometry>  <!-- 반지름 2cm(지름 4cm) -->
      <surface><friction><ode><mu>15</mu><mu2>15</mu2></ode></friction></surface>
      <!-- mu, mu2 = 마찰 계수. 값이 높을수록 미끄러지지 않고 그리퍼에 잘 붙잡힘 (aruco_cube 모델과 같은 값 사용) -->
    </collision>
    <visual name='ball_visual'>       <!-- "눈에 보이는" 모양. 충돌 판정과 별개로, 화면에 그려지는 모습만 담당 -->
      <geometry><sphere><radius>0.02</radius></sphere></geometry>
      <material>
        <ambient>1 0 0 1</ambient>    <!-- R G B A 순서, 1 0 0 1 = 순수한 빨간색 -->
        <diffuse>1 0 0 1</diffuse>
        <specular>0.3 0.3 0.3 1</specular>
      </material>
    </visual>
  </link>
</model>
```

**왜 collision과 visual을 따로 정의하나?**: 비유하면 collision은 "만질 수 있는 몸통"이고 visual은 "겉모습(피부색)"입니다. 보통은 둘을 똑같은 모양으로 맞추지만, 이론적으로는 다르게 만들 수도 있습니다 (예: 충돌 판정은 단순한 상자로 하고, 겉모습만 복잡한 3D 모델로 보여주기).

### `model.config` (모델의 이름표/설명서)

Gazebo가 `model://red_ball` 이라는 이름을 보고 실제 폴더를 찾을 때 참고하는 메타데이터 파일입니다. `<sdf version="1.9">model.sdf</sdf>` 줄이 "이 모델의 실제 3D 정의는 같은 폴더의 `model.sdf` 파일에 있다"고 알려주는 역할을 합니다.

## 두 월드 파일에 추가한 `<include>` 자세히 보기

두 곳 모두 아래와 같은 형태로 "모델 이름 + 이름표(name) + 놓을 위치(pose)"를 지정해서 공을 세상에 등장시킵니다.

```xml
<include>
  <uri>model://red_ball</uri>      <!-- 어떤 모델을 불러올지 (GZ_SIM_RESOURCE_PATH 안에서 이 이름의 폴더를 찾음) -->
  <name>red_ball_1</name>          <!-- 이 월드 안에서 이 공을 부를 이름 (같은 모델을 여러 번 놓을 때 구분용) -->
  <pose>0.2 0 0.02 0 0 0</pose>    <!-- x=0.2m y=0 z=0.02m, 회전 없음(0 0 0). z=0.02는 공 반지름만큼 띄워서 바닥에 딱 닿게 한 값 -->
</include>
```

**중요한 시행착오**: 처음엔 `package://redball_sort_project/models/red_ball` 형태(ROS2 ament 인덱스로 패키지를 직접 찾는 방식)로 바꿨었는데, **이게 world 파일을 완전히 고장냈습니다.** world 파일은 ROS 없이 `gz sim`이 직접 읽기 때문에 `package://` 스킴 자체를 이해하지 못하고(`Unable to find uri` 에러), Gazebo가 아예 켜지지도 못하고 죽었습니다. 그래서 원래 방식인 `model://이름`으로 되돌렸고, 대신 `open_manipulator_x_gazebo.launch.py`에서 `GZ_SIM_RESOURCE_PATH` 환경변수에 `redball_sort_project`의 `models` 폴더 경로를 추가해서 `model://red_ball`이 찾아지도록 했습니다.

## 실행 방법 (처음부터 끝까지)

이 프로젝트를 실제로 돌려보려면 터미널을 **3개** 띄워야 합니다. 하나는 시뮬레이터(Gazebo)를 켜는 용도, 하나는 우리가 만든 추적 코드를 실행하는 용도, 나머지는 상태 확인용입니다.

### 0단계. 사전 준비 (최초 1번, 또는 코드를 고칠 때마다)

새로 만들거나 고친 코드는 반드시 **빌드(build)** 를 해야 실제로 반영됩니다. 빌드란, 우리가 쓴 파이썬/XML 파일들을 ROS2가 실행할 수 있는 형태로 `install/` 폴더에 복사·정리하는 과정이라고 생각하면 됩니다. (비유: 원고를 고쳤으면 인쇄소에 다시 넘겨서 새로 인쇄해야 하는 것과 같음 — `src/`가 원고, `install/`이 인쇄된 책)

```bash
cd ~/kongju_manipulator_2026/ros_ws
colcon build --symlink-install --packages-select redball_sort_project tf2_basic camera_opencv
source install/setup.bash
```

(파일을 `이서현_project`로 옮기면서 `tf2_basic`, `camera_opencv`에도 옛날 흔적이 install 폴더에 남아있을 수 있어서, 처음 한 번은 이 세 패키지를 같이 빌드하는 게 안전합니다. 이후에는 `redball_sort_project`만 다시 빌드하면 됩니다.)

`--symlink-install`을 쓰면 파이썬 파일(.py)은 복사가 아니라 "바로가기(symlink)"로 연결되기 때문에, 이후 `.py` 코드만 고칠 때는 다시 빌드하지 않아도 됩니다. 다만 `model.sdf`, `model.config`, `setup.py`처럼 **새 파일을 추가했거나 setup.py의 entry_points를 바꾼 경우**에는 다시 빌드해야 합니다.

### 1단계. (터미널 1) Gazebo 시뮬레이터 + 로봇 팔 띄우기

```bash
cd ~/open_manipulator_ws
source install/setup.bash
ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
```

- Gazebo 창이 뜨고, 로봇 팔이 원점(0,0,0)에 나타납니다.
- 팔 앞쪽 20cm 지점 바닥에 **빨간 공**이 보이면 성공입니다.
- 콘솔에 `joint_state_broadcaster`, `arm_controller`, `gripper_controller` 관련 로그가 순서대로 뜨면서 컨트롤러들이 켜집니다. 이 로그가 다 뜰 때까지(몇 초) 기다려주세요 — 안 그러면 다음 단계에서 "액션 서버를 찾지 못했습니다" 로그가 뜰 수 있습니다.

### 2단계. (터미널 2) 빨간 공 추적 노드 실행

```bash
cd ~/kongju_manipulator_2026/ros_ws
source install/setup.bash
ros2 run redball_sort_project redball_tracking_project
```

- OpenCV 창 2개(`img`, `mask`)가 뜹니다. `img`는 그리퍼 카메라가 보는 실제 화면, `mask`는 그중 빨간색만 하얗게 표시한 화면입니다.
- 콘솔에 `공 중심 좌표: x=..., y=...`, `공 면적(거리 지표, 클수록 가까움): ...` 로그가 반복해서 찍히면 카메라가 공을 잘 찾고 있는 것입니다.
- 팔이 공을 화면 중앙에 맞추려고 천천히 움직이기 시작하면 정상 동작입니다.

### 3단계 (선택). (터미널 3) 상태 확인용

문제가 있을 때 아래 명령으로 실제 토픽/액션이 살아있는지 확인할 수 있습니다.

```bash
source ~/kongju_manipulator_2026/ros_ws/install/setup.bash
ros2 topic list          # /gripper_camera/image_raw, /joint_states 등이 보여야 함
ros2 topic hz /gripper_camera/image_raw   # 카메라 이미지가 실제로 흐르고 있는지 (숫자가 계속 찍히면 정상)
```

### 자주 만나는 문제

- **Gazebo에 공이 안 보임**: 0단계에서 `tf2_basic`을 다시 빌드하지 않았을 가능성이 큽니다 (모델은 `install/` 폴더 기준으로 찾기 때문에 `src/`에만 파일을 만들어 둔 상태로는 안 보입니다).
- **`gripper_controller Action 서버를 찾지 못했습니다` 로그만 계속 뜸**: 1단계 Gazebo 쪽 컨트롤러들이 아직 다 안 켜졌거나, 1단계를 아예 실행하지 않은 상태에서 2단계만 실행한 경우입니다. 1단계 콘솔 로그가 안정될 때까지 기다린 후 2단계를 실행하세요.
- **OpenCV 창(`img`, `mask`)이 안 뜸 / 새까맣기만 함**: `/gripper_camera/image_raw` 토픽이 실제로 발행되고 있는지 위 3단계 명령으로 확인해보세요. 안 뜬다면 1단계 Gazebo 로딩이 아직 안 끝났거나 실패한 것입니다.
- **팔이 아예 안 움직임**: 아직 공을 못 찾았거나(`MIN_BALL_AREA`보다 작게 보임), `joint_states`를 아직 못 받은 상태(`아직 joint_states를 못 받아서 대기 중...` 로그)일 수 있습니다.
- **`mask` 창은 계속 까맣고 로그가 하나도 안 찍힘 (카메라는 정상 작동 중인데도)**: 아래 "카메라가 아무것도 안 보일 때" 참고 — 팔 기본 자세 문제일 가능성이 큽니다.

## 실제로 Gazebo에 연결하면서 추가로 고친 것들

`redball_tracking_project.py`와 공 모델만으로는 실제 시뮬레이터에서 동작하지 않았습니다. `open_manipulator_ws`(별도 워크스페이스)에 원래 있던 `Add_camera.md` 체크리스트 중 **3곳이 실제로는 적용이 안 되어 있었고**, 이번에 확인하면서 마저 고쳤습니다.

1. **`open_manipulator_bringup/worlds/empty_world.sdf`에 `gz::sim::systems::Sensors` 플러그인 누락** — 이 플러그인이 없으면 Gazebo가 카메라 센서 자체를 렌더링하지 않습니다. GPU든 CPU든 상관없이 이게 없으면 이미지가 원천적으로 안 나옵니다.
2. **`open_manipulator_x_gazebo.launch.py`의 `ros_gz_bridge`에 카메라 토픽 누락** — `/clock`만 Gazebo↔ROS2로 통역하도록 되어 있었고, `/gripper_camera/image_raw`, `/gripper_camera/camera_info`는 통역 목록에 없었습니다. Gazebo 내부에서는 카메라가 촬영해도 ROS2 쪽 토픽으로는 전혀 안 넘어오는 상태였습니다.
3. **`open_manipulator_description`에 그리퍼 카메라 자체가 없었음** — `open_manipulator_x.gazebo.xacro`(카메라 센서 정의)와 `open_manipulator_x_arm.urdf.xacro`(camera_link/camera_joint)에 카메라 관련 내용이 아예 없었습니다. 로봇 모델 자체에 카메라가 안 달려 있었던 셈입니다.

세 가지 다 `Add_camera.md`에 정확히 뭘 고쳐야 하는지 이미 적혀 있었는데, 실제 파일에는 반영이 안 된 상태였습니다. 지금은 문서에 적힌 대로 세 곳 다 적용해뒀습니다.

## GPU 렌더링 관련 — `LIBGL_ALWAYS_SOFTWARE` 쓰지 마세요

Gazebo를 켜면 콘솔에 아래 에러가 뜹니다:
```
MESA: error: ZINK: failed to choose pdev
glx: failed to create drisw screen
```
**이건 무시해도 되는 경고입니다.** Mesa(그래픽 라이브러리)가 처음에 Zink(Vulkan 경유 OpenGL)로 시도했다가 실패하면, 자동으로 **D3D12 드라이버로 전환해서 실제 GPU(NVIDIA)를 정상적으로 사용**합니다. `glxinfo -B`로 확인하면 `OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 3050 Ti Laptop GPU)`가 나오고, `nvidia-smi`로도 GPU 사용률이 잡힙니다.

`LIBGL_ALWAYS_SOFTWARE=1` 환경변수를 강제로 켜면 이 자동 GPU 전환을 막고 CPU 소프트웨어 렌더링(느림, 렉 심함)으로 돌리게 됩니다 — **켜지 마세요.** 그냥 `ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py`만 실행하면 됩니다.

## 카메라가 아무것도 안 보일 때 (팔 기본 자세 문제)

로봇 팔이 시작 자세(모든 관절 각도 ≈ 0)일 때, 그리퍼 카메라는 **거의 수평으로 먼 곳을 바라봅니다.** TF로 확인해보면 카메라 위치는 `world` 기준 `(0.23, 0, 0.237)`이고 회전은 거의 없습니다(수평). 카메라 높이 23.7cm, 시야각(수직 약 47도)을 계산해보면, **카메라 바로 앞 약 55cm 이내의 바닥은 화면에 아예 안 잡힙니다** (화면 아래쪽 프레임 밖으로 벗어남). 처음에 공을 팔 앞 20cm에 놨었는데, 이 계산 범위보다 가까워서 전혀 안 보였던 것입니다.

**해결 방법 2가지:**
1. **MANUAL 모드로 팔을 숙이기**: `w`/`e`/`r` 등으로 팔을 앞으로 굽혀서 카메라가 아래(가까운 바닥)를 보게 만듭니다. *주의*: 관절 값을 감으로 크게 바꾸면 팔이 바닥에 처박힐 수 있습니다. 한 번에 조금씩(`STEP=0.05`만큼) 움직이면서 `img` 창을 계속 보세요.
2. **공을 카메라가 보는 위치로 옮기기**: Gazebo 창 툴바의 이동 도구(단축키 `T`)로 공을 클릭해서 드래그하거나, 터미널에서 아래처럼 직접 좌표를 지정할 수 있습니다.
   ```bash
   source ~/open_manipulator_ws/install/setup.bash
   gz service -s /world/empty/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
     --req 'name: "red_ball_1", position: {x: 1.0, y: 0.0, z: 0.02}, orientation: {x: 0, y: 0, z: 0, w: 1}'
   ```
   (팔 기본 자세 기준으로는 카메라에서 약 1m 앞이 화면에 잘 잡힙니다. 다만 그 정도 거리는 실제로 팔이 닿는 범위 밖이라 "잡기"까지 확인하려면 방법 1로 카메라를 더 가까운 곳을 보게 만드는 게 낫습니다.)

## 오늘 찾은 버그 4개 (검증 과정)

"진짜 다 되는지 확인했냐"는 질문을 받고 실제로 Gazebo에 붙여서 `run_command`(집기→이동→놓기)를 직접 돌려본 끝에, **팔이 사실상 한 번도 제대로 움직인 적이 없었다**는 걸 발견했습니다. 순서대로 4개의 버그가 겹쳐 있었습니다.

### 버그 1 — `/joint_states`의 순서를 잘못 가정함

`/joint_states` 메시지는 `[gripper_left_joint, gripper_right_joint, joint1, joint2, joint3, joint4]` 순서로 옵니다 (그리퍼가 먼저!). 그런데 코드는 `self.current_joint_position = list(msg.position)`처럼 **그대로** 받아서 `[0]`을 joint1인 것처럼 썼습니다. 실제로는 그리퍼 값을 joint1으로 착각하고 있었던 것입니다.

**고친 법**: `keyboard_manipulator.py`가 이미 쓰고 있던 방식대로, 이름으로 정확히 찾아서 순서를 다시 맞췄습니다.
```python
name_to_position = dict(zip(msg.name, msg.position))
self.current_joint_position = [name_to_position[name] for name in JOINT_NAMES]  # JOINT_NAMES = ["joint1","joint2","joint3","joint4"]
```

### 버그 2 — 관절 이름 4개에 좌표값 6개를 보냄

버그 1의 결과로 `self.current_joint_position`이 6개짜리 리스트였는데, 이걸 그대로 `point.positions`에 넣어서 보냈습니다. 그런데 `goal.trajectory.joint_names`는 `["joint1","joint2","joint3","joint4"]` 4개뿐이었습니다. **이름 4개에 좌표 6개**를 보내는 것 자체가 모순된 목표였던 것입니다 (버그 1을 고치면서 자동으로 같이 해결됨).

### 버그 3 — 목표가 거절돼도 영원히 "응답 대기 중"으로 남음

`goal_joint_callback`에서 `goal_handle.accepted` 여부를 확인하지 않고 바로 `get_result_async()`를 불렀습니다. 목표가 거절되면 결과 콜백이 아예 안 오는데, `arm_goal_in_progress` 플래그는 계속 `True`로 남아서 다음 명령을 영원히 못 보내게 됩니다.

**고친 법**: `goal_handle.accepted`가 `False`면 그 자리에서 바로 `arm_goal_in_progress = False`로 풀어주고 로그를 남기도록 함.

### 버그 4 (진짜 원인) — 시뮬레이션 시간과 실제 시간이 안 맞음

가장 찾기 어려웠던 버그입니다. `goal.trajectory.header.stamp = self.get_clock().now().to_msg()`로 목표에 **현재 시각(wall-clock)** 을 채워서 보냈는데, Gazebo 쪽 `arm_controller`는 `use_sim_time=true`로 **시뮬레이션 시간**을 씁니다. 두 시계가 서로 다른 기준이다 보니, 컨트롤러가 "이 궤적을 언제 시작해야 하나"를 계산 못 하고 그냥 무한정 대기 상태에 빠졌습니다 — 목표는 `accepted=True`로 받아들여지는데 결과가 몇십 초를 기다려도 절대 안 왔습니다.

**검증 과정**: `ros2 action send_goal` CLI로 직접 같은 목표를 보내보니 1초 안에 성공했고, 우리 코드와 똑같은 조건(같은 노드, 같은 `/joint_states` 구독)을 재현한 실험 스크립트에서 `header.stamp`에 일부러 현재 시각을 채워보니 **정확히 같은 증상**(accepted는 되는데 결과가 안 옴)이 재현됐습니다. `header.stamp`를 아예 안 채우면(기본값 0 = "받는 즉시 시작") 정상 동작했습니다.

**고친 법**: `goal.trajectory.header.stamp` 줄을 아예 삭제 (기본값 유지).

### 최종 검증 결과

버그 4개를 전부 고친 뒤, 실제 Gazebo에 붙여서 `box_sort_project.py`의 `run_command(1, 2)`를 직접 호출해봤습니다:
```
그리퍼 이동 성공: 0.019...
팔 이동 성공: Goal successfully reached!
그리퍼 이동 성공: -0.0009...
팔 이동 성공: Goal successfully reached!
그리퍼 이동 성공: 0.0099...
1번 -> 2번 이동 완료      (총 4.9초 소요, joint1 최종값 0.29997 ≈ 목표 0.3)
```
`redball_tracking_project.py`의 AUTO 모드 쪽 팔 이동(`move_joint`, 콜백 기반 fire-and-forget 방식)도 별도로 직접 호출해서 `팔 이동 성공: Goal successfully reached!`까지 확인했습니다 (약 1초 소요, joint1이 목표값 0.2로 정확히 도착).

**단, 키보드 입력(TEACH/COMMAND 모드로 실제 타자 치는 부분)은 실제 tty가 필요해서 자동 테스트가 불가능하고, 사람이 터미널에서 직접 눌러봐야 확인됩니다.** 그 외 팔/그리퍼 동작 로직은 전부 실제로 재현 검증했습니다.

## 버그 5 — 액션이 "성공"해도 실제로는 목표에 못 미친 채 멈춤 (진짜 집기 테스트에서 발견)

"진짜 검증됐냐"는 질문을 받고 실제로 박스를 놓고 집는 것까지 확인해보니, **집는 데 실패**했습니다. 원인을 추적한 결과:

1. **`end_effector_link`은 손가락이 실제로 오므라드는 지점이 아닙니다.** URDF상 `link5`에서 0.126m 떨어진 위치인데, 그리퍼 손가락 조인트 원점은 0.0817m 지점이라 **약 4.4cm 차이**가 있습니다. 자리를 가르칠 때 `end_effector_link` 기준으로 좌표를 쟀었는데, 실제 집는 지점은 그보다 안쪽/아래였습니다.
2. **`FollowJointTrajectory` 액션이 `SUCCEEDED`를 반환해도 실제 각도가 목표에 못 미친 채 멈추는 경우가 있습니다.** `gripper_left_link`/`gripper_right_link`의 TF 중점을 직접 재보니, "mid" 자세에서 joint2가 목표 0.85 대신 0.68~0.71에서 멈춘 걸 발견했습니다. 위치 게인이 낮아서(P 제어 추정) 무게가 실리는 자세일수록 정상상태 오차가 남는 것으로 보입니다.

**고친 법**: `_move_and_wait()`를 "보내고 끝"이 아니라 "보내고 → `joint_states`로 실제 도착 여부 확인 → 오차가 있으면 그 오차만큼 보정해서 재전송 (최대 3회)" 방식으로 바꿨습니다 (`_send_trajectory_once` + `_move_and_wait`의 재시도 루프, `POSITION_TOLERANCE = 0.03`rad).

**추가로 발견한 물리적 한계**: joint2는 약 0.85rad 근처가 재시도를 해도 못 넘어가는 실질적 한계였습니다 (원래 "far" 템플릿이 joint2=1.1을 썼는데 안 됨). joint3/joint4로 반지름을 더 늘려보려는 시도들도 mid(r≈0.327)보다 나은 결과가 안 나와서, **이 팔은 그리퍼 높이 기준 수평 반지름 한계가 약 0.33m 정도**라고 결론 내렸습니다. `far` 템플릿을 `joint2=0.85, joint3=-0.35, joint4=-0.55`(r≈0.314, 높이만 다름)로 수정하고 24개 자리를 재생성했습니다.

**그 시점에 남아있던 문제**: 재시도-보정 방식은 **빈 공간으로 이동할 때는** 정확히 도착하지만, **실제 박스가 목표 지점에 있는 상태**에서는 접근 경로 중간에 팔/그리퍼가 박스와 부딪히면서 재시도해도 수렴이 잘 안 되는 걸 확인했습니다. (이 문제는 이후 "위에서 아래로 접근 + 안전 높이로 들어올린 뒤 회전" 방식으로 최종 해결되었습니다 — 아래 최신 섹션 참고.)

## 버그 11 / 구조 변경 — 24개 자리 → 8개 자리로 단순화 (box_sort_project.py)

**"near" 템플릿(joint2=0.3, 짧게 뻗음)이 실제로는 많은 박스 위치에 안 닿는 경우가 많다는 걸** COMMAND 모드로 여러 자리를 테스트하며 확인했습니다 (아루코 거리 표시가 0.126~0.14m 근처에서 더 안 줄어들고 멈춤 — 같은 방향의 "mid" 자리는 정상적으로 더 가까이 감). "near"는 이름과 달리 반지름이 짧은 템플릿이라, 오히려 로봇에서 먼 물체는 못 잡는 역설적인 상황이었습니다.

**단순화**: 8방향 × 거리 3단계(24개) 대신, **8방향 × mid 거리 하나(8개)**로 줄였습니다.
- `box_sort_project.py`의 `TOTAL_SLOTS`를 24 → 8로 변경.
- `config/box_slots.yaml`을 8개 자리로 재작성 — 전부 mid 템플릿(joint2=0.85, joint3=0.05, joint4=-0.85) 값을 8방향(0°, 45°, 90°, 135°, 180°, -135°, -90°, -45°)에 적용.
- **near/far 값은 완전히 버렸습니다.** far도 이번 세션에서 검증은 못 했지만, near가 예상과 다르게 동작한 전례가 있어서 신뢰도가 낮다고 판단했습니다. 다음에 3단계 거리가 다시 필요하면, `search_grasp_pose()`류의 FK 탐색으로 각 거리별 값을 새로 검증하고 추가하는 걸 권장합니다.

**다음에 고려해볼 것 (사용자 제안)**: TEACH/COMMAND처럼 "미리 정해둔 자리로 순간이동"하는 방식 대신, **OpenCV로 아루코 마커를 실시간으로 보면서 카메라 피드백 기반으로 접근하는 방식(visual servoing)**을 시도해보면 좋겠다는 아이디어가 나왔습니다. 지금의 "고정된 자리" 방식보다 유연할 수 있지만, 이번 세션에서는 구현 못 했습니다.

## RViz 아루코 시각화 (box_sort_project.py)

카메라 화면(OpenCV 창)만으로는 부족해서, RViz에서도 확인할 수 있게 3가지를 추가로 발행합니다.

| 무엇을 | 토픽/방식 | RViz에서 보는 법 |
|---|---|---|
| 테두리 그려진 카메라 화면 | `/aruco_detection/image` (`sensor_msgs/Image`) | Image 디스플레이 추가, 토픽 선택 |
| 마커의 3D 위치/방향 | TF (`open_manipulator_x/link5/gripper_camera` → `aruco_marker_<id>`) | TF 디스플레이 켜기 |
| "id=.. dist=..m" 글자 | `/aruco_detection/markers` (`visualization_msgs/Marker`, TEXT_VIEW_FACING) | Marker 디스플레이 추가, 토픽 선택 |

**겪은 버그**: `cv2.aruco`가 주는 마커 id는 numpy 정수 타입인데, ROS 메시지의 정수 필드는 파이썬 기본 `int`만 받습니다. 그대로 `marker.id = marker_id`처럼 넣으면 `PyLong_Check` 단언(assert) 실패로 노드가 그 자리에서 죽습니다. `marker_id = int(marker_id)`로 형변환해서 해결했습니다.

회전(rvec)을 TF의 쿼터니언으로 바꾸는 부분은 `cv2.Rodrigues`로 회전행렬을 구한 뒤 표준 행렬→쿼터니언 공식을 직접 구현했습니다 (`rvec_to_quaternion` 함수).

실제로 박스를 놓고 감지시켜서 이미지 토픽, 마커 텍스트(id=0, dist=0.156m), TF(`aruco_marker_0`) 세 가지 다 정상 발행되는 것까지 확인했습니다.
