# 로봇 실제 작동 한계 (OpenManipulator-X)

## 비유로 먼저 이해하기

이 문서는 이 프로젝트가 쓰는 로봇 팔(OpenManipulator-X)이 **실제로 어디까지 움직일 수 있는지**를 정리한 것입니다.

사람 팔에 비유하면 이해가 쉽습니다:
- 팔꿈치가 뒤로 꺾이지 않고 정해진 각도까지만 굽혀지듯, 이 로봇의 관절 4개도 각각 정해진 각도 범위 안에서만 회전합니다.
- 사람이 무거운 물건을 들면 팔이 떨리거나 못 들듯, 이 로봇도 들 수 있는 무게(payload)에 한계가 있습니다.
- 사람 손이 최대로 뻗을 수 있는 거리가 있듯, 로봇 팔도 바닥 중심에서 뻗을 수 있는 최대 거리(reach)가 정해져 있습니다.

로컬 저장소(`이서현_project`)에는 이 하드웨어 정의 파일들이 없어서(서브모듈로 별도 관리되는 ROBOTIS 공식 패키지라 이 repo에는 안 들어있음), 사용자가 알려준 GitHub 저장소(`open_manipulator_ws/src`)의 `.gitmodules`를 따라가 ROBOTIS 공식 저장소(`ROBOTIS-GIT/open_manipulator`)에서 직접 확인했습니다.

## 관절별 각도 한계 (URDF 기준, `open_manipulator_x_arm.urdf.xacro`)

| 관절 | 역할 | 회전축 | 최소 각도 | 최대 각도 | 최대 각속도 |
|---|---|---|---|---|---|
| joint1 | 베이스 회전(허리) | Z축 | -180° (-π rad) | +180° (+π rad) | 4.8 rad/s |
| joint2 | 어깨 | Y축 | -85.9° (-1.5 rad) | +85.9° (+1.5 rad) | 4.8 rad/s |
| joint3 | 팔꿈치 | Y축 | -85.9° (-1.5 rad) | +80.2° (+1.4 rad) | 4.8 rad/s |
| joint4 | 손목 | Y축 | -97.4° (-1.7 rad) | +112.9° (+1.97 rad) | 4.8 rad/s |
| gripper_left_joint | 그리퍼(직선 이동) | Y축 | -0.011 m | +0.02 m | 4.8 rad/s |

- URDF에는 `effort="1000"`으로 적혀 있는데, 이건 실제 모터 토크 스펙이 아니라 시뮬레이터(Gazebo/MoveIt)가 "일단 힘 제한 없다고 치자"고 두는 플레이스홀더 값입니다. 실제 힘 한계는 아래 모터 스펙을 보세요.

## MoveIt 속도/가속도 제한 (`joint_limits.yaml`)

기본적으로 MoveIt은 모든 관절에 대해 속도/가속도를 **정격의 10%로 제한**(`default_velocity_scaling_factor: 0.1`, `default_acceleration_scaling_factor: 0.1`)해서 움직입니다. 즉 코드에서 별도로 스케일을 올리지 않는 한, 실제로는 모터 최대 속도의 10%로만 천천히 움직인다는 뜻입니다. 필요하면 모션 요청 시 최대 1.0(100%)까지 올릴 수 있습니다.

## 실제 하드웨어(모터/기구부) 스펙 — ROBOTIS 공식 e-Manual

| 항목 | 값 |
|---|---|
| 자유도 (DOF) | 4 (관절) + 1 (그리퍼) |
| 사용 모터 | DYNAMIXEL XM430-W350-T (관절 4개 전부 동일) |
| 최대 리치(reach) | 380 mm |
| 최대 페이로드(들 수 있는 무게) | 500 g |
| 위치 반복 정밀도 | < 0.2 mm |
| 관절 최대 속도 | 46 RPM (무부하 기준) |
| 로봇 자체 무게 | 0.70 kg |
| 입력 전압 | 12 V |
| 그리퍼 스트로크(벌어지는 폭) | 20 ~ 75 mm |
| 모터 스톨 토크(멈춤 상태 최대 힘) | 4.1 N·m (12 V, 2.3 A 기준) |

## 정리하면

- **관절 각도**: joint1(허리)만 ±180° 통째로 돌고, 나머지 3개(어깨/팔꿈치/손목)는 대략 ±80~113° 정도로 제한됩니다. 사람 팔처럼 완전히 한 바퀴 도는 관절은 허리뿐입니다.
- **속도**: 하드웨어 최대는 46 RPM이지만, MoveIt 설정상 기본은 그 10%로 느리게 움직입니다.
- **힘**: 500 g가 넘는 물체(예: 무거운 박스, 여러 개 겹친 아루코 큐브)는 안정적으로 들지 못할 가능성이 높습니다.
- **작업 반경**: 베이스에서 380 mm 이상 떨어진 물체는 애초에 팔이 닿지 않습니다. 박스/슬롯 배치(`config/box_slots.yaml`)를 조정할 때 이 반경 안에 있는지 확인하는 게 좋습니다.

## 출처

- 프로젝트가 참조한 로봇 모델: `open_manipulator_x` (이 저장소 `launch/box_sort_moveit.launch.py`에서 확인)
- 관절 각도 한계: [ROBOTIS-GIT/open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) `open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x_arm.urdf.xacro`
- 속도/가속도 스케일: 같은 저장소 `open_manipulator_moveit_config/config/open_manipulator_x/joint_limits.yaml`
- 하드웨어 스펙(리치/페이로드/모터): [ROBOTIS e-Manual — OpenMANIPULATOR-X Specification](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/)
- 모터 토크: [ROBOTIS e-Manual — XM430-W350-T/R](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)
