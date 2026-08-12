# Team 1 Manipulator

ROS 2 Jazzy와 Gazebo Harmonic에서 ROBOTIS OpenMANIPULATOR-X가 `link5` 카메라로 ArUco ID 0·1을 인식하고, 두 박스를 정해진 위치로 옮기는 프로젝트입니다.

저장소 안의 고정 위치는 다음과 같습니다.

```text
ROS_Team1/
└── Team/
    └── Manipulator/
        ├── workspace.sh
        └── aruco_arm_sorter/
```

사용자명이나 이전 작업공간의 절대경로를 직접 입력하지 않습니다. `workspace.sh`가 자신의 위치를 기준으로 저장소 루트, 소스 패키지와 설치 prefix를 계산합니다.

저장소 루트에서 실행합니다.

```bash
./Team/Manipulator/workspace.sh build
./Team/Manipulator/workspace.sh doctor
./Team/Manipulator/workspace.sh run
```

사용자가 제공한 `.bashrc`를 이 저장소 기준으로 정리한 파일과 안전한 적용 방법은 [BASHRC_SETUP.md](./BASHRC_SETUP.md)에 있습니다. 적용 후에는 `cb`, `team1doctor`, `team1run`, `team1status` 단축 명령을 사용할 수 있습니다.

기존 작업공간에 같은 이름의 `aruco_arm_sorter`가 있어도 `doctor`와 `run`은 현재 저장소의 `install/aruco_arm_sorter`가 활성 상태인지 확인합니다. 구버전이 선택되거나 `aruco_detector` 실행 파일이 빠졌으면 Gazebo를 시작하기 전에 중단합니다.

- 교체 방법: [MIGRATION.md](./MIGRATION.md)
- `.bashrc` 적용: [BASHRC_SETUP.md](./BASHRC_SETUP.md)
- 패키지 설명과 문제 해결: [aruco_arm_sorter/README.md](./aruco_arm_sorter/README.md)
- 정적 검증 범위: [aruco_arm_sorter/VALIDATION.md](./aruco_arm_sorter/VALIDATION.md)
