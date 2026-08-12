# 기존 Manipulator 폴더 교체 안내

이 배포본의 최상위 `Manipulator` 폴더는 팀 저장소의 `Team/Manipulator`에 대응합니다. `Team/Manipulator/Manipulator`처럼 한 단계 중복해서 넣지 않습니다.

## 1. 현재 작업 보존

먼저 팀 저장소에서 현재 브랜치와 수정 상태를 확인합니다.

```bash
cd ~/ROS_Team1
git branch --show-current
git status
```

커밋하지 않은 사용자 수정이 있으면 커밋하거나 별도로 백업한 뒤 교체합니다.

## 2. 폴더 위치

압축을 푼 뒤 다음과 같아야 합니다.

```text
~/ROS_Team1/Team/Manipulator/workspace.sh
~/ROS_Team1/Team/Manipulator/aruco_arm_sorter/package.xml
~/ROS_Team1/Team/Manipulator/aruco_arm_sorter/setup.py
```

기존 저장소의 다른 `Team` 폴더나 다른 팀원 작업물은 교체 대상이 아닙니다.

## 3. 다시 빌드

저장소 루트에서 실행합니다.

```bash
cd ~/ROS_Team1
chmod +x Team/Manipulator/workspace.sh
./Team/Manipulator/workspace.sh build
./Team/Manipulator/workspace.sh doctor
```

사용자가 제공한 기존 `.bashrc`에는 `~/Ros/move_arm/install/setup.bash`가 자동 source되므로 구버전 동명 패키지가 다시 선택될 수 있습니다. 폴더 교체 후 [BASHRC_SETUP.md](./BASHRC_SETUP.md)의 백업·적용 절차도 한 번 수행합니다.

공식 OpenMANIPULATOR 패키지가 별도 작업공간에 있고 자동 탐색되지 않을 때만 해당 setup 파일을 명시합니다.

```bash
ARUCO_DEPENDENCY_SETUP="/의존성/워크스페이스/install/setup.bash" \
  ./Team/Manipulator/workspace.sh build
```

## 4. 실행

```bash
./Team/Manipulator/workspace.sh run
```

구버전 패키지가 다른 작업공간에 남아 있어도 삭제할 필요는 없습니다. 이 명령은 현재 `ROS_Team1`의 설치본을 마지막에 활성화하고 정확한 prefix와 세 실행 파일을 확인한 뒤 실행합니다.
