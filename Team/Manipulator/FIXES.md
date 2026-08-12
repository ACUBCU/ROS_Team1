# 수정 내역

기준일: 2026-08-12

## 기준 자료

- 첨부된 `Manipulator.zip`을 최신 코드 기준으로 사용
- `ACUBCU/ROS_Team1`의 동명 패키지는 코드 병합 대상이 아닌 저장소 구조 확인용으로만 사용
- 공식 ROBOTIS Jazzy 소스에서 Xacro 위치, controller 이름과 Gazebo launch 구조 확인
- Gazebo Harmonic Sensors 소스와 `ros_gz_bridge` 문서에서 영상·CameraInfo 토픽 및 YAML 브리지 형식 확인

## 경로·overlay 수정

- 최종 위치를 `Team/Manipulator/aruco_arm_sorter`로 고정
- 사용자 홈과 이전 작업공간의 절대경로 의존 제거
- `workspace.sh`가 스크립트 위치에서 저장소 루트와 설치 prefix를 계산하도록 추가
- 다른 작업공간의 동명 패키지가 선택되면 `doctor`와 `run`이 즉시 실패하도록 수정
- `AMENT_PREFIX_PATH` 안의 중복 패키지 위치 표시
- 기존 패키지의 생성물만 정리하고 현재 저장소의 소스만 지정해 빌드
- 사용자 제공 `.bashrc`에서 `~/Ros/move_arm` 구버전 overlay 자동 source 제거
- 현재 터미널에 상속된 `move_arm`의 ROS·Python·library·실행·Gazebo 경로를 source 전에 정리
- `~/ROS_Team1/install/setup.bash`를 마지막 overlay로 적용
- OpenMANIPULATOR 의존성 workspace는 `workspace.sh`에서만 불러오도록 분리
- 의존성 자동 탐색 중 구버전 `aruco_arm_sorter`가 함께 설치된 workspace는 건너뜀
- `ARUCO_DEPENDENCY_SETUP`에 동명 구버전 package가 포함되면 명시적 오류로 중단
- 소스 폴더 기반 `GZ_SIM_RESOURCE_PATH` 전역 설정 제거(런치가 설치 경로로 설정)
- 프로젝트용 전체 `.bashrc`, 백업·적용·복구 안내와 단축 명령 추가

## 패키지·런치 수정

- 패키지 버전을 `3.1.0`으로 통일
- `config/project.yaml`을 설치해 저장소와 패키지 경로 식별
- `preflight`에서 세 console executable 설치 여부 검사
- 런치에서 기대 prefix와 활성 prefix가 다르면 Gazebo 시작 전 중단
- `position_controllers` 직접 실행 의존성 명시 및 사전 점검 추가
- 시작 로그에 실제 활성 `aruco_arm_sorter` prefix 표시

## 검증

- Python/launch AST 및 XML/SDF/YAML 파싱
- 동작 YAML 관절 제한·마커 매핑·관찰 자세 복귀 확인
- 카메라 위치와 `link5` 장착, 두 DetachableJoint 확인
- ArUco 안정 검출 gate 확인
- Bash 구문 검사
- 로그인 셸과 빈 홈 환경을 모사한 `.bashrc` 로드 검사
- wheel 빌드에서 launch, config, world, 모델, 텍스처와 프로젝트 식별 파일 포함 확인

제작 환경에는 ROS 2 Jazzy와 Gazebo Harmonic 실행 파일이 없어 실제 GUI·물리 실행은 포함하지 않았습니다. 사용자 환경에서는 `./Team/Manipulator/workspace.sh doctor`가 이를 검사하고, 통과 후 `run`이 실행합니다.
