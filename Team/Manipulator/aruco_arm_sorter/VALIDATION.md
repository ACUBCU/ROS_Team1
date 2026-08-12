# Validation

정적 검증 항목:

- Python 및 launch 파일 AST 파싱
- package.xml, world/model SDF, model.config XML 파싱
- YAML 파싱과 관절 제한 검사
- `joint1`~`joint4` 전체 자세 입력 확인
- 공식 controller Action 이름 확인
- pick/place 자세의 OpenMANIPULATOR-X 순기구학 좌표 확인
- ArUco ID별 attach/detach 이벤트 한 쌍 확인
- 자체 제작 `simple_sorting_arm` 제거 확인
- `link5` 카메라·Gazebo Sensors·영상/CameraInfo 브리지 연결 확인
- 관찰 자세의 카메라 원점이 두 마커 중앙 위에 오는지 순기구학으로 확인
- `READY` 상태 제한, 5프레임 연속 검출, ID별 1회 발행 필터 확인
- OpenCV 4.6 및 최신 `ArucoDetector` API 분기 정적 확인
- `preflight`에서 활성 패키지 prefix, 설치된 world의 Sensors 시스템, 카메라 bridge 매핑 확인
- 활성 설치본에 `arm_sequence_controller`, `aruco_detector`, `preflight` 세 실행 파일이 모두 있는지 확인
- `config/project.yaml`로 `ACUBCU/ROS_Team1`의 `Team/Manipulator/aruco_arm_sorter` 설치본인지 확인
- 같은 이름의 패키지가 다른 `AMENT_PREFIX_PATH`에 남아 있으면 경고
- `workspace.sh`가 사용자명과 이전 작업공간의 절대경로 없이 현재 저장소 루트를 계산하는지 확인
- 런치 전에 기대 prefix와 실제 활성 prefix가 다르면 중단하는 보호 로직 확인
- 설치될 wheel 안의 config, world, model, texture 확인

현재 제작 환경에는 ROS 2 Jazzy와 Gazebo Harmonic 실행 파일이 없으므로 GUI/물리 실행은 사용자 환경에서 최종 확인해야 합니다.

ROS 없이 실행 가능한 정적 검증:

```bash
python3 Team/Manipulator/aruco_arm_sorter/tools/static_validate.py
```
