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
- 설치될 wheel 안의 config, world, model, texture 확인

현재 제작 환경에는 ROS 2 Jazzy와 Gazebo Harmonic 실행 파일이 없으므로 GUI/물리 실행은 사용자 환경에서 최종 확인해야 합니다.
