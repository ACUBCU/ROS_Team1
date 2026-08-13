# 프로젝트 기술서: ArUco ID 기반 Pick and Place

## 1. 프로젝트 개요

**프로젝트명** : ArUco ID 기반 Pick and Place
**수행 기간** : 2026.08.10 ~ 2026.08.14

### 팀원 및 담당 역할

* **이동헌**: PM, 코드 통합, 문서 작업
* **주동건**: 아루코 마커 생성 및 인지
* **이서현**: 대시보드 제작
* **이호영**: 매니퓰레이터 이동

---

## 2. 사용 기술 스택

| 구분 | 사용 기술 |
| --- | --- |
| **로봇 미들웨어** | ROS 2 (Jazzy), rclpy |
| **시뮬레이터** | Gazebo (gz sim), ros_gz_bridge |
| **로봇 제어** | Action Client (`FollowJointTrajectory`, `GripperCommand`) |
| **컴퓨터 비전** | OpenCV, ArUco Marker ID 기반 식별 |
| **웹 대시보드** | Flask (Python), MJPEG 스트리밍 (`cv2.imencode`), HTML/CSS/JS |
| **동기화 및 병렬 처리** | Python `threading` (Event 객체 기반 상태 제어 및 무한 루프) |

---

## 3. 세부 수행 목표 및 주요 기능

### 3.1. 수행 목표

* 카메라가 ArUco 마커를 인식하고 올바른 큐브를 지정된 장소로 정확하게 이동(Pick and Place).
* 웹 대시보드를 통해 실시간 모니터링을 수행하고, 대상 타겟과 이동할 장소 설정 및 시뮬레이션 환경 초기화를 제어.

### 3.2. 주요 기능

* **비전 기반 마커 식별**: 로봇 팔 끝단에 부착된 카메라가 목표 ID를 판별.
* **대시보드 원격 통제**: 사용자가 지정한 대상 큐브 ID와 목표 장소 ID를 기반으로 로봇의 작업 경로 결정
* **시뮬레이션 월드 리셋**: 기존에 배치된 큐브 모델을 Gazebo 서비스 통신으로 삭제하고 서브프로세스를 통해 새로운 큐브를 스폰.
* **Pick and Place 시퀀스**: 그리퍼 개폐 확인, 사전 접근(Approach Pose), 잡기(Grasp), 들어올리기(Lift), 목표 배치(Place)로 이어지는 순차적 관절 제어 및 무한 대기 기반 예외 처리.

---

## 4. 시스템 및 하드웨어 구성

### 4.1. 시스템 아키텍처

* **`/dashboard_node`**: Flask 웹 서버와 통합되어 구동되며, 두 대의 카메라 영상을 수신(구독)하고 사용자 웹 조작에 따라 통제 명령을 1회성 발행(Publish).
* **`/aruco_detect_node`**: 그리퍼 카메라 영상을 바탕으로 마커를 감지하여 추출된 대상 및 장소 ID를 발행.
* **`/robot_control_node`**: 액션 클라이언트를 구동하는 핵심 노드. 비전 노드와 대시보드의 명령을 구독하며, 매 작업 후 내부 상태(Event)를 초기화하는 루프를 통해 다중 명령 수행.
* **`/world_reboot_node`**: 대시보드의 리셋 신호를 받아 `gz service`로 기존 모델(`aruco_cube_X`)을 식별하여 삭제하고 스폰 스크립트를 재실행.
* **`/spawn_marker_node`**: 대시보드의 벽면 마커 변경 명령을 수신하여 시각적 타겟 인디케이터를 동적 생성.

### 4.2. 하드웨어 구성 (시뮬레이션 기준)

* **매니퓰레이터**: 관절 제어용 Arm(Joints) + 물체 파지를 위한 평행 그리퍼.
* **카메라 센서**  
**Top Camera (탑 뷰)**: 작업 공간 전체를 내려다보는 시점.  
**Gripper Camera (손목 뷰)**: 그리퍼 끝단에 부착되어 물체 접근 및 마커 인식을 담당.
* **ArUco 큐브**: 환경 내 스폰되는 식별 가능한 정육면체 모델.

---

## 5. 통신 규격

### 5.1. 데이터 송수신 스키마

* **ROS 2 내부 데이터 흐름**:
* 비전 토픽: `/camera_top/image_raw`, `/gripper_camera/image_raw`
* 제어 토픽: `/detected_aruco_ids`, `/start_command`, `/reset_command`, `/set_wall_markers`
* 액션 통신: `/arm_controller/follow_joint_trajectory`, `/gripper_controller/gripper_cmd`
* **웹 ↔ 서버**: HTTP REST API (POST/JSON 송수신) 및 MJPEG Multipart 스트리밍.

---

## 6. Backend API 명세

| Method | Path | 역할 |
| --- | --- | --- |
| GET | `/` | 대시보드 HTML 메인 페이지 렌더링 |
| GET | `/video/top` | 320x240 해상도로 최적화된 탑 카메라 실시간 스트리밍 |
| GET | `/video/gripper` | 320x240 해상도로 최적화된 그리퍼 카메라 실시간 스트리밍 |
| POST | `/api/set_markers` | 지정한 큐브/장소 번호에 따라 벽면 마커 변경 토픽 발행 |
| POST | `/api/start` | 사용자가 선택한 타겟 ID 값을 담아 이동 시작 토픽 발행 |
| POST | `/api/reset` | 시뮬레이션 환경(큐브) 초기화 토픽 발행 |

---

## 7. 화면 구성 (UI/UX)

* **카메라 영역**: 탑 뷰와 그리퍼 뷰 2개의 카메라 화면으로 실시간 프레임 출력.
* **타겟 설정 영역**: 이동할 큐브의 마커 ID와 배치할 목표 장소의 ID를 입력 또는 선택하는 인터페이스.
* **제어 버튼 영역**:  
**마커 변경**: 타겟 설정 영역의 값을 시뮬레이션 벽면에 시각적으로 반영.  
**이동 시작**: 현재 설정된 ID 값을 컨트롤러 노드에 전송하여 로봇 팔 작업을 기동.  
**환경 초기화**: 작업 내역을 무효화하고 큐브들을 원래 상태로 재생성.  

---

## 8. 핵심 코드 로직 및 데이터 플로우

### 8.1. 실시간 동작 환경

스트리밍 영상 해상도를 `cv2.resize()`로 낮추고 움직임을 보여주는 최소한의 카메라만 사용하여 시뮬레이터 구동

### 8.2. 타임아웃 무한 대기 (Event 제어)

시뮬레이션 환경 상 발생할 수 있는 일시적 딜레이로 인해 스레드가 비정상 종료되는 것을 막기 위해, Wall-clock 기반의 타임아웃 에러 처리를 배제하고 로봇의 액션 목표 도달(Goal Reached) 응답이 올 때까지 대기하는 안정적 시퀀스 적용.

### 8.3. 상태 머신 무한 루프 및 데이터 덮어쓰기

작업이 끝난 후 ROS 2 큐에 남은 과거 비전 데이터가 재실행을 유도하는 것을 방지하기 위해 매 루프마다 플래그를 초기화. 대시보드에서 `start_command` 수신 시 기존에 인지된 데이터를 무시하고 사용자 지정 타겟 ID를 덮어씌워 강제로 작업을 확정 및 실행함.

---

## 9. 참조

* **GitHub Repository**: [https://github.com/ACUBCU/ROS_Team1](https://github.com/ACUBCU/ROS_Team1)
* **발표 자료**: [발표 자료](https://docs.google.com/presentation/d/1QUFMRrnHLzvYb9o2nvBDHGcFmhemkZrwfRGsW1nHXsU/edit?usp=sharing)