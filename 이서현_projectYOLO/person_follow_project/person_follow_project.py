# redball_tracking_project.py("빨간 공을 눈으로 쫓아가는 로봇 팔")를 그대로 뼈대로 삼아,
# "사람을 눈으로 보고 다가가는 로봇 팔"로 바꾼 프로젝트 파일입니다.
#
# [비유로 먼저 이해하기]
#   redball_tracking_project.py가 "빨간색"이라는 아주 단순한 특징으로 공을 찾았다면,
#   이 파일은 YOLO26이라는 이미 학습된 "눈"을 빌려와서 "이건 사람이다"를 훨씬 똑똑하게
#   알아봅니다. 나머지(화면 중심으로 다가가기, 관절 움직이기)는 완전히 같은 방식입니다.
#
# [Gazebo에서 "사람"을 어떻게 보여주나]
#   시뮬레이션 안에서는 진짜 사람이 걸어다닐 수 없으므로, 저작권 없는 사람 실루엣 그림을
#   텍스처로 붙인 고정 평면(models/person_photo)을 world에 세워두고, 그 "사진"을
#   YOLO26이 사람으로 인식하게 합니다. (사람 사진/그림을 사람으로 착각하는 건 실제 얼굴
#   인식 스푸핑에서도 흔히 쓰이는 성질이라, 이 데모 목적에는 오히려 딱 맞습니다.)
#
# [모드 3가지]
#   - MANUAL  (수동)   : 키보드로 사람이 직접 관절/그리퍼를 움직임 (기본 시작 모드)
#   - AUTO    (자동)   : 카메라가 YOLO26으로 사람을 찾아서 알아서 다가감
#   - GESTURE (손동작) : 손동작(MediaPipe)으로 조작. 엄지가 가리키는 방향으로 팔이 움직이고,
#                        손바닥을 펴거나 주먹을 쥐면 정지함. gesture_callback() 참고.
#   'm' 키를 누를 때마다 MANUAL -> AUTO -> GESTURE -> MANUAL 순서로 돌아감.
#
# [카메라 이미지 한 장이 처리되는 순서] (image_callback, 모드 상관없이 탐지/로깅은 항상 함)
#   1) 이미지를 받는다
#   2) N프레임에 한 번만 YOLO26 추론을 돌려서(추론이 색상 마스크보다 훨씬 무거움, CPU 환경
#      고려) "person" 클래스 중 가장 confidence가 높은 바운딩박스를 찾는다
#   3) 화면 중심과 박스 중심의 차이(오차)를 계산한다
#   4) 박스 높이(height)를 "얼마나 가까운지"의 대략적인 지표로 쓴다 (실제 미터 단위 아님)
#   5) AUTO 모드일 때만: 오차만큼 joint1(좌우), joint2~4(상하)를 아주 조금씩 움직이고,
#      충분히 가까워지면 그리퍼를 한 번 닫았다 열어서 "인사"를 표현한다
#
# [사람 사진은 평면이라 "잡을" 대상이 아님]
#   빨간 공 프로젝트는 다 다가가면 그리퍼를 닫아서 "잡기"였지만, 사람 사진은 평면 소품이라
#   실제로 쥘 것이 없습니다. 대신 충분히 가까워지면 그리퍼를 CLOSE -> OPEN 순서로 한 번만
#   보내서 "인사" 동작으로 표현하고, 사람이 화면에서 멀어지거나 벗어나면 다시 인사할 수
#   있도록 상태를 초기화합니다.
#
# [사람이 화면에서 안 보이면 -> 천천히 회전하며 탐색]
#   AUTO 모드에서 사람을 NO_DETECTION_SEARCH_AFTER 프레임 넘게 연속으로 못 찾으면,
#   joint1을 SEARCH_STEP만큼씩 천천히 좌우로 왕복시키며 "둘러봅니다"(사람이 다시 보이면
#   즉시 탐색을 멈추고 원래 추적 로직으로 돌아감). _search_if_needed() 참고.
#
# [웹뷰]
#   같은 패키지의 web_control_person.py를 실행하면, 카메라 화면 + 상태(모드/인사/탐색 여부)를
#   웹 브라우저(http://localhost:8080)에서 보고 버튼으로 조작할 수 있습니다. 이 파일의
#   handle_key()를 그대로 재사용하는 방식이라 로직은 완전히 같습니다.
#
# [주의 / 한계] - redball_tracking_project.py와 동일한 한계를 그대로 가짐
#   - 바운딩박스 높이 기반 거리 추정은 카메라 보정을 안 했으므로 "대략 이 정도" 지표일 뿐임.
#   - joint2~4에 같은 값을 그대로 더하는 것은 실제 역기구학이 아니라 초심자용 단순화.
#   - 나중에 실물 로봇에 연동할 계획이므로, 이 노드도 ROS2 표준 액션/토픽(FollowJointTrajectory,
#     GripperCommand, /joint_states, /gripper_camera/image_raw)만 쓰고 Gazebo 전용 API는
#     직접 호출하지 않음.
#
# 실행 방법:
#   1) colcon build --symlink-install --packages-select person_follow_project
#   2) source install/setup.bash
#   3) ros2 run person_follow_project person_follow_project
#   (처음 실행할 때 YOLO26 가중치 파일을 인터넷에서 자동으로 내려받으므로 인터넷 연결 필요)
#
# 키 배치 (MANUAL 모드일 때) - redball_tracking_project.py와 동일:
#   q/a : joint1 +/-      w/s : joint2 +/-
#   e/d : joint3 +/-      r/f : joint4 +/-
#   z   : 그리퍼 열기     x   : 그리퍼 닫기
#   m   : MANUAL -> AUTO -> GESTURE -> MANUAL 모드 순환 전환
#   Ctrl+C : 종료

import math
import os
import select
import sys
import termios
import tty

import cv2
import mediapipe as mp
import rclpy
from ament_index_python.packages import get_package_share_directory
from mediapipe.tasks.python import vision as mp_vision
from mediapipe.tasks.python import BaseOptions as MpBaseOptions
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import (
    FollowJointTrajectory,
    FollowJointTrajectory_GetResult_Response,
    GripperCommand,
    GripperCommand_GetResult_Response,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from ultralytics import YOLO

# ---- 튜닝 상수 (redball_tracking_project.py와 같은 이름/의미의 값은 그대로 재사용) ----
X_GAIN = 0.0008          # (AUTO) 화면 x 오차(픽셀) 1당 joint1을 얼마나 돌릴지 (라디안/픽셀)
# 부호가 -로 바뀜: 시뮬레이션의 거의 편 기본자세와 달리 실물 로봇 기본자세는 팔꿈치가 많이
# 굽어있어서(joint3=-1.0, joint4=1.0 근처), 원래 부호(+)로 joint2~4에 델타를 더하면 얼굴이
# 화면 위쪽에 있을 때 오히려 카메라가 바닥 쪽으로 내려감(실제로 겪음). 부호를 반대로 바꿔서
# 얼굴이 위에 있으면 카메라도 위로 가게 함(실제 확인 필요 - 처음엔 아주 작은 스텝으로 확인할 것).
Y_GAIN = -0.0008         # (AUTO) 화면 y 오차(픽셀) 1당 joint2~4를 얼마나 움직일지 (라디안/픽셀)
MAX_STEP = 0.015         # (AUTO) 한 프레임에 관절 하나가 움직일 수 있는 최대량 (라디안).
                          # 0.008(안전 확인용) -> 0.025(속도 올려달라는 요청)까지 올렸다가,
                          # 너무 빠르다는 피드백을 받아서 그 중간 정도로 다시 낮춤.

# ---- 안전 여유 관절 한계 ----
# ROBOT_LIMITS.md에 정리된 ROBOTIS 공식 하드웨어 한계(URDF 기준)에서, 양쪽 끝을 5도씩
# 깎아서 절대 하드웨어 한계까지 안 가게 함(부동소수점 오차나 오버슈트로 실제 한계를 넘겨서
# 목표가 거부되거나 무리한 힘이 걸리는 걸 방지). 모든 팔 이동(수동/자동/탐색)이 이 범위
# 안에서만 움직이도록 send_joint_positions()에서 항상 clamp함.
SAFETY_MARGIN_RAD = math.radians(5)
JOINT_LIMITS_SAFE = {
    # 이름: (하드웨어 한계 최소, 최대) - ROBOT_LIMITS.md 기준 (rad)
    "joint1": (-math.pi + SAFETY_MARGIN_RAD, math.pi - SAFETY_MARGIN_RAD),      # 원래 -180~180도
    "joint2": (-1.5 + SAFETY_MARGIN_RAD, 1.5 - SAFETY_MARGIN_RAD),              # 원래 -85.9~85.9도
    "joint3": (-1.5 + SAFETY_MARGIN_RAD, 1.4 - SAFETY_MARGIN_RAD),              # 원래 -85.9~80.2도
    "joint4": (-1.7 + SAFETY_MARGIN_RAD, 1.97 - SAFETY_MARGIN_RAD),             # 원래 -97.4~112.9도
}

CAMERA_DEVICE_INDEX = 0          # 실물 그리퍼에 붙인 USB 웹캠의 장치 번호. 보통 /dev/video0이
                                  # 첫 번째 카메라(캡처용), /dev/video1은 같은 카메라의 메타데이터
                                  # 전용 노드인 경우가 많아서 0번을 씀. 카메라가 여러 개면 바꿔야 함.

YOLO_MODEL_NAME = "yolo26n-pose.pt"  # YOLO26 "포즈" 모델. 일반 yolo26n.pt는 "사람"이라는 사각
                                  # 박스만 주지만, 포즈 모델은 사람 몸의 관절 17개 지점(코, 눈,
                                  # 어깨 등) 좌표까지 같이 줌. 그 중 "코(nose)" 지점을 그대로
                                  # 얼굴 중심으로 쓰면 박스 비율로 어림짐작하는 것보다 훨씬 정확함.
PERSON_CLASS_ID = 0              # COCO 데이터셋 기준 "person"의 클래스 번호 (포즈 모델도 동일)
NOSE_KEYPOINT_INDEX = 0          # COCO 17개 키포인트 순서: 0=코, 1~2=눈, 3~4=귀, 5~6=어깨 ...
KEYPOINT_CONF_THRESHOLD = 0.3    # 코 키포인트의 확신도가 이보다 낮으면(예: 사람이 뒤돌아서 얼굴이
                                  # 안 보이는 경우) 코 위치를 못 믿고 기존 방식(박스 비율)으로 대체함
CONF_THRESHOLD = 0.5             # 이 확신도(confidence)보다 낮으면 "사람 아님"으로 무시.
                                  # Gazebo 정지 사진 재탐색 때는 0.2로 낮춰야 했지만(스쳐 지나가는
                                  # 순간엔 확신도가 낮게 나옴), 실물 카메라에서는 오히려 너무 낮아서
                                  # 의자 등 가구를 사람으로 오탐하는 문제가 있었음(실제로 겪음).
                                  # 실시간 카메라는 매 프레임 다시 시도하니 이렇게 높여도 괜찮음.
INFER_EVERY_N_FRAMES = 3         # 몇 프레임에 1번 YOLO 추론(및 로봇 명령)을 할지.
                                  # 처음엔 1(매 프레임)로 뒀는데, GESTURE 모드에서 손동작을 계속
                                  # 유지하면 카메라 프레임(10Hz)마다 로봇에 새 명령을 계속 보내게
                                  # 되고, 이게 실물 로봇의 USB 시리얼 통신(안 그래도 USB/IP라
                                  # 대역폭이 부족함, 위 카메라 문제 참고)에 부담을 줘서 결국
                                  # 통신 에러가 쌓이다가 컨트롤러 전체가 죽어버리는 문제가 있었음
                                  # (실제로 겪음 - arm_controller/gripper_controller 모두
                                  # inactive로 꺼짐). 명령 빈도를 3분의 1로 줄여서 완화함.

HEAD_Y_FRACTION = 0.15           # 세로 방향 추적 목표: 박스 맨 위(y1)에서 이 비율만큼 아래 지점.
                                  # 0.5면 몸통 중심(배꼽 근처), 0.15면 대략 얼굴 높이. 다가갈수록
                                  # 자연스럽게 카메라가 얼굴 쪽으로 들리게 하려고 낮은 값을 씀.

GREET_BOX_HEIGHT = 70            # (AUTO) 바운딩박스 높이가 이 값(px)보다 크면 "충분히 가까움" -> 인사.
                                  # 카메라 해상도를 다시 160x120으로 되돌렸으므로 그 기준으로 복원.
GREET_RESET_MARGIN = 12          # 다시 인사하려면 박스 높이가 (GREET_BOX_HEIGHT - 이 값) 아래로
                                  # 한 번은 내려가야 함 (경계선에서 값이 떨렸다 넘었다 하며
                                  # 그리퍼를 계속 여닫는 것을 막기 위한 여유, 일종의 히스테리시스)
                                  # (GREET_BOX_HEIGHT를 줄인 비율에 맞춰 40 -> 12로 같이 줄임)

# ---- 탐색(사람을 못 찾았을 때 천천히 둘러보기) 관련 상수 ----
NO_DETECTION_SEARCH_AFTER = 15   # (추론 프레임 기준, INFER_EVERY_N_FRAMES마다 1씩 증가) 이만큼
                                  # 연속으로 사람을 못 찾으면 "안 보임"으로 판단하고 탐색 시작
SEARCH_STEP = 0.03               # 탐색 중 한 번에 joint1을 얼마나 돌릴지 (MAX_STEP=0.02보다 살짝
                                  # 커서 실제로 "돌고 있다"가 눈에 보이지만, 그래도 느리게)
SEARCH_MIN = -3.0                # joint1의 물리적 회전 한계는 -pi~+pi인데, 정확히 경계값을 쓰면
SEARCH_MAX = 3.0                 # 부동소수점 오차로 목표가 거부될 위험이 있어(box_sort_project.py의
                                  # 3.13 라디안과 같은 이유) 살짝 여유를 둔 범위 안에서만 왕복함.
                                  # (마지막으로 본 위치를 모를 때만, 즉 시작 직후 등에만 이 전체
                                  # 범위를 씀 - 알고 있으면 아래 LOCAL_SEARCH_RANGE를 씀.)
LOCAL_SEARCH_RANGE = 0.2         # 마지막으로 사람을 본 joint1 각도가 있으면, 그 각도 ± 이 값
                                  # 범위 안에서만 왕복 탐색함. 전체 범위(SEARCH_MIN~MAX)를 다
                                  # 훑게 했더니, 마지막 위치 쪽으로 방향은 맞게 잡고도 그 지점을
                                  # 그냥 지나쳐서 반대쪽 끝까지 가버리는 문제가 있었음(실제로
                                  # 겪음). 사람이 방금 전까지 그 근처에 있었던 게 확실하므로
                                  # 좁게 왔다갔다 하는 게 훨씬 안전함.
SEARCH_HOLD_FRAMES = 5           # 마지막으로 본 자세로 돌아간 직후, 옆으로 더 움직이기 전에
                                  # 그 자리에서 몇 번(추론 프레임 기준) 더 확인해볼지. 도착
                                  # 직후 딱 한 프레임만 보고 바로 옆으로 옮겨버리면, 자세가 아직
                                  # 안정되는 중이라 인식이 안 될 수도 있어서 여유를 줌.

STEP = 0.05              # (MANUAL) 키 한 번 누를 때 관절이 움직이는 양
GRIPPER_OPEN = 0.019
GRIPPER_CLOSE = -0.01
GRIPPER_RAMP_STEP = 0.003        # GESTURE 모드에서 손가락 자세를 유지하는 동안 한 프레임(약
                                  # 10Hz)마다 그리퍼를 얼마나 움직일지. 전체 범위(0.029)를 다
                                  # 움직이는 데 약 1초 정도 걸리는 속도 - "자세를 유지하면 천천히
                                  # 계속 열리고/닫히게 해달라"는 요청에 맞춘 값. (0.001은 한 번에
                                  # 움직이는 양이 너무 작아서 눈에 잘 안 띄어서 조금 올림)
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
# 실물 로봇을 켜면(bringup) 자동으로 이동하는 기본 자세. "디폴트로 옮기기" 버튼을 누르면
# 이 자세로 되돌아감(어느 모드에서 시작하든 안전하게 되돌아올 수 있는 기준 자세로 씀).
DEFAULT_POSITION = [0.0377562243001, 0.0, -1.000155473701645, 1.0246991663074017]

# ---- GESTURE(손동작 조작) 모드 관련 상수 ----
# MediaPipe의 "제스처 인식기"를 씀 - 이미 학습되어 있는 손동작 모델이라 우리가 따로 학습시킬
# 필요 없음. 손 마디(landmark) 21개 좌표 + "이 손동작이 무엇인지"(Open_Palm 등)를 한 번에 줌.
GESTURE_MODEL_FILENAME = "gesture_recognizer.task"  # 패키지 안(models_data/)에 미리 받아둔 모델 파일
STOP_GESTURE_NAMES = {"Open_Palm", "Closed_Fist"}  # 이 손동작들이 보이면 "정지" (움직이던 걸 멈춤)
GRIPPER_OPEN_FINGER_COUNT = 2    # 편 손가락이 이 개수면 그리퍼 열기
GRIPPER_CLOSE_FINGER_COUNT = 3   # 편 손가락이 이 개수면 그리퍼 닫기
GRIPPER_DEFAULT_FINGER_COUNT = 4  # 편 손가락이 이 개수면 디폴트 자세로 이동
# MediaPipe 캔 제스처 목록엔 "손가락 N개" 같은 항목이 없어서(Open_Palm/Closed_Fist/Victory 등
# 정해진 것만 있음), 손가락을 직접 세어야 함. 각 손가락의 "끝(tip)"이 "중간 마디(pip)"보다
# 손목에서 더 멀리 떨어져 있으면 "펴짐"으로 판단(엄지는 움직임이 달라서 셈에서 제외 - 검지~
# 새끼 4개만 셈). 손이 어느 방향으로 돌아가 있어도 비교적 안정적으로 동작하는 방법.
FINGER_TIP_PIP_LANDMARKS = [(8, 6), (12, 10), (16, 14), (20, 18)]  # (검지,중지,약지,새끼) 각각 (끝, 중간마디)
GESTURE_MIN_HAND_CONF = 0.5      # 손을 찾았다고 믿을 최소 확신도
GESTURE_STEP = 0.025             # 손동작 모드에서 한 프레임에 관절을 얼마나 움직일지 (라디안)
                                  # (속도를 높였다가 너무 빠르다는 피드백을 받아서 다시 낮춤)
# MediaPipe 손 관절(landmark) 번호: 0=손목, 1~4=엄지(1:CMC,2:MCP,3:IP,4:끝), 5~8=검지 ...
WRIST_LANDMARK = 0
THUMB_TIP_LANDMARK = 4

MANUAL_KEY_BINDINGS = {
    "q": (0, +STEP),
    "a": (0, -STEP),
    "w": (1, +STEP),
    "s": (1, -STEP),
    "e": (2, +STEP),
    "d": (2, -STEP),
    "r": (3, +STEP),
    "f": (3, -STEP),
}


def get_key(timeout=0.0):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if rlist else ""


class PersonFollowProject(Node):
    def __init__(self):
        super().__init__("person_follow_project")

        self.joint_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")

        self.create_subscription(JointState, "joint_states", self.joint_callback, 10)
        self.current_joint_position = [0.0, 0.0, 0.0, 0.0]
        self.current_gripper_position = 0.0
        self.joint_state_received = False

        # Gazebo 시뮬레이션에서는 카메라가 "/gripper_camera/image_raw"라는 ROS 토픽으로
        # 발행됐지만, 실물 로봇은 그리퍼 위에 USB 웹캠을 그냥 직접 붙인 것이므로 ROS 토픽을
        # 거치지 않고 OpenCV로 카메라 장치를 직접 열어서 읽음. 카메라 프레임 속도(30Hz)에 맞춰
        # 타이머로 계속 읽어옴 - 기존 image_callback과 동일한 처리 로직을 그대로 재사용함.
        self.camera = cv2.VideoCapture(CAMERA_DEVICE_INDEX)
        # 이 컴퓨터는 카메라가 USB/IP(usbipd-win)로 WSL에 가상 연결되어 있어서, 대역폭이
        # 실제 USB보다 훨씬 부족함. 320x240, 240x180 둘 다 짧은 단독 테스트에서는 안 깨졌지만,
        # YOLO 추론 + 웹 스트리밍까지 같이 도는 실제 상황에서는 계속 "Corrupt JPEG data"가
        # 발생함(실제로 겪음 - 240x180에서도 약 7% 프레임이 깨짐, 화면이 끊기는 것으로 느껴짐).
        # 짧은 캡처 테스트만으로는 실제 부하 상황을 재현 못 해서 검증이 충분치 않았음. 화질보다
        # 끊김 없이 안정적인 게 더 중요하다고 판단해서, 완전히 안정적이었던 160x120으로 되돌림.
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
        self.camera.set(cv2.CAP_PROP_FPS, 10)
        if not self.camera.isOpened():
            self.get_logger().error(
                f"카메라({CAMERA_DEVICE_INDEX})를 열지 못했습니다. "
                f"'ls /dev/video*'로 장치가 있는지, 권한이 있는지 확인하세요."
            )
        self.create_timer(1.0 / 10.0, self.camera_timer_callback)  # 카메라를 10fps로 맞췄으므로

        self.get_logger().info(f"YOLO26 모델 로드 중... ({YOLO_MODEL_NAME}, 처음이면 다운로드함)")
        self.yolo = YOLO(YOLO_MODEL_NAME)
        self.get_logger().info("YOLO26 모델 로드 완료")

        gesture_model_path = os.path.join(
            get_package_share_directory("person_follow_project"), "models_data", GESTURE_MODEL_FILENAME
        )
        gesture_options = mp_vision.GestureRecognizerOptions(
            base_options=MpBaseOptions(model_asset_path=gesture_model_path),
            num_hands=1,
            min_hand_detection_confidence=GESTURE_MIN_HAND_CONF,
        )
        self.gesture_recognizer = mp_vision.GestureRecognizer.create_from_options(gesture_options)
        self.get_logger().info("손동작 인식 모델(MediaPipe) 로드 완료")

        self.arm_goal_in_progress = False
        self.gripper_goal_in_progress = False
        self.frame_count = 0

        # "인사" 상태 머신: IDLE(대기) -> CLOSING(닫는 중) -> OPENING(여는 중) -> DONE(인사 끝, 재접근 대기)
        self.greet_state = "IDLE"

        # 탐색(사람을 못 찾았을 때 천천히 둘러보기) 관련 상태
        self.no_detection_count = 0
        self.searching = False
        self.search_direction = 1  # +1: joint1 증가 방향, -1: 감소 방향. 경계에서 뒤집힘.
        # 마지막으로 사람을 실제로 봤을 때의 관절 4개 자세(joint1~4) 전부. 탐색을 새로 시작할
        # 때 이 자세로 먼저 돌아가봄. 처음엔 joint1(좌우)만 기억했었는데, 그러면 세로 방향
        # (joint2~4, 얼굴을 쫓아가다 카메라가 너무 위를 보게 된 경우 등)이 틀어진 채로 좌우만
        # 훑어서 다시 못 찾는 문제가 있었음(실제로 겪음) - 그래서 4개 전부 기억하도록 고침.
        self.last_seen_position: list[float] | None = None
        # 마지막으로 본 자세로 되돌아간 뒤, 곧바로 좌우로 더 움직이지 않고 "그 자리에서 몇 번
        # 더 확인해보는" 횟수를 셈. 원래는 도착하자마자 한 번 못 찾으면 바로 옆으로 더 움직였는데,
        # 도착 직후 한 프레임만에 딱 맞게 인식이 안 될 수도 있어서(자세가 아직 안정되는 중이거나
        # 타이밍이 안 맞아서) 몇 번 더 그 자리에서 재시도하도록 여유를 줌(실제로 겪은 문제).
        self.search_hold_count = 0

        # 웹뷰(web_control_person.py)가 이 값을 읽어서 MJPEG로 스트리밍함. 추론을 실제로 돌린
        # 프레임에서만 갱신되고(INFER_EVERY_N_FRAMES마다 1번), 그 사이 프레임은 이전 값을
        # 그대로 유지함 - 정지된 사진(person_photo) 대상이라 10fps 정도로도 충분함.
        self.latest_gripper_jpeg: bytes | None = None
        # 박스/글자 표시 없는 깨끗한 원본 화면 (매 카메라 프레임마다 갱신됨)
        self.latest_gripper_jpeg_raw: bytes | None = None

        self.mode = "MANUAL"

    # ------------------------------------------------------------------
    def camera_timer_callback(self):
        ok, img = self.camera.read()
        if not ok:
            return  # 카메라에서 이번 프레임을 못 읽음(드물게 발생) - 다음 타이머 호출을 기다림
        self.image_callback(img)

    def image_callback(self, img):
        image_h, image_w = img.shape[:2]

        # 아무 표시(박스/글자) 없는 깨끗한 원본 화면. 웹뷰에 큼직하게 따로 보여주기 위함 -
        # YOLO 추론과 상관없이 카메라가 읽어온 모든 프레임마다 갱신해서 매끄럽게 나오게 함.
        ok, raw_encoded = cv2.imencode(".jpg", img)
        if ok:
            self.latest_gripper_jpeg_raw = raw_encoded.tobytes()

        self.frame_count += 1
        if self.frame_count % INFER_EVERY_N_FRAMES != 0:
            cv2.imshow("img", img)
            cv2.waitKey(10)
            return  # 이번 프레임은 추론을 건너뜀 (CPU 부담을 줄이기 위함)

        if self.mode == "GESTURE":
            self.gesture_callback(img, image_w, image_h)
            return

        results = self.yolo(img, classes=[PERSON_CLASS_ID], conf=CONF_THRESHOLD, verbose=False)
        boxes = results[0].boxes
        keypoints = results[0].keypoints  # 포즈 모델이라 사람마다 17개 관절 좌표도 같이 나옴

        debug_img = results[0].plot()  # YOLO가 박스+확신도를 그려주는 디버그용 이미지
        cv2.imshow("img", debug_img)
        cv2.waitKey(10)
        ok, encoded = cv2.imencode(".jpg", debug_img)
        if ok:
            self.latest_gripper_jpeg = encoded.tobytes()

        if boxes is None or len(boxes) == 0:
            self.greet_state = "IDLE"  # 사람이 화면에서 사라졌으니 다시 인사할 수 있게 초기화
            self.no_detection_count += 1
            self._search_if_needed()
            return

        # 사람을 다시 찾았으니 탐색 상태 해제하고, 지금 관절 자세 전부를 "마지막으로 본 자세"로 기억해둠
        self.no_detection_count = 0
        self.searching = False
        self.search_hold_count = 0
        self.last_seen_position = list(self.current_joint_position)

        # 여러 사람이 한 화면에 잡히면, 확신도가 아니라 "박스가 가장 큰(=화면에 가장 크게
        # 보이는=제일 가까운) 사람"을 고름. 박스 높이를 거리 지표로 이미 쓰고 있으므로
        # (box_h), 같은 지표로 제일 가까운 사람을 고르는 것과 같음.
        box_heights_all = (boxes.xyxy[:, 3] - boxes.xyxy[:, 1])
        best_idx = int(box_heights_all.argmax())
        x1, y1, x2, y2 = boxes.xyxy[best_idx].tolist()
        box_h = y2 - y1

        # 얼굴 목표 좌표: 코(nose) 키포인트가 충분히 확실하게 보이면 그 좌표를 그대로 씀
        # (실제 얼굴 위치라 훨씬 정확함). 사람이 뒤돌아 있는 등 코가 안 보이면, 기존처럼
        # 박스 위쪽 15% 지점(HEAD_Y_FRACTION)으로 어림짐작함(대략적인 얼굴 높이).
        face_x, face_y, used_nose = None, None, False
        if keypoints is not None and keypoints.conf is not None and len(keypoints.conf) > best_idx:
            nose_conf = float(keypoints.conf[best_idx][NOSE_KEYPOINT_INDEX])
            if nose_conf >= KEYPOINT_CONF_THRESHOLD:
                nose_xy = keypoints.xy[best_idx][NOSE_KEYPOINT_INDEX].tolist()
                face_x, face_y = int(nose_xy[0]), int(nose_xy[1])
                used_nose = True
        if not used_nose:
            face_x = int((x1 + x2) / 2)
            face_y = int(y1 + (y2 - y1) * HEAD_Y_FRACTION)

        self.get_logger().info(
            f"사람 박스: x=({x1:.0f}~{x2:.0f}), y=({y1:.0f}~{y2:.0f}), "
            f"얼굴 목표=({face_x},{face_y}){' [코 인식]' if used_nose else ' [어림짐작]'}, "
            f"박스 높이(거리 지표): {box_h:.0f}"
        )

        if self.mode != "AUTO":
            return

        if not self.joint_state_received:
            self.get_logger().info("아직 joint_states를 못 받아서 대기 중...")
            return

        # 히스테리시스: 아직 DONE 상태인데 사람이 다시 멀어졌으면(박스가 작아졌으면) IDLE로 되돌려서
        # 다음에 다시 가까워지면 재인사 가능하게 함.
        if self.greet_state == "DONE" and box_h < GREET_BOX_HEIGHT - GREET_RESET_MARGIN:
            self.greet_state = "IDLE"

        if box_h > GREET_BOX_HEIGHT and self.greet_state == "IDLE":
            self.get_logger().info("사람이 충분히 가까움 -> 인사(그리퍼 닫았다 열기) 시작")
            self.greet_state = "CLOSING"
            self.move_gripper(GRIPPER_CLOSE)
            return  # 인사하는 동안에는 팔을 더 움직이지 않음

        if self.greet_state in ("CLOSING", "OPENING"):
            return  # 그리퍼 액션 응답을 기다리는 중이면 팔은 가만히 둠

        if self.arm_goal_in_progress:
            return

        error_x = face_x - image_w // 2
        error_y = face_y - image_h // 2

        delta_joint1 = self._clamp(-error_x * X_GAIN, -MAX_STEP, MAX_STEP)
        delta_y = self._clamp(-error_y * Y_GAIN, -MAX_STEP, MAX_STEP)

        new_position = list(self.current_joint_position)
        new_position[0] += delta_joint1
        new_position[1] += delta_y
        new_position[2] += delta_y
        new_position[3] += delta_y

        self.send_joint_positions(new_position)

    # ------------------------------------------------------------------
    # GESTURE 모드: 손동작으로 팔을 조작함.
    #   - 손바닥을 쫙 펴거나(Open_Palm) 주먹을 쥐면(Closed_Fist): 정지
    #   - 손가락 4개(엄지 제외 검지~새끼)를 펴면: 디폴트 자세로 이동 (한 번만)
    #   - 손가락 2개를 펴면: 그리퍼를 조금씩 계속 열기 (자세 유지하는 동안 계속)
    #   - 손가락 3개를 펴면: 그리퍼를 조금씩 계속 닫기 (자세 유지하는 동안 계속)
    #   - 그 외에는 "엄지손가락이 화면에서 가리키는 방향"으로 팔을 조금씩 움직임
    #     (손목 -> 엄지 끝 방향 벡터를 계산해서, x성분은 joint1, y성분은 joint2~4에 반영)
    # ------------------------------------------------------------------
    def gesture_callback(self, img, image_w, image_h):
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        result = self.gesture_recognizer.recognize(mp_image)

        debug_img = img.copy()
        if not result.hand_landmarks:
            cv2.putText(debug_img, "no hand", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            self._show_and_stream(debug_img)
            return  # 손이 안 보이면 아무것도 안 함 (안전하게 정지)

        landmarks = result.hand_landmarks[0]
        gesture_name = result.gestures[0][0].category_name if result.gestures and result.gestures[0] else "None"
        cv2.putText(debug_img, gesture_name, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # MediaPipe 손 랜드마크 좌표는 0~1 사이의 비율(정규화된 값)이라 실제 픽셀로 바꿔줌.
        # 화면에 손목(파란 점) -> 엄지 끝(빨간 점) 을 잇는 화살표로 직접 그려서, 엄지를 정말
        # 인식하고 있는지/방향이 어느 쪽으로 잡히는지 눈으로 바로 확인할 수 있게 함
        # (로그 숫자만으로는 "인식이 되는 건지 아닌지 모르겠다"는 문제가 있었음).
        wrist = landmarks[WRIST_LANDMARK]
        thumb_tip = landmarks[THUMB_TIP_LANDMARK]
        wrist_px = (int(wrist.x * image_w), int(wrist.y * image_h))
        thumb_px = (int(thumb_tip.x * image_w), int(thumb_tip.y * image_h))
        cv2.circle(debug_img, wrist_px, 4, (255, 0, 0), -1)       # 손목 = 파란 점
        cv2.circle(debug_img, thumb_px, 4, (0, 0, 255), -1)       # 엄지 끝 = 빨간 점
        cv2.arrowedLine(debug_img, wrist_px, thumb_px, (0, 255, 255), 2, tipLength=0.3)  # 노란 화살표

        dx = (thumb_tip.x - wrist.x) * image_w  # 오른쪽으로 갈수록 +
        dy = (thumb_tip.y - wrist.y) * image_h  # 아래쪽으로 갈수록 +
        length = math.hypot(dx, dy)
        if length >= 1e-6:
            dx, dy = dx / length, dy / length  # 방향만 남기고 크기는 1로 정규화
            cv2.putText(debug_img, f"dx={dx:.2f} dy={dy:.2f}", (5, image_h - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

        self._show_and_stream(debug_img)

        finger_count = self._count_extended_fingers(landmarks)
        cv2.putText(debug_img, f"fingers={finger_count}", (5, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 0, 255), 1)
        self._show_and_stream(debug_img)  # 손가락 개수 표시까지 더해서 다시 스트리밍

        if gesture_name in STOP_GESTURE_NAMES:
            self.get_logger().info(f"{gesture_name} 인식 -> 정지")
            return  # 손바닥을 펴거나 주먹을 쥐면 그 자리에서 정지

        # 편 손가락 개수(엄지 제외, 검지~새끼 4개 중)로 그리퍼/자세를 조작함. 자세를 유지하는
        # 동안 계속 이 코드가 반복 호출되므로, "이미 보낸 명령이 아직 진행 중이면 새로 안 보냄"
        # (arm_goal_in_progress / gripper_goal_in_progress) 방식으로 계속 조금씩 움직이게 함 -
        # 매번 새로 "지금부터 다시 시작"하는 게 아니라 자연스럽게 이어서 움직임.
        #   4개(전부 폄) -> 디폴트 자세로 이동
        #   2개 -> 그리퍼 열기 쪽으로 조금씩 계속 이동
        #   3개 -> 그리퍼 닫기 쪽으로 조금씩 계속 이동
        if finger_count == GRIPPER_DEFAULT_FINGER_COUNT:
            if not self.arm_goal_in_progress:
                self.get_logger().info("손가락 4개 인식 -> 디폴트 자세로 이동")
                self.reset_to_default()
            return

        if finger_count in (GRIPPER_OPEN_FINGER_COUNT, GRIPPER_CLOSE_FINGER_COUNT):
            if not self.gripper_goal_in_progress:
                opening = finger_count == GRIPPER_OPEN_FINGER_COUNT
                step = GRIPPER_RAMP_STEP if opening else -GRIPPER_RAMP_STEP
                new_gripper_position = self._clamp(
                    self.current_gripper_position + step, GRIPPER_CLOSE, GRIPPER_OPEN
                )
                self.get_logger().info(
                    f"손가락 {finger_count}개 인식 -> 그리퍼 {'열기' if opening else '닫기'} "
                    f"({self.current_gripper_position:.3f} -> {new_gripper_position:.3f})"
                )
                self.move_gripper(new_gripper_position)
            return  # 그리퍼 동작 중에는 팔은 움직이지 않음 (사용자 요청)

        if self.mode != "GESTURE":  # 콜백 도중 모드가 바뀌었으면(예: 웹에서 전환) 무시
            return
        if not self.joint_state_received or self.arm_goal_in_progress:
            return
        if length < 1e-6:
            return

        self.get_logger().info(f"엄지 방향: dx={dx:.2f}, dy={dy:.2f} ({gesture_name})")

        # AUTO 모드(error_y가 음수=목표가 위쪽 -> delta_y가 음수 -> 카메라가 위로 가도록 실측
        # 조정함, Y_GAIN<0 참고)와 같은 부호 규칙을 그대로 씀: dy가 음수(엄지가 위를 가리킴)이면
        # GESTURE_VERTICAL_SIGN도 음수가 곱해져서 vertical_delta가 음수가 되어야 함.
        # Y_GAIN이 음수이므로 GESTURE_VERTICAL_SIGN = -1 이면 dy(음수) * -1 = 양수가 되어 버려서
        # 잘못됨 -> 그냥 dy를 그대로 곱해서 부호를 맞춤(아래 검증 필요, 처음엔 작은 스텝으로 확인).
        new_position = list(self.current_joint_position)
        new_position[0] += -dx * GESTURE_STEP  # 엄지가 오른쪽을 가리키면 joint1도 그쪽으로
        vertical_delta = dy * GESTURE_STEP
        new_position[1] += vertical_delta
        new_position[2] += vertical_delta
        new_position[3] += vertical_delta
        self.send_joint_positions(new_position)

    @staticmethod
    def _count_extended_fingers(landmarks) -> int:
        # 검지~새끼(엄지 제외) 4개 중, "끝(tip)"이 "중간 마디(pip)"보다 손목에서 더 멀리
        # 떨어져 있으면 그 손가락은 "펴짐"으로 셈. 엄지는 다른 손가락과 움직이는 방향이 달라서
        # 이 방식으로는 잘 안 맞아 세지 않음 - 검지~새끼 4개만으로도 2개/3개 구분에는 충분함.
        wrist = landmarks[WRIST_LANDMARK]
        count = 0
        for tip_idx, pip_idx in FINGER_TIP_PIP_LANDMARKS:
            tip = landmarks[tip_idx]
            pip = landmarks[pip_idx]
            dist_tip = math.hypot(tip.x - wrist.x, tip.y - wrist.y)
            dist_pip = math.hypot(pip.x - wrist.x, pip.y - wrist.y)
            if dist_tip > dist_pip:
                count += 1
        return count

    def _show_and_stream(self, debug_img):
        cv2.imshow("img", debug_img)
        cv2.waitKey(10)
        ok, encoded = cv2.imencode(".jpg", debug_img)
        if ok:
            self.latest_gripper_jpeg = encoded.tobytes()

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _search_if_needed(self):
        # 사람이 NO_DETECTION_SEARCH_AFTER 프레임 넘게 안 보이면, joint1을 천천히 좌우로
        # 왕복시켜서 카메라가 다른 방향을 보게 함(사람이 다시 보이면 image_callback이
        # no_detection_count를 0으로 리셋하고 원래 추적 로직으로 즉시 돌아감).
        if self.mode != "AUTO":
            return
        if not self.joint_state_received or self.arm_goal_in_progress:
            return
        if self.greet_state != "IDLE":
            return  # 인사 동작 중에는 탐색하지 않음
        if self.no_detection_count <= NO_DETECTION_SEARCH_AFTER:
            return

        current_joint1 = self.current_joint_position[0]

        # 세로 방향(joint2~4)은 탐색 내내 "마지막으로 사람을 봤을 때"의 값으로 고정해둠.
        # 처음엔 joint1(좌우)만 되돌리고 joint2~4는 놓친 순간 값 그대로 뒀는데, 사람을 쫓아가다
        # (특히 얼굴 쪽을 보려고 카메라를 계속 들어올리다가) 세로 각도가 어긋난 채로 놓치면,
        # 좌우만 아무리 훑어도 세로가 안 맞아서 다시 못 찾는 문제가 있었음(실제로 겪음).
        # 그래서 탐색 중엔 매번 joint2~4를 마지막으로 맞았던 값으로 다시 고정하고 joint1만 훑음.
        base_position = (
            list(self.last_seen_position) if self.last_seen_position is not None
            else list(self.current_joint_position)
        )

        # 마지막으로 본 위치를 알면, 그 근처 좁은 범위(LOCAL_SEARCH_RANGE) 안에서만 왕복함.
        # 처음엔 SEARCH_MIN~SEARCH_MAX(관절 전체 범위)를 다 훑게 했는데, 그러면 "그 방향으로
        # 돌아가기" 시작은 맞게 해놓고도 목표 지점을 그냥 지나쳐서 반대쪽 끝까지 계속 가버리는
        # 문제가 있었음(실제로 겪음 - 사람은 +0.57인데 팔이 -1.9까지 가버림). 사람이 방금 전까지
        # 거기 있었던 게 확실하므로, 그 근처만 좁게 왔다갔다 하는 게 훨씬 안전하고 효율적임.
        if self.last_seen_position is not None:
            last_seen_joint1 = self.last_seen_position[0]
            local_min = self._clamp(last_seen_joint1 - LOCAL_SEARCH_RANGE, SEARCH_MIN, SEARCH_MAX)
            local_max = self._clamp(last_seen_joint1 + LOCAL_SEARCH_RANGE, SEARCH_MIN, SEARCH_MAX)
        else:
            local_min, local_max = SEARCH_MIN, SEARCH_MAX

        if not self.searching:
            self.searching = True
            self.search_hold_count = 0
            if self.last_seen_position is not None:
                self.search_direction = -1 if current_joint1 > last_seen_joint1 else 1
                # 아직 마지막으로 본 자세로 안 돌아가 있으면, 좌우/세로 전부 그 자세로
                # 한 번에 맞춰서 정확히 그 지점부터 다시 훑기 시작함.
                if abs(current_joint1 - last_seen_joint1) > SEARCH_STEP:
                    self.send_joint_positions(base_position)
                    return

        # 마지막으로 본 자세 근처에 이미 도착해 있으면, 곧바로 옆으로 더 움직이지 말고 그
        # 자리에서 몇 번(SEARCH_HOLD_FRAMES) 더 확인해봄 - 도착 직후 자세가 아직 안정되는 중
        # 이거나 타이밍이 안 맞아서 첫 프레임에 인식이 안 될 수 있었음(실제로 겪은 문제).
        if self.last_seen_position is not None and abs(current_joint1 - last_seen_joint1) <= SEARCH_STEP:
            if self.search_hold_count < SEARCH_HOLD_FRAMES:
                self.search_hold_count += 1
                return  # 움직이지 않고 대기 - 다음 추론 프레임에서 다시 확인됨

        if current_joint1 >= local_max:
            self.search_direction = -1
        elif current_joint1 <= local_min:
            self.search_direction = 1

        new_position = list(base_position)
        new_position[0] = self._clamp(
            current_joint1 + self.search_direction * SEARCH_STEP, local_min, local_max
        )
        self.send_joint_positions(new_position)

    def joint_callback(self, msg: JointState):
        name_to_position = dict(zip(msg.name, msg.position))
        try:
            self.current_joint_position = [name_to_position[name] for name in JOINT_NAMES]
        except KeyError:
            return
        self.joint_state_received = True
        # 그리퍼(손가락) 현재 위치. GESTURE 모드에서 손동작을 유지하는 동안 이 값을 기준으로
        # 조금씩 열고/닫히게 하려고 별도로 기억해둠 (joint1~4와 달리 JOINT_NAMES에는 없음).
        if "gripper_left_joint" in name_to_position:
            self.current_gripper_position = name_to_position["gripper_left_joint"]

    # ------------------------------------------------------------------
    def reset_to_default(self):
        """웹뷰의 "디폴트 위치로 옮기기" 버튼에서 호출됨. 모드와 상관없이 DEFAULT_POSITION으로 이동."""
        self.get_logger().info("디폴트 자세로 이동")
        self.send_joint_positions(list(DEFAULT_POSITION), wait_for_previous=False)

    # ------------------------------------------------------------------
    def handle_key(self, key: str):
        if key == "m":
            # MANUAL -> AUTO(사람 추적) -> GESTURE(손동작 조작) -> 다시 MANUAL 순서로 돌아감
            next_mode = {"MANUAL": "AUTO", "AUTO": "GESTURE", "GESTURE": "MANUAL"}
            self.mode = next_mode[self.mode]
            self.get_logger().info(f"모드 전환 -> {self.mode}")
            return

        if key == "z":
            self.move_gripper(GRIPPER_OPEN)
            return
        if key == "x":
            self.move_gripper(GRIPPER_CLOSE)
            return

        if self.mode != "MANUAL":
            return

        if key not in MANUAL_KEY_BINDINGS:
            return
        if not self.joint_state_received:
            self.get_logger().info("아직 joint_states를 못 받아서 대기 중...")
            return

        joint_index, delta = MANUAL_KEY_BINDINGS[key]
        new_position = list(self.current_joint_position)
        new_position[joint_index] += delta
        self.send_joint_positions(new_position, wait_for_previous=False)
        self.get_logger().info(f"[MANUAL] joint{joint_index + 1} -> {new_position[joint_index]:.3f}")

    # ------------------------------------------------------------------
    def move_gripper(self, position: float, max_effort=10.0, timeout_sec=5.0):
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().info("gripper_controller Action 서버를 찾지 못했습니다.")
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        self.gripper_goal_in_progress = True
        send_goal_future = self.gripper_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.gripper_goal_callback)

    def gripper_goal_callback(self, future: Future):
        goal_handle = future.result()  # type: ignore
        if not goal_handle.accepted:
            # 목표가 거절된 경우 여기서 바로 풀어주지 않으면 gripper_goal_in_progress가 계속
            # True로 남아서, GESTURE 모드에서 그 뒤로 영영 새 그리퍼 명령을 못 보내고 조용히
            # 멈춰버림(move_joint의 goal_joint_callback과 같은 이유로 겪은 문제 - 그때 이미
            # 고쳐뒀던 걸 여기엔 빠뜨렸었음).
            self.gripper_goal_in_progress = False
            self.get_logger().info("그리퍼 목표가 거절되었습니다.")
            return
        get_result_future = goal_handle.get_result_async()  # type: ignore
        get_result_future.add_done_callback(self.gripper_get_result_callback)

    def gripper_get_result_callback(self, future: Future):
        self.gripper_goal_in_progress = False
        result: GripperCommand_GetResult_Response = future.result()  # type: ignore
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"그리퍼 이동 성공: {result.result.position}")
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("그리퍼 이동 중단됨")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("그리퍼 이동 취소됨")

        # 인사 상태 머신 진행: 닫기가 끝났으면 열기를 보내고, 열기가 끝났으면 완료 처리.
        # AUTO 모드에서 인사하는 도중에만 해당(수동 z/x 키 입력 때는 greet_state가 IDLE이라 무시됨).
        if self.greet_state == "CLOSING":
            self.greet_state = "OPENING"
            self.move_gripper(GRIPPER_OPEN)
        elif self.greet_state == "OPENING":
            self.greet_state = "DONE"
            self.get_logger().info("인사 완료")

    # ------------------------------------------------------------------
    def send_joint_positions(self, positions, wait_for_previous=True):
        if wait_for_previous and self.arm_goal_in_progress:
            return
        # 어떤 경로(수동 조작/자동 추적/탐색)로 계산됐든, 실제로 로봇에 보내기 직전에 항상
        # 안전 여유 한계(JOINT_LIMITS_SAFE)로 한 번 더 clamp함 - 계산 과정에서 실수로 하드웨어
        # 한계 근처까지 값이 나가도 여기서 걸러짐.
        safe_positions = [
            self._clamp(pos, *JOINT_LIMITS_SAFE[name])
            for pos, name in zip(positions, JOINT_NAMES)
        ]
        point = JointTrajectoryPoint()
        point.positions = safe_positions
        # 이동 한 번에 걸리는 시간. 300ms -> 150ms까지 줄였다가 너무 빠르다는 피드백을 받아서
        # 중간인 200ms로 다시 늘림.
        point.time_from_start = Duration(sec=0, nanosec=200_000_000)
        self.move_joint(point)

    def move_joint(self, point: JointTrajectoryPoint):
        if not self.joint_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("joint_controller Action 서버를 찾지 못했습니다.")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.frame_id = "move_manipulator"
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points.append(point)  # type: ignore

        self.arm_goal_in_progress = True
        send_goal_future = self.joint_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_joint_callback)

    def goal_joint_callback(self, future: Future):
        goal_handle = future.result()  # type: ignore
        if not goal_handle.accepted:
            self.arm_goal_in_progress = False
            self.get_logger().info("팔 목표가 거절되었습니다.")
            return
        get_result_future = goal_handle.get_result_async()  # type: ignore
        get_result_future.add_done_callback(self.get_joint_result_callback)

    def get_joint_result_callback(self, future: Future):
        self.arm_goal_in_progress = False
        result: FollowJointTrajectory_GetResult_Response = future.result()  # type: ignore
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"팔 이동 성공: {result.result.error_string}")
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("팔 이동 중단됨")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("팔 이동 취소됨")


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowProject()

    settings = termios.tcgetattr(sys.stdin)
    print(
        "사람 추적 프로젝트 (시작 모드: MANUAL)\n"
        "  q/a : joint1 +/-   w/s : joint2 +/-   e/d : joint3 +/-   r/f : joint4 +/-\n"
        "  z   : 그리퍼 열기   x   : 그리퍼 닫기\n"
        "  m   : MANUAL -> AUTO -> GESTURE -> MANUAL 모드 순환 전환\n"
        "  Ctrl+C : 종료\n"
    )

    tty.setraw(sys.stdin.fileno())
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key(timeout=0.0)
            if key == "\x03":
                break
            if key:
                node.handle_key(key)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()


if __name__ == "__main__":
    main()
