# 아루코 박스를 매니퓰레이터 주변 "자리"에서 다른 "자리"로 옮기는 프로젝트입니다.
#
# [비유로 먼저 이해하기]
#   이 파일은 로봇에게 "몇 번 자리가 어디인지"를 사람이 먼저 손잡고 가르쳐준 다음,
#   천장 카메라로 "몇 번 박스가 지금 어디 있는지"를 스스로 보고 있다가,
#   "0번 박스를 3번 자리로 옮겨" 라고 말하면 그대로 해주는 심부름꾼을 만드는 것과 같습니다.
#   사람이 한 번 가르쳐두면(TEACH), 로봇은 그 자리를 계속 기억합니다(파일로 저장).
#   박스의 "지금 위치"는 사람이 안 알려줘도 되고, 천장 카메라가 실시간으로 압니다.
#
# [자리(slot) 규칙]
#   매니퓰레이터를 중심으로 빙 둘러(TOTAL_SLOTS, 지금은 4개 = 90도씩),
#   전부 같은 거리("mid" 자세)에 놓습니다. 자리 번호는 그냥 1번부터 순서대로.
#   (참고: 실제로 "몇 번 방향이 몇 도"인지는 이 파일이 정하는 게 아니라, 사람이 MANUAL로
#    팔을 움직여서 원하는 위치를 직접 정하고 번호만 붙이는 것입니다. 즉 자리 배치도는
#    가르치는 사람 머릿속에 있고, 이 파일은 "그 번호에 대응하는 관절 각도"만 기억합니다.
#    단, 천장 카메라로 "자리 점유"를 판단할 때는 모든 자리가 SLOT_RADIUS_M 반지름의
#    원 위에 있다고 가정하므로, 자리를 새로 가르칠 땐 이 가정에 맞게 만들어야 함.)
#
# [두 가지 모드]
#   - TEACH  모드(키보드로 팔 조작): 팔을 원하는 자리로 움직인 뒤 'g' 키로 그 자세를 저장
#   - COMMAND 모드('c' 키로 진입): "0 3" 처럼 "마커id 도착자리"를 글자로 입력하면
#     로봇이 천장 카메라로 그 마커가 지금 어느 자리에 있는지 스스로 찾아서
#     (그 자리로 이동 -> 집기 -> 도착자리로 이동 -> 놓기)를 수행
#
# [카메라 / 아루코 인식 - 카메라 2개]
#   1) 그리퍼(손목) 카메라: "camera" 창. 아루코 마커(DICT_4X4_50)를 찾아서 테두리 + 대략적인
#      거리(m)를 표시. 눈으로 확인하는 용도이고 동작에는 영향 없음.
#   2) 천장 고정 카메라: "overhead" 창. 작업공간 전체를 위에서 내려다보며 마커를 찾아서
#      "어느 자리(slot)에 있는지"까지 계산함. 이 결과는 실제로 동작에 씀 - COMMAND로
#      이미 박스가 있는 자리로 옮기려고 하면 거부됩니다 (self.slot_occupancy).
#   거리는 카메라를 실제로 보정(calibration)한 게 아니라 SDF에 적어둔 시야각(horizontal_fov)으로
#   추정한 값이라 정확한 실측치는 아니고 "대략 이 정도"로만 참고하세요.
#
# [아루코 박스 2개]
#   aruco_cube(마커 id=0)와 aruco_cube_2(마커 id=1), 두 개가 world에 기본으로 있습니다.
#   서로 다른 자리에서 시작하고, 어느 쪽이든 COMMAND로 옮길 수 있습니다.
#
# [자리를 저장하는 위치]
#   config/box_slots.yaml 파일에 저장됩니다. --symlink-install 로 빌드했다면 이 파일은
#   install 폴더가 아니라 src(원본) 폴더의 파일을 직접 가리키는 바로가기라서,
#   프로그램이 저장한 내용이 그대로 src 쪽 파일에도 남아 git으로 관리할 수 있습니다.
#
# 키 배치:
#   q/a : joint1 +/-      w/s : joint2 +/-      e/d : joint3 +/-      r/f : joint4 +/-
#   z   : 그리퍼 열기      x   : 그리퍼 닫기
#   n   : 자리 번호 커서 다음으로 (1~TOTAL_SLOTS, 팔은 안 움직이고 "지금 몇 번을 가르칠지"만 바뀜)
#   b   : 자리 번호 커서 이전으로
#   g   : 지금 팔 자세를 "현재 커서 번호" 자리로 저장 (기록/gather)
#   c   : COMMAND 모드로 진입해서 "마커id 도착자리" 명령 입력 (예: 0 3)
#   Ctrl+C : 종료
#
# 실행 방법:
#   1) colcon build --symlink-install --packages-select redball_sort_project
#   2) source install/setup.bash
#   3) ros2 run redball_sort_project box_sort_project

import os
import select
import sys
import termios
import time
import tty

import cv2
import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker

# RViz에서 카메라 화면을 보여줄 프레임 이름. /gripper_camera/camera_info 의 실제 frame_id를
# 그대로 확인해서 씀 (짐작으로 적으면 TF가 하나도 안 맞아서 RViz에 아무것도 안 뜸).
CAMERA_FRAME = "open_manipulator_x/link5/gripper_camera"

# 천장 고정 카메라. world의 (0, 0, 0.8)에서 수직 아래를 봄 (empty_world.sdf의
# overhead_camera 모델 pose와 동일해야 함). 팔에 안 달려있고 완전히 고정이라
# TF 프레임 이름도 그냥 상수로 둠 (링크가 아니라 world 트리 밖 static 모델이라 TF 트리에 안 걸림).
OVERHEAD_CAMERA_FRAME = "overhead_camera"
OVERHEAD_CAMERA_HEIGHT_M = 0.8

# ---- 튜닝 상수 ----
STEP = 0.05               # 키 한 번 누를 때 관절이 움직이는 양(라디안). keyboard_manipulator.py 와 동일.
GRIPPER_OPEN = 0.019
GRIPPER_CLOSE = -0.01
ANGULAR_SPEED_LIMIT_RAD_S = 0.4  # 관절이 이동할 때 허용하는 최대 각속도(라디안/초). 그리퍼가 비어있을 때
                          # (박스를 집으러 가는 길)만 이 속도로 이동함.
                          # 항상 이 속도 이하가 되도록 이동 시간을 거리에 비례해서 늘림.
GRIP_SPEED_LIMIT_RAD_S = 0.15  # 박스를 쥔 채로 이동할 때만 쓰는 더 낮은 각속도(라디안/초).
                          # 실제로 겪은 문제: 박스를 쥔 채로 ANGULAR_SPEED_LIMIT_RAD_S(0.4)만큼 빠르게
                          # 돌리면 그립이 못 버티고 박스가 튕겨나감(실측). 자리가 늘어나서(4개, 최대
                          # 180도 회전) 오히려 각속도가 빨라지는 문제 때문에 이동 시간을 거리에 비례해서
                          # 늘리는 방식은 그대로 쓰되, 쥐고 있을 때는 이 더 낮은 상한을 씀.
SAFE_LIFT_JOINT2 = 0.40   # 자리 사이를 이동(joint1 회전)할 때 항상 이 joint2 값까지 들어올린 뒤 돎.
                          # 실측: joint2=0.40일 때 그리퍼 높이가 바닥에서 약 14.6cm - 박스 높이(5cm)보다
                          # 훨씬 높아서 회전 중간에 다른 자리에 있는 박스를 안 침. (joint2=0.70(예전 값)은
                          # 겨우 6.6cm라 박스를 치고 지나가는 게 실제로 확인됨 - 부족했음.)
                          # 순서: (현재 각도에서 들어올리기) -> (들어올린 채로 목표 각도까지 회전) ->
                          # (목표 각도에서 내려놓기). _safe_move_to() 참고.
TOTAL_SLOTS = 4           # 2자리 픽앤플레이스가 안정적으로 검증되어 4자리로 확장함
SLOT_RADIUS_M = 0.283     # 자리들이 놓인 원의 반지름(m). "mid" 자세의 실측 도달 거리와 동일.
                          # 자리의 world (x,y) = (SLOT_RADIUS_M*cos(joint1), SLOT_RADIUS_M*sin(joint1))
                          # -> 천장 카메라로 본 마커의 world 위치와 비교해서 "이 자리에 박스가 있는지" 판단.
OCCUPANCY_MATCH_DIST_M = 0.18  # 마커의 world (x,y)가 어떤 자리 위치와 이 거리(m) 안이면 "그 자리에 있다"고 판단.
                          # 자리 사이 간격(90도, 반지름 0.283m 기준 약 40cm 직선거리)보다 훨씬 작게 잡아서
                          # 옆 자리와 헷갈리지 않게 함. 실제로 겪은 문제: 처음엔 0.1(10cm)이었는데,
                          # 실제 픽앤플레이스 배치 오차가 8~11cm까지도 나서(그리퍼가 놓는 위치가 항상
                          # 완벽하게 정확하진 않음) 정상적으로 자리에 놓인 박스도 "빈 자리"로 오판되는
                          # 경우가 있었음. 옆 자리까지의 거리(약 40cm)에 비하면 여유가 충분해서 0.18로 늘림.
OCCUPANCY_MISS_LIMIT = 10  # 천장 카메라가 이 프레임 수만큼 연속으로 마커를 못 보면 그제서야
                          # "그 자리 비었음"으로 처리함. 실제로 겪은 문제: 인식이 가끔 한두 프레임씩
                          # 놓치는 게 있어서, 그 순간에 점유 확인을 하면 "비어있음"으로 잘못 판단해서
                          # 이미 박스가 있는 자리로도 이동이 그냥 실행돼버림.
MOVE_TIMEOUT_SEC = 60.0   # COMMAND 실행 중 "팔이 도착할 때까지" 기다리는 최대 시간 (더 걸리면 포기).
                          # 180도 회전은 ANGULAR_SPEED_LIMIT_RAD_S 기준으로도 약 21초가 걸리므로
                          # 여유있게 잡음. 실제로 겪은 문제: 시스템 부하가 높을 때 실제로는 제때
                          # 도착했는데도 액션 결과 콜백이 늦게 와서 응답을 못 받아 "타임아웃"으로
                          # 잘못 실패 처리되는 경우가 있었음 (팔은 정상적으로 목표에 도착해 있었음).
GRIPPER_TIMEOUT_SEC = 20.0  # 그리퍼 열기/닫기 액션 응답을 기다리는 최대 시간. 원래 5초였는데,
                          # MOVE_TIMEOUT_SEC(팔 이동)은 부하 대비 60초로 늘려놓고 이건 안 늘려서
                          # "그리퍼를 닫는 데 실패했습니다" 오류가 계속 났음 - 실제로는 액션이 늦게
                          # 응답한 것뿐인데 짧은 타임아웃 때문에 실패로 잘못 처리된 것으로 추정.
SETTLE_SEC = 1.0          # 팔이 도착한 뒤, 그리퍼를 움직이기 전에 흔들림이 가라앉기를 기다리는 시간
GRASP_SETTLE_SEC = 1.5    # 그리퍼를 "닫은" 직후에만 추가로 더 기다리는 시간(초). 실제로 겪은 문제:
                          # 집자마자 바로 팔을 들어올리기 시작하면(특히 시스템 부하가 높아 물리 스텝이
                          # 밀릴 때), 물리엔진이 그리퍼-박스 접촉을 아직 안정화시키기 전이라 겹친 접촉이
                          # 한번에 풀리면서 박스가 튕겨나감(실측: 목표에서 60cm 이상 벗어난 위치로 날아감,
                          # 같은 명령을 접촉이 안정된 상태에서 다시 실행하니 정상적으로 도착함). 집은 뒤
                          # SETTLE_SEC에 이 시간을 더해서 접촉이 확실히 가라앉은 뒤에 움직이기 시작함.
POSITION_TOLERANCE = 0.03  # 목표 각도와 실제 각도 차이(라디안)가 이보다 크면 "도착 실패"로 취급.
                           # 실제로 겪은 문제: 액션 서버가 SUCCEEDED를 반환해도 목표에 살짝
                           # 못 미친 채로 멈추는 경우가 있어서, 결과 상태만 믿지 않고 joint_states로
                           # 직접 재확인함.
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]  # 팔 관절 4개 이름, 항상 이 순서로 다룸

# ---- 아루코 인식용 상수 ----
# (Dictionary_get/DetectorParameters_create/무료함수 detectMarkers는 OpenCV 5.0에서
#  제거되어 ArucoDetector 클래스 기반 API로 교체함)
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # aruco_cube 모델이 쓰는 것과 같은 사전
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
MARKER_SIZE_M = 0.04            # aruco_cube 위 마커의 실제 크기 (model.sdf의 aruco_top_visual 참고, 4cm)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CAMERA_HFOV_RAD = 1.0472        # open_manipulator_x.gazebo.xacro의 camera horizontal_fov 값과 동일 (60도)
# 카메라를 실제로 보정(calibration)한 적은 없어서, SDF에 적힌 시야각으로부터 초점거리(fx,fy)를
# "핀홀 카메라 모델" 공식(fx = (가로/2) / tan(시야각/2))으로 역산한 근사값입니다.
# -> 여기서 나오는 거리(m)는 참고용 근사치이지, 정밀 계측값이 아닙니다.
_FX = (IMAGE_WIDTH / 2) / np.tan(CAMERA_HFOV_RAD / 2)
_FY = _FX  # 픽셀이 정사각형이라고 가정 (aspect_ratio 보정 안 함, 근사)
CAMERA_MATRIX = np.array(
    [[_FX, 0, IMAGE_WIDTH / 2], [0, _FY, IMAGE_HEIGHT / 2], [0, 0, 1]], dtype=np.float64
)
DIST_COEFFS = np.zeros((5, 1))  # 시뮬레이션 카메라라 렌즈 왜곡은 없다고 가정


def _estimate_pose_single_markers(corners, marker_size, camera_matrix, dist_coeffs):
    # cv2.aruco.estimatePoseSingleMarkers()가 OpenCV 5.0에서 제거되어 직접 구현.
    # 마커(정사각형) 4개 코너의 3D 좌표를 알고 있으므로 solvePnP로 자세를 추정함 —
    # 예전 함수가 내부적으로 쓰던 것과 동일한 방식(SOLVEPNP_IPPE_SQUARE).
    half = marker_size / 2.0
    obj_points = np.array(
        [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
        dtype=np.float32,
    )
    rvecs, tvecs = [], []
    for c in corners:
        _ok, rvec, tvec = cv2.solvePnP(
            obj_points, c, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        rvecs.append(rvec.reshape(1, 3))
        tvecs.append(tvec.reshape(1, 3))
    return np.array(rvecs), np.array(tvecs), None

# 천장 카메라는 그리퍼 카메라와 해상도/시야각이 둘 다 달라서 따로 계산해야 함.
# empty_world.sdf의 overhead_camera 센서 값들과 반드시 같아야 함.
# 실제로 겪은 문제 2가지:
#   1) 그리퍼 카메라와 같은 60도를 그대로 썼더니, 640x480 화면비 때문에 세로 시야각이
#      가로보다 좁아져서 1번 자리(반지름 방향이 화면 세로축과 겹침)가 화면 가장자리에
#      걸려 거의 인식이 안 됐음. -> 80도로 넓혀서 세로/가로 모두 여유 있게 만듦.
#   2) 근데 80도로 넓히니 이번엔 마커가 화면에서 너무 작게(4cm 마커가 약 20px) 나와서
#      또 인식이 잘 안 됐음. -> 해상도를 640x480 -> 1280x960(2배)으로 올려서
#      마커가 충분히 크게(약 40px) 나오게 함.
OVERHEAD_CAMERA_HFOV_RAD = 1.4
OVERHEAD_IMAGE_WIDTH = 1280
OVERHEAD_IMAGE_HEIGHT = 960
_OVERHEAD_FX = (OVERHEAD_IMAGE_WIDTH / 2) / np.tan(OVERHEAD_CAMERA_HFOV_RAD / 2)
OVERHEAD_CAMERA_MATRIX = np.array(
    [
        [_OVERHEAD_FX, 0, OVERHEAD_IMAGE_WIDTH / 2],
        [0, _OVERHEAD_FX, OVERHEAD_IMAGE_HEIGHT / 2],
        [0, 0, 1],
    ],
    dtype=np.float64,
)

# key -> (관절 인덱스, 증감량)
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


def slots_file_path() -> str:
    # config/box_slots.yaml 의 실제 경로.
    # get_package_share_directory 는 "install" 쪽 경로를 돌려주는데,
    # --symlink-install 로 빌드했다면 그 경로 자체가 src/config/box_slots.yaml 을
    # 가리키는 심볼릭 링크라서, 여기 쓰면 src 파일에 그대로 저장됩니다.
    share_dir = get_package_share_directory("redball_sort_project")
    return os.path.join(share_dir, "config", "box_slots.yaml")


def get_key(timeout=0.0):
    # timeout초 동안 키 입력을 기다리다가, 안 눌리면 빈 문자열을 반환 (넌블로킹).
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if rlist else ""


def rvec_to_quaternion(rvec):
    # cv2.aruco.estimatePoseSingleMarkers가 주는 회전은 "회전 벡터(rvec)" 형태인데,
    # TF/RViz는 쿼터니언(quaternion)을 씁니다. cv2.Rodrigues로 회전행렬을 구한 뒤
    # 표준 공식으로 쿼터니언으로 바꿉니다.
    rot_matrix, _ = cv2.Rodrigues(rvec)
    m = rot_matrix
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class BoxSortProject(Node):
    def __init__(self):
        super().__init__("box_sort_project")

        self.joint_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")

        self.create_subscription(JointState, "joint_states", self.joint_callback, 10)
        self.current_joint_position = [0.0, 0.0, 0.0, 0.0]
        self.joint_state_received = False

        self.arm_goal_in_progress = False

        # 실물 로봇에서는 그리퍼 열기/닫기가 시뮬레이션과 반대로 동작함(실측 확인,
        # 2026-08-12) - 웹 대시보드가 실물 모드로 전환할 때 set_gripper_reversed(True)를
        # 호출해서 여기서만 뒤집어줌 (시뮬레이션에서는 건드리지 않음).
        self.gripper_reversed = False

        # 그리퍼가 "닫기"를 누르면 항상 끝까지(GRIPPER_CLOSE) 닫히던 걸, 1~10 단계로 닫는
        # 정도를 조절할 수 있게 함. 10=완전히 닫힘(기존과 동일한 GRIPPER_CLOSE), 1=거의 안 닫힘.
        # 웹 대시보드의 슬라이더에서 set_gripper_close_level()로 바꿈.
        self.gripper_close_level = 10

        # "지금 몇 번 자리를 가르치는 중인지" 가리키는 커서. n/b 키로 바꿈.
        self.teach_cursor = 1

        # 카메라로 아루코 박스를 보면서 거리를 표시하기 위한 구독 (동작에 영향은 안 주고 보여주기만 함)
        self.bridge = CvBridge()
        self.create_subscription(Image, "/gripper_camera/image_raw", self.image_callback, 10)

        # 천장 고정 카메라: 작업공간 전체를 내려다보면서 "어느 자리에 박스가 있는지" 판단하는 용도.
        # 이 결과는 run_command()에서 "이미 박스가 있는 자리로는 못 옮기게" 막는 데 실제로 씀
        # (그리퍼 카메라와 달리 눈으로 확인만 하는 용도가 아님).
        self.create_subscription(Image, "/overhead_camera/image_raw", self.overhead_image_callback, 10)

        # RViz에서 볼 수 있게: 테두리 그려진 화면(이미지 토픽), 마커 3D 위치(TF), id/거리 글자(Marker)
        self.annotated_image_pub = self.create_publisher(Image, "aruco_detection/image", 10)
        self.marker_text_pub = self.create_publisher(Marker, "aruco_detection/markers", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 천장 카메라로 매 프레임 다시 계산하는 "자리 번호 -> 그 자리에 있는 마커 id" (없으면 키가 안 생김).
        self.slot_occupancy: dict[int, int] = {}
        self._slot_miss_count: dict[int, int] = {}  # 디바운스용 - OCCUPANCY_MISS_LIMIT 설명 참고

        # 이번 프레임에 실제로 검출된 마커의 화면 픽셀 중심 좌표: {marker_id: (u, v)}.
        # 자리(slot)에 붙어있는지와 무관하게, "지금 화면에 보이는 마커"를 그대로 클릭으로 고르기
        # 위한 것 (slot_occupancy는 미리 가르친 자리 근처에 있을 때만 채워지므로, 자리에서
        # 살짝 벗어난 박스는 slot_occupancy로는 못 찾음).
        self.marker_pixel_positions: dict[int, tuple[float, float]] = {}
        # 이번 프레임에 실제 검출된 마커의 world (x, y) 좌표: {marker_id: (x, y)}. MoveIt 기반
        # 정밀 집기(moveit_pick_place.py)가 "미리 가르친 자리"가 아니라 지금 실제 마커가 있는
        # 좌표로 팔을 뻗을 때 씀.
        self.marker_world_positions: dict[int, tuple[float, float]] = {}

        # 천장 카메라가 지금까지 한 번이라도 본 적 있는 마커 id들 (웹 대시보드의 마커 선택
        # 드롭다운을 실제 박스 id로 채우기 위함 - 자리에 없어도(들려있어도) 계속 남아있음).
        self.known_marker_ids: set[int] = set()

        # 웹 페이지(web_control.py)에서 카메라 화면을 실시간으로 보여주기 위해, 매 프레임
        # jpg로 인코딩해서 여기 저장해둠 (web_control.py가 이 바이트를 그대로 스트리밍함).
        # cv2.imshow용 창과 별개로, 웹 스트리밍을 안 쓰는 keyboard 모드에서도 그냥 계산만 해두는
        # 정도라 성능에 큰 영향은 없음.
        self.latest_gripper_jpeg: bytes | None = None
        self.latest_overhead_jpeg: bytes | None = None

        # 저장된 자리들. {번호(int): {"joints": [j1,j2,j3,j4], "gripper": 값}}
        self.slots: dict[int, dict] = {}
        self.load_slots()

    # ------------------------------------------------------------------
    def load_slots(self):
        path = slots_file_path()
        try:
            with open(path, encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            data = {}
        # YAML에서는 키가 문자열로 저장되므로 int로 바꿔서 씀
        raw = data.get("slots", {}) or {}
        self.slots = {int(k): v for k, v in raw.items()}
        self.get_logger().info(f"저장된 자리 {len(self.slots)}개 불러옴: {sorted(self.slots.keys())}")

    def save_slots(self):
        path = slots_file_path()
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump({"slots": self.slots}, f, allow_unicode=True, sort_keys=True)

    def joint_callback(self, msg: JointState):
        # 주의: /joint_states 는 [gripper_left_joint, gripper_right_joint, joint1, joint2, joint3, joint4]
        # 순서로 옴 (그리퍼가 먼저!). msg.position을 그대로 쓰면 그리퍼 값을 joint1으로 착각하게 되므로,
        # 이름으로 정확히 찾아서 joint1~4 순서로 다시 뽑아야 함. (keyboard_manipulator.py 와 동일한 방식)
        name_to_position = dict(zip(msg.name, msg.position))
        try:
            self.current_joint_position = [name_to_position[name] for name in JOINT_NAMES]
        except KeyError:
            return  # 메시지에 아직 joint1~4 이름이 다 안 들어있으면 무시
        self.joint_state_received = True

    # ------------------------------------------------------------------
    # 카메라 콜백: 아루코 마커를 찾아서 화면에 테두리 + 거리(근사치)를 표시
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _rejected = ARUCO_DETECTOR.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            rvecs, tvecs, _obj_points = _estimate_pose_single_markers(
                corners, MARKER_SIZE_M, CAMERA_MATRIX, DIST_COEFFS
            )
            for i, marker_id in enumerate(ids.flatten()):
                marker_id = int(marker_id)  # numpy int -> 파이썬 int (안 바꾸면 ROS 메시지에 못 넣고 죽음)
                tvec = tvecs[i][0]  # 카메라 기준 (x, y, z), 단위 m
                distance = float(np.linalg.norm(tvec))
                cv2.drawFrameAxes(img, CAMERA_MATRIX, DIST_COEFFS, rvecs[i], tvecs[i], MARKER_SIZE_M * 0.5)

                # 마커 중심 픽셀 좌표 계산 (거리 글자를 마커 근처에 표시하기 위함)
                center = corners[i][0].mean(axis=0)
                cx, cy = int(center[0]), int(center[1])
                cv2.putText(
                    img,
                    f"id={marker_id} dist={distance:.3f}m",
                    (cx - 60, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
                self.get_logger().info(f"아루코 마커 감지: id={marker_id}, 거리(근사)={distance:.3f}m")

                # RViz에서 볼 수 있게 TF와 텍스트 마커도 같이 내보냄
                self.publish_marker_tf(marker_id, tvec, rvecs[i][0])
                self.publish_marker_text(marker_id, tvec, distance)

        cv2.imshow("camera", img)
        cv2.waitKey(10)

        ok, encoded = cv2.imencode(".jpg", img)
        if ok:
            self.latest_gripper_jpeg = encoded.tobytes()

        # RViz의 Image 디스플레이 패널로 같은 화면(테두리/거리 글자 포함)을 볼 수 있게 발행
        out_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        out_msg.header = msg.header
        self.annotated_image_pub.publish(out_msg)

    def publish_marker_tf(self, marker_id: int, tvec, rvec):
        # RViz의 TF 디스플레이를 켜두면, 인식된 마커 위치에 좌표축이 실시간으로 나타남.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = CAMERA_FRAME
        t.child_frame_id = f"aruco_marker_{marker_id}"
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        qx, qy, qz, qw = rvec_to_quaternion(rvec)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def publish_marker_text(self, marker_id: int, tvec, distance: float):
        # RViz의 MarkerArray/Marker 디스플레이를 켜두면, 마커 위치 위에 "id=.. dist=..m"
        # 글자가 3D 공간에 둥둥 떠 있는 것처럼 보임.
        marker = Marker()
        marker.header.frame_id = CAMERA_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "aruco_detection"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(tvec[0])
        marker.pose.position.y = float(tvec[1])
        marker.pose.position.z = float(tvec[2]) + 0.03  # 마커보다 살짝 위에 글자가 뜨도록
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.02  # 글자 크기(m)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        marker.text = f"id={marker_id}\ndist={distance:.3f}m"
        marker.lifetime.sec = 1  # 1초 넘게 새 감지가 없으면 RViz에서 자동으로 사라짐
        self.marker_text_pub.publish(marker)

    # ------------------------------------------------------------------
    # 천장 카메라 콜백: 마커를 찾아서 "어느 자리에 있는지" 계산하고, 화면에도 표시
    #
    # [world 좌표 계산 방법]
    #   천장 카메라는 world (0, 0, OVERHEAD_CAMERA_HEIGHT_M)에서 수직 아래를 보고 있음
    #   (empty_world.sdf의 overhead_camera 모델과 반드시 같은 값이어야 함).
    #   cv2.aruco가 주는 tvec은 "카메라 광학 좌표계" 기준 (x=오른쪽, y=아래, z=깊이)인데,
    #   실제로 팔을 여러 각도로 돌려가며 화면에 나온 위치와 비교해서 확인한 결과
    #   (카메라가 world 원점 바로 위에서 수직으로 아래를 보는 경우):
    #     world_x = -tvec.y,  world_y = -tvec.x
    #   가 성립함 (카메라가 world (0,0) 위에 있으므로 오프셋 없이 이 식 그대로 씀).
    # ------------------------------------------------------------------
    def overhead_image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _rejected = ARUCO_DETECTOR.detectMarkers(gray)

        detected_this_frame: dict[int, int] = {}  # slot -> marker_id, 이번 프레임에서만 본 것
        self.marker_pixel_positions = {}  # 이번 프레임 기준으로 통째로 다시 채움 (안 보이면 자동으로 빠짐)
        self.marker_world_positions = {}

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            rvecs, tvecs, _obj_points = _estimate_pose_single_markers(
                corners, MARKER_SIZE_M, OVERHEAD_CAMERA_MATRIX, DIST_COEFFS
            )
            for i, marker_id in enumerate(ids.flatten()):
                marker_id = int(marker_id)
                self.known_marker_ids.add(marker_id)
                tvec = tvecs[i][0]
                world_x = -float(tvec[1])
                world_y = -float(tvec[0])

                center = corners[i][0].mean(axis=0)
                cx, cy = int(center[0]), int(center[1])
                self.marker_pixel_positions[marker_id] = (float(cx), float(cy))
                self.marker_world_positions[marker_id] = (world_x, world_y)

                nearest_slot = self._nearest_slot(world_x, world_y)
                if nearest_slot is not None:
                    detected_this_frame[nearest_slot] = marker_id

                slot_text = f"slot={nearest_slot}" if nearest_slot is not None else "slot=?"
                cv2.putText(
                    img,
                    f"id={marker_id} ({world_x:.2f},{world_y:.2f}) {slot_text}",
                    (cx - 90, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

        # 점유 정보는 "생길 때는 바로 반영, 없어질 때는 여러 프레임 연속으로 안 보여야 반영"하는
        # 방식(디바운스)으로 갱신함. 실제로 겪은 문제: 인식이 가끔 한 프레임씩 놓치는 게 있어서,
        # 그 순간에 점유 확인을 하면 "비어있음"으로 잘못 판단해서 이미 박스가 있는 자리로도
        # 이동이 그냥 실행돼버림. OCCUPANCY_MISS_LIMIT번 연속으로 못 봐야 진짜로 없어진 걸로 침.
        changed = False
        for slot_num, marker_id in detected_this_frame.items():
            if self.slot_occupancy.get(slot_num) != marker_id:
                changed = True
            self.slot_occupancy[slot_num] = marker_id
            self._slot_miss_count[slot_num] = 0

        for slot_num in list(self.slot_occupancy.keys()):
            if slot_num in detected_this_frame:
                continue
            self._slot_miss_count[slot_num] = self._slot_miss_count.get(slot_num, 0) + 1
            if self._slot_miss_count[slot_num] >= OCCUPANCY_MISS_LIMIT:
                del self.slot_occupancy[slot_num]
                del self._slot_miss_count[slot_num]
                changed = True

        if changed:
            self.get_logger().info(f"자리 점유 현황 갱신: {self.slot_occupancy}")

        # 박스가 없어도(빈 자리여도) "여기가 몇 번 자리인지" 항상 화면에 표시함.
        self._draw_slot_labels(img)

        cv2.imshow("overhead", img)
        cv2.waitKey(10)

        ok, encoded = cv2.imencode(".jpg", img)
        if ok:
            self.latest_overhead_jpeg = encoded.tobytes()

    def find_slot_of_marker(self, marker_id: int):
        # marker_id가 지금 어느 자리에 있는지 천장 카메라 인식 결과에서 역으로 찾음.
        for slot_num, occupied_by in self.slot_occupancy.items():
            if occupied_by == marker_id:
                return slot_num
        return None

    def _slot_to_pixel(self, slot_num: int):
        # _nearest_slot()의 반대 방향 계산: 자리 번호 -> world (x,y) -> 화면 픽셀 좌표.
        # world_x = -optical_y, world_y = -optical_x (overhead_image_callback 상단 설명 참고)를
        # 거꾸로 풀면 optical_x = -world_y, optical_y = -world_x.
        # 핀홀 카메라 모델로 픽셀 = 중심 + 초점거리 * (광학좌표 / 깊이).
        slot = self.slots.get(slot_num)
        if slot is None:
            return None
        angle = slot["joints"][0]
        world_x = SLOT_RADIUS_M * np.cos(angle)
        world_y = SLOT_RADIUS_M * np.sin(angle)
        depth = OVERHEAD_CAMERA_HEIGHT_M - 0.05  # 박스 윗면 높이 기준 깊이
        optical_x = -world_y
        optical_y = -world_x
        fx = OVERHEAD_CAMERA_MATRIX[0, 0]
        cx = OVERHEAD_CAMERA_MATRIX[0, 2]
        cy = OVERHEAD_CAMERA_MATRIX[1, 2]
        u = cx + fx * optical_x / depth
        v = cy + fx * optical_y / depth
        return int(u), int(v)

    def _draw_slot_labels(self, img):
        for slot_num in self.slots:
            pixel = self._slot_to_pixel(slot_num)
            if pixel is None:
                continue
            u, v = pixel
            if not (0 <= u < img.shape[1] and 0 <= v < img.shape[0]):
                continue
            occupied = slot_num in self.slot_occupancy
            color = (0, 165, 255) if occupied else (255, 255, 0)  # 점유=주황, 빈자리=하늘색
            cv2.drawMarker(img, (u, v), color, markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
            cv2.putText(
                img, f"{slot_num}", (u + 10, v + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2
            )

    def nearest_marker_for_pixel(self, px: float, py: float, max_dist_px: float = 60.0):
        # 웹 대시보드에서 박스를 고를 때 씀. 미리 가르친 자리(slot) 위치가 아니라, 지금 이 순간
        # 화면에 실제로 검출된 마커 중심(marker_pixel_positions)과 직접 비교함 - 그래서 박스가
        # 자리에서 살짝 벗어나 있어도(놓을 때 오차 등) 화면에 보이는 그대로 클릭해서 고를 수 있음.
        best_id = None
        best_dist = max_dist_px
        for marker_id, (u, v) in self.marker_pixel_positions.items():
            dist = float(np.hypot(px - u, py - v))
            if dist < best_dist:
                best_dist = dist
                best_id = marker_id
        return best_id

    def slot_world_xy(self, slot_num: int):
        # 자리 번호 -> world (x, y). MoveIt 정밀 집기의 "놓을 위치" 목표로 씀 (도착 자리는
        # 실시간 마커가 없어도(비어있는 자리) 정해야 하므로, 미리 가르친 자세의 joint1 각도로
        # 계산한 이상적인 원 위 좌표를 그대로 씀 - _nearest_slot()과 동일한 가정).
        slot = self.slots.get(slot_num)
        if slot is None:
            return None
        angle = slot["joints"][0]
        return SLOT_RADIUS_M * np.cos(angle), SLOT_RADIUS_M * np.sin(angle)

    def nearest_slot_for_pixel(self, px: float, py: float, max_dist_px: float = 100.0):
        # 웹 대시보드에서 천장 카메라 화면을 마우스로 클릭했을 때 씀. _slot_to_pixel()로
        # "이 자리는 화면 어디에 그려지는지"를 이미 알고 있으므로, 그 반대로 world 좌표를
        # 다시 계산할 필요 없이 픽셀끼리 거리 비교만 하면 됨 (더 간단하고, 카메라 기하 가정이
        # 틀려도 화면에 실제로 그려진 십자가 위치 기준이라 항상 눈에 보이는 대로 동작함).
        best_slot = None
        best_dist = max_dist_px
        for slot_num in self.slots:
            pixel = self._slot_to_pixel(slot_num)
            if pixel is None:
                continue
            u, v = pixel
            dist = float(np.hypot(px - u, py - v))
            if dist < best_dist:
                best_dist = dist
                best_slot = slot_num
        return best_slot

    def _nearest_slot(self, world_x: float, world_y: float):
        # 저장된 자리들의 world (x,y)는 joints[0](joint1, 자리 방향 각도)로부터 계산함
        # (자리들이 반지름 SLOT_RADIUS_M인 원 위에 놓여있다고 가정 - box_sort_project의
        # "mid" 자세 실측값과 같음).
        best_slot = None
        best_dist = OCCUPANCY_MATCH_DIST_M
        for slot_num, slot in self.slots.items():
            angle = slot["joints"][0]
            slot_x = SLOT_RADIUS_M * np.cos(angle)
            slot_y = SLOT_RADIUS_M * np.sin(angle)
            dist = float(np.hypot(world_x - slot_x, world_y - slot_y))
            if dist < best_dist:
                best_dist = dist
                best_slot = slot_num
        return best_slot

    # ------------------------------------------------------------------
    # 시작 직후 한 번만 호출: joint1=0(로봇 기본/home 자세)이 1번 자리(0도)와 정확히
    # 겹쳐서, 막 켰을 때는 천장 카메라가 1번 자리 박스를 못 봄(실제로 겪음). 명령을 한 번도
    # 안 내렸으면 팔이 계속 0도에 머물러 있으므로, 시작하자마자 살짝 옆으로 비켜만 둠
    # (자리 각도 자체는 안 건드림 - 실제로 집을 때는 여전히 정확히 0도로 접근함).
    # ------------------------------------------------------------------
    def park_away_from_home(self):
        for _ in range(100):
            if self.joint_state_received:
                break
            rclpy.spin_once(self, timeout_sec=0.05)  # joint_states 콜백이 돌 기회를 줘야 함
        else:
            return
        if abs(self.current_joint_position[0]) > 0.05:
            return  # 이미 누가 팔을 움직였으면(=0도가 아니면) 손대지 않음
        self._move_and_wait([0.7, 0.0, 0.0, 0.0])

    # ------------------------------------------------------------------
    # 웹 대시보드의 "매니퓰레이터 디폴트 위치로" 버튼용. 그리퍼를 열고, 관절을 전부 0(로봇
    # 기본 home 자세)으로 안전하게(들어올리기→회전→내려놓기) 되돌림. park_away_from_home()과
    # 달리 이건 사용자가 아무 때나 눌러서 쓰는 용도라 현재 각도와 상관없이 항상 동작함.
    # ------------------------------------------------------------------
    def set_gripper_reversed(self, reversed_: bool) -> None:
        self.gripper_reversed = reversed_
        self.get_logger().info(f"그리퍼 방향 반전: {reversed_}")

    def set_gripper_close_level(self, level: int) -> None:
        self.gripper_close_level = max(1, min(10, int(level)))
        self.get_logger().info(f"그리퍼 닫는 정도 -> {self.gripper_close_level}/10")

    def _current_close_position(self) -> float:
        # level=10일 때 기존과 똑같이 GRIPPER_CLOSE(완전히 닫힘), level=1일 때는 GRIPPER_OPEN에
        # 가까운(거의 안 닫힘) 값을 씀 - OPEN과 CLOSE 사이를 선형으로 나눔.
        t = self.gripper_close_level / 10.0
        return GRIPPER_OPEN + (GRIPPER_CLOSE - GRIPPER_OPEN) * t

    def _effective_gripper_position(self, position: float) -> float:
        # gripper_reversed가 켜져 있으면 OPEN<->CLOSE 축을 기준으로 값을 대칭시켜 보냄
        # (실물 로봇 전용 보정). GRIPPER_OPEN, GRIPPER_CLOSE 뿐 아니라 그 사이의 부분 닫힘
        # 값(그리퍼 닫는 정도 조절, set_gripper_close_level 참고)에도 그대로 적용됨.
        if not self.gripper_reversed:
            return position
        return GRIPPER_OPEN + GRIPPER_CLOSE - position

    def reset_arm_to_default(self) -> bool:
        self._gripper_and_wait(GRIPPER_OPEN)
        return self._safe_move_to([0.0, 0.0, 0.0, 0.0])

    # ------------------------------------------------------------------
    # 키보드 입력 처리 (TEACH 모드)
    # ------------------------------------------------------------------
    def handle_key(self, key: str):
        if key == "n":
            self.teach_cursor = self.teach_cursor % TOTAL_SLOTS + 1
            self.get_logger().info(f"현재 가르칠 자리 번호 -> {self.teach_cursor}")
            return
        if key == "b":
            self.teach_cursor = (self.teach_cursor - 2) % TOTAL_SLOTS + 1
            self.get_logger().info(f"현재 가르칠 자리 번호 -> {self.teach_cursor}")
            return

        if key == "g":
            if not self.joint_state_received:
                self.get_logger().info("아직 joint_states를 못 받아서 저장할 수 없음")
                return
            self.slots[self.teach_cursor] = {
                "joints": [round(v, 4) for v in self.current_joint_position],
                "gripper": GRIPPER_OPEN,  # 저장 시점 그리퍼 값은 모르므로 기본은 "열림"으로 기록
            }
            self.save_slots()
            self.get_logger().info(
                f"{self.teach_cursor}번 자리 저장 완료: {self.slots[self.teach_cursor]['joints']}"
            )
            return

        if key == "z":
            self.move_gripper(GRIPPER_OPEN)
            return
        if key == "x":
            self.move_gripper(self._current_close_position())
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
        self.get_logger().info(f"[TEACH] joint{joint_index + 1} -> {new_position[joint_index]:.3f}")

    # ------------------------------------------------------------------
    # 자리 사이를 안전하게 이동하는 공통 동작: (현재 각도에서 들어올리기) ->
    # (들어올린 채로 목표 각도까지 회전) -> (목표 각도에서 내려놓기).
    # 실제로 겪은 문제: 이 세 단계로 안 나누고 그냥 한 번에 목표 자세로 이동하면, 팔이
    # 회전하는 동안 다른 자리에 있는 박스를 치고 지나감(SAFE_LIFT_JOINT2 설명 참고).
    # 한때 사용자 요청으로 최단경로(대각선 직행)로 바꿨었는데, 실제로 다른 자리 박스를 치는
    # 문제가 재발해서 다시 이 3단계 방식으로 되돌림 - 속도보다 충돌 방지를 우선함.
    # ------------------------------------------------------------------
    def _safe_move_to(self, target_joints, speed_limit=ANGULAR_SPEED_LIMIT_RAD_S) -> bool:
        current = list(self.current_joint_position)
        raise_pose = [current[0], SAFE_LIFT_JOINT2, current[2], current[3]]
        if not self._move_and_wait(raise_pose, speed_limit=speed_limit):
            return False

        rotate_pose = [target_joints[0], SAFE_LIFT_JOINT2, target_joints[2], target_joints[3]]
        if not self._move_and_wait(rotate_pose, speed_limit=speed_limit):
            return False

        return self._move_and_wait(list(target_joints), speed_limit=speed_limit)

    # ------------------------------------------------------------------
    # COMMAND 모드: "마커id 도착자리" 실행 (예: 0 3 -> 마커 id=0인 박스를 3번 자리로)
    #
    # 예전에는 "출발자리 도착자리"를 사람이 직접 알려줘야 했는데, 이제 천장 카메라가
    # 어느 마커가 어느 자리에 있는지 실시간으로 알고 있으므로(self.slot_occupancy),
    # "이 박스를 어디로 옮겨"라고만 말하면 출발 자리는 카메라로 알아서 찾음.
    # ------------------------------------------------------------------
    def run_command(self, marker_id: int, to_slot: int) -> str:
        # 실패하면 사람이 읽을 메시지를 담아 RuntimeError를 던짐 (로그로만 남기면 web_control.py처럼
        # 화면이 따로 있는 경우 사용자가 실패 이유를 알 방법이 없어서, 호출한 쪽까지 전달함).
        if to_slot not in self.slots:
            msg = f"{to_slot}번 자리는 아직 안 가르쳤습니다."
            self.get_logger().info(msg)
            raise RuntimeError(msg)

        from_slot = self.find_slot_of_marker(marker_id)
        if from_slot is None:
            msg = f"마커 id={marker_id} 박스를 천장 카메라에서 못 찾았습니다."
            self.get_logger().info(msg)
            raise RuntimeError(msg)

        if from_slot == to_slot:
            msg = f"마커 id={marker_id} 박스는 이미 {to_slot}번 자리에 있습니다."
            self.get_logger().info(msg)
            raise RuntimeError(msg)

        # 천장 카메라로 본 결과, 도착 자리에 이미 "다른" 박스가 있으면 이동을 거부함.
        occupied_by = self.slot_occupancy.get(to_slot)
        if occupied_by is not None and occupied_by != marker_id:
            msg = f"{to_slot}번 자리에는 이미 마커 id={occupied_by} 박스가 있어서 이동할 수 없습니다."
            self.get_logger().info(msg)
            raise RuntimeError(msg)

        self.get_logger().info(
            f"마커 id={marker_id} 박스: {from_slot}번 자리 -> {to_slot}번 자리로 옮기기 시작"
        )
        self._pick_and_place(from_slot, to_slot)
        msg = f"마커 id={marker_id} 박스: {from_slot}번 -> {to_slot}번 이동 완료"
        self.get_logger().info(msg)
        return msg

    # ------------------------------------------------------------------
    # 카메라 없이(실물 로봇처럼 천장 카메라가 없는 경우) 쓰는 COMMAND: "거기 박스가 있다고
    # 믿고" 자리 번호만으로 바로 픽앤플레이스를 실행함. run_command()와 달리 마커 인식/점유
    # 확인을 전혀 안 하므로, 실제로 그 자리에 물체가 없거나 도착 자리에 이미 뭔가 있으면
    # 조용히 허공을 잡거나 부딪힐 수 있음 - 호출하는 쪽(web_control.py)에서 사용자에게
    # 이 위험을 미리 알려야 함.
    # ------------------------------------------------------------------
    def run_command_blind(self, from_slot: int, to_slot: int) -> str:
        if from_slot not in self.slots:
            msg = f"{from_slot}번 자리는 아직 안 가르쳤습니다."
            self.get_logger().info(msg)
            raise RuntimeError(msg)
        if to_slot not in self.slots:
            msg = f"{to_slot}번 자리는 아직 안 가르쳤습니다."
            self.get_logger().info(msg)
            raise RuntimeError(msg)
        if from_slot == to_slot:
            msg = "출발 자리와 도착 자리가 같습니다."
            self.get_logger().info(msg)
            raise RuntimeError(msg)

        self.get_logger().info(
            f"[카메라 없이] {from_slot}번 자리 -> {to_slot}번 자리로 옮기기 시작"
        )
        self._pick_and_place(from_slot, to_slot)
        msg = f"[카메라 없이] {from_slot}번 -> {to_slot}번 이동 완료"
        self.get_logger().info(msg)
        return msg

    # ------------------------------------------------------------------
    # run_command()/run_command_blind()가 공유하는 실제 픽앤플레이스 동작:
    # 그리퍼 열기 -> 출발 자리로 이동 -> 집기 -> 도착 자리로 이동(느리게) -> 놓기 -> 비켜주기.
    # ------------------------------------------------------------------
    def _pick_and_place(self, from_slot: int, to_slot: int) -> None:
        # 1) 출발 자리로 안전하게 이동 (위에서 내려오듯 접근 - 실제로 겪은 문제: 낮게 스쳐 지나가면
        #    박스를 슬쩍 건드리고, 그 상태에서 그리퍼를 닫으면 물리엔진이 겹친 충돌을 한번에
        #    풀어내면서 박스가 멀리 튕겨나감(실측: 36cm 이상 날아감).)
        if not self._gripper_and_wait(GRIPPER_OPEN):
            msg = "그리퍼를 여는 데 실패해서 중단합니다."
            self.get_logger().info(msg)
            raise RuntimeError(msg)
        if not self._safe_move_to(self.slots[from_slot]["joints"]):
            msg = "팔이 출발 자리에 도착하지 못해서 중단합니다."
            self.get_logger().info(msg)
            raise RuntimeError(msg)

        # 2) 집기
        if not self._gripper_and_wait(self._current_close_position()):
            msg = "그리퍼를 닫는 데 실패해서 중단합니다."
            self.get_logger().info(msg)
            raise RuntimeError(msg)
        time.sleep(GRASP_SETTLE_SEC)  # 접촉이 완전히 가라앉을 시간을 더 줌 (GRASP_SETTLE_SEC 설명 참고)

        # 3) 도착 자리로 안전하게 이동
        if not self._safe_move_to(self.slots[to_slot]["joints"], speed_limit=GRIP_SPEED_LIMIT_RAD_S):
            msg = "팔이 도착 자리에 도착하지 못해서 중단합니다 (그리퍼는 아직 닫힌 상태)."
            self.get_logger().info(msg)
            raise RuntimeError(msg)

        # 4) 놓기
        if not self._gripper_and_wait(GRIPPER_OPEN):
            msg = "그리퍼를 여는 데 실패했습니다 (팔은 도착 자리에 있음)."
            self.get_logger().info(msg)
            raise RuntimeError(msg)

        # 5) 놓은 자리 바로 위에서 비켜주기: 팔이 그대로 자리 위에 머물러 있으면 천장 카메라가
        # 방금 놓은 박스를 계속 못 보고, OCCUPANCY_MISS_LIMIT 프레임이 지나면 "빈 자리"로 잘못
        # 지워버림(박스는 실제로 있는데 자리 점유 현황에서만 사라짐). 들어올리기만 하고 회전은
        # 안 하므로 실패해도 박스 이동 자체는 이미 끝난 뒤라 치명적이지 않음 - 로그만 남김.
        to_joints = self.slots[to_slot]["joints"]
        lift_pose = [to_joints[0], SAFE_LIFT_JOINT2, to_joints[2], to_joints[3]]
        if not self._move_and_wait(lift_pose):
            self.get_logger().info("놓은 뒤 비켜주기 이동에 실패했습니다 (박스 이동 자체는 완료됨).")

    # ------------------------------------------------------------------
    # COMMAND 모드 전용 "확실히 기다리는" 헬퍼들.
    #
    # 왜 move_joint()/move_gripper()를 그대로 안 쓰냐면: 그 둘은 콜백 + 플래그(arm_goal_in_progress)
    # 방식이라 TEACH 모드(사람이 키를 누를 때마다 발사하고 잊어버리는 용도)에는 맞지만,
    # COMMAND 모드처럼 "다음 단계로 넘어가기 전에 반드시 결과를 확인해야 하는" 곳에는 안 맞았습니다.
    # 실제로 겪은 문제: joint_states가 100Hz로 계속 들어오는 상황에서 while + rclpy.spin_once()로
    # 수동 폴링을 하면, 액션 결과 콜백이 좀처럼 실행 기회를 못 얻고 계속 밀려서 타임아웃이 났습니다.
    # rclpy.spin_until_future_complete()는 "이 future가 끝날 때까지"를 정확히 기다려주기 때문에
    # 이 문제가 없어서, COMMAND 모드에서는 이 방식만 씁니다.
    # ------------------------------------------------------------------
    def _send_trajectory_once(self, positions, speed_limit=ANGULAR_SPEED_LIMIT_RAD_S) -> bool:
        # 목표 지점으로 한 번 보내고 액션이 SUCCEEDED를 반환할 때까지만 기다림.
        # (실제 각도가 목표에 도착했는지는 여기서 확인 안 함 - _move_and_wait에서 따로 확인)
        if not self.joint_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("joint_controller Action 서버를 찾지 못했습니다.")
            return False

        goal = FollowJointTrajectory.Goal()
        # 주의: 여기에 self.get_clock().now()(실제 시간=wall-clock)를 채우면 안 됨.
        # Gazebo 쪽 컨트롤러는 시뮬레이션 시간(use_sim_time)을 쓰는데 이 노드는 그렇지 않아서,
        # 두 시간이 안 맞아 컨트롤러가 트리거를 영영 못 걸고 응답도 안 옴 (실제로 겪은 버그:
        # 목표는 accepted 되지만 결과가 몇십 초를 기다려도 안 옴). stamp를 기본값(0)으로
        # 비워두면 "받는 즉시 시작"으로 처리되어 정상 동작함.
        goal.trajectory.header.frame_id = "move_manipulator"
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        # 이동 시간을 거리에 비례하게 계산함. 실제로 겪은 문제: 자리가 4개(90도 간격, 최대 180도
        # 회전)로 늘었는데 이동 시간은 예전 45도 기준 고정값(4~5초)을 그대로 써서, 오히려 각속도가
        # 더 빨라져 박스를 쥔 채로 빠르게 돌다가 박스가 튕겨나감(실측: 목표 자리와 전혀 다른 곳에
        # 떨어짐). 항상 비슷한(느린) 각속도가 되도록 거리만큼 시간을 늘림.
        current = self.current_joint_position if self.joint_state_received else [0.0, 0.0, 0.0, 0.0]
        max_delta = max(abs(p - c) for p, c in zip(positions, current))
        duration_sec = max(3.0, max_delta / speed_limit)
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        goal.trajectory.points.append(point)  # type: ignore

        send_future = self.joint_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=MOVE_TIMEOUT_SEC)
        if not send_future.done():
            self.get_logger().info("팔 목표 전송이 타임아웃됐습니다.")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().info("팔 목표가 거절되었습니다.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=MOVE_TIMEOUT_SEC)
        if not result_future.done():
            self.get_logger().info("팔 이동 결과를 못 받았습니다 (타임아웃).")
            return False

        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"팔 이동 실패 (status={result.status})")
            return False

        time.sleep(SETTLE_SEC)  # 도착 직후 살짝 흔들리는 걸 가라앉힘
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)  # joint_states를 최신값으로 갱신
        return True

    def _move_and_wait(self, positions, max_retries=6, speed_limit=ANGULAR_SPEED_LIMIT_RAD_S) -> bool:
        # 실제로 겪은 문제: 액션이 SUCCEEDED를 반환해도, 무게가 많이 실리는 자세(joint2가 크게
        # 움직이는 경우 등)에서는 목표에 못 미친 채로 멈춥니다. 아마도 컨트롤러의 위치 게인이
        # 낮아서 중력을 못 이기고 정상상태 오차가 남는 것으로 보입니다 (P 제어의 전형적인 문제).
        # 컨트롤러 설정을 건드리는 대신, 여기서 "목표 - 실제 도착값"만큼을 다음 시도에 더 보내는
        # 방식으로 오차를 스스로 보정합니다 (사람이 손으로 "좀 더!"라고 여러 번 밀어주는 것과 비슷).
        target = list(positions)
        command = list(positions)
        for attempt in range(1, max_retries + 1):
            if not self._send_trajectory_once(command, speed_limit=speed_limit):
                return False

            errors = [a - b for a, b in zip(target, self.current_joint_position)]
            max_error = max(abs(e) for e in errors)
            if max_error <= POSITION_TOLERANCE:
                if attempt > 1:
                    self.get_logger().info(f"팔 이동 성공 ({attempt}번째 시도에서 오차 보정됨)")
                else:
                    self.get_logger().info("팔 이동 성공")
                return True

            self.get_logger().info(
                f"[{attempt}/{max_retries}] 목표 {[round(p,3) for p in target]} 인데 실제 "
                f"{[round(p,3) for p in self.current_joint_position]} 로 도착 "
                f"(최대 오차 {max_error:.3f}rad) -> 오차만큼 보정해서 재시도"
            )
            # 다음 시도는 "원래 목표 + 이번에 부족했던 만큼"을 새 명령으로 보냄
            command = [c + e for c, e in zip(command, errors)]

        self.get_logger().info(f"팔 이동 실패: {max_retries}번 시도해도 허용 오차 안으로 못 들어옴")
        return False

    def _gripper_and_wait(self, position: float) -> bool:
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("gripper_controller Action 서버를 찾지 못했습니다.")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(self._effective_gripper_position(position))
        goal.command.max_effort = 10.0

        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=GRIPPER_TIMEOUT_SEC)
        if not send_future.done():
            self.get_logger().info("그리퍼 목표 전송이 타임아웃됐습니다.")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().info("그리퍼 목표가 거절되었습니다.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=GRIPPER_TIMEOUT_SEC)
        if not result_future.done():
            self.get_logger().info("그리퍼 결과를 못 받았습니다 (타임아웃).")
            return False

        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"그리퍼 이동 실패 (status={result.status})")
            return False

        self.get_logger().info(f"그리퍼 이동 성공: {result.result.position}")
        time.sleep(SETTLE_SEC)
        return True

    # ------------------------------------------------------------------
    # 그리퍼(손가락) 움직이기
    # ------------------------------------------------------------------
    def move_gripper(self, position: float, max_effort=10.0, timeout_sec=5.0):
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().info("gripper_controller Action 서버를 찾지 못했습니다.")
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(self._effective_gripper_position(position))
        goal.command.max_effort = float(max_effort)
        send_goal_future = self.gripper_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.gripper_goal_callback)

    def gripper_goal_callback(self, future: Future):
        goal_handle = future.result()  # type: ignore
        get_result_future = goal_handle.get_result_async()  # type: ignore
        get_result_future.add_done_callback(self.gripper_get_result_callback)

    def gripper_get_result_callback(self, future: Future):
        result = future.result()  # type: ignore
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"그리퍼 이동 성공: {result.result.position}")
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("그리퍼 이동 중단됨")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("그리퍼 이동 취소됨")

    # ------------------------------------------------------------------
    # 팔(joint1~4) 움직이기
    # ------------------------------------------------------------------
    def send_joint_positions(self, positions, wait_for_previous=True):
        if wait_for_previous and self.arm_goal_in_progress:
            return False
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(sec=1)  # COMMAND 이동은 좀 더 여유있게 1초
        return self.move_joint(point)

    def move_joint(self, point: JointTrajectoryPoint) -> bool:
        # 반환값: 목표를 실제로 "보냈는지" 여부. False면 arm_goal_in_progress가 절대
        # True가 되지 않으므로, 이걸 확인 안 하고 _wait_until_arrived()만 부르면
        # "명령을 보내지도 못했는데 이미 도착한 것"으로 착각하는 버그가 생김 (실제로 겪음).
        if not self.joint_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("joint_controller Action 서버를 찾지 못했습니다.")
            return False
        goal = FollowJointTrajectory.Goal()
        # 주의: 여기에 self.get_clock().now()(실제 시간=wall-clock)를 채우면 안 됨.
        # Gazebo 쪽 컨트롤러는 시뮬레이션 시간(use_sim_time)을 쓰는데 이 노드는 그렇지 않아서,
        # 두 시간이 안 맞아 컨트롤러가 트리거를 영영 못 걸고 응답도 안 옴 (실제로 겪은 버그:
        # 목표는 accepted 되지만 결과가 몇십 초를 기다려도 안 옴). stamp를 기본값(0)으로
        # 비워두면 "받는 즉시 시작"으로 처리되어 정상 동작함.
        goal.trajectory.header.frame_id = "move_manipulator"
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points.append(point)  # type: ignore

        self.arm_goal_in_progress = True
        send_goal_future = self.joint_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_joint_callback)
        return True

    def goal_joint_callback(self, future: Future):
        goal_handle = future.result()  # type: ignore
        if not goal_handle.accepted:
            # 목표가 거절된 경우: get_result_async()를 불러도 콜백이 영영 안 올 수 있으므로
            # 여기서 바로 arm_goal_in_progress를 풀어줘야 대기 루프가 무한정 안 걸림.
            self.arm_goal_in_progress = False
            self.get_logger().info("팔 목표가 거절되었습니다.")
            return
        get_result_future = goal_handle.get_result_async()  # type: ignore
        get_result_future.add_done_callback(self.get_joint_result_callback)

    def get_joint_result_callback(self, future: Future):
        self.arm_goal_in_progress = False
        result = future.result()  # type: ignore
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"팔 이동 성공: {result.result.error_string}")
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("팔 이동 중단됨")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("팔 이동 취소됨")


def enter_command_mode(node: BoxSortProject, settings):
    # COMMAND 모드는 "0 3" 처럼 여러 글자를 한 번에 입력받아야 하므로,
    # 잠깐 터미널을 raw 모드에서 원래 모드(line-buffered)로 되돌려서 input()을 씀.
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(f"\r\n[COMMAND 모드] 저장된 자리: {sorted(node.slots.keys())}, 지금 점유 현황: {node.slot_occupancy}")
    print("마커id 도착자리 형식으로 입력하세요 (예: 0 3 -> 마커 id=0 박스를 3번 자리로). 그냥 Enter만 누르면 취소.\r")
    try:
        line = input("> ").strip()
    except (EOFError, KeyboardInterrupt):
        line = ""
    finally:
        tty.setraw(sys.stdin.fileno())  # TEACH 모드로 돌아가기 전에 다시 raw 모드로

    if not line:
        print("취소했습니다.\r")
        return
    parts = line.split()
    if len(parts) != 2 or not all(p.isdigit() for p in parts):
        print("'마커id 도착자리' 형식으로 숫자 2개를 입력해야 합니다. 예: 0 3\r")
        return
    marker_id, to_slot = int(parts[0]), int(parts[1])
    try:
        node.run_command(marker_id, to_slot)
    except RuntimeError:
        pass  # 실패 이유는 run_command 안에서 이미 로그로 출력함


def main(args=None):
    rclpy.init(args=args)
    node = BoxSortProject()

    settings = termios.tcgetattr(sys.stdin)
    print(
        "아루코 박스 자리 이동 프로젝트\n"
        "  q/a/w/s/e/d/r/f : 관절 수동 조작   z/x : 그리퍼 열기/닫기\n"
        "  n/b : 가르칠 자리 번호 다음/이전 (지금: 1)\n"
        "  g   : 지금 팔 자세를 현재 자리 번호로 저장\n"
        "  c   : 명령 모드 ('마커id 도착자리' 입력, 예: 0 3)\n"
        "  Ctrl+C : 종료\n"
    )

    tty.setraw(sys.stdin.fileno())
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key(timeout=0.0)
            if key == "\x03":  # Ctrl+C
                break
            if key == "c":
                enter_command_mode(node, settings)
            elif key:
                node.handle_key(key)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()


if __name__ == "__main__":
    main()
