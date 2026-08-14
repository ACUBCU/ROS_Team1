# find_redball.py 에 힌트 주석으로만 남아있던 "빨간 공을 눈으로 쫓아가는 로봇 팔"을
# 실제로 동작하는 코드로 완성한 프로젝트 파일입니다.
#
# [비유로 먼저 이해하기]
#   - 카메라           = 로봇의 "눈"
#   - image_callback   = 눈으로 본 것을 처리하는 "뇌"
#   - joint1~4         = 사람의 "목 + 어깨 + 팔꿈치" 같은 관절들
#   - 그리퍼(gripper)  = 사람의 "손가락"
#
# [모드 2가지]
#   - MANUAL (수동) : 키보드로 사람이 직접 관절/그리퍼를 움직임 (기본 시작 모드, 안전을 위해)
#   - AUTO   (자동) : 카메라가 빨간 공을 보고 알아서 따라감 (find_redball.py의 원래 목표)
#   'm' 키를 누르면 두 모드가 서로 전환됩니다. 동시에 켜두면 명령이 충돌하므로
#   반드시 한 번에 하나의 모드만 실제로 팔을 움직이도록 만들었습니다.
#
# [카메라 이미지 한 장이 처리되는 순서] (image_callback 안에서 매번 반복됨, 모드 상관없이 공 인식/로깅은 항상 함)
#   1) 이미지를 받는다
#   2) 이미지에서 "빨간색"만 골라내서(마스크) 공의 위치(중심 x, y)와 크기(area)를 찾는다
#   3) 화면 중심과 공의 위치 차이(오차)를 계산한다
#   4) 공의 면적(area)으로 "얼마나 가까운지"를 로그로 찍는다 (실제 미터 단위는 아님)
#   5) AUTO 모드일 때만: 오차만큼 joint1(좌우), joint2~4(상하)를 아주 조금씩 움직이고,
#      공이 충분히 커지면(=충분히 가까워지면) 그리퍼를 닫아 공을 잡는다
#
# [주의 / 한계]
#   - area 기반 거리 추정은 카메라 보정(calibration)을 하지 않았기 때문에
#     "실제 몇 cm" 값이 아니라 "클수록 가깝다" 정도의 대략적인 지표입니다.
#   - joint2~4에 같은 값을 그대로 더하는 것은 실제 역기구학(inverse kinematics)이
#     아니라, 초심자용으로 단순화한 방식입니다. 실제 로봇에서 방향이 이상하면
#     아래 Y_GAIN 부호(+/-)를 바꿔가며 테스트해보세요.
#   - 로봇 팔의 기본(홈) 자세에서는 그리퍼 카메라가 아주 가까운 바닥을 못 봅니다
#     (카메라가 거의 수평으로 앞을 보기 때문). AUTO 모드로 공을 따라가게 하려면
#     먼저 MANUAL 모드로 팔을 움직여서 카메라가 공을 보게 만든 다음 'm'으로 전환하세요.
#
# 실행 방법:
#   1) colcon build --symlink-install --packages-select redball_sort_project
#   2) source install/setup.bash
#   3) ros2 run redball_sort_project redball_tracking_project
#
# 키 배치 (MANUAL 모드일 때):
#   q/a : joint1 +/-      w/s : joint2 +/-
#   e/d : joint3 +/-      r/f : joint4 +/-
#   z   : 그리퍼 열기     x   : 그리퍼 닫기
#   m   : MANUAL <-> AUTO 모드 전환
#   Ctrl+C : 종료

import select
import sys
import termios
import tty

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import (
    FollowJointTrajectory,
    FollowJointTrajectory_GetResult_Response,
    GripperCommand,
    GripperCommand_GetResult_Response,
)
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# ---- 튜닝 상수 (실습하면서 값을 바꿔가며 실제 로봇 반응을 확인해보세요) ----
X_GAIN = 0.0008          # (AUTO) 화면 x 오차(픽셀) 1당 joint1을 얼마나 돌릴지 (라디안/픽셀)
Y_GAIN = 0.0008          # (AUTO) 화면 y 오차(픽셀) 1당 joint2~4를 얼마나 움직일지 (라디안/픽셀)
MAX_STEP = 0.02          # (AUTO) 한 프레임에 관절 하나가 움직일 수 있는 최대량 (라디안)
                         # -> 카메라가 너무 자주(예: 30장/초) 이미지를 보내기 때문에,
                         #    오차를 그대로 다 반영하면 팔이 홱홱 튀므로 한 번에 움직이는
                         #    양을 작게 제한합니다. (사람이 목을 천천히 돌리는 것과 같은 이치)
MIN_BALL_AREA = 20       # 이 값보다 작은 빨간 덩어리는 "공이 아니라 노이즈"로 무시
GRAB_AREA = 8000         # (AUTO) 공의 면적이 이 값보다 커지면 "충분히 가까움" -> 그리퍼로 잡기

STEP = 0.05              # (MANUAL) 키 한 번 누를 때 관절이 움직이는 양 (keyboard_manipulator.py 와 동일)
GRIPPER_OPEN = 0.019      # keyboard_manipulator.py 와 동일한 값 사용
GRIPPER_CLOSE = -0.01
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]  # 팔 관절 4개 이름, 항상 이 순서로 다룸

# key -> (관절 인덱스, 증감량). keyboard_manipulator.py 와 동일한 배치.
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
    # timeout초 동안 키 입력을 기다리다가, 안 눌리면 빈 문자열을 반환 (넌블로킹).
    # -> 이렇게 해야 키를 안 누르고 있는 동안에도 카메라 콜백(AUTO 추적)이 계속 돌아감.
    # 주의: raw 모드는 main()에서 루프 시작 전 딱 한 번만 켜야 함. 여기서 매번 켰다 끄면
    # 대부분의 시간은 '일반(line-buffered) 모드'로 있게 되어 키 입력이 줄바꿈 전까지
    # 터미널 입력 큐에 갇혀버리고 이 함수에서 전혀 못 읽게 된다 (실제로 겪은 버그).
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if rlist else ""


class RedBallTrackingProject(Node):
    def __init__(self):
        super().__init__("redball_tracking_project")  # 노드 이름

        # 팔(joint1~4)을 움직이는 액션 클라이언트
        self.joint_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        # 그리퍼(손가락)를 움직이는 액션 클라이언트
        self.gripper_client = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")

        # 로봇의 "현재 관절 각도"를 계속 받아두는 구독
        self.create_subscription(JointState, "joint_states", self.joint_callback, 10)
        self.current_joint_position = [0.0, 0.0, 0.0, 0.0]
        self.joint_state_received = False  # 현재 위치를 한 번이라도 받았는지

        # 카메라 이미지 구독 (여기서 빨간 공을 찾음)
        self.bridge = CvBridge()
        self.create_subscription(Image, "/gripper_camera/image_raw", self.image_callback, 10)

        # 목표(goal)를 이미 보내놓고 응답을 기다리는 중이면 True.
        # -> AUTO 모드는 카메라가 30장/초로 콜백을 부르기 때문에, 응답을 기다리는 동안
        #    또 새 목표를 보내면 액션 서버가 밀립니다. 그래서 AUTO만 이 값을 확인합니다.
        #    (MANUAL은 사람이 누른 만큼만 명령이 나가서 이 정도로 자주 밀릴 일이 없음)
        self.arm_goal_in_progress = False

        # 공을 이미 한 번 잡았으면 True (AUTO 모드에서 계속 반복해서 그리퍼를 닫지 않기 위함)
        self.ball_grabbed = False

        # 시작은 항상 MANUAL. 카메라가 아직 공을 못 보는 자세일 수 있으므로,
        # 안전하게 사람이 먼저 팔을 확인하고 'm'으로 AUTO를 켜도록 함.
        self.mode = "MANUAL"

    # ------------------------------------------------------------------
    # 카메라 콜백: 이미지 한 장이 들어올 때마다 호출됨
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image_h, image_w = img.shape[:2]

        # 1) 빨간색만 골라내기 (HSV 색공간이 색으로 필터링하기 더 쉬움)
        #    빨간색은 색상(Hue) 값이 0 근처와 180 근처, 양 끝에 걸쳐 있어서
        #    두 구간(0~10, 170~180)을 합쳐야 빨간색을 놓치지 않고 잡습니다.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower1 = np.array([0, 40, 40], dtype=np.uint8)
        upper1 = np.array([10, 255, 255], dtype=np.uint8)
        lower2 = np.array([170, 40, 40], dtype=np.uint8)
        upper2 = np.array([180, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)  # type: ignore

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.imshow("img", img)
        cv2.imshow("mask", mask)
        cv2.waitKey(10)

        if not contours:
            return  # 화면에 빨간 공이 안 보이면 아무것도 안 함

        # 2) 가장 큰 빨간 덩어리 = 공이라고 판단
        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < MIN_BALL_AREA:
            return  # 너무 작으면 노이즈로 취급하고 무시

        x, y, w, h = cv2.boundingRect(contour)
        center_x = x + w // 2
        center_y = y + h // 2
        self.get_logger().info(f"공 중심 좌표: x={center_x}, y={center_y}")

        # 3) 면적(area)은 "공이 얼마나 가까이 있는지"의 대략적인 지표
        #    (사람이 손을 뻗어보고 거리를 어림짐작하는 것과 비슷합니다)
        self.get_logger().info(f"공 면적(거리 지표, 클수록 가까움): {area:.1f}")

        # 여기부터는 AUTO 모드일 때만 팔을 실제로 움직임 (MANUAL이면 인식/로깅만 하고 끝)
        if self.mode != "AUTO":
            return

        # 4) 아직 현재 관절 위치를 못 받았으면, 어디서부터 움직여야 할지 모르므로 대기
        if not self.joint_state_received:
            self.get_logger().info("아직 joint_states를 못 받아서 대기 중...")
            return

        # 5) 공이 충분히 가까우면 그리퍼를 닫아서 잡기 (한 번만)
        if area > GRAB_AREA and not self.ball_grabbed:
            self.get_logger().info("공이 충분히 가까움 -> 그리퍼로 잡기 시도")
            self.move_gripper(GRIPPER_CLOSE)
            self.ball_grabbed = True
            return  # 잡는 동안에는 팔을 더 움직이지 않음

        # 6) 이미 목표를 보내고 응답을 기다리는 중이면, 이번 프레임은 건너뜀
        if self.arm_goal_in_progress:
            return

        # 7) 화면 중심과 공 중심의 오차를 계산 -> 관절을 얼마나 움직일지 결정
        error_x = center_x - image_w // 2   # 양수면 공이 화면 오른쪽에 있음
        error_y = center_y - image_h // 2   # 양수면 공이 화면 아래쪽에 있음

        delta_joint1 = self._clamp(-error_x * X_GAIN, -MAX_STEP, MAX_STEP)
        delta_y = self._clamp(-error_y * Y_GAIN, -MAX_STEP, MAX_STEP)

        new_position = list(self.current_joint_position)
        new_position[0] += delta_joint1  # joint1: 좌우 회전으로 x 오차를 줄임
        # joint2~4: 셋 다 같은 값만큼 움직여서 y 오차를 줄임 (초심자용 단순화 버전)
        new_position[1] += delta_y
        new_position[2] += delta_y
        new_position[3] += delta_y

        self.send_joint_positions(new_position)

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

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
    # 키보드 입력 처리 (MANUAL 모드 조작 + 모드 전환)
    # ------------------------------------------------------------------
    def handle_key(self, key: str):
        if key == "m":
            self.mode = "AUTO" if self.mode == "MANUAL" else "MANUAL"
            self.get_logger().info(f"모드 전환 -> {self.mode}")
            return

        if key == "z":
            self.move_gripper(GRIPPER_OPEN)
            return
        if key == "x":
            self.move_gripper(GRIPPER_CLOSE)
            return

        if self.mode != "MANUAL":
            return  # AUTO 모드에서는 관절 키 입력 무시 (충돌 방지)

        if key not in MANUAL_KEY_BINDINGS:
            return
        if not self.joint_state_received:
            self.get_logger().info("아직 joint_states를 못 받아서 대기 중...")
            return

        joint_index, delta = MANUAL_KEY_BINDINGS[key]
        new_position = list(self.current_joint_position)
        new_position[joint_index] += delta
        # 사람이 키를 누른 만큼만 움직이므로, AUTO처럼 arm_goal_in_progress를 확인하지 않고 바로 보냄.
        self.send_joint_positions(new_position, wait_for_previous=False)
        self.get_logger().info(f"[MANUAL] joint{joint_index + 1} -> {new_position[joint_index]:.3f}")

    # ------------------------------------------------------------------
    # 그리퍼(손가락) 움직이기
    # ------------------------------------------------------------------
    def move_gripper(self, position: float, max_effort=10.0, timeout_sec=5.0):
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().info("gripper_controller Action 서버를 찾지 못했습니다.")
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        send_goal_future = self.gripper_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.gripper_goal_callback)

    def gripper_goal_callback(self, future: Future):
        goal_handle = future.result()  # type: ignore
        get_result_future = goal_handle.get_result_async()  # type: ignore
        get_result_future.add_done_callback(self.gripper_get_result_callback)

    def gripper_get_result_callback(self, future: Future):
        result: GripperCommand_GetResult_Response = future.result()  # type: ignore
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
            return
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=300_000_000)
        self.move_joint(point)

    def move_joint(self, point: JointTrajectoryPoint):
        if not self.joint_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("joint_controller Action 서버를 찾지 못했습니다.")
            return
        goal = FollowJointTrajectory.Goal()
        # 주의: 여기에 self.get_clock().now()(실제 시간=wall-clock)를 채우면 안 됨.
        # Gazebo 쪽 컨트롤러는 시뮬레이션 시간(use_sim_time)을 쓰는데 이 노드는 그렇지 않아서,
        # 두 시간이 안 맞아 컨트롤러가 트리거를 영영 못 걸고 응답도 안 옴 (실제로 겪은 버그:
        # 목표는 accepted 되지만 결과 콜백이 몇십 초를 기다려도 안 옴). stamp를 기본값(0)으로
        # 비워두면 "받는 즉시 시작"으로 처리되어 정상 동작함.
        goal.trajectory.header.frame_id = "move_manipulator"
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points.append(point)  # type: ignore

        self.arm_goal_in_progress = True
        send_goal_future = self.joint_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_joint_callback)

    def goal_joint_callback(self, future: Future):
        goal_handle = future.result()  # type: ignore
        if not goal_handle.accepted:
            # 목표가 거절된 경우 여기서 바로 풀어주지 않으면 arm_goal_in_progress가 계속 True로
            # 남아서, AUTO 모드가 그 뒤로 영영 새 목표를 못 보내고 조용히 멈춰버림.
            self.arm_goal_in_progress = False
            self.get_logger().info("팔 목표가 거절되었습니다.")
            return
        get_result_future = goal_handle.get_result_async()  # type: ignore
        get_result_future.add_done_callback(self.get_joint_result_callback)

    def get_joint_result_callback(self, future: Future):
        self.arm_goal_in_progress = False  # 응답이 왔으니 다음 목표를 보내도 됨
        result: FollowJointTrajectory_GetResult_Response = future.result()  # type: ignore
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"팔 이동 성공: {result.result.error_string}")
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("팔 이동 중단됨")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("팔 이동 취소됨")


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = RedBallTrackingProject()

    settings = termios.tcgetattr(sys.stdin)
    print(
        "빨간 공 추적 프로젝트 (시작 모드: MANUAL)\n"
        "  q/a : joint1 +/-   w/s : joint2 +/-   e/d : joint3 +/-   r/f : joint4 +/-\n"
        "  z   : 그리퍼 열기   x   : 그리퍼 닫기\n"
        "  m   : MANUAL <-> AUTO 모드 전환\n"
        "  Ctrl+C : 종료\n"
    )

    tty.setraw(sys.stdin.fileno())  # 루프 시작 전 딱 한 번만 raw 모드로 전환
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key(timeout=0.0)
            if key == "\x03":  # Ctrl+C
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
