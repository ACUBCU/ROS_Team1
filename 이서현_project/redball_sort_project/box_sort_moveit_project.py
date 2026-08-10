# MoveIt(MoveItPy)을 이용한 아루코 박스 집기/옮기기 프로젝트입니다.
#
# [box_sort_project.py 와 무엇이 다른가]
#   box_sort_project.py 는 "관절 각도(joint1~4)"를 직접 정해서 팔을 보내는 방식이었습니다.
#   이 방식의 한계를 실제로 겪었습니다:
#     - 목표 관절각을 사람이 직접 찾아야 함 (TF로 실측하며 탐색)
#     - 액션이 "성공"해도 실제로는 목표에 못 미친 채 멈추는 경우가 있어서 보정 로직이 필요했음
#     - 실제 물체가 그 자리에 있으면, 팔이 그리로 다가가는 "경로 중간"에 충돌해서 더 불안정해짐
#
#   이 파일은 대신 **"어디(x, y, z)로 가고 싶다"만 말하면 MoveIt이 역기구학(inverse kinematics)으로
#   관절 각도를 스스로 계산**하고, **충돌 없는 경로까지 계획**해줍니다. 우리가 관절 각도를 몰라도
#   되고, 경로 중간에 뭔가 있으면 MoveIt이 알아서 피해가려고 시도합니다 (planning scene에 그
#   물체를 등록해뒀을 때).
#
# [비유]
#   box_sort_project.py = 목적지까지 "몇 미터 가서 몇 도 꺾고..."를 사람이 하나하나 알려주는 것
#   box_sort_moveit_project.py = 네비게이션 앱에 "여기로 가줘"라고 주소만 찍으면 알아서 길을 찾는 것
#
# [실행 순서]
#   1) 터미널 1: Gazebo (오픈매니퓰레이터 + 그리퍼카메라)
#      ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
#   2) 터미널 2: MoveIt (move_group) - 반드시 Gazebo가 켜진 뒤에, 시뮬레이션 시간을 쓰도록 use_sim:=true
#      ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py use_sim:=true
#   3) 터미널 3: 이 노드 실행
#      ros2 run redball_sort_project box_sort_moveit_project
#
# [주의] MoveItPy는 rclpy.init() 이후, move_group이 완전히 켜진 다음에 생성해야 합니다.
#        move_group이 없으면 MoveItPy(...) 생성에서 계속 멈춰있습니다.

import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from rclpy.action import ActionClient
from rclpy.logging import get_logger
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

# 실제 aruco_cube 모델(models/aruco_cube/model.sdf, models/red_ball과 같은 폴더)의 크기와 맞춤.
# box_sort_project.py 와 마찬가지로 "실제 모델 치수"를 그대로 쓰는 게 중요합니다 - 대충 다른 크기를
# 쓰면 충돌 계산이 실제 모양과 안 맞아서 집을 때 헛돌거나 반대로 멀쩡한 경로를 막게 됩니다.
BOX_SIZE = [0.05, 0.05, 0.05]

# 그리퍼가 물건을 잡을 때 쓰는 자세(orientation). 실제로 동작 확인된 mid 자세(joints=[0,0.85,0.05,-0.85])
# 에서 end_effector_link의 TF를 실측해보니 사실상 무회전(항등 quaternion)에 가까웠습니다.
# -> 모든 x,y,z 목표에 이 방향을 그대로 재사용합니다. (필요하면 나중에 방향도 바꿔볼 수 있음)
GRASP_ORIENTATION = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w

# end_effector_link 은 실제 손가락이 오므라드는 지점(pinch point)이 아니라 그보다 약
# 4.4cm 더 먼 곳에 있습니다 (URDF: end_effector_joint origin 0.126m vs gripper_*_joint
# origin 0.0817m, box_sort_project.py 개발 중 TF 실측으로 발견). 좌표 목표를 세울 때 이 오프셋을
# 보정하지 않으면, "박스 중심"으로 보낸 좌표에 실제로는 손가락이 아니라 그 옆 허공이 가게 됩니다.
FINGER_TO_EE_OFFSET_M = 0.08

GRIPPER_OPEN_NAME = "open"
GRIPPER_CLOSE_NAME = "close"
ARM_CONTROLLER = "arm_controller"
GRIPPER_CONTROLLER = "gripper_controller"
PLANNING_LINK = "end_effector_link"

# ------------------------------------------------------------------
# 키보드 TEACH 모드 (box_sort_project.py의 TEACH 모드와 같은 패턴). 나중에 장애물이
# 복잡해질 걸 대비해서, MoveIt 충돌 검사와 함께 "지금 이 자세, 충돌 없이 되는지"를
# 사람이 직접 조종하면서 눈+코드로 같이 확인하고 저장할 수 있게 만든 도구입니다.
# ------------------------------------------------------------------
ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
STEP = 0.05  # 키 한 번 누를 때 관절이 움직이는 양(라디안). box_sort_project.py와 동일.
TEACH_KEY_BINDINGS = {
    "q": (0, +STEP), "a": (0, -STEP),
    "w": (1, +STEP), "s": (1, -STEP),
    "e": (2, +STEP), "d": (2, -STEP),
    "r": (3, +STEP), "f": (3, -STEP),
}


def get_key(timeout=0.0):
    # timeout초 동안 키 입력을 기다리다가, 안 눌리면 빈 문자열을 반환 (넌블로킹).
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if rlist else ""


def waypoints_file_path() -> str:
    share_dir = get_package_share_directory("redball_sort_project")
    return os.path.join(share_dir, "config", "moveit_waypoints.yaml")


class BoxSortMoveItNode(Node):
    def __init__(self):
        super().__init__("box_sort_moveit_project")
        self.logger = get_logger("box_sort_moveit_project")

        # move_group이 이미 떠 있어야 이 생성자가 끝남 (안 떠 있으면 여기서 멈춰있음)
        self.moveit = MoveItPy(node_name="box_sort_moveit_py")
        self.arm = self.moveit.get_planning_component("arm")
        self.gripper = self.moveit.get_planning_component("gripper")
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()

        self.object_id = "sorted_box"

        # TEACH 모드용: MoveIt 계획을 거치지 않고 바로 컨트롤러에 관절각을 보내는 액션
        # 클라이언트 (box_sort_project.py와 동일한 방식) - 키 누를 때마다 매번 OMPL로
        # 계획하면 느리고 뚝뚝 끊기므로, 조금씩 움직이는 용도로는 이게 더 반응이 빠릅니다.
        self.joint_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")
        # 스크립트가 막 켜진 직후에는 이 액션 서버 연결(ROS2 디스커버리)이 아직 안 끝나
        # 있을 수 있음 - 이 상태에서 MoveItPy.execute()를 부르면 "Action client not
        # connected"로 조용히(에러 로그만 남기고) 실패함. 첫 이동 전에 여기서 미리 기다림.
        if not self.joint_client.wait_for_server(timeout_sec=5.0):
            self.logger.info("arm_controller 액션 서버 연결 대기 시간 초과 (5초)")
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.logger.info("gripper_controller 액션 서버 연결 대기 시간 초과 (5초)")
        self.waypoints: list = self.load_waypoints()

    # ------------------------------------------------------------------
    # 이동 헬퍼: "이름(named state)" / "관절값(dict)" / "좌표(x,y,z)" 셋 다 지원.
    # moveit_class.py 에 있던 "my_pose는 어떻게 넣지?" 라는 질문에 대한 답이
    # 바로 이 pose 분기입니다 - PoseStamped + pose_link를 주면 MoveIt이 역기구학을 풀어줍니다.
    # ------------------------------------------------------------------
    def plan_and_execute(self, component, controller_name: str, *, configuration_name=None,
                          joint_positions=None, pose_xyz=None) -> bool:
        component.set_start_state_to_current_state()

        if configuration_name is not None:
            component.set_goal_state(configuration_name=configuration_name)
        elif joint_positions is not None:
            robot_model = self.moveit.get_robot_model()
            robot_state = RobotState(robot_model)
            robot_state.joint_positions = joint_positions
            joint_model_group = robot_model.get_joint_model_group(component.planning_group_name)
            # 기본 tolerance(0.01rad)로는 목표 근처가 좁은 경우("Insufficient states in
            # sampleable goal region") 계획 자체가 안 되는 경우가 있어서 넉넉하게 늘림.
            joint_constraint = construct_joint_constraint(
                robot_state=robot_state, joint_model_group=joint_model_group, tolerance=0.05
            )
            component.set_goal_state(motion_plan_constraints=[joint_constraint])
        elif pose_xyz is not None:
            x, y, z = pose_xyz
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "world"
            pose_goal.pose.position.x = x
            pose_goal.pose.position.y = y
            pose_goal.pose.position.z = z
            ox, oy, oz, ow = GRASP_ORIENTATION
            pose_goal.pose.orientation.x = ox
            pose_goal.pose.orientation.y = oy
            pose_goal.pose.orientation.z = oz
            pose_goal.pose.orientation.w = ow
            component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=PLANNING_LINK)
        else:
            raise ValueError("configuration_name / joint_positions / pose_xyz 중 하나는 있어야 함")

        plan_result = component.plan()
        if not plan_result:
            self.logger.info("계획(planning) 실패 - 이 목표는 도달할 수 없거나 충돌이 있음")
            return False

        # MoveItPy.execute()는 내부적으로 자기만의 컨트롤러 액션 클라이언트를 새로 만드는데,
        # 이게 "Action client not connected"로 계속(재시도해도) 실패하는 문제를 겪었음
        # (원인 불명 - 아마 이 환경(WSL)의 DDS 디스커버리 문제로 추정). 대신 이미 연결이
        # 확인된 self.joint_client/self.gripper_client로 계획된 궤적을 직접 실행함 -
        # MoveIt은 "계획(충돌 검사 포함)"까지만 담당하고, 실제 전송은 저희가 함.
        return self._execute_trajectory_directly(plan_result.trajectory, controller_name)

    def _send_goal_and_wait(self, client: ActionClient, goal, timeout_sec: float = 30.0):
        # ActionClient.send_goal()(블로킹 버전)은 우리 노드가 어딘가에서 spin되고 있어야
        # 내부적으로 결과를 받는데, run_grab_demo 경로는 별도 spin 루프가 없어서 그냥
        # 영원히 멈춰있었음 (실제로 겪음 - 60초 타임아웃에도 안 끝남). spin_until_future_
        # complete로 "이 goal 하나 끝날 때까지만" 직접 spin해주는 방식으로 바꿈.
        send_goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=timeout_sec)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        return result_future.result()

    def _execute_trajectory_directly(self, robot_trajectory, controller_name: str) -> bool:
        joint_traj = robot_trajectory.get_robot_trajectory_msg().joint_trajectory

        if controller_name == ARM_CONTROLLER:
            if not self.joint_client.wait_for_server(timeout_sec=5.0):
                self.logger.info("arm_controller 연결 안 됨")
                return False
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = joint_traj
            response = self._send_goal_and_wait(self.joint_client, goal)
            ok = (
                response is not None
                and response.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
            self.logger.info(f"arm_controller 직접 실행 결과: {ok}")
            return ok

        if controller_name == GRIPPER_CONTROLLER:
            if not self.gripper_client.wait_for_server(timeout_sec=5.0):
                self.logger.info("gripper_controller 연결 안 됨")
                return False
            # 그리퍼는 궤적이 아니라 "최종 목표 위치" 하나만 받으므로, 계획된 궤적의
            # 마지막 지점에서 gripper_left_joint 값만 뽑아서 씀.
            index = joint_traj.joint_names.index("gripper_left_joint")
            target_position = joint_traj.points[-1].positions[index]
            goal = GripperCommand.Goal()
            goal.command.position = target_position
            goal.command.max_effort = 10.0
            response = self._send_goal_and_wait(self.gripper_client, goal)
            ok = response is not None
            self.logger.info(f"gripper_controller 직접 실행 결과: {ok}")
            return ok

        raise ValueError(f"알 수 없는 controller_name: {controller_name}")

    def close_gripper_on_object(self, target_position: float, max_effort: float = 10.0) -> bool:
        # 그리퍼로 물체를 실제로 쥐려면 손가락이 물체에 "닿아야" 하는데, MoveIt의 계획
        # 파이프라인은 시작 자세가 이미 충돌(접촉) 중이면 계획 자체를 거부함 (실제로 겪음:
        # "CheckStartStateCollision ... 1 contact(s) detected: gripper_right_link -
        # sorted_box"). 닫는 동작은 장애물을 피해갈 경로가 필요한 게 아니라 단순 직선
        # 동작이므로, MoveIt 계획을 아예 거치지 않고 컨트롤러에 바로 명령을 보냄.
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.logger.info("gripper_controller 연결 안 됨")
            return False
        goal = GripperCommand.Goal()
        goal.command.position = target_position
        goal.command.max_effort = max_effort
        response = self._send_goal_and_wait(self.gripper_client, goal)
        ok = response is not None
        self.logger.info(f"그리퍼 직접 닫기(목표={target_position}) 결과: {ok}")
        return ok

    def move_arm_to(self, **kwargs) -> bool:
        return self.plan_and_execute(self.arm, ARM_CONTROLLER, **kwargs)

    @staticmethod
    def _to_end_effector_target(xyz):
        # 베이스(원점)에서 봤을 때 바깥쪽(반지름 방향)으로 FINGER_TO_EE_OFFSET_M만큼 더 밀어서,
        # 실제 손가락 집는 지점이 xyz에 오도록 end_effector_link의 목표 좌표를 계산.
        x, y, z = xyz
        r = (x**2 + y**2) ** 0.5
        if r < 1e-6:
            return xyz
        scale = (r + FINGER_TO_EE_OFFSET_M) / r
        return (x * scale, y * scale, z)

    def move_gripper(self, name: str) -> bool:
        return self.plan_and_execute(self.gripper, GRIPPER_CONTROLLER, configuration_name=name)

    def grip_object(self) -> bool:
        # "close"(-0.01)는 "사이에 아무것도 없을 때 완전히 닫힌 값"이라, 폭 5cm짜리 박스가
        # 손가락 사이에 있으면 이 값까지 닫으려는 계획 자체가 충돌로 거부됩니다 (실제로 겪음:
        # "Found a contact between 'sorted_box' and 'gripper_left_link'"). 그래서 완전히 닫힌
        # 값 대신, 박스와 충돌 없이 계획이 성공하는 가장 좁은 값을 순서대로 시도합니다.
        for value in (0.012, 0.009, 0.006, 0.003, 0.0):
            self.logger.info(f"그리퍼를 {value}까지 닫아보는 중...")
            if self.plan_and_execute(
                self.gripper, GRIPPER_CONTROLLER,
                joint_positions={"gripper_left_joint": value, "gripper_right_joint": value},
            ):
                self.logger.info(f"그리퍼 닫기 성공 (값={value})")
                return True
        self.logger.info("어떤 값으로도 충돌 없이 못 닫았습니다 - 박스가 그리퍼 최대 폭보다 큰 것일 수 있습니다.")
        return False

    # ------------------------------------------------------------------
    # 충돌 물체(박스) 관리: world에 추가/제거, 그리퍼에 부착/분리.
    # moveit_attached.py 의 패턴을 그대로 재사용하되, 크기만 실제 aruco_cube에 맞춤.
    # ------------------------------------------------------------------
    def add_box_to_world(self, x: float, y: float, z: float):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = self.object_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = BOX_SIZE

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives.append(box)  # type: ignore
        obj.primitive_poses.append(pose)  # type: ignore
        obj.operation = CollisionObject.ADD

        ok = self.planning_scene_monitor.process_collision_object(obj)
        self.logger.info(f"world에 박스 추가({x},{y},{z}): {ok}")
        return ok

    def remove_box_from_world(self):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = self.object_id
        obj.operation = CollisionObject.REMOVE
        return self.planning_scene_monitor.process_collision_object(obj)

    def attach_box(self):
        self.remove_box_from_world()
        attached = AttachedCollisionObject()
        attached.link_name = PLANNING_LINK
        attached.object.id = self.object_id
        attached.object.header.frame_id = PLANNING_LINK
        attached.object.header.stamp = self.get_clock().now().to_msg()

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = BOX_SIZE
        pose = Pose()
        pose.orientation.w = 1.0  # 그리퍼(end_effector_link) 기준 원점에 붙임

        attached.object.primitives.append(box)  # type: ignore
        attached.object.primitive_poses.append(pose)  # type: ignore
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = [PLANNING_LINK, "gripper_left_link", "gripper_right_link"]

        with self.planning_scene_monitor.read_write() as scene:
            ok = scene.process_attached_collision_object(attached)
            scene.current_state.update()
        self.logger.info(f"그리퍼에 박스 부착: {ok}")
        return ok

    def detach_box(self, place_xyz):
        attached = AttachedCollisionObject()
        attached.link_name = PLANNING_LINK
        attached.object.id = self.object_id
        attached.object.header.frame_id = PLANNING_LINK
        attached.object.header.stamp = self.get_clock().now().to_msg()
        attached.object.operation = CollisionObject.REMOVE

        with self.planning_scene_monitor.read_write() as scene:
            ok = scene.process_attached_collision_object(attached)
            scene.current_state.update()
        self.logger.info(f"그리퍼에서 박스 분리: {ok}")
        self.add_box_to_world(*place_xyz)
        return ok

    # ------------------------------------------------------------------
    # 실제 집기/옮기기 시퀀스. box_sort_project.py의 run_command와 같은 4단계지만,
    # 관절각이 아니라 "좌표"로 지정하고 MoveIt이 경로/역기구학을 다 계산합니다.
    # ------------------------------------------------------------------
    def pick_and_place(self, pick_xyz, place_xyz) -> bool:
        px, py, pz = pick_xyz
        self.add_box_to_world(px, py, pz)

        # end_effector_link은 실제 손가락 집는 지점보다 베이스에서 더 먼 곳에 있음(실측 4.4cm,
        # box_sort_project.py 개발 중 발견). 손가락이 박스 중심에 오도록, 목표는 그만큼 더
        # 바깥쪽으로 밀어서 잡음.
        pick_target = self._to_end_effector_target(pick_xyz)
        self.logger.info(f"1) 집을 위치로 이동: {pick_xyz} (end_effector 목표: {pick_target})")
        if not self.move_arm_to(pose_xyz=pick_target):
            return False

        self.logger.info("2) 그리퍼 열기")
        if not self.move_gripper(GRIPPER_OPEN_NAME):
            return False

        self.logger.info("3) 그리퍼 닫기 (집기)")
        if not self.grip_object():
            return False

        self.attach_box()

        place_target = self._to_end_effector_target(place_xyz)
        self.logger.info(f"4) 놓을 위치로 이동: {place_xyz} (end_effector 목표: {place_target})")
        if not self.move_arm_to(pose_xyz=place_target):
            return False

        self.logger.info("5) 그리퍼 열기 (놓기)")
        if not self.move_gripper(GRIPPER_OPEN_NAME):
            return False

        self.detach_box(place_xyz)
        self.logger.info("이동 완료")
        return True

    def shutdown(self):
        self.gripper = None
        self.arm = None
        self.moveit.shutdown()

    # ------------------------------------------------------------------
    # 키보드 TEACH 모드: 관절을 직접 조종하고, 원하는 자세에서 스페이스바로
    # "충돌 없는지 자동 확인 + 저장"을 한 번에 처리합니다.
    # ------------------------------------------------------------------
    def load_waypoints(self) -> list:
        try:
            with open(waypoints_file_path(), encoding="utf-8") as f:
                return yaml.safe_load(f) or []
        except FileNotFoundError:
            return []

    def save_waypoints(self):
        path = waypoints_file_path()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.waypoints, f, allow_unicode=True, sort_keys=False)

    def get_current_arm_joints(self) -> list:
        # planning_scene_monitor가 /joint_states를 계속 구독하며 current_state를
        # 최신으로 유지하고 있으므로, 별도 구독 없이 여기서 바로 읽으면 됩니다.
        with self.planning_scene_monitor.read_only() as scene:
            positions = scene.current_state.joint_positions
            return [float(positions[name]) for name in ARM_JOINT_NAMES]

    def jog_joint(self, joint_index: int, delta: float):
        # MoveIt 계획(OMPL)을 거치지 않고 컨트롤러에 직접 보냄 - 키 입력마다 계획하면
        # 느려서 조종감이 끊기므로, box_sort_project.py와 같은 방식으로 빠르게 보냄.
        # (주의: 이 방식은 충돌 검사를 안 하므로, 눈으로/스페이스바 확인으로 판단해야 함)
        current = self.get_current_arm_joints()
        current[joint_index] += delta
        if not self.joint_client.wait_for_server(timeout_sec=1.0):
            self.logger.info("arm_controller 액션 서버를 찾지 못했습니다.")
            return
        point = JointTrajectoryPoint()
        point.positions = current
        point.time_from_start = Duration(sec=1)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINT_NAMES
        goal.trajectory.points.append(point)  # type: ignore
        self.joint_client.send_goal_async(goal)
        self.logger.info(f"[TEACH] joint{joint_index + 1} -> {current[joint_index]:.3f}")

    def check_and_save_waypoint(self):
        joints = self.get_current_arm_joints()
        with self.planning_scene_monitor.read_only() as scene:
            valid = scene.is_state_valid(scene.current_state, "arm")
        degrees = [round(math.degrees(v), 1) for v in joints]
        status = "충돌 없음(OK)" if valid else "충돌 있음(NG)"
        label = f"waypoint_{len(self.waypoints) + 1}"
        self.logger.info(f"[저장] {label} {status} joints(deg)={degrees}")
        self.waypoints.append({
            "label": label,
            "joints_deg": degrees,
            "joints_rad": [round(v, 4) for v in joints],
            "collision_free": bool(valid),
        })
        self.save_waypoints()

    def handle_teach_key(self, key: str):
        if key == " ":
            self.check_and_save_waypoint()
            return
        if key == "z":
            self.move_gripper(GRIPPER_OPEN_NAME)
            return
        if key == "x":
            self.move_gripper(GRIPPER_CLOSE_NAME)
            return
        if key not in TEACH_KEY_BINDINGS:
            return
        joint_index, delta = TEACH_KEY_BINDINGS[key]
        self.jog_joint(joint_index, delta)


def search_grasp_pose(node, box_xyz, pre_grasp_gripper_opening=0.019):
    # 실제 로봇을 움직이지 않고, FK(정방향기구학) 계산만으로 여러 관절값 조합을 빠르게
    # 훑어서 "그리퍼 손가락 중점(pinch point)이 박스 중심에 가까우면서 충돌도 없는" 자세를
    # 찾는다. gripper_left_link/right_link의 실제 세계 좌표를 박스 좌표와 직접 비교하므로,
    # end_effector_link 기준 offset을 역산하는 것보다 훨씬 정확하다.
    bx, by, bz = box_xyz
    robot_model = node.moveit.get_robot_model()
    candidates = []
    joint1_deg = 1
    for joint2_deg in range(30, 50, 1):
        for joint3_deg in range(5, 55, 2):
            for joint4_deg in range(-100, -50, 2):
                joints = {
                    "joint1": math.radians(joint1_deg),
                    "joint2": math.radians(joint2_deg),
                    "joint3": math.radians(joint3_deg),
                    "joint4": math.radians(joint4_deg),
                    "gripper_left_joint": pre_grasp_gripper_opening,
                    "gripper_right_joint": pre_grasp_gripper_opening,
                }
                state = RobotState(robot_model)
                state.joint_positions = joints
                state.update()
                left = state.get_global_link_transform("gripper_left_link")[:3, 3]
                right = state.get_global_link_transform("gripper_right_link")[:3, 3]
                pinch = (left + right) / 2.0
                dist = math.dist(pinch, (bx, by, bz))
                with node.planning_scene_monitor.read_only() as scene:
                    valid = scene.is_state_valid(state, "arm")
                if not valid:
                    continue
                candidates.append({
                    "dist": dist, "pinch": pinch,
                    "joint2": joint2_deg, "joint3": joint3_deg, "joint4": joint4_deg,
                })
    if not candidates:
        node.logger.info("[탐색] 충돌 없는 자세를 하나도 못 찾음")
        return []
    candidates.sort(key=lambda c: c["dist"])
    top = candidates[:10]
    for c in top:
        node.logger.info(
            f"[탐색] 후보: joint2={c['joint2']} joint3={c['joint3']} joint4={c['joint4']} "
            f"pinch={c['pinch']} 박스와 거리={c['dist']:.4f}m"
        )
    return [
        {
            "joint1": math.radians(joint1_deg),
            "joint2": math.radians(c["joint2"]),
            "joint3": math.radians(c["joint3"]),
            "joint4": math.radians(c["joint4"]),
        }
        for c in top
    ]


def run_grab_demo(node, box_xyz=(0.283, 0.0, 0.025)):
    # 경유지(박스 위로 뜬 안전한 자세)는 고정, 최종 집기 자세는 FK 탐색으로 매번 찾음 -
    # 이전에는 최종 자세를 사람이 RViz에서 눈으로 찾았는데, 충돌만 피하고 손가락 중점이
    # 박스 중심과 5cm 넘게 떨어져 있어서 허공을 잡는 문제가 있었음 (실제로 겪음).
    waypoint = {"joint1": math.radians(1), "joint2": math.radians(20),
                "joint3": math.radians(20), "joint4": math.radians(-40)}
    ok = node.move_arm_to(joint_positions=waypoint)
    node.logger.info(f"[집기 시연] 경유지1 이동: {ok}")
    if not ok:
        return

    # 그리퍼를 완전히 열고(0.019) 접근하면 몸체 폭 때문에 더 못 들어갈 수 있어서,
    # 좁게 여러 단계로도 시도해봄 (더 좁을수록 몸체가 얇아져서 더 깊이 들어갈 여지가 있음).
    ok = False
    for opening in (0.019, 0.014, 0.010):
        candidates = search_grasp_pose(node, box_xyz, pre_grasp_gripper_opening=opening)
        node.logger.info(f"[집기 시연] pre-grasp opening={opening}, 후보 {len(candidates)}개")
        for i, final_joints in enumerate(candidates):
            ok = node.move_arm_to(joint_positions=final_joints)
            node.logger.info(
                f"[집기 시연] opening={opening} 최종 집기 자세 이동 (후보 {i + 1}): {ok}"
            )
            if ok:
                break
        if ok:
            break
    if not ok:
        return

    ok = node.close_gripper_on_object(0.008)
    node.logger.info(f"[집기 시연] 그리퍼 닫기: {ok}")


def check_slot_heights(node):
    # box_sort_project.py의 config/box_slots.yaml에 저장된 템플릿 자세들이 실제로
    # 어느 높이(z)에 있는지 FK로 확인 (사용자가 "1번 자리에서만 팔이 공중에 떠 있다"고
    # 보고해서 만든 진단용 - 실제 로봇은 안 움직임).
    robot_model = node.moveit.get_robot_model()
    templates = {
        "near(1,4,7...)": [0.0, 0.3, 0.7, -1.25],
        "mid(2,5,8...)": [0.0, 0.85, 0.05, -0.85],
        "far(3,6,9...)": [0.0, 0.85, -0.35, -0.55],
    }
    for label, j in templates.items():
        state = RobotState(robot_model)
        state.joint_positions = {
            "joint1": j[0], "joint2": j[1], "joint3": j[2], "joint4": j[3],
            "gripper_left_joint": 0.019, "gripper_right_joint": 0.019,
        }
        state.update()
        ee = state.get_global_link_transform("end_effector_link")[:3, 3]
        gl = state.get_global_link_transform("gripper_left_link")[:3, 3]
        node.logger.info(f"[슬롯 높이] {label}: end_effector={ee}, gripper_left={gl}")


def search_near_template(node):
    # joint2=0.3(반지름 고정)은 유지한 채, joint3/joint4만 바꿔가며 gripper_left_link의
    # 높이(z)가 mid 템플릿과 비슷한 지면 근처(z≈0.025)가 되는 조합을 FK로 탐색.
    robot_model = node.moveit.get_robot_model()
    best = None
    for joint3_deg in range(20, 70, 2):
        for joint4_deg in range(-100, -40, 2):
            state = RobotState(robot_model)
            state.joint_positions = {
                "joint1": 0.0, "joint2": math.radians(17),
                "joint3": math.radians(joint3_deg), "joint4": math.radians(joint4_deg),
                "gripper_left_joint": 0.019, "gripper_right_joint": 0.019,
            }
            state.update()
            gl = state.get_global_link_transform("gripper_left_link")[:3, 3]
            dist = abs(gl[2] - 0.025)
            if best is None or dist < best["dist"]:
                best = {"dist": dist, "joint3": joint3_deg, "joint4": joint4_deg, "gl": gl}
    node.logger.info(
        f"[near 재탐색] joint2=17deg 고정, 최적 joint3={best['joint3']} joint4={best['joint4']} "
        f"gripper_left={best['gl']}"
    )


def main(args=None):
    rclpy.init(args=args)
    node = BoxSortMoveItNode()

    if "--check-slots" in sys.argv:
        check_slot_heights(node)
        search_near_template(node)
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        return

    node.move_arm_to(configuration_name="init")
    real_box_xyz = (0.283, 0.0, 0.025)
    node.add_box_to_world(*real_box_xyz)

    if "--teach" not in sys.argv:
        # 기본 실행: 검증된 집기 시연을 한 번 재생하고 끝냄.
        try:
            run_grab_demo(node, real_box_xyz)
        finally:
            node.shutdown()
            node.destroy_node()
            rclpy.try_shutdown()
        return

    settings = termios.tcgetattr(sys.stdin)
    print(
        "MoveIt TEACH 모드 (장애물 충돌 확인용)\n"
        "  q/a/w/s/e/d/r/f : 관절1~4 수동 조작   z/x : 그리퍼 열기/닫기\n"
        "  스페이스바      : 지금 자세를 충돌 검사와 함께 저장"
        f" ({waypoints_file_path()})\n"
        "  Ctrl+C : 종료\n"
    )
    tty.setraw(sys.stdin.fileno())
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key(timeout=0.0)
            if key == "\x03":  # Ctrl+C
                break
            if key:
                node.handle_teach_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
