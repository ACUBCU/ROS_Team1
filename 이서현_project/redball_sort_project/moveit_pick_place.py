# MoveIt(MoveItPy)로 "좌표(x, y, z)"만 주면 역기구학+충돌회피 경로까지 알아서 계산해서
# 집고 옮기는 기능만 모아둔 모듈입니다. box_sort_project.py(사전에 가르친 관절 각도로만
# 이동)와 달리, 천장 카메라가 실시간으로 본 마커의 실제 좌표로 정확히 팔을 뻗을 수 있습니다.
#
# [실행 순서 - 반드시 필요]
#   MoveItPy가 SRDF/kinematics/OMPL 설정을 파라미터로 받아야 하므로, 그냥
#   `ros2 run redball_sort_project web_control_moveit` 로는 안 되고 launch 파일이 필요합니다.
#   1) 터미널 1: Gazebo
#      ros2 launch open_manipulator_bringup open_manipulator_x_gazebo.launch.py
#   2) 터미널 2: 이 launch 파일이 web_control_moveit을 MoveIt 설정과 함께 띄움
#      ros2 launch redball_sort_project box_sort_moveit.launch.py use_sim_time:=true node_executable:=web_control_moveit
#
# 이 모듈은 box_sort_moveit_project.py(2026-08-10, 삭제됐다가 복구함)에서
# "좌표 기반 집기/옮기기" 부분만 그대로 가져왔습니다 - 여기 있는 우회/보정 로직들은
# 전부 실제로 겪은 문제를 해결하며 만든 것이라 손대지 않고 재사용합니다.

import math

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from rclpy.action import ActionClient
from rclpy.logging import get_logger
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

# 실제 aruco_cube 모델(models/aruco_cube/model.sdf)의 크기와 맞춤.
BOX_SIZE = [0.05, 0.05, 0.05]

# 그리퍼가 물건을 잡을 때 쓰는 자세(orientation). mid 자세(joints=[0,0.85,0.05,-0.85])에서
# end_effector_link TF를 실측하니 사실상 무회전(항등 quaternion)에 가까웠음 - 재사용.
GRASP_ORIENTATION = (0.0, 0.0, 0.0, 1.0)  # x, y, z, w  (지금은 pose_xyz 기반 접근에만 쓰임)

# 그리퍼 열기/닫기 목표값. box_sort_project.py의 GRIPPER_OPEN/GRIPPER_CLOSE와 동일 - 그리퍼는
# MoveIt 계획 없이 컨트롤러에 직접 보내므로(open_gripper/grip_object 참고) 같은 값을 재사용.
GRIPPER_OPEN_POSITION = 0.019
GRIPPER_CLOSE_POSITION = -0.01
ARM_CONTROLLER = "arm_controller"
PLANNING_LINK = "end_effector_link"

# box_sort_project.py의 SAFE_LIFT_JOINT2와 동일한 값·근거: joint2=0.40일 때 그리퍼 높이가
# 바닥에서 약 14.6cm - 박스 높이(5cm)보다 훨씬 높아서 자리 사이를 이동할 때 다른 자리의
# 박스를 안 침.
SAFE_LIFT_JOINT2 = 0.40


class MoveItPickPlace(Node):
    def __init__(self):
        super().__init__("box_sort_moveit_project")
        self.logger = get_logger("box_sort_moveit_project")

        # move_group 파라미터(SRDF 등)가 이미 이 프로세스에 로드돼 있어야 여기서 안 멈춤
        # (launch 파일이 --ros-args로 넘겨줌 - box_sort_moveit.launch.py 참고).
        self.moveit = MoveItPy(node_name="box_sort_moveit_py")
        self.arm = self.moveit.get_planning_component("arm")
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()

        self.object_id = "sorted_box"

        self.joint_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")
        if not self.joint_client.wait_for_server(timeout_sec=5.0):
            self.logger.info("arm_controller 액션 서버 연결 대기 시간 초과 (5초)")
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.logger.info("gripper_controller 액션 서버 연결 대기 시간 초과 (5초)")

    # ------------------------------------------------------------------
    def plan_and_execute(self, component, controller_name: str, *, configuration_name=None,
                          joint_positions=None, pose_xyz=None) -> bool:
        component.set_start_state_to_current_state()

        if configuration_name is not None:
            component.set_goal_state(configuration_name=configuration_name)
        elif joint_positions is not None:
            from moveit.core.kinematic_constraints import construct_joint_constraint

            robot_model = self.moveit.get_robot_model()
            robot_state = RobotState(robot_model)
            robot_state.joint_positions = joint_positions
            joint_model_group = robot_model.get_joint_model_group(component.planning_group_name)
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
        # "Action client not connected"로 계속 실패하는 문제가 있었음(WSL DDS 디스커버리 추정).
        # 대신 이미 연결이 확인된 self.joint_client/self.gripper_client로 직접 실행함.
        return self._execute_trajectory_directly(plan_result.trajectory, controller_name)

    def _send_goal_and_wait(self, client: ActionClient, goal, timeout_sec: float = 30.0):
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

        raise ValueError(f"알 수 없는 controller_name: {controller_name}")

    def close_gripper_on_object(self, target_position: float, max_effort: float = 10.0) -> bool:
        # 닫는 동작은 시작 자세가 이미 물체와 접촉 중이라 MoveIt 계획이 거부됨 - 컨트롤러에
        # 바로 명령을 보냄 (계획/충돌검사 없이 단순 직선 동작이라 문제 없음).
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

    def open_gripper(self) -> bool:
        # 그리퍼 열기/닫기는 MoveIt 계획(충돌 검사)을 거치지 않고 컨트롤러에 바로 명령을 보냄.
        # 실제로 겪은 문제: 예전 테스트에서 남은 그리퍼 자세가 URDF의 joint 하한(-0.011)과
        # 부동소수점 오차로 살짝 어긋나 있으면, MoveIt의 "시작 상태가 범위 안에 있는지" 검사가
        # 그 자체로 실패해서 열기 계획 자체를 거부함 - 열기/닫기는 장애물을 피해갈 경로가
        # 필요한 단순 직선 동작이므로, 애초에 MoveIt 계획을 거치지 않는 게 더 안전함.
        return self.close_gripper_on_object(GRIPPER_OPEN_POSITION)

    def grip_object(self) -> bool:
        return self.close_gripper_on_object(GRIPPER_CLOSE_POSITION)

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
        pose.orientation.w = 1.0

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
    def search_grasp_pose(self, target_xyz, pre_grasp_gripper_opening=0.019):
        # 목표 (x,y,z)로 pose_xyz IK를 직접 시키면, 그 자리에 이미 등록해둔 박스 충돌 모델과
        # 거의 겹쳐서 "goal tree에서 유효한 상태를 하나도 못 찾음"으로 항상 실패했음(실제로
        # 겪음). 대신 FK(정방향기구학)로 여러 관절 조합을 빠르게 훑어서, "그리퍼 손가락
        # 중점(pinch point)이 목표에 가까우면서 충돌도 없는" 자세를 찾아 관절값으로 직접 이동함
        # (box_sort_moveit_project.py의 옛 집기 시연에서 실제로 검증된 방식).
        # joint1은 목표 방향으로 바로 겨냥하고(atan2), joint2~4만 훑어서 반지름/높이를 맞춤 -
        # box_sort_project.py의 "자리는 joint1 각도로 정해지는 원 위에 있다"는 가정과 동일.
        tx, ty, tz = target_xyz
        joint1 = math.atan2(ty, tx)
        robot_model = self.moveit.get_robot_model()
        candidates = []
        for joint2_deg in range(20, 60, 2):
            for joint3_deg in range(-10, 60, 2):
                for joint4_deg in range(-100, -30, 2):
                    joints = {
                        "joint1": joint1,
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
                    dist = math.dist(pinch, (tx, ty, tz))
                    if dist > 0.08:  # 너무 먼 후보는 충돌 검사(느림)까지 갈 필요 없이 미리 거름
                        continue
                    with self.planning_scene_monitor.read_only() as scene:
                        valid = scene.is_state_valid(state, "arm")
                    if not valid:
                        continue
                    candidates.append({"dist": dist, "joints": joints})
        if not candidates:
            self.logger.info(f"[집기 자세 탐색] {target_xyz} 근처에서 충돌 없는 자세를 못 찾음")
            return None
        candidates.sort(key=lambda c: c["dist"])
        best = candidates[0]
        self.logger.info(f"[집기 자세 탐색] 최적 자세(거리={best['dist']:.4f}m): {best['joints']}")
        return {k: v for k, v in best["joints"].items() if k.startswith("joint")}

    def get_current_arm_joints(self) -> dict:
        with self.planning_scene_monitor.read_only() as scene:
            positions = scene.current_state.joint_positions
            return {name: float(positions[name]) for name in ("joint1", "joint2", "joint3", "joint4")}

    def _move_via_safe_lift(self, target_joints: dict) -> bool:
        # box_sort_project.py의 _safe_move_to()와 같은 패턴: (지금 위치에서 안전 높이로
        # 들어올리기) -> (들어올린 채로 목표 joint1까지 회전) -> (목표 자세로 내려놓기).
        # 실제로 겪은 문제: 집은 뒤 목표 자세로 곧장(관절 궤적 한 번에) 이동시켰더니, 팔이
        # 낮은 높이로 회전하면서 박스가 바닥에 끌리거나 다른 자리를 스쳐서 목표에서 50cm
        # 이상 벗어난 곳에 떨어짐 (실측). 3단계로 나눠서 항상 충분히 들어올린 채로 회전해야 함.
        current = self.get_current_arm_joints()
        raise_pose = {**current, "joint2": SAFE_LIFT_JOINT2}
        if not self.move_arm_to(joint_positions=raise_pose):
            return False

        rotate_pose = {**target_joints, "joint2": SAFE_LIFT_JOINT2}
        if not self.move_arm_to(joint_positions=rotate_pose):
            return False

        return self.move_arm_to(joint_positions=target_joints)

    def pick_and_place(self, pick_xyz, place_xyz) -> bool:
        px, py, pz = pick_xyz
        self.add_box_to_world(px, py, pz)

        self.logger.info(f"1) 집을 위치로 이동(자세 탐색): {pick_xyz}")
        pick_joints = self.search_grasp_pose(pick_xyz)
        if pick_joints is None or not self._move_via_safe_lift(pick_joints):
            self.remove_box_from_world()
            return False

        self.logger.info("2) 그리퍼 열기")
        if not self.open_gripper():
            return False

        self.logger.info("3) 그리퍼 닫기 (집기)")
        if not self.grip_object():
            return False

        self.attach_box()

        self.logger.info(f"4) 놓을 위치로 이동(자세 탐색): {place_xyz}")
        place_joints = self.search_grasp_pose(place_xyz)
        if place_joints is None or not self._move_via_safe_lift(place_joints):
            return False

        self.logger.info("5) 그리퍼 열기 (놓기)")
        if not self.open_gripper():
            return False

        self.detach_box(place_xyz)
        self.logger.info("이동 완료")
        return True

    def shutdown(self):
        self.arm = None
        self.moveit.shutdown()
