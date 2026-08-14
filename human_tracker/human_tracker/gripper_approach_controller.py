#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
GRIPPER_OPEN = 0.019
GRIPPER_CLOSE = -0.01
HOME_STANDBY_POSE = [0.0, -1.05, 1.25, -0.30]
FOLDED_JOINT_ELEVATION = [-0.80, 1.10, -0.30]
KNOWN_BOX_BASELINE_DIST_M = 0.24


class GripperApproachController:
    """
    Gripper & Manipulator Approach Controller for real_robot:
    Executes direct 3D approach using YOLO + Depth calculated joint angles (no search loops).
    Checks gripper torque load (effort) AFTER attempting to grasp to verify object capture.
    """
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()

        self.joint_client = ActionClient(
            node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(
            node, GripperCommand, "/gripper_controller/gripper_cmd"
        )

        self.current_joint_position = [0.0, 0.0, 0.0, 0.0]
        self.gripper_effort = 0.0
        self.gripper_position = 0.0

    def update_joint_states(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_eff = dict(zip(msg.name, msg.effort)) if len(msg.effort) == len(msg.name) else {}

        if all(j in name_to_pos for j in JOINT_NAMES):
            self.current_joint_position = [name_to_pos[j] for j in JOINT_NAMES]

        if "gripper_left_joint" in name_to_pos:
            self.gripper_position = name_to_pos["gripper_left_joint"]
        if "gripper_left_joint" in name_to_eff:
            self.gripper_effort = abs(name_to_eff["gripper_left_joint"])

    def send_arm_trajectory(self, target_positions: list, duration_sec: float = None) -> bool:
        if not self.joint_client.wait_for_server(timeout_sec=2.0):
            return False

        if duration_sec is None:
            max_diff = max(abs(curr - tgt) for curr, tgt in zip(self.current_joint_position, target_positions))
            duration_sec = max(0.6, max_diff / 0.15)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = target_positions
        pt.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec - int(duration_sec)) * 1e9))
        goal.trajectory.points = [pt]

        future = self.joint_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        handle = future.result()
        if not handle or not handle.accepted:
            return False

        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res_future)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def send_gripper_command(self, position: float) -> bool:
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            return False
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 100.0
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        handle = future.result()
        if not handle or not handle.accepted:
            return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res_future)
        return res_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def move_to_home_standby(self):
        self.logger.info(" 🚀 로봇 팔을 초기 대기 위치로 빠르게 이동합니다...")
        return self.send_arm_trajectory(HOME_STANDBY_POSE, duration_sec=0.7)

    def calculate_pick_joints_from_3d(self, target_j1: float, distance_m: float) -> list:
        """Direct 3D joint angle calculation without search loops."""
        depth_clamped = max(0.20, min(0.35, distance_m))
        delta_r = depth_clamped - KNOWN_BOX_BASELINE_DIST_M

        # Deep descent until gripper tip is ~2cm above ground/table level
        joint2_reach = 0.52 + delta_r * 0.70
        joint3_reach = 0.54 + delta_r * 0.50
        joint4_reach = - (joint2_reach + joint3_reach)

        return [target_j1, joint2_reach, joint3_reach, joint4_reach]

    def direct_approach_and_pick(self, pick_joints: list, dst_joints: list) -> bool:
        """
        Direct approach execution:
        1. Direct approach to pick_joints (no search loops).
        2. Attempt grasp (GRIPPER_CLOSE).
        3. Verify gripper torque effort AFTER grasp.
        4. Lift & transfer to target slot while keeping gripper securely closed.
        """
        self.send_gripper_command(GRIPPER_OPEN)
        time.sleep(0.3)

        self.logger.info(" ➡️ [1단계 하강 접근] YOLO + Depth 3D 좌표로 파지 위치 상공 도달...")
        self.send_arm_trajectory(pick_joints, duration_sec=1.2)
        time.sleep(0.4)

        # Move 5cm forward right before closing the gripper
        self.logger.info(" ⏩ [2단계 5cm 전진] 그리퍼를 닫기 전 5cm 앞으로 전진 슬라이딩...")
        j1, j2, j3, j4 = pick_joints
        j2_fwd = j2 + 0.060
        j3_fwd = j3 + 0.035
        j4_fwd = - (j2_fwd + j3_fwd)
        forward_pick_joints = [j1, j2_fwd, j3_fwd, j4_fwd]
        self.send_arm_trajectory(forward_pick_joints, duration_sec=0.5)
        time.sleep(0.3)

        self.logger.info(" ✊ [3단계 파지 시도] 그리퍼 감싸기 닫기 실행...")
        self.send_gripper_command(GRIPPER_CLOSE)
        time.sleep(0.5)

        for _ in range(8):
            rclpy.spin_once(self.node, timeout_sec=0.04)

        initial_effort = self.gripper_effort
        self.logger.info(f" 📊 [파지 후 그리퍼 토크 검증] 측정 토크 부하: {initial_effort:.2f} Nm")

        # Lift box to elevated transfer posture
        curr_j1 = self.current_joint_position[0]
        lifted_pose = [curr_j1] + FOLDED_JOINT_ELEVATION
        self.send_arm_trajectory(lifted_pose, duration_sec=1.0)
        time.sleep(0.3)

        # Validate initial grasp effort (In Gazebo/Real robot, initial effort >= 0.005 Nm confirms grasp)
        if initial_effort < 0.005:
            self.logger.warn(f" ⚠️ [파지 토크 부하 미달 ({initial_effort:.2f} Nm)] 파지 실패로 이송을 취소합니다.")
            self.send_gripper_command(GRIPPER_OPEN)
            self.move_to_home_standby()
            return False

        self.logger.info(" ✅ [파지 및 토크 검증 성공!] 물체를 공중 유지하며 목표 슬롯으로 이송을 진행합니다.")

        # Transfer to destination slot while keeping gripper tightly closed
        folded_dst = [dst_joints[0]] + FOLDED_JOINT_ELEVATION
        self.send_arm_trajectory(folded_dst, duration_sec=1.2)
        time.sleep(0.3)

        # Lower to destination slot posture
        self.send_arm_trajectory(dst_joints, duration_sec=1.0)
        time.sleep(0.4)

        self.logger.info(" 🖐️ [물체 안착 및 놓기] 그리퍼 오픈...")
        self.send_gripper_command(GRIPPER_OPEN)
        time.sleep(0.5)

        self.move_to_home_standby()
        return True
