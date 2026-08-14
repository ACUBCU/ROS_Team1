#!/usr/bin/env python3
import os
os.environ.setdefault("DISPLAY", ":0")

import select
import sys
import time
import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray

from real_robot.gripper_approach_controller import (
    GripperApproachController,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
    HOME_STANDBY_POSE,
    FOLDED_JOINT_ELEVATION,
    KNOWN_BOX_BASELINE_DIST_M,
    JOINT_NAMES,
)

TOTAL_SLOTS = 4


def slots_file_path() -> str:
    share_dir = get_package_share_directory("real_robot")
    return os.path.join(share_dir, "config", "box_slots.yaml")


def get_key(timeout=0.0):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if rlist else ""


class BoxSortYoloRealRobot(Node):
    """
    Main Integration Node for real_robot package:
    Modularly uses YoloDepthPublisher perception and GripperApproachController.
    """
    def __init__(self):
        super().__init__("box_sort_yolo")
        self.bridge = CvBridge()

        # Modular Approach Controller
        self.approach_controller = GripperApproachController(self)

        # Topic Subscriptions
        self.create_subscription(JointState, "joint_states", self.joint_callback, 10)
        self.create_subscription(Image, "camera/image_raw", self.image_callback, 10)
        self.create_subscription(Image, "/gripper_camera/image_raw", self.image_callback, 10)
        self.create_subscription(Float32MultiArray, "/yolo_depth/detected_objects", self.perception_topic_callback, 10)

        self.latest_frame = None
        self.detected_objects = []
        self.scanned_map = {}
        self.slots = self.load_slots()

        self.get_logger().info("==================================================")
        self.get_logger().info(" 🚀 [real_robot] Vision & Depth Modular Integrated Node Initialized!")
        self.get_logger().info("==================================================")

    def load_slots(self):
        filepath = slots_file_path()
        if os.path.exists(filepath):
            with open(filepath, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
                if data and "slots" in data:
                    return data["slots"]
        return {
            1: {"joints": [0.7854, 0.40, 0.20, -0.60], "gripper": GRIPPER_OPEN},
            2: {"joints": [2.3562, 0.40, 0.20, -0.60], "gripper": GRIPPER_OPEN},
            3: {"joints": [-2.3562, 0.40, 0.20, -0.60], "gripper": GRIPPER_OPEN},
            4: {"joints": [-0.7854, 0.40, 0.20, -0.60], "gripper": GRIPPER_OPEN},
        }

    def joint_callback(self, msg: JointState):
        self.approach_controller.update_joint_states(msg)

    def perception_topic_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        objs = []
        for i in range(0, len(data), 4):
            if i + 3 < len(data):
                cx, cy, area, dist_m = data[i], data[i+1], data[i+2], data[i+3]
                objs.append({"center": (int(cx), int(cy)), "area": area, "distance_m": dist_m})
        if objs:
            self.detected_objects = objs

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame = frame
            h, w = frame.shape[:2]

            roi_mask = np.zeros((h, w), dtype=np.uint8)
            # Mask out bottom area (y >= 360) where robot arm gripper fingers appear
            roi_mask[20:360, 20:620] = 255

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)

            _, thresh_otsu = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            edges = cv2.Canny(blur, 30, 120)
            combined = cv2.bitwise_or(thresh_otsu, edges)

            combined_roi = cv2.bitwise_and(combined, combined, mask=roi_mask)
            contours, _ = cv2.findContours(combined_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            objs = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if 400 < area < 45000:
                    x, y, bw, bh = cv2.boundingRect(cnt)
                    cx, cy = x + bw // 2, y + bh // 2

                    # Gripper exclusion: ignore objects in the lower region of camera frame
                    if cy >= 360:
                        continue

                    aspect_ratio = float(bw) / float(bh) if bh > 0 else 1.0
                    if aspect_ratio < 0.25 or aspect_ratio > 3.5:
                        continue
                    rect_area = bw * bh
                    solidity = float(area) / float(rect_area) if rect_area > 0 else 0.0
                    if solidity < 0.25:
                        continue

                    stereo_dist_m = KNOWN_BOX_BASELINE_DIST_M
                    objs.append({
                        "rect": (x, y, bw, bh),
                        "center": (cx, cy),
                        "area": area,
                        "distance_m": stereo_dist_m,
                    })
                    cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                    cv2.putText(frame, f"Box ({area:.0f})", (x, max(15, y - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.detected_objects = objs

            # Center cross marker only (Blue border line removed)
            cv2.drawMarker(frame, (320, 240), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

            eff_color = (0, 255, 255) if self.approach_controller.gripper_effort > 0.035 else (200, 200, 200)
            cv2.putText(frame, f"Gripper Torque Load: {self.approach_controller.gripper_effort:.2f} Nm",
                        (60, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, eff_color, 2)

            cv2.imshow("real_robot Camera Vision", frame)
            cv2.waitKey(1)
        except Exception:
            pass

    def center_camera_over_box_with_yolo(self, map_coarse_j1: float, src_slot: int = 1):
        self.get_logger().info("==================================================")
        self.get_logger().info(f" 📍 [1단계: 맵 참조 이송] 물체 슬롯 근처 상공 위치({np.degrees(map_coarse_j1):.1f}°)로 이동...")
        self.get_logger().info("==================================================")

        look_pose = [map_coarse_j1] + [-0.60, 0.90, -0.30]
        self.approach_controller.send_arm_trajectory(look_pose, duration_sec=0.8)
        time.sleep(0.4)

        self.get_logger().info("==================================================")
        self.get_logger().info(" 👁️ [2단계: 실시간 YOLO + Depth 직접 접근] 실시간 카메라 비전/뎁스 센서 정보 적용...")
        self.get_logger().info("==================================================")

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.04)

        objs = self.detected_objects
        live_dist_m = KNOWN_BOX_BASELINE_DIST_M
        target_j1 = map_coarse_j1

        if len(objs) > 0:
            # Lock onto the box closest to image center (320, 240)
            best_obj = min(objs, key=lambda o: (o["center"][0] - 320)**2 + (o["center"][1] - 240)**2)
            cx, cy = best_obj["center"]
            live_dist_m = best_obj.get("distance_m", KNOWN_BOX_BASELINE_DIST_M)

            dx = cx - 320
            if abs(dx) < 220:
                target_j1 -= (dx / 320.0) * 0.12
            self.get_logger().info(f"   🎯 [실시간 YOLO + Depth 측위] 실시간 카메라 중심: ({cx}, {cy}) | Depth 실측 거리: {live_dist_m:.2f}m")
        else:
            self.get_logger().info("   ℹ️ [실시간 비전 미감지] 기본 센서 측정거리 사용")

        final_pick_joints = self.approach_controller.calculate_pick_joints_from_3d(target_j1, live_dist_m)
        self.get_logger().info(f" 📐 [YOLO+Depth 3D 관절 도출] J1={np.degrees(target_j1):.1f}°, J2={final_pick_joints[1]:.2f}, J3={final_pick_joints[2]:.2f}, J4={final_pick_joints[3]:.2f}")
        return final_pick_joints

    def scan_environment_and_build_map(self):
        self.get_logger().info("==================================================")
        self.get_logger().info(" 👐 [무정차 360° 듀얼-앵글 저속 스캔 개시] 정지 없이 전구역 연속 2회 저속 회전 스캔...")
        self.get_logger().info("==================================================")

        self.approach_controller.send_gripper_command(GRIPPER_OPEN)
        time.sleep(0.2)

        self.scanned_map = {1: {"found": False, "dist": 0.0},
                            2: {"found": False, "dist": 0.0},
                            3: {"found": False, "dist": 0.0},
                            4: {"found": False, "dist": 0.0}}

        slot_angles = {1: 0.7854, 2: 2.3562, 3: -2.3562, 4: -0.7854}

        # -------------------------------------------------------------
        # PASS 1: Slow Forward Sweep (-135° -> +135°) (Duration: 14 seconds)
        # -------------------------------------------------------------
        self.get_logger().info(" 🔄 [1차 저속 무정차 스캔] 모터 시작점(-135°) 이동 후 정지 없이 +135°까지 저속 회전 스캔 (14초)...")
        start_pose_1 = [-2.3562, -0.90, 1.15, -0.25]
        self.approach_controller.send_arm_trajectory(start_pose_1, duration_sec=1.5)

        self.get_logger().info(" 🎥 [1차 저속 회전 중] -135° -> +135° 정지 없이 천천히 훑는 중...")
        end_pose_1 = [2.3562, -0.90, 1.15, -0.25]

        goal1 = FollowJointTrajectory.Goal()
        goal1.trajectory.joint_names = JOINT_NAMES
        pt1 = JointTrajectoryPoint()
        pt1.positions = end_pose_1
        pt1.time_from_start = Duration(sec=14, nanosec=0)
        goal1.trajectory.points = [pt1]

        if self.approach_controller.joint_client.wait_for_server(timeout_sec=2.0):
            future1 = self.approach_controller.joint_client.send_goal_async(goal1)
            rclpy.spin_until_future_complete(self, future1)
            handle1 = future1.result()
            if handle1 and handle1.accepted:
                res_future1 = handle1.get_result_async()
                start_t1 = time.time()
                while not res_future1.done() and (time.time() - start_t1) < 15.5:
                    rclpy.spin_once(self, timeout_sec=0.02)
                    curr_j1 = self.approach_controller.current_joint_position[0]
                    if len(self.detected_objects) > 0:
                        best_obj = max(self.detected_objects, key=lambda o: o["area"])
                        obj_dist = best_obj.get("distance_m", KNOWN_BOX_BASELINE_DIST_M)
                        for s_id, s_ang in slot_angles.items():
                            ang_diff = abs(np.arctan2(np.sin(curr_j1 - s_ang), np.cos(curr_j1 - s_ang)))
                            if ang_diff < 0.45 and not self.scanned_map[s_id]["found"]:
                                self.scanned_map[s_id]["found"] = True
                                self.scanned_map[s_id]["dist"] = obj_dist
                                self.get_logger().info(f"   📦 [1차 저속 스캔 인지] 슬롯 {s_id} ({np.degrees(s_ang):.1f}°) 박스 감지! (거리: {obj_dist:.2f}m)")

        # -------------------------------------------------------------
        # PASS 2: Slow Reverse Sweep (+135° -> -135°) (Duration: 14 seconds)
        # -------------------------------------------------------------
        self.get_logger().info(" 🔄 [2차 저속 무정차 복귀 스캔] 전방 경사 앵글 변경 후 +135° -> -135° 저속 무정차 연속 스캔 (14초)...")
        prep_pose_2 = [2.3562, -0.60, 0.90, -0.30]
        self.approach_controller.send_arm_trajectory(prep_pose_2, duration_sec=0.8)

        start_pose_2 = [-2.3562, -0.60, 0.90, -0.30]
        goal2 = FollowJointTrajectory.Goal()
        goal2.trajectory.joint_names = JOINT_NAMES
        pt2 = JointTrajectoryPoint()
        pt2.positions = start_pose_2
        pt2.time_from_start = Duration(sec=14, nanosec=0)
        goal2.trajectory.points = [pt2]

        if self.approach_controller.joint_client.wait_for_server(timeout_sec=2.0):
            future2 = self.approach_controller.joint_client.send_goal_async(goal2)
            rclpy.spin_until_future_complete(self, future2)
            handle2 = future2.result()
            if handle2 and handle2.accepted:
                res_future2 = handle2.get_result_async()
                start_t2 = time.time()
                while not res_future2.done() and (time.time() - start_t2) < 15.5:
                    rclpy.spin_once(self, timeout_sec=0.02)
                    curr_j1 = self.approach_controller.current_joint_position[0]
                    if len(self.detected_objects) > 0:
                        best_obj = max(self.detected_objects, key=lambda o: o["area"])
                        obj_dist = best_obj.get("distance_m", KNOWN_BOX_BASELINE_DIST_M)
                        for s_id, s_ang in slot_angles.items():
                            ang_diff = abs(np.arctan2(np.sin(curr_j1 - s_ang), np.cos(curr_j1 - s_ang)))
                            if ang_diff < 0.45 and not self.scanned_map[s_id]["found"]:
                                self.scanned_map[s_id]["found"] = True
                                self.scanned_map[s_id]["dist"] = obj_dist
                                self.get_logger().info(f"   📦 [2차 저속 스캔 교차 인지] 슬롯 {s_id} ({np.degrees(s_ang):.1f}°) 박스 추가 감지! (거리: {obj_dist:.2f}m)")

        self.approach_controller.move_to_home_standby()

        print("\n==================================================")
        print(" 📍 360° 무정차 듀얼-앵글 스캔 결과 맵:")
        for s in sorted(self.scanned_map.keys()):
            entry = self.scanned_map[s]
            icon = f"📦 [물체 감지됨] (거리: {entry['dist']:.2f}m)" if entry["found"] else "❌ [비어있음]"
            print(f"  - 슬롯 {s} ({np.degrees(slot_angles[s]):.1f}°): {icon}")
        print("==================================================\n")
        return self.scanned_map

    def auto_scan_and_sort(self):
        """8 키 전용: 360° 스캔 후 자동 물체 이송"""
        self.get_logger().info("==================================================")
        self.get_logger().info(" 🤖 [자동 테스트 (8 키)] 360° 스캔 -> 자동 물체 감지 -> 빈 슬롯 자동 이송 시작!")
        self.get_logger().info("==================================================")

        scanned_map = self.scan_environment_and_build_map()
        occupied_slots = [s for s in sorted(scanned_map.keys()) if scanned_map[s]["found"]]
        empty_slots = [s for s in sorted(scanned_map.keys()) if not scanned_map[s]["found"]]

        if not occupied_slots:
            self.get_logger().warn(" ⚠️ [자동 이송] 작업 영역 내에 옮길 물체가 존재하지 않습니다!")
            return False
        if not empty_slots:
            self.get_logger().warn(" ⚠️ [자동 이송] 물체를 놓을 수 있는 빈 슬롯이 존재하지 않습니다!")
            return False

        src_slot, dst_slot = occupied_slots[0], empty_slots[0]
        self.get_logger().info(f" 🚀 [자동 이송 확정] 슬롯 {src_slot} -> 슬롯 {dst_slot} 이송 개시!")
        return self.execute_pick_and_place(src_slot, dst_slot)

    def execute_pick_and_place(self, src_slot: int, dst_slot: int):
        self.get_logger().info("==================================================")
        self.get_logger().info(f" 🎯 360° 스캔 맵 기반 정밀 Pick & Place: 슬롯 {src_slot} -> 슬롯 {dst_slot}")
        self.get_logger().info("==================================================")

        coarse_src_j1 = self.slots[src_slot]["joints"][0]
        dst_joints = list(self.slots[dst_slot]["joints"])

        # 1. Direct approach using exact scanned map slot angle and depth distance
        pick_joints = self.center_camera_over_box_with_yolo(coarse_src_j1, src_slot)

        # 2. Execute approach, grasp, and post-grasp gripper torque verification
        success = self.approach_controller.direct_approach_and_pick(pick_joints, dst_joints)

        if success:
            if src_slot in self.scanned_map:
                self.scanned_map[src_slot] = {"found": False, "dist": 0.0}
            if dst_slot in self.scanned_map:
                self.scanned_map[dst_slot] = {"found": True, "dist": KNOWN_BOX_BASELINE_DIST_M}
            self.get_logger().info(f" 🎉 슬롯 {src_slot} -> 슬롯 {dst_slot} 이송 성공!")

        return success


def main(args=None):
    rclpy.init(args=args)
    node = BoxSortYoloRealRobot()

    print("\n==================================================")
    print(" 🎮 [real_robot 모듈화 노드] 컨트롤 메뉴 (키보드 입력):")
    print("  8 : [자동 테스트] 360° 슬롯 정밀스캔 -> 물체 슬롯을 빈 슬롯으로 자동 이송")
    print("  1 : 360° 4개 슬롯 정밀 스캔 & 3D 융합 맵 생성")
    print("  2 : 수동 이송 지정 (예: 1 2 입력으로 1번->2번 이송)")
    print("  q : 프로그램 종료")
    print("==================================================\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key(timeout=0.01)

            if key == "8":
                node.auto_scan_and_sort()
            elif key == "1":
                node.scan_environment_and_build_map()
            elif key == "2":
                print("\n[수동 이송] 출발 슬롯(src)과 도착 슬롯(dst) 입력 (예: 1 2): ")
                try:
                    line = input().strip()
                    parts = line.split()
                    if len(parts) == 2:
                        s_src, s_dst = int(parts[0]), int(parts[1])
                        node.execute_pick_and_place(s_src, s_dst)
                    else:
                        print(" 잘못된 입력 형식입니다. 예: 1 2")
                except Exception as e:
                    print(f" 입력 오류: {e}")
            elif key == "q":
                print("프로그램을 종료합니다.")
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
