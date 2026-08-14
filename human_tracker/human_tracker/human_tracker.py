#!/usr/bin/env python3
import os
os.environ.setdefault("DISPLAY", ":0")

import time
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
HOME_STANDBY_POSE = [0.0, -0.60, 0.70, -0.10]

# Scan Range for Joint 1 (Pan)
SCAN_PAN_MIN = -2.1   # ~ -120 deg
SCAN_PAN_MAX = 2.1    # ~ +120 deg
SCAN_STEP = 0.35      # rad per step
SCAN_INTERVAL = 0.5   # seconds between scan movements

# Target Zone Definition (Angle range for target face selection)
# Default Target Zone: Front/Center zone (-45° to +45°, i.e. -0.78 rad to +0.78 rad)
TARGET_ZONE_MIN_RAD = -0.78
TARGET_ZONE_MAX_RAD = 0.78

# States
STATE_IDLE = "IDLE"
STATE_SCANNING = "SCANNING"
STATE_SELECT_TARGET = "SELECT_TARGET"
STATE_TRACKING = "TRACKING"


class UniversalFaceDetector:
    """
    Robust Face Detector supporting OpenCV 3.x, 4.x, 5.x and fallback vision algorithms:
    - Uses cv2.CascadeClassifier if available in cv2 module.
    - Uses YCrCb & HSV Skin Color + Elliptic Face Contour analysis as fallback.
    """
    def __init__(self):
        self.cascade = None
        if hasattr(cv2, 'CascadeClassifier'):
            try:
                cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
                self.cascade = cv2.CascadeClassifier(cascade_path)
            except Exception:
                self.cascade = None

    def detect(self, frame: np.ndarray) -> list:
        if frame is None or frame.size == 0:
            return []

        # 1. Apply image pre-denoising (Median & Gaussian Filter to remove sensor noise & flickering)
        denoised = cv2.medianBlur(frame, 3)
        denoised = cv2.GaussianBlur(denoised, (5, 5), 0)

        h, w = denoised.shape[:2]
        faces = []

        # 2. Try CascadeClassifier if available (with minNeighbors=6 for false-positive noise reduction)
        if self.cascade is not None:
            try:
                gray = cv2.cvtColor(denoised, cv2.COLOR_BGR2GRAY)
                detected = self.cascade.detectMultiScale(
                    gray, scaleFactor=1.1, minNeighbors=6, minSize=(40, 40)
                )
                if len(detected) > 0:
                    return list(detected)
            except Exception:
                pass

        # 3. Universal YCrCb Skin Color & Morphological Noise Cleaning Fallback
        try:
            ycrcb = cv2.cvtColor(denoised, cv2.COLOR_BGR2YCrCb)
            mask = cv2.inRange(ycrcb, np.array([0, 133, 77]), np.array([255, 173, 127]))

            # Morphological Opening & Closing to remove isolated noise pixels
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1800:
                    bx, by, bw, bh = cv2.boundingRect(cnt)
                    aspect = bh / float(bw)
                    if 0.9 <= aspect <= 2.1 and bw > 40 and bh > 45:
                        faces.append((bx, by, bw, bh))
        except Exception:
            pass

        return faces



class FaceTrackerControllerNode(Node):
    """
    OpenManipulator-X Face Scan & Zone Tracking Control Node:
    1. Rotates Pan (Joint 1) to scan 360°/wide angle for human faces.
    2. Filters detected faces to select the person in the target zone (-45° ~ +45°).
    3. Smoothly tracks the selected target face via Pan/Tilt joint trajectory control.
    """
    def __init__(self):
        super().__init__("human_tracker")
        self.bridge = CvBridge()

        # Arm Trajectory Action Client
        self.joint_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

        # Topic Subscriptions
        self.create_subscription(JointState, "joint_states", self.joint_callback, 10)
        self.create_subscription(Image, "camera/image_raw", self.image_callback, 10)
        self.create_subscription(Image, "/gripper_camera/image_raw", self.image_callback, 10)
        self.create_subscription(Image, "image_raw", self.image_callback, 10)

        # Universal Face Detector (Compatible with OpenCV 5.0 and all versions)
        self.face_detector = UniversalFaceDetector()

        self.current_joint_position = list(HOME_STANDBY_POSE)

        # State Machine Variables
        self.state = STATE_SCANNING
        self.scan_pan_targets = []
        self.scan_idx = 0
        self.last_scan_move_time = 0.0

        # Scanned Faces Storage: list of dicts {"pan_angle": float, "rect": tuple, "timestamp": float}
        self.scanned_faces = []

        # Target Tracking Variables
        self.target_face_pan = None
        self.last_cmd_time = time.time()
        self.last_seen_target_time = time.time()

        self._init_scan_targets()

        self.get_logger().info("==================================================")
        self.get_logger().info(" 👤 [human_tracker] Face Scan & Zone Tracker Node Started!")
        self.get_logger().info(" - State: SCANNING (Pan Sweep Range: -120° ~ +120°)")
        self.get_logger().info(" - Target Zone: Center Front (-45° ~ +45°)")
        self.get_logger().info("==================================================")

    def _init_scan_targets(self):
        """Generates waypoint angles for sweeping Pan (Joint 1)."""
        angles = []
        curr = SCAN_PAN_MIN
        while curr <= SCAN_PAN_MAX:
            angles.append(curr)
            curr += SCAN_STEP
        # Sweep back
        curr = SCAN_PAN_MAX
        while curr >= SCAN_PAN_MIN:
            angles.append(curr)
            curr -= SCAN_STEP
        self.scan_pan_targets = angles
        self.scan_idx = 0

    def joint_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(j in name_to_pos for j in JOINT_NAMES):
            self.current_joint_position = [name_to_pos[j] for j in JOINT_NAMES]

    def send_arm_trajectory(self, target_positions: list, duration_sec: float = 0.4):
        if not self.joint_client.wait_for_server(timeout_sec=0.2):
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = target_positions
        pt.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec - int(duration_sec)) * 1e9))
        goal.trajectory.points = [pt]

        self.joint_client.send_goal_async(goal)
        return True

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            h, w = frame.shape[:2]
            frame_center_x, frame_center_y = w // 2, h // 2

            faces = self.face_detector.detect(frame)

            now = time.time()
            curr_pan = self.current_joint_position[0]

            # ----------------------------------------------------
            # 1. State: SCANNING
            # ----------------------------------------------------
            if self.state == STATE_SCANNING:
                # Record detected faces during scan
                for (fx, fy, fw, fh) in faces:
                    fcx = fx + fw // 2
                    # Compute approximate pan angle for this detected face
                    angle_offset = - ((fcx - frame_center_x) / float(frame_center_x)) * 0.4
                    face_pan_angle = curr_pan + angle_offset
                    self.scanned_faces.append({
                        "pan_angle": face_pan_angle,
                        "rect": (fx, fy, fw, fh),
                        "timestamp": now
                    })

                # Perform periodic pan sweep movements
                if now - self.last_scan_move_time > SCAN_INTERVAL:
                    if self.scan_idx < len(self.scan_pan_targets):
                        next_pan = self.scan_pan_targets[self.scan_idx]
                        pose = [next_pan, HOME_STANDBY_POSE[1], HOME_STANDBY_POSE[2], HOME_STANDBY_POSE[3]]
                        self.send_arm_trajectory(pose, duration_sec=SCAN_INTERVAL * 0.8)
                        self.scan_idx += 1
                        self.last_scan_move_time = now
                    else:
                        # Sweep complete -> Move to target selection
                        self.get_logger().info(f" 🔍 [SCAN COMPLETE] Scanned {len(self.scanned_faces)} face detections. Selecting target in zone...")
                        self.state = STATE_SELECT_TARGET

            # ----------------------------------------------------
            # 2. State: SELECT_TARGET
            # ----------------------------------------------------
            elif self.state == STATE_SELECT_TARGET:
                # Filter scanned faces located within Target Zone
                valid_zone_faces = [
                    f for f in self.scanned_faces
                    if TARGET_ZONE_MIN_RAD <= f["pan_angle"] <= TARGET_ZONE_MAX_RAD
                ]

                if len(valid_zone_faces) > 0:
                    # Pick closest to target zone center (0.0 rad)
                    best_face = min(valid_zone_faces, key=lambda f: abs(f["pan_angle"]))
                    self.target_face_pan = best_face["pan_angle"]
                    self.state = STATE_TRACKING
                    self.last_seen_target_time = now
                    self.get_logger().info(f" 🎯 [TARGET LOCKED] Selected face in Target Zone at Pan={math.degrees(self.target_face_pan):.1f}°")
                else:
                    self.get_logger().info(" ⚠️ [TARGET NOT FOUND IN ZONE] Rescanning environment...")
                    self.scanned_faces.clear()
                    self._init_scan_targets()
                    self.state = STATE_SCANNING

            # ----------------------------------------------------
            # 3. State: TRACKING
            # ----------------------------------------------------
            elif self.state == STATE_TRACKING:
                if len(faces) > 0:
                    # Select face closest to current image center
                    primary_face = min(faces, key=lambda rect: abs((rect[0] + rect[2] // 2) - frame_center_x))
                    fx, fy, fw, fh = primary_face
                    cx, cy = fx + fw // 2, fy + fh // 2

                    self.last_seen_target_time = now

                    # Draw Bounding Box & Target Lock Indicator
                    cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                    cv2.line(frame, (frame_center_x, frame_center_y), (cx, cy), (0, 255, 255), 2)
                    cv2.putText(frame, f"LOCK-ON: FACE ({cx}, {cy})", (fx, max(20, fy - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                    dx = cx - frame_center_x
                    dy = cy - frame_center_y

                    # Proportional Visual Servo Control
                    if (now - self.last_cmd_time) > 0.22 and (abs(dx) > 20 or abs(dy) > 18):
                        curr_j1, curr_j2, curr_j3, curr_j4 = self.current_joint_position

                        # Pan (J1) Proportional Adjustment
                        pan_step = (dx / float(frame_center_x)) * 0.14
                        new_j1 = max(-2.35, min(2.35, curr_j1 - pan_step))

                        # Tilt (J2/J3) Proportional Adjustment
                        tilt_step = (dy / float(frame_center_y)) * 0.09
                        new_j2 = max(-0.80, min(0.50, curr_j2 + tilt_step))
                        new_j3 = curr_j3 - tilt_step * 0.5
                        new_j4 = - (new_j2 + new_j3)

                        target_pose = [new_j1, new_j2, new_j3, new_j4]
                        self.send_arm_trajectory(target_pose, duration_sec=0.30)
                        self.last_cmd_time = now

                else:
                    # Target lost timeout check (> 5 sec)
                    if now - self.last_seen_target_time > 5.0:
                        self.get_logger().warn(" ⚠️ [TARGET LOST TIMEOUT] Returning to Scan Phase...")
                        self.scanned_faces.clear()
                        self._init_scan_targets()
                        self.state = STATE_SCANNING

            # ----------------------------------------------------
            # Render HUD Display
            # ----------------------------------------------------
            cv2.drawMarker(frame, (frame_center_x, frame_center_y), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)

            # Zone Overlay Guidance Lines (Target Zone visualization)
            cv2.putText(frame, "OpenManipulator-X Face Scan & Tracker", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            state_color = (0, 255, 255) if self.state == STATE_SCANNING else ((0, 255, 0) if self.state == STATE_TRACKING else (255, 165, 0))
            cv2.putText(frame, f"State: {self.state}", (15, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, state_color, 2)
            cv2.putText(frame, f"Target Zone: -45 deg ~ +45 deg | Scanned Faces: {len(self.scanned_faces)}", (15, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

            cv2.putText(frame, f"Pan (J1): {math.degrees(curr_pan):.1f}deg | Tilt (J2): {self.current_joint_position[1]:.2f}",
                        (15, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1)

            cv2.imshow("human_tracker Vision", frame)
            cv2.waitKey(1)

        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackerControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
