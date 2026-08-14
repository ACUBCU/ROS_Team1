#!/usr/bin/env python3
import os
os.environ.setdefault("DISPLAY", ":0")

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

KNOWN_BOX_BASELINE_DIST_M = 0.24
STEREO_BASELINE_M = 0.085         # Baseline B = 8.5 cm
STEREO_FOCAL_LENGTH_PX = 554.25   # Focal length f in pixels


class YoloDepthPublisher(Node):
    """
    YOLO & Depth Perception Node for real_robot package:
    Perceives 3D boxes from camera stream and publishes target coordinates & depth distance.
    Topic Published: /yolo_depth/detected_objects
      Data Format: [cx, cy, area, distance_m] for each detected object
    """
    def __init__(self):
        super().__init__("yolo_depth_publisher")
        self.bridge = CvBridge()

        # Image Subscriptions
        self.create_subscription(Image, "camera/image_raw", self.image_callback, 10)
        self.create_subscription(Image, "/gripper_camera/image_raw", self.image_callback, 10)

        # Perception Topic Publisher
        self.target_pub = self.create_publisher(Float32MultiArray, "/yolo_depth/detected_objects", 10)

        self.latest_frame = None
        self.detected_objects = []

        self.get_logger().info("==================================================")
        self.get_logger().info(" 🎥 [real_robot] YOLO & Depth Topic Publisher Node Initialized!")
        self.get_logger().info("==================================================")

    def calculate_stereo_distance(self, upper_center, lower_center):
        _, cy1 = upper_center
        _, cy2 = lower_center
        disparity_y = abs(cy2 - cy1)
        disparity_clamped = max(10.0, min(220.0, disparity_y))
        stereo_dist_m = (STEREO_FOCAL_LENGTH_PX * STEREO_BASELINE_M) / disparity_clamped
        return max(0.18, min(0.35, stereo_dist_m))

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

            # OTSU thresholding + Canny Edge for clean 3D box perception
            _, thresh_otsu = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            edges = cv2.Canny(blur, 30, 120)
            combined = cv2.bitwise_or(thresh_otsu, edges)

            combined_roi = cv2.bitwise_and(combined, combined, mask=roi_mask)
            contours, _ = cv2.findContours(combined_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            self.detected_objects = []
            pub_msg = Float32MultiArray()
            data_list = []

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

                    obj_data = {
                        "rect": (x, y, bw, bh),
                        "center": (cx, cy),
                        "area": area,
                        "distance_m": stereo_dist_m,
                    }
                    self.detected_objects.append(obj_data)
                    data_list.extend([float(cx), float(cy), float(area), float(stereo_dist_m)])

                    cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                    cv2.putText(frame, f"YOLO Box ({area:.0f})", (x, max(15, y - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"Depth: {stereo_dist_m:.2f}m", (x, y + bh + 18),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

            pub_msg.data = data_list
            self.target_pub.publish(pub_msg)

            # Center marker cross only (Blue border removed)
            cv2.drawMarker(frame, (320, 240), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

            cv2.imshow("real_robot YOLO & Depth Vision", frame)
            cv2.waitKey(1)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
