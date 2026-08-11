"""Detect stable ArUco IDs from the simulated link5 camera."""

from typing import Iterable, Tuple

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

from aruco_arm_sorter.detection_gate import StableMarkerGate


class ArucoDetector(Node):
    """Publish one confirmed marker ID whenever the arm enters READY."""

    def __init__(self) -> None:
        super().__init__("aruco_detector")

        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "현재 OpenCV에 aruco 모듈이 없습니다. python3-opencv를 설치하세요."
            )

        self.declare_parameter("image_topic", "/gripper_camera/image_raw")
        self.declare_parameter("status_topic", "/arm/status")
        self.declare_parameter("marker_topic", "/detected_marker_id")
        self.declare_parameter("annotated_topic", "/aruco/detection_image")
        self.declare_parameter("allowed_marker_ids", [0, 1])
        self.declare_parameter("required_consecutive_detections", 5)

        allowed_ids = [
            int(value)
            for value in self.get_parameter("allowed_marker_ids").value
        ]
        required_frames = int(
            self.get_parameter("required_consecutive_detections").value
        )
        self.gate = StableMarkerGate(allowed_ids, required_frames)
        self.bridge = CvBridge()

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.detector_parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.detector_parameters = cv2.aruco.DetectorParameters()
        self.detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, self.detector_parameters
            )

        image_topic = str(self.get_parameter("image_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        annotated_topic = str(self.get_parameter("annotated_topic").value)

        self.marker_publisher = self.create_publisher(Int32, marker_topic, 10)
        self.annotated_publisher = self.create_publisher(
            Image, annotated_topic, qos_profile_sensor_data
        )
        self.status_subscription = self.create_subscription(
            String, status_topic, self._status_callback, 10
        )
        self.image_subscription = self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"ArUco 검출 대기: {image_topic}; 허용 ID={sorted(allowed_ids)}; "
            f"연속 확인={required_frames}프레임"
        )

    def _status_callback(self, msg: String) -> None:
        was_ready = self.gate.ready
        is_ready = msg.data.strip() == "READY"
        self.gate.set_ready(is_ready)
        if is_ready and not was_ready:
            remaining = sorted(self.gate.allowed_ids - self.gate.published_ids)
            self.get_logger().info(f"마커 인식 활성화; 남은 ID={remaining}")

    def _detect(self, gray: np.ndarray) -> Tuple[Iterable, object]:
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.detector_parameters,
            )
        return corners, ids

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"카메라 영상 변환 실패: {exc}")
            return

        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect(gray)
        detected_ids = [] if ids is None else [int(value) for value in ids.flatten()]

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        confirmed_id = self.gate.observe(detected_ids)
        if confirmed_id is not None:
            self.marker_publisher.publish(Int32(data=confirmed_id))
            self.get_logger().info(
                f"ArUco ID {confirmed_id} 확정 -> /detected_marker_id 발행"
            )

        gate_state = "READY" if self.gate.ready and self.gate.armed else "WAIT"
        stable_text = (
            f"candidate={self.gate.candidate} "
            f"stable={self.gate.consecutive_frames}/{self.gate.required_frames}"
        )
        cv2.putText(
            frame,
            f"{gate_state} {stable_text}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 220, 0) if gate_state == "READY" else (0, 180, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"detected={detected_ids} published={sorted(self.gate.published_ids)}",
            (12, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 220, 0),
            1,
            cv2.LINE_AA,
        )

        annotated = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated.header = msg.header
        self.annotated_publisher.publish(annotated)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
