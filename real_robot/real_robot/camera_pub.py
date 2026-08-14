#!/usr/bin/env python3
import os
os.environ.setdefault("DISPLAY", ":0")

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraPub(Node):
    def __init__(self):
        super().__init__("camera_pub")
        self.pub = self.create_publisher(Image, "camera/image_raw", 10)
        self.pub_info = self.create_publisher(CameraInfo, "camera/camera_info", 10)
        self.bridge = CvBridge()
        
        self.width = 640
        self.height = 480
        
        # 1. Raw GStreamer pipeline (MacBook camera / YUYV without MJPEG)
        raw_pipeline = (
            "v4l2src device=/dev/video0 ! "
            f"video/x-raw,width={self.width},height={self.height} ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true sync=false"
        )
        # 2. MJPEG GStreamer pipeline (for cameras using MJPEG)
        mjpeg_pipeline = (
            "v4l2src device=/dev/video0 ! "
            f"image/jpeg,width={self.width},height={self.height},framerate=30/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true sync=false"
        )
        
        test_pipeline = (
            f"videotestsrc ! video/x-raw,width={self.width},height={self.height},framerate=30/1 ! "
            "videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false"
        )
        if os.path.exists("/dev/video0"):
            self.get_logger().info("Opening /dev/video0 via MJPEG GStreamer pipeline...")
            self.cap = cv2.VideoCapture(mjpeg_pipeline, cv2.CAP_GSTREAMER)
            if not self.cap.isOpened() or not self.cap.read()[0]:
                self.get_logger().info("MJPEG GStreamer failed, trying V4L2 backend with MJPG FOURCC...")
                self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
                if self.cap.isOpened():
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if not self.cap.isOpened() or not self.cap.read()[0]:
                self.get_logger().info("V4L2 failed, trying Raw GStreamer pipeline...")
                self.cap = cv2.VideoCapture(raw_pipeline, cv2.CAP_GSTREAMER)
        else:
            self.cap = cv2.VideoCapture(test_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().warn("Physical camera /dev/video0 not accessible. Falling back to test video stream.")
            self.cap = cv2.VideoCapture(test_pipeline, cv2.CAP_GSTREAMER)
        
        self.camera_info = self.create_camera_info()
        
        self.create_timer(1 / 30, self.img_gen_callback)
        self.get_logger().info("Camera Publisher Node Started.")

    def create_camera_info(self):
        msg = CameraInfo()
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx, fy = 600.0, 600.0
        cx, cy = self.width / 2.0, self.height / 2.0
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def img_gen_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None or frame.size == 0:
            return

        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif len(frame.shape) == 3:
            if frame.shape[2] == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)
            elif frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        now = self.get_clock().now().to_msg()

        img_msg = Image()
        img_msg.header.stamp = now
        img_msg.header.frame_id = "camera_link"
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = False
        img_msg.step = frame.shape[1] * 3
        img_msg.data = frame.tobytes()

        self.camera_info.header.stamp = now
        self.camera_info.header.frame_id = "camera_link"

        self.pub.publish(img_msg)
        self.pub_info.publish(self.camera_info)

        try:
            cv2.imshow("Camera Raw", frame)
            cv2.waitKey(1)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CameraPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Camera Pub 노드 종료")
    finally:
        cv2.destroyAllWindows()
        node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()