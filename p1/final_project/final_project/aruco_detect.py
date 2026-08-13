import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Int32MultiArray

class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__('aruco_detect_node')
        self.bridge = CvBridge()
        self.target_logged = False
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.image_sub = self.create_subscription(
            Image,
            '/gripper_camera/image_raw',
            self.image_callback,
            10
        )
        self.marker_pub = self.create_publisher(Int32MultiArray, '/detected_aruco_ids', 10)
        # self.get_logger().info('대비(Contrast) 조절 테스트 노드 시작')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        alpha = 2.0
        beta = -50
        enhanced_gray = cv2.convertScaleAbs(gray, alpha=alpha, beta=beta)

        corners, ids, rejected = cv2.aruco.detectMarkers(enhanced_gray, self.dictionary, parameters=self.parameters)

        if ids is not None and len(ids) == 2:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            detected_ids = [int(ids[0][0]), int(ids[1][0])]
            
            cube_id = -1
            target_id = -1
            
            for m_id in detected_ids:
                if 0 <= m_id <= 5:
                    cube_id = m_id
                elif 6 <= m_id <= 10:
                    target_id = m_id
            
            if cube_id != -1 and target_id != -1:
                # 1. 1회 로그 출력
                if not self.target_logged:
                    self.get_logger().info(f"대상 큐브: ID {cube_id} -> 목표 장소: ID {target_id}")
                    self.target_logged = True
                
                # 2. 토픽 발행 (핵심 추가 부분)
                pub_msg = Int32MultiArray()
                pub_msg.data = [cube_id, target_id]
                self.marker_pub.publish(pub_msg)

        # cv2.imshow("Enhanced Gray", enhanced_gray)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()