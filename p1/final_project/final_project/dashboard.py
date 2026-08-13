import os
import rclpy
import subprocess
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32MultiArray
from cv_bridge import CvBridge
import cv2
import threading
import time
from flask import Flask, render_template, Response, jsonify, request
from ament_index_python.packages import get_package_share_directory

package_name = 'final_project'
package_share_directory = get_package_share_directory(package_name)
template_dir = os.path.join(package_share_directory, 'templates')

app = Flask(__name__, template_folder=template_dir)
bridge = CvBridge()

frames = {
    'gripper': None,
    'top': None,
    'cube': None,
    'place': None
}
ros_node = None

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        
        self.sub_gripper = self.create_subscription(Image, '/gripper_camera/image_raw', self.cb_gripper, 10)
        self.sub_top = self.create_subscription(Image, '/camera_top/image_raw', self.cb_top, 10)
        # self.sub_cube = self.create_subscription(Image, '/camera_cube/image_raw', self.cb_cube, 10)
        # self.sub_place = self.create_subscription(Image, '/camera_place/image_raw', self.cb_place, 10)
        
        self.pub_markers = self.create_publisher(Int32MultiArray, '/set_wall_markers', 10)
        self.pub_start = self.create_publisher(Int32MultiArray, '/start_command', 10)
        self.pub_reset = self.create_publisher(Bool, '/reset_command', 10)

    def cb_gripper(self, msg): frames['gripper'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def cb_top(self, msg): frames['top'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def cb_cube(self, msg): frames['cube'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    def cb_place(self, msg): frames['place'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def send_wall_markers(self, cube_id, place_id):
        msg = Int32MultiArray()
        msg.data = [cube_id, place_id]
        self.pub_markers.publish(msg)
        self.get_logger().info(f"대시보드: 벽면 마커 변경 요청 (큐브: {cube_id}, 장소: {place_id})")

    def send_start_command(self, cube_id, place_id):
        msg = Int32MultiArray()
        msg.data = [cube_id, place_id]
        self.pub_start.publish(msg)
        self.get_logger().info(f"대시보드: 이동 시작 명령 전송 (큐브: {cube_id}, 장소: {place_id})")

    def send_reset_command(self):
        msg = Bool()
        msg.data = True
        self.pub_reset.publish(msg)
        self.get_logger().info("대시보드: 초기화 명령 전송")

def generate_frames(cam_name):
    while True:
        frame = frames.get(cam_name)
        if frame is not None:
            frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
            ret, buffer = cv2.imencode('.jpg', frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            else:
                time.sleep(0.05)
        else:
            time.sleep(0.05)

@app.route('/')
def index(): return render_template('index.html')

@app.route('/video/gripper')
def video_gripper(): return Response(generate_frames('gripper'), mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/video/top')
def video_top(): return Response(generate_frames('top'), mimetype='multipart/x-mixed-replace; boundary=frame')
# @app.route('/video/cube')
# def video_cube(): return Response(generate_frames('cube'), mimetype='multipart/x-mixed-replace; boundary=frame')
# @app.route('/video/place')
# def video_place(): return Response(generate_frames('place'), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/set_markers', methods=['POST'])
def set_markers_api():
    data = request.get_json()
    cube_id = int(data.get('cube', 0))
    place_id = int(data.get('place', 6))
    if ros_node:
        ros_node.send_wall_markers(cube_id, place_id)
    return jsonify({"status": "success"})

@app.route('/api/start', methods=['POST'])
def start_api():
    env = os.environ.copy()
    try:
        subprocess.Popen(['ros2', 'run', 'final_project', 'aruco_detect'], env=env)
    except Exception as e:
        print(f"aruco_detect 실행 실패: {e}")

    data = request.get_json()
    cube_id = int(data.get('cube', 0))
    place_id = int(data.get('place', 6))
    if ros_node:
        ros_node.send_start_command(cube_id, place_id)
    return jsonify({"status": "success"})

@app.route('/api/reset', methods=['POST'])
def reset_api():
    if ros_node:
        ros_node.send_reset_command()
    return jsonify({"status": "success"})

def run_ros_thread():
    rclpy.init()
    global ros_node
    ros_node = DashboardNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    ros_thread = threading.Thread(target=run_ros_thread, daemon=True)
    ros_thread.start()
    print("====================================================")
    print("멀티 카메라 대시보드 서버가 시작되었습니다.")
    print("주소: http://localhost:5050")
    print("====================================================")
    app.run(host='0.0.0.0', port=5050, debug=False, use_reloader=False)

if __name__ == '__main__':
    main()