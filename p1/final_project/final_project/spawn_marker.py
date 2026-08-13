import subprocess
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class SpawnMarkerNode(Node):
    def __init__(self):
        super().__init__('spawn_marker_node')
        self.subscription = self.create_subscription(
            Int32MultiArray, '/set_wall_markers', self.marker_callback, 10
        )
        self.env = os.environ.copy()
        
        # 벽의 절대 좌표와 상대 좌표를 합산한 마커별 절대 포즈 정의
        self.marker_poses = {
            'wall_marker_1': '0.949 0.2 0.25 0 -1.5708 0',
            'wall_marker_2': '0.949 -0.2 0.25 0 -1.5708 0'
        }
        
        self.get_logger().info("마커 스폰 노드 준비 완료.")

    def marker_callback(self, msg: Int32MultiArray) -> None:
        if len(msg.data) == 2:
            self.update_markers(msg.data[0], msg.data[1])

    def remove_model(self, model_name: str):
        subprocess.run([
            'gz', 'service', '-s', '/world/final/remove',
            '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000', '--req', f'name: "{model_name}" type: MODEL'
        ], env=self.env, check=False)

    def spawn_model(self, model_name: str, marker_id: int, pose_str: str):
        sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <pose>{pose_str}</pose>
        <geometry>
          <box><size>0.3 0.3 0.0001</size></box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>model://aruco_cube/materials/textures/aruco_{marker_id}.png</albedo_map>
              <roughness>1.0</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-string', sdf_content,
            '-name', model_name
        ]
        subprocess.Popen(cmd, env=self.env)
        self.get_logger().info(f"스폰 완료: {model_name} (aruco_{marker_id}.png)")

    def update_markers(self, cube_id: int, place_id: int) -> None:
        # 기존 동적 생성 모델 삭제
        self.remove_model("wall_marker_1")
        self.remove_model("wall_marker_2")
        
        # 새로운 마커를 정확한 절대 좌표에 생성
        self.spawn_model("wall_marker_1", cube_id, self.marker_poses['wall_marker_1'])
        self.spawn_model("wall_marker_2", place_id, self.marker_poses['wall_marker_2'])
        
        self.get_logger().info("벽면 마커 이미지 교체 완료.")

def main(args=None):
    rclpy.init(args=args)
    node = SpawnMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()