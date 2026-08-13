import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class WorldRebootNode(Node):
    def __init__(self):
        super().__init__('world_reboot_node')
        self.subscription = self.create_subscription(
            Bool, '/reset_command', self.reset_callback, 10
        )
        self.get_logger().info("월드 리부트 노드 실행 완료.")

    def reset_callback(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info("월드 초기화 요청 수신")
            self.reboot_world()

    def reboot_world(self) -> None:
        for i in range(6):
            subprocess.run(['gz', 'service', '-s', '/world/final/remove', '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean', '--timeout', '2000', '--req', f'name: "aruco_cube_{i}" type: MODEL'], check=False)
        
        subprocess.Popen(['ros2', 'run', 'final_project', 'spawn_cube'])
        self.get_logger().info("월드 초기화 및 큐브 재생성 완료")

def main(args=None):
    rclpy.init(args=args)
    node = WorldRebootNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()