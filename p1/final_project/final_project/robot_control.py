import threading
import time
from typing import Any, Dict, List

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

def _duration(seconds: float) -> Duration:
    whole = int(seconds)
    nanos = int(round((seconds - whole) * 1_000_000_000))
    if nanos >= 1_000_000_000:
        whole += 1
        nanos -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanos)


class RobotControlNode(Node):
    def __init__(self):
        super().__init__("robot_control_node")

        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

        self.subscription = self.create_subscription(
            Int32MultiArray, '/detected_aruco_ids', self._marker_callback, 10
        )
        
        self.start_subscription = self.create_subscription(
            Int32MultiArray, '/start_command', self._start_callback, 10
        )

        self.gripper_open = 0.019
        self.gripper_close = -0.004999999999999999
        self.pre_grasp_pose = [0.0, 0.20011967400443872, -0.15859986254409464, 0.019461108826370054]
        
        self.pick_poses = {
            0: {
                'approach': [-1.2999998449730557, 0.3001196746476289, -0.11859986140199331, -0.0005388888602444933],
                'grasp': [-1.2999998449730557, 0.6001196746476292, -0.11859986140199331, -0.0005388888602444933]
            },
            1: {
                'approach': [-0.8199999038136074, 0.008331547800554902, 0.1614156814758045, -0.0005388929096217653],
                'grasp': [-0.8199999038136074, 0.428331547800555, 0.1614156814758045, -0.0005388929096217653]
            },
            2: {
                'approach': [-0.3399998416322402, 0.6492403426121606, -0.13858397669228173, -0.0005388890538846811],
                'grasp': [-0.3399998416322402, 0.6492403426121606, -0.13858397669228173, -0.0005388890538846811]
            },
            3: {
                'approach': [0.3200029267947044, 0.2724274651759895, -0.13858376126194577, -0.0005388990932245909],
                'grasp': [0.3200001550269398, 0.6001196741180913, -0.07859986228656035, 0.019461109332147966]
            },
            4: {
                'approach': [0.8200001550269402, -0.21988032588190892, 0.2614001377134396, 0.019461109332147966],
                'grasp': [0.8200001550269402, 0.3401196741180911, 0.2614001377134396, 0.019461109332147966]
            },
            5: {
                'approach': [1.2800001519437905, 0.26012042777294064, -0.11859918198145757, -0.0005388861472491246],
                'grasp': [1.2800001519437911, 0.6001204278960247, -0.11859918173765524, -0.0005388856374552998]
            }
        }
        
        self.place_poses = {
            6: {
                'approach': [2.2399999999999993, -0.4198803259443542, 0.4014001375887688, 0.019461109076998058],
                'place': [2.2399999999999993, 0.22011967405564586, 0.4014001375887688, 0.019461109076998058]
            },
            7: {
                'approach': [2.5999999999999996, -0.5798803259443543, 0.8214001375887692, 0.019461109076998058],
                'place': [2.5999999999999996, -0.11988032594435412, 0.8214001375887692, 0.019461109076998058]
            },
            8: {
                'approach': [3.14, -0.8398803259443546, 1.0614001375887694, -0.08053889092300195],
                'place': [3.14, -0.3198803259443541, 1.0614001375887694, -0.08053889092300195]
            },
            9: {
                'approach': [-2.600000000000004, -0.7398803259443545, 0.8214001375887692, 0.019461109076998058],
                'place': [-2.600000000000004, -0.13988032594435412, 0.8214001375887692, 0.019461109076998058]
            },
            10: {
                'approach': [-2.2400000000000038, -0.21988032594435408, 0.34140013758876875, 0.019461109076998058],
                'place': [-2.2400000000000038, 0.3001196740556459, 0.34140013758876875, 0.019461109076998058]
            }
        }

        self.task_received = False
        self.target_cube = -1
        self.target_place = -1
        
        self.stop_event = threading.Event()
        self.task_ready = threading.Event()
        self.start_command_ready = threading.Event()
        
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        self.get_logger().info("로봇 제어 노드 초기화 완료. 액션 서버 연결 대기 중...")

    def _marker_callback(self, msg: Int32MultiArray) -> None:
        if self.task_received:
            return

        ids = list(msg.data)
        if len(ids) < 2:
            return

        cube_id = -1
        place_id = -1

        for m_id in ids:
            if 0 <= m_id <= 5 and cube_id == -1:
                cube_id = m_id
            elif 6 <= m_id <= 10 and place_id == -1:
                place_id = m_id

        if cube_id != -1 and place_id != -1:
            self.target_cube = cube_id
            self.target_place = place_id
            self.task_received = True
            
            self.get_logger().info(f"수신된 마커 ID 배열: {ids}")
            self.get_logger().info(f"작업 확정 - 큐브: {self.target_cube}, 장소: {self.target_place}")
            
            # self.destroy_subscription(self.subscription)
            self.task_ready.set()

    def _start_callback(self, msg: Int32MultiArray) -> None:
        self.get_logger().info("대시보드로부터 [이동 시작] 명령을 수신했습니다.")
        self.start_command_ready.set()

    def _worker_loop(self) -> None:
        try:
            self._wait_for_action_servers()
            time.sleep(2.0)
            self.get_logger().info("Action Server 연결 완료.")

            while not self.stop_event.is_set():
                # 매 작업 시작 전 상태 초기화
                self.task_received = False
                self.target_cube = -1
                self.target_place = -1
                self.task_ready.clear()
                self.start_command_ready.clear()

                self.get_logger().info("새로운 작업을 위한 카메라 마커 인지 대기 중...")

                while not self.stop_event.is_set():
                    if self.task_ready.wait(timeout=0.2):
                        break
                
                if self.stop_event.is_set():
                    return

                self.get_logger().info("마커 인지 완료. 대시보드의 [이동 시작] 버튼 입력을 대기합니다.")

                while not self.stop_event.is_set():
                    if self.start_command_ready.wait(timeout=0.2):
                        break

                if self.stop_event.is_set():
                    return

                self.get_logger().info("작업을 시작합니다.")
                self._execute_pick_and_place()

        except Exception as exc:
            self.get_logger().error(f"스레드 실행 중 치명적 오류 발생: {exc}")

    def _execute_pick_and_place(self) -> None:
        target_pick_data = self.pick_poses.get(self.target_cube)
        target_place_data = self.place_poses.get(self.target_place)

        if not target_pick_data or not target_place_data:
            self.get_logger().error(f"지정된 큐브({self.target_cube}) 또는 장소({self.target_place})의 자세 데이터가 없습니다.")
            return

        approach_angles = target_pick_data['approach']
        grasp_angles = target_pick_data['grasp']
        place_approach_angles = target_place_data['approach']
        place_angles = target_place_data['place']

        # 큐브에 접근하기 전 그리퍼 열기 동작 추가
        self._safe_send_gripper_goal(self.gripper_open, label="initial_gripper_open")
        time.sleep(1.0)

        self._send_arm_goal(approach_angles, duration=3.0, label="approach_pose")
        self._send_arm_goal(grasp_angles, duration=2.0, label="grasp_pose")
        self._safe_send_gripper_goal(self.gripper_close, label="gripper_close")
        time.sleep(1.0)
        self._send_arm_goal(approach_angles, duration=2.0, label="lift_pose")
        self._send_arm_goal(place_approach_angles, duration=3.0, label="place_approach")
        self._send_arm_goal(place_angles, duration=2.0, label="place_pose")
        self._safe_send_gripper_goal(self.gripper_open, label="gripper_open")
        time.sleep(1.0)
        self._send_arm_goal(place_approach_angles, duration=2.0, label="place_lift_pose")
        self._send_arm_goal(self.pre_grasp_pose, duration=2.0, label="return_home")
        self.get_logger().info("작업 완료")

    def _safe_send_gripper_goal(self, position: float, label: str) -> None:
        try:
            self._send_gripper_goal(position, label)
        except Exception as e:
            self.get_logger().warn(f"그리퍼 제어 중 타임아웃 발생 (동작 유지): {e}")

    def _wait_for_action_servers(self) -> None:
        for client, name in [(self.arm_client, "arm"), (self.gripper_client, "gripper")]:
            self.get_logger().info(f"Action Server 연결 대기 중: {name}...")
            while not self.stop_event.is_set():
                if client.wait_for_server(timeout_sec=1.0):
                    self.get_logger().info(f"Action Server 연결 완료: {name}")
                    break

    def _send_arm_goal(self, positions: List[float], duration: float, label: str) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = _duration(duration)
        
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = _duration(3.0)
        
        result = self._send_goal_and_wait(self.arm_client, goal, f"팔/{label}", duration + 25.0)
        if getattr(result, "error_code", 0) != 0:
            raise RuntimeError(f"팔 이동 실패: {getattr(result, 'error_string', '')}")

    def _send_gripper_goal(self, position: float, label: str) -> None:
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 2.0
        self._send_goal_and_wait(self.gripper_client, goal, f"그리퍼/{label}", 5.0)

    def _send_goal_and_wait(self, client: ActionClient, goal: Any, label: str, timeout: float) -> Any:
        sent = threading.Event()
        result_ready = threading.Event()
        state: Dict[str, Any] = {}

        def sent_callback(future):
            try:
                state["goal_handle"] = future.result()
            except Exception as exc:
                state["error"] = exc
            sent.set()

        client.send_goal_async(goal).add_done_callback(sent_callback)
        self._wait_event(sent, timeout, f"{label} 전송")
        
        if "error" in state:
            raise state["error"]
        goal_handle = state["goal_handle"]
        if not goal_handle.accepted:
            raise RuntimeError(f"{label} 거부됨")

        def result_callback(future):
            try:
                state["wrapped_result"] = future.result()
            except Exception as exc:
                state["error"] = exc
            result_ready.set()

        goal_handle.get_result_async().add_done_callback(result_callback)
        self._wait_event(result_ready, timeout, f"{label} 대기")
        
        if "error" in state:
            raise state["error"]
        wrapped = state["wrapped_result"]
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"{label} 상태 오류: {wrapped.status}")
        return wrapped.result

    def _wait_event(self, event: threading.Event, timeout: float, label: str) -> None:
        # deadline = time.monotonic() + timeout
        while not event.wait(0.1):
            if self.stop_event.is_set():
                raise RuntimeError("노드 종료됨")
            # if time.monotonic() >= deadline:
            #     raise TimeoutError(f"{label} 시간 초과")

    def destroy_node(self):
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()